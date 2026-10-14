import fs from 'node:fs/promises';
import path from 'node:path';
import { Workbook } from '@oai/artifact-tool';

// Produces an illustrative data set with the same columns as run_csv_logger.py.
// Values show a damped swing, two disturbances, and a moving cart. They are
// synthetic and must not be used to assess controller or hardware performance.
const columns = [
  'record_type', 'elapsed_s', 'cycle', 'esp32_packet', 'raw_angle_1',
  'raw_angle_2', 'raw_angle_3', 'upright_correction_counts',
  'measured_angle_rad', 'measured_angle_deg', 'cart_used_m',
  'cart_steps', 'cart_m', 'pred_angle_rad', 'pred_angle_rate_rad_s',
  'pred_cart_m', 'pred_cart_velocity_m_s', 'est_angle_rad',
  'est_angle_rate_rad_s', 'est_cart_m', 'est_cart_velocity_m_s',
  'previous_force_n', 'force_command_n', 'ramp_fraction',
  'step_frequency_hz', 'motor_timer_command', 'sensor_read_ms',
  'position_read_ms', 'loop_ms', 'cycle_period_ms', 'message',
];
const dt = 0.01;
const correction = 10420;
const countsPerRadian = 16384 / (2 * Math.PI);
const round = (value, places = 6) => Number(value.toFixed(places));
const row = values => columns.map(name => values[name] ?? '');
const csvField = value => {
  const string = String(value);
  return /[",\r\n]/.test(string) ? `"${string.replaceAll('"', '""')}"` : string;
};
const rows = [columns];
rows.push(row({
  record_type: 'config', elapsed_s: 0,
  message: JSON.stringify({schema_version: 1, simulated: true, sample_period_s: dt,
    upright_correction_counts: correction, pendulum_length_m: 0.2,
    description: 'Illustrative example for the CSV visualizer; not a measured run.'}),
}));
rows.push(row({record_type: 'start', elapsed_s: 0,
  message: 'SIMULATED EXAMPLE — no physical pendulum or motor was run'}));
rows.push(row({record_type: 'check', elapsed_s: 0.2,
  message: 'upright_angle_deg=+7.500; raw_first=10761; raw_last=10761'}));
rows.push(row({record_type: 'motor_start', elapsed_s: 0.25,
  message: 'initial_command=-65535; initial_cart_steps=0; initial_cart_m=0'}));

let previousTheta = 0;
let previousCart = 0;
let previousEstTheta = 0;
let previousEstThetaRate = 0;
let previousEstCart = 0;
let previousEstCartSpeed = 0;
let previousForce = 0;
for (let i = 0; i <= 2000; i++) {
  const t = i * dt;
  const elapsed = 0.26 + t;
  const disturbance1 = Math.max(0, t - 8.5);
  const disturbance2 = Math.max(0, t - 14.2);
  const theta = (7.5 * Math.PI / 180) * Math.exp(-0.35 * t) * Math.cos(2.8 * t)
    + (t >= 8.5 ? (4.2 * Math.PI / 180) * Math.exp(-0.8 * disturbance1) * Math.sin(4.0 * disturbance1) : 0)
    + (t >= 14.2 ? (-3.1 * Math.PI / 180) * Math.exp(-0.9 * disturbance2) * Math.sin(4.5 * disturbance2) : 0);
  const cart = 0.075 * Math.exp(-0.11 * t) * Math.sin(1.35 * t)
    + (t >= 8.5 ? 0.028 * Math.exp(-0.55 * disturbance1) * Math.sin(2.5 * disturbance1) : 0)
    + (t >= 14.2 ? -0.019 * Math.exp(-0.65 * disturbance2) * Math.sin(2.9 * disturbance2) : 0);
  const thetaRate = i === 0 ? 0 : (theta - previousTheta) / dt;
  const cartSpeed = i === 0 ? 0 : (cart - previousCart) / dt;
  const predTheta = previousEstTheta + previousEstThetaRate * dt;
  const predThetaRate = previousEstThetaRate;
  const predCart = previousEstCart + previousEstCartSpeed * dt;
  const predCartSpeed = previousEstCartSpeed;
  const estTheta = 0.96 * theta + 0.04 * predTheta;
  const estThetaRate = 0.93 * thetaRate + 0.07 * predThetaRate;
  const estCart = 0.96 * cart + 0.04 * predCart;
  const estCartSpeed = 0.93 * cartSpeed + 0.07 * predCartSpeed;
  const ramp = Math.min((i + 1) / 200, 1);
  const force = (-3.2 * estTheta - 0.65 * estThetaRate - 1.8 * estCart - 0.8 * estCartSpeed) * ramp;
  const frequency = -(estCartSpeed * 6400) / 0.638175;
  const motor = Math.abs(frequency) > 2 ? Math.trunc((0.5 / frequency) * 250000) : (frequency >= 0 ? 65535 : -65535);
  const rawAngle = ((Math.round(correction + theta * countsPerRadian) % 16384) + 16384) % 16384;
  rows.push(row({
    record_type: 'sample', elapsed_s: round(elapsed, 2), cycle: i + 1,
    esp32_packet: i + 61, raw_angle_1: rawAngle,
    raw_angle_2: 4200 + Math.round(18 * Math.sin(0.8 * t)),
    raw_angle_3: 9800 + Math.round(23 * Math.cos(0.6 * t)),
    upright_correction_counts: correction,
    measured_angle_rad: round(theta, 8), measured_angle_deg: round(theta * 180 / Math.PI, 5),
    cart_used_m: round(previousCart, 7), cart_steps: Math.round(cart * 6400 / 0.638175),
    cart_m: round(cart, 7),
    pred_angle_rad: round(predTheta, 8), pred_angle_rate_rad_s: round(predThetaRate, 7),
    pred_cart_m: round(predCart, 7), pred_cart_velocity_m_s: round(predCartSpeed, 7),
    est_angle_rad: round(estTheta, 8), est_angle_rate_rad_s: round(estThetaRate, 7),
    est_cart_m: round(estCart, 7), est_cart_velocity_m_s: round(estCartSpeed, 7),
    previous_force_n: round(previousForce, 7), force_command_n: round(force, 7),
    ramp_fraction: round(ramp, 3), step_frequency_hz: round(frequency, 4),
    motor_timer_command: motor, sensor_read_ms: round(0.28 + 0.06 * Math.sin(t * 4) ** 2, 4),
    position_read_ms: round(0.37 + 0.07 * Math.cos(t * 3) ** 2, 4),
    loop_ms: round(2.05 + 0.23 * Math.sin(t * 5) ** 2, 4),
    cycle_period_ms: i === 0 ? '' : 10,
  }));
  previousTheta = theta;
  previousCart = cart;
  previousEstTheta = estTheta;
  previousEstThetaRate = estThetaRate;
  previousEstCart = estCart;
  previousEstCartSpeed = estCartSpeed;
  previousForce = force;
}
rows.push(row({record_type: 'stop', elapsed_s: 20.27,
  message: 'Ctrl+C; dropped_samples=0; SIMULATED EXAMPLE'}));

if (rows.length !== 2007 || rows.some(values => values.length !== columns.length)) {
  throw new Error('Generated sample count or CSV width is incorrect');
}
const csvText = rows.map(values => values.map(csvField).join(',')).join('\n') + '\n';
const output = path.resolve('outputs/example-20-second-run/example_SIMULATED_20_second_run.csv');
await fs.mkdir(path.dirname(output), {recursive: true});
await fs.writeFile(output, csvText, 'utf8');

// The artifact tool imports the finished CSV so the expected layout can be
// inspected through the same spreadsheet runtime as other workbook artifacts.
const workbook = await Workbook.fromCSV(csvText, {sheetName: 'Run data'});
workbook.recalculate();
const check = await workbook.inspect({kind: 'table', range: 'A1:D6',
  include: 'values', tableMaxRows: 6, tableMaxCols: 4, maxChars: 1800});
console.log(check.ndjson);
console.log(`Saved ${output} (${rows.length - 6} samples)`);
