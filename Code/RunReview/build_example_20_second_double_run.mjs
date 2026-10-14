import fs from 'node:fs/promises';
import path from 'node:path';
import { Workbook } from '@oai/artifact-tool';

// Synthetic demonstration for the double-pendulum CSV format.
const columns = [
  'record_type', 'elapsed_s', 'cycle', 'esp32_packet',
  'raw_angle_1', 'raw_angle_2', 'raw_angle_3',
  'reference_upright_1_counts', 'reference_aligned_2_counts',
  'measured_angle_1_rad', 'measured_angle_1_deg',
  'measured_angle_2_rad', 'measured_angle_2_deg',
  'joint_angle_2_rad', 'joint_angle_2_deg', 'cart_used_m',
  'cart_steps', 'cart_m',
  'pred_theta1_rad', 'pred_theta1_rate_rad_s',
  'pred_theta2_rad', 'pred_theta2_rate_rad_s',
  'pred_cart_m', 'pred_cart_velocity_m_s',
  'est_theta1_rad', 'est_theta1_rate_rad_s',
  'est_theta2_rad', 'est_theta2_rate_rad_s',
  'est_cart_m', 'est_cart_velocity_m_s',
  'previous_force_n', 'force_command_n',
  'cart_velocity_command_m_s', 'ramp_fraction',
  'step_frequency_hz', 'motor_timer_command',
  'reference_upright_1_next_counts',
  'sensor_read_ms', 'position_read_ms', 'loop_ms',
  'cycle_period_ms', 'message',
];
const row = values => columns.map(name => values[name] ?? '');
const round = (value, places = 7) => Number(value.toFixed(places));
const csvField = value => {
  const string = String(value);
  return /[",\r\n]/.test(string) ? `"${string.replaceAll('"', '""')}"` : string;
};
const rows = [columns];
const dt = 0.01;
const radians = Math.PI / 180;
const countsPerRadian = 16384 / (2 * Math.PI);
let reference1 = 12200;
const reference2 = 4500;
rows.push(row({record_type: 'config', elapsed_s: 0, message: JSON.stringify({
  schema_version: 1, pendulum_count: 2, simulated: true, sample_period_s: dt,
  reference_upright_1_counts: reference1, reference_aligned_2_counts: reference2,
  description: 'Illustrative example; no physical controller or motor was run.'
})}));
rows.push(row({record_type: 'start', elapsed_s: 0,
  message: 'SIMULATED EXAMPLE — no physical pendulum or motor was run'}));
rows.push(row({record_type: 'check', elapsed_s: 0.2,
  message: 'arm1_deg=+6.000; arm2_deg=+9.301; joint2_deg=+3.301'}));
rows.push(row({record_type: 'motor_start', elapsed_s: 0.25,
  message: 'initial_command=-65535; initial_cart_steps=0; initial_cart_m=0'}));

let prevArm1 = 0, prevArm2 = 0, prevCart = 0, prevForce = 0;
let prevEst = [0, 0, 0, 0, 0, 0];
for (let i = 0; i <= 2000; i++) {
  const t = i * dt;
  const d1 = Math.max(0, t - 8.4), d2 = Math.max(0, t - 14.1);
  const arm1 = radians * (6 * Math.exp(-0.23 * t) * Math.cos(2.55 * t)
    + (t >= 8.4 ? 3.5 * Math.exp(-0.8 * d1) * Math.sin(4.0 * d1) : 0)
    + (t >= 14.1 ? -2.6 * Math.exp(-0.8 * d2) * Math.sin(3.9 * d2) : 0));
  const joint2 = radians * (4 * Math.exp(-0.30 * t) * Math.cos(3.5 * t + 0.6)
    + (t >= 8.4 ? -3.2 * Math.exp(-0.75 * d1) * Math.sin(4.5 * d1) : 0)
    + (t >= 14.1 ? 2.8 * Math.exp(-0.85 * d2) * Math.sin(4.1 * d2) : 0));
  const arm2 = arm1 + joint2;
  const cart = 0.065 * Math.exp(-0.12 * t) * Math.sin(1.3 * t)
    + (t >= 8.4 ? 0.026 * Math.exp(-0.6 * d1) * Math.sin(2.2 * d1) : 0)
    + (t >= 14.1 ? -0.017 * Math.exp(-0.6 * d2) * Math.sin(2.6 * d2) : 0);
  const arm1Rate = i ? (arm1 - prevArm1) / dt : 0;
  const arm2Rate = i ? (arm2 - prevArm2) / dt : 0;
  const cartSpeed = i ? (cart - prevCart) / dt : 0;
  const predicted = [
    prevEst[0] + prevEst[1] * dt, prevEst[1],
    prevEst[2] + prevEst[3] * dt, prevEst[3],
    prevEst[4] + prevEst[5] * dt, prevEst[5]
  ];
  const measured = [arm1, arm1Rate, arm2, arm2Rate, cart, cartSpeed];
  const est = measured.map((value, index) => 0.94 * value + 0.06 * predicted[index]);
  const force = Math.max(-5, Math.min(5,
    -2.8 * est[0] - 0.55 * est[1] - 2.2 * est[2] - 0.5 * est[3] - est[4] - 0.8 * est[5]));
  const ramp = Math.min((i + 1) / 20, 1);
  const cartVelocityCommand = Math.max(-5, Math.min(5, est[5] * ramp));
  const stepHz = -(cartVelocityCommand * 6400) / 0.638175;
  const timerCommand = Math.abs(stepHz) > 2
    ? Math.trunc((0.5 / stepHz) * 250000)
    : (stepHz >= 0 ? 65535 : -65535);
  const raw1 = ((Math.round(reference1 - arm1 * countsPerRadian) % 16384) + 16384) % 16384;
  const raw2 = ((Math.round(reference2 - joint2 * countsPerRadian) % 16384) + 16384) % 16384;
  const usedReference = reference1;
  reference1 += cart / 60 + cartVelocityCommand / 30;
  const sample = {
    record_type: 'sample', elapsed_s: round(0.26 + t, 2), cycle: i + 1,
    esp32_packet: i + 61, raw_angle_1: raw1, raw_angle_2: raw2,
    raw_angle_3: 9200 + Math.round(18 * Math.sin(t * 0.8)),
    reference_upright_1_counts: round(usedReference, 5),
    reference_aligned_2_counts: reference2,
    measured_angle_1_rad: round(arm1, 8), measured_angle_1_deg: round(arm1 / radians, 5),
    measured_angle_2_rad: round(arm2, 8), measured_angle_2_deg: round(arm2 / radians, 5),
    joint_angle_2_rad: round(joint2, 8), joint_angle_2_deg: round(joint2 / radians, 5),
    cart_used_m: round(prevCart), cart_steps: Math.round(cart * 6400 / 0.638175),
    cart_m: round(cart),
    pred_theta1_rad: round(predicted[0], 8), pred_theta1_rate_rad_s: round(predicted[1]),
    pred_theta2_rad: round(predicted[2], 8), pred_theta2_rate_rad_s: round(predicted[3]),
    pred_cart_m: round(predicted[4]), pred_cart_velocity_m_s: round(predicted[5]),
    est_theta1_rad: round(est[0], 8), est_theta1_rate_rad_s: round(est[1]),
    est_theta2_rad: round(est[2], 8), est_theta2_rate_rad_s: round(est[3]),
    est_cart_m: round(est[4]), est_cart_velocity_m_s: round(est[5]),
    previous_force_n: round(prevForce), force_command_n: round(force),
    cart_velocity_command_m_s: round(cartVelocityCommand), ramp_fraction: round(ramp, 3),
    step_frequency_hz: round(stepHz, 4), motor_timer_command: timerCommand,
    reference_upright_1_next_counts: round(reference1, 5),
    sensor_read_ms: round(0.32 + 0.06 * Math.sin(t * 4) ** 2, 4),
    position_read_ms: round(0.41 + 0.07 * Math.cos(t * 3) ** 2, 4),
    loop_ms: round(2.3 + 0.2 * Math.sin(t * 4.5) ** 2, 4),
    cycle_period_ms: i === 0 ? '' : 10,
  };
  rows.push(row(sample));
  prevArm1 = arm1; prevArm2 = arm2; prevCart = cart; prevForce = force; prevEst = est;
}
rows.push(row({record_type: 'stop', elapsed_s: 20.27,
  message: 'Ctrl+C; dropped_samples=0; SIMULATED EXAMPLE'}));
if (rows.length !== 2007 || rows.some(values => values.length !== columns.length)) {
  throw new Error('Generated sample count or CSV width is incorrect');
}
const csvText = rows.map(values => values.map(csvField).join(',')).join('\n') + '\n';
const output = path.resolve('outputs/example-20-second-double-run/example_SIMULATED_20_second_double_run.csv');
await fs.mkdir(path.dirname(output), {recursive: true});
await fs.writeFile(output, csvText, 'utf8');
const workbook = await Workbook.fromCSV(csvText, {sheetName: 'Double run data'});
workbook.recalculate();
const check = await workbook.inspect({kind: 'table', range: 'A1:D6',
  include: 'values', tableMaxRows: 6, tableMaxCols: 4, maxChars: 1800});
console.log(check.ndjson);
console.log(`Saved ${output} (${rows.length - 6} samples)`);
