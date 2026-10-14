const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

const html = fs.readFileSync('balancing_visualizer.html', 'utf8');
const body = html.slice(html.indexOf('  function parseCsv(text) {'), html.indexOf('  async function loadFile(file) {'));
assert(body.includes('function parseCsv'));
const parseCsv = new Function(`${body}\nreturn parseCsv;`)();

assert.deepEqual(parseCsv('a,b\r\n1,"a,b"\r\n2,"a""b"\r\n'), [
  ['a', 'b'], ['1', 'a,b'], ['2', 'a"b']
]);
assert.deepEqual(parseCsv('kind,message\nconfig,"{\""gain\"":1}"\n'), [
  ['kind', 'message'], ['config', '{"gain":1}']
]);
assert.throws(() => parseCsv('a,"unfinished'), /quoted field/);

class FakeElement {
  constructor(id = '') {
    this.id = id;
    this.textContent = '';
    this.value = '';
    this.hidden = false;
    this.disabled = false;
    this.style = {};
    this.parentElement = {clientWidth: 400};
    this.listeners = {};
    this.children = [];
    this.classList = {add() {}, remove() {}};
  }
  addEventListener(name, handler) { this.listeners[name] = handler; }
  replaceChildren(...children) { this.children = children; }
  append(...children) { this.children.push(...children); }
  getBoundingClientRect() { return {width: 400, height: this.id === 'movement' ? 330 : 115}; }
  getContext() {
    return {
      setTransform() {}, clearRect() {}, beginPath() {}, moveTo() {}, lineTo() {},
      stroke() {}, fillRect() {}, arc() {}, fill() {}, fillText() {},
      measureText(text) { return {width: text.length * 6}; }
    };
  }
}

const elements = new Map();
const getElement = id => {
  if (!elements.has(id)) elements.set(id, new FakeElement(id));
  return elements.get(id);
};
const cursors = Array.from({length: 4}, () => new FakeElement());
const document = {
  getElementById: getElement,
  createElement: () => new FakeElement(),
  createTextNode: text => ({textContent: text}),
  querySelectorAll: selector => selector === '.cursor' ? cursors : [],
};
vm.runInNewContext(html.match(/<script>(.*?)<\/script>/s)[1], {
  document, window: {addEventListener() {}}, requestAnimationFrame() {},
  devicePixelRatio: 1, Float64Array, Number, Math, console,
});

async function checkLoad(filePath, double, diagnostics = false) {
  let csv = fs.readFileSync(filePath, 'utf8');
  if (diagnostics) {
    const rows = parseCsv(csv);
    const extra = ['esp32_sequence', 'receiver_uptime_ms',
      'angle_status_1', 'angle_status_2', 'angle_status_3',
      'radio_age_1_ms', 'radio_age_2_ms', 'radio_age_3_ms',
      'serial_crc_errors', 'serial_discarded_bytes', 'serial_sequence_gaps', 'serial_superseded_frames'];
    const message = rows[0].indexOf('message');
    const kind = rows[0].indexOf('record_type');
    for (const row of rows.slice(1)) {
      if (row[kind] === 'config') {
        const config = JSON.parse(row[message]);
        config.schema_version = 2;
        config.angle_protocol_version = 1;
        row[message] = JSON.stringify(config);
      }
      row.splice(message, 0, ...(row[kind] === 'sample'
        ? ['123', '1230', 'valid', double ? 'valid' : 'never_received', 'never_received',
           '2', double ? '3' : '', '', '1', '37', '1', '2']
        : extra.map(() => '')));
    }
    rows[0].splice(message, 0, ...extra);
    const escape = value => '"' + String(value).replaceAll('"', '""') + '"';
    csv = rows.map(row => row.map(escape).join(',')).join('\n');
  }
  await getElement('csvFile').listeners.change({target: {files: [{
    name: filePath.split('/').at(-1), text: async () => csv,
  }]}});
  assert.equal(getElement('notice').textContent, '');
  assert.equal(getElement('duration').textContent, '20.00 s');
  assert.equal(getElement('sampleCount').textContent, '2,001');
  assert.equal(getElement('angle2Readout').hidden, !double);
  assert.match(getElement('anglePlotTitle').textContent, double ? /arm 2/ : /Measured tilt/i);
  if (double) assert.match(getElement('angle2Value').textContent, /°$/);
  if (diagnostics) {
    const values = Object.fromEntries(getElement('allValues').children.map(item =>
      item.children.map(child => child.textContent)));
    assert.equal(values.angle_status_1, 'valid');
    assert.equal(values.radio_age_1_ms, '2');
    assert.equal(values.serial_crc_errors, '1');
    assert(getElement('metric').children.some(option => option.value === 'radio_age_1_ms'));
  }
}

(async () => {
  await checkLoad('outputs/example-20-second-run/example_SIMULATED_20_second_run.csv', false);
  await checkLoad('outputs/example-20-second-double-run/example_SIMULATED_20_second_double_run.csv', true);
  await checkLoad('outputs/example-20-second-run/example_SIMULATED_20_second_run.csv', false, true);
  await checkLoad('outputs/example-20-second-double-run/example_SIMULATED_20_second_double_run.csv', true, true);
  console.log('Visualizer CSV parsing and single/double loading with both CSV schemas OK');
})().catch(error => { console.error(error); process.exitCode = 1; });
