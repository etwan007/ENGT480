// Load a digital-twin CSV into Code/RunReview/balancing_visualizer.html (headless).
// Usage: node check_visualizer_load.js RUN.csv
// Harness copied from Code/RunReview/test_balancing_visualizer.js.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, '..', '..', 'RunReview', 'balancing_visualizer.html'), 'utf8');

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

(async () => {
  const filePath = process.argv[2];
  const csv = fs.readFileSync(filePath, 'utf8');
  await getElement('csvFile').listeners.change({target: {files: [{
    name: path.basename(filePath), text: async () => csv,
  }]}});
  assert.equal(getElement('notice').textContent, '');
  assert.equal(getElement('angle2Readout').hidden, false);
  assert.match(getElement('anglePlotTitle').textContent, /arm 2/);
  console.log('visualizer loaded', path.basename(filePath), getElement('duration').textContent,
              getElement('sampleCount').textContent, 'samples');
})().catch(error => { console.error(error); process.exitCode = 1; });
