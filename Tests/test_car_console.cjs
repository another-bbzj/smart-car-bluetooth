// Run the actual inline script; replace only browser/serial/timer boundaries.
const {test} = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../Tools/car_console.html'), 'utf8').match(/<script>([\s\S]*?)<\/script>/)[1];

function harness({savedTheme = null, storageBlocked = false} = {}) {
  const nodes = new Map(), timers = new Map(), intervals = [];
  const storage = new Map(savedTheme ? [['car-console-theme', savedTheme]] : []);
  let time = 100, timerId = 0;
  function element() {
    return {value: '', checked: false, disabled: false, textContent: '', children: [], attributes: {}, listeners: {},
      setAttribute(name, value) {this.attributes[name] = value;},
      classList: {add() {}, remove() {}, toggle() {}}, addEventListener(name, handler) {this.listeners[name] = handler;}, focus() {},
      appendChild(node) {this.children.push(node);}, removeChild() {this.children.shift();}};
  }
  const document = {documentElement: {dataset: {}}, hidden: false, addEventListener() {},
    querySelectorAll: () => [], createElement: element,
    getElementById(id) {if (!nodes.has(id)) nodes.set(id, element()); return nodes.get(id);}};
  const context = vm.createContext({document, navigator: {serial: {addEventListener() {}}, userAgent: ''},
    window: {addEventListener() {}}, performance: {now: () => time},
    setTimeout(fn, ms) {const id = ++timerId; timers.set(id, {fn, at: time + ms}); return id;},
    clearTimeout(id) {timers.delete(id);}, setInterval(fn) {intervals.push(fn);},
    localStorage: {
      getItem(key) {if (storageBlocked) throw new Error('storage denied'); return storage.get(key) ?? null;},
      setItem(key, value) {if (storageBlocked) throw new Error('storage denied'); storage.set(key,value);}
    },
    requestAnimationFrame() {}, getComputedStyle: () => ({getPropertyValue: key =>
      key === '--chart-grid' ? (document.documentElement.dataset.theme === 'dark' ? '#29364a' : '#e8edf3') : ''}),
    Uint8Array, DataView, console});
  vm.runInContext(source, context);
  const run = code => vm.runInContext(code, context);
  run(`app.connected = true; app.lastSample = [0,0,...Array(14).fill(0)];
    app.lastRx = performance.now(); safeCheck.checked = true; baudSel.value = '9600';
    app.port = {close: async () => {}, getInfo: () => ({})};
    globalThis.sent = []; app.writer = {write: async bytes => sent.push([...bytes]), releaseLock() {}, abort: async () => {}};`);
  return {run, nodes, intervals, storage, async advance(ms) {
    time += ms;
    for (const [id, timer] of [...timers]) if (timer.at <= time) {timers.delete(id); timer.fn();}
    await flush();
  }};
}
async function flush() {for (let i = 0; i < 30; i++) await Promise.resolve();}

test('wire commands match firmware golden vectors', () => {
  const h = harness();
  assert.equal(h.run('hex(frame(CMD_STOP))'), 'C3 3C 02 00 2A');
  assert.equal(h.run('hex(wheelsFrame(80,-80)).slice(12,23)'), '00 50 FF B0');
  assert.equal(h.run("crc8(new TextEncoder().encode('123456789'))".replace("new TextEncoder().encode('123456789')", '[49,50,51,52,53,54,55,56,57]')), 244);
});

test('STOP during outstanding ARM cannot restore pending or armed state', async () => {
  const h = harness();
  h.run('app.writer.write = () => new Promise(resolve => globalThis.finishWrite = resolve); globalThis.arming = arm();');
  h.run('const finishArm = finishWrite; stop(); finishArm();');
  await flush();
  assert.equal(h.run('app.pendingArm'), false);
  h.run('onSample([1,0,...Array(14).fill(0)],1)');
  assert.equal(h.run('app.armed'), false);
});

test('RUN arriving before ARM write resolves is not lost', async () => {
  const h = harness();
  h.run('app.writer.write = () => new Promise(resolve => globalThis.finishWrite = resolve); arm();');
  h.run('onSample([1,0,...Array(14).fill(0)],1); finishWrite();');
  await flush();
  assert.equal(h.run('app.armed'), true);
  assert.equal(h.run('app.pendingArm'), false);
});

test('wheel traffic is bounded and STOP discards queued motion', async () => {
  const h = harness();
  h.run(`app.armed = true; app.keys.add('w');
    app.writer.write = bytes => {sent.push([...bytes]); return new Promise(resolve => globalThis.finishWrite = resolve);};
    for(let i=0;i<40;i++) drive(); stop();`);
  assert.equal(h.run('sent.length'), 1, 'only one write may be in flight');
  h.run('finishWrite()'); await flush();
  assert.equal(h.run('sent[1][2]'), 2, 'STOP must be the next wire command');
});

test('stalled writes disarm and disconnect instead of buffering forever', async () => {
  const h = harness();
  h.run('app.armed = true; app.writer.write = () => new Promise(() => {}); drive();');
  await h.advance(600);
  assert.equal(h.run('app.connected'), false);
  assert.equal(h.run('app.armed'), false);
});

test('normal readable EOF disconnects the UI', async () => {
  const h = harness();
  h.run(`app.port.readable = {getReader: () => ({read: async () => ({done:true}), releaseLock() {}, cancel: async () => {}})};
    app.readLoopPromise = readLoop();`);
  await flush(); await h.advance(1);
  assert.equal(h.run('app.connected'), false);
});

test('reconnect is blocked until previous port cleanup finishes', async () => {
  const h = harness();
  h.run('app.reader = {cancel: () => new Promise(resolve => globalThis.finishCancel = resolve)}; disconnect(false); updateButtons();');
  assert.equal(h.nodes.get('btnConnect').disabled, true);
  h.run('finishCancel()'); await flush();
});

test('PID write failure never reports success', async () => {
  const h = harness();
  await h.run(`$('kp0').value = '2'; $('ki0').value = '0.12'; $('kd0').value = '0';
    app.writer.write = async () => {throw new Error('link lost');}; writePid(0);`);
  assert.equal(h.nodes.get('log').children.some(n => /已写入/.test(n.textContent)), false);
});

test('fractional state and invalid fault are not accepted as fresh telemetry', () => {
  const h = harness();
  h.run(`app.lastSample = null; const raw = new Uint8Array(64); const dv = new DataView(raw.buffer);
    dv.setFloat32(0,0.4,true); parseSample(raw);`);
  assert.equal(h.run('app.lastSample'), null);
  h.run('dv.setFloat32(0,0,true); dv.setFloat32(4,99,true); parseSample(raw);');
  assert.equal(h.run('app.lastSample'), null);
});

test('fragmented and concatenated telemetry retains every complete sample', () => {
  const h = harness();
  h.run(`const raw = new Uint8Array(68); raw.set([0,0,128,127],64);
    feed(new Uint8Array([1,2,3])); feed(raw.slice(0,35)); feed(raw.slice(35)); feed(raw);`);
  assert.equal(h.run('app.rows.length'), 2);
});

test('physical WASD works when input method changes event.key', () => {
  const h = harness();
  assert.equal(h.run("keyName({key:'Process', code:'KeyW'})"), 'w');
});

function chartHarness() {
  const h = harness();
  h.run(`globalThis.strokes = []; globalThis.labels = [];
    const context = {setTransform(){}, clearRect(){}, save(){}, restore(){}, beginPath(){},
      moveTo(){}, lineTo(){}, setLineDash(){}, stroke(){strokes.push({color:this.strokeStyle,width:this.lineWidth});},
      fillText(text){labels.push(String(text));}};
    for (const canvas of [speedChart,pwmChart]) {
      canvas.getBoundingClientRect = () => ({width:800,height:240});
      canvas.getContext = () => context;
    }
    app.series = [{t:100,lRef:10000,lRpm:20,rRef:30,rRpm:25,lPwm:10,rPwm:12}];`);
  return h;
}

test('hiding a speed channel removes its line and its influence on axis scaling', () => {
  const h = chartHarness();
  h.run("setCurveVisible('lRef',false); drawSpeedChart();");
  assert.equal(h.run('strokes.filter(s => s.width > 1).length'), 3);
  assert.equal(h.run("labels.includes('60')"), true);
  assert.equal(h.run("labels.includes('10000')"), false);
});

test('presets and individual PWM visibility produce only selected curves', () => {
  const h = chartHarness();
  h.run("selectCurves('pwm'); setCurveVisible('rPwm',false); drawPwmChart();");
  assert.equal(h.run('strokes.filter(s => s.width > 1).length'), 1);
  h.run('strokes.length = 0; drawSpeedChart();');
  assert.equal(h.run('strokes.filter(s => s.width > 1).length'), 0);
  assert.equal(h.run("labels.some(text => text.includes('未选择'))"), true);
});

test('hiding all curves keeps telemetry and CSV rows, reselecting restores lines', () => {
  const h = chartHarness();
  h.run("selectCurves('none'); onSample([0,0,10,9,20,18,2,3,0,1,2,0.12,0,2,0.12,0],0); drawSpeedChart(); drawPwmChart();");
  assert.equal(h.run('strokes.filter(s => s.width > 1).length'), 0);
  assert.equal(h.run('app.rows.length'), 1);
  assert.equal(h.run('app.rows[0].length'), 17);
  h.run("selectCurves('all'); strokes.length=0; drawSpeedChart(); drawPwmChart();");
  assert.equal(h.run('strokes.filter(s => s.width > 1).length'), 6);
});

test('theme toggle remembers preference and leaves serial and curve selection untouched', () => {
  const h = harness();
  assert.equal(h.run('document.documentElement.dataset.theme'), 'light');
  h.run("selectCurves('pwm'); app.armed=true;");
  h.nodes.get('btnTheme').listeners.click();
  assert.equal(h.run('document.documentElement.dataset.theme'), 'dark');
  assert.equal(h.storage.get('car-console-theme'), 'dark');
  assert.equal(h.run('app.armed && app.connected && visibleCurves.size === 2 && sent.length === 0'), true);
  h.nodes.get('btnTheme').listeners.click();
  assert.equal(h.storage.get('car-console-theme'), 'light');
});

test('saved dark mode is restored; unavailable or invalid storage falls back safely', () => {
  assert.equal(harness({savedTheme:'dark'}).run('document.documentElement.dataset.theme'), 'dark');
  assert.equal(harness({savedTheme:'invalid'}).run('document.documentElement.dataset.theme'), 'light');
  const h = harness({storageBlocked:true});
  h.run("setTheme('dark')");
  assert.equal(h.run('document.documentElement.dataset.theme'), 'dark');
});

test('canvas palette updates when the theme changes', () => {
  const h = chartHarness();
  h.run("setTheme('light'); drawSpeedChart();");
  assert.equal(h.run('strokes[0].color'), '#e8edf3');
  h.run("setTheme('dark'); strokes.length=0; drawSpeedChart();");
  assert.equal(h.run('strokes[0].color'), '#29364a');
});
