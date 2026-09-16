// The editor end to end in headless Chrome over CDP, on a TEMP copy of the scenes: enter edit
// mode, select, edit, undo/redo, add, save to disk (plant rebuilt), 3D click, exit, no page
// errors. Slow (~15 s), so it runs only with MIO_BROWSER=1 (MIO_CHROME = path to chrome.exe).
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const REPO = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const CHROME = process.env.MIO_CHROME || 'C:/Program Files/Google/Chrome/Application/chrome.exe';
if (!process.env.MIO_BROWSER) { console.log('  SKIP  editor in headless Chrome: set MIO_BROWSER=1 (and MIO_CHROME if Chrome is elsewhere)'); process.exit(0); }
if (!fs.existsSync(CHROME)) { console.log('  SKIP  editor in headless Chrome: no Chrome at ' + CHROME + ' (set MIO_CHROME)'); process.exit(0); }

const { serve } = await import('../server/http.js');
const PORT = 7674, DBG = 9338, LINKS = ['web', 'lib', 'node_modules'];
const sleep = ms => new Promise(r => setTimeout(r, ms));
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x !== undefined && x !== '' ? '   ' + x : '')); };

const root = fs.mkdtempSync(path.join(os.tmpdir(), 'mio-ed-'));
const prof = fs.mkdtempSync(path.join(os.tmpdir(), 'mio-chrome-'));
for (const d of LINKS) fs.symlinkSync(path.join(REPO, d), path.join(root, d), 'junction');
fs.mkdirSync(path.join(root, 'scenes'));
// a-to-b comes along for the hand check at the end: it has loose parts on a belt to grab.
for (const f of ['cyl-on-slide.json', 'cyl-on-slide.ctl.js', 'a-to-b.json', 'a-to-b.ctl.js']) fs.copyFileSync(path.join(REPO, 'scenes', f), path.join(root, 'scenes', f));
fs.writeFileSync(path.join(root, 'package.json'), '{"type":"module"}');
const sceneFile = path.join(root, 'scenes/cyl-on-slide.json');

const s = await serve({ root, port: PORT, internal: true, log: () => {} });
const chrome = spawn(CHROME, ['--headless=new', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--no-first-run',
  '--no-default-browser-check', '--user-data-dir=' + prof, '--window-size=1400,850', '--remote-debugging-port=' + DBG, 'about:blank'], { stdio: 'ignore' });
let ws;
const errors = [];
try {
  let targets = [];
  for (let i = 0; i < 50 && !targets.some(t => t.type === 'page'); i++) {
    try { targets = await (await fetch(`http://127.0.0.1:${DBG}/json/list`)).json(); } catch { await sleep(200); }
  }
  ws = new WebSocket(targets.find(t => t.type === 'page').webSocketDebuggerUrl);
  await new Promise(r => ws.addEventListener('open', r));
  let id = 0;
  const pending = new Map();
  ws.addEventListener('message', m => {
    const d = JSON.parse(m.data);
    if (d.id && pending.has(d.id)) { pending.get(d.id)(d); pending.delete(d.id); }
    if (d.method === 'Runtime.exceptionThrown') errors.push('EXC ' + (d.params.exceptionDetails.exception?.description || d.params.exceptionDetails.text));
    if (d.method === 'Runtime.consoleAPICalled' && ['error', 'warning'].includes(d.params.type)) errors.push(d.params.type + ' ' + d.params.args.map(a => a.value ?? a.description).join(' '));
    if (d.method === 'Log.entryAdded' && d.params.entry.level === 'error') errors.push('LOG ' + d.params.entry.text + ' ' + (d.params.entry.url || ''));
  });
  const cmd = (method, params = {}) => new Promise(r => { const i = ++id; pending.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
  const ev = async expr => { const r = await cmd('Runtime.evaluate', { expression: expr, returnByValue: true, awaitPromise: true }); return r.result.result.value ?? r.result.exceptionDetails?.exception?.description; };
  const waitFor = async (expr, ms = 8000) => { for (const t0 = Date.now(); Date.now() - t0 < ms; await sleep(100)) if (await ev(expr)) return true; return false; };

  await cmd('Runtime.enable'); await cmd('Log.enable'); await cmd('Page.enable');
  await cmd('Emulation.setDeviceMetricsOverride', { width: 1400, height: 850, deviceScaleFactor: 1, mobile: false });
  await cmd('Page.navigate', { url: `http://127.0.0.1:${PORT}/` });
  await waitFor(`document.title.startsWith('cyl-on-slide')`);
  await ev(`addEventListener('unhandledrejection', e => console.error('UNHANDLED ' + (e.reason?.stack || e.reason)));
    window.$ = id => document.getElementById(id);
    window.fld = lab => [...document.querySelectorAll('#ed-props .ed-f')].find(f => f.firstChild.textContent === lab);
    window.setIn = (inp, v) => { inp.value = v; inp.dispatchEvent(new Event('change', { bubbles: true })); }; 1`);
  const nComp = JSON.parse(fs.readFileSync(sceneFile, 'utf8')).components.length;
  const atX = `+fld('at [mm]').querySelectorAll('input')[0].value`;

  await ev(`$('edit-btn').click()`);
  chk('Edit opens the editor and hides the IO table', await waitFor(`$('ed-msg').textContent.startsWith('editing')`)
    && await ev(`!$('editor').hidden && document.querySelector('.tablewrap').hidden && $('edit-btn').textContent === 'Exit edit'`));
  chk('the plant stops in edit mode', s.plant.mode === 'stop', s.plant.mode);
  chk('the tree lists every component', await ev(`document.querySelectorAll('.ed-row').length`) === nComp, nComp);
  chk('no validation problems on the committed scene', await ev(`$('ed-errs').children.length`) === 0, await ev(`$('ed-errs-h').textContent`));

  await ev(`[...document.querySelectorAll('.ed-row')].find(r => r.firstChild.textContent === 'part1').click()`);
  chk('selecting part1 in the tree fills the properties', await waitFor(`fld('id')?.querySelector('input').value === 'part1'`));
  const x0 = await ev(atX);
  await ev(`setIn(fld('at [mm]').querySelectorAll('input')[0], ${x0 + 20})`);
  chk('editing at[0] marks the scene dirty and enables Undo', await waitFor(`$('ed-save').textContent === 'Save *' && !$('ed-undo').disabled`));
  await ev(`$('ed-undo').click()`);
  chk('Undo returns to the saved scene', await waitFor(`$('ed-save').textContent === 'Save' && ${atX} === ${x0}`));
  await ev(`$('ed-redo').click()`);
  chk('Redo re-applies the edit', await waitFor(`${atX} === ${x0 + 20}`));

  await ev(`$('ed-type').value = 'lamp'; $('ed-add').click()`);
  chk('Add mounts a new lamp on the selected part', await waitFor(`fld('parent')?.querySelector('select').value === 'part1'`), await ev(`fld('id').querySelector('input').value`));
  await ev(`$('ed-undo').click()`);
  chk('Undo removes the added lamp', await waitFor(`document.querySelectorAll('.ed-row').length === ${nComp}`));

  await ev(`$('ed-save').click()`);
  await waitFor(`$('ed-msg').textContent.startsWith('saved') || $('ed-msg').className === 'bad'`, 15000);
  const disk = JSON.parse(fs.readFileSync(sceneFile, 'utf8'));
  chk('Save writes the edit to disk', disk.components.find(c => c.id === 'part1').at[0] === x0 + 20, await ev(`$('ed-msg').textContent`));
  chk('the saved file is canonical (stringify)', fs.readFileSync(sceneFile, 'utf8') === (await import('../lib/scene.js')).stringify(disk));
  chk('the server serves the saved scene', (await (await fetch(`http://127.0.0.1:${PORT}/api/scene/cyl-on-slide`)).json()).scene.components.find(c => c.id === 'part1').at[0] === x0 + 20);
  chk('after Save the editor is clean', await ev(`$('ed-save').textContent`) === 'Save');

  for (const type of ['mousePressed', 'mouseReleased']) await cmd('Input.dispatchMouseEvent', { type, x: 480, y: 430, button: 'left', clickCount: 1 });
  chk('a click in 3D selects the part under the cursor', await waitFor(`!!document.querySelector('.ed-row.sel')`), await ev(`document.querySelector('.ed-row.sel')?.firstChild.textContent`));

  await ev(`$('edit-btn').click()`);
  chk('Exit edit shows the IO table again', await waitFor(`$('editor').hidden && !document.querySelector('.tablewrap').hidden`));
  // -------------------------------------------------------------- the hand, through a real mouse
  // plant.test.js calls holdPart() directly, so nothing else covers the viewer wiring
  // (pointerdown on a part -> POST /api/hold). a-to-b gets its own server: the object serve()
  // returns keeps the plant it was built with, so a /api/switch would leave us reading the old one.
  const ab = await serve({ root, port: PORT + 1, internal: true, sceneName: 'a-to-b', log: () => {} });
  try {
    const abPost = (p, b) => fetch(`http://127.0.0.1:${PORT + 1}${p}`, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(b) });
    await cmd('Page.navigate', { url: `http://127.0.0.1:${PORT + 1}/` });
    chk('the viewer loads the a-to-b scene', await waitFor(`document.title.startsWith('a-to-b')`), await ev(`location.href + ' ' + document.title`));
    await abPost('/api/cmd', { op: 'run' });
    // The whole start-up through the HTML operator panel, with real pointer events: MASTER ON,
    // HOME POS, START. The page sends edges only; the controller decides what each one may do.
    const opBtn = txt => "[...document.querySelectorAll('#op-body .op-btn')].find(b => b.textContent === " + JSON.stringify(txt) + ')';
    const tapBtn = async txt => {
      for (const type of ['pointerdown', 'pointerup']) {
        await ev(opBtn(txt) + ".dispatchEvent(new PointerEvent(" + JSON.stringify(type) + ", { bubbles: true }))");
        await sleep(150);
      }
    };
    chk("the operator panel lists the machine's buttons", await waitFor(opBtn('START') + ' && ' + opBtn('MASTER ON') + ' && ' + opBtn('HOME POS') + ' && ' + opBtn('IND BELT')),
      await ev("[...document.querySelectorAll('#op-body .op-btn')].map(b => b.textContent).join(', ')"));
    chk('the speed override is on the panel as a slider', await waitFor("document.querySelector('#op-body .op-dial input[type=range]')"),
      await ev("document.querySelectorAll('#op-body .op-dial').length + ' dials'"));
    await tapBtn('MASTER ON');
    chk('pressing MASTER ON in the panel energises the machine', ab.plant.io.PL_MASTER === true, 'PL_MASTER ' + ab.plant.io.PL_MASTER);
    await tapBtn('HOME POS');
    await sleep(600);
    chk('HOME POS homes it, and the button lights', ab.plant.io.PL_HOME === true, 'PL_HOME ' + ab.plant.io.PL_HOME);
    chk('the lit state reaches the page, so the operator sees which outputs are on',
      await waitFor(opBtn('HOME POS') + ".classList.contains('lit')"), await ev(opBtn('HOME POS') + '.className'));
    await tapBtn('START');
    chk('START then runs the sequence (AUTO_RUN on)', ab.plant.io.PB_START === false && ab.plant.io.AUTO_RUN === true,
      'PB_START ' + ab.plant.io.PB_START + ', AUTO_RUN ' + ab.plant.io.AUTO_RUN);
    const pinned = () => [...ab.plant.parts.values()].filter(p => p.pin).length;
    for (let i = 0; i < 200 && ab.plant.parts.size === 0; i++) await sleep(100);
    await sleep(1200);                                          // the viewer renders 50 ms behind
    // Click the part where it actually is. Sweeping the view with clicks instead presses the
    // machine's own pushbuttons: a first attempt hit STOP, the line stopped feeding, and the
    // sweep then had nothing left to grab (1380 points, 62 s, no hit).
    const mouse = (type, x, y) => cmd('Input.dispatchMouseEvent', { type, x, y, button: 'left', buttons: type === 'mousePressed' ? 1 : 0, clickCount: 1 });
    let hit = null, where = '';
    // The part is on a running belt, so the point the page reports is already a little stale by
// the time the press lands. Try again quickly rather than waiting: a slow retry is a wronger aim.
    for (let i = 0; i < 12 && !hit; i++) {
      const uid = [...ab.plant.parts.keys()][0];
      const at = uid && await ev(`window.mioPartScreen(${JSON.stringify(uid)})`);
      if (!at) { await sleep(500); continue; }
      const x = Math.round(at[0]), y = Math.round(at[1]);
      where = uid + ' at ' + x + ',' + y;
      await mouse('mousePressed', x, y);
      await sleep(120);
      if (pinned() > 0) hit = [x, y]; else await mouse('mouseReleased', x, y);
    }
    chk('pressing the mouse on a part in 3D jams it in the plant', !!hit, where || 'no part on screen');
    if (hit) {
      const held = [...ab.plant.parts.values()].find(p => p.pin), x0 = held.body.translation().x;
      await sleep(1500);
      chk('the jammed part stays put while the belt runs under it', Math.abs(held.body.translation().x - x0) < 5e-4,
        ((held.body.translation().x - x0) * 1000).toFixed(3) + ' mm');
      await cmd('Input.dispatchMouseEvent', { type: 'mouseMoved', x: hit[0] - 120, y: hit[1] - 60, button: 'left', buttons: 1 });
      await sleep(500);
      const dx = (held.body.translation().x - x0) / 0.001;
      chk('dragging the mouse moves the part with it', Math.abs(dx) > 20, dx.toFixed(0) + ' mm');
      await mouse('mouseReleased', hit[0] - 120, hit[1] - 60);
      await sleep(800);
      chk('releasing the mouse lets the part go', pinned() === 0, pinned() + ' still pinned');
    }
  } finally { await ab.close(); }

  chk('no page errors', errors.length === 0, errors.join('\n  '));
} finally {
  ws?.close();
  chrome.kill();
  await s.close();
  await sleep(300);
  // Junctions first, non-recursively: a recursive delete must never walk into the real repo.
  for (const d of LINKS) { const j = path.join(root, d); try { fs.unlinkSync(j); } catch { try { fs.rmdirSync(j); } catch { /* checked below */ } } }
  if (LINKS.every(d => !fs.existsSync(path.join(root, d)))) fs.rmSync(root, { recursive: true, force: true });
  else console.log('  note: left ' + root + ': a junction could not be removed');
  fs.rmSync(prof, { recursive: true, force: true, maxRetries: 5, retryDelay: 200 });
}
process.exit(fail ? 1 : 0);
