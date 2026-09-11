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
for (const f of ['cyl-on-slide.json', 'cyl-on-slide.ctl.js']) fs.copyFileSync(path.join(REPO, 'scenes', f), path.join(root, 'scenes', f));
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
