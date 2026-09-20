// Static checks on web/ and the HTTP layer: the rules that make the picture lie, or open the
// server, without any error.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { staticPath, postAllowed } from '../server/http.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const read = f => fs.readFileSync(path.join(ROOT, f), 'utf8');
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

// importmap: three from node_modules through the server, never a CDN
const html = read('web/index.html');
const map = JSON.parse(/<script type="importmap">([\s\S]*?)<\/script>/.exec(html)[1]).imports;
chk('importmap has no CDN (offline on factory PCs)', Object.values(map).every(u => u.startsWith('/vendor/three/')) && !/https?:\/\//.test(html), JSON.stringify(map));
chk('importmap targets exist in node_modules', ['three', 'three/addons/'].every(k => fs.existsSync(staticPath(ROOT, map[k] + (k.endsWith('/') ? 'controls/OrbitControls.js' : '')))));

// lib/ is imported by Node AND the browser
for (const f of fs.readdirSync(path.join(ROOT, 'lib')).filter(f => f.endsWith('.js'))) {
  const src = read('lib/' + f);
  const bad = [...src.matchAll(/^\s*import\s[^;]*?from\s+['"]([^'"]+)['"]/gm)].map(m => m[1]).filter(s => !s.startsWith('./'));
  chk('lib/' + f + ' imports only lib/ (no three, Rapier or node:)', bad.length === 0, bad.join(' '));
}

// web/: poses only from worldPoses, the rot rule only from lib/math.js
const app = read('web/app.js');
chk('web/app.js takes poses from worldPoses() in lib/scene.js', /import\s*\{[^}]*\bworldPoses\b[^}]*\}\s*from\s*'\/lib\/scene\.js'/.test(app));
chk('web/ has no pose math of its own (no Euler, no second worldPoses)', !/Euler|function\s+worldPoses|\bcompose\(/.test(app));
chk('Z-up set before anything is created', /DEFAULT_UP\.set\(0,\s*0,\s*1\)/.test(app) && app.indexOf('DEFAULT_UP') < app.indexOf('new THREE.'));
const onState = app.slice(app.indexOf('function onState('), app.indexOf('function dofAt('));
chk('SSE handler updates data only (no DOM rebuild per message)', onState.length > 50 && !/innerHTML|replaceChildren|createElement|textContent/.test(onState));
const pm = /PANEL_MS\s*=\s*(\d+)/.exec(app);
chk('panel text throttled to about 8 per second', pm && +pm[1] >= 100 && +pm[1] <= 200, pm && pm[1] + ' ms');
chk('inputs send on change, never on input', !/addEventListener\(\s*['"]input['"]|\.oninput\s*=/.test(app));
chk('no innerHTML in web/app.js (tag names never become markup)', !/innerHTML/.test(app));

// web/editor.js: the same rules, plus the editor's own
const edj = read('web/editor.js');
chk('editor: mount math only from lib/scene.js (mountPoses/mountFrom), no Euler, no compose', /import\s*\{[^}]*\bmountFrom\b[^}]*\}\s*from\s*'\/lib\/scene\.js'/.test(edj) && !/Euler|\bcompose\(|\binvert\(/.test(edj));
chk('editor: saves only after the shared validate()', /import\s*\{[^}]*\bvalidate\b[^}]*\}\s*from\s*'\/lib\/scene\.js'/.test(edj) && /async function save\(\)\s*\{\s*const errs = validate\(scene\)/.test(edj));
chk('editor: PUT carries baseVersion (409 on stale)', /baseVersion:\s*base/.test(edj));
chk('editor: TransformControls added through getHelper() (r169+)', /tc\.getHelper\(\)/.test(edj) && !/scene3\.add\([^)]*\btc\b[,)]/.test(edj));
chk('editor: undo is capped at 100 snapshots', /UNDO_MAX\s*=\s*100\b/.test(edj));
chk('editor: inputs send on change, never on input; no innerHTML', !/addEventListener\(\s*['"]input['"]|\boninput\b/.test(edj) && !/innerHTML/.test(edj));
chk('editor: every #id it uses exists in index.html', [...edj.matchAll(/\$\('([\w-]+)'\)/g)].map(m => m[1]).every(id => html.includes('id="' + id + '"')),
  [...new Set([...edj.matchAll(/\$\('([\w-]+)'\)/g)].map(m => m[1]))].filter(id => !html.includes('id="' + id + '"')).join(' '));
chk('viewer: pressing parts is off in edit mode', /editor\.active\)\s*return/.test(app));

// scene and controller pickers: the page asks, the server loads and connects
chk('viewer: the scene and controller pickers exist and post to /api/switch',
  html.includes('id="scene-pick"') && html.includes('id="mode-pick"') && /'\/api\/switch'/.test(app));
chk('viewer: the pickers send on change, never on input', /\.onchange\s*=\s*switchTo/.test(app));
chk('editor: the pickers are locked while editing', /'scene-pick', 'mode-pick'/.test(edj));

// staticPath: fixed prefixes only, never out of them
const is = (u, rel) => staticPath(ROOT, u) === (rel && path.join(ROOT, ...rel.split('/')));
chk('/ -> web/index.html', is('/', 'web/index.html'));
chk('/lib/scene.js and /vendor/three/... map to their folders', is('/lib/scene.js', 'lib/scene.js') && is('/vendor/three/build/three.module.js', 'node_modules/three/build/three.module.js'));
chk('/assets/... serves a scene\'s 3D shells', is('/assets/robots/lrmate200id/meshes/link_1.stl', 'assets/robots/lrmate200id/meshes/link_1.stl'));
chk('the STL loader the viewer imports exists in node_modules', fs.existsSync(staticPath(ROOT, '/vendor/three/examples/jsm/loaders/STLLoader.js')));
// Robot makers do not agree on a format: ROS-Industrial ships the Fanuc as STL, DENSO ships its
// arms as COLLADA. The viewer imports both loaders, so both have to be there.
chk('the COLLADA loader the viewer imports exists in node_modules', fs.existsSync(staticPath(ROOT, '/vendor/three/examples/jsm/loaders/ColladaLoader.js')));
for (const u of ['/web/../server/pki/key.pem', '/web/%2e%2e/server/plant.js', '/lib/..%2fserver%2fplant.js', '/web/..\\server', '/web//x',
                 '/server/plant.js', '/scenes/cyl-on-slide.ctl.js', '/web/', '/node_modules/three/package.json', '/web/%E0%A4%A']) {
  chk('refused: ' + u, staticPath(ROOT, u) === null);
}

// The operator panel: HTML beside the view, hideable; operator devices are not drawn in 3D
{
  chk('viewer: operator devices (selector, pushbuttons, lamps) are drawn in the HTML panel, not in 3D', /group === 'operator'\) continue/.test(app) && /function buildOpPanel/.test(app));
  chk('viewer: operator panel buttons send edges (down AND up), never a state', /edge\(c\.id, 'pb', true\)/.test(app) && /edge\(c\.id, 'pb', false\)/.test(app) && /onpointerleave = up/.test(app));
  chk('viewer: the operator panel is built once per scene and can be hidden', /buildOpPanel\(scene\)/.test(app) && /op-hide/.test(app)
    && ['oppanel', 'op-hide', 'op-body'].every(id => html.includes('id="' + id + '"')));
}

// The speed override is a slider, and a slider sends on change, never while it is dragged
chk('viewer: the speed override is drawn as a slider that sends on change', /t\.dialKey/.test(app) && /inp\.onchange = \(\) =>/.test(app)
  && /api\/dial/.test(app) && !/inp\.oninput/.test(app));

// Slow motion: the render clock has to tick at the plant's rate, or the interpolation offset
// drifts away from it and the picture jumps.
chk('viewer: the world speed picker sends on change and the render clock follows the plant',
  html.includes('id="scale-pick"') && /\$\('scale-pick'\)\.onchange/.test(app) && /api\/scale/.test(app)
  && /performance\.now\(\) \* simScale \+ offset/.test(app) && /m\.t - performance\.now\(\) \* simScale/.test(app));

// Colour: a machine drawn all in grey is unreadable, so each family has its own material. A
// material a component type uses but the viewer does not know would silently fall back to grey.
{
  const { TYPES, withDefaults } = await import('../lib/components.js');
  const used = new Set();
  for (const ty of Object.values(TYPES)) {
    const p = withDefaults(ty, {});
    for (const sh of (ty.shapes ? ty.shapes(p) : [])) used.add(sh.mat);
  }
  const blk = app.slice(app.indexOf('const MAT = {'), app.indexOf('};', app.indexOf('const MAT = {')));
  const known = new Set([...blk.matchAll(/(\w+): \{ color/g)].map(m => m[1]));
  const missing = [...used].filter(m => !known.has(m));
  chk('viewer: every material a component uses is in the palette', missing.length === 0, missing.join(' '));
  chk('viewer: the palette separates the families, it is not all grey',
    ['motion', 'tool', 'holder', 'sensor'].every(m => known.has(m)), [...known].join(' '));
}

// The warnings list can be tidied without losing anything: the plant's event log and the
// recording on disk still have every warning.
chk('viewer: the warnings list has a clear button, and it only empties the list',
  html.includes('id="warn-clear"') && app.includes("$('warn-clear').onclick = () => $('warns').replaceChildren();"));

// The operator panel must not move under the pointer: the status block above it keeps a fixed
// height however many hints it is showing, or the buttons shift and get pressed by mistake.
chk('viewer: the status block has a fixed height, so the panel below it stays put', /#status \{[^}]*min-height/.test(html));
chk('viewer: the status shows both clocks and the cycle time', /function hms\(/.test(app) && /sim ' \+ hms\(pl\.t\)/.test(app)
  && /wall ' \+ hms\(Date\.now\(\) - pageAt\)/.test(app) && /pl\.cycleMs/.test(app) && /pl\.avgMs/.test(app));
chk('viewer: a latching mushroom shows its own state, not a lamp', /kind === 'alternate'/.test(app) && /'latched'/.test(app));

// Walking in (web/pov.js) and the hall it walks in (web/workshop.js). Both are the VIEWER's
// alone: the plant never hears about either, and every rule here is one that fails quietly.
{
  const povjs = read('web/pov.js'), shop = read('web/workshop.js');
  chk('viewer: the walk-in camera is written here - PointerLockControls is Y-up only and tips a Z-up horizon over',
    !/from\s+'three\/addons\/controls\/PointerLockControls/.test(povjs + app));
  chk('viewer: the walk is a CAMERA - pov.js reaches no tag, no DOF and no server',
    !/fetch\(|sendBeacon|worldPoses|lib\/scene/.test(povjs.replace(/\/\/.*$/gm, '')));
  chk('viewer: pressing and grabbing from the walk go through the same press/hold the orbit camera uses',
    /if \(pov\.active\) return pickNdc\(0, 0, list\)/.test(app) && /dragFromCrosshair/.test(app));
  chk('viewer: the walk gives the orbit camera back exactly as it found it',
    /saved\.p\.copy\(camera\.position\)/.test(povjs) && /camera\.position\.copy\(saved\.p\)/.test(povjs) && /controls\.enabled = true/.test(povjs));
  chk('viewer: while walking, OrbitControls is off - two cameras on one camera shake the picture',
    /if \(pov\.active\) \{ pov\.update\(dt\); dragFromCrosshair\(now\); \} else controls\.update\(\);/.test(app)
    && /controls\.enabled = !pov\.active/.test(app));
  chk('viewer: a jump is latched on the key EDGE, so a tap between two frames is not lost', /jumpWanted = true/.test(povjs) && /jumpWanted && onGround/.test(povjs));
  chk('viewer: a ghost zone is drawn, not walked into', /userData\.ghost = !!s\.ghost/.test(app) && /!h\.object\.userData\.ghost/.test(povjs));
  chk('viewer: the step a walk takes is tested by probing DOWN where the foot lands (rays alone miss a belt deck)',
    /function free\(x, y, d\)/.test(povjs) && /STEP_UP/.test(povjs));
  chk('viewer: editing and walking are not both on at once', /if \(!editor\.active\) pov\.toggle\(\)/.test(app) && /addEventListener\('click', \(\) => pov\.exit\(\)\)/.test(app));
  chk('workshop: it is scenery - nothing in lib/ or server/ knows it exists',
    !fs.readdirSync(path.join(ROOT, 'lib')).concat(fs.readdirSync(path.join(ROOT, 'server')))
      .some(f => f.endsWith('.js') && /workshop/.test(read(fs.existsSync(path.join(ROOT, 'lib', f)) ? 'lib/' + f : 'server/' + f))));
  chk('workshop: it imports three and nothing else - no scene, no tag, no server',
    /^import \* as THREE from 'three';$/m.test(shop) && [...shop.matchAll(/^import .*$/gm)].length === 1
    && !/fetch\(|lib\/scene|colliderDesc/.test(shop));
  chk('workshop: it is sized to the machine, and it replaces the grid rather than standing on it',
    /workshop\.fit\(box\)/.test(app) && /grid\.visible = floor\.visible = !on/.test(app));
  chk('viewer: no innerHTML in the new viewer files either', !/innerHTML/.test(povjs + shop));
}

// Every panel hides, and a hidden panel can be brought back: a page with no controls at all is
// the one way this feature breaks for good.
{
  const targets = [...html.matchAll(/data-hide="([\w-]+)"/g)].map(m => m[1]);
  chk('viewer: every hide button names an element that exists', targets.length >= 3 && targets.every(id => html.includes('id="' + id + '"')), targets.join(' '));
  chk('viewer: the whole side panel hides and comes back', /id="panel-hide"/.test(html) && /id="panel-show"/.test(html)
    && /\$\('panel-show'\)\.hidden = on/.test(app) && /\$\('panel-show'\)\.onclick = \(\) => setPanel\(true\)/.test(app));
  chk('viewer: aside[hidden] really is hidden (display:flex would beat the attribute)', /aside\[hidden\] \{ display: none; \}/.test(html));
  chk('viewer: F walks in and H hides the panel, and the hint says so',
    /k === 'f'\) pov\.toggle\(\)/.test(app) && /k === 'h'\) setPanel\(aside\.hidden\)/.test(app) && /F: walk in/.test(html) && /H: hide the panel/.test(html));
  chk('viewer: the editor hides the IO table, and its heading follows it', /MutationObserver\(\(\) => \{ \$\('io-head'\)\.hidden = editor\.active; \}\)/.test(app));
}

// POST: local only unless --lan-control, and the browser Origin must match the Host
const req = (ra, origin, host = '127.0.0.1:7660') => ({ socket: { remoteAddress: ra }, headers: { host, ...(origin ? { origin } : {}) } });
chk('POST from this PC without Origin (curl)', postAllowed(req('127.0.0.1')));
chk('POST from this PC, same-origin page', postAllowed(req('::1', 'http://127.0.0.1:7660')));
chk('POST refused: page from another site', !postAllowed(req('127.0.0.1', 'http://evil.example')));
chk('POST refused: from the LAN by default', !postAllowed(req('192.168.1.20', 'http://192.168.1.5:7660', '192.168.1.5:7660')));
chk('POST from the LAN with --lan-control', postAllowed(req('192.168.1.20', 'http://192.168.1.5:7660', '192.168.1.5:7660'), { lanControl: true }));
chk('POST refused: DNS rebinding (local browser, Origin == Host == another name)', !postAllowed(req('127.0.0.1', 'http://evil.example:7660', 'evil.example:7660')));
chk('POST from this PC as localhost', postAllowed(req('::1', 'http://localhost:7660', 'localhost:7660')));

process.exit(fail ? 1 : 0);
