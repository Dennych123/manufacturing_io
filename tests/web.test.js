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

// staticPath: fixed prefixes only, never out of them
const is = (u, rel) => staticPath(ROOT, u) === (rel && path.join(ROOT, ...rel.split('/')));
chk('/ -> web/index.html', is('/', 'web/index.html'));
chk('/lib/scene.js and /vendor/three/... map to their folders', is('/lib/scene.js', 'lib/scene.js') && is('/vendor/three/build/three.module.js', 'node_modules/three/build/three.module.js'));
for (const u of ['/web/../server/pki/key.pem', '/web/%2e%2e/server/plant.js', '/lib/..%2fserver%2fplant.js', '/web/..\\server', '/web//x',
                 '/server/plant.js', '/scenes/cyl-on-slide.ctl.js', '/web/', '/node_modules/three/package.json', '/web/%E0%A4%A']) {
  chk('refused: ' + u, staticPath(ROOT, u) === null);
}

// POST: local only unless --lan-control, and the browser Origin must match the Host
const req = (ra, origin, host = '127.0.0.1:7660') => ({ socket: { remoteAddress: ra }, headers: { host, ...(origin ? { origin } : {}) } });
chk('POST from this PC without Origin (curl)', postAllowed(req('127.0.0.1')));
chk('POST from this PC, same-origin page', postAllowed(req('::1', 'http://127.0.0.1:7660')));
chk('POST refused: page from another site', !postAllowed(req('127.0.0.1', 'http://evil.example')));
chk('POST refused: from the LAN by default', !postAllowed(req('192.168.1.20', 'http://192.168.1.5:7660', '192.168.1.5:7660')));
chk('POST from the LAN with --lan-control', postAllowed(req('192.168.1.20', 'http://192.168.1.5:7660', '192.168.1.5:7660'), { lanControl: true }));

process.exit(fail ? 1 : 0);
