// Scene storage for the editor: versions, 409 on stale saves, validation, byte-identical saves.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { readScene, saveScene, versionOf } from '../server/scenes.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };
const code = fn => { try { fn(); return 0; } catch (e) { return e.code ?? String(e.message); } };

const root = fs.mkdtempSync(path.join(os.tmpdir(), 'mio-scenes-'));
fs.mkdirSync(path.join(root, 'scenes'));
const orig = fs.readFileSync(path.join(ROOT, 'scenes/cyl-on-slide.json'));
const file = path.join(root, 'scenes/cyl-on-slide.json');
fs.writeFileSync(file, orig);
try {
  const a = readScene(root, 'cyl-on-slide');
  chk('read: version is the hash of the file bytes', a.version === versionOf(orig), a.version);

  const same = saveScene(root, 'cyl-on-slide', a.version, a.scene);
  chk('save unchanged scene: no write, same version', !same.changed && same.version === a.version && fs.readFileSync(file).equals(orig));

  const s = structuredClone(a.scene);
  s.components.find(c => c.id === 'part1').at[0] += 10;
  const b = saveScene(root, 'cyl-on-slide', a.version, s);
  const t1 = fs.readFileSync(file);
  chk('save edited scene: written, new version', b.changed && b.version !== a.version && b.version === versionOf(t1));
  chk('no .tmp left behind', !fs.existsSync(file + '.tmp'));

  chk('stale baseVersion -> 409', code(() => saveScene(root, 'cyl-on-slide', a.version, a.scene)) === 409);
  chk('file untouched by the refused save', fs.readFileSync(file).equals(t1));

  // Key order and number formatting from the client must not matter: canonical text only.
  const c = saveScene(root, 'cyl-on-slide', b.version, JSON.parse(JSON.stringify(s, null, 7)));
  chk('two saves of the same scene are byte-identical', !c.changed && fs.readFileSync(file).equals(t1));

  const bad = structuredClone(s); bad.components.push({ id: 'x', type: 'nope' });
  chk('invalid scene -> 422 with errors', code(() => saveScene(root, 'cyl-on-slide', b.version, bad)) === 422);
  chk('scene.name must equal the file name -> 422', code(() => saveScene(root, 'other', null, s)) === 422);
  chk('bad file name -> 400', code(() => saveScene(root, '../x', null, s)) === 400 && code(() => readScene(root, 'A B')) === 400);
  chk('missing scene -> 404', code(() => readScene(root, 'none')) === 404);

  const n = structuredClone(s); n.name = 'copy';
  chk('create with baseVersion null', saveScene(root, 'copy', null, n).changed && fs.existsSync(path.join(root, 'scenes/copy.json')));
  chk('create over an existing scene -> 409', code(() => saveScene(root, 'copy', null, n)) === 409);

  // Live: a PUT on the running scene rebuilds the plant behind the same server.
  fs.copyFileSync(path.join(ROOT, 'scenes/cyl-on-slide.ctl.js'), path.join(root, 'scenes/cyl-on-slide.ctl.js'));
  fs.writeFileSync(path.join(root, 'package.json'), '{"type":"module"}');
  const { serve } = await import('../server/http.js');
  const srv = await serve({ root, sceneName: 'cyl-on-slide', port: 0, internal: true, log: () => {} });
  try {
    const base = 'http://127.0.0.1:' + srv.server.address().port;
    const got = await (await fetch(base + '/api/scene/cyl-on-slide')).json();
    chk('GET /api/scene returns the file version', got.version === versionOf(fs.readFileSync(file)));
    const e = structuredClone(got.scene); e.components.find(c => c.id === 'part1').at[0] += 5;
    const put = (bv, sc, headers = {}) => fetch(base + '/api/scene/cyl-on-slide',
      { method: 'PUT', headers: { 'content-type': 'application/json', ...headers }, body: JSON.stringify({ baseVersion: bv, scene: sc }) });
    const r1 = await put(got.version, e), j1 = await r1.json();
    chk('PUT on the running scene saves and rebuilds the plant', r1.status === 200 && j1.changed && j1.rebuilt === true, JSON.stringify(j1));
    chk('PUT with a stale version -> 409', (await put(got.version, e)).status === 409);
    chk('PUT from another site -> 403', (await put(j1.version, e, { origin: 'http://evil.example' })).status === 403);
    const ping = async () => (await (await fetch(base + '/api/ping')).json()).t;
    const t0 = await ping();
    await new Promise(r => setTimeout(r, 200));
    const t1 = await ping();
    chk('the rebuilt plant runs', t1 > t0, t0 + ' -> ' + t1 + ' ms');
  } finally {
    await srv.close();
  }
} finally {
  fs.rmSync(root, { recursive: true, force: true });
}
process.exit(fail ? 1 : 0);
