// @ts-check
// HTTP + SSE in front of one plant. The browser renders, edits and analyses; it never runs the
// plant. Transport is SSE + POST with JSON (docs/PLAN.md §6).
//
//   GET  /                     web/index.html
//   GET  /web/* /lib/* /vendor/three/*   static, fixed prefixes only
//   GET  /api/stream           SSE: scene, state (30 Hz deltas, full every 5 s), status (1 Hz), warn
//   GET  /api/ping /api/scenes /api/tags
//   POST /api/cmd {op: run|stop|reset}   /api/press {id, key, down}   /api/force {tag, value|null}
//   GET  /api/scene/:name -> {version, scene}   PUT /api/scene/:name {baseVersion, scene} (409 when stale)
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import { validate, bindings, tags as sceneTags, NAME_RE } from '../lib/scene.js';
import { createPlant, createRecorder } from './plant.js';
import { createDriver } from './opcua.js';
import { readScene, saveScene, SceneError } from './scenes.js';

const MIME = /** @type {Record<string, string>} */ ({
  '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.json': 'application/json; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.png': 'image/png', '.map': 'application/json',
});
const STATIC = /** @type {Record<string, string>} */ ({ '/web/': 'web', '/lib/': 'lib', '/vendor/three/': 'node_modules/three' });
const FRAME_MS = 33, FULL_MS = 5000, STATUS_MS = 1000;

/**
 * URL path -> file, or null. Only the fixed prefixes, never `..`, `.`, empty segments or
 * backslashes, so no request can climb out of them.
 * @param {string} root @param {string} urlPath
 */
export function staticPath(root, urlPath) {
  let p;
  try { p = decodeURIComponent(urlPath); } catch { return null; }
  if (p === '/') p = '/web/index.html';
  if (p.includes('\0') || p.includes('\\')) return null;
  const pre = Object.keys(STATIC).find(k => p.startsWith(k));
  if (!pre) return null;
  const segs = p.slice(pre.length).split('/');
  if (segs.some(s => s === '' || s === '.' || s === '..')) return null;
  return path.join(root, STATIC[pre], ...segs);
}

/**
 * POST (anything that changes the plant) comes from this PC only, unless --lan-control.
 * A browser Origin must match the Host it talks to (rb4axis's check), so a page from
 * another site cannot drive the machine through the viewer's browser.
 * @param {{socket: {remoteAddress?: string}, headers: Record<string, any>}} req @param {{lanControl?: boolean}} opt
 */
export function postAllowed(req, { lanControl = false } = {}) {
  const ra = req.socket.remoteAddress || '';
  const local = ra === '127.0.0.1' || ra === '::1' || ra === '::ffff:127.0.0.1';
  if (!local && !lanControl) return false;
  const o = req.headers.origin;
  if (o) { try { if (new URL(o).host !== req.headers.host) return false; } catch { return false; } }
  return true;
}

/** @param {http.IncomingMessage} req @param {number} [max] @returns {Promise<any>} */
function body(req, max = 65536) {
  return new Promise((res, rej) => {
    let s = '';
    req.setEncoding('utf8');
    req.on('data', d => { s += d; if (s.length > max) { rej(new Error('body too large')); req.destroy(); } });
    req.on('end', () => { try { res(s ? JSON.parse(s) : {}); } catch { rej(new Error('body is not JSON')); } });
    req.on('error', rej);
  });
}

const round2 = (/** @type {any} */ v) => (typeof v === 'number' ? Math.round(v * 100) / 100 : v);

/**
 * Loads scenes/<name>.json, builds the plant, connects the PLC (or the internal controller)
 * and serves it.
 * @param {{root: string, sceneName?: string, port?: number, internal?: boolean, lan?: boolean, lanControl?: boolean,
 *          log?: (s: string) => void}} opt
 */
export async function serve({ root, sceneName = 'cyl-on-slide', port = 7660, internal = false, lan = false, lanControl = false, log = console.log }) {
  if (!NAME_RE.test(sceneName)) throw new Error('scene name must match ' + NAME_RE);
  const scene = JSON.parse(fs.readFileSync(path.join(root, 'scenes', sceneName + '.json'), 'utf8'));
  const errs = validate(scene);
  if (errs.length) throw new Error('scenes/' + sceneName + '.json is invalid:\n  ' + errs.join('\n  '));
  if (scene.io?.mode === 'twin') throw new Error('twin mode arrives in P7 (docs/PLAN.md §8)');

  const usePlc = !internal && scene.io?.driver === 'opcua';
  let controller = null;
  const ctlFile = path.join(root, 'scenes', sceneName + '.ctl.js');
  if (!usePlc && fs.existsSync(ctlFile)) controller = (await import(pathToFileURL(ctlFile).href)).create(scene);
  const t = [...sceneTags(scene)];
  /** @type {any} */
  let plant = null;
  const driver = usePlc ? createDriver({
    endpoint: scene.io.endpoint, prefix: scene.io.prefix,
    outs: t.filter(([, i]) => i.dir === 'out').map(([n]) => n), ins: t.filter(([, i]) => i.dir === 'in').map(([n]) => n),
  }, {
    onOut: (tag, v, src) => plant?.fromPlc(tag, v, src),
    onIn: (tag, v, pub) => plant?.plcSaw(tag, v, pub),
    onUp: () => plant?.driverUp(),
    warn: msg => plant?.warn(msg),
  }) : null;
  const recorder = createRecorder(path.join(root, 'runs'), scene, usePlc ? 'opcua' : 'internal');
  plant = await createPlant(scene, { driver, controller, recorder });

  // ------------------------------------------------------------ SSE
  /** @type {Set<http.ServerResponse>} */
  const clients = new Set();
  /** Serialised ONCE per broadcast, then written to every viewer. @param {string} ev @param {any} data */
  const broadcast = (ev, data) => { const s = 'event: ' + ev + '\ndata: ' + JSON.stringify(data) + '\n\n'; for (const c of clients) c.write(s); };
  const sendTo = (/** @type {http.ServerResponse} */ c, /** @type {string} */ ev, /** @type {any} */ data) => c.write('event: ' + ev + '\ndata: ' + JSON.stringify(data) + '\n\n');
  const sceneMsg = () => ({ v: 1, scene });
  const full = () => {
    const s = plant.snapshot();
    return { t: s.t, full: true, dof: Object.fromEntries(Object.entries(s.dof).map(([k, v]) => [k, round2(v)])), io: s.io, forced: s.forced, parts: s.parts };
  };
  /** @type {Record<string, any>} */
  let lastDof = {}, lastIo = {}, lastForced = '', lastFull = 0;
  const frame = setInterval(() => {
    if (!clients.size) return;
    const now = Date.now();
    if (now - lastFull >= FULL_MS) {
      lastFull = now;
      const f = full();
      lastDof = { ...f.dof }; lastIo = { ...f.io }; lastForced = JSON.stringify(f.forced);
      broadcast('state', f);
      return;
    }
    // Every frame carries t, even with nothing changed: the viewer interpolates on sim time.
    const s = plant.snapshot();
    /** @type {any} */
    const m = { t: s.t, dof: {}, io: {} };
    for (const [k, v] of Object.entries(s.dof)) { const r = round2(v); if (lastDof[k] !== r) m.dof[k] = lastDof[k] = r; }
    for (const [k, v] of Object.entries(s.io)) if (lastIo[k] !== v) m.io[k] = lastIo[k] = v;
    const f = JSON.stringify(s.forced);
    if (f !== lastForced) { m.forced = s.forced; lastForced = f; }
    broadcast('state', m);
  }, FRAME_MS);
  const status = setInterval(() => clients.size && broadcast('status', plant.status()), STATUS_MS);
  plant.warnListeners.push((/** @type {string} */ msg, /** @type {number} */ tt) => { log('warn  ' + msg); broadcast('warn', { t: tt, msg }); });

  // ------------------------------------------------------------ routes
  /** @param {http.ServerResponse} res @param {number} code @param {any} data */
  const json = (res, code, data) => { res.writeHead(code, { 'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store' }); res.end(JSON.stringify(data)); };

  const server = http.createServer(async (req, res) => {
    const url = new URL(req.url || '/', 'http://x');
    try {
      if (req.method === 'GET' && url.pathname === '/api/stream') {
        res.writeHead(200, { 'Content-Type': 'text/event-stream', 'Cache-Control': 'no-cache', Connection: 'keep-alive' });
        res.write('retry: 2000\n\n');
        sendTo(res, 'scene', sceneMsg());
        sendTo(res, 'state', full());
        sendTo(res, 'status', plant.status());
        clients.add(res);
        req.on('close', () => clients.delete(res));
        return;
      }
      if (req.method === 'GET' && url.pathname === '/api/ping') return json(res, 200, { ok: true, t: plant.t, scene: scene.name });
      if (req.method === 'GET' && url.pathname === '/api/scenes') {
        return json(res, 200, fs.readdirSync(path.join(root, 'scenes')).filter(f => f.endsWith('.json')).map(f => f.slice(0, -5)).filter(n => NAME_RE.test(n)));
      }
      // ponytail: the scene's own tags. The PLC browse cache for editor autocomplete arrives with P2.
      if (req.method === 'GET' && url.pathname === '/api/tags') return json(res, 200, bindings(scene).map(b => ({ tag: b.tag, dir: b.dir, type: b.type, comp: b.comp, key: b.key })));
      const sm = /^\/api\/scene\/([^/]+)$/.exec(url.pathname);
      if (sm && req.method === 'GET') return json(res, 200, readScene(root, decodeURIComponent(sm[1])));
      if (sm && req.method === 'PUT') {
        if (!postAllowed(req, { lanControl })) return json(res, 403, { error: 'saving only from this PC (start with --lan-control to allow the LAN)' });
        const b = await body(req, 4 << 20);
        const r = saveScene(root, decodeURIComponent(sm[1]), b.baseVersion ?? null, b.scene);
        // ponytail: the running plant keeps the scene it started with; rebuilding it on save comes next.
        return json(res, 200, { ok: true, ...r, active: sm[1] === sceneName });
      }
      if (req.method === 'POST' && url.pathname.startsWith('/api/')) {
        if (!postAllowed(req, { lanControl })) return json(res, 403, { error: 'POST only from this PC (start with --lan-control to allow the LAN)' });
        const b = await body(req);
        if (url.pathname === '/api/cmd') {
          if (b.op === 'run') plant.start(); else if (b.op === 'stop') plant.stop(); else if (b.op === 'reset') plant.reset();
          else return json(res, 400, { error: 'op must be run, stop or reset' });
          broadcast('status', plant.status());
          return json(res, 200, { ok: true });
        }
        if (url.pathname === '/api/press') { plant.press(String(b.id), String(b.key), !!b.down); return json(res, 200, { ok: true }); }
        if (url.pathname === '/api/force') { plant.force(String(b.tag), b.value ?? null); return json(res, 200, { ok: true }); }
        return json(res, 404, { error: 'no such endpoint' });
      }
      if (req.method !== 'GET' && req.method !== 'HEAD') return json(res, 405, { error: 'method not allowed' });
      const file = staticPath(root, url.pathname);
      if (!file) return json(res, 404, { error: 'not found' });
      fs.readFile(file, (err, data) => {
        if (err) { json(res, 404, { error: 'not found' }); return; }
        res.writeHead(200, { 'Content-Type': MIME[path.extname(file)] || 'application/octet-stream', 'Cache-Control': 'no-cache' });
        res.end(req.method === 'HEAD' ? undefined : data);
      });
    } catch (e) {
      if (e instanceof SceneError) return json(res, e.code, { error: e.message, ...e.extra });
      json(res, 400, { error: String(/** @type {any} */ (e).message || e) });
    }
  });

  // Viewers on the LAN may watch (GET); POST stays local unless --lan-control.
  const host = lan || lanControl ? '0.0.0.0' : '127.0.0.1';
  await new Promise((res, rej) => { server.once('error', rej); server.listen(port, host, () => res(null)); });
  // Connect BEFORE the plant's timer runs: connect() has synchronous setup (client, certificate
  // manager) that stalled the plant ~100 ms at t = 8 ms (measured). A failed first attempt
  // returns quickly and retries on its own.
  await driver?.start();
  plant.start();
  log('manufacturing_io  http://127.0.0.1:' + port + '/   scene ' + scene.name + '   ' + (usePlc ? 'PLC ' + scene.io.endpoint : 'INTERNAL CONTROLLER - not a PLC'));
  log('recording ' + path.relative(root, recorder.file));

  return {
    server, plant, recorder,
    async close() {
      clearInterval(frame); clearInterval(status);
      for (const c of clients) c.end();
      await new Promise(r => server.close(() => r(null)));
      await plant.close();
      await recorder.close();
    },
  };
}
