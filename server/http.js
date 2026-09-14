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
  let scene = JSON.parse(fs.readFileSync(path.join(root, 'scenes', sceneName + '.json'), 'utf8'));
  const errs = validate(scene);
  if (errs.length) throw new Error('scenes/' + sceneName + '.json is invalid:\n  ' + errs.join('\n  '));

  /**
   * One scene's driver, recorder and plant. The driver hooks reach THIS plant only, so a driver
   * that is still closing never feeds the next one.
   * @param {any} sc
   */
  async function build(sc) {
    if (sc.io?.mode === 'twin') throw new Error('twin mode arrives in P7 (docs/PLAN.md §8)');
    const plc = !internal && sc.io?.driver === 'opcua';
    let controller = null;
    const ctlFile = path.join(root, 'scenes', sceneName + '.ctl.js');
    if (!plc && fs.existsSync(ctlFile)) controller = (await import(pathToFileURL(ctlFile).href)).create(sc);
    const t = [...sceneTags(sc)];
    /** @type {any} */
    let p = null;
    const drv = plc ? createDriver({
      endpoint: sc.io.endpoint, prefix: sc.io.prefix,
      outs: t.filter(([, i]) => i.dir === 'out').map(([n]) => n), ins: t.filter(([, i]) => i.dir === 'in').map(([n]) => n),
    }, {
      onOut: (tag, v, src) => p?.fromPlc(tag, v, src),
      onIn: (tag, v, pub) => p?.plcSaw(tag, v, pub),
      onUp: () => p?.driverUp(),
      warn: msg => p?.warn(msg),
    }) : null;
    const rec = createRecorder(path.join(root, 'runs'), sc, plc ? 'opcua' : 'internal');
    p = await createPlant(sc, { driver: drv, controller, recorder: rec });
    p.warnListeners.push((/** @type {string} */ msg, /** @type {number} */ tt) => { log('warn  ' + msg); broadcast('warn', { t: tt, msg }); });
    return { plant: p, driver: drv, recorder: rec, usePlc: plc };
  }
  /** @type {any} */
  let plant, driver, recorder, usePlc;
  ({ plant, driver, recorder, usePlc } = await build(scene));

  // ------------------------------------------------------------ SSE
  /** @type {Set<http.ServerResponse>} */
  const clients = new Set();
  /** Serialised ONCE per broadcast, then written to every viewer. @param {string} ev @param {any} data */
  const broadcast = (ev, data) => { const s = 'event: ' + ev + '\ndata: ' + JSON.stringify(data) + '\n\n'; for (const c of clients) c.write(s); };
  const sendTo = (/** @type {http.ServerResponse} */ c, /** @type {string} */ ev, /** @type {any} */ data) => c.write('event: ' + ev + '\ndata: ' + JSON.stringify(data) + '\n\n');
  const sceneMsg = () => ({ v: 1, scene });
  // Free parts stream as transforms (mm, then quaternion): they are Rapier-dynamic, so no DOF
  // describes them. Machine links still stream DOF values only.
  const partPose = (/** @type {number[]} */ a) => a.map((v, i) => (i < 3 ? Math.round(v * 100) / 100 : Math.round(v * 1e5) / 1e5));
  const full = () => {
    const s = plant.snapshot();
    const parts = Object.fromEntries(Object.entries(s.parts).map(([k, v]) => [k, partPose(v)]));
    return { t: s.t, full: true, dof: Object.fromEntries(Object.entries(s.dof).map(([k, v]) => [k, round2(v)])), io: s.io, forced: s.forced, parts, ptpl: s.ptpl };
  };
  /** @type {Record<string, any>} */
  let lastDof = {}, lastIo = {}, lastForced = '', lastFull = 0, lastParts = {};
  const frame = setInterval(() => {
    if (!clients.size) return;
    const now = Date.now();
    if (now - lastFull >= FULL_MS) {
      lastFull = now;
      const f = full();
      lastDof = { ...f.dof }; lastIo = { ...f.io }; lastForced = JSON.stringify(f.forced);
      lastParts = Object.fromEntries(Object.entries(f.parts).map(([k, v]) => [k, v.join()]));
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
    for (const [uid, a] of Object.entries(s.parts)) {
      const r = partPose(a), key = r.join();
      if (lastParts[uid] === key) continue;
      if (lastParts[uid] === undefined) (m.ptpl ??= {})[uid] = s.ptpl[uid];
      (m.parts ??= {})[uid] = r;
      lastParts[uid] = key;
    }
    for (const uid of Object.keys(lastParts)) if (!(uid in s.parts)) { (m.pgone ??= []).push(uid); delete lastParts[uid]; }
    broadcast('state', m);
  }, FRAME_MS);
  const status = setInterval(() => clients.size && broadcast('status', plant.status()), STATUS_MS);

  // ------------------------------------------------------------ rebuild on save (docs/PLAN.md §10)
  /** @type {Promise<any>} */
  let rebuilding = Promise.resolve();
  /**
   * A saved scene replaces the running one: same URL and viewers, a new plant. The new plant is
   * built BEFORE the old one stops, so a scene that fails to build leaves the old one running.
   * The old driver closes before the new one connects: one session to the PLC at a time.
   * @param {any} sc
   */
  function rebuild(sc, name = sceneName, wantInternal = internal) {
    rebuilding = rebuilding.catch(() => {}).then(async () => {
      const wasName = sceneName, wasInternal = internal;
      sceneName = name;                                  // build() reads both for the .ctl.js and the driver
      internal = wantInternal;
      let next;
      try {
        next = await build(sc);
      } catch (e) {
        sceneName = wasName;                             // a scene that cannot be built leaves the old one running
        internal = wasInternal;
        throw e;
      }
      const wasRunning = plant.mode === 'run';
      const old = plant, oldRec = recorder;
      old.stop();
      ({ plant, driver, recorder, usePlc } = next);
      scene = sc;
      lastFull = 0;
      broadcast('scene', sceneMsg());
      await old.close();
      await oldRec.close();
      await driver?.start();
      if (wasRunning) plant.start();
      broadcast('status', plant.status());
      log('rebuilt from scenes/' + sceneName + '.json   recording ' + path.relative(root, recorder.file));
    });
    return rebuilding;
  }

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
      if (req.method === 'GET' && url.pathname === '/api/ping') return json(res, 200, { ok: true, t: plant.t, scene: scene.name, internal: !usePlc });
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
        const name = decodeURIComponent(sm[1]);
        const r = saveScene(root, name, b.baseVersion ?? null, b.scene);
        if (name !== sceneName || !r.changed) return json(res, 200, { ok: true, ...r, rebuilt: false });
        try { await rebuild(readScene(root, name).scene); } catch (e) {
          return json(res, 200, { ok: true, ...r, rebuilt: false, rebuildError: String(/** @type {any} */ (e).message || e) });
        }
        return json(res, 200, { ok: true, ...r, rebuilt: true });
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
        // Load another scene, or run this one against the PLC instead of its own controller.
        if (url.pathname === '/api/switch') {
          const name = String(b.scene ?? sceneName);
          if (!NAME_RE.test(name)) return json(res, 400, { error: 'scene name must match ' + NAME_RE });
          const f = path.join(root, 'scenes', name + '.json');
          if (!fs.existsSync(f)) return json(res, 404, { error: 'no scene ' + name });
          const sc = JSON.parse(fs.readFileSync(f, 'utf8'));
          const errs = validate(sc);
          if (errs.length) return json(res, 422, { error: 'scenes/' + name + '.json is invalid', errors: errs });
          try { await rebuild(sc, name, b.internal == null ? internal : !!b.internal); } catch (e) {
            return json(res, 400, { error: String(/** @type {any} */ (e).message || e) });
          }
          return json(res, 200, { ok: true, scene: sceneName, internal: !usePlc });
        }
        if (url.pathname === '/api/press') { plant.press(String(b.id), String(b.key), !!b.down); return json(res, 200, { ok: true }); }
        // The hand: hold a loose part still to jam the line on purpose (docs/PLAN.md §3).
        if (url.pathname === '/api/hold') { plant.holdPart(String(b.uid), !!b.down); return json(res, 200, { ok: true }); }
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
      await rebuilding.catch(() => {});
      clearInterval(frame); clearInterval(status);
      for (const c of clients) c.end();
      await new Promise(r => server.close(() => r(null)));
      await plant.close();
      await recorder.close();
    },
  };
}
