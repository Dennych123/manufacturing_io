// @ts-check
// A scene whose controller is a CX-Programmer project, run by the ladder soft-PLC in the plant's
// own step. The scene says which .cxp and which PLC address each of its tags is:
//
//   "io": { "driver": "ladder", "ladder": { "cxp": "plc/final-caulking.cxp",
//                                           "map": { "SOL_SHUTTER1_UP": "3205.07", ... } } }
//
// Per plant step: the plant's `in` tags are written to their addresses, ONE scan runs, and the
// `out` tags are read back - the same step, so a sensor edge is seen by the very next scan and a
// coil the scan sets moves its actuator in the same 2 ms. Nothing crosses a network.
//
// Things that are neither the PLC nor a scene component - a double-solenoid valve's memory, a
// magnet controller's start/stop latch, the machine next door that this PLC only talks to over
// interlock wires - are `wiring`: plain functions the scene's .ctl.js passes in, run around the
// scan with the same io object.
import fs from 'node:fs';
import path from 'node:path';
import { readCxp, writeCxp } from './cxp.js';
import { createLadder } from './ladder.js';
import { createTwin } from './twin.js';
import { tags as sceneTags } from '../lib/scene.js';
import { parseAddr } from '../lib/ladder.js';

/**
 * @typedef {{before?: (io: Record<string, any>, t: number, plc: any) => void,
 *            after?: (io: Record<string, any>, t: number, plc: any) => void,
 *            reset?: () => void}} Wiring
 */

/**
 * @param {any} scene
 * @param {{root: string, wiring?: Wiring, log?: (s: string) => void}} opt
 */
export function createLadderController(scene, { root, wiring = {}, log = console.log }) {
  const cfg = scene.io?.ladder;
  if (!cfg?.cxp) throw new Error('scene ' + scene.name + ' has no io.ladder.cxp');
  let file = path.resolve(root, cfg.cxp);
  const plcDir = path.resolve(root, 'plc');
  let loaded = load(file);
  /** tag -> address, split by direction once. */
  const info = sceneTags(scene);
  /** @type {Array<[string, string, string]>} */
  const ins = [], outs = [];
  for (const [tag, addr] of Object.entries(cfg.map || {})) {
    const i = info.get(tag);
    if (!parseAddr(String(addr))) throw new Error('io.ladder.map.' + tag + ': "' + addr + '" is not a PLC address');
    if (!i) continue;                                   // wiring-only tags are the .ctl.js's business
    (i.dir === 'in' ? ins : outs).push([tag, String(addr), i.type]);
  }

  /**
   * Memory the program reads and never writes, which the real CPU holds from before (retained H
   * and D, set once by hand or by an earlier program): `io.ladder.init`, written at power-up.
   * @param {any} plc
   */
  function initMemory(plc) { for (const [a, v] of Object.entries(cfg.init || {})) plc.set(a, Number(v)); }

  /** @param {string} f */
  function load(f) {
    const { tree, project } = readCxp(fs.readFileSync(f));
    const plc = createLadder(project, { binaryTimers: !!cfg.binaryTimers });
    for (const e of plc.errors) log('ladder: ' + project.programs[e.prog]?.name + ' rung ' + e.rung + ': ' + e.msg);
    const twin = createTwin(project, plc, cfg.map || {}), patched = patch(project, plc);
    // After the patches, so it is the code that will run that gets compiled (plc.warm says why).
    plc.warm(cfg.warmScans ?? 7000, Object.values(cfg.map || {}).map(String));
    initMemory(plc);
    return { tree, project, plc, twin, patched };
  }

  /**
   * `io.ladder.patches`: rungs the simulation has to run differently from the file, each with the
   * reason, applied in memory only. A patch names the rung it replaces AND what that rung says now,
   * and is skipped (loudly) when the program no longer says that - a patch written for one version
   * of a program must not rewrite another. The patched rung carries "[SIM PATCH]" in its comment, so
   * the ladder page shows it, and Save / Download write the ORIGINAL rung back, never the patch.
   * @param {any} project @param {any} plc
   */
  function patch(project, plc) {
    /** @type {Array<{s: any, rung: any, orig: any}>} */
    const done = [];
    const norm = (/** @type {string[]} */ il) => il.filter(l => !l.trim().startsWith('//')).map(l => l.trim().split(/\s+/).join(' '));
    for (const pa of cfg.patches || []) {
      const pi = project.programs.findIndex((/** @type {any} */ p) => p.name === pa.prog);
      const si = pi < 0 ? -1 : project.programs[pi].sections.findIndex((/** @type {any} */ s) => s.name === pa.sec);
      const s = project.programs[pi]?.sections[si], orig = s?.rungs[pa.rung];
      const where = pa.prog + ' / ' + pa.sec + ' R' + pa.rung;
      if (!orig || JSON.stringify(norm(orig.il)) !== JSON.stringify(norm(pa.expect))) { log('ladder: sim patch NOT applied, the rung is not what it was written for: ' + where); continue; }
      const r = plc.edit({ prog: pi, sec: si, at: pa.rung, del: 1, rungs: [{ il: pa.il, comment: '[SIM PATCH - not in the .cxp] ' + pa.why + (orig.comment ? '\n' + orig.comment : '') }] });
      if (!r.ok) { log('ladder: sim patch ' + where + ' does not compile: ' + r.errors.join('; ')); continue; }
      done.push({ s, rung: s.rungs[pa.rung], orig });
      log('ladder: sim patch applied to ' + where + ': ' + pa.why);
    }
    return done;
  }

  // The machine's touch panel (Keyence VT STUDIO, `io.ladder.hmi`): its screens, and the switches
  // held down on it right now. A held switch is written after the scene's own inputs every scan,
  // so a panel bit the scene ALSO drives (a pushbutton mapped to W450.08) is held, not overwritten.
  let hmiFile = cfg.hmi ? path.resolve(root, cfg.hmi) : '';
  /** @type {any} */
  let hmiCache = null;
  /** @type {Set<string>} */
  const hmiHeld = new Set();

  const ctl = {
    /** @param {Record<string, any>} io @param {number} t */
    scan(io, t) {
      const plc = loaded.plc;
      wiring.before?.(io, t, plc);
      for (const [tag, addr] of ins) plc.set(addr, typeof io[tag] === 'boolean' ? +io[tag] : Number(io[tag]) || 0);
      for (const a of hmiHeld) plc.set(a, 1);
      plc.scan(t);
      // PROGRAM mode: the CPU drives every output OFF.
      for (const [tag, addr, type] of outs) io[tag] = !plc.running ? (type === 'BOOL' ? false : 0) : type === 'BOOL' ? !!plc.bit(addr) : plc.word(addr);
      wiring.after?.(io, t, plc);
      loaded.twin.sample(t);
    },
    reset() { loaded.plc.reset(true); initMemory(loaded.plc); loaded.twin.reset(); wiring.reset?.(); },
    /** What the viewer's status line shows: which program runs, and whether it is scanning. */
    status() {
      const p = loaded.plc, st = p.stats;
      return { driver: 'ladder', ok: p.running && !p.errors.length,
               msg: 'LADDER SOFT-PLC ' + ctl.ladder.file + (p.running ? '' : ' - PROGRAM mode') + (p.errors.length ? ' - ' + p.errors.length + ' rung(s) not compiled' : '')
                 + (loaded.patched.length ? ' - ' + loaded.patched.length + ' sim patch(es)' : '') + '  scan ' + st.scanUs + ' us',
               heartbeat: st.scans };
    },
    /** What the ladder page and its API use. */
    ladder: {
      get plc() { return loaded.plc; },
      get project() { return loaded.project; },
      /** Cycle time per unit and step, and why a condition is off (server/twin.js). */
      get twin() { return loaded.twin; },
      get file() { return path.relative(root, file).split(path.sep).join('/'); },
      map: cfg.map || {},
      /** The .cxp bytes of the project as it is now: edits included, simulation patches NOT. */
      bytes() {
        const sw = loaded.patched.map(p => { const i = p.s.rungs.indexOf(p.rung); if (i >= 0) p.s.rungs[i] = p.orig; return i; });
        try { return writeCxp(loaded.tree, loaded.project); }
        finally { loaded.patched.forEach((p, k) => { if (sw[k] >= 0) p.s.rungs[sw[k]] = p.rung; }); }
      },
      /** The rungs the simulation runs differently from the file. */
      get patches() { return (cfg.patches || []).map((/** @type {any} */ p) => ({ prog: p.prog, sec: p.sec, rung: p.rung, why: p.why })); },
      /** Writes the edited project over its file, keeping the previous one as .bak. */
      save() {
        const bak = file.replace(/\.cxp$/i, '') + '.' + new Date().toISOString().replace(/[-:]/g, '').replace(/\..*/, '') + '.cxp.bak';
        const bytes = ctl.ladder.bytes();
        // Refuse to write what cannot be read back.
        const back = readCxp(bytes).project;
        if (back.programs.length !== loaded.project.programs.length) throw new Error('the written project does not read back');
        fs.copyFileSync(file, bak);
        fs.writeFileSync(file + '.tmp', bytes);
        fs.renameSync(file + '.tmp', file);
        return { file: ctl.ladder.file, backup: path.relative(root, bak).split(path.sep).join('/') };
      },
      /** The .cxp files a scene may switch to: plc/*.cxp. */
      files() { return fs.existsSync(plcDir) ? fs.readdirSync(plcDir).filter(f => /\.cxp$/i.test(f)).map(f => 'plc/' + f) : []; },
      /** The touch panel's screens (server/vs4.js), read once per file. */
      async hmi() {
        const rel = hmiFile ? path.relative(root, hmiFile).split(path.sep).join('/') : '';
        const files = fs.existsSync(plcDir) ? fs.readdirSync(plcDir).filter(f => /\.vs4$/i.test(f)).map(f => 'plc/' + f) : [];
        if (!hmiFile || !fs.existsSync(hmiFile)) return { file: rel, files, screens: [], warnings: [hmiFile ? rel + ' is not on disk (company file, not in git)' : 'this scene names no io.ladder.hmi'] };
        if (!hmiCache || hmiCache.file !== hmiFile) {
          const { readVs4 } = await import('./vs4.js');
          hmiCache = { file: hmiFile, ...readVs4(fs.readFileSync(hmiFile)) };
        }
        return { file: rel, files, screens: hmiCache.screens, warnings: hmiCache.warnings };
      },
      /** @param {string} rel */
      hmiOpen(rel) {
        const f = path.resolve(root, rel);
        if (path.dirname(f) !== plcDir || !/\.vs4$/i.test(f) || !fs.existsSync(f)) throw new Error('only plc/*.vs4 can be opened');
        hmiFile = f; hmiCache = null; hmiHeld.clear();
      },
      /** A panel switch pressed (held until released) or released. @param {string} addr @param {boolean} down */
      hmiPress(addr, down) {
        const a = String(addr);
        if (!parseAddr(a)) throw new Error('not a PLC address: ' + a);
        if (down) hmiHeld.add(a); else { hmiHeld.delete(a); loaded.plc.set(a, 0); }
      },
      /** Load another project from plc/. Forces are dropped with the old program. @param {string} rel */
      open(rel) {
        const f = path.resolve(root, rel);
        if (path.dirname(f) !== plcDir || !/\.cxp$/i.test(f) || !fs.existsSync(f)) throw new Error('only plc/*.cxp can be opened');
        loaded = load(f);
        file = f;
        wiring.reset?.();
      },
    },
  };
  return ctl;
}

/**
 * The /api/ladder endpoints. Returns true when it answered.
 *   GET  /api/ladder                  the project: programs, sections, rungs, symbols, compile errors
 *   GET  /api/ladder/peek?k=b:W100.01,w:D5046,l:D5131   live values, plus forced list and scan stats
 *   GET  /api/ladder/download         the project as a .cxp (edits included)
 *   GET  /api/ladder/ct               cycle time per unit (busy, period, utilisation) and per step
 *   GET  /api/ladder/diag             alarms, master / start / home conditions, with why each is off
 *   GET  /api/ladder/why?a=W482.05    the rungs that drive a bit and the contacts that stop them
 *   GET  /api/ladder/hmi              the touch panel's screens (io.ladder.hmi, a VT STUDIO .vs4)
 *   POST /api/ladder/hmipress {addr, down}   /api/ladder/hmiopen {file}   /api/ladder/ctreset {}
 *   POST /api/ladder/force {addr, value|null}   /api/ladder/set {addr, value}   /api/ladder/release {}
 *   POST /api/ladder/edit {prog, sec, at, del, rungs: [{il: [...], comment}]}
 *   POST /api/ladder/mode {run}   /api/ladder/save {}   /api/ladder/open {file}
 * @param {import('node:http').IncomingMessage} req @param {import('node:http').ServerResponse} res
 * @param {URL} url @param {any} lad the controller's `ladder`, or null
 * @param {{json: (res: any, code: number, data: any) => void, body: () => Promise<any>, t: () => number}} h
 */
export async function ladderApi(req, res, url, lad, h) {
  if (!url.pathname.startsWith('/api/ladder')) return false;
  if (!lad) { h.json(res, 404, { error: 'this scene has no ladder program (io.driver "ladder")' }); return true; }
  const plc = lad.plc, sub = url.pathname.slice('/api/ladder'.length);
  if (req.method === 'GET') {
    if (sub === '') {
      const p = lad.project;
      h.json(res, 200, { file: lad.file, files: lad.files(), name: p.name, device: p.device, cpu: p.cpu, globals: p.globals,
        programs: p.programs.map((/** @type {any} */ x) => ({ name: x.name, comment: x.comment, taskId: x.taskId, locals: x.locals,
          sections: x.sections.map((/** @type {any} */ s) => ({ name: s.name, rungs: s.rungs.map((/** @type {any} */ r) => ({ comment: r.comment, il: r.il })) })) })),
        errors: plc.errors, map: lad.map, running: plc.running, patches: lad.patches });
      return true;
    }
    if (sub === '/peek') {
      const keys = (url.searchParams.get('k') || '').split(',').filter(Boolean).slice(0, 4000);
      const v = keys.map(k => {
        try {
          const a = k.slice(2);
          return k[0] === 'w' ? plc.word(a) : k[0] === 'l' ? plc.long(a) : plc.bit(a);
        } catch { return null; }
      });
      h.json(res, 200, { t: h.t(), v, forced: plc.forced(), stats: plc.stats });
      return true;
    }
    if (sub === '/hmi') { h.json(res, 200, await lad.hmi()); return true; }
    if (sub === '/ct') { h.json(res, 200, lad.twin.ct()); return true; }
    if (sub === '/diag') { h.json(res, 200, lad.twin.diag()); return true; }
    if (sub === '/steps') { h.json(res, 200, lad.twin.steps()); return true; }
    if (sub === '/why') { h.json(res, 200, { addr: url.searchParams.get('a'), rungs: lad.twin.why(String(url.searchParams.get('a') || '')) }); return true; }
    if (sub === '/download') {
      const bytes = lad.bytes();
      res.writeHead(200, { 'Content-Type': 'application/octet-stream', 'Content-Disposition': 'attachment; filename="' + path.basename(lad.file) + '"', 'Cache-Control': 'no-store' });
      res.end(bytes);
      return true;
    }
    h.json(res, 404, { error: 'no such endpoint' });
    return true;
  }
  if (req.method !== 'POST') { h.json(res, 405, { error: 'method not allowed' }); return true; }
  const b = await h.body();
  switch (sub) {
    case '/force': plc.force(String(b.addr), b.value == null ? null : Number(b.value)); break;
    case '/set': plc.set(String(b.addr), Number(b.value)); break;
    case '/release': plc.releaseAll(); break;
    case '/ctreset': lad.twin.reset(); break;
    case '/hmipress': lad.hmiPress(String(b.addr), !!b.down); break;
    case '/hmiopen': lad.hmiOpen(String(b.file)); break;
    case '/mode': plc.setRunning(!!b.run); break;
    case '/edit': {
      const r = plc.edit({ prog: +b.prog, sec: +b.sec, at: +b.at, del: +(b.del || 0), rungs: Array.isArray(b.rungs) ? b.rungs : [] });
      h.json(res, r.ok ? 200 : 422, r);
      return true;
    }
    case '/save': h.json(res, 200, { ok: true, ...lad.save() }); return true;
    case '/open': lad.open(String(b.file)); break;
    default: h.json(res, 404, { error: 'no such endpoint' }); return true;
  }
  h.json(res, 200, { ok: true, forced: lad.plc.forced(), running: lad.plc.running });
  return true;
}
