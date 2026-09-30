// @ts-check
// What a machine's own program says about itself, read live out of the ladder soft-PLC: where the
// cycle time goes (per unit, per step), and why the machine will not start.
//
// It leans on the way the program is written - the Denso/Omron house style every add-on here
// follows - and finds everything by the program's OWN symbol comments, never by address:
//
//   - one program per unit, P11_.. to P17_..; each has 'Pnn AutoMotionStart' (held while the
//     unit's cycle runs), 'Pnn AutoMotion CycleEnd' and 'Pnn CycleStartCond';
//   - a unit's sequence is its Autorun section, whose coils are local bits with a comment each
//     ('MB LoadIn DOWN', 'MB LoadIn DOWN CONF'): the first scan each one comes on in a cycle is
//     when that step was reached, so the time to the next one is what that step cost;
//   - the conditions an operator is told to satisfy are bits commented 'MASTER CONDITION n',
//     'START CONDITION n-m', '... HOME POS.', and the alarms are the AL symbols.
//
// A unit's BUSY time is from its AutoMotionStart to its CycleEnd; its PERIOD is start to next
// start. The unit whose busy time is closest to its period is the one the others wait for.
import { checkRung, network, addrText, parseAddr, commentText } from '../lib/ladder.js';

const KEEP_CYCLES = 30, KEEP_TIMELINE_MS = 180_000;
/** A comment for display: Shift-JIS decoded, one line. @param {string|undefined} s */
const txt = s => commentText(s || '').replace(/\s+/g, ' ').trim();

/**
 * @param {any} project the parsed .cxp (server/cxp.js projectOf)
 * @param {any} plc the running soft-PLC (server/ladder.js)
 */
export function createTwin(project, plc, map = {}) {
  /** PLC address -> the scene tag wired to it (the operator's button, a sensor), to name a root cause. */
  const tagOf = new Map(Object.entries(map).map(([t, a]) => [normal(String(a)), t]));
  const byName = new Map(project.globals.filter((/** @type {any} */ g) => g.name).map((/** @type {any} */ g) => [g.name, g]));
  const byAddr = new Map(project.globals.filter((/** @type {any} */ g) => g.addr).map((/** @type {any} */ g) => [g.addr, g]));
  const scopes = project.programs.map((/** @type {any} */ p) => {
    const L = new Map(p.locals.filter((/** @type {any} */ g) => g.name).map((/** @type {any} */ g) => [g.name, g]));
    return { find: (/** @type {string} */ x) => L.get(x) ?? byName.get(x), locals: L };
  });
  /** A symbol's comment, wherever it is declared. @param {string} addr */
  const commentOf = addr => byAddr.get(addr)?.comment || '';
  /** Every symbol, local and global, with a comment matching `re`, as [addr, sym]. @param {RegExp} re */
  const symbolsLike = re => {
    const out = new Map();
    for (const g of project.globals) if (g.addr && re.test(g.comment || '')) out.set(normal(g.addr), g);
    for (const p of project.programs) for (const g of p.locals) if (g.addr && re.test(g.comment || '')) out.set(normal(g.addr), g);
    return out;
  };

  // ------------------------------------------------------------------ units and their steps
  /**
   * @typedef {{i: number, addr: string, name: string, comment: string}} Step
   * @typedef {{n: number, prog: string, start: string, end: string, cond: string, steps: Step[],
   *            busy: boolean, t0: number, seen: Float64Array, cycles: Array<{t0: number, t1: number, at: Array<[number, number]>}>,
   *            spans: Array<[number, number]>}} Unit
   */
  /** @type {Unit[]} */
  const units = [];
  const find = (/** @type {RegExp} */ re, /** @type {number} */ n) => {
    for (const [a, g] of symbolsLike(re)) { const m = re.exec(g.comment); if (m && +m[1] === n) return a; }
    return '';
  };
  project.programs.forEach((/** @type {any} */ p, /** @type {number} */ pi) => {
    const m = /^P(\d+)_/.exec(p.name);
    if (!m || +m[1] < 11) return;
    const n = +m[1];
    const start = find(/^\s*P(\d+)\s*AutoMotion\s*Start\s*$/i, n), end = find(/^\s*P(\d+)\s*AutoMotion\s*CycleEnd\s*$/i, n);
    const cond = find(/^\s*P(\d+)\s*Cycle\s*Start\s*Cond\s*$/i, n);
    if (!start) return;
    // The steps: every local bit an Autorun rung drives, in the order the rungs drive them.
    /** @type {Step[]} */
    const steps = [];
    const have = new Set();
    for (const s of p.sections) {
      if (!/autorun/i.test(s.name)) continue;
      for (const r of s.rungs) {
        if (!r.il.length) continue;
        let c;
        try { c = checkRung(r.il, scopes[pi]); } catch { continue; }
        c.lines.forEach((ln, k) => {
          if (ln.def.k !== 'out' || !['OUT', 'SET', 'KEEP'].includes(ln.name)) return;
          const o = c.ops[k]?.[ln.args.length - 1];
          if (!o || o.area === 'TR' || o.bit < 0) return;
          const nm = ln.args[ln.args.length - 1], sym = scopes[pi].locals.get(nm);
          if (!sym || !sym.comment) return;                    // unit bits and outputs are not steps
          const a = addrText(o);
          if (have.has(a)) return;
          have.add(a);
          steps.push({ i: steps.length, addr: a, name: nm, comment: txt(sym.comment) });
        });
      }
    }
    units.push({ n, prog: p.name, start, end, cond, steps, busy: false, t0: 0, seen: new Float64Array(steps.length).fill(-1), cycles: [], spans: [] });
  });

  // ------------------------------------------------------------------ sampling (every scan)
  let tNow = 0;
  /** @param {number} t sim ms */
  function sample(t) {
    tNow = t;
    for (const u of units) {
      const busy = !!plc.bit(u.start);
      if (busy && !u.busy) { u.t0 = t; u.seen.fill(-1); }
      if (busy) for (const s of u.steps) if (u.seen[s.i] < 0 && plc.bit(s.addr)) u.seen[s.i] = t - u.t0;
      if (!busy && u.busy) {
        /** @type {Array<[number, number]>} */
        const at = [];
        u.steps.forEach(s => { if (u.seen[s.i] >= 0) at.push([s.i, u.seen[s.i]]); });
        at.sort((a, b) => a[1] - b[1]);
        u.cycles.push({ t0: u.t0, t1: t, at });
        if (u.cycles.length > KEEP_CYCLES) u.cycles.shift();
        u.spans.push([u.t0, t]);
        while (u.spans.length && u.spans[0][1] < t - KEEP_TIMELINE_MS) u.spans.shift();
      }
      u.busy = busy;
    }
  }
  function reset() { for (const u of units) { u.busy = false; u.cycles = []; u.spans = []; u.seen.fill(-1); } }

  /** Cycle-time report: per unit busy / period / utilisation, and the last cycle's steps. */
  function ct() {
    const out = units.map(u => {
      const c = u.cycles.slice(-10);
      const busy = c.length ? c.reduce((s, x) => s + x.t1 - x.t0, 0) / c.length : 0;
      const per = c.length > 1 ? (c[c.length - 1].t0 - c[0].t0) / (c.length - 1) : 0;
      // Step durations: from each step to the next one reached, the last one to the cycle end;
      // averaged per step over the recent cycles.
      /** @type {Map<number, number[]>} */
      const dur = new Map();
      for (const cy of c) cy.at.forEach(([i, a], k) => {
        const next = k + 1 < cy.at.length ? cy.at[k + 1][1] : cy.t1 - cy.t0;
        (dur.get(i) || dur.set(i, []).get(i))?.push(next - a);
      });
      const last = u.cycles[u.cycles.length - 1];
      const steps = (last ? last.at : []).map(([i, a], k, arr) => {
        const d = dur.get(i) || [];
        return { name: u.steps[i].name, comment: u.steps[i].comment, at: Math.round(a),
          ms: Math.round((k + 1 < arr.length ? arr[k + 1][1] : last.t1 - last.t0) - a),
          avg: d.length ? Math.round(d.reduce((s, x) => s + x, 0) / d.length) : 0 };
      });
      return { unit: 'P' + u.n, prog: u.prog, cycles: u.cycles.length, busy: u.busy, busyMs: Math.round(busy), periodMs: Math.round(per),
        lastMs: last ? Math.round(last.t1 - last.t0) : 0, util: per > 0 ? Math.min(1, busy / per) : 0,
        running: u.busy ? Math.round(tNow - u.t0) : 0, steps, spans: u.spans.map(s => [Math.round(s[0]), Math.round(s[1])]) };
    });
    const ranked = out.filter(u => u.cycles >= 2).sort((a, b) => b.busyMs - a.busyMs);
    return { t: Math.round(tNow), units: out, bottleneck: ranked[0]?.unit ?? null };
  }

  // ------------------------------------------------------------------ why is it off
  /** addr -> the rungs that drive it. */
  /** @type {Map<string, Array<{pi: number, si: number, ri: number, line: number, name: string}>>} */
  const writers = new Map();
  /** @type {Map<string, {lines: any[], ops: any[], net: any}>} */
  const parsed = new Map();
  project.programs.forEach((/** @type {any} */ p, /** @type {number} */ pi) => p.sections.forEach((/** @type {any} */ s, /** @type {number} */ si) => s.rungs.forEach((/** @type {any} */ r, /** @type {number} */ ri) => {
    if (!r.il.length) return;
    let c;
    try { c = checkRung(r.il, scopes[pi]); } catch { return; }
    c.lines.forEach((ln, k) => {
      if (ln.def.k !== 'out' || !['OUT', 'SET', 'KEEP', 'OUTNOT'].includes(ln.name)) return;
      const o = c.ops[k]?.[ln.args.length - 1];
      if (!o || o.area === 'TR') return;
      const a = addrText(o);
      (writers.get(a) || writers.set(a, []).get(a))?.push({ pi, si, ri, line: k, name: ln.name });
    });
  })));
  /** @param {number} pi @param {number} si @param {number} ri */
  const rungOf = (pi, si, ri) => {
    const key = pi + '/' + si + '/' + ri;
    let r = parsed.get(key);
    if (!r) {
      const c = checkRung(project.programs[pi].sections[si].rungs[ri].il, scopes[pi]);
      r = { lines: c.lines, ops: c.ops, net: network(c.lines) };
      parsed.set(key, r);
    }
    return r;
  };
  /**
   * A contact's live state.
   * @param {any} r @param {number} i @param {number} pi
   */
  const contact = (r, i, pi) => {
    const ln = r.lines[i], o = r.ops[i] || [];
    if (!o[0] || o[0].area === 'TR') return { text: ln.src, value: 1, skip: true };
    let v;
    if (ln.cmp) {
      const val = (/** @type {any} */ x) => x.area === 'const' ? x.value : ln.cmp.long ? plc.long(addrText(x)) : plc.word(addrText(x));
      const x = val(o[0]), y = val(o[1]);
      v = { '=': x === y, '<>': x !== y, '<': x < y, '>': x > y, '<=': x <= y, '>=': x >= y }[/** @type {'='} */ (ln.cmp.op)] ? 1 : 0;
      return { text: ln.args.join(' ' + ln.cmp.op + ' ') + '  (' + x + ' ' + ln.cmp.op + ' ' + y + ')', value: v, cmp: true };
    }
    const a = addrText(o[0]);
    v = plc.bit(a);
    const sym = scopes[pi].find(ln.args[0]);
    return { text: ln.args[0], addr: a, comment: txt(sym?.comment || commentOf(a)),
      value: ln.def.neg ? v ^ 1 : v, raw: v, not: !!ln.def.neg, edge: ln.pre || '' };
  };
  /**
   * The contacts that keep a block false: all the failing ones in series, every branch of a parallel.
   * @param {any} r @param {any} b @param {number} pi @returns {{ok: boolean, why: any}}
   */
  const explain = (r, b, pi) => {
    if (!b) return { ok: true, why: null };
    if (b.t === 'c') { const c = contact(r, b.i, pi); return { ok: !!c.value, why: c.value ? null : c }; }
    const kids = b.a.map((/** @type {any} */ x) => explain(r, x, pi));
    if (b.t === 's') { const bad = kids.filter((/** @type {any} */ k) => !k.ok); return { ok: !bad.length, why: bad.length ? (bad.length === 1 ? bad[0].why : { all: bad.map((/** @type {any} */ k) => k.why) }) : null }; }
    const ok = kids.some((/** @type {any} */ k) => k.ok);
    return { ok, why: ok ? null : { any: kids.map((/** @type {any} */ k) => k.why) } };
  };
  /** The chain of conditions from the rail to output line `line`. @param {any} net @param {number} line */
  const pathTo = (net, line) => {
    /** @type {any[]|null} */
    let found = null;
    const walk = (/** @type {any} */ node, /** @type {any[]} */ conds) => {
      if (found) return;
      const here = node.cond ? [...conds, node.cond] : conds;
      for (const o of node.outs) {
        if (o.t === 'o') { if (o.i === line) { found = here; return; } }
        else walk(o, here);
      }
    };
    for (const n of net.roots) walk(n, []);
    return found;
  };
  /** Why `addr` is what it is: each rung that drives it, with the contacts that stop it. @param {string} addr */
  function why(addr) {
    const a = normal(addr);
    return (writers.get(a) || []).slice(0, 4).map(w => {
      const r = rungOf(w.pi, w.si, w.ri), p = project.programs[w.pi];
      const conds = r.net.error ? null : pathTo(r.net, w.line);
      const ex = conds ? explain(r, { t: 's', a: conds }, w.pi) : { ok: false, why: null };
      return { where: p.name + ' / ' + p.sections[w.si].name + ' R' + w.ri, p: w.pi, s: w.si, r: w.ri, comment: txt((p.sections[w.si].rungs[w.ri].comment || '').split('\n')[0]),
        coil: w.name, true: ex.ok, why: ex.why };
    });
  }

  // ------------------------------------------------------------------ what an operator must satisfy
  const group = (/** @type {RegExp} */ re) => [...symbolsLike(re)].map(([a, g]) => ({ addr: a, name: g.name || '', comment: txt(g.comment) }))
    .sort((x, y) => cmpAddr(x.addr, y.addr));
  const G = {
    master: group(/MASTER\s*CONDITION/i),
    start: group(/START\s*CONDITION\s*\d/i),
    home: group(/HOME\s*POS/i),
    unitStart: group(/Cycle\s*Start\s*Cond\s*$/i),
    alarms: project.globals.filter((/** @type {any} */ g) => /^AL\d+$/.test(g.name || '') && g.addr).map((/** @type {any} */ g) => ({ addr: normal(g.addr), name: g.name, comment: txt(g.comment) }))
      .sort((/** @type {any} */ x, /** @type {any} */ y) => cmpAddr(x.addr, y.addr)),
  };
  /** Every condition with its value; the ones that are OFF (or, for alarms, ON) come with why. */
  function diag() {
    const v = (/** @type {any} */ x) => ({ ...x, value: plc.bit(x.addr) });
    const withWhy = (/** @type {any} */ x, /** @type {boolean} */ bad) => (bad ? { ...x, why: why(x.addr) } : x);
    return {
      t: Math.round(tNow), running: plc.running,
      alarms: G.alarms.map(v).filter(x => x.value),
      master: G.master.map(v).map(x => withWhy(x, !x.value)),
      start: G.start.map(v).map(x => withWhy(x, !x.value)),
      home: G.home.map(v).map(x => withWhy(x, !x.value)),
      unitStart: G.unitStart.map(v),
    };
  }

  /**
   * What finally holds `addr` off: follow each failing contact that the program itself drives
   * (an internal W/H bit) back through ITS rung, until the contact is something from outside - an
   * input, a panel button, a timer - or depth runs out. Those leaves are what an operator can act on.
   * @param {string} addr @param {number} [depth] @param {Set<string>} [seen]
   * @returns {Array<{addr: string, text: string, comment: string, not: boolean, value: number, tag?: string, kind: string, via: string}>}
   */
  function rootCause(addr, depth = 5, seen = new Set()) {
    /** @type {any[]} */
    const out = [];
    const a0 = normal(addr);
    if (seen.has(a0)) return out;
    seen.add(a0);
    const r = why(a0)[0];
    if (!r || !r.why) return out;
    /** @type {any[]} */
    const flat = [];
    const walk = (/** @type {any} */ w) => { if (!w) return; if (w.all) w.all.forEach(walk); else if (w.any) w.any.forEach(walk); else flat.push(w); };
    walk(r.why);
    for (const c of flat) {
      if (c.cmp || !c.addr) { out.push({ addr: '', text: c.text, comment: '', not: false, value: 0, kind: 'compare', via: r.where }); continue; }
      const o = parseAddr(c.addr);
      const internal = o && (o.area === 'W' || o.area === 'H' || /^E/.test(o.area)) && writers.has(c.addr) && !tagOf.has(c.addr);
      if (internal && depth > 0) { const sub = rootCause(c.addr, depth - 1, seen); if (sub.length) { out.push(...sub); continue; } }
      const kind = tagOf.has(c.addr) ? 'scene' : o?.area === 'CIO' ? 'input' : o?.area === 'T' ? 'timer' : o?.area === 'W' && o.word >= 450 && o.word <= 479 ? 'hmi' : 'memory';
      out.push({ addr: c.addr, text: c.text, comment: c.comment, not: !!c.not, value: c.raw ?? 0, tag: tagOf.get(c.addr), kind, via: r.where });
    }
    const uniq = new Map(out.map(x => [x.addr + '|' + x.text, x]));
    return [...uniq.values()].slice(0, 12);
  }

  const autoRunning = [...symbolsLike(/^\s*AutoRunning\s*$/i)][0]?.[0] ?? '';
  /**
   * The start-up order as steps an operator follows - safety, MASTER ON, HOME, the start
   * conditions, running - each with whether it is done and, for the first that is not, the
   * outside things that hold it.
   */
  function steps() {
    const d = diag();
    const offOf = (/** @type {any[]} */ g) => g.filter(x => !x.value);
    const causes = (/** @type {any[]} */ g) => { const m = new Map(); for (const x of offOf(g).slice(0, 6)) for (const l of rootCause(x.addr)) m.set(l.addr + '|' + l.text, l); return [...m.values()].slice(0, 12); };
    const homeOff = offOf(d.start).filter(x => /HOME/i.test(x.comment));
    const startOff = offOf(d.start).filter(x => !/HOME/i.test(x.comment));
    const list = [
      { id: 'safety', title: 'No alarm, E-STOP out (then FAULT RESET on the HMI)', ok: !d.alarms.length, off: d.alarms.map(a => a.comment || a.name), causes: d.alarms.slice(0, 3).flatMap(a => rootCause(a.addr)).slice(0, 10) },
      { id: 'master', title: 'MASTER ON', ok: !offOf(d.master).length, off: offOf(d.master).map(x => x.comment), causes: causes(d.master) },
      { id: 'home', title: 'ALL UNIT HOME', ok: !homeOff.length, off: offOf(d.home).map(x => x.comment).slice(0, 12), causes: causes(homeOff) },
      { id: 'start', title: 'Start conditions', ok: !startOff.length, off: startOff.map(x => x.comment), causes: causes(startOff) },
      { id: 'run', title: 'AUTO RUN', ok: autoRunning ? !!plc.bit(autoRunning) : false, off: [], causes: autoRunning && !plc.bit(autoRunning) ? rootCause(autoRunning) : [] },
    ];
    // Once AUTO runs, HOME and the start conditions were met to get there: units mid-cycle are
    // away from home by design, and a red HOME step then only confuses.
    if (list[4].ok) for (const x of list) if (x.id === 'home' || x.id === 'start') Object.assign(x, { ok: true, off: [], causes: [] });
    return { t: d.t, running: d.running, steps: list, alarms: d.alarms };
  }

  return { units, sample, reset, ct, diag, why, rootCause, steps };
}

/** 'W482.1' and 'W482.01' are the same bit. @param {string} a */
function normal(a) { const o = parseAddr(a); return o ? addrText(o) : a; }
/** @param {string} a @param {string} b */
function cmpAddr(a, b) {
  const x = parseAddr(a), y = parseAddr(b);
  if (!x || !y) return a < b ? -1 : 1;
  return x.area !== y.area ? (x.area < y.area ? -1 : 1) : x.word - y.word || x.bit - y.bit;
}
