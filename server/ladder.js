// @ts-check
// Soft-PLC for Omron CS/CJ ladder (CX-Programmer mnemonic). It runs INSIDE the plant's step, in
// lockstep with the physics - one PLC scan per plant step - so there is no network between the
// program and the machine and no latency to measure: an input the plant publishes is read by
// the very next scan, and an output the scan writes is acted on in the same step.
//
// Each program is compiled ONCE into a JavaScript function (V8 then JITs it); a rung edited
// online recompiles only its program. Memory is the CJ2's: CIO, W, H, A, D, EM banks, timer and
// counter PVs as words beside their completion flags, TR bits, condition flags. Bits and words
// overlap exactly as on the PLC (W100.01 is bit 1 of W100), which is what a program written for
// the real CPU relies on and what an ST translation would lose.
//
// Timers and counters are BCD by default, as CX-Programmer compiles them unless the project
// says "execute timers/counters as binary" (the program under test converts counter PVs with
// BIN(023), which only makes sense in BCD). Timers run on SIM time, so a scaled or paused plant
// times exactly as the machine would.
//
// Forcing is the PLC's own: a forced bit keeps its value whatever the program or the plant
// writes to it, until it is released.
import { parseLine, parseOperand, checkRung, addrText, parseAddr } from '../lib/ladder.js';
import { rng } from '../lib/math.js';

const SIZE = /** @type {Record<string, number>} */ ({
  CIO: 6144, W: 512, H: 1536, A: 11536, D: 32768, E0: 32768, E1: 32768, E2: 32768, E3: 32768, T: 4096, C: 4096,
});
/** @type {Record<string, number>} */
const BASE = {};
let TOTAL = 0;
for (const [a, n] of Object.entries(SIZE)) { BASE[a] = TOTAL; TOTAL += n; }
/** Rungs per compiled function (see compile()). */
const CHUNK = 16;

const fromBcd = (/** @type {number} */ v) => {
  let r = 0, m = 1;
  for (let i = 0; i < 8 && v; i++, v = Math.floor(v / 16), m *= 10) r += (v % 16 > 9 ? 9 : v % 16) * m;
  return r;
};
const toBcd = (/** @type {number} */ v) => {
  let r = 0, m = 1;
  for (v = Math.max(0, Math.floor(v)); v; v = Math.floor(v / 10), m *= 16) r += (v % 10) * m;
  return r >>> 0;
};

/**
 * @typedef {import('../lib/ladder.js').Operand} Operand
 * @typedef {import('../server/cxp.js').Project} Project
 * @typedef {import('../server/cxp.js').Program} Program
 * @typedef {{prog: number, sec: number, rung: number, msg: string}} CompileError
 */

/**
 * @param {Project} project
 * @param {{binaryTimers?: boolean}} [opt]
 */
export function createLadder(project, { binaryTimers = false } = {}) {
  const M = new Uint16Array(TOTAL);          // every word area, one array
  const FM = new Uint16Array(TOTAL);         // force mask: 1 = this bit is forced
  const FV = new Uint16Array(TOTAL);         // forced values
  const TF = new Uint8Array(SIZE.T);         // timer completion flags
  const CF = new Uint8Array(SIZE.C);         // counter completion flags
  const TS = new Float64Array(SIZE.T).fill(NaN);   // when each timer started, sim ms
  const FL = new Uint8Array(128);            // condition flags CF000..CF127
  const TR = new Uint8Array(16);
  const G = new Map(project.globals.filter(g => g.name).map(g => [g.name, g]));

  const H = {
    now: 0, bcd: !binaryTimers, fromBcd, toBcd,
    /** TIM/TIMX/TIMH/TMHH: `unit` ms per count. */
    tim(/** @type {number} */ n, /** @type {number} */ sv, /** @type {number} */ c, /** @type {number} */ unit, /** @type {boolean} */ bcd) {
      const pv = BASE.T + n;
      if (!c) { TS[n] = NaN; TF[n] = 0; M[pv] = sv; return; }
      if (TS[n] !== TS[n]) TS[n] = H.now;
      const left = (bcd ? fromBcd(sv) : sv) * unit - (H.now - TS[n]);
      if (left <= 1e-9) { TF[n] = 1; M[pv] = 0; } else { TF[n] = 0; const k = Math.ceil(left / unit - 1e-9); M[pv] = bcd ? toBcd(k) : k; }
    },
    /** CNTR: reversible, cyclic. ED[e] / ED[e+1] hold the last increment / decrement input. */
    cntr(/** @type {Uint8Array} */ ED, /** @type {number} */ e, /** @type {number} */ n, /** @type {number} */ inc, /** @type {number} */ dec,
         /** @type {number} */ rst, /** @type {number} */ sv, /** @type {boolean} */ bcd) {
      const up = inc && !ED[e], dn = dec && !ED[e + 1];
      ED[e] = inc; ED[e + 1] = dec;
      const pv = BASE.C + n;
      if (rst) { M[pv] = 0; CF[n] = 0; return; }
      if (!!up === !!dn) return;
      let v = bcd ? fromBcd(M[pv]) : M[pv];
      const s = bcd ? fromBcd(sv) : sv;
      if (up) { if (v >= s) { v = 0; CF[n] = 1; } else { v++; CF[n] = 0; } }
      else if (v <= 0) { v = s; CF[n] = 1; } else { v--; CF[n] = 0; }
      M[pv] = bcd ? toBcd(v) : v;
    },
    /** CNT: counts down from SV on each rising edge; reset loads SV. */
    cnt(/** @type {Uint8Array} */ ED, /** @type {number} */ e, /** @type {number} */ n, /** @type {number} */ cin, /** @type {number} */ rst,
        /** @type {number} */ sv, /** @type {boolean} */ bcd) {
      const up = cin && !ED[e];
      ED[e] = cin;
      const pv = BASE.C + n;
      if (rst) { M[pv] = sv; CF[n] = 0; return; }
      if (!up) return;
      let v = bcd ? fromBcd(M[pv]) : M[pv];
      if (v > 0) v--;
      if (v === 0) CF[n] = 1;
      M[pv] = bcd ? toBcd(v) : v;
    },
    /** A word write that leaves forced bits alone. */
    ww(/** @type {number} */ i, /** @type {number} */ v) { const f = FM[i]; M[i] = f ? (v & ~f & 0xffff) | (M[i] & f) : v & 0xffff; },
    wl(/** @type {number} */ i, /** @type {number} */ v) { H.ww(i, v & 0xffff); H.ww(i + 1, (v >>> 16) & 0xffff); },
    rl(/** @type {number} */ i) { return (M[i] | (M[i + 1] << 16)) >>> 0; },
    cmp(/** @type {number} */ a, /** @type {number} */ b) {
      FL[6] = +(a === b); FL[5] = +(a > b); FL[7] = +(a < b); FL[0] = +(a >= b); FL[1] = +(a !== b); FL[2] = +(a <= b);
    },
    bcnt(/** @type {number} */ n, /** @type {number} */ s, /** @type {number} */ d) {
      let c = 0;
      for (let i = 0; i < n; i++) for (let w = M[s + i]; w; w &= w - 1) c++;
      H.ww(d, c); FL[6] = +(c === 0);
    },
    bset(/** @type {number} */ v, /** @type {number} */ s, /** @type {number} */ e) { for (let i = s; i <= e; i++) H.ww(i, v); },
    xfer(/** @type {number} */ n, /** @type {number} */ s, /** @type {number} */ d) {
      const tmp = M.slice(s, s + n);
      for (let i = 0; i < n; i++) H.ww(d + i, tmp[i]);
    },
    wsft(/** @type {number} */ v, /** @type {number} */ s, /** @type {number} */ e) { for (let i = e; i > s; i--) H.ww(i, M[i - 1]); H.ww(s, v); },
    seta(/** @type {number} */ d, /** @type {number} */ beg, /** @type {number} */ n, /** @type {number} */ on) {
      for (let k = beg; k < beg + n; k++) {
        const i = d + (k >> 4), m = 1 << (k & 15);
        if (FM[i] & m) continue;
        M[i] = on ? M[i] | m : M[i] & ~m;
      }
    },
    add(/** @type {number} */ a, /** @type {number} */ b, /** @type {number} */ d, /** @type {number} */ sub) {
      const r = sub ? a - b : a + b;
      FL[4] = sub ? +(a < b) : +(r > 0xffff);
      H.ww(d, r & 0xffff); FL[6] = +((r & 0xffff) === 0); FL[8] = +!!(r & 0x8000);
    },
    addl(/** @type {number} */ a, /** @type {number} */ b, /** @type {number} */ d, /** @type {number} */ sub) {
      const r = sub ? a - b : a + b;
      H.wl(d, r >>> 0); FL[6] = +((r >>> 0) === 0);
    },
    mul(/** @type {number} */ a, /** @type {number} */ b, /** @type {number} */ d, /** @type {number} */ signed) {
      const r = signed ? ((a << 16) >> 16) * ((b << 16) >> 16) : a * b;
      H.wl(d, r >>> 0); FL[6] = +(r === 0);
    },
    div(/** @type {number} */ a, /** @type {number} */ b, /** @type {number} */ d, /** @type {number} */ signed) {
      if (!b) { FL[3] = 1; return; }
      const x = signed ? (a << 16) >> 16 : a, y = signed ? (b << 16) >> 16 : b;
      const q = Math.trunc(x / y), r = x - q * y;
      H.ww(d, q & 0xffff); H.ww(d + 1, r & 0xffff);
    },
    addb(/** @type {number} */ a, /** @type {number} */ b, /** @type {number} */ d, /** @type {number} */ sub) {
      let r = sub ? fromBcd(a) - fromBcd(b) : fromBcd(a) + fromBcd(b);
      FL[4] = +(r > 9999 || r < 0);
      r = ((r % 10000) + 10000) % 10000;
      H.ww(d, toBcd(r));
    },
  };

  /** @type {Array<{fn: Function|null, ed: Uint8Array, errors: CompileError[]}>} */
  let progs = [];
  /** @type {CompileError[]} */
  let errors = [];
  let first = true, running = true, scans = 0, scanUs = 0, scanMaxUs = 0;

  /** @param {Program} p */
  const scopeOf = p => {
    const L = new Map(p.locals.filter(g => g.name).map(g => [g.name, g]));
    return { find: (/** @type {string} */ x) => L.get(x) ?? G.get(x) };
  };

  /** Absolute word index of an address operand (not a constant). @param {Operand} o */
  function index(o) {
    const b = BASE[o.area];
    if (b === undefined) throw new Error('area ' + o.area + ' is not in this soft-PLC');
    if (o.word >= SIZE[o.area]) throw new Error(o.text + ' is past the end of ' + o.area);
    return b + o.word;
  }
  /** Index EXPRESSION (an indirect @D operand is resolved at run time). @param {Operand} o */
  function ix(o) {
    if (o.area === 'ind') return `(${BASE.D}+(M[${BASE.D + /** @type {any} */ (o.ind).word}]&0x7fff))`;
    if (o.area === 'const') throw new Error(o.text + ' is a constant where an address is needed');
    return String(index(o));
  }
  /** @param {Operand} o */
  function rword(o) { return o.area === 'const' ? String(/** @type {number} */ (o.value) & 0xffff) : `M[${ix(o)}]`; }
  /** @param {Operand} o */
  function rlong(o) { return o.area === 'const' ? String(/** @type {number} */ (o.value) >>> 0) : `H.rl(${ix(o)})`; }
  /** @param {Operand} o */
  function rbit(o) {
    if (o.area === 'T') return `TF[${o.word}]`;
    if (o.area === 'C') return `CF[${o.word}]`;
    if (o.area === 'TR') return `TR[${o.word}]`;
    if (o.area === 'CF') return `FL[${o.word}]`;
    if (o.bit < 0) throw new Error(o.text + ' is a word, not a bit');
    return `((M[${index(o)}]>>${o.bit})&1)`;
  }
  /** @param {Operand} o @param {string} v */
  function wbit(o, v) {
    if (o.area === 'TR') return `TR[${o.word}]=${v};`;
    if (o.area === 'T' || o.area === 'C' || o.area === 'CF' || o.bit < 0) throw new Error(o.text + ' cannot be written as a bit');
    const i = index(o), m = 1 << o.bit;
    return `if(!(FM[${i}]&${m}))M[${i}]=(${v})?(M[${i}]|${m}):(M[${i}]&${~m & 0xffff});`;
  }

  /**
   * Compiles one program. A rung that does not compile is left out and reported: the rest of
   * the program runs, and the report says exactly which rung is missing.
   * @param {number} pi
   */
  function compile(pi) {
    const p = project.programs[pi], scope = scopeOf(p);
    /** @type {CompileError[]} */
    const errs = [];
    /** @type {string[]} */
    const out = [];
    let ed = 0, maxDepth = 0;
    p.sections.forEach((s, si) => s.rungs.forEach((r, ri) => {
      if (!r.il.length) return;
      try {
        const code = rungCode(r.il, scope, () => ed++, d => { maxDepth = Math.max(maxDepth, d); });
        out.push(code);
      } catch (e) {
        errs.push({ prog: pi, sec: si, rung: ri, msg: /** @type {Error} */ (e).message });
      }
    }));
    // Chunks of CHUNK rungs, one function each. Measured on the final-caulking program: one
    // function per program ran interpreted for ~15000 scans (240 us/scan) before V8 optimised
    // it; 16-rung functions are optimised within ~5000 scans and then run 13 us/scan (48: 16 us,
    // 96: slow to warm again). The interlock state crosses
    // chunk boundaries as the return value, and END(001) returns -1.
    const vars = Array.from({ length: maxDepth + 1 }, (_, i) => 's' + i + '=0').join(',');
    /** @type {Function[]} */
    const fns = [];
    try {
      for (let i = 0; i < out.length; i += CHUNK) {
        fns.push(new Function('M', 'FM', 'TF', 'CF', 'TR', 'FL', 'ED', 'H', 'IL', `let ${vars};\n` + out.slice(i, i + CHUNK).join('\n') + '\nreturn IL;'));
      }
    } catch (e) {
      errs.push({ prog: pi, sec: -1, rung: -1, msg: 'generated code does not compile: ' + /** @type {Error} */ (e).message });
      fns.length = 0;
    }
    /** @type {(M: Uint16Array, FM: Uint16Array, TF: Uint8Array, CF: Uint8Array, TR: Uint8Array, FL: Uint8Array, ED: Uint8Array, H: any) => void} */
    const fn = (M, FM, TF, CF, TR, FL, ED, H) => {
      let il = 1;
      for (let i = 0; i < fns.length; i++) { il = fns[i](M, FM, TF, CF, TR, FL, ED, H, il); if (il < 0) return; }
    };
    return { fn, ed: new Uint8Array(ed + 2), errors: errs };
  }

  /**
   * One rung's code. The stack lives in s0..sN; `d` is the index of its top.
   * @param {string[]} il @param {any} scope @param {() => number} edge @param {(d: number) => void} depth
   */
  function rungCode(il, scope, edge, depth) {
    const chk = checkRung(il, scope);
    if (chk.errors.length) throw new Error(chk.errors.join('; '));
    let d = -1;
    /** @type {string[]} */
    const c = [];
    const top = () => 's' + d;
    chk.lines.forEach((ln, li) => {
      const ops = chk.ops[li], k = ln.def.k;
      if (k === 'nop') return;
      // A contact's value, differentiated when it carries @ or %.
      const contact = () => {
        let v;
        if (ln.cmp) {
          const [a, b] = ops;
          let x = ln.cmp.long ? rlong(a) : rword(a), y = ln.cmp.long ? rlong(b) : rword(b);
          if (ln.cmp.signed) { x = ln.cmp.long ? `(${x}|0)` : `((${x}<<16)>>16)`; y = ln.cmp.long ? `(${y}|0)` : `((${y}<<16)>>16)`; }
          const op = ln.cmp.op === '=' ? '===' : ln.cmp.op === '<>' ? '!==' : ln.cmp.op;
          v = `+(${x}${op}${y})`;
        } else v = rbit(ops[0]);
        if (!ln.pre) return { pre: '', v: ln.def.neg ? `(${v}^1)` : v };
        // A differentiated NOT contact is the NOT of the edge, not the edge of the NOT: @ANDNOT X
        // is OFF for the one scan X rises. Every @ANDNOT in the final-caulking program breaks a
        // self-hold that way (AutoRunning on Discharge_Complete, a magnet START on its STOP); read
        // the other way round - ON only for the scan X falls - none of those circuits could hold,
        // and AUTO RUN never latched (measured).
        const e = edge();
        const d = ln.pre === '@' ? `(v&(ED[${e}]^1))` : `((v^1)&ED[${e}])`;
        return { pre: `{const v=${v};`, v: ln.def.neg ? `(${d}^1)` : d, post: `ED[${e}]=v;}` };
      };
      if (k === 'ld' || k === 'and' || k === 'or') {
        const x = contact();
        if (k === 'ld') { d++; depth(d); }
        const set = k === 'ld' ? `${top()}=${x.v};` : k === 'and' ? `${top()}&=${x.v};` : `${top()}|=${x.v};`;
        c.push((x.pre ?? '') + set + (x.post ?? ''));
        return;
      }
      if (k === 'andld') { c.push(`s${d - 1}&=s${d};`); d--; return; }
      if (k === 'orld') { c.push(`s${d - 1}|=s${d};`); d--; return; }
      if (k === 'conn') {
        if (ln.name === 'NOT') { c.push(`${top()}^=1;`); return; }
        const e = edge();
        c.push(ln.name === 'UP' ? `{const v=${top()};${top()}=v&(ED[${e}]^1);ED[${e}]=v;}` : `{const v=${top()};${top()}=(v^1)&ED[${e}];ED[${e}]=v;}`);
        return;
      }
      if (k === 'il') { c.push(`IL&=${top()};`); return; }
      if (k === 'ilc') { c.push('IL=1;'); return; }
      if (k === 'end') { c.push('return -1;'); return; }
      if (k !== 'out') throw new Error('cannot compile ' + ln.src);
      // Outputs. `cond` is the execution condition; a differentiated instruction runs on its edge.
      let cond = top();
      let pre = '', post = '';
      if (ln.pre) {
        const e = edge();
        pre = `{const v=${cond};`;
        cond = ln.pre === '@' ? `(v&(ED[${e}]^1))` : `((v^1)&ED[${e}])`;
        post = `ED[${e}]=v;}`;
      }
      const run = (/** @type {string} */ body) => c.push(pre + `if(${cond}&IL){${body}}` + post);
      const [a, b, x] = ops;
      switch (ln.name) {
        case 'OUT': c.push(pre + wbit(a, `${cond}&IL`) + post); return;
        case 'OUTNOT': c.push(pre + wbit(a, `(${cond}^1)&IL`) + post); return;
        case 'SET': run(wbit(a, '1')); return;
        case 'RSET': run(wbit(a, '0')); return;
        case 'KEEP': {
          const set = `s${d - 1}`, rst = top();
          c.push(`if(IL){if(${rst}){${wbit(a, '0')}}else if(${set}){${wbit(a, '1')}}}`);
          d -= 1;
          return;
        }
        case 'DIFU': case 'DIFD': {
          const e = edge();
          const v = ln.name === 'DIFU' ? `(v&(ED[${e}]^1))` : `((v^1)&ED[${e}])`;
          c.push(`if(IL){const v=${cond};${wbit(a, v)}ED[${e}]=v;}`);
          return;
        }
        case 'TIM': case 'TIMX': case 'TIMH': case 'TIMHX': case 'TMHH': case 'TMHHX': {
          if (a.area !== 'T') throw new Error(ln.name + ' needs a timer number, got ' + a.text);
          const unit = ln.name.startsWith('TIMH') ? 10 : ln.name.startsWith('TMHH') ? 1 : 100;
          const bcd = ln.name.endsWith('X') ? 'false' : 'H.bcd';
          c.push(pre + `H.tim(${a.word},${rword(b)},${cond}&IL,${unit},${bcd});` + post);
          return;
        }
        case 'CNTR': case 'CNTRX': {
          if (a.area !== 'C') throw new Error(ln.name + ' needs a counter number, got ' + a.text);
          const e = edge(); edge();
          const bcd = ln.name.endsWith('X') ? 'false' : 'H.bcd';
          c.push(`if(IL)H.cntr(ED,${e},${a.word},s${d - 2},s${d - 1},s${d},${rword(b)},${bcd});`);
          d -= 2;
          return;
        }
        case 'CNT': case 'CNTX': {
          if (a.area !== 'C') throw new Error(ln.name + ' needs a counter number, got ' + a.text);
          const e = edge();
          const bcd = ln.name.endsWith('X') ? 'false' : 'H.bcd';
          c.push(`if(IL)H.cnt(ED,${e},${a.word},s${d - 1},s${d},${rword(b)},${bcd});`);
          d -= 1;
          return;
        }
        case 'MOV': run(`H.ww(${ix(b)},${rword(a)});`); return;
        case 'MOVL': run(`H.wl(${ix(b)},${rlong(a)});`); return;
        case 'CMP': run(`H.cmp(${rword(a)},${rword(b)});`); return;
        case 'CMPL': run(`H.cmp(${rlong(a)},${rlong(b)});`); return;
        case 'BIN': run(`H.ww(${ix(b)},H.fromBcd(${rword(a)}));`); return;
        case 'BINL': run(`H.wl(${ix(b)},H.fromBcd(${rlong(a)}));`); return;
        case 'BCD': run(`H.ww(${ix(b)},H.toBcd(${rword(a)}));`); return;
        case 'BCDL': run(`H.wl(${ix(b)},H.toBcd(${rlong(a)}));`); return;
        case 'BCNT': run(`H.bcnt(${rword(a)},${ix(b)},${ix(x)});`); return;
        case 'BSET': sameArea(b, x); run(`H.bset(${rword(a)},${ix(b)},${ix(x)});`); return;
        case 'XFER': run(`H.xfer(${rword(a)},${ix(b)},${ix(x)});`); return;
        case 'WSFT': sameArea(b, x); run(`H.wsft(${rword(a)},${ix(b)},${ix(x)});`); return;
        case 'SETA': run(`H.seta(${ix(a)},${rword(b)},${rword(x)},1);`); return;
        case 'RSTA': run(`H.seta(${ix(a)},${rword(b)},${rword(x)},0);`); return;
        case 'ASL': run(`{const i=${ix(a)};FL[4]=M[i]>>15;H.ww(i,M[i]<<1);}`); return;
        case 'ASR': run(`{const i=${ix(a)};FL[4]=M[i]&1;H.ww(i,M[i]>>1);}`); return;
        case '+': run(`H.add(${rword(a)},${rword(b)},${ix(x)},0);`); return;
        case '-': run(`H.add(${rword(a)},${rword(b)},${ix(x)},1);`); return;
        case '+L': run(`H.addl(${rlong(a)},${rlong(b)},${ix(x)},0);`); return;
        case '-L': run(`H.addl(${rlong(a)},${rlong(b)},${ix(x)},1);`); return;
        case '*': run(`H.mul(${rword(a)},${rword(b)},${ix(x)},1);`); return;
        case '*U': run(`H.mul(${rword(a)},${rword(b)},${ix(x)},0);`); return;
        case '/': run(`H.div(${rword(a)},${rword(b)},${ix(x)},1);`); return;
        case '/U': run(`H.div(${rword(a)},${rword(b)},${ix(x)},0);`); return;
        case '+B': run(`H.addb(${rword(a)},${rword(b)},${ix(x)},0);`); return;
        case '-B': run(`H.addb(${rword(a)},${rword(b)},${ix(x)},1);`); return;
        case '++': run(`{const i=${ix(a)};H.ww(i,M[i]+1);}`); return;
        case '--': run(`{const i=${ix(a)};H.ww(i,M[i]-1);}`); return;
        case '++L': run(`{const i=${ix(a)};H.wl(i,H.rl(i)+1);}`); return;
        case '--L': run(`{const i=${ix(a)};H.wl(i,H.rl(i)-1);}`); return;
        case '++B': run(`{const i=${ix(a)};H.ww(i,H.toBcd((H.fromBcd(M[i])+1)%10000));}`); return;
        case '--B': run(`{const i=${ix(a)};H.ww(i,H.toBcd((H.fromBcd(M[i])+9999)%10000));}`); return;
        default: throw new Error('the soft-PLC does not execute ' + ln.name + (ln.fn ? '(' + ln.fn + ')' : '') + ' yet');
      }
    });
    return c.join('');
  }
  /** @param {Operand} a @param {Operand} b */
  function sameArea(a, b) {
    if (a.area === 'ind' || b.area === 'ind') return;
    if (a.area !== b.area || b.word < a.word) throw new Error(a.text + '..' + b.text + ' is not one range in one area');
  }

  function compileAll() {
    progs = project.programs.map((_, i) => compile(i));
    errors = progs.flatMap(p => p.errors);
  }
  compileAll();
  // Programs run in task order, as the CPU runs cyclic tasks 0, 1, 2 ...
  const order = () => project.programs.map((p, i) => [p.taskId, i]).sort((a, b) => a[0] - b[0]).map(x => x[1]);
  let runOrder = order();

  // ---------------------------------------------------------------- addresses for the outside
  /** @type {Map<string, {i: number, bit: number, area: string, word: number}>} */
  const addrCache = new Map();
  /** Canonical address text -> where it lives. @param {string} text */
  function locate(text) {
    let r = addrCache.get(text);
    if (r) return r;
    const o = parseAddr(text);
    if (!o) throw new Error('not an address: ' + text);
    r = { i: o.area === 'TR' || o.area === 'CF' ? -1 : index(o), bit: o.bit, area: o.area, word: o.word };
    addrCache.set(text, r);
    return r;
  }

  const plc = {
    project, M, TF, CF,
    get errors() { return errors; },
    get running() { return running; },
    get stats() { return { scans, scanUs: Math.round(scanUs * 10) / 10, scanMaxUs: Math.round(scanMaxUs), running }; },

    /** One scan at sim time `now` (ms). */
    scan(/** @type {number} */ now) {
      if (!running) return;
      const t0 = performance.now();
      H.now = now;
      FL[113] = 1; FL[114] = 0;
      FL[100] = +(now % 100 < 50); FL[101] = +(now % 200 < 100); FL[102] = +(now % 1000 < 500);
      FL[103] = +(now % 20 < 10); FL[104] = +(now % 60000 < 30000); FL[105] = +(now % 10 < 5);
      FL[106] ^= 1; FL[107] ^= 1;
      const a200 = BASE.A + 200;
      M[a200] = first ? M[a200] | (1 << 11) : M[a200] & ~(1 << 11);
      for (const pi of runOrder) {
        const p = progs[pi];
        TR.fill(0);
        p.fn?.(M, FM, TF, CF, TR, FL, p.ed, H);
      }
      first = false;
      scans++;
      const us = (performance.now() - t0) * 1000;
      scanUs = scans === 1 ? us : scanUs * 0.98 + us * 0.02;
      if (us > scanMaxUs) scanMaxUs = us;
    },
    /**
     * Runs the program `n` times on scratch inputs and then clears every trace of it (a cold
     * reset), so that V8 has compiled it before the machine starts. Measured on final-caulking: a
     * chunk is optimised after about 6500 calls, and until then a scan costs 500-1100 us against
     * 50-120 warm - thirteen seconds of plant in which the step is over its 2 ms budget on a busy
     * laptop, and again whenever the first product reaches a station whose rungs had never been
     * true ("plant stalled" at sim 27-45 s of every run, none after). The inputs are FUZZED, not
     * idle: with idle inputs the code is still at 159 us after 8000 scans, because half of every
     * rung has never run and is thrown out again the first time it does.
     * @param {number} n @param {string[]} addrs the addresses to toggle (the scene's io map)
     */
    warm(n, addrs) {
      const r = rng(1);
      for (let i = 0; i < n; i++) {
        if (i % 8 === 0) for (const a of addrs) plc.set(a, r() < 0.5 ? 1 : 0);
        plc.scan(i * 2);
      }
      plc.reset(true);
      scanUs = 0;
    },
    /** RUN / PROGRAM mode. PROGRAM stops scanning; the adapter drives every output OFF. */
    setRunning(/** @type {boolean} */ on) { if (on && !running) first = true; running = on; },
    /**
     * Power cycle: memory kept where the PLC keeps it (H, D, EM, counters), the rest cleared.
     * `cold` clears the retained areas too - a simulation Reset takes every part out of the
     * machine, and work memory that still says 'have work' would be a lie the program acts on.
     */
    reset(cold = false) {
      const keep = cold ? new Set() : new Set(['H', 'D', 'E0', 'E1', 'E2', 'E3', 'C']);
      for (const [a, n] of Object.entries(SIZE)) if (!keep.has(a)) M.fill(0, BASE[a], BASE[a] + n);
      TF.fill(0); TS.fill(NaN); TR.fill(0); FL.fill(0);
      if (cold) CF.fill(0);                       // counter PVs are cleared above: their flags go with them
      for (const p of progs) p.ed.fill(0);
      applyForces();
      first = true; scans = 0; scanMaxUs = 0;
    },

    /** A bit (0/1) or a word, by canonical address: '3205.07', 'W100', 'T0450' (flag), 'T0450' as word via word(). @param {string} a */
    bit(a) {
      const r = locate(a);
      if (r.area === 'T' && r.bit < 0) return TF[r.word];
      if (r.area === 'C' && r.bit < 0) return CF[r.word];
      if (r.area === 'TR') return TR[r.word];
      if (r.area === 'CF') return FL[r.word];
      return r.bit < 0 ? +(M[r.i] !== 0) : (M[r.i] >> r.bit) & 1;
    },
    /** @param {string} a */
    word(a) { const r = locate(a); return r.i < 0 ? 0 : M[r.i]; },
    /** @param {string} a */
    long(a) { const r = locate(a); return r.i < 0 ? 0 : (M[r.i] | (M[r.i + 1] << 16)) >>> 0; },
    /** Writes as the plant's inputs do: forced bits keep their forced value. @param {string} a @param {number|boolean} v */
    set(a, v) {
      const r = locate(a);
      if (r.i < 0) return;
      if (r.bit >= 0) { const m = 1 << r.bit; if (!(FM[r.i] & m)) M[r.i] = v ? M[r.i] | m : M[r.i] & ~m; }
      else H.ww(r.i, Number(v));
    },
    /**
     * Force a bit ON/OFF, or a whole word to a value; null releases it. The forced value wins
     * over the program and over the plant until released, as CX-Programmer's force does.
     * @param {string} a @param {number|boolean|null} v
     */
    force(a, v) {
      const r = locate(a);
      if (r.i < 0) throw new Error(a + ' cannot be forced');
      const m = r.bit >= 0 ? 1 << r.bit : 0xffff;
      if (v === null) { FM[r.i] &= ~m; return; }
      FM[r.i] |= m;
      const val = r.bit >= 0 ? (v ? m : 0) : Number(v) & 0xffff;
      FV[r.i] = (FV[r.i] & ~m) | val;
      M[r.i] = (M[r.i] & ~m) | val;
    },
    /** Every forced address with its value. */
    forced() {
      /** @type {Array<[string, number]>} */
      const out = [];
      for (const [a, n] of Object.entries(SIZE)) {
        for (let w = 0; w < n; w++) {
          const i = BASE[a] + w, f = FM[i];
          if (!f) continue;
          const pre = a === 'CIO' ? '' : a.startsWith('E') ? a + '_' : a;
          if (f === 0xffff) { out.push([pre + w, M[i]]); continue; }
          for (let b = 0; b < 16; b++) if (f & (1 << b)) out.push([pre + w + '.' + String(b).padStart(2, '0'), (M[i] >> b) & 1]);
        }
      }
      return out;
    },
    releaseAll() { FM.fill(0); },

    /**
     * Replaces, inserts or deletes rungs of one section and recompiles that program - online
     * edit. Refused (nothing changes) when the new rung does not compile.
     * @param {{prog: number, sec: number, at: number, del?: number, rungs?: Array<{il: string[], comment?: string}>}} ed
     * @returns {{ok: boolean, errors: string[]}}
     */
    edit(ed) {
      const p = project.programs[ed.prog], s = p?.sections[ed.sec];
      if (!s) return { ok: false, errors: ['no such section'] };
      if (!(ed.at >= 0 && ed.at <= s.rungs.length)) return { ok: false, errors: ['rung index out of range'] };
      const scope = scopeOf(p);
      const add = (ed.rungs ?? []).map(r => ({ comment: String(r.comment ?? ''), il: r.il.map(x => x.trim()).filter(Boolean), notes: /** @type {string[]} */ ([]) }));
      /** @type {string[]} */
      const errs = [];
      add.forEach((r, i) => { try { rungCode(r.il, scope, () => 0, () => {}); } catch (e) { errs.push('rung ' + (ed.at + i) + ': ' + /** @type {Error} */ (e).message); } });
      if (errs.length) return { ok: false, errors: errs };
      const old = s.rungs.splice(ed.at, ed.del ?? 0, ...add);
      // Keep the notes of a rung that was replaced in place.
      if (old.length === add.length) old.forEach((o, i) => { add[i].notes = o.notes; });
      const c = compile(ed.prog);
      if (c.errors.length && c.errors.some(e => e.sec === ed.sec)) {
        s.rungs.splice(ed.at, add.length, ...old);
        return { ok: false, errors: c.errors.map(e => e.msg) };
      }
      progs[ed.prog] = c;
      errors = progs.flatMap(x => x.errors);
      runOrder = order();
      return { ok: true, errors: [] };
    },
  };

  function applyForces() { for (let i = 0; i < TOTAL; i++) if (FM[i]) M[i] = (M[i] & ~FM[i]) | (FV[i] & FM[i]); }
  return plc;
}

/**
 * The canonical address of an operand as written in a program (symbol, array element, bare
 * address), so the viewer and the scene can name the same bit.
 * @param {string} text @param {any} scope @param {string} [kind]
 */
export function resolve(text, scope, kind = 'b') { return addrText(parseOperand(text, kind, scope)); }

export { parseLine };
