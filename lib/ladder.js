// @ts-check
// Omron CS/CJ ladder in mnemonic form (CX-Programmer IL): the instruction set, operands, and the
// rung NETWORK a viewer draws. Pure: no three, no Rapier, no fs, so the browser draws the same
// rung the soft-PLC runs (server/ladder.js), from the same parse.
//
// A rung is its mnemonic lines, exactly as CX-Programmer stores them (.cxp SL strings). The
// executor runs the lines as the stack machine they are; this file only says what each line IS.
// Drawing a rung from its mnemonic is the conversion CX-Programmer itself does, and a rung that
// cannot be drawn (an OR after an output) is still executed correctly - the view falls back to
// the mnemonic, never the PLC.

let sjisDecoder = /** @type {TextDecoder|null|undefined} */ (undefined);
/**
 * A comment as a person reads it. The .cxp keeps text as its bytes (one char per byte, so a
 * project writes back byte for byte), and a Japanese maker's comments are Shift-JIS bytes: shown
 * as they are stored they read '©®N®ðñHP@...'. For DISPLAY only - never write the result back.
 * @param {string} s @returns {string}
 */
export function commentText(s) {
  if (!s || !/[\x80-\xff]/.test(s) || /[^\x00-\xff]/.test(s)) return s;
  if (sjisDecoder === undefined) { try { sjisDecoder = new TextDecoder('shift_jis', { fatal: true }); } catch { sjisDecoder = null; } }
  if (!sjisDecoder) return s;
  try { return sjisDecoder.decode(Uint8Array.from(s, c => c.charCodeAt(0))); } catch { return s; }
}

/**
 * k: ld | and | or   a contact (LD/AND/OR, NOT, @ up, % down, or a compare box)
 *    andld | orld    block logic
 *    out             an output that takes the rung condition (coils and function boxes)
 *    conn            changes the condition in place (UP, DOWN, NOT)
 *    il | ilc | end  interlock, and the end of a program
 *    nop             a display hint (`//`) kept for writing back
 * pops: how many conditions an output takes off the stack (KEEP 2, CNTR 3, CNT 2)
 * ops: operand kinds - b bit, w word (source or destination), l double word, t timer number,
 *      c counter number, n a count or a constant word
 * @typedef {{k: string, ops?: string[], pops?: number, neg?: boolean}} Instr
 */

/** @type {Record<string, Instr>} */
const BASE = {
  LD: { k: 'ld', ops: ['b'] }, LDNOT: { k: 'ld', ops: ['b'], neg: true },
  AND: { k: 'and', ops: ['b'] }, ANDNOT: { k: 'and', ops: ['b'], neg: true },
  OR: { k: 'or', ops: ['b'] }, ORNOT: { k: 'or', ops: ['b'], neg: true },
  ANDLD: { k: 'andld' }, ORLD: { k: 'orld' },
  OUT: { k: 'out', ops: ['b'] }, OUTNOT: { k: 'out', ops: ['b'] },
  SET: { k: 'out', ops: ['b'] }, RSET: { k: 'out', ops: ['b'] },
  KEEP: { k: 'out', ops: ['b'], pops: 2 },
  DIFU: { k: 'out', ops: ['b'] }, DIFD: { k: 'out', ops: ['b'] },
  UP: { k: 'conn' }, DOWN: { k: 'conn' }, NOT: { k: 'conn' },
  IL: { k: 'il' }, ILC: { k: 'ilc' }, END: { k: 'end' }, NOP: { k: 'nop' },
  TIM: { k: 'out', ops: ['t', 'w'] }, TIMX: { k: 'out', ops: ['t', 'w'] },
  TIMH: { k: 'out', ops: ['t', 'w'] }, TIMHX: { k: 'out', ops: ['t', 'w'] },
  TMHH: { k: 'out', ops: ['t', 'w'] }, TMHHX: { k: 'out', ops: ['t', 'w'] },
  CNT: { k: 'out', ops: ['c', 'w'], pops: 2 }, CNTX: { k: 'out', ops: ['c', 'w'], pops: 2 },
  CNTR: { k: 'out', ops: ['c', 'w'], pops: 3 }, CNTRX: { k: 'out', ops: ['c', 'w'], pops: 3 },
  MOV: { k: 'out', ops: ['w', 'w'] }, MOVL: { k: 'out', ops: ['l', 'l'] },
  CMP: { k: 'out', ops: ['w', 'w'] }, CMPL: { k: 'out', ops: ['l', 'l'] },
  BIN: { k: 'out', ops: ['w', 'w'] }, BINL: { k: 'out', ops: ['l', 'l'] },
  BCD: { k: 'out', ops: ['w', 'w'] }, BCDL: { k: 'out', ops: ['l', 'l'] },
  BCNT: { k: 'out', ops: ['n', 'w', 'w'] }, BSET: { k: 'out', ops: ['w', 'w', 'w'] },
  XFER: { k: 'out', ops: ['n', 'w', 'w'] }, WSFT: { k: 'out', ops: ['w', 'w', 'w'] },
  SETA: { k: 'out', ops: ['w', 'n', 'n'] }, RSTA: { k: 'out', ops: ['w', 'n', 'n'] },
  ASL: { k: 'out', ops: ['w'] }, ASR: { k: 'out', ops: ['w'] },
  '+': { k: 'out', ops: ['w', 'w', 'w'] }, '-': { k: 'out', ops: ['w', 'w', 'w'] },
  '*': { k: 'out', ops: ['w', 'w', 'l'] }, '*U': { k: 'out', ops: ['w', 'w', 'l'] },
  '/': { k: 'out', ops: ['w', 'w', 'l'] }, '/U': { k: 'out', ops: ['w', 'w', 'l'] },
  '+L': { k: 'out', ops: ['l', 'l', 'l'] }, '-L': { k: 'out', ops: ['l', 'l', 'l'] },
  '++': { k: 'out', ops: ['w'] }, '--': { k: 'out', ops: ['w'] },
  '++L': { k: 'out', ops: ['l'] }, '--L': { k: 'out', ops: ['l'] },
  '+B': { k: 'out', ops: ['w', 'w', 'w'] }, '-B': { k: 'out', ops: ['w', 'w', 'w'] },
  '++B': { k: 'out', ops: ['w'] }, '--B': { k: 'out', ops: ['w'] },
};
/** Compare contacts: LD=(300), AND<>L(306), OR>=S(327) ... The function number is ignored. */
const CMP_RE = /^(LD|AND|OR)(=|<>|<=|>=|<|>)(S?)(L?)$/;

/**
 * @typedef {{src: string, pre: '' | '@' | '%', name: string, fn: string, args: string[], def: Instr,
 *            cmp?: {op: string, signed: boolean, long: boolean}}} Line
 */

/**
 * One mnemonic line. Unknown instructions come back with def.k 'bad' so the caller can refuse
 * them by name; a soft-PLC that silently skips an instruction is a PLC that lies.
 * @param {string} src @returns {Line}
 */
export function parseLine(src) {
  const s = src.trim();
  if (s.startsWith('//')) return { src: s, pre: '', name: '//', fn: '', args: [], def: { k: 'nop' } };
  const [head, ...args] = s.split(/\s+/);
  const pre = /** @type {'' | '@' | '%'} */ (head[0] === '@' || head[0] === '%' ? head[0] : '');
  const h = pre ? head.slice(1) : head;
  const m = /^(.*?)(?:\((\d+)\))?$/.exec(h) ?? ['', h, ''];
  const name = m[1], fn = m[2] ?? '';
  const c = CMP_RE.exec(name);
  if (c) {
    const def = { k: c[1].toLowerCase(), ops: [c[4] ? 'l' : 'w', c[4] ? 'l' : 'w'] };
    return { src: s, pre, name, fn, args, def, cmp: { op: c[2], signed: !!c[3], long: !!c[4] } };
  }
  const def = BASE[name] ?? { k: 'bad' };
  return { src: s, pre, name, fn, args, def };
}

// --------------------------------------------------------------------------- operands

/**
 * A resolved operand.
 *   area: CIO W H A D E0..E? T C TR CF (condition flags) | const | ind (@D: the word holds a D address)
 *   word/bit: bit -1 means a whole word
 * @typedef {{area: string, word: number, bit: number, value?: number, name?: string, text: string,
 *            ind?: {area: string, word: number}, dim?: number}} Operand
 */

/**
 * CX-Programmer's predefined symbols. The CF addresses are the CJ2's condition flags; the rest
 * are the auxiliary-area bits the manuals give them. The clocks are computed from sim time.
 */
export const SYSTEM = /** @type {Record<string, string>} */ ({
  P_GE: 'CF000', P_NE: 'CF001', P_LE: 'CF002', P_ER: 'CF003', P_CY: 'CF004', P_GT: 'CF005', P_EQ: 'CF006',
  P_LT: 'CF007', P_N: 'CF008', P_OF: 'CF009', P_UF: 'CF010', P_AER: 'CF011',
  P_0_1s: 'CF100', P_0_2s: 'CF101', P_1s: 'CF102', P_0_02s: 'CF103', P_1min: 'CF104',
  P_On: 'CF113', P_Off: 'CF114',
  // Not in the CF area on every CPU; kept as flags so a program that tests them still runs.
  P_0_01s: 'CF105', P_0_1ms: 'CF106', P_1ms: 'CF107',
  P_First_Cycle: 'A200.11', P_Step: 'A200.12', P_First_Cycle_Task: 'A200.15',
  P_Cycle_Time_Error: 'A401.08', P_Low_Battery: 'A402.04', P_IO_Verify_Error: 'A402.09',
  P_Output_Off_Bit: 'A500.15', P_Max_Cycle_Time: 'A262', P_Cycle_Time_Value: 'A264',
});

const ADDR_RE = /^(?:(W|H|A|D|TR|CF|T|C)|E(\d+)_)?(\d+)(?:\.(\d+))?$/;

/**
 * An ADDRESS string as the symbol tables and the mnemonic write it: 3205.07, W450.08, H1,
 * D5046, D6067.00, E0_3.10, T0450, C470, TR0, CF113.
 * @param {string} s @returns {Operand|null}
 */
export function parseAddr(s) {
  const m = ADDR_RE.exec(s);
  if (!m) return null;
  const area = m[1] ?? (m[2] !== undefined ? 'E' + (+m[2]) : 'CIO');
  const word = +m[3], bit = m[4] !== undefined ? +m[4] : -1;
  if (bit > 15) return null;
  if ((area === 'TR' || area === 'CF') && bit >= 0) return null;
  return { area, word, bit, text: s };
}

/**
 * @typedef {{name: string, addr: string, type: string, comment: string}} Sym
 * @typedef {{find: (name: string) => Sym|undefined}} Scope   locals first, then globals
 */

/** Words an array element or a typed symbol spans. @param {string} type */
const dimOf = type => { const m = /\[(\d+)\]$/.exec(type); return m ? +m[1] : 0; };

/**
 * One operand of a mnemonic line.
 * @param {string} s the operand text
 * @param {string} kind the instruction's operand kind (b w l t c n)
 * @param {Scope} [scope]
 * @returns {Operand}
 */
export function parseOperand(s, kind, scope) {
  // Constants: #hex, &decimal, +/-decimal (signed), and for TIM/CNT a bare number is the
  // timer/counter NUMBER - everywhere else a bare number is a CIO address.
  if (/^#[0-9A-Fa-f]+$/.test(s)) return { area: 'const', word: 0, bit: -1, value: parseInt(s.slice(1), 16) >>> 0, text: s };
  if (/^&\d+$/.test(s)) return { area: 'const', word: 0, bit: -1, value: +s.slice(1) >>> 0, text: s };
  if (/^[+-]\d+$/.test(s)) return { area: 'const', word: 0, bit: -1, value: (+s) >>> 0, text: s };
  if ((kind === 't' || kind === 'c') && /^\d+$/.test(s)) return { area: kind === 't' ? 'T' : 'C', word: +s, bit: -1, text: s };
  if (/^@D\d+$/.test(s)) return { area: 'ind', word: 0, bit: -1, ind: { area: 'D', word: +s.slice(2) }, text: s };
  const a = parseAddr(s);
  if (a) return a;
  // A symbol, possibly an array element.
  const m = /^([A-Za-z_][A-Za-z0-9_]*)(?:\[(\d+)\])?$/.exec(s);
  if (!m) throw new Error('operand "' + s + '" is not an address, a constant or a symbol');
  const sys = SYSTEM[m[1]];
  const sym = scope?.find(m[1]);
  const addr = sym?.addr ?? sys;
  if (!addr) throw new Error('symbol "' + m[1] + '" is not declared');
  const r = parseAddr(addr);
  if (!r) throw new Error('symbol "' + m[1] + '" has an address this soft-PLC cannot read: ' + addr);
  if (m[2] !== undefined) {
    const dim = dimOf(sym?.type ?? '');
    if (!dim) throw new Error('"' + m[1] + '" is not an array');
    if (+m[2] >= dim) throw new Error('"' + s + '" is past the end of ' + m[1] + ' (' + dim + ' elements)');
    r.word += +m[2];
  }
  return { ...r, name: s, text: s };
}

/** Canonical text of an address, for monitoring keys: 'W450.08', 'D5046', 'T0450', '3205.07'. @param {Operand} o */
export function addrText(o) {
  if (o.area === 'const') return o.text;
  if (o.area === 'ind') return '@D' + o.ind?.word;
  const p = o.area === 'CIO' ? '' : o.area.startsWith('E') ? o.area + '_' : o.area;
  const w = o.area === 'T' || o.area === 'C' ? String(o.word).padStart(4, '0') : String(o.word);
  return p + w + (o.bit >= 0 ? '.' + String(o.bit).padStart(2, '0') : '');
}

// --------------------------------------------------------------------------- the network

/**
 * What a viewer draws. A block is the logic in front of the outputs; a node is a condition
 * followed by what hangs off it (outputs, and further conditions with their own outputs, as
 * CX-Programmer draws `LD a  OUT b  AND c  OUT d`).
 * @typedef {{t: 'c', i: number} | {t: 's', a: Block[]} | {t: 'p', a: Block[]}} Block
 * @typedef {{cond: Block|null, outs: Array<{t: 'o', i: number, ins?: Block[]} | Node>, tr?: boolean}} Node
 */

/** @param {Block} a @param {Block} b @returns {Block} */
const series = (a, b) => ({ t: 's', a: [...(a.t === 's' ? a.a : [a]), ...(b.t === 's' ? b.a : [b])] });
/** @param {Block} a @param {Block} b @returns {Block} */
const parallel = (a, b) => ({ t: 'p', a: [...(a.t === 'p' ? a.a : [a]), ...(b.t === 'p' ? b.a : [b])] });

/**
 * The rung as a drawable network, or {error} when the mnemonic is not something CX-Programmer
 * could have drawn. Instruction indices point into `lines`.
 * @param {Line[]} lines
 * @returns {{roots: Node[], error?: string}}
 */
export function network(lines) {
  /** @typedef {{b: Block|null, n: Node|null}} Ent   b: logic not yet anchored; n: the node outputs hang off */
  /** @type {Ent[]} */
  const st = [];
  /** @type {Node[]} */
  const roots = [];
  /** @type {Record<string, Ent>} */
  const tr = {};
  const top = () => st[st.length - 1];
  /** Extend an entry with more logic: into its node when nothing hangs off it yet, else a new branch. @param {Ent} e @param {Block} b */
  const andInto = (e, b) => {
    if (!e.n) { e.b = e.b ? series(e.b, b) : b; return; }
    if (!e.n.outs.length && !e.n.tr) { e.n.cond = e.n.cond ? series(e.n.cond, b) : b; return; }
    const child = { cond: b, outs: [] };
    e.n.outs.push(child);
    e.n = child;
  };
  try {
    lines.forEach((ln, i) => {
      const k = ln.def.k;
      if (k === 'nop') return;
      if (k === 'ld') {
        if (ln.args[0]?.startsWith('TR')) { const s = tr[ln.args[0]]; if (!s) throw new Error(ln.args[0] + ' read before it is set'); st.push({ b: s.b, n: s.n }); return; }
        st.push({ b: { t: 'c', i }, n: null });
        return;
      }
      if (k === 'and') { if (!st.length) throw new Error('AND with nothing loaded'); andInto(top(), { t: 'c', i }); return; }
      if (k === 'or') {
        const e = top();
        if (!e) throw new Error('OR with nothing loaded');
        if (e.n) throw new Error('OR after an output cannot be drawn');
        e.b = parallel(/** @type {Block} */ (e.b), { t: 'c', i });
        return;
      }
      if (k === 'andld' || k === 'orld') {
        const b = st.pop(), a = top();
        if (!a || !b || !b.b || b.n) throw new Error(ln.name + ' without two blocks');
        if (k === 'orld') { if (a.n) throw new Error('ORLD after an output cannot be drawn'); a.b = parallel(/** @type {Block} */ (a.b), b.b); }
        else andInto(a, b.b);
        return;
      }
      if (k === 'conn') { if (!st.length) throw new Error(ln.name + ' with nothing loaded'); andInto(top(), { t: 'c', i }); return; }
      if (k === 'out') {
        if (ln.name === 'OUT' && ln.args[0]?.startsWith('TR')) {           // branch point: remember where we are
          const e = top();
          if (!e) throw new Error('OUT TR with nothing loaded');
          anchor(e);
          /** @type {Node} */ (e.n).tr = true;                  // what follows is a branch, not more of this condition
          tr[ln.args[0]] = { b: null, n: e.n };
          return;
        }
        const pops = ln.def.pops ?? 1;
        if (st.length < pops) throw new Error(ln.name + ' needs ' + pops + ' conditions');
        if (pops > 1) {
          // KEEP / CNT / CNTR: the extra inputs are drawn as their own lines into the box.
          const ins = st.splice(st.length - pops + 1).map(e => /** @type {Block} */ (e.b));
          const e = top();
          anchor(e);
          /** @type {Node} */ (e.n).outs.push({ t: 'o', i, ins });
          return;
        }
        const e = top();
        anchor(e);
        /** @type {Node} */ (e.n).outs.push({ t: 'o', i });
        return;
      }
      if (k === 'il' || k === 'ilc' || k === 'end') {
        if (k === 'il') { const e = top(); if (!e) throw new Error('IL with nothing loaded'); anchor(e); /** @type {Node} */ (e.n).outs.push({ t: 'o', i }); return; }
        roots.push({ cond: null, outs: [{ t: 'o', i }] });
        return;
      }
      throw new Error('unknown instruction ' + ln.src);
    });
  } catch (e) {
    return { roots: [], error: /** @type {Error} */ (e).message };
  }
  return { roots };

  /** Make the entry's logic a node hanging off the rail (the first output does it). @param {Ent} e */
  function anchor(e) {
    if (e.n) return;
    const n = { cond: e.b, outs: [] };
    roots.push(n);
    e.n = n; e.b = null;
  }
}

/**
 * Parses and checks a whole rung the way the soft-PLC will compile it: every instruction known,
 * every operand resolvable, the stack balanced. Returns the lines and a list of problems.
 * @param {string[]} il @param {Scope} [scope]
 * @returns {{lines: Line[], ops: Operand[][], errors: string[]}}
 */
export function checkRung(il, scope) {
  const lines = il.map(parseLine);
  /** @type {string[]} */
  const errors = [];
  const ops = lines.map((ln, i) => {
    if (ln.def.k === 'bad') { errors.push('line ' + (i + 1) + ': unknown instruction ' + ln.name + (ln.fn ? '(' + ln.fn + ')' : '')); return []; }
    const kinds = ln.def.ops ?? [];
    if (ln.def.k !== 'nop' && ln.args.length !== kinds.length) {
      errors.push('line ' + (i + 1) + ': ' + ln.name + ' takes ' + kinds.length + ' operand(s), got ' + ln.args.length);
      return [];
    }
    return ln.args.map((a, j) => {
      if (/^TR\d+$/.test(a)) return /** @type {Operand} */ ({ area: 'TR', word: +a.slice(2), bit: -1, text: a });
      try { return parseOperand(a, kinds[j], scope); } catch (e) { errors.push('line ' + (i + 1) + ': ' + /** @type {Error} */ (e).message); return /** @type {any} */ (null); }
    });
  });
  let depth = 0;
  lines.forEach((ln, i) => {
    const k = ln.def.k;
    if (k === 'ld') depth++;
    else if (k === 'andld' || k === 'orld') { depth--; if (depth < 1) errors.push('line ' + (i + 1) + ': ' + ln.name + ' without two blocks'); }
    else if ((k === 'and' || k === 'or' || k === 'conn' || k === 'out' || k === 'il') && depth < 1) errors.push('line ' + (i + 1) + ': ' + ln.name + ' with nothing loaded');
    if (k === 'out' && (ln.def.pops ?? 1) > 1) { depth -= (ln.def.pops ?? 1) - 1; if (depth < 1) errors.push('line ' + (i + 1) + ': ' + ln.name + ' needs ' + ln.def.pops + ' conditions'); }
  });
  return { lines, ops, errors };
}
