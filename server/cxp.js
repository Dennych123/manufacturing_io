// @ts-check
// CX-Programmer project files (.cxp): read, edit, write. Node only (Buffer, fs).
//
// A .cxp is ONE PKWARE Data Compression Library stream ("TTComp archive", the implode that
// predates zip's deflate): header byte 0 = binary literals, byte 1 = dictionary bits (6: 4 KB).
// Inside is plain text, CX-Programmer's own serialisation (measured on files written by CX-P
// 9.82 and 9.88, CXProgVer 2.3):
//
//   Key:=value;                       a scalar, the value exactly as written (quotes included)
//   Key:=\r\n BEGIN ... END;          a block, children indented one space deeper
//   Key:=\r\n$?St$Bk?_#[n]\r\n...$?St$Bk?_#[n]\r\n
//                                     a raw string (rung mnemonic, rung comment, hex settings)
//                                     that can span lines; its markers sit in column 0
//   Key:=\r\n BEGIN_LIST_$#[n] ... END_LIST_$#[n];
//                                     a list of lines (the symbol tables)
//
// Nothing here is documented by Omron. The rule the parser keeps is the one that makes it safe
// to write back: serialize(parse(text)) is byte-identical to text, on every file it accepts.
// tests/cxp.test.js pins that, so an edit changes the rungs it touched and nothing else.

// --------------------------------------------------------------------------- DCL explode / implode
// Ported from zlib contrib/blast/blast.c (Mark Adler), which decodes this format; the encoder
// writes the same codes the other way round.

const LITLEN = [11, 124, 8, 7, 28, 7, 188, 13, 76, 4, 10, 8, 12, 10, 12, 10, 8, 23, 8, 9, 7, 6, 7, 8, 7, 6, 55, 8, 23, 24,
  12, 11, 7, 9, 11, 12, 6, 7, 22, 5, 7, 24, 6, 11, 9, 6, 7, 22, 7, 11, 38, 7, 9, 8, 25, 11, 8, 11, 9, 12, 8, 12, 5, 38, 5,
  38, 5, 11, 7, 5, 6, 21, 6, 10, 53, 8, 7, 24, 10, 27, 44, 253, 253, 253, 252, 252, 252, 13, 12, 45, 12, 45, 12, 61, 12,
  45, 44, 173];
const LENLEN = [2, 35, 36, 53, 38, 23];
const DISTLEN = [2, 20, 53, 230, 247, 151, 248];
const BASE = [3, 2, 4, 5, 6, 7, 8, 9, 10, 12, 16, 24, 40, 72, 136, 264];
const EXTRA = [0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8];

/** Canonical Huffman table from blast's compact run-length form. @param {number[]} rep */
function huff(rep) {
  /** @type {number[]} */
  const length = [];
  for (const b of rep) for (let i = (b >> 4) + 1; i > 0; i--) length.push(b & 15);
  const count = new Array(14).fill(0);
  for (const l of length) count[l]++;
  const offs = new Array(14).fill(0);
  for (let len = 1; len < 13; len++) offs[len + 1] = offs[len] + count[len];
  /** @type {number[]} */
  const symbol = [];
  for (let s = 0; s < length.length; s++) if (length[s]) symbol[offs[length[s]]++] = s;
  // Encoder side: the canonical code of each symbol, MSB first.
  const code = new Array(length.length).fill(0);
  let c = 0;
  for (let len = 1; len <= 13; len++) {
    for (let s = 0; s < length.length; s++) if (length[s] === len) code[s] = c++;
    c <<= 1;
  }
  return { count, symbol, code, length };
}
const H_LIT = huff(LITLEN), H_LEN = huff(LENLEN), H_DIST = huff(DISTLEN);

/** @param {Uint8Array} buf a .cxp file @returns {Buffer} the text inside */
export function explode(buf) {
  let pos = 0, bitbuf = 0, bitcnt = 0;
  const bits = (/** @type {number} */ n) => {
    while (bitcnt < n) {
      if (pos >= buf.length) throw new Error('cxp: compressed stream ends early');
      bitbuf |= buf[pos++] << bitcnt; bitcnt += 8;
    }
    const v = bitbuf & ((1 << n) - 1);
    bitbuf >>>= n; bitcnt -= n;
    return v;
  };
  const decode = (/** @type {{count: number[], symbol: number[]}} */ h) => {
    let code = 0, first = 0, index = 0;
    for (let len = 1; len <= 13; len++) {
      code |= bits(1) ^ 1;                           // PKWARE sends its codes bit-inverted
      const count = h.count[len];
      if (code < first + count) return h.symbol[index + (code - first)];
      index += count; first += count; first <<= 1; code <<= 1;
    }
    throw new Error('cxp: bad Huffman code');
  };
  const lit = bits(8), dict = bits(8);
  if (lit > 1 || dict < 4 || dict > 6) throw new Error('cxp: not a PKWARE DCL stream (header ' + lit + ',' + dict + ')');
  let out = Buffer.alloc(Math.max(1 << 16, buf.length * 8)), n = 0;
  const put = (/** @type {number} */ b) => {
    if (n === out.length) { const o = Buffer.alloc(out.length * 2); out.copy(o); out = o; }
    out[n++] = b;
  };
  for (;;) {
    if (bits(1)) {
      const sym = decode(H_LEN);
      const len = BASE[sym] + bits(EXTRA[sym]);
      if (len === 519) break;                        // end of stream
      const s = len === 2 ? 2 : dict;
      const dist = (decode(H_DIST) << s) + bits(s) + 1;
      if (dist > n) throw new Error('cxp: distance past the start');
      for (let i = 0; i < len; i++) put(out[n - dist]);
    } else put(lit ? decode(H_LIT) : bits(8));
  }
  return out.subarray(0, n);
}

/**
 * Compresses text the way CX-Programmer's own files are: binary literals, 4 KB dictionary.
 * Greedy LZ77 over a hash chain; explode(implode(x)) is x (tests/cxp.test.js).
 * @param {Uint8Array} data @returns {Buffer}
 */
export function implode(data) {
  const DICT = 6, WIN = 64 << DICT, MAXLEN = 518, CHAIN = 48;
  /** @type {number[]} */
  const out = [0, DICT];
  let acc = 0, nacc = 0;
  const put = (/** @type {number} */ v, /** @type {number} */ n) => {            // n bits of v, LSB first
    for (let i = 0; i < n; i++) { acc |= ((v >> i) & 1) << nacc; if (++nacc === 8) { out.push(acc); acc = 0; nacc = 0; } }
  };
  const putCode = (/** @type {{code: number[], length: number[]}} */ h, /** @type {number} */ s) => {
    const c = h.code[s], l = h.length[s];
    for (let i = l - 1; i >= 0; i--) put(((c >> i) & 1) ^ 1, 1);                // MSB first, inverted
  };
  const lenSym = (/** @type {number} */ len) => {
    for (let s = 0; s < 16; s++) if (len >= BASE[s] && len < BASE[s] + (1 << EXTRA[s])) return s;
    throw new Error('cxp: length ' + len);
  };
  const head = new Int32Array(1 << 15).fill(-1), prev = new Int32Array(data.length).fill(-1);
  const hash = (/** @type {number} */ i) => ((data[i] << 10) ^ (data[i + 1] << 5) ^ data[i + 2]) & 0x7fff;
  const insert = (/** @type {number} */ i) => { if (i + 2 < data.length) { const h = hash(i); prev[i] = head[h]; head[h] = i; } };
  let i = 0;
  while (i < data.length) {
    let bestLen = 0, bestDist = 0;
    if (i + 2 < data.length) {
      for (let j = head[hash(i)], k = 0; j >= 0 && i - j <= WIN && k < CHAIN; j = prev[j], k++) {
        let l = 0;
        const max = Math.min(MAXLEN, data.length - i);
        while (l < max && data[j + l] === data[i + l]) l++;
        if (l > bestLen) { bestLen = l; bestDist = i - j; if (l === max) break; }
      }
    }
    if (bestLen >= 3 || (bestLen === 2 && bestDist <= 256)) {
      if (bestLen === 2 && bestDist > 256) bestLen = 0;
      put(1, 1);
      const s = lenSym(bestLen);
      putCode(H_LEN, s);
      put(bestLen - BASE[s], EXTRA[s]);
      const sh = bestLen === 2 ? 2 : DICT, d = bestDist - 1;
      putCode(H_DIST, d >> sh);
      put(d & ((1 << sh) - 1), sh);
      for (let k = 0; k < bestLen; k++) insert(i + k);
      i += bestLen;
    } else {
      put(0, 1); put(data[i], 8);
      insert(i);
      i++;
    }
  }
  put(1, 1); putCode(H_LEN, 15); put(255, 8);                                    // length 519: end
  if (nacc) out.push(acc);
  return Buffer.from(out);
}

// --------------------------------------------------------------------------- the text tree

/**
 * @typedef {{t: 's', raw: string}} Scalar      the value between ':=' and the final ';'
 * @typedef {{t: 'r', id: number, text: string}} Raw
 * @typedef {{t: 'l', id: number, text: string}} List   the lines between the list markers, verbatim
 * @typedef {{t: 'b', kids: Entry[]}} Block
 * @typedef {{k: string, v: Scalar|Raw|List|Block}} Entry
 */

const RAW_RE = /^\$\?St\$Bk\?_#\[(\d+)\]$/;
const LIST_RE = /^ *BEGIN_LIST_\$#\[(\d+)\]$/;

/** @param {string} text the decompressed file, read as latin1 so every byte survives @returns {Entry[]} */
export function parseText(text) {
  let i = 0;
  const line = () => {
    const e = text.indexOf('\r\n', i);
    if (e < 0) throw new Error('cxp: line without CRLF at ' + i);
    const s = text.slice(i, e);
    i = e + 2;
    return s;
  };
  /** @param {number} depth @returns {Entry[]} */
  const block = depth => {
    /** @type {Entry[]} */
    const kids = [];
    const pad = ' '.repeat(depth);
    for (;;) {
      if (i >= text.length) { if (depth === 0) return kids; throw new Error('cxp: file ends inside a block'); }
      const s = line();
      if (depth > 0 && s === ' '.repeat(depth - 1) + 'END;') return kids;
      if (!s.startsWith(pad) || s[depth] === ' ') throw new Error('cxp: bad indent at ' + i + ': ' + s.slice(0, 60));
      const at = s.indexOf(':=');
      if (at < 0) throw new Error('cxp: no := at ' + i + ': ' + s.slice(0, 60));
      const k = s.slice(depth, at), rest = s.slice(at + 2);
      if (rest !== '') {
        if (!rest.endsWith(';')) throw new Error('cxp: scalar without ; at ' + i);
        kids.push({ k, v: { t: 's', raw: rest.slice(0, -1) } });
        continue;
      }
      const nx = line();
      if (nx === pad + 'BEGIN') { kids.push({ k, v: { t: 'b', kids: block(depth + 1) } }); continue; }
      let m = RAW_RE.exec(nx);
      if (m) {
        const mark = '$?St$Bk?_#[' + m[1] + ']';
        const e = text.indexOf(mark, i);
        if (e < 0) throw new Error('cxp: raw string ' + m[1] + ' never closes');
        kids.push({ k, v: { t: 'r', id: +m[1], text: text.slice(i, e) } });
        i = e + mark.length;
        if (text.slice(i, i + 2) !== '\r\n') throw new Error('cxp: raw string ' + m[1] + ' not followed by CRLF');
        i += 2;
        continue;
      }
      m = LIST_RE.exec(nx);
      if (m && nx.startsWith(pad + 'BEGIN_LIST')) {
        const mark = '\r\n' + pad + 'END_LIST_$#[' + m[1] + '];\r\n';
        const e = text.indexOf(mark, i - 2);
        if (e < 0) throw new Error('cxp: list ' + m[1] + ' never closes');
        kids.push({ k, v: { t: 'l', id: +m[1], text: text.slice(i, Math.max(i, e + 2)) } });
        i = e + mark.length;
        continue;
      }
      throw new Error('cxp: cannot read the value of ' + k + ' at ' + i);
    }
  };
  return block(0);
}

/** @param {Entry[]} kids @returns {string} */
export function serializeText(kids) {
  /** @type {string[]} */
  const out = [];
  /** @param {Entry[]} list @param {number} depth */
  const block = (list, depth) => {
    const pad = ' '.repeat(depth);
    for (const { k, v } of list) {
      if (v.t === 's') { out.push(pad, k, ':=', v.raw, ';\r\n'); continue; }
      out.push(pad, k, ':=\r\n');
      if (v.t === 'b') { out.push(pad, 'BEGIN\r\n'); block(v.kids, depth + 1); out.push(pad, 'END;\r\n'); }
      else if (v.t === 'r') out.push('$?St$Bk?_#[', String(v.id), ']\r\n', v.text, '$?St$Bk?_#[', String(v.id), ']\r\n');
      else out.push(pad, 'BEGIN_LIST_$#[', String(v.id), ']\r\n', v.text, pad, 'END_LIST_$#[', String(v.id), '];\r\n');
    }
  };
  block(kids, 0);
  return out.join('');
}

// --------------------------------------------------------------------------- the project model

/** @param {Entry[]|undefined} kids @param {string} k */
const get = (kids, k) => kids?.find(e => e.k === k)?.v;
/** @param {Entry[]|undefined} kids @param {string} k @returns {Entry[]|undefined} */
const blk = (kids, k) => { const v = get(kids, k); return v && v.t === 'b' ? v.kids : undefined; };
/** A scalar's value: quotes removed. @param {any} v */
const str = v => (v?.t === 's' ? (v.raw.startsWith('"') && v.raw.endsWith('"') ? v.raw.slice(1, -1) : v.raw) : v?.t === 'r' ? v.text : '');

/** Splits one symbol-table line (CSV with "quotes", ending in ';'). @param {string} s */
function csv(s) {
  s = s.replace(/;$/, '');
  /** @type {string[]} */
  const f = [];
  let cur = '', q = false;
  for (let i = 0; i < s.length; i++) {
    const c = s[i];
    if (q) { if (c === '"') { if (s[i + 1] === '"') { cur += '"'; i++; } else q = false; } else cur += c; }
    else if (c === '"') q = true;
    else if (c === ',') { f.push(cur); cur = ''; }
    else cur += c;
  }
  f.push(cur);
  return f;
}

/**
 * @typedef {{name: string, addr: string, type: string, comment: string}} Sym
 * @typedef {{comment: string, il: string[], notes: string[]}} Rung   il: the mnemonic lines, `'` notes kept verbatim
 * @typedef {{name: string, rungs: Rung[]}} Section
 * @typedef {{name: string, comment: string, taskId: number, locals: Sym[], sections: Section[]}} Program
 * @typedef {{name: string, device: string, cpu: string, globals: Sym[], programs: Program[]}} Project
 */

/** @param {List|undefined} v @returns {Sym[]} */
function symbols(v) {
  if (!v || v.t !== 'l' || !v.text) return [];
  return v.text.split('\r\n').filter(Boolean).map(l => {
    const f = csv(l.trim());
    return { name: f[0], addr: f[1], type: f[2], comment: f[6] ?? '' };
  });
}

/** @param {Entry[]} tree @returns {Project} */
export function projectOf(tree) {
  const res = blk(blk(tree, 'Resource[0]'), 'Resource[0]') ?? blk(tree, 'Resource[0]');
  const plc = blk(res, 'PLC');
  const cfg = str(get(plc, 'Config'));
  const gv = blk(res, 'GlobalVariables');
  /** @type {Program[]} */
  const programs = [];
  const progs = blk(res, 'Programs') ?? [];
  for (const e of progs) {
    if (!e.k.startsWith('Program[') || e.v.t !== 'b') continue;
    const p = e.v.kids;
    const lv = blk(p, 'LocalVariables');
    /** @type {Section[]} */
    const sections = [];
    for (const s of blk(p, 'Sections') ?? []) {
      if (!s.k.startsWith('Sec[') || s.v.t !== 'b') continue;
      const pd = blk(s.v.kids, 'ProgramData') ?? [];
      /** @type {Rung[]} */
      const rungs = [];
      for (const r of pd) {
        if (!r.k.startsWith('R[') || r.v.t !== 'b') continue;
        const sl = str(get(r.v.kids, 'SL'));
        const lines = sl.split(/\r?\n/).map(x => x.trim()).filter(Boolean);
        rungs.push({ comment: str(get(r.v.kids, 'Com')).replace(/\r?\n/g, '\n'),
                     il: lines.filter(x => !x.startsWith("'")), notes: lines.filter(x => x.startsWith("'")) });
      }
      sections.push({ name: str(get(s.v.kids, 'SecName')), rungs });
    }
    programs.push({ name: str(get(p, 'Name')), comment: str(get(p, 'Comment')), taskId: +str(get(p, 'TaskID')),
                    locals: symbols(/** @type {any} */ (get(lv, 'VariableList'))), sections });
  }
  return {
    name: str(get(plc, 'Name')),
    device: /DEV:([^;]+)/.exec(cfg)?.[1] ?? '', cpu: /CPU:([^;]+)/.exec(cfg)?.[1] ?? '',
    globals: symbols(/** @type {any} */ (get(gv, 'VariableList'))), programs,
  };
}

/** @param {Uint8Array} file a .cxp @returns {{tree: Entry[], project: Project}} */
export function readCxp(file) {
  const text = explode(file).toString('latin1');
  const tree = parseText(text);
  if (serializeText(tree) !== text) throw new Error('cxp: this file does not round-trip; refusing to edit it');
  return { tree, project: projectOf(tree) };
}

/**
 * Writes the project's rungs back into the tree it came from and returns the new .cxp bytes.
 * Only ProgramData (rung count, rung blocks) is rewritten; everything else - settings, IO table,
 * symbols, task assignment - is the original text byte for byte. A rung that did not change
 * keeps its original raw strings exactly.
 * @param {Entry[]} tree @param {Project} project @returns {Buffer}
 */
export function writeCxp(tree, project) {
  let nextId = 0;
  /** @param {Entry[]} kids */
  const scan = kids => { for (const { v } of kids) { if (v.t === 'r' || v.t === 'l') nextId = Math.max(nextId, v.id + 1); if (v.t === 'b') scan(v.kids); } };
  scan(tree);
  const res = blk(tree, 'Resource[0]');
  const progs = (blk(res, 'Programs') ?? []).filter(e => e.k.startsWith('Program[') && e.v.t === 'b');
  if (progs.length !== project.programs.length) throw new Error('cxp: adding or removing programs is not supported');
  progs.forEach((e, pi) => {
    const P = project.programs[pi];
    const secs = (blk(/** @type {Block} */ (e.v).kids, 'Sections') ?? []).filter(s => s.k.startsWith('Sec[') && s.v.t === 'b');
    if (secs.length !== P.sections.length) throw new Error('cxp: adding or removing sections is not supported (' + P.name + ')');
    secs.forEach((s, si) => {
      const pd = /** @type {Block} */ (get(/** @type {Block} */ (s.v).kids, 'ProgramData'));
      const old = pd.kids.filter(r => r.k.startsWith('R[')).map(r => /** @type {Block} */ (r.v).kids);
      const rungs = P.sections[si].rungs;
      /** @type {Entry[]} */
      const kids = [{ k: 'RC', v: { t: 's', raw: String(rungs.length) } }];
      // An unchanged rung keeps its own block. Identical rungs exist (the same interlock written
      // twice), so each old block is used once, and the one at the same position first.
      const used = new Set();
      rungs.forEach((r, ri) => {
        const body = [...r.notes, ...r.il];
        let oi = ri < old.length && sameRung(old[ri], r) ? ri : -1;
        if (oi < 0 || used.has(oi)) oi = old.findIndex((k, j) => !used.has(j) && sameRung(k, r));
        if (oi >= 0) { used.add(oi); kids.push({ k: 'R[' + ri + ']', v: { t: 'b', kids: old[oi] } }); return; }
        const com = r.comment ? { t: 'r', id: nextId++, text: r.comment.replace(/\r?\n/g, '\n') } : { t: 's', raw: '""' };
        const sl = body.length ? { t: 'r', id: nextId++, text: body.join('\n') + '\n' } : { t: 's', raw: '""' };
        kids.push({ k: 'R[' + ri + ']', v: { t: 'b', kids: [
          { k: 'Com', v: /** @type {any} */ (com) }, { k: 'Flags', v: { t: 's', raw: '"1,0"' } },
          { k: 'FBversion', v: { t: 's', raw: '""' } }, { k: 'SL', v: /** @type {any} */ (sl) },
          { k: 'AtchCmts', v: { t: 'b', kids: [{ k: 'CC', v: { t: 's', raw: '0' } }] } },
        ] } });
      });
      pd.kids = kids;
    });
  });
  return implode(Buffer.from(serializeText(tree), 'latin1'));
}

/** @param {Entry[]} kids an original rung block @param {Rung} r */
function sameRung(kids, r) {
  const sl = str(get(kids, 'SL')).split(/\r?\n/).map(x => x.trim()).filter(Boolean);
  const com = str(get(kids, 'Com')).replace(/\r?\n/g, '\n');
  const want = [...r.notes, ...r.il];
  return com === r.comment && sl.length === want.length && sl.every((x, i) => x === want[i]);
}
