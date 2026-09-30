// The ladder soft-PLC's program, drawn as ladder and monitored live: /ladder.
// The page only draws, edits and asks. The program runs in the plant (server/ladder.js), in the
// same step as the machine; this page reads values with /api/ladder/peek and writes only through
// the PLC's own operations - force, set, online edit - so a value it shows is the value the
// program saw.
import { checkRung, network, addrText, parseOperand } from '/lib/ladder.js';

const $ = (/** @type {string} */ id) => /** @type {HTMLElement} */ (document.getElementById(id));
const CW = 104, RH = 58, WY = 40, RAIL = 12, OUTW = 2.7, POLL_MS = 150;
const esc = (/** @type {any} */ s) => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c] ?? c);
const clip = (/** @type {string} */ s, /** @type {number} */ n) => (s.length > n ? s.slice(0, n - 1) + '…' : s);
const store = { get(/** @type {string} */ k, /** @type {any} */ d) { try { const v = localStorage.getItem(k); return v ? JSON.parse(v) : d; } catch { return d; } },
                set(/** @type {string} */ k, /** @type {any} */ v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { /* private window */ } } };

/** @type {any} */
let P = null;
/** @type {Array<{find: (n: string) => any}>} */
let scopes = [];
/** @type {Map<string, any>} */
let G = new Map();
/** address -> the symbol (with a comment) that names it: IO lists often have no name, only a comment. */
let byAddr = new Map();
let cur = { prog: 0, sec: 0 };
/** Rendered rungs of the current section: monitor entries per rung. @type {Array<{el: HTMLElement, mons: any[]}>} */
let rendered = [];
const visible = new Set();
let forced = new Set();
/** @type {Array<{label: string, keys: string[]}>} */
let watch = [];
/** @type {Array<{tag: string, addr: string, dir: string, key: string, tr?: HTMLElement}>} */
let ioRows = [];
/** @type {Map<string, Array<{pi: number, si: number, ri: number, line: string}>>|null} */
let xindex = null;
/** Latest values by key. */
let vals = new Map();

function toast(/** @type {string} */ msg, ms = 2500) {
  const t = $('toast');
  t.textContent = msg; t.style.display = 'block';
  clearTimeout(/** @type {any} */ (t).tm);
  /** @type {any} */ (t).tm = setTimeout(() => { t.style.display = 'none'; }, ms);
}
async function post(/** @type {string} */ url, /** @type {any} */ body) {
  const r = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const d = await r.json().catch(() => ({}));
  if (!r.ok && !d.errors) throw new Error(d.error || r.statusText);
  return d;
}

// ------------------------------------------------------------------ loading
async function load() {
  const r = await fetch('/api/ladder');
  if (!r.ok) { $('rungs').innerHTML = '<p>' + esc((await r.json().catch(() => ({}))).error || r.statusText) + '</p><p><a href="/">back to the 3D view</a></p>'; return; }
  P = await r.json();
  G = new Map(P.globals.filter((/** @type {any} */ g) => g.name).map((/** @type {any} */ g) => [g.name, g]));
  byAddr = new Map();
  for (const g of P.globals) if (g.comment && g.comment !== '.' && !byAddr.has(g.addr)) byAddr.set(g.addr, g);
  scopes = P.programs.map((/** @type {any} */ p) => {
    const L = new Map(p.locals.filter((/** @type {any} */ s) => s.name).map((/** @type {any} */ s) => [s.name, s]));
    return { find: (/** @type {string} */ n) => L.get(n) ?? G.get(n) };
  });
  xindex = null;
  watch = store.get('mio.ladder.watch.' + P.name, []);
  $('proj').textContent = P.name + '  ' + P.device + ' ' + P.cpu;
  const f = /** @type {HTMLSelectElement} */ ($('file'));
  f.innerHTML = P.files.map((/** @type {string} */ x) => '<option' + (x === P.file ? ' selected' : '') + '>' + esc(x) + '</option>').join('');
  $('errs').textContent = P.errors.length ? P.errors.length + ' rung(s) do not compile and are NOT running:\n'
    + P.errors.map((/** @type {any} */ e) => P.programs[e.prog]?.name + ' / ' + (P.programs[e.prog]?.sections[e.sec]?.name ?? '?') + ' R' + e.rung + ': ' + e.msg).join('\n') : '';
  modeButton(P.running);
  tree();
  await ioMap();
  renderWatch();
  const h = new URLSearchParams(location.hash.slice(1));
  const pi = +(h.get('p') ?? 0), si = +(h.get('s') ?? 0);
  cur = { prog: P.programs[pi] ? pi : 0, sec: P.programs[pi]?.sections[si] ? si : 0 };
  showSection(cur.prog, cur.sec, h.has('r') ? +(/** @type {string} */ (h.get('r'))) : -1);
}

function modeButton(/** @type {boolean} */ run) {
  const b = $('mode');
  b.textContent = run ? 'RUN' : 'PROGRAM';
  b.className = run ? 'run' : 'prog';
  b.title = run ? 'Running. Click for PROGRAM mode (scan stops, outputs OFF).' : 'Stopped. Click to RUN.';
}

function tree() {
  const errSec = new Set(P.errors.map((/** @type {any} */ e) => e.prog + '/' + e.sec));
  $('tree').innerHTML = P.programs.map((/** @type {any} */ p, pi) => '<div class="prog" data-p="' + pi + '">' + esc(p.name) + ' <small>' + esc(p.comment || '') + '</small></div>'
    + p.sections.map((/** @type {any} */ s, si) => '<div class="sec' + (errSec.has(pi + '/' + si) ? ' err' : '') + '" data-p="' + pi + '" data-s="' + si + '">' + esc(s.name)
      + '<span>' + s.rungs.filter((/** @type {any} */ r) => r.il.length).length + '</span></div>').join('')).join('');
  for (const el of $('tree').querySelectorAll('.sec')) el.addEventListener('click', () => showSection(+(/** @type {any} */ (el).dataset.p), +(/** @type {any} */ (el).dataset.s)));
}

// ------------------------------------------------------------------ drawing one rung
/**
 * The rung as SVG, and what to monitor on it. A rung that cannot be drawn is shown as its
 * mnemonic - it still runs; drawing is only the view.
 * @param {number} pi @param {string[]} il @param {boolean} live register monitor entries
 */
function draw(pi, il, live = true) {
  const scope = scopes[pi];
  const chk = checkRung(il, scope);
  if (chk.errors.length) return { html: '<div class="err">' + esc(chk.errors.join('\n')) + '</div><pre class="il">' + esc(il.join('\n')) + '</pre>', mons: [], errors: chk.errors };
  const net = network(chk.lines);
  if (net.error) return { html: '<div class="err">not drawable (' + esc(net.error) + ') - it runs as written:</div><pre class="il">' + esc(il.join('\n')) + '</pre>', mons: [], errors: [] };
  const L = chk.lines, O = chk.ops;
  /** @type {any[]} */
  const mons = [];
  /** @type {string[]} */
  const s = [];
  /** @param {any} b */
  const bw = b => (b.t === 'c' ? 1 : b.t === 's' ? b.a.reduce((/** @type {number} */ a, /** @type {any} */ x) => a + bw(x), 0) : Math.max(...b.a.map(bw)));
  /** @param {any} b */
  const bh = b => (b.t === 'c' ? 1 : b.t === 's' ? Math.max(...b.a.map(bh)) : b.a.reduce((/** @type {number} */ a, /** @type {any} */ x) => a + bh(x), 0));
  /** @param {any} n @returns {number} */
  const nw = n => (n.cond ? bw(n.cond) : 0) + Math.max(0, ...n.outs.filter((/** @type {any} */ o) => o.t !== 'o').map(nw));
  const TW = Math.max(1, ...net.roots.map(nw), ...net.roots.flatMap((/** @type {any} */ n) => n.outs.filter((/** @type {any} */ o) => o.ins).flatMap((/** @type {any} */ o) => o.ins.map(bw))));
  const X = (/** @type {number} */ c) => RAIL + c * CW, Y = (/** @type {number} */ r) => r * RH + WY;
  const wire = (/** @type {number} */ x1, /** @type {number} */ r, /** @type {number} */ x2) => { if (x2 > x1) s.push(`<path class="w" d="M${X(x1)} ${Y(r)}H${X(x2)}"/>`); };
  const vline = (/** @type {number} */ c, /** @type {number} */ r1, /** @type {number} */ r2) => { if (r2 > r1) s.push(`<path class="w" d="M${X(c)} ${Y(r1)}V${Y(r2)}"/>`); };
  /** Name as written, and a comment from the symbol or from whatever names that address. @param {number} i @param {number} j */
  const opInfo = (i, j) => {
    const o = O[i][j], text = L[i].args[j];
    const sym = o?.name ? scope.find(text.replace(/\[.*/, '')) : null;
    const a = o ? addrText(o) : text;
    const cm = (sym?.comment || byAddr.get(sym?.addr ?? a)?.comment || '').trim();
    return { text, addr: a, comment: cm === '.' ? '' : cm, o };
  };
  const keyOf = (/** @type {any} */ o, /** @type {string} */ k) => (!o || o.area === 'const' || o.area === 'ind' ? null : k + ':' + addrText(o));
  const reg = (/** @type {any} */ m) => { mons.push(m); return mons.length - 1; };

  /** A contact (or compare box, or UP/DOWN/NOT) in cell (c, r). @param {number} i @param {number} c @param {number} r */
  function contact(i, c, r) {
    const ln = L[i], x = X(c), y = Y(r);
    if (ln.def.k === 'conn') {
      s.push(`<path class="w" d="M${x} ${y}H${x + 26}M${x + CW - 26} ${y}H${x + CW}"/><rect class="box" x="${x + 26}" y="${y - 11}" width="${CW - 52}" height="22" rx="3"/>`
        + `<text x="${x + CW / 2}" y="${y + 4}" text-anchor="middle">${esc(ln.name)}</text>`);
      return;
    }
    if (ln.cmp) {
      const a = opInfo(i, 0), b = opInfo(i, 1), k = ln.cmp.long ? 'l' : 'w';
      const mi = reg({ kind: 'cmp', keys: [keyOf(a.o, k), keyOf(b.o, k)].filter(Boolean), a: a.o, b: b.o, k, op: ln.cmp.op, signed: ln.cmp.signed, info: [a, b], line: ln.src });
      s.push(`<g class="el" data-i="${mi}"><rect class="hl" x="${x + 4}" y="${y - 30}" width="${CW - 8}" height="${RH - 4}" rx="3"/>`
        + `<path class="w" d="M${x} ${y}H${x + 8}M${x + CW - 8} ${y}H${x + CW}"/><rect class="box" x="${x + 8}" y="${y - 24}" width="${CW - 16}" height="40" rx="3"/>`
        + `<text x="${x + 13}" y="${y - 11}">${esc(clip((ln.pre || '') + ln.cmp.op + (ln.cmp.signed ? 'S' : '') + (ln.cmp.long ? 'L' : '') + ' ' + a.text, 13))}</text>`
        + `<text x="${x + 13}" y="${y + 2}">${esc(clip(b.text, 13))}</text><text class="v" x="${x + 13}" y="${y + 13}" data-v="${mi}"></text>`
        + `<title>${esc(ln.src + '\n' + a.addr + ' ' + a.comment + '\n' + b.addr + ' ' + b.comment)}</title></g>`);
      return;
    }
    const a = opInfo(i, 0);
    const mi = reg({ kind: 'c', keys: [keyOf(a.o, 'b')].filter(Boolean), neg: !!ln.def.neg, info: [a], line: ln.src, addr: a.addr });
    const m = x + CW / 2;
    let sym = `<path class="w" d="M${x} ${y}H${m - 9}M${m - 9} ${y - 11}V${y + 11}M${m + 9} ${y - 11}V${y + 11}M${m + 9} ${y}H${x + CW}"/>`;
    if (ln.def.neg) sym += `<path class="w" d="M${m - 12} ${y + 10}L${m + 12} ${y - 10}"/>`;
    if (ln.pre) sym += `<text x="${m}" y="${y + 4}" text-anchor="middle">${ln.pre === '@' ? '↑' : '↓'}</text>`;
    s.push(`<g class="el" data-i="${mi}"><rect class="hl" x="${x + 2}" y="${y - 38}" width="${CW - 4}" height="${RH - 2}" rx="3"/>${sym}`
      + `<text class="c" x="${m}" y="${y - 27}" text-anchor="middle">${esc(clip(a.comment, 16))}</text>`
      + `<text x="${m}" y="${y - 15}" text-anchor="middle">${esc(clip(a.text, 14))}</text>`
      + `<title>${esc(ln.src + '\n' + a.addr + (a.comment ? '  ' + a.comment : ''))}</title></g>`);
  }
  /** @param {any} b @param {number} c @param {number} r @param {number} w */
  function block(b, c, r, w) {
    if (b.t === 'c') { contact(b.i, c, r); wire(c + 1, r, c + w); return; }
    if (b.t === 's') {
      let cc = c;
      b.a.forEach((/** @type {any} */ x, /** @type {number} */ k) => { const ww = k === b.a.length - 1 ? c + w - cc : bw(x); block(x, cc, r, ww); cc += bw(x); });
      return;
    }
    let rr = r;
    const rows = [];
    for (const x of b.a) { block(x, c, rr, w); rows.push(rr); rr += bh(x); }
    vline(c, rows[0], rows[rows.length - 1]);
    vline(c + w, rows[0], rows[rows.length - 1]);
  }
  /** An output at the output column, row r; its extra inputs (KEEP reset, CNTR dec/reset) start at row `insFrom`. Returns rows used. */
  function output(/** @type {any} */ o, /** @type {number} */ r, /** @type {number} */ insFrom) {
    const ln = L[o.i], x = X(TW), y = Y(r), w = OUTW * CW, name = (ln.pre || '') + ln.name;
    const coil = ['OUT', 'OUTNOT', 'SET', 'RSET', 'DIFU', 'DIFD'].includes(ln.name);
    let rows = 1;
    if (coil) {
      const a = opInfo(o.i, 0), m = x + w / 2;
      const mi = reg({ kind: 'coil', keys: [keyOf(a.o, 'b')].filter(Boolean), info: [a], line: ln.src, addr: a.addr });
      const mark = ln.name === 'SET' ? 'S' : ln.name === 'RSET' ? 'R' : ln.name === 'OUTNOT' ? '/' : ln.name === 'DIFU' ? '↑' : ln.name === 'DIFD' ? '↓' : '';
      s.push(`<g class="el" data-i="${mi}"><rect class="hl" x="${x + 2}" y="${y - 38}" width="${w - 4}" height="${RH - 2}" rx="3"/>`
        + `<path class="w" d="M${x} ${y}H${m - 12}M${m + 12} ${y}H${x + w}"/>`
        + `<path class="w" d="M${m - 6} ${y - 12}A 14 14 0 0 0 ${m - 6} ${y + 12}M${m + 6} ${y - 12}A 14 14 0 0 1 ${m + 6} ${y + 12}"/>`
        + (mark ? `<text x="${m}" y="${y + 4}" text-anchor="middle">${mark}</text>` : '')
        + `<text class="c" x="${m}" y="${y - 27}" text-anchor="middle">${esc(clip(a.comment, 26))}</text>`
        + `<text x="${m}" y="${y - 15}" text-anchor="middle">${esc(name + ' ' + a.text)}</text>`
        + `<title>${esc(ln.src + '\n' + a.addr + (a.comment ? '  ' + a.comment : ''))}</title></g>`);
    } else {
      const n = ln.args.length, ins = o.ins || [];
      const insRows = ins.reduce((/** @type {number} */ a, /** @type {any} */ b) => a + bh(b), 0);
      const r0 = Math.max(r + 1, insFrom);
      // The name line, one line per operand, and a margin: a MOV's destination was cut off at one row.
      rows = Math.max(1, Math.ceil((26 + n * 15) / (RH - 4)), ins.length ? r0 - r + insRows : 0);
      const isT = /^(TIM|TMH|CNT)/.test(ln.name);
      const first = n ? opInfo(o.i, 0) : null;
      const mi = reg({ kind: isT ? 'coil' : 'box', keys: isT && first ? [keyOf(first.o, 'b')].filter(Boolean) : [], info: ln.args.map((/** @type {any} */ _, /** @type {number} */ j) => opInfo(o.i, j)), line: ln.src, addr: first?.addr });
      const h = rows * RH - 20;
      let t = `<g class="el" data-i="${mi}"><rect class="hl" x="${x + 2}" y="${y - 18}" width="${w - 4}" height="${h + 6}" rx="3"/>`
        + `<rect class="box" x="${x + 10}" y="${y - 14}" width="${w - 20}" height="${h - 2}" rx="3"/><path class="w" d="M${x} ${y}H${x + 10}M${x + w - 10} ${y}H${x + w}"/>`
        + `<text x="${x + 16}" y="${y + 1}" style="font-weight:700">${esc(name)}</text>`;
      ln.args.forEach((/** @type {string} */ _, /** @type {number} */ j) => {
        const a = opInfo(o.i, j), yy = y + 15 + j * 15;
        const kind = ln.def.ops?.[j] === 'l' ? 'l' : 'w';
        const vi = keyOf(a.o, kind) && !(j === 0 && isT) ? reg({ kind: 'val', keys: [keyOf(a.o, kind)], bcd: isT || /^(BIN|BINL)$/.test(ln.name) && j === 0, info: [a], line: ln.src, addr: a.addr }) : -1;
        const tv = isT && j === 0 ? reg({ kind: 'val', keys: [keyOf(a.o, 'w')], bcd: !ln.name.endsWith('X'), info: [a], line: ln.src, addr: a.addr }) : vi;
        t += `<text x="${x + 16}" y="${yy}">${esc(clip(a.text, 15))}</text>` + (tv >= 0 ? `<text class="v" x="${x + w - 16}" y="${yy}" text-anchor="end" data-v="${tv}"></text>` : '')
          + (a.comment ? `<title>${esc(a.addr + '  ' + a.comment)}</title>` : '');
      });
      t += `<title>${esc(ln.src)}</title></g>`;
      s.push(t);
      // KEEP / CNT / CNTR: the extra inputs come from the left rail into the box.
      let rr = r0;
      for (const b of ins) { block(b, 0, rr, TW); rr += bh(b); }
    }
    s.push(`<path class="w" d="M${x + w} ${y}H${X(TW) + w + 4}"/>`);
    return rows;
  }
  /** @param {any} n @param {number} c @param {number} r @returns {number} rows */
  function node(n, c, r) {
    const cw = n.cond ? bw(n.cond) : 0, ch = n.cond ? bh(n.cond) : 1;
    if (n.cond) block(n.cond, c, r, cw);
    const bx = c + cw;
    let cr = r;
    for (const o of n.outs) {
      if (cr > r) vline(bx, r, cr);
      if (o.t === 'o') { wire(bx, cr, TW); cr += output(o, cr, r + ch); }
      else cr += node(o, bx, cr);
    }
    return Math.max(ch, cr - r, 1);
  }
  let rows = 0;
  for (const n of net.roots) rows += node(n, 0, rows);
  const W = X(TW) + OUTW * CW + RAIL, H = rows * RH + 6;
  const svg = `<svg class="lad" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}"><path class="w" d="M${RAIL} 2V${H - 2}M${W - RAIL + 4} 2V${H - 2}" style="stroke-width:3"/>${s.join('')}</svg>`;
  return { html: svg, mons: live ? mons : [], errors: [] };
}

// ------------------------------------------------------------------ a section
const io = new IntersectionObserver(es => { for (const e of es) { const i = +(/** @type {any} */ (e.target).dataset.r); if (e.isIntersecting) visible.add(i); else visible.delete(i); } }, { root: $('rungs'), rootMargin: '200px' });

function showSection(/** @type {number} */ pi, /** @type {number} */ si, /** @type {number} */ focus = -1) {
  cur = { prog: pi, sec: si };
  for (const el of $('tree').querySelectorAll('.sec')) el.classList.toggle('cur', +(/** @type {any} */ (el).dataset.p) === pi && +(/** @type {any} */ (el).dataset.s) === si);
  history.replaceState(null, '', '#p=' + pi + '&s=' + si + (focus >= 0 ? '&r=' + focus : ''));
  const sec = P.programs[pi].sections[si], box = $('rungs');
  io.disconnect(); visible.clear();
  box.innerHTML = '';
  rendered = [];
  const showIl = /** @type {HTMLInputElement} */ ($('showIl')).checked;
  const errs = new Map(P.errors.filter((/** @type {any} */ e) => e.prog === pi && e.sec === si).map((/** @type {any} */ e) => [e.rung, e.msg]));
  sec.rungs.forEach((/** @type {any} */ r, /** @type {number} */ ri) => {
    const el = document.createElement('div');
    el.className = 'rung'; el.dataset.r = String(ri);
    const d = r.il.length ? draw(pi, r.il) : { html: '<pre class="il">(empty rung)</pre>', mons: [] };
    el.innerHTML = `<div class="rh"><span class="no">${ri}</span><span class="cm">${esc(r.comment)}</span><span class="tools">`
      + `<button data-a="edit" title="Edit this rung (online)">✎ edit</button><button data-a="ins" title="Insert a rung below">＋ below</button>`
      + `<button data-a="insA" title="Insert a rung above">＋ above</button><button data-a="del" title="Delete this rung">✕</button></span></div>`
      + (errs.has(ri) ? '<div class="err">NOT RUNNING: ' + esc(errs.get(ri)) + '</div>' : '')
      + `<div class="body">${d.html}${showIl ? '<pre class="il">' + esc(r.il.join('\n')) + '</pre>' : ''}</div>`;
    for (const g of el.querySelectorAll('[data-i]')) d.mons[+(/** @type {any} */ (g).dataset.i)].g = g;
    for (const t of el.querySelectorAll('[data-v]')) d.mons[+(/** @type {any} */ (t).dataset.v)].t = t;
    el.querySelector('.rh .tools')?.addEventListener('click', ev => rungTool(/** @type {any} */ (ev.target).dataset.a, ri));
    box.appendChild(el);
    rendered.push({ el, mons: d.mons });
    io.observe(el);
  });
  if (focus >= 0 && rendered[focus]) {
    const el = rendered[focus].el;
    el.scrollIntoView({ block: 'center' });
    el.classList.add('hit');
    setTimeout(() => el.classList.remove('hit'), 2500);
  }
}

$('rungs').addEventListener('click', ev => {
  const g = /** @type {any} */ (ev.target).closest?.('g.el');
  if (!g) return;
  const r = g.closest('.rung');
  const m = rendered[+r.dataset.r]?.mons[+g.dataset.i];
  if (m) menu(ev, m);
});

// ------------------------------------------------------------------ monitoring
const fmt = (/** @type {any} */ v, /** @type {boolean} */ bcd) => (v == null ? '?' : bcd ? '#' + v.toString(16).toUpperCase() : String(v));
async function poll() {
  try {
    const keys = new Set();
    for (const i of visible) for (const m of rendered[i]?.mons ?? []) for (const k of m.keys) keys.add(k);
    for (const w of watch) for (const k of w.keys) keys.add(k);
    for (const r of ioRows) keys.add(r.key);
    const list = [...keys];
    const parts = [];
    for (let i = 0; i < list.length || i === 0; i += 300) parts.push(list.slice(i, i + 300));
    const res = await Promise.all(parts.map(p => fetch('/api/ladder/peek?k=' + encodeURIComponent(p.join(','))).then(r => r.json())));
    vals = new Map();
    res.forEach((d, n) => parts[n].forEach((k, i) => vals.set(k, d.v[i])));
    const d = res[0];
    forced = new Set(d.forced.map((/** @type {any} */ f) => f[0]));
    $('stats').textContent = 'scan ' + d.stats.scanUs + ' µs  max ' + d.stats.scanMaxUs + '  ·  ' + d.stats.scans + ' scans  ·  t ' + (d.t / 1000).toFixed(1) + ' s' + (forced.size ? '  ·  ' + forced.size + ' forced' : '');
    modeButton(d.stats.running);
    paint();
  } catch { $('stats').textContent = 'no connection'; }
  setTimeout(poll, POLL_MS);
}
const isForced = (/** @type {string} */ a) => !!a && (forced.has(a) || forced.has(a.replace(/\.\d+$/, '')));
function paint() {
  for (const i of visible) {
    for (const m of rendered[i]?.mons ?? []) {
      if (m.kind === 'c') { const v = vals.get(m.keys[0]); m.g?.classList.toggle('on', v != null && (m.neg ? v === 0 : v === 1)); m.g?.classList.toggle('forced', isForced(m.addr)); }
      else if (m.kind === 'coil') { const v = vals.get(m.keys[0]); m.g?.classList.toggle('on', v === 1); m.g?.classList.toggle('forced', isForced(m.addr)); }
      else if (m.kind === 'cmp') {
        const val = (/** @type {any} */ o) => (o.area === 'const' ? (m.k === 'l' ? o.value >>> 0 : o.value & 0xffff) : vals.get(m.k + ':' + addrText(o)));
        let a = val(m.a), b = val(m.b);
        if (a == null || b == null) continue;
        if (m.signed) { a = m.k === 'l' ? a | 0 : (a << 16) >> 16; b = m.k === 'l' ? b | 0 : (b << 16) >> 16; }
        const on = m.op === '=' ? a === b : m.op === '<>' ? a !== b : m.op === '<' ? a < b : m.op === '<=' ? a <= b : m.op === '>' ? a > b : a >= b;
        m.g?.classList.toggle('on', on);
        if (m.t) m.t.textContent = a + ' ' + m.op + ' ' + b;
      } else if (m.kind === 'val' && m.t) m.t.textContent = fmt(vals.get(m.keys[0]), m.bcd);
    }
  }
  for (const w of watch) if (/** @type {any} */ (w).td) {
    /** @type {any} */ (w).td.textContent = w.keys.map(k => fmt(vals.get(k), false)).join(' / ');
    /** @type {any} */ (w).tr.classList.toggle('on', w.keys[0].startsWith('b:') && vals.get(w.keys[0]) === 1);
    /** @type {any} */ (w).tr.classList.toggle('forced', isForced(w.keys[0].slice(2)));
  }
  for (const r of ioRows) if (r.tr) {
    const v = vals.get(r.key);
    /** @type {any} */ (r.tr.lastChild).textContent = v == null ? '?' : String(v);
    r.tr.classList.toggle('on', v === 1);
    r.tr.classList.toggle('forced', isForced(r.addr));
  }
}

// ------------------------------------------------------------------ the element menu: force, set, watch, cross-reference
function menu(/** @type {MouseEvent} */ ev, /** @type {any} */ m) {
  const box = $('menu');
  const infos = m.info.filter((/** @type {any} */ a) => a.o && a.o.area !== 'const');
  if (!infos.length) return;
  const a = infos[0];
  // A timer/counter operand is its PV word here (its flag cannot be forced, as on the CPU's
  // own force list), TR and condition flags are read-only.
  const tc = /^[TC]\d/.test(a.addr), ro = /^(TR|CF)/.test(a.addr);
  const bit = !tc && (m.kind === 'c' || m.kind === 'coil' || a.o?.bit >= 0);
  const val = vals.get((bit || ro ? 'b:' : 'w:') + a.addr);
  box.innerHTML = `<div class="t">${esc(a.text)}</div><div class="a">${esc(a.addr)}${a.comment ? ' · ' + esc(a.comment) : ''}</div>`
    + `<div class="a">value: ${esc(fmt(val, false))}${isForced(a.addr) ? '  (FORCED)' : ''}</div><div class="a">${esc(m.line)}</div>`
    + (ro ? '' : bit ? `<div class="row"><button data-a="f1">Force ON</button><button data-a="f0">Force OFF</button><button data-a="rel">Release</button></div>`
      + `<div class="row"><button data-a="s1">Set</button><button data-a="s0">Reset</button></div>`
      : `<div class="row"><input id="mval" class="mono" style="width:90px" value="${esc(val ?? 0)}"><button data-a="w">Write</button><button data-a="fw">Force</button><button data-a="rel">Release</button></div>`)
    + `<div class="row"><button data-a="watch">Watch</button><button data-a="xref">Cross-reference</button><button data-a="close">Close</button></div>`
    + (infos.length > 1 ? `<div class="row">${infos.slice(1).map((/** @type {any} */ x, /** @type {number} */ i) => `<button data-a="op${i + 1}">${esc(clip(x.text, 14))}</button>`).join('')}</div>` : '');
  box.hidden = false;
  box.style.left = Math.min(ev.clientX, innerWidth - 280) + 'px';
  box.style.top = Math.min(ev.clientY + 8, innerHeight - 220) + 'px';
  box.onclick = async e => {
    const act = /** @type {any} */ (e.target).dataset?.a;
    if (!act) return;
    try {
      if (act === 'f1' || act === 'f0') await post('/api/ladder/force', { addr: a.addr, value: act === 'f1' ? 1 : 0 });
      else if (act === 'rel') await post('/api/ladder/force', { addr: a.addr, value: null });
      else if (act === 's1' || act === 's0') await post('/api/ladder/set', { addr: a.addr, value: act === 's1' ? 1 : 0 });
      else if (act === 'w') await post('/api/ladder/set', { addr: a.addr, value: Number(/** @type {HTMLInputElement} */ ($('mval')).value) });
      else if (act === 'fw') await post('/api/ladder/force', { addr: a.addr, value: Number(/** @type {HTMLInputElement} */ ($('mval')).value) });
      else if (act === 'watch') addWatch(a.text, a.o);
      else if (act === 'xref') { /** @type {HTMLInputElement} */ ($('search')).value = a.addr; search(a.addr); }
      else if (act.startsWith('op')) { menu(ev, { ...m, info: [infos[+act.slice(2)]] }); return; }
    } catch (err) { toast(String(/** @type {any} */ (err).message || err)); }
    box.hidden = true;
  };
}
document.addEventListener('mousedown', e => { if (!$('menu').contains(/** @type {any} */ (e.target)) && !/** @type {any} */ (e.target).closest?.('g.el, #iomap tr, #watch tr')) $('menu').hidden = true; });
document.addEventListener('keydown', e => { if (e.key === 'Escape') $('menu').hidden = true; });

// ------------------------------------------------------------------ watch window and IO map
function addWatch(/** @type {string} */ label, /** @type {any} */ o) {
  const a = addrText(o);
  const keys = /^(T|C)\d/.test(a) ? ['b:' + a, 'w:' + a] : o.bit >= 0 || o.area === 'TR' || o.area === 'CF' ? ['b:' + a] : ['w:' + a];
  if (watch.some(w => w.keys[0] === keys[0])) return;
  watch.push({ label, keys });
  store.set('mio.ladder.watch.' + P.name, watch.map(w => ({ label: w.label, keys: w.keys })));
  renderWatch();
}
function renderWatch() {
  const t = /** @type {HTMLTableElement} */ ($('watch'));
  t.innerHTML = '';
  watch.forEach((w, i) => {
    const tr = t.insertRow();
    const a = w.keys[0].slice(2);
    const cm = byAddr.get(a)?.comment || scopes[cur.prog]?.find(w.label)?.comment || '';
    tr.innerHTML = `<td class="n" title="${esc(a + ' ' + cm)}">${esc(w.label)}</td><td class="n c" style="color:var(--muted)">${esc(clip(cm, 18))}</td><td class="val"></td><td><button title="Remove" style="padding:0 5px">✕</button></td>`;
    /** @type {any} */ (w).td = tr.cells[2]; /** @type {any} */ (w).tr = tr;
    tr.cells[3].onclick = e => { e.stopPropagation(); watch.splice(i, 1); store.set('mio.ladder.watch.' + P.name, watch.map(x => ({ label: x.label, keys: x.keys }))); renderWatch(); };
    tr.onclick = ev => { try { const o = parseOperand(a, 'b'); menu(ev, { kind: w.keys[0].startsWith('b:') ? 'c' : 'box', info: [{ text: w.label, addr: a, comment: cm, o }], line: '' }); } catch { /* not an address */ } };
  });
}
$('watchAdd').addEventListener('keydown', e => {
  if (e.key !== 'Enter') return;
  const inp = /** @type {HTMLInputElement} */ (e.target), text = inp.value.trim();
  try { addWatch(text, parseOperand(text, 'w', scopes[cur.prog])); inp.value = ''; } catch (err) { toast(String(/** @type {any} */ (err).message)); }
});

async function ioMap() {
  const tags = await fetch('/api/tags').then(r => r.json()).catch(() => []);
  const dir = new Map(tags.map((/** @type {any} */ t) => [t.tag, t.dir]));
  ioRows = Object.entries(P.map || {}).map(([tag, addr]) => ({ tag, addr: String(addr), dir: dir.get(tag) || '', key: (String(addr).includes('.') ? 'b:' : 'w:') + addr }));
  ioRows.sort((a, b) => a.addr.localeCompare(b.addr, undefined, { numeric: true }));
  renderIo();
}
function renderIo() {
  const f = /** @type {HTMLInputElement} */ ($('ioFilter')).value.trim().toLowerCase();
  const t = /** @type {HTMLTableElement} */ ($('iomap'));
  t.innerHTML = '';
  for (const r of ioRows) {
    r.tr = undefined;
    const cm = byAddr.get(r.addr)?.comment || '';
    if (f && !(r.tag + ' ' + r.addr + ' ' + cm).toLowerCase().includes(f)) continue;
    const tr = t.insertRow();
    tr.innerHTML = `<td>${esc(r.addr)}</td><td>${r.dir === 'in' ? '→' : r.dir === 'out' ? '←' : '·'}</td><td class="n" title="${esc(r.tag + '\n' + cm)}">${esc(r.tag)}</td><td class="val">?</td>`;
    tr.title = (r.dir === 'in' ? 'plant → PLC input' : r.dir === 'out' ? 'PLC output → plant' : '') + '\n' + cm;
    tr.onclick = ev => menu(ev, { kind: 'c', info: [{ text: r.tag, addr: r.addr, comment: cm, o: parseOperand(r.addr, 'b') }], line: r.tag });
    r.tr = tr;
  }
}
$('ioFilter').addEventListener('input', renderIo);

// ------------------------------------------------------------------ cross-reference and search
function buildIndex() {
  xindex = new Map();
  P.programs.forEach((/** @type {any} */ p, /** @type {number} */ pi) => p.sections.forEach((/** @type {any} */ s, /** @type {number} */ si) => s.rungs.forEach((/** @type {any} */ r, /** @type {number} */ ri) => {
    if (!r.il.length) return;
    const chk = checkRung(r.il, scopes[pi]);
    chk.lines.forEach((ln, li) => (chk.ops[li] || []).forEach((/** @type {any} */ o) => {
      if (!o || o.area === 'const' || o.area === 'ind') return;
      const keys = [addrText(o)];
      if (o.bit >= 0) keys.push(addrText({ ...o, bit: -1 }));           // a word search finds its bits
      for (const k of keys) { if (!xindex?.has(k)) xindex?.set(k, []); xindex?.get(k)?.push({ pi, si, ri, line: ln.src }); }
    }));
  })));
}
function search(/** @type {string} */ q) {
  q = q.trim();
  const out = $('xref');
  if (!q) { out.innerHTML = ''; return; }
  if (!xindex) buildIndex();
  /** @type {Array<{pi: number, si: number, ri: number, line: string}>} */
  let hits = [];
  let addr = null;
  try { addr = addrText(parseOperand(q, 'w', scopes[cur.prog])); } catch { /* not an address or symbol */ }
  if (addr) hits = xindex?.get(addr) ?? [];
  if (!hits.length) {
    // Comments: symbols whose comment matches, then rungs whose comment matches.
    const ql = q.toLowerCase();
    const syms = P.globals.filter((/** @type {any} */ g) => (g.name + ' ' + g.comment).toLowerCase().includes(ql)).slice(0, 30);
    for (const g of syms) for (const h of xindex?.get(g.addr) ?? []) if (hits.length < 200) hits.push(h);
    P.programs.forEach((/** @type {any} */ p, /** @type {number} */ pi) => p.sections.forEach((/** @type {any} */ s, /** @type {number} */ si) => s.rungs.forEach((/** @type {any} */ r, /** @type {number} */ ri) => {
      if (r.comment.toLowerCase().includes(ql) && hits.length < 300) hits.push({ pi, si, ri, line: '// ' + r.comment.split('\n')[0] });
    })));
  }
  out.innerHTML = hits.length ? hits.slice(0, 300).map((h, i) => `<div data-i="${i}" title="${esc(h.line)}">${esc(P.programs[h.pi].name.slice(0, 12))}/${esc(P.programs[h.pi].sections[h.si].name)} R${h.ri}: ${esc(h.line)}</div>`).join('')
    : '<div style="cursor:default;color:var(--muted)">nothing found</div>';
  for (const el of out.querySelectorAll('[data-i]')) el.addEventListener('click', () => { const h = hits[+(/** @type {any} */ (el).dataset.i)]; showSection(h.pi, h.si, h.ri); });
}
let st = 0;
$('search').addEventListener('input', e => { clearTimeout(st); st = setTimeout(() => search(/** @type {HTMLInputElement} */ (e.target).value), 200); });

// ------------------------------------------------------------------ online edit
function rungTool(/** @type {string} */ act, /** @type {number} */ ri) {
  if (!act) return;
  const sec = P.programs[cur.prog].sections[cur.sec];
  if (act === 'del') {
    if (!confirm('Delete rung ' + ri + ' of ' + sec.name + '? It stops running at once.')) return;
    apply(ri, 1, []).catch(e => toast(String(e.message || e)));
    return;
  }
  const isNew = act !== 'edit', at = act === 'ins' ? ri + 1 : ri;
  const r = isNew ? { comment: '', il: ['LD P_Off', 'OUT TR0'] } : sec.rungs[ri];
  const host = rendered[ri].el.querySelector('.body');
  if (!host) return;
  const ed = document.createElement('div');
  ed.className = 'editor';
  ed.innerHTML = `<div><b>${isNew ? 'New rung at ' + at : 'Edit rung ' + ri}</b> - mnemonic, one instruction per line (CX-Programmer IL). Applied ONLINE: the next scan runs it.</div>`
    + `<textarea class="cm" rows="2" placeholder="rung comment">${esc(r.comment)}</textarea>`
    + `<textarea class="il" rows="${Math.min(24, Math.max(6, r.il.length + 2))}" spellcheck="false">${esc(r.il.join('\n'))}</textarea>`
    + `<div class="msg"></div><div class="pv"></div><div><button class="ok">Apply online</button> <button class="no">Cancel</button></div>`;
  const ilBox = /** @type {HTMLTextAreaElement} */ (ed.querySelector('textarea.il')), cmBox = /** @type {HTMLTextAreaElement} */ (ed.querySelector('textarea.cm'));
  const msg = /** @type {HTMLElement} */ (ed.querySelector('.msg')), pv = /** @type {HTMLElement} */ (ed.querySelector('.pv'));
  const lines = () => ilBox.value.split('\n').map(x => x.trim()).filter(Boolean);
  const check = () => {
    const d = draw(cur.prog, lines(), false);
    pv.innerHTML = d.html;
    msg.className = 'msg' + (d.errors.length ? '' : ' ok');
    msg.textContent = d.errors.length ? d.errors.join('\n') : 'compiles';
  };
  ilBox.addEventListener('input', check);
  check();
  const wrap = document.createElement('div');
  if (isNew) { wrap.className = 'rung'; wrap.appendChild(ed); rendered[ri].el[act === 'ins' ? 'after' : 'before'](wrap); }
  else { host.innerHTML = ''; host.appendChild(ed); }
  ilBox.focus();
  /** @type {HTMLButtonElement} */ (ed.querySelector('.no')).onclick = () => showSection(cur.prog, cur.sec, ri);
  /** @type {HTMLButtonElement} */ (ed.querySelector('.ok')).onclick = async () => {
    try {
      const res = await apply(at, isNew ? 0 : 1, [{ il: lines(), comment: cmBox.value }]);
      if (!res.ok) { msg.className = 'msg'; msg.textContent = res.errors.join('\n'); }
    } catch (e) { msg.className = 'msg'; msg.textContent = String(/** @type {any} */ (e).message || e); }
  };
}
/** @param {number} at @param {number} del @param {Array<{il: string[], comment: string}>} rungs */
async function apply(at, del, rungs) {
  const r = await post('/api/ladder/edit', { prog: cur.prog, sec: cur.sec, at, del, rungs });
  if (!r.ok) return r;
  P.programs[cur.prog].sections[cur.sec].rungs.splice(at, del, ...rungs.map(x => ({ comment: x.comment, il: x.il })));
  xindex = null;
  tree();
  showSection(cur.prog, cur.sec, Math.min(at, P.programs[cur.prog].sections[cur.sec].rungs.length - 1));
  toast(del && !rungs.length ? 'rung deleted - running now' : 'rung applied - running now (Save writes it to the .cxp)');
  return r;
}

// ------------------------------------------------------------------ header
$('mode').addEventListener('click', async () => {
  const run = $('mode').textContent !== 'RUN';
  if (!run && !confirm('PROGRAM mode: the scan stops and every output goes OFF. Continue?')) return;
  await post('/api/ladder/mode', { run }).catch(e => toast(String(e.message)));
});
$('release').addEventListener('click', () => post('/api/ladder/release', {}).then(() => toast('all forces released')).catch(e => toast(String(e.message))));
$('save').addEventListener('click', async () => {
  if (!confirm('Write the program as it runs now over ' + P.file + '? The previous file is kept as a .bak beside it.')) return;
  try { const r = await post('/api/ladder/save', {}); toast('saved ' + r.file + '  (backup ' + r.backup + ')', 5000); } catch (e) { toast(String(/** @type {any} */ (e).message)); }
});
$('file').addEventListener('change', async e => {
  const file = /** @type {HTMLSelectElement} */ (e.target).value;
  if (!confirm('Load ' + file + '? Online edits that were not saved are lost, and forces are dropped.')) { /** @type {HTMLSelectElement} */ (e.target).value = P.file; return; }
  try { await post('/api/ladder/open', { file }); await load(); toast('loaded ' + file); } catch (err) { toast(String(/** @type {any} */ (err).message)); }
});
$('showIl').addEventListener('change', () => showSection(cur.prog, cur.sec));

load().then(poll);
