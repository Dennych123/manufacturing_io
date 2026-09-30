// @ts-check
// The machine's touch panel, drawn from its own Keyence VT STUDIO file (server/vs4.js) and run
// against the ladder soft-PLC: lamps light from the bits the file names, switches write the bits
// the file names (held while the finger is down, like the panel's momentary switches), and a
// part that changes screens changes screens. The PLC program enforces every condition - this page
// only presses.

const $ = (/** @type {string} */ id) => /** @type {HTMLElement} */ (document.getElementById(id));
const esc = (/** @type {any} */ s) => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c] ?? c);
const post = (/** @type {string} */ url, /** @type {any} */ body) => fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }).then(r => r.json());
const POLL_MS = 200;

/** @type {any} */
let H = { screens: [] };
/** @type {Map<number, any>} */
let byNo = new Map();
let cur = 1;
/** @type {number[]} */
const history = [];
/** @type {Array<{el: HTMLElement, addr: string, labels: string[]}>} */
let lamps = [];
/** @type {string[]} */
let keys = [];

async function load() {
  H = await (await fetch('/api/ladder/hmi')).json();
  if (H.error) { $('warn').textContent = H.error; return; }
  byNo = new Map(H.screens.filter((/** @type {any} */ s) => s.no > 0).map((/** @type {any} */ s) => [s.no, s]));
  const f = /** @type {HTMLSelectElement} */ ($('file'));
  f.innerHTML = H.files.map((/** @type {string} */ x) => '<option' + (x === H.file ? ' selected' : '') + '>' + esc(x) + '</option>').join('');
  $('warn').textContent = (H.warnings || []).join('\n');
  $('screens').innerHTML = [...byNo.values()].sort((a, b) => a.no - b.no).map(s => '<div data-no="' + s.no + '"><span>' + s.no + '</span>' + esc(s.title.replace(/\n/g, ' ')) + '</div>').join('');
  const want = +(new URLSearchParams(location.hash.slice(1)).get('s') ?? 1);
  show(byNo.has(want) ? want : byNo.has(1) ? 1 : [...byNo.keys()][0]);
}

/** @param {number} no @param {boolean} [push] */
function show(no, push = false) {
  const s = byNo.get(no);
  if (!s) return;
  if (push && cur !== no) history.push(cur);
  cur = no;
  history.length = Math.min(history.length, 50);
  location.replace('#s=' + no);
  $('where').textContent = s.id + '  ' + s.title.replace(/\n/g, ' ');
  $('scrno').textContent = 'SCREEN ' + no;
  for (const d of $('screens').querySelectorAll('div')) d.classList.toggle('cur', d.getAttribute('data-no') === String(no));
  const lcd = $('lcd');
  lcd.style.width = s.w + 'px'; lcd.style.height = s.h + 'px';
  lcd.innerHTML = '';
  lamps = [];
  /** @type {Set<string>} */
  const addrs = new Set();
  /** @type {number[][]} */
  const switchRects = [];
  for (const p of s.parts) if ((p.kind === 'Ls' || p.write) && p.items[0]) switchRects.push(p.items[0].rect);
  const overlap = (/** @type {number[]} */ a, /** @type {number[]} */ b) => {
    const w = Math.min(a[0] + a[2], b[0] + b[2]) - Math.max(a[0], b[0]), h = Math.min(a[1] + a[3], b[1] + b[3]) - Math.max(a[1], b[1]);
    return w > 0 && h > 0 ? (w * h) / (a[2] * a[3]) : 0;
  };
  for (const p of s.parts) {
    const goes = p.go === 'back' ? history.length > 0 || true : byNo.has(p.go);
    p.items.forEach((/** @type {any} */ it, /** @type {number} */ k) => {
      const [x, y, w, h] = it.rect;
      const el = document.createElement('div');
      el.style.left = x + 'px'; el.style.top = y + 'px'; el.style.width = w + 'px'; el.style.height = h + 'px';
      const labels = it.labels.filter((/** @type {string} */ l) => l !== '');
      let cls = 'it';
      const body = k === 0;
      if (body && (p.kind === 'Ls' || p.write)) {
        cls += ' sw';
        if (p.write) {
          const a = p.write;
          addrs.add(a);
          const down = () => { el.classList.add('dn'); post('/api/ladder/hmipress', { addr: a, down: true }); };
          const up = () => { if (!el.classList.contains('dn')) return; el.classList.remove('dn'); post('/api/ladder/hmipress', { addr: a, down: false }); };
          el.addEventListener('pointerdown', e => { e.preventDefault(); el.setPointerCapture(e.pointerId); down(); });
          el.addEventListener('pointerup', up); el.addEventListener('pointercancel', up); el.addEventListener('lostpointercapture', up);
          el.title = p.name + '  writes ' + a + (p.lamp ? '  lamp ' + p.lamp : '');
        }
        if (p.go != null && goes) { el.addEventListener('click', () => (p.go === 'back' ? back() : show(p.go, true))); }
      } else if (body && p.kind === 'Lp') {
        cls += ' lamp';
        el.title = p.name + (p.lamp ? '  lamp ' + p.lamp : '');
      } else if (body && p.kind === 'N') {
        cls += ' num';
      } else if (body && p.go != null && goes) {
        // A text or a shape that changes screens: a touch switch. One that only covers a switch
        // (the panel's transparent 'go to FAULT' over ALARM RESET) is left to the switch.
        if (!labels.length && switchRects.some(r => overlap(it.rect, r) > 0.8)) return;
        cls += ' sw go';
        el.addEventListener('click', () => (p.go === 'back' ? back() : show(/** @type {number} */ (p.go), true)));
        el.title = 'go to ' + (p.go === 'back' ? 'the previous screen' : p.go + ' ' + (byNo.get(p.go)?.title ?? ''));
      } else {
        cls += ' text' + (labels.length ? '' : ' box');
      }
      el.className = cls;
      // A text's own colours, as the file gives them (black background = none).
      if (cls.includes(' text')) { if (it.fg) el.style.color = it.fg; if (it.bg && it.bg !== '#000') el.style.background = it.bg; }
      if (p.lamp && body && (cls.includes('lamp') || cls.includes('sw'))) {
        addrs.add(p.lamp);
        lamps.push({ el, addr: p.lamp, labels });
        el.textContent = labels[0] ?? '';
        if (labels.length > 1 && /FAULT|ALARM|NG|ERROR|ABNORMAL/i.test(labels[1])) el.dataset.fault = '1';
      } else el.textContent = labels[labels.length > 1 && !cls.includes('sw') ? 0 : 0] ?? '';
      fit(el, w, h, labels);
      lcd.appendChild(el);
    });
  }
  keys = [...addrs];
  $('devs').innerHTML = keys.length ? keys.map(a => '<span data-a="' + esc(a) + '">' + esc(a) + '=<b>?</b></span>').join('  ') : 'none';
  scale();
  poll();
}

/**
 * The panel's fonts are sized to their boxes; a page font at one size overflows the small ones
 * ('MULTI PURPOSE' in a 27-dot button) and dwarfs the big ones. Size each text to its box.
 * @param {HTMLElement} el @param {number} w @param {number} h @param {string[]} labels
 */
function fit(el, w, h, labels) {
  const lines = Math.max(1, ...labels.map(l => l.split(/\n/).length));
  const chars = Math.max(1, ...labels.flatMap(l => l.split(/\n/)).map(l => l.length));
  const px = Math.max(5, Math.min(12, (h - 3) / lines / 1.1, (w - 3) / chars / 0.56));
  el.style.fontSize = px.toFixed(1) + 'px';
}

function back() { const to = history.pop(); if (to != null) show(to); else show(1); }

function scale() {
  const z = +(/** @type {HTMLInputElement} */ ($('zoom')).value);
  const s = byNo.get(cur);
  if (!s) return;
  $('lcd').style.transform = 'scale(' + z + ')';
  $('wrap').style.width = s.w * z + 'px'; $('wrap').style.height = s.h * z + 'px';
}

let busy = false;
async function poll() {
  if (busy || !keys.length) return;
  busy = true;
  try {
    const r = await (await fetch('/api/ladder/peek?k=' + keys.map(a => 'b:' + a).join(','))).json();
    const v = new Map(keys.map((a, i) => [a, r.v[i]]));
    for (const l of lamps) {
      const on = !!v.get(l.addr);
      l.el.classList.toggle('lit', on);
      l.el.classList.toggle('fault', on && l.el.dataset.fault === '1');
      if (l.labels.length > 1) l.el.textContent = l.labels[on ? 1 : 0];
    }
    for (const sp of $('devs').querySelectorAll('span')) { const b = sp.querySelector('b'); if (b) b.textContent = String(v.get(sp.getAttribute('data-a') || '') ?? '?'); }
  } catch { /* server gone: the next poll tries again */ }
  finally { busy = false; }
}

$('screens').addEventListener('click', e => {
  const d = /** @type {HTMLElement} */ (e.target).closest('[data-no]');
  if (d) show(+(/** @type {string} */ (d.getAttribute('data-no'))), true);
});
$('zoom').addEventListener('input', scale);
$('file').addEventListener('change', async e => {
  await post('/api/ladder/hmiopen', { file: /** @type {HTMLSelectElement} */ (e.target).value });
  history.length = 0;
  load();
});
load();
setInterval(poll, POLL_MS);
