// @ts-check
// The digital-twin page: why the machine will not start (the program's own master / start / home
// conditions, each OFF one with the contacts that hold it off), and where the cycle time goes
// (per unit busy / period, a timeline, and the steps of one unit's last cycle). Everything comes
// from server/twin.js over /api/ladder/diag and /api/ladder/ct; nothing is computed here.

const $ = (/** @type {string} */ id) => /** @type {HTMLElement} */ (document.getElementById(id));
const esc = (/** @type {any} */ s) => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c] ?? c);
const POLL_MS = 500, WINDOW_MS = 60_000;

let sel = '';                                       // the unit whose steps are shown ('' = the bottleneck)
/** @type {Record<string, string>} */
const last = {};                                    // what each block was built from, so it is rebuilt only when it changes

/** One contact, or a group of them, that holds a condition off. @param {any} w @returns {string} */
function whyHtml(w) {
  if (!w) return '';
  if (w.all) return w.all.map(whyHtml).join('');
  if (w.any) return '<li class="any"><span>one of:</span><ul>' + w.any.map((/** @type {any} */ x) => '<ul>' + whyHtml(x) + '</ul>').join('') + '</ul></li>';
  if (w.cmp) return '<li class="c">✗ ' + esc(w.text) + '</li>';
  const want = w.not ? 'is ON, must be OFF' : 'is OFF';
  return '<li class="c">✗ ' + (w.not ? 'NOT ' : '') + esc(w.text) + (w.addr && w.addr !== w.text ? ' <span class="addr">' + esc(w.addr) + '</span>' : '')
    + (w.edge ? ' <span class="addr">(' + (w.edge === '@' ? 'rising edge' : 'falling edge') + ')</span>' : '')
    + ' <span class="cm">' + esc(w.comment) + ' — ' + want + '</span></li>';
}
/** @param {any} x */
function condHtml(x) {
  const rungs = (x.why || []).map((/** @type {any} */ r) => '<div class="rung"><a href="/ladder#p=' + r.p + '&s=' + r.s + '&r=' + r.r + '" target="mioladder">' + esc(r.where) + '</a> '
    + esc(r.coil !== 'OUT' ? r.coil : '') + (r.comment ? ' · ' + esc(r.comment) : '') + '</div>' + (r.why ? '<ul>' + whyHtml(r.why) + '</ul>' : '')).join('');
  return '<li class="' + (x.value ? 'on' : 'off') + '"><div class="h"><span class="mark">' + (x.value ? '✓' : '✗') + '</span><span>' + esc(x.comment || x.name)
    + '</span><span class="addr">' + esc(x.addr) + '</span></div>' + (rungs ? '<div class="why">' + rungs + '</div>' : '') + '</li>';
}

/** @param {any} d */
function renderDiag(d) {
  const off = (/** @type {any[]} */ a) => a.filter(x => !x.value).length;
  const blocking = d.alarms.length + off(d.master) + off(d.start) + off(d.home);
  const v = $('verdict');
  v.className = 'verdict ' + (blocking ? 'bad' : 'ok');
  v.textContent = !d.running ? 'The soft-PLC is in PROGRAM mode: nothing runs.'
    : blocking ? blocking + ' thing(s) stop AUTO: ' + [d.alarms.length && d.alarms.length + ' alarm(s)', off(d.master) && off(d.master) + ' master condition(s)',
      off(d.start) && off(d.start) + ' start condition(s)', off(d.home) && off(d.home) + ' home position(s)'].filter(Boolean).join(', ')
    : 'Every master, start and home condition is ON and no alarm is active: AUTO RUN will start.';
  const al = JSON.stringify(d.alarms);
  if (al !== last.al) {
    last.al = al;
    $('alarms').innerHTML = d.alarms.length ? '<h3>Alarms <span class="n">reset with HMI: FAULT RESET once the cause is gone</span></h3>'
      + d.alarms.map((/** @type {any} */ a) => '<div class="al">⚠ ' + esc(a.comment || a.name) + ' <span class="addr">' + esc(a.addr) + '</span></div>').join('') : '';
  }
  const groups = [['master', 'Master conditions', 'MASTER ON needs these'], ['start', 'Start conditions', 'AUTO RUN needs all of these'],
    ['home', 'Home positions', 'HOME POS brings the units here'], ['unitStart', 'Unit cycle start conditions', 'each unit starts its cycle when ON']];
  const g = JSON.stringify(groups.map(([k]) => d[k]));
  if (g !== last.g) {
    last.g = g;
    $('groups').innerHTML = groups.map(([k, t, n]) => {
      const a = d[k];
      return '<h3>' + esc(t) + ' <span class="n">' + esc(n) + ' · ' + (a.length - off(a)) + '/' + a.length + ' ON</span></h3><ul class="cond">' + a.map(condHtml).join('') + '</ul>';
    }).join('');
  }
}

/** What an operator does about one root cause. @param {any} l */
function action(l) {
  const want = l.not ? 'must be OFF' : 'must be ON';
  const hint = l.kind === 'scene' ? (l.not ? 'release / clear ' : 'press / make ') + '<b>' + esc(l.tag) + '</b>'
    : l.kind === 'hmi' ? 'HMI switch ' + esc(l.addr) + (l.comment ? ' <b>' + esc(l.comment) + '</b>' : '')
    : l.kind === 'input' ? 'input ' + esc(l.addr) + (l.comment ? ' <b>' + esc(l.comment) + '</b>' : '') + ' ' + want
    : l.kind === 'timer' ? 'timer ' + esc(l.text) + ' still running - wait'
    : l.kind === 'compare' ? esc(l.text)
    : (l.not ? 'NOT ' : '') + esc(l.text) + (l.comment ? ' <b>' + esc(l.comment) + '</b>' : '') + ' ' + want;
  return '<li><span class="k">' + esc(l.kind) + '</span> ' + hint + ' <span class="a">(' + esc(l.via) + ')</span></li>';
}

/** @param {any} st */
function renderSteps(st) {
  if (st.steps[st.steps.length - 1]?.ok) { $('verdict').className = 'verdict ok'; $('verdict').textContent = 'AUTO is running.'; }
  const key = JSON.stringify(st.steps);
  if (key === last.steps) return;
  last.steps = key;
  const first = st.steps.findIndex((/** @type {any} */ x) => !x.ok);
  $('stepper').innerHTML = st.steps.map((/** @type {any} */ x, /** @type {number} */ i) => {
    const cls = x.ok ? 'ok' : i === first ? 'bad' : 'wait';
    const body = i === first
      ? (x.off.length ? '<div class="off">not yet: ' + x.off.slice(0, 6).map(esc).join(' · ') + '</div>' : '')
        + (x.causes.length ? '<div class="do">What holds it:<ul>' + x.causes.map(action).join('') + '</ul></div>' : '')
      : '';
    return '<li class="' + cls + '"><div class="t">' + esc(x.title) + '</div>' + body + '</li>';
  }).join('');
}

/** @param {any} c */
function renderCt(c) {
  const us = c.units, neck = c.bottleneck;
  $('neck').textContent = neck ? 'bottleneck: ' + neck + ' (longest unit cycle)' : 'waiting for two cycles of each unit';
  const rows = us.map((/** @type {any} */ u) => '<tr data-u="' + u.unit + '" class="' + (u.unit === (sel || neck) ? 'sel ' : '') + (u.unit === neck ? 'neck' : '') + '"><td>' + esc(u.unit)
    + ' <span class="addr">' + esc(u.prog.replace(/^P\d+_/, '')) + '</span></td><td class="num">' + u.cycles + '</td><td class="num">' + (u.busyMs / 1000).toFixed(2)
    + '</td><td class="num">' + (u.periodMs ? (u.periodMs / 1000).toFixed(2) : '–') + '</td><td><div class="bar"><i style="width:' + Math.round(u.util * 100) + '%"></i></div></td><td class="num">'
    + (u.periodMs ? Math.round(u.util * 100) + '%' : '–') + '</td><td class="num">' + (u.busy ? (u.running / 1000).toFixed(1) + ' s' : '') + '</td></tr>').join('');
  $('units').innerHTML = '<tr><th>unit</th><th>cycles</th><th>busy s</th><th>period s</th><th colspan="2">busy / period</th><th>now</th></tr>' + rows;
  // Timeline.
  const svg = $('gantt'), W = svg.clientWidth || 600, rowH = 22, left = 150, H = us.length * rowH + 20;
  svg.setAttribute('viewBox', '0 0 ' + W + ' ' + H); svg.setAttribute('height', String(H));
  const t1 = c.t, t0 = t1 - WINDOW_MS, x = (/** @type {number} */ t) => left + (t - t0) / WINDOW_MS * (W - left - 4);
  let s = '';
  for (let k = 0; k <= 6; k++) { const tx = x(t0 + k * 10_000); s += '<line class="grid" x1="' + tx + '" x2="' + tx + '" y1="0" y2="' + (H - 16) + '"/><text class="t" x="' + tx + '" y="' + (H - 4) + '" text-anchor="middle">' + (k === 6 ? 'now' : '-' + (60 - k * 10) + ' s') + '</text>'; }
  us.forEach((/** @type {any} */ u, /** @type {number} */ i) => {
    const y = i * rowH + 3;
    s += '<text x="4" y="' + (y + 14) + '">' + esc(u.unit + ' ' + u.prog.replace(/^P\d+_/, '')) + '</text>';
    const spans = u.busy ? [...u.spans, [t1 - u.running, t1]] : u.spans;
    for (const [a, b] of spans) {
      if (b < t0) continue;
      const xa = x(Math.max(a, t0)), xb = x(b);
      s += '<rect class="b' + (u.unit === neck ? ' neck' : '') + '" x="' + xa.toFixed(1) + '" y="' + y + '" width="' + Math.max(1, xb - xa).toFixed(1) + '" height="' + (rowH - 8) + '" rx="2"><title>'
        + esc(u.unit) + ' ' + ((b - a) / 1000).toFixed(2) + ' s</title></rect>';
    }
  });
  svg.innerHTML = s;
  // Steps of one unit's last cycle.
  const u = us.find((/** @type {any} */ x) => x.unit === (sel || neck)) || us[0];
  if (!u) return;
  $('stepsH').innerHTML = 'Steps of ' + esc(u.unit + ' ' + u.prog) + ' <span class="n">last cycle ' + (u.lastMs / 1000).toFixed(2) + ' s; each step lasts until the next one is reached · click a unit above</span>';
  const max = Math.max(1, ...u.steps.map((/** @type {any} */ st) => st.ms));
  const top = new Set([...u.steps].sort((a, b) => b.ms - a.ms).slice(0, 5).filter((/** @type {any} */ st) => st.ms > 0).map((/** @type {any} */ st) => st.name));
  $('steps').innerHTML = u.steps.length ? '<tr><td class="num">at ms</td><td>step</td><td class="num">ms</td><td class="num">avg</td><td></td></tr>' + u.steps.map((/** @type {any} */ st) =>
    '<tr class="' + (top.has(st.name) ? 'top' : '') + '"><td class="num">' + st.at + '</td><td class="nm" title="' + esc(st.name) + '">' + esc(st.comment) + '</td><td class="num">' + st.ms
    + '</td><td class="num">' + st.avg + '</td><td style="width:35%"><div class="sbar" style="width:' + (st.ms / max * 100).toFixed(1) + '%"></div></td></tr>').join('')
    : '<tr><td class="muted">no cycle of this unit recorded yet</td></tr>';
}

$('units').addEventListener('click', e => {
  const tr = /** @type {HTMLElement} */ (e.target).closest('tr[data-u]');
  if (tr) { sel = /** @type {string} */ (tr.getAttribute('data-u')); poll(); }
});
$('onlyBad').addEventListener('change', e => document.body.classList.toggle('hide-ok', /** @type {HTMLInputElement} */ (e.target).checked));
document.body.classList.add('hide-ok');
$('reset').addEventListener('click', async () => { await fetch('/api/ladder/ctreset', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' }); poll(); });

// What-if speeds: every cylinder, electric axis, gripper, belt and table, with its timing params.
const UNIT = /** @type {Record<string, string>} */ ({ extendMs: 'ms', retractMs: 'ms', closeMs: 'ms', openMs: 'ms', buildMs: 'ms', dropMs: 'ms', camMs: 'ms', vmax: 'mm/s or deg/s', acc: 'mm/s2', speed: 'mm/s or deg/s' });
async function loadTune() {
  const t = await (await fetch('/api/tune')).json();
  $('tuneFile').textContent = t.file;
  $('tune').innerHTML = '<tr><th>actuator</th><th>station</th><th colspan="4">speed</th></tr>' + t.items.map((/** @type {any} */ it) => '<tr><td>' + esc(it.label) + '</td><td class="st">' + esc(it.station) + '</td>'
    + Object.entries(it.keys).map(([k, v]) => '<td class="st">' + esc(k) + '</td><td><input data-id="' + esc(it.id) + '" data-k="' + esc(k) + '" value="' + /** @type {any} */ (v).value + '" placeholder="' + /** @type {any} */ (v).base + '"'
      + (/** @type {any} */ (v).value !== /** @type {any} */ (v).base ? ' class="changed"' : '') + ' title="' + esc(UNIT[k] || '') + ', design ' + /** @type {any} */ (v).base + '"></td>').join('') + '</tr>').join('');
}
$('tune').addEventListener('change', async e => {
  const i = /** @type {HTMLInputElement} */ (e.target);
  if (!i.dataset.id) return;
  const r = await fetch('/api/tune', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ id: i.dataset.id, key: i.dataset.k, value: i.value === '' ? null : Number(i.value) }) }).then(x => x.json());
  if (r.error) { i.style.borderColor = 'var(--bad)'; i.title = r.error; return; }
  i.value = String(r.value);
  i.classList.toggle('changed', String(r.value) !== i.placeholder);
});
loadTune().catch(() => {});

let busy = false;
async function poll() {
  if (busy) return;
  busy = true;
  try {
    const [d, c, st] = await Promise.all([fetch('/api/ladder/diag').then(r => r.json()), fetch('/api/ladder/ct').then(r => r.json()), fetch('/api/ladder/steps').then(r => r.json())]);
    if (d.error) { $('verdict').textContent = d.error; return; }
    $('state').textContent = 'sim ' + (c.t / 1000).toFixed(1) + ' s';
    renderDiag(d);
    renderSteps(st);
    renderCt(c);
  } catch (e) { $('state').textContent = 'no server: ' + /** @type {Error} */ (e).message; }
  finally { busy = false; }
}
fetch('/api/ladder').then(r => r.json()).then(p => { $('proj').textContent = (p.name || '') + '  ' + (p.file || ''); document.title = 'Twin · ' + (p.name || ''); }).catch(() => {});
poll();
setInterval(poll, POLL_MS);
