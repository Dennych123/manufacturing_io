// Scene editor (docs/PLAN.md §10). It edits a LOCAL copy of the scene while the plant is
// stopped. Save validates with the same validate() the server uses and PUTs with baseVersion,
// so a scene someone else saved first answers 409 instead of being overwritten. Every change
// is one commit(): a new scene object plus an undo snapshot, so no mutation path can skip undo
// or the rebuild. Mount math (world drag -> at/rot) comes from lib/scene.js only.
import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { compile, validate, stringify, params, bindings, mountPoses, mountFrom } from '/lib/scene.js';
import { TYPES } from '/lib/components.js';

const UNDO_MAX = 100;
const ID_RE = /^[A-Za-z_][A-Za-z0-9_-]*$/;
const $ = id => document.getElementById(id);
// `list` is a read-only property on inputs: it can only be set as an attribute.
const el = (tag, props = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(props)) if (k === 'list') e.setAttribute(k, v); else e[k] = v;
  e.append(...kids);
  return e;
};
const find = (s, id) => s.components.find(c => c.id === id);

/** Ids of `id` and everything mounted on it, root first. */
function subtree(s, id) {
  const out = [id];
  for (let i = 0; i < out.length; i++) for (const c of s.components) if (c.parent === out[i] && !out.includes(c.id)) out.push(c.id);
  return out;
}

/**
 * @param {{scene3: THREE.Scene, camera: THREE.Camera, renderer: THREE.WebGLRenderer, controls: any,
 *          rebuild: (sc: any) => void, preview: (sc: any) => void, model: () => any, dof: () => any,
 *          pick: (e: PointerEvent) => any, sceneName: () => string, serverScene: () => any,
 *          post: (url: string, data: any) => Promise<void>}} ctx
 */
export function createEditor(ctx) {
  const ed = { active: false, enter, exit, onServerScene };
  let scene = null, base = null, saved = '', sel = null, mode = 'translate';
  const undo = [], redo = [];

  // ---------------------------------------------------------------- 3D: gizmo and highlight
  const tc = new TransformControls(ctx.camera, ctx.renderer.domElement);
  const proxy = new THREE.Object3D();
  const helper = tc.getHelper();                 // since r169 TransformControls is not an Object3D
  helper.visible = false;
  const box = new THREE.Box3(), boxH = new THREE.Box3Helper(box, 0xff8800);
  boxH.visible = false;
  ctx.scene3.add(proxy, helper, boxH);
  tc.addEventListener('dragging-changed', e => { ctx.controls.enabled = !e.value; });
  tc.addEventListener('objectChange', () => { const m = fromProxy(); if (m) { ctx.preview(withMount(scene, m)); refreshBox(); } });
  tc.addEventListener('mouseUp', () => {
    const m = fromProxy();
    if (m) commit(s => applyMount(find(s, sel), m));
  });

  /** at/rot of the selected component from where the gizmo left the proxy; translation snaps in the parent's frame, moved axes only. */
  function fromProxy() {
    const mp = sel && mountPoses(scene, ctx.dof(), sel);
    if (!mp) return null;
    const m = mountFrom(mp.base, { p: proxy.position.toArray(), q: proxy.quaternion.toArray() });
    if (mode === 'translate') {
      const c = find(scene, sel), step = +$('ed-snap').value, old = c.at || [0, 0, 0];
      m.at = m.at.map((v, i) => (Math.abs(v - (old[i] || 0)) > 1e-6 ? Math.round(v / step) * step : old[i] || 0));
      m.rot = c.rot ? [...c.rot] : [0, 0, 0];
    }
    return m;
  }
  const applyMount = (c, m) => { c.at = m.at; if (m.rot.every(v => v === 0)) delete c.rot; else c.rot = m.rot; };
  const withMount = (s, m) => { const n = structuredClone(s); applyMount(find(n, sel), m); return n; };

  function placeProxy() {
    const mp = ed.active && sel && mountPoses(scene, ctx.dof(), sel);
    if (!mp) { tc.detach(); helper.visible = false; return; }
    proxy.position.fromArray(mp.frame.p);
    proxy.quaternion.fromArray(mp.frame.q);
    proxy.updateMatrixWorld();
    tc.attach(proxy);
    tc.setMode(mode);
    tc.rotationSnap = THREE.MathUtils.degToRad(+$('ed-rsnap').value);
    helper.visible = true;
  }
  function refreshBox() {
    box.makeEmpty();
    if (ed.active && sel) for (const m of ctx.model()?.meshes || []) if (m.userData.id === sel) { m.updateWorldMatrix(true, false); box.expandByObject(m); }
    boxH.visible = !box.isEmpty();
  }

  // Select on click (not on drag, which orbits); never when the click lands on the gizmo.
  let down = null;
  ctx.renderer.domElement.addEventListener('pointerdown', e => { down = ed.active && e.button === 0 ? [e.clientX, e.clientY] : null; });
  ctx.renderer.domElement.addEventListener('pointerup', e => {
    if (!down || tc.dragging || tc.axis || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4) { down = null; return; }
    down = null;
    select(ctx.pick(e)?.object.userData.id ?? null);
  });

  // ---------------------------------------------------------------- scene changes
  function commit(fn) {
    const next = structuredClone(scene);
    fn(next);
    undo.push(JSON.stringify(scene));
    if (undo.length > UNDO_MAX) undo.shift();
    redo.length = 0;
    setScene(next);
  }
  function setScene(s) {
    scene = s;
    if (sel && !find(s, sel)) sel = null;
    ctx.rebuild(s);
    refresh();
  }
  function step(from, to) {
    if (!from.length) return;
    to.push(JSON.stringify(scene));
    setScene(JSON.parse(from.pop()));
  }
  const dirty = () => !!scene && stringify(scene) !== saved;
  function select(id) { sel = id; refresh(); }
  function refresh() {
    placeProxy(); refreshBox(); renderTree(); renderProps(); renderErrors();
    $('ed-undo').disabled = !undo.length; $('ed-redo').disabled = !redo.length;
    $('ed-dup').disabled = $('ed-del').disabled = !sel;
    $('ed-save').textContent = dirty() ? 'Save *' : 'Save';
  }
  function msg(text, bad = false) { const m = $('ed-msg'); m.textContent = text; m.className = bad ? 'bad' : ''; }

  function freshId(taken, stem) {
    stem = stem.replace(/\d+$/, '') || 'c';
    let n = 1;
    while (taken.has(stem + n)) n++;
    taken.add(stem + n);
    return stem + n;
  }
  function add() {
    const [type, pi] = $('ed-type').value.split(':');                  // 'cylinder:1' = the type's preset #1
    const pr = pi != null ? TYPES[type].presets[+pi] : null;
    const id = freshId(new Set(scene.components.map(c => c.id)), (pr ? pr.label.toLowerCase() : type).replace(/[^A-Za-z0-9_]/g, ''));
    const c = { id, type };
    if (pr) c.params = structuredClone(pr.params);
    const p = sel && find(scene, sel);
    if (p) {
      c.parent = p.id;
      const socks = Object.keys(compile(scene).defs.get(p.id)?.sockets || {});
      if (socks.length) c.socket = socks[0];
    }
    c.at = [0, 0, 0];
    commit(s => s.components.push(c));
    select(id);
  }
  /** Copy of the subtree with new ids and NO io: two components writing one tag is the top silent failure. */
  function duplicate() {
    const ids = subtree(scene, sel), taken = new Set(scene.components.map(c => c.id)), map = new Map();
    for (const id of ids) map.set(id, freshId(taken, id));
    commit(s => {
      for (const id of ids) {
        const k = structuredClone(find(s, id));
        k.id = map.get(id);
        delete k.io;
        if (map.has(k.parent)) k.parent = map.get(k.parent);
        else k.at = [(k.at?.[0] ?? 0) + 50, k.at?.[1] ?? 0, k.at?.[2] ?? 0];
        s.components.push(k);
      }
    });
    select(map.get(sel));
    msg('copied ' + ids.length + ' part(s) without io tags: assign new ones');
  }
  function remove() {
    const ids = new Set(subtree(scene, sel));
    if (ids.size > 1 && !confirm('Delete ' + sel + ' and the ' + (ids.size - 1) + ' part(s) mounted on it?')) return;
    commit(s => { s.components = s.components.filter(c => !ids.has(c.id)); });
  }
  function rename(old, id) {
    if (id === old) return;
    if (!ID_RE.test(id)) { msg('id must match ' + ID_RE, true); renderProps(); return; }
    if (find(scene, id)) { msg('id ' + id + ' is taken', true); renderProps(); return; }
    commit(s => { find(s, old).id = id; for (const c of s.components) if (c.parent === old) c.parent = id; });
    select(id);
  }
  const edit = fn => commit(s => fn(find(s, sel)));

  // ---------------------------------------------------------------- panels (rebuilt on user action only)
  function renderTree() {
    const box = $('ed-tree');
    box.replaceChildren();
    const ids = new Set(scene.components.map(c => c.id)), kids = new Map();
    for (const c of scene.components) {
      const k = c.parent != null && ids.has(c.parent) ? c.parent : '';
      if (!kids.has(k)) kids.set(k, []);
      kids.get(k).push(c);
    }
    const walk = (p, depth, seen) => {
      for (const c of kids.get(p) || []) {
        if (seen.has(c.id)) continue;
        seen.add(c.id);
        const row = el('div', { className: 'ed-row' + (c.id === sel ? ' sel' : ''), onclick: () => select(c.id) },
          el('b', { textContent: c.id }), ' ' + (TYPES[c.type]?.label || c.type));
        row.style.paddingLeft = 6 + depth * 14 + 'px';
        box.append(row);
        walk(c.id, depth + 1, seen);
      }
    };
    walk('', 0, new Set());
  }

  function field(label, ...inputs) {
    return el('label', { className: 'ed-f' }, el('span', { textContent: label }), el('span', { className: 'ed-in' }, ...inputs));
  }
  const num = (v, on, extra = {}) => el('input', { type: 'number', step: 'any', value: v ?? '', onchange: e => e.target.value !== '' && on(Number(e.target.value)), ...extra });
  const vec3 = (v, on) => [0, 1, 2].map(i => num(v?.[i] ?? 0, x => { const n = [...(v || [0, 0, 0])]; n[i] = x; on(n); }, { className: 'v3' }));
  const text = (v, on, extra = {}) => el('input', { type: 'text', value: v ?? '', onchange: e => on(e.target.value.trim()), ...extra });
  const select1 = (opts, v, on) => el('select', { onchange: e => on(e.target.value) },
    ...opts.map(([val, lab]) => el('option', { value: val, textContent: lab, selected: val === (v ?? '') })));

  function renderProps() {
    const box = $('ed-props');
    box.replaceChildren();
    const c = sel && find(scene, sel);
    if (!c) { box.append(el('p', { className: 'ed-hint', textContent: 'Click a part in 3D or in the tree. Add mounts the new part on the selected one.' })); return; }
    const t = TYPES[c.type], p = params(c);
    box.append(field('id', text(c.id, v => rename(c.id, v))));
    box.append(field('type', el('span', { textContent: t.label + ' (' + c.type + ')' })));
    box.append(field('label', text(c.label, v => edit(x => { if (v) x.label = v; else delete x.label; }))));
    const below = new Set(subtree(scene, c.id));
    box.append(field('parent', select1([['', '(world)'], ...scene.components.filter(x => !below.has(x.id)).map(x => [x.id, x.id])], c.parent,
      v => edit(x => {
        if (v) x.parent = v; else delete x.parent;
        const socks = v ? Object.keys(compile(scene).defs.get(v)?.sockets || {}) : [];
        if (socks.length) x.socket = socks[0]; else delete x.socket;
      }))));
    if (c.parent != null) {
      const socks = Object.keys(compile(scene).defs.get(c.parent)?.sockets || {});
      box.append(field('socket', select1([['', '(root link)'], ...socks.map(s => [s, s])], c.socket, v => edit(x => { if (v) x.socket = v; else delete x.socket; }))));
    }
    box.append(field('at [mm]', ...vec3(c.at, v => edit(x => { x.at = v; }))));
    box.append(field('rot [°]', ...vec3(c.rot, v => edit(x => { if (v.every(n => n === 0)) delete x.rot; else x.rot = v; }))));

    if (t.params.length) box.append(el('h4', { textContent: 'parameters' }));
    const setP = (k, v) => edit(x => { x.params = { ...(x.params || {}), [k]: v }; });
    for (const d of t.params) {
      const v = p[d.k], lab = d.k + (d.unit ? ' [' + d.unit + ']' : '');
      if (d.type === 'num') box.append(field(lab, num(v, x => setP(d.k, x), { min: d.min ?? '', max: d.max ?? '' })));
      else if (d.type === 'enum') box.append(field(lab, select1(d.of.map(o => [o, o]), v, x => setP(d.k, x))));
      else if (d.type === 'vec3') box.append(field(lab, ...vec3(v, x => setP(d.k, x))));
      else if (d.type === 'bool') box.append(field(lab, el('input', { type: 'checkbox', checked: !!v, onchange: e => setP(d.k, e.target.checked) })));
      else if (d.type === 'str') box.append(field(lab, text(v, x => setP(d.k, x))));
      // `of: 'part'` is a descriptor, not a type name: a feeder can emit a workpiece or a pallet.
      else if (d.type === 'ref') box.append(field(lab, select1([['', '(none)'],
        ...scene.components.filter(x => (d.of === 'part' ? TYPES[x.type]?.part : x.type === d.of)).map(x => [x.id, x.id])], v, x => setP(d.k, x))));
      else box.append(field(lab, text(JSON.stringify(v), x => { try { setP(d.k, JSON.parse(x)); } catch { msg(d.k + ': not valid JSON', true); } }, { title: 'JSON' })));
    }

    const schema = t.io ? t.io(p) : {};
    if (Object.keys(schema).length) box.append(el('h4', { textContent: 'io tags' }));
    for (const [k, s] of Object.entries(schema)) {
      box.append(field(k + ' ' + (s.dir === 'out' ? 'PLC→' : '→PLC') + ' ' + s.type,
        text(c.io?.[k], v => edit(x => { x.io = { ...(x.io || {}) }; if (v) x.io[k] = v; else delete x.io[k]; if (!Object.keys(x.io).length) delete x.io; }),
          { list: 'ed-tags', placeholder: 'tag name', spellcheck: false })));
    }
  }

  function renderErrors() {
    const ul = $('ed-errs');
    ul.replaceChildren(...validate(scene).map(e => el('li', { textContent: e })));
    $('ed-errs-h').textContent = ul.children.length ? ul.children.length + ' problem(s): Save is blocked' : 'no problems';
  }

  async function loadTags() {
    const known = new Set(bindings(scene).map(b => b.tag));
    try { for (const t of await (await fetch('/api/tags')).json()) known.add(t.tag); } catch { /* scene tags are enough */ }
    $('ed-tags').replaceChildren(...[...known].sort().map(t => el('option', { value: t })));
  }

  // ---------------------------------------------------------------- save, enter, exit
  async function save() {
    const errs = validate(scene);
    if (errs.length) { msg('fix ' + errs.length + ' problem(s) first', true); return; }
    msg('saving…');
    const r = await fetch('/api/scene/' + encodeURIComponent(scene.name),
      { method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ baseVersion: base, scene }) });
    const j = await r.json().catch(() => ({}));
    if (r.status === 409) { msg('Someone saved this scene after you opened it. Exit (discarding your edits) and edit again to get their version.', true); return; }
    if (!r.ok) { msg((j.error || 'HTTP ' + r.status) + (j.errors ? ': ' + j.errors.join('; ') : ''), true); return; }
    base = j.version;
    saved = stringify(scene);
    msg(!j.changed ? 'no changes' : j.rebuildError ? 'saved, but the plant could not rebuild: ' + j.rebuildError : 'saved' + (j.rebuilt ? '; plant rebuilt (press Run)' : ''), !!j.rebuildError);
    refresh();
  }

  async function enter() {
    const name = ctx.sceneName();
    if (!name) return;
    const r = await fetch('/api/scene/' + encodeURIComponent(name));
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { msg('cannot edit: ' + (j.error || r.status), true); return; }
    await ctx.post('/api/cmd', { op: 'stop' });              // edit mode stops the plant
    base = j.version;
    saved = stringify(j.scene);
    undo.length = redo.length = 0;
    sel = null;
    ed.active = true;
    show(true);
    setScene(j.scene);
    loadTags();
    msg('editing ' + name + ' (plant stopped)');
  }
  function exit() {
    if (dirty() && !confirm('Discard unsaved edits?')) return;
    ed.active = false;
    sel = null;
    tc.detach(); helper.visible = false; boxH.visible = false;
    show(false);
    ctx.rebuild(ctx.serverScene());
  }
  /** The server broadcast a scene while editing (our save, or someone else's). @returns {boolean} handled */
  function onServerScene(sc) {
    if (!ed.active) return false;
    if (stringify(sc) === saved) return true;                // our own save coming back
    msg('The server scene changed (another editor?). Saving now answers 409.', true);
    return true;
  }
  function show(on) {
    $('editor').hidden = !on;
    document.querySelector('.tablewrap').hidden = on;
    $('edit-btn').textContent = on ? 'Exit edit' : 'Edit';
    // no swapping the scene or the controller out from under an edit
    for (const id of ['scene-pick', 'mode-pick']) $(id).disabled = on;
  }

  // ---------------------------------------------------------------- toolbar and keys
  $('ed-type').replaceChildren(...Object.entries(TYPES).flatMap(([k, t]) => [
    el('option', { value: k, textContent: t.label }),
    ...(t.presets || []).map((pr, i) => el('option', { value: k + ':' + i, textContent: '  ' + t.label + ' – ' + pr.label })),
  ]));
  $('edit-btn').onclick = () => (ed.active ? exit() : enter());
  $('ed-add').onclick = add;
  $('ed-dup').onclick = duplicate;
  $('ed-del').onclick = remove;
  $('ed-undo').onclick = () => step(undo, redo);
  $('ed-redo').onclick = () => step(redo, undo);
  $('ed-save').onclick = save;
  $('ed-move').onclick = () => { mode = 'translate'; placeProxy(); };
  $('ed-rot').onclick = () => { mode = 'rotate'; placeProxy(); };
  $('ed-rsnap').onchange = placeProxy;
  addEventListener('keydown', e => {
    if (!ed.active || /^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName)) return;
    const k = e.key.toLowerCase(), mod = e.ctrlKey || e.metaKey;
    if (mod && k === 'z' && !e.shiftKey) step(undo, redo);
    else if (mod && (k === 'y' || (k === 'z' && e.shiftKey))) step(redo, undo);
    else if (mod && k === 's') save();
    else if (mod && k === 'd' && sel) duplicate();
    else if (k === 'delete' && sel) remove();
    else if (k === 'w') { mode = 'translate'; placeProxy(); }
    else if (k === 'e') { mode = 'rotate'; placeProxy(); }
    else if (k === 'escape') select(null);
    else return;
    e.preventDefault();
  });
  addEventListener('beforeunload', e => { if (ed.active && dirty()) e.preventDefault(); });

  return ed;
}
