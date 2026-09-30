// @ts-check
// Reads an iCAD SX "3D Browser" web export (one self-contained .html) into bodies, meshes and the
// assembly tree - the way the lathe-line's robot came from its maker's STEP files, except that an
// SPM builder's own machine only exists as this export. Nothing here is documented by the vendor:
// the layout below was worked out against the file's own bounding boxes (every body's triangles
// land exactly inside the box the file stores for it), so tools/gen_ce_insert.js checks that again
// on every run and refuses a file that does not.
//
// The page carries its model as script blocks `_ZZZ[n] = [...]`:
//   [1]  99 unit vectors, flat: a body's rotation is three of them (its X, Y and Z axes)
//   [2]  bodies, 21 numbers each: [quant, body, 0, r, g, b, a, bbox min xyz, bbox max xyz,
//        centre xyz, axis X, axis Y, axis Z (indices into [1]), ?, owner node]
//   [3]  quantisation, 5 each: [sx, sy, sz, mesh, 1]
//   [4]  meshes, 6 each: fields 4 and 5 are the first index and the index count in [5]
//   [5]  Uint32 triangle STRIPS, 0xFFFFFFFF restarts a strip
//   [6]  Int16 vertices, 6 each (position, then a packed normal we do not need)
//   [10] tree, 7 each: [id, ?, parent, next sibling, first child, name pair, flag]
//   [11] name pairs into [12]; [12] names as character codes
// A vertex is  centre + axisX * (int16 x * sx) + axisY * (y * sy) + axisZ * (z * sz).
//
// iCAD is Y-UP. The scene is Z-up in mm, so a point comes out as (x, -z, y + lift): a turn about X,
// never a mirror. `lift` puts the lowest point of the model on the floor.
import fs from 'node:fs';
import vm from 'node:vm';

const RESTART = 4294967295;

/**
 * @param {string} file the iCAD .html
 * @returns {{nodes: Array<{id: number, name: string, desc: string, parent: number, kids: number[], bodies: number[]}>,
 *            bodies: Array<{id: number, node: number, color: number[], box: number[]}>,
 *            tris: (b: number) => number[][], parts: (b: number) => Array<{color: number[], tris: number[][]}>, lift: number}}
 */
export function readIcad(file) {
  const html = fs.readFileSync(file, 'utf8');
  /** @type {any} */
  const ctx = { _ZZZ: [] };
  vm.createContext(ctx);
  let n = 0;
  for (const m of html.matchAll(/<script id="_S(\d+)">([\s\S]*?)<\/script>/g)) { vm.runInContext(m[2], ctx); n++; }
  const Z = ctx._ZZZ;
  if (!n || !Z[2] || !Z[10]) throw new Error(file + ': not an iCAD SX 3D Browser export (no _ZZZ model data)');
  const U = Z[1], B = Z[2], Q = Z[3], M = Z[4], I = Z[5], V = Z[6], T = Z[10], P = Z[11];
  const names = Z[12].map((/** @type {any} */ c) => (Array.isArray(c) ? Buffer.from(c).toString('latin1') : String(c)));
  const nNode = T.length / 7, nBody = B.length / 21;
  const nodes = [];
  for (let i = 0; i < nNode; i++) {
    const pair = T[i * 7 + 5];
    nodes.push({ id: i, name: names[P[pair * 2]] ?? '', desc: P[pair * 2 + 1] ? names[P[pair * 2 + 1]] ?? '' : '',
                 parent: T[i * 7 + 2], kids: /** @type {number[]} */ ([]), bodies: /** @type {number[]} */ ([]) });
  }
  for (const x of nodes) if (x.id > 0 && x.parent !== x.id && nodes[x.parent]) nodes[x.parent].kids.push(x.id);
  const lift = -Z[0][4];                                 // the model's lowest Y, which becomes the floor
  const out = (/** @type {number[]} */ p) => [p[0], -p[2], p[1] + lift];
  const bodies = [];
  for (let b = 1; b < nBody; b++) {
    const r = B.slice(b * 21, b * 21 + 21);
    const lo = out(r.slice(7, 10)), hi = out(r.slice(10, 13));
    const box = [0, 1, 2].map(d => Math.min(lo[d], hi[d])).concat([0, 1, 2].map(d => Math.max(lo[d], hi[d])));
    bodies.push({ id: b, node: r[20], color: [r[3], r[4], r[5]], box });
    nodes[r[20]]?.bodies.push(b);
  }
  const vec = (/** @type {number} */ k) => [U[k * 3], U[k * 3 + 1], U[k * 3 + 2]];
  /**
   * The body's triangles in scene coordinates (mm), three points each, split by colour. A body is
   * `Q[q*5+4]` consecutive meshes from `Q[q*5+3]`, and each mesh carries its OWN colour ([4]: r g b
   * a start count; a = 0 means the body's colour) - a cylinder is a white body with a blue tube, a
   * bracket a grey body with a green face. Reading only the first mesh drew the rest missing and
   * everything in the body's colour, which for bought-in parts is white.
   * @param {number} b @returns {Array<{color: number[], tris: number[][]}>}
   */
  const parts = b => {
    const r = B.slice(b * 21, b * 21 + 21), q = r[0], m0 = Q[q * 5 + 3], nm = Q[q * 5 + 4] || 1;
    /** @type {Map<string, {color: number[], tris: number[][]}>} */
    const by = new Map();
    for (let k = 0; k < nm; k++) {
      const mesh = m0 + k, a = M[mesh * 6 + 3];
      const color = a > 0 ? [M[mesh * 6], M[mesh * 6 + 1], M[mesh * 6 + 2]] : [r[3], r[4], r[5]];
      const key = color.join();
      const g = by.get(key) || by.set(key, { color, tris: [] }).get(key);
      /** @type {any} */ (g).tris.push(...meshTris(r, q, mesh));
    }
    return [...by.values()];
  };
  /** @param {number} b */
  const tris = b => parts(b).flatMap(p => p.tris);
  /** @param {number[]} r @param {number} q @param {number} mesh */
  const meshTris = (r, q, mesh) => {
    const s = Q.slice(q * 5, q * 5 + 3);
    const st = M[mesh * 6 + 4], cnt = M[mesh * 6 + 5];
    const ax = [vec(r[16]), vec(r[17]), vec(r[18])], c = [r[13], r[14], r[15]];
    const w = (/** @type {number} */ i) => {
      const l = [V[i * 6] * s[0], V[i * 6 + 1] * s[1], V[i * 6 + 2] * s[2]];
      return out([0, 1, 2].map(d => c[d] + ax[0][d] * l[0] + ax[1][d] * l[1] + ax[2][d] * l[2]));
    };
    /** @type {number[][]} */
    const res = [];
    /** @type {number[]} */
    let strip = [];
    const flush = () => {
      for (let k = 2; k < strip.length; k++) {
        const [a, bb, cc] = k % 2 ? [strip[k - 1], strip[k - 2], strip[k]] : [strip[k - 2], strip[k - 1], strip[k]];
        if (a !== bb && bb !== cc && a !== cc) res.push(w(a), w(bb), w(cc));
      }
      strip = [];
    };
    for (let k = st; k < st + cnt; k++) { if (I[k] === RESTART) flush(); else strip.push(I[k]); }
    flush();
    return res;
  };
  return { nodes, bodies, tris, parts, lift };
}

/**
 * Binary STL of triangles given as consecutive point triples (mm). Normals are left zero: the
 * viewer computes its own (web/app.js).
 * @param {number[][]} pts
 */
export function stlBinary(pts) {
  const n = Math.floor(pts.length / 3), buf = Buffer.alloc(84 + n * 50);
  buf.write('mio shell', 0, 'latin1');
  buf.writeUInt32LE(n, 80);
  let o = 84;
  for (let t = 0; t < n; t++) {
    o += 12;
    for (let v = 0; v < 3; v++) for (let d = 0; d < 3; d++) { buf.writeFloatLE(pts[t * 3 + v][d], o); o += 4; }
    o += 2;
  }
  return buf;
}
