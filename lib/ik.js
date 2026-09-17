// Numeric inverse kinematics over a chain of `joint` components - for BUILD scripts and tests.
// The plant never solves IK: a scene's controller carries joint-angle tables the way the real
// program does (rb4axis keeps its IK in the PLC; blurobot keeps a pose table). This solver is
// damped least squares on a finite-difference Jacobian of worldPoses(), so it has no kinematic
// model of its own and cannot disagree with the picture the viewer draws.
//
// A goal is a point plus up to two directions: where the tool axis must point (`dir`, e.g.
// straight down for a suction head) and where the head's own X must point (`along`, the row of
// cups). Both are optional; a bare point is a 3-DoF goal and the arm picks any orientation.
import { worldPoses } from './scene.js';
import { apply, qrot } from './math.js';

/**
 * @param {any} scene
 * @param {Array<{id: string, min: number, max: number}>} joints chain order, base first
 * @param {{id: string, link?: string, at?: number[], dir?: number[], along?: number[]}} tool the TCP in a link frame
 * @param {{p: number[], dir?: number[], along?: number[]}} goal world
 * @param {{start?: Record<string, number>, iters?: number, tol?: number, lambda?: number, step?: number}} [opts]
 * @returns {{dof: Record<string, number>, err: number, dirErr: number, iters: number, ok: boolean}}
 */
export function solveIk(scene, joints, tool, goal, opts = {}) {
  const { iters = 300, tol = 0.05, lambda = 8, step = 25 } = opts;
  const link = tool.link ?? 'arm', at = tool.at ?? [0, 0, 0];
  const tdir = tool.dir ?? [0, 0, 1], talong = tool.along ?? [1, 0, 0];
  const wantDir = goal.dir ? unit(goal.dir) : null, wantAlong = goal.along ? unit(goal.along) : null;
  /** @type {Record<string, number>} */
  const dof = {};
  for (const j of joints) dof[j.id] = opts.start?.[j.id] ?? 0;

  /** TCP position and the two tool directions in the world, for a dof set. @param {Record<string, number>} d */
  const fk = d => {
    const F = worldPoses(scene, d)[tool.id][link];
    return { p: apply(F, at), dir: qrot(F.q, tdir), along: qrot(F.q, talong) };
  };
  /** The residual the solver drives to zero: mm for position, unitless for directions. */
  const residual = (/** @type {{p: number[], dir: number[], along: number[]}} */ f) => {
    const e = [goal.p[0] - f.p[0], goal.p[1] - f.p[1], goal.p[2] - f.p[2]];
    const K = 200;                                   // 1.0 of direction error weighs like 200 mm
    if (wantDir) e.push(...[0, 1, 2].map(i => K * (wantDir[i] - f.dir[i])));
    if (wantAlong) e.push(...[0, 1, 2].map(i => K * (wantAlong[i] - f.along[i])));
    return e;
  };

  let it = 0, f = fk(dof), e = residual(f);
  for (; it < iters; it++) {
    const posErr = Math.hypot(e[0], e[1], e[2]);
    const dirErr = Math.hypot(...e.slice(3)) / 200;
    if (posErr < tol && dirErr < 2e-4) break;
    // Jacobian by central differences, one column per joint, in degrees.
    const n = joints.length, m = e.length, h = 0.05;
    /** @type {number[][]} */
    const J = Array.from({ length: m }, () => new Array(n).fill(0));
    for (let c = 0; c < n; c++) {
      const id = joints[c].id, x0 = dof[id];
      dof[id] = x0 + h; const ep = residual(fk(dof));
      dof[id] = x0 - h; const em = residual(fk(dof));
      dof[id] = x0;
      for (let r = 0; r < m; r++) J[r][c] = (em[r] - ep[r]) / (2 * h);   // d(residual)/d(theta) = -d(f)/d(theta)
    }
    // Damped least squares: dtheta = J^T (J J^T + lambda^2 I)^-1 e
    const A = Array.from({ length: m }, (_, r) => Array.from({ length: m }, (_, c) => {
      let s = 0; for (let k = 0; k < n; k++) s += J[r][k] * J[c][k];
      return s + (r === c ? lambda * lambda : 0);
    }));
    const y = solve(A, e);
    for (let c = 0; c < n; c++) {
      // J here is d(f)/d(theta) (the residual is goal - f, and the differences were taken as
      // e(-h) - e(+h)), so the update is +J^T y. With the sign wrong it walked the arm AWAY from
      // the goal until the limits stopped it: measured, 1191 mm off after 300 iterations.
      let d = 0; for (let r = 0; r < m; r++) d += J[r][c] * y[r];
      d = Math.max(-step, Math.min(step, d));
      const j = joints[c];
      dof[j.id] = Math.max(j.min, Math.min(j.max, dof[j.id] + d));
    }
    f = fk(dof); e = residual(f);
  }
  const err = Math.hypot(e[0], e[1], e[2]), dirErr = Math.hypot(...e.slice(3)) / 200;
  return { dof, err, dirErr, iters: it, ok: err < tol && dirErr < 2e-4 };
}

/** @param {number[]} v */
function unit(v) { const n = Math.hypot(v[0], v[1], v[2]) || 1; return v.map(x => x / n); }

/** Gaussian elimination with partial pivoting, small dense systems only. @param {number[][]} A @param {number[]} b */
function solve(A, b) {
  const n = b.length, M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    const d = M[c][c] || 1e-12;
    for (let r = c + 1; r < n; r++) { const k = M[r][c] / d; for (let k2 = c; k2 <= n; k2++) M[r][k2] -= k * M[c][k2]; }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) { let s = M[r][n]; for (let c = r + 1; c < n; c++) s -= M[r][c] * x[c]; x[r] = s / (M[r][r] || 1e-12); }
  return x;
}
