// @ts-check
// Poses in mm, Z-up, quaternions (x, y, z, w). Shared by Node and the browser, so no three.
//
// THE rotation rule, written once: `rot: [rx, ry, rz]` in degrees is applied about the fixed
// axes X first, then Y, then Z, i.e. q = qz * qy * qx. three's Euler 'XYZ' is the opposite
// order, which is why nothing outside this file ever turns `rot` into a rotation.

/** @typedef {[number, number, number]} V3 */
/** @typedef {[number, number, number, number]} Q */
/** @typedef {{p: V3, q: Q}} Pose */

export const DEG = Math.PI / 180;

/** @param {Q} a @param {Q} b @returns {Q} */
export function qmul(a, b) {
  const [ax, ay, az, aw] = a, [bx, by, bz, bw] = b;
  return [aw * bx + ax * bw + ay * bz - az * by,
          aw * by - ax * bz + ay * bw + az * bx,
          aw * bz + ax * by - ay * bx + az * bw,
          aw * bw - ax * bx - ay * by - az * bz];
}

/** @param {Q} q @returns {Q} */
export function qnorm(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

/** Rotation of `deg` degrees about `axis`. @param {V3} axis @param {number} deg @returns {Q} */
export function qaxis(axis, deg) {
  const h = deg * DEG / 2, s = Math.sin(h), n = Math.hypot(axis[0], axis[1], axis[2]) || 1;
  return [axis[0] / n * s, axis[1] / n * s, axis[2] / n * s, Math.cos(h)];
}

/** `rot` in degrees -> quaternion, X then Y then Z about fixed axes. @param {number[]|undefined} rot @returns {Q} */
export function qeuler(rot) {
  if (!rot) return [0, 0, 0, 1];
  return qmul(qaxis([0, 0, 1], rot[2] || 0), qmul(qaxis([0, 1, 0], rot[1] || 0), qaxis([1, 0, 0], rot[0] || 0)));
}

/**
 * Quaternion -> `rot` degrees, the inverse of qeuler (q = qz * qy * qx). ry is in [-90, 90]; at
 * ry = ±90 (gimbal lock) rx is 0 and rz carries the whole turn.
 * @param {Q} q @returns {V3}
 */
export function eulerOf(q) {
  const [x, y, z, w] = qnorm(q);
  const r20 = 2 * (x * z - w * y);
  if (Math.abs(r20) > 1 - 1e-9) {
    return [0, -Math.sign(r20) * 90, Math.atan2(-2 * (x * y - w * z), 1 - 2 * (x * x + z * z)) / DEG];
  }
  return [Math.atan2(2 * (y * z + w * x), 1 - 2 * (x * x + y * y)) / DEG, Math.asin(-r20) / DEG,
          Math.atan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z)) / DEG];
}

/** Rotates vector v by q. @param {Q} q @param {number[]} v @returns {V3} */
export function qrot(q, v) {
  const [x, y, z, w] = q, [vx, vy, vz] = v;
  const tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
  return [vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx];
}

/** @param {number[]} [at] @param {number[]} [rot] @returns {Pose} */
export function pose(at, rot) {
  return { p: at ? [at[0] || 0, at[1] || 0, at[2] || 0] : [0, 0, 0], q: qeuler(rot) };
}

export const IDENTITY = Object.freeze(pose());

/** a then b: b is expressed in a's frame. @param {Pose} a @param {Pose} b @returns {Pose} */
export function compose(a, b) {
  const r = qrot(a.q, b.p);
  return { p: [a.p[0] + r[0], a.p[1] + r[1], a.p[2] + r[2]], q: qnorm(qmul(a.q, b.q)) };
}

/** @param {Pose} a @returns {Pose} */
export function invert(a) {
  /** @type {Q} */
  const qi = [-a.q[0], -a.q[1], -a.q[2], a.q[3]];
  const p = qrot(qi, a.p);
  return { p: [-p[0], -p[1], -p[2]], q: qi };
}

/** Point in a's frame -> world. @param {Pose} a @param {number[]} v @returns {V3} */
export function apply(a, v) {
  const r = qrot(a.q, v);
  return [a.p[0] + r[0], a.p[1] + r[1], a.p[2] + r[2]];
}

/** @param {number} x @param {number} lo @param {number} hi */
export const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
