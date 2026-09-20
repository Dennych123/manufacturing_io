// Walk into the cell: a first-person camera with WASD, mouse look, run, crouch and jump.
//
// It is a CAMERA and nothing else. It never touches a DOF, never writes a tag and never enters
// the plant: the man in the hall is not a body Rapier knows about, he is where you are standing to
// look at the machine from. Pressing a button or grabbing a part from in here goes through exactly
// the same /api/press and /api/hold the orbit camera uses, so a run replays the same either way.
//
// Why the camera is not PointerLockControls: that addon is Y-up only. Its look math builds the
// camera orientation through a 'YXZ' rotation and moveForward() says so in its own comment
// ("assumes camera.up is y-up"). This repo is Z-up everywhere, so the addon tips the horizon over
// on the first mouse move. Yaw about world Z and pitch clamped short of the pole is all that is
// needed, and lookAt() with DEFAULT_UP = +Z builds the orientation from it.
import * as THREE from 'three';

const EYE = 1650, EYE_CROUCH = 1050;   // mm: standing and crouched eye height above the feet
const RADIUS = 320;                    // how close the body gets to a wall or a machine frame
const STEP_UP = 260;                   // a step or a kerb this high is walked up, not blocked
const G = 9810;                        // mm/s^2
const WALK = 1500, RUN = 3600, CREEP = 700, JUMP = 3050;   // JUMP clears ~470 mm
const LOOK = 0.0022;                   // radians per pixel of mouse movement
const PITCH_MAX = 1.5;                 // short of straight up: no pole, no flipped horizon

/**
 * @param ctx.camera      the viewer's own camera - borrowed while walking, restored on the way out
 * @param ctx.dom         the canvas (pointer lock is requested on it)
 * @param ctx.controls    OrbitControls, disabled while walking
 * @param ctx.solids      () => Object3D[] to stand on and bump into (the machine and the workshop)
 * @param ctx.onChange    (active) => void, for the HUD and the panel
 */
export function createPov(ctx) {
  const { camera, dom, controls } = ctx;
  const pov = { active: false, enter, exit, toggle, update, spawnFrom, keyHeld };

  const pos = new THREE.Vector3(0, -4000, 0);      // FEET, in mm, world
  let yaw = Math.PI / 2, pitch = -0.05, vz = 0, onGround = true, bob = 0, eye = EYE, jumpWanted = false;
  const keys = new Set();
  const saved = { p: new THREE.Vector3(), q: new THREE.Quaternion(), t: new THREE.Vector3() };
  const ray = new THREE.Raycaster();
  const dir = new THREE.Vector3(), side = new THREE.Vector3(), step = new THREE.Vector3(), look = new THREE.Vector3();
  // Hoisted: update() runs in the render loop, and a Vector3 a frame is a Vector3 the collector
  // has to take back sixty times a second for nothing.
  const from = new THREE.Vector3(), probe = new THREE.Vector3(), dirTo = new THREE.Vector3();
  const DOWN = new THREE.Vector3(0, 0, -1);
  let spawn = null;

  const keyHeldSet = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD', 'Space', 'ShiftLeft', 'ShiftRight', 'KeyC', 'ControlLeft']);
  function keyHeld(code) { return keys.has(code); }

  /**
   * Where to stand when the walk starts: a few paces in front of the machine, outside it, with
   * the eye already on it. Standing level looks over a conveyor deck entirely - the machine is
   * knee to chest height and the eye is at 1650 - so the pitch is aimed at the middle of it.
   */
  function spawnFrom(box) {
    if (!box || box.isEmpty()) return;
    const c = box.getCenter(new THREE.Vector3()), s = box.getSize(new THREE.Vector3());
    const back = s.y / 2 + Math.max(2600, s.z * 1.2);
    spawn = { x: c.x, y: c.y - back, yaw: Math.PI / 2, pitch: Math.atan2(c.z - EYE, back) };
  }

  function reset() {
    if (spawn) { pos.set(spawn.x, spawn.y, 0); yaw = spawn.yaw; pitch = spawn.pitch; }
    else pitch = -0.05;
    vz = 0; onGround = true;
  }

  function enter() {
    if (pov.active) return;
    pov.active = true;
    saved.p.copy(camera.position); saved.q.copy(camera.quaternion); saved.t.copy(controls.target);
    controls.enabled = false;
    reset();
    addEventListener('keydown', onKey, true);
    addEventListener('keyup', onKey, true);
    document.addEventListener('pointerlockchange', onLock);
    dom.addEventListener('mousemove', onMove);
    // Pointer lock can be refused (a headless browser, an embedded frame, a user gesture the
    // browser did not like). The walk still works from the keyboard when it is; what must not
    // happen is an unhandled rejection in the console every time.
    try { dom.requestPointerLock?.()?.catch?.(() => {}); } catch { /* look stays where it is */ }
    ctx.onChange?.(true);
  }

  function exit() {
    if (!pov.active) return;
    pov.active = false;
    keys.clear();
    jumpWanted = false;
    removeEventListener('keydown', onKey, true);
    removeEventListener('keyup', onKey, true);
    document.removeEventListener('pointerlockchange', onLock);
    dom.removeEventListener('mousemove', onMove);
    if (document.pointerLockElement === dom) document.exitPointerLock();
    camera.position.copy(saved.p); camera.quaternion.copy(saved.q);
    controls.target.copy(saved.t);
    controls.enabled = true;
    controls.update();
    ctx.onChange?.(false);
  }

  function toggle() { if (pov.active) exit(); else enter(); }

  // Esc leaves pointer lock, and leaving pointer lock leaves the walk: one way out, the one every
  // browser already teaches.
  function onLock() { if (pov.active && document.pointerLockElement !== dom) exit(); }

  function onKey(e) {
    if (/^(INPUT|SELECT|TEXTAREA)$/.test(e.target?.tagName)) return;
    const down = e.type === 'keydown';
    if (e.code === 'KeyR' && down) { reset(); e.preventDefault(); return; }
    if (!keyHeldSet.has(e.code)) return;
    if (down) keys.add(e.code); else keys.delete(e.code);
    // A jump is an EDGE, held until the next frame can use it. Measured headless at about 8 fps:
    // a 120 ms tap of Space landed entirely between two frames and nothing happened, because the
    // key was already up again when the frame asked whether it was down.
    if (e.code === 'Space' && down) jumpWanted = true;
    e.preventDefault();                    // Space scrolls the page otherwise, and it is the jump
  }

  function onMove(e) {
    if (document.pointerLockElement !== dom) return;
    yaw -= e.movementX * LOOK;
    pitch = Math.max(-PITCH_MAX, Math.min(PITCH_MAX, pitch - e.movementY * LOOK));
  }

  /**
   * The first SOLID thing a ray hits among the machine and the workshop, or null. A zone drawn as
   * a ghost box (a remover, a sensor's beam volume) is not a thing to walk into: it is drawn
   * see-through precisely because it is not there.
   */
  function hit(at, into, far) {
    ray.set(at, into);
    ray.far = far;
    const list = ctx.solids();
    if (!list.length) return null;
    for (const h of ray.intersectObjects(list, true)) if (!h.object.userData.ghost) return h;
    return null;
  }

  /**
   * May the body stand at x,y? It may if the surface there is no more than a kerb above the one
   * it is on (so a deck, a bench top or a stack of pallets stops it, and a shallow step does
   * not), and if nothing tall is in the way between here and there.
   */
  function free(x, y, d) {
    probe.set(x, y, pos.z + EYE);
    const g = hit(probe, DOWN, EYE + 500);
    if (g && g.point.z - pos.z > STEP_UP) return false;
    // A wall or a column has no top under the eye to find, so it takes a ray of its own. Two
    // heights: one at the waist, one at the chest, and the shin is the probe above.
    dirTo.set(x - pos.x, y - pos.y, 0);
    if (dirTo.lengthSq() < 1e-12) return true;
    dirTo.normalize();
    for (const h of [900, 1500]) {
      from.set(pos.x, pos.y, pos.z + h);
      if (hit(from, dirTo, RADIUS + d)) return false;
    }
    return true;
  }

  function update(dt) {
    if (!pov.active) return;
    dt = Math.min(dt, 0.05);                                  // a tab that was in the background
    const crouch = keys.has('KeyC') || keys.has('ControlLeft');
    const run = keys.has('ShiftLeft') || keys.has('ShiftRight');
    const speed = crouch ? CREEP : run ? RUN : WALK;

    // ---- where the feet want to go: forward is the heading, flat, whatever the pitch is doing
    dir.set(Math.cos(yaw), Math.sin(yaw), 0);
    side.set(dir.y, -dir.x, 0);
    step.set(0, 0, 0);
    if (keys.has('KeyW')) step.add(dir);
    if (keys.has('KeyS')) step.sub(dir);
    if (keys.has('KeyD')) step.add(side);
    if (keys.has('KeyA')) step.sub(side);
    const moving = step.lengthSq() > 0;
    if (moving) {
      step.normalize();
      const d = speed * dt;
      // Forward rays alone do not find a machine. Measured on `a-to-b` with rays at 300, 900 and
      // 1500 mm: a belt deck sits at 800 with thin legs under it, so every ray passed over or
      // under the conveyor and six seconds of walking covered 8660 mm of a possible 9000 -
      // straight through it. What catches a deck, a bench or a pallet is the test a real step
      // makes: probe DOWN where the foot is about to land, and refuse the step when the surface
      // there is more than a kerb above the one you are on. Forward rays stay for what that
      // cannot see - a wall or a column, whose top is over your head.
      if (!free(pos.x + step.x * d, pos.y + step.y * d, d)) {
        // Blocked: keep whichever single axis is still free, which is what sliding along a
        // machine frame with a shoulder to it comes to.
        if (step.x && free(pos.x + step.x * d, pos.y, d)) step.y = 0;
        else if (step.y && free(pos.x, pos.y + step.y * d, d)) step.x = 0;
        else step.set(0, 0, 0);
      }
      pos.addScaledVector(step, speed * dt);
      if (onGround) bob += dt * (run ? 13 : 9);
    }

    // ---- gravity, the floor, and anything you can stand on top of
    if (jumpWanted && onGround) { vz = JUMP; onGround = false; jumpWanted = false; }
    else if (!keys.has('Space')) jumpWanted = false;
    vz -= G * dt;
    pos.z += vz * dt;
    probe.set(pos.x, pos.y, pos.z + STEP_UP);
    const ground = hit(probe, DOWN, STEP_UP + Math.max(60, -vz * dt + 60));
    if (ground && vz <= 0) { pos.z = ground.point.z; vz = 0; onGround = true; }
    else if (pos.z <= 0 && vz <= 0) { pos.z = 0; vz = 0; onGround = true; }
    else onGround = false;

    // ---- the head: crouch eased, and a little bob so walking reads as walking
    const want = crouch ? EYE_CROUCH : EYE;
    eye += (want - eye) * Math.min(1, dt * 12);
    const h = eye + (moving && onGround ? Math.sin(bob) * 14 : 0);
    camera.position.set(pos.x, pos.y, pos.z + h);
    const cp = Math.cos(pitch);
    look.set(camera.position.x + cp * Math.cos(yaw), camera.position.y + cp * Math.sin(yaw), camera.position.z + Math.sin(pitch));
    camera.lookAt(look);                                     // DEFAULT_UP is +Z: no rotation order to get wrong
  }

  return pov;
}
