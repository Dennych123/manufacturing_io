// Phase 0 smoke test: the Rapier features the plant is built on, in Node, Z-up, metres.
// Kinematic bodies moving and pushing, castRay, intersectionsWithShape, determinism.
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

let R;
try {
  const mod = await import('@dimforge/rapier3d-deterministic-compat');
  R = mod.default ?? mod;
  await R.init();
} catch (e) {
  console.log('  SKIP  Rapier not loadable (' + String(e.message || e).split('\n')[0] + '): run npm install');
  process.exit(0);
}

const DT = 0.002;                               // docs/PLAN.md §5: 2 ms plant step

/** Floor, a resting ball, and a kinematic pusher sweeping +X through it. */
function scenario(steps, partCanSleep = false) {
  const w = new R.World({ x: 0, y: 0, z: -9.81 });
  w.timestep = DT;
  const floor = w.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(0, 0, -0.05));
  w.createCollider(R.ColliderDesc.cuboid(2, 2, 0.05), floor);
  const ball = w.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(0, 0, 0.2).setCanSleep(partCanSleep));
  const ballCol = w.createCollider(R.ColliderDesc.ball(0.02).setFriction(0.6), ball);
  const pusher = w.createRigidBody(R.RigidBodyDesc.kinematicPositionBased().setTranslation(-0.3, 0, 0.03));
  w.createCollider(R.ColliderDesc.cuboid(0.01, 0.05, 0.03), pusher);
  const trace = [];
  for (let i = 0; i < steps; i++) {
    const t = i * DT;
    // stay still for 1 s so the ball settles, then sweep 0.6 m in 1 s
    const x = t < 1 ? -0.3 : Math.min(0.3, -0.3 + (t - 1) * 0.6);
    pusher.setNextKinematicTranslation({ x, y: 0, z: 0.03 });
    w.step();
    if (i === 499) trace.push(ball.translation().z);        // resting height after 1 s
  }
  return { w, ball, ballCol, pusher, trace };
}

const a = scenario(1000);
chk('ball rests on the floor, Z-up (z ~ radius 0.02)', Math.abs(a.trace[0] - 0.02) < 0.002, 'z=' + a.trace[0].toFixed(4));
chk('kinematic body reached its target', Math.abs(a.pusher.translation().x - (-0.3 + (2 - 1 - DT) * 0.6)) < 0.01,
  'x=' + a.pusher.translation().x.toFixed(4));

// The trap, measured: a part that fell asleep while resting is NOT woken by a kinematic
// pusher. The pusher passes through it, and the part stays put without any error. Waking
// the pusher does not help. Hence the rule: workpieces never sleep. If this check starts
// failing, Rapier fixed it and the rule can be revisited.
const trap = scenario(1500, true);
chk('trap: a SLEEPING part is not pushed by a kinematic body (Rapier 0.20)', trap.ball.translation().x === 0,
  'ball x=' + trap.ball.translation().x.toFixed(3));

const b = scenario(1500);
chk('rule: a part that cannot sleep IS pushed along +X', b.ball.translation().x > 0.1, 'ball x=' + b.ball.translation().x.toFixed(3));

const hit = b.w.castRay(new R.Ray({ x: 1.5, y: 1.5, z: 1 }, { x: 0, y: 0, z: -1 }), 10, true);
const toi = hit && (hit.timeOfImpact ?? hit.toi);
chk('castRay down hits the floor top at 1 m', hit && Math.abs(toi - 1) < 1e-4, 'toi=' + toi);

let found = false;
const p = b.ball.translation();
b.w.intersectionsWithShape(p, { x: 0, y: 0, z: 0, w: 1 }, new R.Ball(0.01), col => { if (col.handle === b.ballCol.handle) found = true; return true; });
chk('intersectionsWithShape finds the ball at its own position', found);

const c = scenario(1500);
const same = ['x', 'y', 'z'].every(k => b.ball.translation()[k] === c.ball.translation()[k]);
chk('two identical runs end bit-identical (deterministic build)', same,
  JSON.stringify(b.ball.translation()) + ' vs ' + JSON.stringify(c.ball.translation()));

process.exit(fail ? 1 : 0);
