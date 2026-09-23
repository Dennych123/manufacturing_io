#!/usr/bin/env node
// @ts-check
// The lathe-line scene, built from what the video of the real cell shows (docs in the scene's
// .ctl.js): a NDESO NDESO-087 hanging from a traverse beam over two TAKISAWA TCC-2000 lathes that
// run the SAME operation in parallel, with a separate infeed and outfeed conveyor at one end of
// the cell. The lathes' doors slide sideways, the way a TCC-2000's does.
//
//   node tools/gen_lathe_line.js           write scenes/lathe-line.json and the pose blocks
//   node tools/gen_lathe_line.js --check   exit 1 when any committed output is stale
//
// ONE source for the geometry and the joint angles. The scene, the controller's pose table and the
// ST twin's pose table are all written from here: 60 joint angles typed into two programs by hand
// drift, and the drift is silent. The IK runs here only (lib/ik.js), never in the plant.
//
// The generator also CHECKS its own poses against the machines: nothing in the plant notices an
// arm drawn through a machine casting (kinematic bodies do not collide with fixed ones), and the
// only thing that reviews a cell's mechanics is the eye. So every pose is sampled along the arm
// and tested against the lathes' solid blocks before the scene is written.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { stringify, validate, worldPoses } from '../lib/scene.js';
import { solveIk } from '../lib/ik.js';
import { apply, compose, invert, pose } from '../lib/math.js';
import { TYPES, withDefaults } from '../lib/components.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
/** Everything that moves WITH the arm: it cannot clash with itself here. */
const ARM_IDS = new Set(['rail', 'rbase', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'jawA', 'jawB']);
const NAME = 'lathe-line';

// ---------------------------------------------------------------------------- geometry (mm)
/** Machine centres. A TCC-2000 is 1450 wide; the two stand 150 apart and run the same operation. */
export const MX = [-700, 900];
/** Machine front face (the aisle is -Y), depth 1400, height 1700. */
const MW = 1450, MD = 1400, MH = 1700;
/**
 * The work window in the front: sill, head, and its X span relative to the machine centre. It is
 * tall enough that the arm reaches THROUGH it with the elbow still out in the aisle - measured the
 * other way round, with a 450 mm window, the upper arm came down through the machine's hood.
 */
const WIN = { z0: 820, z1: 1480, x0: -640, x1: 60 };
/** Spindle: horizontal along X, the chuck face looking +X, relative to the machine centre. */
export const CHUCK = { dx: -430, y: 200, z: 1120 };
/** The part: a turned casting standing on its end. */
export const PART = { d: 50, h: 70 };
/**
 * The two conveyors are IN SERIES on ONE lane in front of the lathes, not side by side: raw
 * castings come in from the RIGHT and finished parts leave to the LEFT. They are separate machines
 * with a gap between them, and the robot works that gap - it lifts a casting off the end of the
 * infeed and stands the finished part on the head of the outfeed. Neither carries a part from one
 * lathe to the other: both lathes run the same operation.
 *
 * The gap stands CLEAR OF BOTH MACHINES, past the upstream end of the cell. The arm dives at the
 * pin from above with its elbow back over the machines, and swept along the lane (`clashes()` at
 * 100 mm) the only clear bands are x <= -1500, the 150 mm slot between the two lathes, and
 * x >= +1700 - which is exactly the two machines' own spans, -1425..25 and 175..1625. With the
 * stations where they used to be, at -200 and -700, the arm stood 85 mm inside machine 1's NC
 * panel and clipped its tower: the plant cannot notice that (a kinematic link does not collide
 * with a fixed one) and the picture simply lied. Denny saw it in the 3D view.
 */
// The lane is as wide as the casting, not as wide as a belt: 50 mm parts in a 90 mm lane do not
// queue, they ZIG-ZAG - measured, they packed at a 29 mm pitch alternating +-20 mm off centre, so
// the part behind the front one sat over the lift pin and went up with it. The guides are 30 mm
// and the casting is 70, so it stands taller than them and nothing drops between them (CLAUDE.md).
export const LANE = { y: -450, top: 820, w: 60, speed: 250 };
export const CVIN = { ...LANE, x0: 2000, x1: 3600 };
export const CVOUT = { ...LANE, x0: -2600, x1: 1850 };
/** Where the robot works each conveyor: the infeed's last 100 mm, and the outfeed's first. */
export const STX_IN = 2100, STX_OUT = 1750;
/** Pin lift: the stop holds a part, the lift raises it LIFT mm to the robot. */
export const PIN = { sink: 15, stroke: 150,
  // The nest sits PAD above the rod end. A cylinder's rod carries a 12 mm steel block at its end
  // (the shape every rod has), and a nest mounted flat on the rodEnd socket puts its 6 mm floor
  // straight through that block: measured, the part then stood on the 19 mm block instead of the
  // 62 mm floor, slid off it, sank 11 mm and toppled - and rode the rest of the line lying down.
  pad: 6 };
/**
 * Stopper: pops up from under the belt just downstream of the pin.
 *
 * The blade is TALLER than half the casting, and that is the whole of it. Measured with a 25 mm
 * head, which stands 15 mm above the belt: a casting running into it at 250 mm/s is pushed at
 * 7 mm while its centre of mass is at 35, so the stop tips it forward instead of stopping it -
 * castings ended up leaning on the blade at z 845, climbed over it at 458-684 mm/s (against a
 * 250 mm/s belt) and sailed through the next stop as well. A 60 mm blade stands 50 mm above the
 * belt, above the casting's centre of mass, and stops it square.
 *
 * The stroke has to clear the blade: retracted, the head must be BELOW the belt or it is a
 * permanent obstruction, so stroke > head + sink.
 */
const STOP = { sink: 10, stroke: 80, head: [12, 70, 60] };
/**
 * The escapement. A second pop-up stop stands HOLD.gap from each station - upstream on the
 * infeed, downstream on the outfeed - and the line runs stop-and-go through it: one casting at a
 * time, never a queue at the station.
 *
 * Measured with the castings queueing against the station stop instead (200 s of cell): the belt
 * drives the queue while the pin is UP, so the next casting creeps until it overlaps the pin's
 * own column by 12 mm, and the pin flicks it off the line as it comes down - 2 castings of 26
 * thrown to y -802 and -757 mm and lost off the world, with the cell otherwise running. Narrowing
 * the pin and stopping the belt only move the margin around; what removes it is that nothing
 * stands within a part's length of the pin at all (CLAUDE.md: a pin cannot come down between
 * parts that touch, so the parts have to arrive with a gap).
 *
 * `gap` is what that costs in time: 250 mm at 250 mm/s is 1 s of travel, inside the robot's own
 * cycle. The hold beam stands where a waiting casting rests against the pin, at part radius plus
 * half the head.
 */
const HOLD = { gap: 250 };
/**
 * The infeed's third stop, one casting further back: the QUEUE stop. It is what the line queues
 * against, so the cell carries two castings in hand - one at the hold, ready to go, and one at
 * the queue behind it - instead of one.
 *
 * `pitch` is pin centre to pin centre, and it has to be more than a casting: a casting resting
 * against the hold reaches 2406, and the queue pin's own head is 12 wide, so 85 mm leaves 23 mm
 * of free belt for the queue pin to come up in. THAT is the whole design rule for a cascade of
 * stops - each pin holds exactly ONE casting and every pin comes up on free belt. Two castings
 * touching at one pin can never be separated again (CLAUDE.md), so the stops pass castings hand
 * to hand instead of letting them pile up.
 */
const QUE = { pitch: 130 };
/** The traverse: J1's mounting face hangs at this height, the beam over the aisle. */
export const BEAM = { y: -430, j1z: 2000 };
/** Where the carriage stands to work at a machine, relative to the machine centre. */
export const RAIL_DX = -180;
/** Where it stands to work each conveyor station: straight over that station's pin. */
export const RAIL_IN = STX_IN, RAIL_OUT = STX_OUT;
const SK = { carriage: 96 };

// NDESO NDESO-087, from NDESO's technical data sheet: arms 445 + 430 (875), reach 905 at point P,
// J2 30 out from J1, J1->J2 395; ranges J1 +-170, J2 +135/-100, J3 +153/-136, J4 +-270, J5 +-120,
// J6 +-360; top speeds 285 / 252.5 / 303 / 378.75 / 378.75 / 606 deg/s. The flange sits 80 from P.
// The chain is set up the maker's way (upright, J4 along the forearm) and the whole robot is
// turned over by the mount, so nothing below is re-derived for hanging.
const NDESO087 = [
  { id: 'j1', axis: 'z', to: [30, 0, 395], min: -170, max: 170, v: 285, w: 150 },
  { id: 'j2', axis: 'y', to: [0, 0, 445], min: -100, max: 135, v: 252.5, w: 130 },
  { id: 'j3', axis: 'y', to: [0, 0, 20], min: -136, max: 153, v: 303, w: 110 },
  { id: 'j4', axis: 'x', to: [430, 0, 0], min: -270, max: 270, v: 378.75, w: 90 },
  { id: 'j5', axis: 'y', to: [80, 0, 0], min: -120, max: 120, v: 378.75, w: 80 },
  { id: 'j6', axis: 'x', to: [0, 0, 0], min: -360, max: 360, v: 606, w: 70 },
];
/**
 * The maker's own shells, one STEP part per axis, tessellated and decimated to about 4000 triangles
 * each by tools/step_to_stl.py. They are DRAWN only - the primitives stay as the colliders - and
 * they are not in git (see .gitignore): a scene that names them still draws as boxes without them.
 * `at` / `rot` put each part's own frame into the frame the kinematics were built in.
 */
// Read out of the maker's own files rather than eyeballed. Their assembly stands the robot with
// its J1 axis along +Y, so their frame maps to this one by the cyclic permutation (x, y, z) ->
// (z, x, y), which is rot [90, 0, 90]; every joint bore in the assembly then lands exactly on this
// chain's joint origins (J2 at 30, 0, 395 and J3 at 30, 0, 840, to the millimetre - which is the
// check that the kinematics here and the maker's CAD are the same robot).
//
// The maker numbers the part FILES FROM THE BASE, so c001 is the base casting that does not move
// and cN is the link driven by joint N-1. Read out of the assembly, each solid carries the bores of
// the two joints it spans: c001 the J1 bore alone, c002 the J1 and J2 bores, c003 the J2 and J3
// bores, c004 the J3 bore and the J4 axis, c005 the J4 axis. Hanging jN.stl on joint N therefore
// dressed every link in the casting of the joint BELOW it, and J2, J3 and J4 swung a shell whose
// own bore was somewhere else - the arm bent in the right places and the metal did not.
//
// Each part file is its assembly solid translated by `d`, also read out of the assembly (part bbox
// against assembly bbox, in the maker's frame):
//   c001 (0, 0, 0)  c002 (0, 395, -5)  c003 (0, 395, 30)  c004 (0, 840, 30)  c005/c006 (0, 860, 460)
// so the shell offset below is R * d minus the link's own origin, R being that same rot.
const SHELL_ROT = [90, 0, 90];
const SHELL = {
  j1: { file: 'j2', at: [-5, 0, 395] },
  j2: { file: 'j3', at: [0, 0, 0] },
  j3: { file: 'j4', at: [0, 0, 0] },
  j4: { file: 'j5', at: [430, 0, 0] },
  j5: { file: 'j6', at: [0, 0, 0] },
  // J6 is the flange itself: the maker ships no seventh part, so the hub primitive draws it.
};
const SHELL_DIR = '/assets/robots/ndeso087/';
/** The cycle does not run the arm at its catalogue maximum; neither does the cell in the video.
 * At 0.5, and with the traverse at 1.2 m/s, the cell ran a 16.3 s cycle against a 13 s target: the
 * two long rail hauls (infeed to machine, machine to outfeed) were 2.7 s each and the arm's own
 * moves about 5 s of the rest. An 8 s cut on two machines in parallel is never the limit here, the
 * ROBOT is, so the cycle time is bought in the traverse and the arm and nowhere else. */
const SPEED = 0.68;

/**
 * The double hand: jaw A along the flange axis, jaw B at 90 degrees to it.
 *
 * The jaws are VEE, cut for the casting's own radius, which is what the hand in the video wears
 * and what every lathe-tending hand wears: the part is ROUND, so a flat pad holds it on one line
 * and lets it roll, while a vee seats it on two flanks and locates it. The flanks are drawn and
 * do not collide, so this is the picture only - the pad behind them is still the collider the
 * model closes onto `blockAt`.
 *
 * A vee jaw is WIDE - the flanks reach out to jawR past the middle - and the two hands sit 90
 * degrees apart on one flange, so the width is what sets how close they stand. Measured over the
 * whole finger travel (0, 12, 25, 40, 50 mm): hand A clears hand B by 2.0 mm at every position,
 * and the gap does not move, because both are bolted to the same flange. Widen a jaw and that is
 * the number that goes first.
 */
const JAW = { span: 100, fingerLen: 50, fingerW: 14, jaw: 'vee', jawR: PART.d / 2 };
const TCP = JAW.fingerLen * 0.6;                       // the gripper's catch-zone centre

/** @param {number} v */
const r2 = v => Math.round(v * 100) / 100 || 0;

/**
 * The lathe's solid blocks in the machine's own frame. The scene draws them and the clash check
 * tests against them, so there is one description of what a lathe IS.
 * @returns {Array<{id: string, x: number[], y: number[], z: number[]}>}
 */
function machineBlocks() {
  return [
    { id: 'bed', x: [-MW / 2, MW / 2], y: [0, MD], z: [0, WIN.z0] },
    { id: 'tower', x: [WIN.x1, MW / 2], y: [0, MD], z: [WIN.z0, MH] },
    { id: 'back', x: [-MW / 2, WIN.x1], y: [CHUCK.y + 220, MD], z: [WIN.z0, MH] },
    { id: 'wall', x: [-MW / 2, WIN.x0], y: [0, CHUCK.y + 220], z: [WIN.z0, MH] },
    { id: 'hood', x: [WIN.x0, WIN.x1], y: [0, CHUCK.y + 220], z: [WIN.z1, MH] },
    { id: 'head', x: [WIN.x0, CHUCK.dx - 80], y: [CHUCK.y - 130, CHUCK.y + 130], z: [CHUCK.z - 130, CHUCK.z + 130] },
    { id: 'jaws', x: [CHUCK.dx - 80, CHUCK.dx - 6], y: [CHUCK.y - 105, CHUCK.y + 105], z: [CHUCK.z - 105, CHUCK.z + 105] },
  ];
}

export function buildScene() {
  /** @type {any[]} */
  const C = [];
  const add = (/** @type {any} */ c) => { C.push(c); return c; };
  const railZ = BEAM.j1z + SK.carriage;
  const railMin = Math.min(RAIL_OUT, MX[0] + RAIL_DX) - 250, railMax = Math.max(RAIL_IN, MX[1] + RAIL_DX) + 250;

  // ---- the traverse beam and its columns. The columns stand BEYOND the ends of both belts:
  // measured with one of them at the beam's own end, it stood in the lane and a casting travelling
  // down the infeed stopped dead against it, 1.8 m short of the station, with nothing reporting it.
  const colX = [CVOUT.x0 - 500, CVIN.x1 + 500];
  add({ id: 'beam', type: 'frame', label: 'TRAVERSE BEAM', at: [(colX[0] + colX[1]) / 2, BEAM.y, railZ + 96],
        params: { size: [colX[1] - colX[0] + 300, 220, 260], style: 'solid', color: '#c8ccd0' } });
  for (const [id, x] of [['colL', colX[0]], ['colR', colX[1]]]) {
    add({ id, type: 'frame', label: 'BEAM COLUMN', at: [x, BEAM.y, 0], params: { size: [160, 160, railZ + 96], style: 'solid', color: '#b8bdc2' } });
  }
  const jog = (/** @type {string} */ p) => ({ ovr: 'OVR', jogP: p + '_JOG_P', jogN: p + '_JOG_N' });
  const axisIo = (/** @type {string} */ p) => ({ target: p + '_TGT', exec: p + '_EXEC', done: p + '_DONE', busy: p + '_BUSY', actPos: p + '_POS', inPos: p + '_INPOS', ...jog(p) });
  // Turned over about X: the carriage hangs under the rail and everything on it hangs too.
  add({ id: 'rail', type: 'joint', label: 'TRAVERSE AXIS', station: 'ST1', at: [0, BEAM.y, railZ], rot: [180, 0, 0],
        params: { kind: 'prismatic', axis: 'x', len: 0, min: railMin, max: railMax, home: RAIL_IN, vmax: 2000, acc: 5000, width: 160, jogPct: 10 },
        io: axisIo('RX') });

  // ---- the robot. The base casting carries the J1 bearing and does NOT turn with it, so it rides
  // the carriage as a shell of its own; hung on J1 it span with the shoulder.
  add({ id: 'rbase', type: 'shell', label: 'NDESO-087 BASE', parent: 'rail', socket: 'end',
        at: [0, 0, SK.carriage], rot: SHELL_ROT, params: { asset: SHELL_DIR + 'j1.stl', scale: 1, color: '#f1efe8' } });
  NDESO087.forEach((j, i) => {
    const sh = SHELL[/** @type {keyof typeof SHELL} */ (j.id)];
    add({ id: j.id, type: 'joint', label: 'NDESO-087 J' + (i + 1), station: 'ST1', parent: i ? NDESO087[i - 1].id : 'rail', socket: 'end',
          at: i ? [0, 0, 0] : [0, 0, SK.carriage],
          params: { kind: 'revolute', axis: j.axis, len: 0, to: j.to, min: j.min, max: j.max, home: 0,
                    vmax: r2(j.v * SPEED), acc: r2(j.v * SPEED * 3), width: j.w, color: '#f1efe8', jogPct: 5,
                    ...(sh ? { mesh: SHELL_DIR + sh.file + '.stl', meshScale: 1, meshAt: sh.at, meshRot: SHELL_ROT } : {}) },
          io: axisIo('J' + (i + 1)) });
  });
  const jaw = { ...JAW, closeMs: 300, openMs: 300, band: 1.5 };
  // The two gripper BODIES are the hand: a 126 mm disc each, overlapping at 90 degrees on the
  // flange, which is the compact dark block the video shows. There was an adapter plate here as
  // well and it was simply wrong - measured in the flange frame, it ran z -90..0, so the whole
  // 70 x 70 x 90 box sat BEHIND the flange face, inside the robot's own wrist castings, where it
  // read as a grey slab hanging off the back of the hand. Nothing hung from it either: both jaws
  // parent to `j6`, not to it.
  add({ id: 'jawA', type: 'gripper', label: 'HAND A (raw)', station: 'ST1', parent: 'j6', socket: 'end', at: [70, 0, 0], rot: [0, 90, 0], params: jaw,
        io: { close: 'GRIP_A', open: 'AS_A_OPEN', closed: 'AS_A_CLOSED' } });
  add({ id: 'jawB', type: 'gripper', label: 'HAND B (finished)', station: 'ST1', parent: 'j6', socket: 'end', at: [35, 0, 35], rot: [0, 0, 0], params: jaw,
        io: { close: 'GRIP_B', open: 'AS_B_OPEN', closed: 'AS_B_CLOSED' } });

  add({ id: 'partTpl', type: 'workpiece', label: 'CASTING (template)', at: [0, -2600, 0],
        params: { kind: 'cyl', size: [PART.d, PART.d, PART.h], color: '#7b4a2c', material: 'steel' } });

  // ---- the two conveyors. They are SEPARATE lines: raw castings arrive on one and finished parts
  // leave on the other. Neither carries a part from one lathe to the other - both lathes run the
  // same operation, in parallel, which is what doubles the cell's output.
  // Both belts carry toward -X: in from the right, out to the left. Turned about Z, so each one's
  // `start` socket is at its +X end, which is the end a casting is laid on.
  add({ id: 'cvIn', type: 'conveyor', label: 'INFEED CONVEYOR', station: 'ST4', at: [(CVIN.x0 + CVIN.x1) / 2, LANE.y, 0], rot: [0, 0, 180],
        params: { length: CVIN.x1 - CVIN.x0, width: LANE.w, height: LANE.top, speed: LANE.speed, guides: 30 },
        io: { run: 'CVIN_RUN', ovr: 'OVR' } });
  add({ id: 'cvOut', type: 'conveyor', label: 'OUTFEED CONVEYOR', station: 'ST5', at: [(CVOUT.x0 + CVOUT.x1) / 2, LANE.y, 0], rot: [0, 0, 180],
        params: { length: CVOUT.x1 - CVOUT.x0, width: LANE.w, height: LANE.top, speed: LANE.speed, guides: 30 },
        io: { run: 'CVOUT_RUN', ovr: 'OVR' } });
  // At the START of the infeed belt: a casting laid on the far end would run AWAY from the station.
  // Called for ONE AT A TIME (`tag` mode, the command held until the count moves), because a
  // stop-and-go escapement only works where the parts arrive with a gap: a hold pin cannot come
  // back up through a queue whose castings touch. The buffer is the hold pin, not the belt - one
  // casting stands at the hold while the robot works the one at the station, which is what keeps
  // the cell fed without ever putting a second casting beside the lift.
  add({ id: 'emIn', type: 'emitter', label: 'CASTING FEED', station: 'ST6', parent: 'cvIn', socket: 'start', at: [150, 0, 5],
        params: { template: 'partTpl', mode: 'tag' }, io: { emit: 'EM_IN_EMIT', count: 'EM_IN_CNT' } });
  // Off the END into free air, caught by a zone: nothing solid stands in the part's path, and a
  // part leaves a belt as a projectile (CLAUDE.md).
  add({ id: 'rmOut', type: 'remover', label: 'NEXT PROCESS', parent: 'cvOut', socket: 'end', at: [500, 0, -700],
        params: { size: [1200, 400, 700] }, io: { count: 'RM_OUT_CNT' } });

  // ---- the two stations on those conveyors, side by side at the loading end of the cell
  for (const [S, sx] of /** @type {const} */ ([['IN', STX_IN], ['OUT', STX_OUT]])) {
    const CV = LANE;
    const dir = -1;                                  // both belts carry toward -X
    const liftFoot = CV.top - PIN.sink - (PIN.stroke + 32 + 20) - 27 - PIN.pad;
    add({ id: 'lift' + S, type: 'cylinder', label: S + ' PIN LIFT', station: S === 'IN' ? 'ST4' : 'ST5', at: [sx, CV.y, liftFoot],
          params: { bore: 32, stroke: PIN.stroke, valve: '5/2-double', extendMs: 450, retractMs: 450, extWord: 'UP', retWord: 'DOWN' },
          io: { solExt: 'SOL_' + S + '_UP', solRet: 'SOL_' + S + '_DN', 'sw.ret': 'AS_' + S + '_DN', 'sw.ext': 'AS_' + S + '_UP' } });
    // Both pockets are the casting's own size plus a little. The infeed pin was cut to 36 mm
    // while the castings still queued against the station stop, so that it reached under the
    // front one without touching the next - a margin, and margins are what the escapement above
    // replaces: with the hold pin metering, the nearest other casting is 250 mm away and the
    // pocket can be the one that actually locates a part.
    const pinW = PART.d + 4;
    add({ id: 'pin' + S, type: 'nest', label: S + ' PIN', station: S === 'IN' ? 'ST4' : 'ST5', parent: 'lift' + S, socket: 'rodEnd', at: [0, 0, PIN.pad],
          params: { size: [pinW, PART.d + 4, 40], wall: 4 }, io: { clamp: S + '_CLAMP', present: 'PX_' + S } });
    const [, , hz] = STOP.head, sLb = STOP.stroke + 16 + 20;
    add({ id: 'stop' + S, type: 'cylinder', label: S + ' STOPPER', station: S === 'IN' ? 'ST4' : 'ST5',
          at: [sx + dir * (PART.d / 2 + STOP.head[0] / 2 + 1), CV.y, CV.top - STOP.sink - hz - 27 - sLb],
          params: { bore: 16, stroke: STOP.stroke, valve: '5/2-single', extendMs: 120, retractMs: 120, extWord: 'UP', retWord: 'DOWN', head: 'plate', headSize: STOP.head },
          io: { solExt: 'SOL_' + S + '_STOP', 'sw.ret': 'AS_' + S + '_STOP_DN', 'sw.ext': 'AS_' + S + '_STOP_UP' } });
    // The escapement pin and its beam. The infeed's stands UPSTREAM of the station (a casting
    // waits there while the robot works the one on the lift); the outfeed's stands DOWNSTREAM
    // (a finished part waits there until it is let go, so nothing ever backs up into the pin).
    // Both belts carry toward -X, so "upstream" is +X either way.
    const holdX = sx - (S === 'IN' ? dir : -dir) * HOLD.gap;
    add({ id: 'hold' + S, type: 'cylinder', label: S + ' HOLD STOP', station: S === 'IN' ? 'ST4' : 'ST5',
          at: [holdX, CV.y, CV.top - STOP.sink - hz - 27 - sLb],
          params: { bore: 16, stroke: STOP.stroke, valve: '5/2-single', extendMs: 120, retractMs: 120, extWord: 'UP', retWord: 'DOWN', head: 'plate', headSize: STOP.head },
          io: { solExt: 'SOL_' + S + '_HOLD', 'sw.ret': 'AS_' + S + '_HOLD_DN', 'sw.ext': 'AS_' + S + '_HOLD_UP' } });
    add({ id: 'eyeHold' + S, type: 'photoEye', label: S + ' HOLD BEAM', station: S === 'IN' ? 'ST4' : 'ST5',
          at: [holdX + PART.d / 2 + STOP.head[0] / 2, CV.y - CV.w / 2 - 40, CV.top + 30], rot: [0, 0, 90],
          params: { range: CV.w + 80, offDelayMs: 150 }, io: { out: 'PE_' + S + '_HOLD' } });
    // The infeed's queue stop, one casting behind the hold, with its own beam. The outfeed needs
    // none: finished parts arrive one per robot cycle, which is a gap no line can better.
    if (S === 'IN') {
      const queX = holdX + QUE.pitch;
      add({ id: 'queIN', type: 'cylinder', label: 'IN QUEUE STOP', station: 'ST6',
            at: [queX, CV.y, CV.top - STOP.sink - hz - 27 - sLb],
            params: { bore: 16, stroke: STOP.stroke, valve: '5/2-single', extendMs: 120, retractMs: 120, extWord: 'UP', retWord: 'DOWN', head: 'plate', headSize: STOP.head },
            io: { solExt: 'SOL_IN_QUE', 'sw.ret': 'AS_IN_QUE_DN', 'sw.ext': 'AS_IN_QUE_UP' } });
      add({ id: 'eyeQueIN', type: 'photoEye', label: 'IN QUEUE BEAM', station: 'ST6',
            at: [queX + PART.d / 2 + STOP.head[0] / 2, CV.y - CV.w / 2 - 40, CV.top + 30], rot: [0, 0, 90],
            params: { range: CV.w + 80, offDelayMs: 150 }, io: { out: 'PE_IN_QUE' } });
      // The PILE BLADE: a knife escapement that comes in from the SIDE, between the casting
      // standing at the queue stop and the one behind it.
      //
      // Why from the side, and why a knife. A pop-up stop can hold a pile but can never meter
      // one out of it: to let the front casting go it must come down, and it then has to come
      // back up through whatever is standing over it - measured, that throws the casting into
      // the air and it sails over the next stop at 458-684 mm/s against a 250 mm/s belt. A
      // holder with a pocket cannot do it either: its floor is flush with the belt and a casting
      // running onto that edge trips over it (measured: upright at x 2656, 6 degrees at 2616,
      // flat on its side by 2586). A blade that enters from the side touches neither the
      // underside nor the path: the gap between two touching 50 mm castings is 20 mm wide at
      // 20 mm off the lane centre, which is where the blade goes in, and the round nose wedges
      // it in and lets it come out again with a casting pressing on it.
      const bladeX = queX + STOP.head[0] / 2 + PART.d;   // the seam between casting 1 and casting 2
      const bBore = 16, bStroke = 30, bReach = 40, bLb = bStroke + bBore + 20;
      add({ id: 'knifeIN', type: 'cylinder', label: 'IN PILE BLADE', station: 'ST6',
            // The rod points across the lane (+Y). Its tip stands 15 mm inside the near guide
            // when it is in, and 15 mm outside it when it is out.
            at: [bladeX, CV.y - CV.w / 2 - 15 - (bLb + 27 + bReach), CV.top + PART.h / 2], rot: [-90, 0, 0],
            params: { bore: bBore, stroke: bStroke, valve: '5/2-single', extendMs: 120, retractMs: 120,
                      extWord: 'IN', retWord: 'OUT', head: 'knife', headSize: [10, 45, bReach] },
            io: { solExt: 'SOL_IN_BLADE', 'sw.ret': 'AS_IN_BLADE_OUT', 'sw.ext': 'AS_IN_BLADE_IN' } });
    }
    add({ id: 'eye' + S, type: 'photoEye', label: S + ' PART BEAM', station: S === 'IN' ? 'ST4' : 'ST5',
          at: [sx, CV.y - CV.w / 2 - 40, CV.top + 30], rot: [0, 0, 90],
          // 150 ms off-delay, as a real beam is set. The part crosses the ray again as the pin
          // takes it up and puts it back down, and a part rocking as it lands off the pin flickers
          // the ray for tens of milliseconds: measured, an 8 ms and a 20 ms gap in three minutes,
          // which the plant then reports as a stretched pulse. The SENSOR holds a blip like that.
          params: { range: CV.w + 80, offDelayMs: 150 }, io: { out: 'PE_' + S } });
  }

  MX.forEach((mx, k) => {
    const n = k + 1, M = 'M' + n;
    const body = '#d8d3c4', dark = '#2c3034';
    const blk = (/** @type {string} */ id, /** @type {string} */ label, /** @type {number[]} */ x, /** @type {number[]} */ y, /** @type {number[]} */ z, /** @type {string} */ color) =>
      add({ id: id + n, type: 'frame', label: 'TCC-2000 #' + n + ' ' + label, at: [mx + (x[0] + x[1]) / 2, (y[0] + y[1]) / 2, z[0]],
            params: { size: [x[1] - x[0], y[1] - y[0], z[1] - z[0]], style: 'solid', color } });
    for (const b of machineBlocks()) {
      const colour = b.id === 'hood' ? dark : b.id === 'head' ? '#6b7178' : b.id === 'jaws' ? '#3a3f45' : body;
      blk(b.id, b.id.toUpperCase(), b.x, b.y, b.z, colour);
    }
    blk('nc', 'NC PANEL', [WIN.x1 + 250, WIN.x1 + 560], [-80, 0], [WIN.z0 + 100, WIN.z0 + 700], dark);
    blk('lampG', 'LAMP GREEN', [MW / 2 - 90, MW / 2 - 40], [MD - 90, MD - 40], [MH, MH + 60], '#27b045');
    blk('lampY', 'LAMP AMBER', [MW / 2 - 90, MW / 2 - 40], [MD - 90, MD - 40], [MH + 60, MH + 120], '#ff9a1f');
    blk('lampR', 'LAMP RED', [MW / 2 - 90, MW / 2 - 40], [MD - 90, MD - 40], [MH + 120, MH + 180], '#e0322c');

    // The chuck: a nest turned so its pocket looks +X, the way the spindle does.
    add({ id: 'chuck' + n, type: 'nest', label: 'TCC-2000 #' + n + ' CHUCK', station: 'ST' + (2 + k), at: [mx + CHUCK.dx, CHUCK.y, CHUCK.z], rot: [0, 90, 0],
          params: { size: [PART.d + 6, PART.d + 6, 40], wall: 8 }, io: { clamp: M + '_CHUCK', present: M + '_PART' } });

    // The door SLIDES sideways, as a TCC-2000's does: a cylinder lying along X above the window,
    // with the leaf hanging off its rod end. Open = the leaf parked over the NC tower.
    const winW = WIN.x1 - WIN.x0, leafW = winW + 40, bore = 25, stroke = winW + 60;
    const Lb = stroke + bore + 20, foot = mx + WIN.x0 - 120;
    add({ id: 'door' + n, type: 'cylinder', label: 'TCC-2000 #' + n + ' DOOR CYL', station: 'ST' + (2 + k),
          at: [foot, -60, WIN.z1 + 90], rot: [0, 90, 0],
          params: { bore, stroke, valve: '5/2-single', extendMs: 1200, retractMs: 1200, extWord: 'OPEN', retWord: 'CLOSE' },
          io: { solExt: 'SOL_' + M + '_DOOR', 'sw.ret': 'AS_' + M + '_DOOR_CL', 'sw.ext': 'AS_' + M + '_DOOR_OP' } });
    // rot [0,90,0] lays the cylinder along +X, so the rod end travels +X as it extends and the leaf
    // that hangs from it clears the window to the right. The socket frame is the ROTATED one:
    // its +Z is world +X and its +X is world -Z, so the leaf's offsets are written that way round.
    // Written the other way round the leaf sat at z 2250 - 550 mm ABOVE the machine, up against the
    // traverse beam - and 370 mm off in X, covering half the window. Nothing reports a door that is
    // not over its opening: it is drawn, the cylinder's own switches still answer, and the sequence
    // waits for them and runs. Denny saw it in the 3D view.
    const cylZ = WIN.z1 + 90, winCz = (WIN.z0 + WIN.z1) / 2, winCx = mx + (WIN.x0 + WIN.x1) / 2;
    add({ id: 'leaf' + n, type: 'plate', label: 'TCC-2000 #' + n + ' DOOR', parent: 'door' + n, socket: 'rodEnd',
          at: [cylZ - winCz, 20, winCx - (foot + Lb + 27)], rot: [0, -90, 0],
          params: { size: [leafW, 14, WIN.z1 - WIN.z0 + 20] } });
  });

  // ---- the cell panel
  const pb = (/** @type {string} */ id, /** @type {string} */ label, /** @type {any} */ params, /** @type {any} */ io) => add({ id, type: 'pushbutton', label, params, io });
  add({ id: 'sel', type: 'selector', label: 'AUTO / INDIVIDUAL', io: { sel: 'SEL_AUTO' } });
  pb('pbMaster', 'MASTER ON', { kind: 'momentary', color: 'white', lamp: true }, { pb: 'PB_MASTER', lamp: 'PL_MASTER' });
  pb('pbEstop', 'EMERGENCY STOP', { kind: 'alternate', color: 'red', lamp: false }, { pb: 'PB_ESTOP' });
  pb('pbStart', 'START', { kind: 'momentary', color: 'green', lamp: true }, { pb: 'PB_START', lamp: 'PL_START' });
  pb('pbCstop', 'CYCLE STOP', { kind: 'momentary', color: 'yellow', lamp: false }, { pb: 'PB_CSTOP' });
  pb('pbHome', 'HOME POS', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'PB_HOME', lamp: 'PL_HOME' });
  add({ id: 'lampAuto', type: 'lamp', label: 'AUTO RUNNING', params: { color: 'amber' }, io: { lamp: 'AUTO_RUN' } });
  add({ id: 'dialOvr', type: 'speedDial', label: 'SPEED OVERRIDE', params: { min: 10 }, io: { ovr: 'OVR_SET' } });
  for (const [a, t] of [['RX', 'TRAVERSE'], ['J1', 'J1'], ['J2', 'J2'], ['J3', 'J3'], ['J4', 'J4'], ['J5', 'J5'], ['J6', 'J6']]) {
    pb('pbJog' + a + 'P', 'JOG ' + t + ' +', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_' + a + '_P', lamp: a + '_JOG_P' });
    pb('pbJog' + a + 'N', 'JOG ' + t + ' -', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_' + a + '_N', lamp: a + '_JOG_N' });
  }
  pb('pbIndA', 'HAND A', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_A', lamp: 'GRIP_A' });
  pb('pbIndB', 'HAND B', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_B', lamp: 'GRIP_B' });
  // One press, one casting: the feeder is in `tag` mode, so the emitter answers the button's
  // RISING EDGE. It is an INDIVIDUAL button like every other one - in AUTO the station calls for
  // castings itself, and two things feeding one lane is how a lane gets two castings at once.
  // The belts get their own buttons beside it, because a casting dropped onto a belt that is not
  // running stays under the feeder and blocks the next drop - the second press then does nothing
  // and the button looks broken.
  pb('pbIndFeed', 'FEED A CASTING', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_FEED', lamp: 'EM_IN_EMIT' });
  pb('pbIndCvIn', 'INFEED BELT', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_CV_IN', lamp: 'CVIN_RUN' });
  pb('pbIndCvOut', 'OUTFEED BELT', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_CV_OUT', lamp: 'CVOUT_RUN' });
  // Every stop gets a button of its own. They were the only actuators on the machine without
  // one, so the one thing an operator could not do by hand was let a casting past. The lamp is
  // the solenoid, which is UP: each button DROPS its stop and the stops come back up by
  // themselves when INDIVIDUAL ends.
  for (const [id, label, tag, lamp] of [
    ['pbIndStopIn', 'INFEED STOP', 'IND_STOP_IN', 'SOL_IN_STOP'],
    ['pbIndHoldIn', 'INFEED HOLD', 'IND_HOLD_IN', 'SOL_IN_HOLD'],
    ['pbIndQueIn', 'INFEED QUEUE STOP', 'IND_QUE_IN', 'SOL_IN_QUE'],
    ['pbIndBlade', 'INFEED PILE BLADE', 'IND_BLADE_IN', 'SOL_IN_BLADE'],
    ['pbIndStopOut', 'OUTFEED STOP', 'IND_STOP_OUT', 'SOL_OUT_STOP'],
    ['pbIndHoldOut', 'OUTFEED HOLD', 'IND_HOLD_OUT', 'SOL_OUT_HOLD'],
  ]) pb(id, label, { kind: 'momentary', color: 'blue', lamp: true }, { pb: tag, lamp });
  pb('pbIndD1', 'DOOR #1', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_DOOR1', lamp: 'SOL_M1_DOOR' });
  pb('pbIndD2', 'DOOR #2', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_DOOR2', lamp: 'SOL_M2_DOOR' });
  pb('pbIndLI', 'INFEED PIN LIFT', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_LIFT_IN', lamp: 'SOL_IN_UP' });
  pb('pbIndLO', 'OUTFEED PIN LIFT', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_LIFT_OUT', lamp: 'SOL_OUT_UP' });

  return {
    format: 'mio-scene/1', name: NAME, sim: { dtMs: 4 },
    io: { driver: 'opcua', endpoint: 'opc.tcp://127.0.0.1:4840', prefix: 'GlobalVars.', mode: 'sim', minPulseMs: 20 },
    stations: [
      { id: 'ST1', name: 'Robot', members: ['rail', ...NDESO087.map(j => j.id), 'jawA', 'jawB'], stepTag: 'ST1_STEP' },
      { id: 'ST2', name: 'TCC-2000 #1', members: ['chuck1', 'door1'], stepTag: 'ST2_STEP' },
      { id: 'ST3', name: 'TCC-2000 #2', members: ['chuck2', 'door2'], stepTag: 'ST3_STEP' },
      { id: 'ST4', name: 'Infeed', members: ['cvIn', 'liftIN', 'pinIN', 'stopIN', 'eyeIN', 'holdIN', 'eyeHoldIN'], stepTag: 'ST4_STEP' },
      { id: 'ST5', name: 'Outfeed', members: ['cvOut', 'liftOUT', 'pinOUT', 'stopOUT', 'eyeOUT', 'holdOUT', 'eyeHoldOUT', 'rmOut'], stepTag: 'ST5_STEP' },
      // The escapement runs WHILE the lift is busy - that is the whole point of a buffer - so it
      // is its own state machine meeting ST4 on one piece of shared state: a casting standing at
      // the hold (CLAUDE.md: stations that can run at once are separate state machines).
      { id: 'ST6', name: 'Infeed escapement', members: ['emIn', 'queIN', 'eyeQueIN', 'knifeIN'], stepTag: 'ST6_STEP' },
    ],
    cycle: { exitTag: 'RM_OUT_CNT', autoTag: 'AUTO_RUN', countTag: 'CYCLE_CNT', exclude: 1, avgN: 10 },
    components: C,
  };
}

// ---------------------------------------------------------------------------- poses
const JOINTS = NDESO087.map(j => ({ id: j.id, min: j.min, max: j.max }));
/**
 * The catch-zone centre of a jaw, reaching along its +Z. Only the REACH direction is demanded, not
 * where the fingers close: the part is a cylinder, so the jaw may take it from any side, and
 * pinning the finger direction as well made half the goals unreachable inside the machine.
 */
const tool = (/** @type {string} */ id) => ({ id, link: 'body', at: [0, 0, TCP], dir: [0, 0, 1] });

/**
 * Every pose the program commands, as world goals. The machine poses are solved with the carriage
 * at machine 1 and serve machine 2 unchanged, one machine pitch along; the conveyor poses are
 * solved with the carriage at the station.
 */
export function goals() {
  const mx = MX[0], pinZ = LANE.top - PIN.sink + PIN.stroke + PART.h / 2;
  const xc = mx + CHUCK.dx + PART.h / 2;
  /** @type {Record<string, {jaw: string, rail: number, p: number[], dir: number[]}>} */
  const g = {};
  for (const J of ['A', 'B']) {
    const jaw = 'jaw' + J;
    // at the machine: in front of the open door, inside clear of the chuck, and at the chuck
    g['door' + J] = { jaw, rail: mx + RAIL_DX, p: [xc + 100, -150, CHUCK.z], dir: [-1, 0, 0] };
    g['chOut' + J] = { jaw, rail: mx + RAIL_DX, p: [xc + 150, CHUCK.y, CHUCK.z], dir: [-1, 0, 0] };
    g['chAt' + J] = { jaw, rail: mx + RAIL_DX, p: [xc, CHUCK.y, CHUCK.z], dir: [-1, 0, 0] };
  }
  // at the conveyors: hand A works the infeed pin, hand B the outfeed pin
  g.inAt = { jaw: 'jawA', rail: RAIL_IN, p: [STX_IN, LANE.y, pinZ], dir: [0, 0, -1] };
  g.inUp = { jaw: 'jawA', rail: RAIL_IN, p: [STX_IN, LANE.y, pinZ + 140], dir: [0, 0, -1] };
  g.outAt = { jaw: 'jawB', rail: RAIL_OUT, p: [STX_OUT, LANE.y, pinZ], dir: [0, 0, -1] };
  g.outUp = { jaw: 'jawB', rail: RAIL_OUT, p: [STX_OUT, LANE.y, pinZ + 140], dir: [0, 0, -1] };
  g.home = { jaw: 'jawA', rail: RAIL_IN, p: [STX_IN + 150, LANE.y + 200, pinZ + 230], dir: [0, 0, -1] };
  return g;
}
/** Solve order: the free ones first, then the poses that must stay in the same arm configuration.
 * @type {Array<[string, string|null]>} */
const CHAIN = [
  ['chAtA', null], ['chOutA', 'chAtA'], ['doorA', 'chOutA'],
  ['chAtB', null], ['chOutB', 'chAtB'], ['doorB', 'chOutB'],
  ['inAt', null], ['inUp', 'inAt'], ['outAt', null], ['outUp', 'outAt'], ['home', 'inUp'],
];
export const ORDER = CHAIN.map(([k]) => k);

/**
 * Does any part of the arm stand inside anything solid at this pose? The plant cannot tell: every
 * link is kinematic and a kinematic body does not collide with a fixed one. Sampled along the
 * links, against EVERY colliding box the scene draws, with a margin for the link's own thickness.
 *
 * It used to test the lathes' own blocks only, and so said nothing about the arm standing 85 mm
 * inside machine 1's NC PANEL at the infeed pin, or 30 mm inside its hood at the outfeed one -
 * which is what the cell actually did, and what the eye caught in the 3D view. Anything the scene
 * draws solid is now in the test, beam columns and door leaves included.
 *
 * What is left out is only what the tool is SUPPOSED to be inside: the nests (a chuck is a pocket
 * the tool reaches into) and the `jaws` block that draws them, the belts, and the station cylinders
 * it works over. The DOORS are taken OPEN, which is the state the arm meets - the sequence waits
 * for the door's own switch before it goes in, and that interlock is the controller's test, not
 * this one; what this has to answer is whether the open leaf is parked clear of the arm.
 * @param {any} scene @param {Record<string, number>} dof @param {number} rail
 * @returns {string[]} what the arm is inside, worst first
 */
const CLASH_SKIP = new Set(['workpiece', 'emitter', 'remover', 'nest', 'conveyor', 'cylinder', 'photoEye']);
export function clashes(scene, dof, rail) {
  /** @type {Record<string, number>} */
  const doorsOpen = {};
  for (const c of scene.components) if (/^door\d+$/.test(c.id)) doorsOpen[c.id] = c.params.stroke;
  const W = worldPoses(scene, { ...doorsOpen, ...dof, rail });
  /** The arm's centre line: J2's hub, the elbow, the wrist and both tool tips. */
  const chainPts = [W.j1.arm.p, W.j2.arm.p, W.j3.arm.p, W.j4.arm.p, W.j5.arm.p, W.j6.arm.p,
                    apply(W.jawA.body, [0, 0, TCP]), apply(W.jawB.body, [0, 0, TCP])];
  const pts = [];
  for (let i = 0; i < chainPts.length - 1; i++) {
    const a = chainPts[i], b = chainPts[i + 1];
    for (let s = 0; s <= 8; s++) pts.push(a.map((/** @type {number} */ v, /** @type {number} */ j) => v + (b[j] - v) * s / 8));
  }
  /** A link is a bar about 90 mm across, so its centre line may come no closer than this. */
  const M = 45;
  /** @type {Array<[string, number]>} */
  const bad = [];
  for (const c of scene.components) {
    if (ARM_IDS.has(c.id) || CLASH_SKIP.has(c.type) || /^jaws\d+$/.test(c.id)) continue;
    const t = TYPES[c.type];
    if (!t?.shapes || !Array.isArray(t.params) || t.group === 'operator') continue;
    const p = withDefaults(t, c.params || {});
    for (const sh of t.shapes(p)) {
      if (sh.kind !== 'box' || sh.collide === false) continue;
      const lp = W[c.id]?.[sh.link];
      if (!lp) continue;
      const inv = invert(compose(lp, pose(sh.at, sh.rot || [0, 0, 0])));
      const h = sh.size.map((/** @type {number} */ v) => v / 2);
      let worst = 0;
      for (const q of pts) {
        const l = apply(inv, q);
        const d = [0, 1, 2].map(k => h[k] + M - Math.abs(l[k]));
        if (d.every(v => v > 0)) worst = Math.max(worst, Math.min(...d));
      }
      if (worst > 0) bad.push([c.id + '/' + sh.link + ' by ' + Math.round(worst) + ' mm', worst]);
    }
  }
  return bad.sort((a, b) => b[1] - a[1]).map(([m]) => m);
}

/**
 * The joint angles for every goal, solved against the scene's own kinematics. A pose that follows
 * another is seeded FROM it: solved freshly, chOut came back in a different arm configuration from
 * chAt, which would have the arm flip itself over inside the machine between two steps.
 * @param {any} scene
 */
export function solvePoses(scene) {
  const g = goals();
  // The IK only needs the arm, and worldPoses() walks every component it is given: solving against
  // the whole cell (150 components, two of them lathes) costs about eight times as much.
  const keep = new Set(['rail', ...NDESO087.map(j => j.id), 'jawA', 'jawB']);
  const armParts = scene.components.filter((/** @type {any} */ c) => keep.has(c.id));
  // The rail is not one of the solved joints, so the carriage has to be PARKED where the pose is
  // worked from: worldPoses() takes the rail's own `home` when no dof is given for it, and a pose
  // solved with the carriage at the wrong end of the cell simply reports "unreachable".
  /** @type {Map<number, any>} */
  const armAt = new Map();
  const armFor = (/** @type {number} */ rail) => {
    if (!armAt.has(rail)) {
      armAt.set(rail, { ...scene, components: armParts.map((/** @type {any} */ c) => (c.id === 'rail'
        ? { ...c, params: { ...c.params, home: rail } } : c)) });
    }
    return armAt.get(rail);
  };
  const seeds = [];
  for (const j1 of [0, 90, -90, 180]) for (const j3 of [45, 110]) for (const j5 of [-60, 60]) seeds.push({ j1, j3, j5 });
  const margin = (/** @type {Record<string, number>} */ d) => Math.min(...NDESO087.map(j => Math.min(d[j.id] - j.min, j.max - d[j.id])));
  /** @type {Record<string, number[]>} */
  const poses = {};
  /** @type {Record<string, Record<string, number>>} */
  const dofs = {};
  const report = [];
  for (const [k, from] of CHAIN) {
    const G = g[k];
    // Seeded from the pose before it, and from the free seeds as a fallback: a chained seed keeps
    // the arm in one configuration between two steps (solved freshly, the arm flipped itself over
    // inside the machine), but a seed that cannot converge must not leave the pose unsolved.
    const starts = from ? [dofs[from], ...seeds] : seeds;
    let best = null;
    for (const s of starts) {
      const r = solveIk(armFor(G.rail), JOINTS, tool(G.jaw), { p: G.p, dir: G.dir }, { start: s, iters: 1000, lambda: 4 });
      if (!r.ok) continue;
      const bad = clashes(scene, r.dof, G.rail);
      const m = margin(r.dof);
      // A pose that clashes is out; among the rest, prefer the one that keeps the arm where the
      // previous pose left it, and then the one with the most room to its limits.
      const move = from ? Math.max(...NDESO087.map(j => Math.abs(r.dof[j.id] - dofs[from][j.id]))) : 0;
      const score = (bad.length ? -1000 : 0) + m - move / 4;
      if (!best || score > best.score) best = { dof: r.dof, m, err: r.err, bad, score };
    }
    if (!best) throw new Error('pose ' + k + ' did not solve');
    dofs[k] = best.dof;
    poses[k] = NDESO087.map(j => r2(best.dof[j.id]));
    report.push(k.padEnd(8) + ' err ' + best.err.toFixed(3) + ' mm, margin ' + best.m.toFixed(1) + ' deg'
      + (best.bad.length ? '   CLASH: ' + best.bad.join(' | ') : ''));
  }
  return { poses, report, clash: report.filter(l => l.includes('CLASH')) };
}

// ---------------------------------------------------------------------------- writing
/** The pose table as the controller declares it. @param {Record<string, number[]>} poses */
function ctlBlock(poses) {
  const rows = ORDER.map(k => ' ' + JSON.stringify(k) + ':' + JSON.stringify(poses[k]));
  return ['/** J1..J6 in degrees, one entry per goal. The machine poses serve either lathe: the rail',
    ' * carries the robot one machine pitch along and the pose is the same. */',
    'export const POSE = {' + rows.join(',\n').replace(/^ /, '') + '};',
    '/** Where the carriage stands: at each machine, and at the two conveyor stations. */',
    'export const RAIL = ' + JSON.stringify(MX.map(mx => mx + RAIL_DX)) + ';',
    'export const RAIL_IN = ' + RAIL_IN + ', RAIL_OUT = ' + RAIL_OUT + ';'].join('\n');
}

/**
 * The ST twin's pose table. ST has no array literal an XML import will take, so the targets are
 * assigned by a CASE on GO_POSE, which every move step sets. Generated for the same reason as the
 * controller's table: 66 angles typed into two programs by hand drift, silently.
 * @param {Record<string, number[]>} poses
 */
function stBlock(poses) {
  const b = ['CASE GO_POSE OF'];
  ORDER.forEach((k, i) => {
    b.push('\t' + (i + 1) + ':\t// ' + k);
    poses[k].forEach((/** @type {number} */ v, /** @type {number} */ j) => b.push('\t\tJ' + (j + 1) + '_TGT := ' + v.toFixed(2) + ';'));
  });
  b.push('END_CASE;');
  b.push('CASE GO_RAIL OF');
  MX.forEach((mx, i) => b.push('\t' + (i + 1) + ':\tRX_TGT := ' + (mx + RAIL_DX).toFixed(2) + ';'));
  b.push('\t3:\tRX_TGT := ' + RAIL_IN.toFixed(2) + ';');
  b.push('\t4:\tRX_TGT := ' + RAIL_OUT.toFixed(2) + ';');
  b.push('END_CASE;');
  return b.join('\n');
}

/** The pose id the controller and the ST agree on, 1-based as the ST CASE uses it. */
export const POSE_ID = Object.fromEntries(ORDER.map((k, i) => [k, i + 1]));

const BEGIN = '// BEGIN GENERATED - tools/gen_lathe_line.js';
const END = '// END GENERATED';

/** Replace the generated region of a file, keeping everything around it. @param {string} text @param {string} body */
function splice(text, body) {
  const i = text.indexOf(BEGIN), j = text.indexOf(END);
  if (i < 0 || j < 0) throw new Error('the file has no "' + BEGIN + '" ... "' + END + '" region');
  return text.slice(0, i) + BEGIN + '\n' + body + '\n' + text.slice(j);
}

function main() {
  const check = process.argv.includes('--check');
  const scene = buildScene();
  const errs = validate(scene);
  if (errs.length) { console.error('the generated scene is invalid:\n  ' + errs.join('\n  ')); process.exit(1); }
  const { poses, report, clash } = solvePoses(scene);
  if (clash.length && !process.argv.includes('--force')) {
    console.error('poses put the arm inside a machine:\n  ' + clash.join('\n  '));
    console.error('move the beam, the window or the goal - the plant will NOT report this, and the picture will lie.');
    process.exit(1);
  }
  /** @type {Array<[string, string]>} */
  const files = [[path.join(ROOT, 'scenes', NAME + '.json'), stringify(scene)]];
  for (const [f, body] of [['.ctl.js', ctlBlock(poses)], ['.st', stBlock(poses)]]) {
    const p = path.join(ROOT, 'scenes', NAME + f);
    if (fs.existsSync(p)) files.push([p, splice(fs.readFileSync(p, 'utf8'), body)]);
    else console.error('missing ' + path.relative(ROOT, p) + ': write it with a generated region first');
  }
  let stale = 0;
  for (const [p, text] of files) {
    const cur = fs.existsSync(p) ? fs.readFileSync(p, 'utf8') : null;
    if (cur === text) continue;
    stale++;
    if (check) console.error('stale: ' + path.relative(ROOT, p));
    else { fs.writeFileSync(p, text); console.log('wrote ' + path.relative(ROOT, p)); }
  }
  if (check && stale) { console.error('run: node tools/gen_lathe_line.js'); process.exit(1); }
  if (!check) console.log(report.join('\n'));
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) main();
