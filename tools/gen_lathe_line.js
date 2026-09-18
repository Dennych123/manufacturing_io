#!/usr/bin/env node
// @ts-check
// The lathe-line scene, built from what the video of the real cell shows (docs in the scene's
// .ctl.js): a DENSO VS-087 hanging from a traverse beam over two TAKISAWA TCC-2000 lathes that
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
import { apply } from '../lib/math.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
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
 */
export const LANE = { y: -450, top: 820, w: 90, speed: 250 };
export const CVIN = { ...LANE, x0: -300, x1: 2800 };
export const CVOUT = { ...LANE, x0: -3000, x1: -600 };
/** Where the robot works each conveyor: the infeed's last 200 mm, and the outfeed's first. */
export const STX_IN = -200, STX_OUT = -700;
/** Pin lift: the stop holds a part, the lift raises it LIFT mm to the robot. */
export const PIN = { sink: 15, stroke: 150,
  // The nest sits PAD above the rod end. A cylinder's rod carries a 12 mm steel block at its end
  // (the shape every rod has), and a nest mounted flat on the rodEnd socket puts its 6 mm floor
  // straight through that block: measured, the part then stood on the 19 mm block instead of the
  // 62 mm floor, slid off it, sank 11 mm and toppled - and rode the rest of the line lying down.
  pad: 6 };
/** Stopper: pops up from under the belt just downstream of the pin. */
const STOP = { sink: 10, stroke: 40, head: [12, 70, 25] };
/** The traverse: J1's mounting face hangs at this height, the beam over the aisle. */
export const BEAM = { y: -430, j1z: 2000 };
/** Where the carriage stands to work at a machine, relative to the machine centre. */
export const RAIL_DX = -180;
/** Where it stands to work each conveyor station: straight over that station's pin. */
export const RAIL_IN = STX_IN, RAIL_OUT = STX_OUT;
const SK = { carriage: 96 };

// DENSO VS-087, from DENSO's technical data sheet: arms 445 + 430 (875), reach 905 at point P,
// J2 30 out from J1, J1->J2 395; ranges J1 +-170, J2 +135/-100, J3 +153/-136, J4 +-270, J5 +-120,
// J6 +-360; top speeds 285 / 252.5 / 303 / 378.75 / 378.75 / 606 deg/s. The flange sits 80 from P.
// The chain is set up the maker's way (upright, J4 along the forearm) and the whole robot is
// turned over by the mount, so nothing below is re-derived for hanging.
const VS087 = [
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
// check that the kinematics here and the maker's CAD are the same robot). Each per-axis part file
// is its assembly solid translated by `d`, so the shell offset is R * d minus the link's own origin.
const SHELL = {
  j1: { at: [0, 0, 0], rot: [90, 0, 90] },
  j2: { at: [-35, 0, 0], rot: [90, 0, 90] },
  j3: { at: [0, 0, -445], rot: [90, 0, 90] },
  j4: { at: [0, 0, -20], rot: [90, 0, 90] },
  j5: { at: [0, 0, 0], rot: [90, 0, 90] },
  j6: { at: [-80, 0, 0], rot: [90, 0, 90] },
};
const SHELL_DIR = '/assets/robots/vs087/';
/** The cycle does not run the arm at its catalogue maximum; neither does the cell in the video. */
const SPEED = 0.5;

/** The double hand: jaw A along the flange axis, jaw B at 90 degrees to it. */
const JAW = { span: 100, fingerLen: 50, fingerW: 10 };
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
  const railMin = Math.min(RAIL_OUT, MX[0] + RAIL_DX) - 250, railMax = MX[1] + RAIL_DX + 250;

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
        params: { kind: 'prismatic', axis: 'x', len: 0, min: railMin, max: railMax, home: RAIL_IN, vmax: 1200, acc: 3000, width: 160, jogPct: 10 },
        io: axisIo('RX') });

  // ---- the robot
  VS087.forEach((j, i) => {
    add({ id: j.id, type: 'joint', label: 'VS-087 J' + (i + 1), station: 'ST1', parent: i ? VS087[i - 1].id : 'rail', socket: 'end',
          at: i ? [0, 0, 0] : [0, 0, SK.carriage],
          params: { kind: 'revolute', axis: j.axis, len: 0, to: j.to, min: j.min, max: j.max, home: 0,
                    vmax: r2(j.v * SPEED), acc: r2(j.v * SPEED * 3), width: j.w, color: '#f1efe8', jogPct: 5,
                    mesh: SHELL_DIR + j.id + '.stl', meshScale: 1, meshAt: SHELL[j.id].at, meshRot: SHELL[j.id].rot },
          io: axisIo('J' + (i + 1)) });
  });
  const jaw = { span: JAW.span, fingerLen: JAW.fingerLen, fingerW: JAW.fingerW, closeMs: 300, openMs: 300, band: 1.5 };
  add({ id: 'hand', type: 'plate', label: 'DOUBLE HAND BLOCK', parent: 'j6', socket: 'end', at: [0, 0, -45], rot: [0, 90, 0], params: { size: [90, 70, 70] } });
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
  add({ id: 'emIn', type: 'emitter', label: 'CASTING FEED', station: 'ST4', parent: 'cvIn', socket: 'start', at: [150, 0, 5],
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
    add({ id: 'pin' + S, type: 'nest', label: S + ' PIN', station: S === 'IN' ? 'ST4' : 'ST5', parent: 'lift' + S, socket: 'rodEnd', at: [0, 0, PIN.pad],
          params: { size: [PART.d + 4, PART.d + 4, 40], wall: 4 }, io: { clamp: S + '_CLAMP', present: 'PX_' + S } });
    const [, , hz] = STOP.head, sLb = STOP.stroke + 16 + 20;
    add({ id: 'stop' + S, type: 'cylinder', label: S + ' STOPPER', station: S === 'IN' ? 'ST4' : 'ST5',
          at: [sx + dir * (PART.d / 2 + STOP.head[0] / 2 + 1), CV.y, CV.top - STOP.sink - hz - 27 - sLb],
          params: { bore: 16, stroke: STOP.stroke, valve: '5/2-single', extendMs: 120, retractMs: 120, extWord: 'UP', retWord: 'DOWN', head: 'plate', headSize: STOP.head },
          io: { solExt: 'SOL_' + S + '_STOP', 'sw.ret': 'AS_' + S + '_STOP_DN', 'sw.ext': 'AS_' + S + '_STOP_UP' } });
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
    // that hangs from it clears the window to the right.
    add({ id: 'leaf' + n, type: 'plate', label: 'TCC-2000 #' + n + ' DOOR', parent: 'door' + n, socket: 'rodEnd',
          at: [-(WIN.z1 - WIN.z0 + 20), 20, mx + WIN.x0 - 20 - foot - (Lb + 27)], rot: [0, -90, 0],
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
  pb('pbIndD1', 'DOOR #1', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_DOOR1', lamp: 'SOL_M1_DOOR' });
  pb('pbIndD2', 'DOOR #2', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_DOOR2', lamp: 'SOL_M2_DOOR' });
  pb('pbIndLI', 'INFEED PIN LIFT', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_LIFT_IN', lamp: 'SOL_IN_UP' });
  pb('pbIndLO', 'OUTFEED PIN LIFT', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_LIFT_OUT', lamp: 'SOL_OUT_UP' });

  return {
    format: 'mio-scene/1', name: NAME, sim: { dtMs: 4 },
    io: { driver: 'opcua', endpoint: 'opc.tcp://127.0.0.1:4840', prefix: 'GlobalVars.', mode: 'sim', minPulseMs: 20 },
    stations: [
      { id: 'ST1', name: 'Robot', members: ['rail', ...VS087.map(j => j.id), 'jawA', 'jawB'], stepTag: 'ST1_STEP' },
      { id: 'ST2', name: 'TCC-2000 #1', members: ['chuck1', 'door1'], stepTag: 'ST2_STEP' },
      { id: 'ST3', name: 'TCC-2000 #2', members: ['chuck2', 'door2'], stepTag: 'ST3_STEP' },
      { id: 'ST4', name: 'Infeed', members: ['cvIn', 'emIn', 'liftIN', 'pinIN', 'stopIN', 'eyeIN'], stepTag: 'ST4_STEP' },
      { id: 'ST5', name: 'Outfeed', members: ['cvOut', 'liftOUT', 'pinOUT', 'stopOUT', 'eyeOUT', 'rmOut'], stepTag: 'ST5_STEP' },
    ],
    cycle: { exitTag: 'RM_OUT_CNT', autoTag: 'AUTO_RUN', countTag: 'CYCLE_CNT', exclude: 1, avgN: 10 },
    components: C,
  };
}

// ---------------------------------------------------------------------------- poses
const JOINTS = VS087.map(j => ({ id: j.id, min: j.min, max: j.max }));
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
 * Does any part of the arm stand inside a machine casting at this pose? The plant cannot tell:
 * every link is kinematic and a kinematic body does not collide with a fixed one. Sampled along
 * the links, in the machine's own frame, with a margin for the link's own thickness.
 * @param {any} scene @param {Record<string, number>} dof @param {number} rail
 * @returns {string[]} the blocks the arm is inside, worst first
 */
export function clashes(scene, dof, rail) {
  const W = worldPoses(scene, { ...dof, rail });
  /** The arm's centre line: J2's hub, the elbow, the wrist and the tool tip. */
  const pts = [];
  const chainPts = [W.j1.arm.p, W.j2.arm.p, W.j3.arm.p, W.j4.arm.p, W.j5.arm.p, W.j6.arm.p,
                    apply(W.jawA.body, [0, 0, TCP]), apply(W.jawB.body, [0, 0, TCP])];
  for (let i = 0; i < chainPts.length - 1; i++) {
    const a = chainPts[i], b = chainPts[i + 1];
    for (let s = 0; s <= 8; s++) pts.push(a.map((v, j) => v + (b[j] - v) * s / 8));
  }
  /** A link is a bar about 90 mm across, so its centre line may come no closer than this. */
  const M = 45;
  const bad = [];
  for (const mx of MX) {
    for (const b of machineBlocks()) {
      // The chuck is where the arm is SUPPOSED to be: the door opening is a hole in the front and
      // the jaws are what the tool reaches into.
      if (b.id === 'jaws') continue;
      for (const p of pts) {
        const lx = p[0] - mx;
        const inside = lx > b.x[0] + M && lx < b.x[1] - M && p[1] > b.y[0] + M && p[1] < b.y[1] - M
                     && p[2] > b.z[0] + M && p[2] < b.z[1] - M;
        if (inside) { bad.push(b.id + '@' + Math.round(mx) + ' at ' + p.map(v => Math.round(v)).join(',')); break; }
      }
    }
  }
  return bad;
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
  const keep = new Set(['rail', ...VS087.map(j => j.id), 'jawA', 'jawB']);
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
  const margin = (/** @type {Record<string, number>} */ d) => Math.min(...VS087.map(j => Math.min(d[j.id] - j.min, j.max - d[j.id])));
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
      const move = from ? Math.max(...VS087.map(j => Math.abs(r.dof[j.id] - dofs[from][j.id]))) : 0;
      const score = (bad.length ? -1000 : 0) + m - move / 4;
      if (!best || score > best.score) best = { dof: r.dof, m, err: r.err, bad, score };
    }
    if (!best) throw new Error('pose ' + k + ' did not solve');
    dofs[k] = best.dof;
    poses[k] = VS087.map(j => r2(best.dof[j.id]));
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
