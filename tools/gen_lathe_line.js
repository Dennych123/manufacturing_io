#!/usr/bin/env node
// @ts-check
// The lathe-line scene, built from what the video of the real cell shows (docs in the scene's
// .ctl.js): a DENSO VS-087 hanging from a traverse beam over a row of TAKISAWA TCC-2000 lathes,
// one flat conveyor running past their fronts, and at each machine a stopper and a pin lift that
// raises the part to the robot.
//
//   node tools/gen_lathe_line.js           write scenes/lathe-line.json and the pose blocks
//   node tools/gen_lathe_line.js --check   exit 1 when any committed output is stale
//
// ONE source for the geometry and the joint angles. The scene, the controller's pose table and the
// ST twin's pose table are all written from here: 60 joint angles typed into two programs by hand
// drift, and the drift is silent. The IK runs here only (lib/ik.js), never in the plant.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { stringify, validate } from '../lib/scene.js';
import { solveIk } from '../lib/ik.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const NAME = 'lathe-line';

// ---------------------------------------------------------------------------- geometry (mm)
/** Machine centres along the line. A TCC-2000 is 1450 wide; the two stand 150 apart. */
export const MX = [-800, 800];
/** Machine front face (the aisle is -Y), depth 1400, height 1700. */
const MW = 1450, MD = 1400, MH = 1700;
/** The work window in the front: sill, head, and its X span relative to the machine centre. */
const WIN = { z0: 850, z1: 1300, x0: -625, x1: 50 };
/** Spindle: horizontal along X, the chuck face looking +X, relative to the machine centre. */
export const CHUCK = { dx: -420, y: 300, z: 1080 };
/** The part: a turned casting standing on its end. */
export const PART = { d: 50, h: 70 };
/** Conveyor: one flat lane along the fronts. */
export const CV = { y: -220, top: 820, len: 4400, w: 90, speed: 250 };
/** Pin lift at each machine, relative to the machine centre; it raises the part LIFT mm. */
export const PIN = { dx: -150, sink: 15, stroke: 150,
  // The nest sits PAD above the rod end. A cylinder's rod carries a 12 mm steel block at its end
  // (the shape every rod has), and a nest mounted flat on the rodEnd socket puts its 6 mm floor
  // straight through that block: measured, the part then stood on the 19 mm block instead of the
  // 62 mm floor, slid off it, sank 11 mm and toppled - and rode the rest of the line lying down.
  pad: 6 };
/** Stopper: pops up from under the belt just downstream of the pin. */
const STOP = { sink: 10, stroke: 40, head: [12, 70, 25] };
/** The traverse: J1's mounting face hangs at this height, the beam above the aisle. */
export const BEAM = { y: -150, j1z: 2150 };
/** Where the carriage stands for a machine, relative to the machine centre. */
export const RAIL_DX = -180;
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
/** The cycle does not run the arm at its catalogue maximum; neither does the cell in the video. */
const SPEED = 0.5;

/** The double hand: jaw A along the flange axis, jaw B at 90 degrees to it. */
const JAW = { span: 100, fingerLen: 50, fingerW: 10 };
const TCP = JAW.fingerLen * 0.6;                       // the gripper's catch-zone centre

/** @param {number} v */
const r2 = v => Math.round(v * 100) / 100 || 0;

export function buildScene() {
  /** @type {any[]} */
  const C = [];
  const add = (/** @type {any} */ c) => { C.push(c); return c; };
  const railZ = BEAM.j1z + SK.carriage;

  // ---- the traverse beam and its columns
  add({ id: 'beam', type: 'frame', label: 'TRAVERSE BEAM', at: [0, BEAM.y, railZ + 96], params: { size: [5550, 220, 260], style: 'solid', color: '#c8ccd0' } });
  for (const [id, x] of [['colL', -2700], ['colR', 2700]]) {
    add({ id, type: 'frame', label: 'BEAM COLUMN', at: [x, BEAM.y, 0], params: { size: [160, 160, railZ + 96], style: 'solid', color: '#b8bdc2' } });
  }
  const jog = (/** @type {string} */ p) => ({ ovr: 'OVR', jogP: p + '_JOG_P', jogN: p + '_JOG_N' });
  const axisIo = (/** @type {string} */ p) => ({ target: p + '_TGT', exec: p + '_EXEC', done: p + '_DONE', busy: p + '_BUSY', actPos: p + '_POS', inPos: p + '_INPOS', ...jog(p) });
  // Turned over about X: the carriage hangs under the rail and everything on it hangs too.
  add({ id: 'rail', type: 'joint', label: 'TRAVERSE AXIS', station: 'ST1', at: [0, BEAM.y, railZ], rot: [180, 0, 0],
        params: { kind: 'prismatic', axis: 'x', len: 0, min: -1300, max: 1300, home: MX[0] + RAIL_DX, vmax: 1000, acc: 2500, width: 160, jogPct: 10 },
        io: axisIo('RX') });

  // ---- the robot
  VS087.forEach((j, i) => {
    add({ id: j.id, type: 'joint', label: 'VS-087 J' + (i + 1), station: 'ST1', parent: i ? VS087[i - 1].id : 'rail', socket: 'end',
          at: i ? [0, 0, 0] : [0, 0, SK.carriage],
          params: { kind: 'revolute', axis: j.axis, len: 0, to: j.to, min: j.min, max: j.max, home: 0,
                    vmax: r2(j.v * SPEED), acc: r2(j.v * SPEED * 3), width: j.w, color: '#f1efe8', jogPct: 5 },
          io: axisIo('J' + (i + 1)) });
  });
  const jaw = { span: JAW.span, fingerLen: JAW.fingerLen, fingerW: JAW.fingerW, closeMs: 300, openMs: 300, band: 1.5 };
  add({ id: 'hand', type: 'plate', label: 'DOUBLE HAND BLOCK', parent: 'j6', socket: 'end', at: [0, 0, -45], rot: [0, 90, 0], params: { size: [90, 70, 70] } });
  add({ id: 'jawA', type: 'gripper', label: 'HAND A (raw)', station: 'ST1', parent: 'j6', socket: 'end', at: [70, 0, 0], rot: [0, 90, 0], params: jaw,
        io: { close: 'GRIP_A', open: 'AS_A_OPEN', closed: 'AS_A_CLOSED' } });
  add({ id: 'jawB', type: 'gripper', label: 'HAND B (finished)', station: 'ST1', parent: 'j6', socket: 'end', at: [35, 0, 35], rot: [0, 0, 0], params: jaw,
        io: { close: 'GRIP_B', open: 'AS_B_OPEN', closed: 'AS_B_CLOSED' } });

  add({ id: 'partTpl', type: 'workpiece', label: 'CASTING (template)', at: [0, -1500, 0],
        params: { kind: 'cyl', size: [PART.d, PART.d, PART.h], color: '#7b4a2c', material: 'steel' } });

  // ---- the conveyor, the feeder at its head and the discharge off its end
  add({ id: 'cv', type: 'conveyor', label: 'LINE CONVEYOR', station: 'ST4', at: [0, CV.y, 0],
        params: { length: CV.len, width: CV.w, height: CV.top, speed: CV.speed, guides: 30 }, io: { run: 'CV_RUN', ovr: 'OVR' } });
  add({ id: 'emIn', type: 'emitter', label: 'PART FEED', station: 'ST4', parent: 'cv', socket: 'top', at: [-CV.len / 2 + 150, 0, 5],
        params: { template: 'partTpl', mode: 'tag' }, io: { emit: 'EM_IN_EMIT', count: 'EM_IN_CNT' } });
  // Off the END into free air, caught by a zone: nothing solid stands in the part's path (CLAUDE.md).
  add({ id: 'rmOut', type: 'remover', label: 'NEXT PROCESS', parent: 'cv', socket: 'end', at: [220, 0, -700], params: { size: [300, 300, 500] }, io: { count: 'RM_OUT_CNT' } });

  MX.forEach((mx, k) => {
    const n = k + 1, M = 'M' + n, S = 'S' + n, st = 'ST' + (4 + k);
    // ---- the lathe, built as blocks around a hollow work area: a part in the chuck or in the hand
    // is kinematic, and the one step it is dynamic on a hand-over must not be inside a solid.
    const body = '#d8d3c4', dark = '#2c3034';
    const blk = (/** @type {string} */ id, /** @type {string} */ label, /** @type {number[]} */ x, /** @type {number[]} */ y, /** @type {number[]} */ z, /** @type {string} */ color) =>
      add({ id: id + n, type: 'frame', label: 'TCC-2000 #' + n + ' ' + label, at: [mx + (x[0] + x[1]) / 2, (y[0] + y[1]) / 2, z[0]],
            params: { size: [x[1] - x[0], y[1] - y[0], z[1] - z[0]], style: 'solid', color } });
    blk('bed', 'BED', [-MW / 2, MW / 2], [0, MD], [0, WIN.z0], body);
    blk('tower', 'NC SIDE', [WIN.x1, MW / 2], [0, MD], [WIN.z0, MH], body);
    blk('back', 'HEADSTOCK SIDE', [-MW / 2, WIN.x1], [CHUCK.y + 250, MD], [WIN.z0, MH], body);
    blk('wall', 'LEFT WALL', [-MW / 2, WIN.x0], [0, CHUCK.y + 250], [WIN.z0, MH], body);
    blk('hood', 'DOOR HOOD', [WIN.x0, WIN.x1], [0, CHUCK.y + 250], [WIN.z1, MH], dark);
    blk('head', 'SPINDLE', [WIN.x0, CHUCK.dx - 80], [CHUCK.y - 130, CHUCK.y + 130], [CHUCK.z - 130, CHUCK.z + 130], '#6b7178');
    blk('jaws', 'CHUCK', [CHUCK.dx - 80, CHUCK.dx - 6], [CHUCK.y - 105, CHUCK.y + 105], [CHUCK.z - 105, CHUCK.z + 105], '#3a3f45');
    blk('nc', 'NC PANEL', [WIN.x1 + 250, WIN.x1 + 560], [-80, 0], [WIN.z0 + 100, WIN.z0 + 700], dark);
    blk('lampG', 'LAMP GREEN', [MW / 2 - 90, MW / 2 - 40], [MD - 90, MD - 40], [MH, MH + 60], '#27b045');
    blk('lampY', 'LAMP AMBER', [MW / 2 - 90, MW / 2 - 40], [MD - 90, MD - 40], [MH + 60, MH + 120], '#ff9a1f');
    blk('lampR', 'LAMP RED', [MW / 2 - 90, MW / 2 - 40], [MD - 90, MD - 40], [MH + 120, MH + 180], '#e0322c');

    // The chuck: a nest turned so its pocket looks +X, the way the spindle does.
    add({ id: 'chuck' + n, type: 'nest', label: 'TCC-2000 #' + n + ' CHUCK', station: 'ST' + (2 + k), at: [mx + CHUCK.dx, CHUCK.y, CHUCK.z], rot: [0, 90, 0],
          params: { size: [PART.d + 6, PART.d + 6, 40], wall: 8 }, io: { clamp: M + '_CHUCK', present: M + '_PART' } });
    // The automatic door: a cylinder beside the window lifts the leaf, which rides its rod end.
    const bore = 20, Lb = (WIN.z1 - WIN.z0 + 20) + bore + 20, foot = 250;
    add({ id: 'door' + n, type: 'cylinder', label: 'TCC-2000 #' + n + ' DOOR CYL', station: 'ST' + (2 + k), at: [mx + WIN.x1 + 40, -40, foot],
          params: { bore, stroke: WIN.z1 - WIN.z0 + 20, valve: '5/2-single', extendMs: 900, retractMs: 900, extWord: 'OPEN', retWord: 'CLOSE' },
          io: { solExt: 'SOL_' + M + '_DOOR', 'sw.ret': 'AS_' + M + '_DOOR_CL', 'sw.ext': 'AS_' + M + '_DOOR_OP' } });
    add({ id: 'leaf' + n, type: 'plate', label: 'TCC-2000 #' + n + ' DOOR', parent: 'door' + n, socket: 'rodEnd',
          at: [(WIN.x0 + WIN.x1) / 2 - (WIN.x1 + 40), 12, WIN.z0 - 10 - (foot + Lb + 27)], params: { size: [WIN.x1 - WIN.x0 + 40, 12, WIN.z1 - WIN.z0 + 20] } });

    // ---- the station on the conveyor: a pop-up stop, a pin lift, a beam
    const xp = mx + PIN.dx;
    const liftFoot = CV.top - PIN.sink - (PIN.stroke + 32 + 20) - 27 - PIN.pad;
    add({ id: 'lift' + n, type: 'cylinder', label: 'STATION ' + n + ' PIN LIFT', station: st, at: [xp, CV.y, liftFoot],
          params: { bore: 32, stroke: PIN.stroke, valve: '5/2-double', extendMs: 450, retractMs: 450, extWord: 'UP', retWord: 'DOWN' },
          io: { solExt: 'SOL_' + S + '_UP', solRet: 'SOL_' + S + '_DN', 'sw.ret': 'AS_' + S + '_DN', 'sw.ext': 'AS_' + S + '_UP' } });
    add({ id: 'pin' + n, type: 'nest', label: 'STATION ' + n + ' PIN', station: st, parent: 'lift' + n, socket: 'rodEnd', at: [0, 0, PIN.pad],
          params: { size: [PART.d + 4, PART.d + 4, 40], wall: 4 }, io: { clamp: S + '_CLAMP', present: 'PX_' + S } });
    const [, , hz] = STOP.head, sLb = STOP.stroke + 16 + 20;
    add({ id: 'stop' + n, type: 'cylinder', label: 'STATION ' + n + ' STOPPER', station: st, at: [xp + PART.d / 2 + STOP.head[0] / 2 + 1, CV.y, CV.top - STOP.sink - hz - 27 - sLb],
          params: { bore: 16, stroke: STOP.stroke, valve: '5/2-single', extendMs: 120, retractMs: 120, extWord: 'UP', retWord: 'DOWN', head: 'plate', headSize: STOP.head },
          io: { solExt: 'SOL_' + S + '_STOP', 'sw.ret': 'AS_' + S + '_STOP_DN', 'sw.ext': 'AS_' + S + '_STOP_UP' } });
    add({ id: 'eye' + n, type: 'photoEye', label: 'STATION ' + n + ' PART BEAM', station: st, at: [xp, CV.y - CV.w / 2 - 40, CV.top + 30], rot: [0, 0, 90],
          // 150 ms off-delay, as a real beam is set. The part crosses the ray again as the pin
          // takes it up and puts it back down, and a part rocking as it lands off the pin flickers
          // the ray for tens of milliseconds: measured, an 8 ms and a 20 ms gap at station 2 in
          // three minutes, which the plant then reports as a stretched pulse. It is the SENSOR
          // that holds a blip like that on a real line, not the PLC and not the plant.
          params: { range: CV.w + 80, offDelayMs: 150 }, io: { out: 'PE_' + S } });
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
  pb('pbIndL1', 'PIN LIFT #1', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_LIFT1', lamp: 'SOL_S1_UP' });
  pb('pbIndL2', 'PIN LIFT #2', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_LIFT2', lamp: 'SOL_S2_UP' });

  return {
    format: 'mio-scene/1', name: NAME, sim: { dtMs: 4 },
    io: { driver: 'opcua', endpoint: 'opc.tcp://127.0.0.1:4840', prefix: 'GlobalVars.', mode: 'sim', minPulseMs: 20 },
    stations: [
      { id: 'ST1', name: 'Robot', members: ['rail', ...VS087.map(j => j.id), 'jawA', 'jawB'], stepTag: 'ST1_STEP' },
      { id: 'ST2', name: 'TCC-2000 #1 (OP10)', members: ['chuck1', 'door1'], stepTag: 'ST2_STEP' },
      { id: 'ST3', name: 'TCC-2000 #2 (OP20)', members: ['chuck2', 'door2'], stepTag: 'ST3_STEP' },
      { id: 'ST4', name: 'Station 1 + feed', members: ['cv', 'emIn', 'lift1', 'pin1', 'stop1', 'eye1'], stepTag: 'ST4_STEP' },
      { id: 'ST5', name: 'Station 2 + discharge', members: ['lift2', 'pin2', 'stop2', 'eye2', 'rmOut'], stepTag: 'ST5_STEP' },
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
 * Every pose the program commands, as world goals with the carriage at machine 1. A pose is joint
 * angles only, so it serves machine 2 unchanged with the carriage one machine pitch along.
 * Each jaw has its own five: over the pin, on the pin, outside the door, inside the machine clear
 * of the chuck, and at the chuck.
 */
export function goals() {
  const mx = MX[0], pinZ = CV.top - PIN.sink + PIN.stroke + PART.h / 2;
  const xp = mx + PIN.dx, xc = mx + CHUCK.dx + PART.h / 2;
  /** @type {Record<string, {jaw: string, p: number[], dir: number[]}>} */
  const g = {};
  for (const J of ['A', 'B']) {
    const jaw = 'jaw' + J;
    g['pinAt' + J] = { jaw, p: [xp, CV.y, pinZ], dir: [0, 0, -1] };
    g['pinUp' + J] = { jaw, p: [xp, CV.y, pinZ + 130], dir: [0, 0, -1] };
    g['chAt' + J] = { jaw, p: [xc, CHUCK.y, CHUCK.z], dir: [-1, 0, 0] };
    g['chOut' + J] = { jaw, p: [xc + 150, CHUCK.y, CHUCK.z], dir: [-1, 0, 0] };
  }
  // One pose in front of the open door, for hand A: the arm goes in and comes out through it with
  // either jaw, and a second door pose would only be the same place with the wrist turned.
  g.doorA = { jaw: 'jawA', p: [xc + 150, -330, CHUCK.z], dir: [-1, 0, 0] };
  g.home = { jaw: 'jawA', p: [xp + 100, CV.y + 60, pinZ + 230], dir: [0, 0, -1] };
  return g;
}
/** Solve order: the free ones first, then the poses that must stay in the same arm configuration.
 * @type {Array<[string, string|null]>} */
const CHAIN = [
  ['chAtA', null], ['chOutA', 'chAtA'], ['doorA', 'chOutA'],
  ['chAtB', null], ['chOutB', 'chAtB'],
  ['pinAtA', null], ['pinUpA', 'pinAtA'], ['pinAtB', null], ['pinUpB', 'pinAtB'], ['home', 'pinUpA'],
];
export const ORDER = CHAIN.map(([k]) => k);

/**
 * The joint angles for every goal, solved against the scene's own kinematics. A pose that follows
 * another is seeded FROM it: solved freshly, chOut came back in a different arm configuration from
 * chAt, which would have the arm flip itself over inside the machine between two steps.
 * @param {any} scene
 */
export function solvePoses(scene) {
  const g = goals();
  // The IK only needs the arm, and worldPoses() walks every component it is given: solving against
  // the whole cell (150 components, five of them lathes) costs about eight times as much.
  const keep = new Set(['rail', ...VS087.map(j => j.id), 'jawA', 'jawB']);
  const arm = { ...scene, components: scene.components.filter((/** @type {any} */ c) => keep.has(c.id)) };
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
    const starts = from ? [dofs[from]] : seeds;
    let best = null;
    for (const s of starts) {
      const r = solveIk(arm, JOINTS, tool(G.jaw), G, { start: s, iters: 1000, lambda: 4 });
      if (!r.ok) continue;
      const m = margin(r.dof);
      if (!best || m > best.m) best = { dof: r.dof, m, err: r.err };
    }
    if (!best) throw new Error('pose ' + k + ' did not solve from the carriage at ' + (MX[0] + RAIL_DX));
    dofs[k] = best.dof;
    poses[k] = VS087.map(j => r2(best.dof[j.id]));
    report.push(k.padEnd(8) + ' err ' + best.err.toFixed(3) + ' mm, margin ' + best.m.toFixed(1) + ' deg');
  }
  return { poses, report };
}

// ---------------------------------------------------------------------------- writing
/** The pose table as the controller declares it. @param {Record<string, number[]>} poses */
function ctlBlock(poses) {
  const rows = ORDER.map(k => ' ' + JSON.stringify(k) + ':' + JSON.stringify(poses[k]));
  return ['/** J1..J6 in degrees, one entry per goal. The rail owns X, so a pose serves either machine. */',
    'export const POSE = {' + rows.join(',\n').replace(/^ /, '') + '};',
    '/** Where the carriage stands to work at machine n (0-based), and its parking place. */',
    'export const RAIL = ' + JSON.stringify(MX.map(mx => mx + RAIL_DX)) + ';'].join('\n');
}

/**
 * The ST twin's pose table. ST has no array literal an XML import will take, so the targets are
 * assigned by a CASE on GO_POSE, which every move step sets. Generated for the same reason as the
 * controller's table: 66 angles typed twice drift, silently.
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
  const { poses, report } = solvePoses(scene);
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
