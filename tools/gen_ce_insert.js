#!/usr/bin/env node
// @ts-check
// The ce-insert scene: Denny's CE INSERT machine for spark plugs, built from the machine's own
// iCAD SX model (M-5000011-000, "26-drajat") and his NX1P2 program (Ce Insert Track, Prepare CE
// insert4.smc2). Centre electrodes - 22 mm pins, 1.9 or 2.3 mm across, hanging by their heads -
// come out of two bowl feeders onto four 26-degree rail lanes (ST1), are metered in batches of 29
// into a four-lane shuttle on an electric slide (ST2), dropped onto a four-lane chute, and taken
// off it four at a time by a separator and a pusher into four pipes that end over the insulators
// (ST3).
//
//   node tools/gen_ce_insert.js                 write scenes/ce-insert.json and the constant blocks
//   node tools/gen_ce_insert.js --cad FILE.html also rebuild the shells from the iCAD export
//   node tools/gen_ce_insert.js --check         exit 1 when any committed output is stale
//
// Every number below comes from the CAD (read with tools/icad.js; the scene frame is the model's
// own, Z up, floor at 0), except where a comment says it is a choice. The shells built from the CAD
// are the maker's design and stay out of git (assets/cad/ is ignored): what IS committed is
// tools/ce_insert.cad.json, the list of shells with the boxes to draw when they are not there.
//
// Denny's word on the flow (2026-09-26): the left feeder fills lanes 1 and 2 of ST1, and the batch
// is buffered onto all four lanes of ST2 and ST3; the shuttle positions were left to this file; and
// of ST3's two cylinders one takes the pin out of the queue and the other pushes it so it drops into
// the pipe - his program has the two swapped (it drives the EJECTOR valve from its PUSHER symbols),
// which this port does not copy.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { stringify, validate, worldPoses, compile, mountFrom } from '../lib/scene.js';
import { compose, invert, apply, pose, qrot, IDENTITY } from '../lib/math.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const NAME = 'ce-insert';
const CAD_JSON = path.join(ROOT, 'tools', 'ce_insert.cad.json');
const ASSET_DIR = 'assets/cad/ce-insert';

// ---------------------------------------------------------------------------- geometry (mm)
/** Rail slope. The model is named "26-drajat", and every pin row in it falls 0.4877 per mm = tan 26. */
const ANG = 26, TAN = Math.tan(ANG * Math.PI / 180), COS = Math.cos(ANG * Math.PI / 180);
/**
 * The rail-top line all three stations share: the feeder lanes, the shuttle and the chute are ONE
 * straight 26-degree line (checked: the rail tops of all three lie on it within 0.6 mm). Taken from
 * the front 1.9 pin in the chute, whose head top is at z 1137.2 over y -396.8, less its 2 mm head.
 * @param {number} y
 */
const RT = y => 1135.2 + (y + 396.8) * TAN;
/** Lane centres across the machine: the 2.5 mm gaps between the rail pairs, e.g. -58.3..-55.8. */
const LX = [-57.05, -17.05, 22.95, 62.95];
/** Shuttle lanes on the shuttle: the same 40 mm pitch. In the model it stands at RIGHT (23..143). */
const SX = [-60, -20, 20, 60];
/** Where the stations meet along the flow (world y; the flow runs toward -y, the operator side). */
const Y_ST1 = 40, Y_12 = -149.2, Y_23 = -232.5, Y_END = -398.8;
/** s (mm down a lane) of a world y, from a lane start at y0. @param {number} y0 @param {number} y */
const sOf = (y0, y) => (y0 - y) / COS;
const L1 = sOf(Y_ST1, Y_12), L2 = sOf(Y_12, Y_23), L3 = sOf(Y_23, Y_END);
/**
 * ST1's gates. The dividers' plates cross the lanes at y -139.5. The gripper-stoppers are placed so
 * a batch is 29 pins: the shuttle holds 30 per lane before its stopper, and a 31st would be left
 * standing on the end of the feeder lane when the shuttle moves (a choice, inside the gripper's own
 * body in the model, y -77..-38).
 */
const S_DIV = sOf(Y_ST1, -139.5), S_GRIP = 111, S_FULL = 114.2;
/** ST2's stopper plate stands across the shuttle lanes at y -230.5. */
const S_STP1 = sOf(Y_12, -230.5);
/** ST3: the lane-full beam stands half way down the chute (the E32-T11N at y -320.5). */
const S_ST3FULL = sOf(Y_23, -320.5);
/** Servo positions (mm on the slide). CENTER lines the shuttle lanes up with the feeder and chute
 * lanes; LEFT and RIGHT shift it one lane pair (80 mm) so a feeder pair fills the other half. The
 * model shows it at RIGHT. The slide is an E-RBM6-S-220: 220 mm of stroke. */
export const SV = { CENTER: 110, LEFT: 30, RIGHT: 190 };
/** Track origin x over the slide's position: CENTER puts shuttle lane 1 on lane 1. */
const trackX = (/** @type {number} */ u) => LX[0] - SX[0] + (u - SV.CENTER);
/** The separator's slot: where the pins drop, 10 mm to -x of each lane (the CE RECEIVERs, x -65 for lane 1). */
const SLOT = 10;
/** The four insulators the pipes end over (round pallets at x -65 -25 15 55, y -385; top at z 975). */
const INS_Z = 975;

/** The pins, as the model draws them: 30-1.9 is 21.6 long with a 2.73 head 2 mm high over a 1.91
 * shank; 30-2.3 is 24.9 long, a 2.3 shank and a 2.95 collar that is what sits on the rails. */
const CE = {
  ce19: { size: [2.73, 1.91, 21.6], headH: 2, color: '#b87333', label: 'CE 1.9 (TYPE 1)' },
  ce23: { size: [2.95, 2.3, 22.9], headH: 1.5, color: '#9aa3ad', label: 'CE 2.3 (TYPE 2)' },
};

// ---------------------------------------------------------------------------- the scene
/** @param {any[]} shells */
export function build(shells) {
  /** @type {any[]} */
  const comps = [];
  const add = (/** @type {any} */ c) => { comps.push(c); return c; };
  const at = (/** @type {number[]} */ v) => v.map(x => Math.round(x * 1000) / 1000);
  const trackRot = [0, 0, -90];                // local +X (down the lane) = world -Y, local +Y = world +X

  for (const [id, c] of Object.entries(CE)) add({ id, type: 'workpiece', label: c.label + ' TEMPLATE', at: [0, 600, 0],
    params: { kind: 'pin', size: c.size, headH: c.headH, color: c.color, material: 'steel' } });

  // --------------------------------------------------------------- ST1: two bowl feeders, four lanes
  // MHZ2-6D finger grippers pinch the pin under them: STOPPER-2..5, one per lane, chuck = closed.
  const grips = ['st1Stp2', 'st1Stp3', 'st1Stp4', 'st1Stp5'];
  grips.forEach((id, i) => {
    const y = Y_ST1 - S_GRIP * COS, n = i + 2;
    add({ id, type: 'cylinder', label: 'ST1 STOPPER-' + n, station: 'ST1', at: at([LX[i] + 12, y, RT(y) + 20]), rot: [0, -90, 0],
      params: { bore: 6, stroke: 2, rod: 3, valve: '5/2-double', extendMs: 60, retractMs: 60, valveMs: 10, cushionMm: 0, reedBand: 1, reedHyst: 0.2,
        extWord: 'CHUCK', retWord: 'UNCHUCK', switches: [{ id: 'ret', pos: 0.2 }, { id: 'ext', pos: 1.8 }], hidden: true },
      io: { solExt: 'SOL_ST1_STP' + n + '_CHK', solRet: 'SOL_ST1_STP' + n + '_UCHK', 'sw.ret': 'AS_ST1_STP' + n + '_UCHK', 'sw.ext': 'AS_ST1_STP' + n + '_CHK' } });
  });
  // CDUJB6-10 dividers: a plate across a lane pair, out (forward) = closed. The left one pushes -x.
  const divs = [['st1DivL', 'LEFT', 'LFT', -74, [0, -90, 0]], ['st1DivR', 'RIGHT', 'RGT', 80, [0, 90, 0]]];
  for (const [id, word, w, x, rot] of divs) {
    add({ id, type: 'cylinder', label: 'ST1 ' + word + ' DIVIDER', station: 'ST1', at: [x, -128, 1281], rot,
      params: { bore: 6, stroke: 10, rod: 3, valve: '5/2-double', extendMs: 120, retractMs: 120, valveMs: 10, cushionMm: 1,
        extWord: 'FWD', retWord: 'BWD', ...shortSw(10) },
      io: { solExt: 'SOL_ST1_' + w + '_DIV_FWD', solRet: 'SOL_ST1_' + w + '_DIV_BWD', 'sw.ret': 'AS_ST1_' + w + '_DIV_BWD', 'sw.ext': 'AS_ST1_' + w + '_DIV_FWD' } });
  }
  add({ id: 'st1Trk', type: 'track', label: 'ST1 FEEDER LANES', station: 'ST1', at: at([0, Y_ST1, RT(Y_ST1)]), rot: trackRot,
    params: { lanes: LX, length: r3(L1), angle: ANG, next: 'st2Trk', drawRails: false,
      supply: [{ lanes: [0, 1], template: 'ce19', rate: 10 }, { lanes: [2, 3], template: 'ce23', rate: 10 }],
      gates: [
        ...grips.map((id, i) => ({ id, at: S_GRIP, lanes: [i], x: 1 })),
        { id: 'st1DivL', at: r3(S_DIV), lanes: [0, 1], x: 5 }, { id: 'st1DivR', at: r3(S_DIV), lanes: [2, 3], x: 5 },
      ] },
    io: { feed1: 'CR_ST1_PART_FDR1_STR', feed2: 'CR_ST1_PART_FDR2_STR' } });

  // --------------------------------------------------------------- ST2: the shuttle on the slide
  const u0 = SV.CENTER, yS = -171.5;
  add({ id: 'st2Srv', type: 'servoLinear', label: 'ST2 SERVO (E-RBM6)', station: 'ST2', at: at([trackX(u0) - (-384 / 2 + 164 / 2 + u0), yS, 1083]),
    params: { stroke: 220, vmax: 200, acc: 2000, body: [384, 60, 67], jogPct: 10 },
    io: { target: 'SV_ST2_TGT', exec: 'SV_ST2_EXEC', done: 'SV_ST2_DONE', busy: 'SV_ST2_BUSY', actPos: 'SV_ST2_POS', inPos: 'SV_ST2_INPOS',
          ovr: 'OVR', jogP: 'SV_ST2_JOG_P', jogN: 'SV_ST2_JOG_N' } });
  // placed on the carriage after the servo exists: see mountAll below
  add({ id: 'st2Trk', type: 'track', label: 'ST2 SHUTTLE', station: 'ST2', parent: 'st2Srv', socket: 'carriage',
    params: { lanes: SX, length: r3(L2), angle: ANG, next: 'st3Trk', drawRails: false,
      gates: [{ id: 'st2Stp1', at: r3(S_STP1), lanes: [0, 1, 2, 3], x: 5 }] }, want: { p: [trackX(u0), Y_12, RT(Y_12)], rot: trackRot } });
  // CDQ2A20-10 standing on the shuttle, rod down: down = blocking.
  add({ id: 'st2Stp1', type: 'cylinder', label: 'ST2 STOPPER-1', station: 'ST2', parent: 'st2Srv', socket: 'carriage',
    params: { bore: 20, stroke: 10, valve: '5/2-double', extendMs: 120, retractMs: 120, valveMs: 10, cushionMm: 1, extWord: 'DOWN', retWord: 'UP', ...shortSw(10) },
    io: { solExt: 'SOL_ST2_STP1_DN', solRet: 'SOL_ST2_STP1_UP', 'sw.ret': 'AS_ST2_STP1_UP', 'sw.ext': 'AS_ST2_STP1_DN' },
    want: { p: [trackX(u0) + 16, -247, 1304], rot: [180, 0, 0] } });

  // --------------------------------------------------------------- ST3: the chute and the escapement
  // The separator (CE EJECTOR plate, on a CDQ2A20-10 pushing -x) and the pusher (PUSHER CE, on a
  // MGJ10-10 pushing +y, from the operator side).
  add({ id: 'st3Sep', type: 'cylinder', label: 'ST3 SEPARATOR (CE EJECTOR)', station: 'ST3', at: [-79, -352, 1131], rot: [0, -90, 0],
    params: { bore: 20, stroke: SLOT, valve: '5/2-double', extendMs: 150, retractMs: 150, valveMs: 10, cushionMm: 1, extWord: 'FWD', retWord: 'BWD', ...shortSw(SLOT) },
    io: { solExt: 'SOL_ST3_EJC_FWD', solRet: 'SOL_ST3_EJC_BWD', 'sw.ret': 'AS_ST3_EJC_BWD', 'sw.ext': 'AS_ST3_EJC_FWD' } });
  add({ id: 'st3Psh', type: 'cylinder', label: 'ST3 PUSHER', station: 'ST3', at: [-5, -474, 1122], rot: [-90, 0, 0],
    params: { bore: 10, stroke: 10, valve: '5/2-double', extendMs: 150, retractMs: 150, valveMs: 10, cushionMm: 1, extWord: 'FWD', retWord: 'BWD', ...shortSw(10) },
    io: { solExt: 'SOL_ST3_PSH_FWD', solRet: 'SOL_ST3_PSH_BWD', 'sw.ret': 'AS_ST3_PSH_BWD', 'sw.ext': 'AS_ST3_PSH_FWD' } });
  add({ id: 'st3Trk', type: 'track', label: 'ST3 CHUTE', station: 'ST3', at: at([0, Y_23, RT(Y_23)]), rot: trackRot,
    params: { lanes: LX, length: r3(L3), angle: ANG, drawRails: false, escape: { sep: 'st3Sep', push: 'st3Psh', pushAt: 9 } },
    io: { count: 'ST3_DROP_CNT' } });
  // The insulators: a pin whose centre drops into this box has gone into one. One zone for all four.
  add({ id: 'rmIns', type: 'remover', label: 'INSULATORS', station: 'ST3', at: [(LX[0] + LX[3]) / 2 - SLOT, -385, INS_Z - 25],
    params: { size: [170, 30, 50] }, io: { count: 'CE_INS_CNT' } });

  // --------------------------------------------------------------- sensors (E32 fibre heads)
  // Beams at head level see a queue as a solid row (heads overlap in projection); beams at shank
  // level can pass between two touching pins. So queue-presence beams are at head level. A single
  // pin running past cuts a beam for about 14 ms (measured: "pulse stretched 14 -> 20 ms" every
  // 100 ms while a lane filled), so every beam has an off-delay, as a real fibre amplifier is set to: 30 ms still left 14 ms
  // dark gaps between pins arriving from the bowl, 60 ms does not.
  const eye = (/** @type {string} */ id, /** @type {string} */ label, /** @type {string} */ st, /** @type {number[]} */ p, /** @type {number[]} */ rot, /** @type {number} */ range, /** @type {string} */ tag) =>
    add({ id, type: 'photoEye', label, station: st, at: at(p), rot, params: { range, offDelayMs: 60 }, io: { out: tag } });
  LX.forEach((x, i) => { const y = Y_ST1 - S_FULL * COS; eye('st1Full' + (i + 1), 'ST1 SHUTTER-' + (i + 1) + ' FULL', 'ST1', [x + 8, y, RT(y) + 1], [0, 0, 180], 16, 'PH_ST1_SHT' + (i + 1) + '_FULL'); });
  { const y = -144.6; eye('st1Out', 'ST1 INSERT FLOW OUT', 'ST1', [-110, y, RT(y) - 10], [0, 0, 0], 220, 'PH_ST1_INS_FLW_OUT'); }
  { const y = -233.5; eye('st2Out', 'ST2 INSERT FLOW OUT', 'ST2', [-110, y, RT(y) - 10], [0, 0, 0], 220, 'PH_ST2_INS_FLW_OUT'); }
  { const y = Y_23 - S_ST3FULL * COS; eye('st3Full', 'ST3 SHUTTER FULL', 'ST3', [-110, y, RT(y) + 1], [0, 0, 0], 220, 'PH_ST3_SHT_FULL'); }
  LX.forEach((x, i) => {
    const y = Y_23 - (L3 - 1.5) * COS;
    eye('st3Exs' + (i + 1), 'ST3 SHUTTER-' + (i + 1) + ' EXIST', 'ST3', [x + 5, y, RT(y) - 3], [0, 0, 180], 20, 'PH_ST3_SHT' + (i + 1) + '_EXS');
    eye('st3Flw' + (i + 1), 'ST3 SHUTTER-' + (i + 1) + ' FLOW OUT', 'ST3', [x - SLOT, -420, 1033], [0, 0, 90], 70, 'PH_ST3_SHT' + (i + 1) + '_FLW_OUT');
  });

  // --------------------------------------------------------------- the operator panel
  const pb = (/** @type {string} */ id, /** @type {string} */ label, /** @type {any} */ params, /** @type {any} */ io) => add({ id, type: 'pushbutton', label, params, io });
  add({ id: 'sel', type: 'selector', label: 'AUTO / INDIVIDUAL', io: { sel: 'SEL_AUTO' } });
  pb('pbMaster', 'MASTER ON', { kind: 'momentary', color: 'white', lamp: true }, { pb: 'PB_MASTER', lamp: 'PL_MASTER' });
  pb('pbEstop', 'EMERGENCY STOP', { kind: 'alternate', color: 'red', lamp: false }, { pb: 'PB_ESTOP' });
  pb('pbStart', 'START', { kind: 'momentary', color: 'green', lamp: true }, { pb: 'PB_START', lamp: 'PL_START' });
  pb('pbCstop', 'CYCLE STOP', { kind: 'momentary', color: 'yellow', lamp: false }, { pb: 'PB_CSTOP' });
  pb('pbHome', 'HOME POS', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'PB_HOME', lamp: 'PL_HOME' });
  // The type is a setting on the real machine's HMI (TYPE_1). Here it is a latching button: in = 2.3.
  pb('pbType', 'TYPE 2 (CE 2.3)', { kind: 'alternate', color: 'white', lamp: false }, { pb: 'SEL_TYPE2' });
  add({ id: 'lampAuto', type: 'lamp', label: 'AUTO RUNNING', params: { color: 'amber' }, io: { lamp: 'AUTO_RUN' } });
  add({ id: 'dialOvr', type: 'speedDial', label: 'SPEED OVERRIDE', params: { min: 10 }, io: { ovr: 'OVR_SET' } });
  pb('pbJogP', 'JOG SHUTTLE +', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_SV_P', lamp: 'SV_ST2_JOG_P' });
  pb('pbJogN', 'JOG SHUTTLE -', { kind: 'momentary', color: 'blue', lamp: true }, { pb: 'IND_SV_N', lamp: 'SV_ST2_JOG_N' });
  for (const [id, label, tag, lamp] of [
    ['pbIndF1', 'FEEDER-1 (LEFT)', 'IND_FDR1', 'CR_ST1_PART_FDR1_STR'], ['pbIndF2', 'FEEDER-2 (RIGHT)', 'IND_FDR2', 'CR_ST1_PART_FDR2_STR'],
    ['pbIndS2', 'STOPPER-2', 'IND_STP2', 'SOL_ST1_STP2_UCHK'], ['pbIndS3', 'STOPPER-3', 'IND_STP3', 'SOL_ST1_STP3_UCHK'],
    ['pbIndS4', 'STOPPER-4', 'IND_STP4', 'SOL_ST1_STP4_UCHK'], ['pbIndS5', 'STOPPER-5', 'IND_STP5', 'SOL_ST1_STP5_UCHK'],
    ['pbIndDL', 'LEFT DIVIDER', 'IND_DIVL', 'SOL_ST1_LFT_DIV_BWD'], ['pbIndDR', 'RIGHT DIVIDER', 'IND_DIVR', 'SOL_ST1_RGT_DIV_BWD'],
    ['pbIndS1', 'ST2 STOPPER-1', 'IND_STP1', 'SOL_ST2_STP1_UP'],
    ['pbIndSep', 'SEPARATOR', 'IND_SEP', 'SOL_ST3_EJC_FWD'], ['pbIndPsh', 'PUSHER', 'IND_PSH', 'SOL_ST3_PSH_FWD'],
  ]) pb(id, label, { kind: 'momentary', color: 'blue', lamp: true }, { pb: tag, lamp });
  for (const [id, label, tag, lamp] of [['pbSvC', 'SHUTTLE CENTER', 'IND_SV_CTR', 'PL_SV_CTR'], ['pbSvL', 'SHUTTLE LEFT', 'IND_SV_LFT', 'PL_SV_LFT'],
    ['pbSvR', 'SHUTTLE RIGHT', 'IND_SV_RGT', 'PL_SV_RGT']]) pb(id, label, { kind: 'momentary', color: 'blue', lamp: true }, { pb: tag, lamp });

  // --------------------------------------------------------------- the maker's CAD
  for (const s of shells) {
    const c = { id: s.id, type: 'shell', label: s.label, params: { asset: s.asset, scale: 1, color: s.color, boxes: s.boxes } };
    // A moving shell's STL is written in its socket's frame (tools --cad), so it mounts with no offset.
    if (s.parent) Object.assign(c, { parent: s.parent, socket: s.socket });
    add(c);
  }

  const scene = {
    format: 'mio-scene/1', name: NAME,
    sim: { dtMs: 2 },
    io: { driver: 'opcua', endpoint: 'opc.tcp://127.0.0.1:4840', prefix: 'GlobalVars.', mode: 'sim', minPulseMs: 20 },
    stations: [
      { id: 'ST1', name: 'Supply feeder', members: comps.filter(c => c.station === 'ST1').map(c => c.id), stepTag: 'ST1_STEP' },
      { id: 'ST2', name: 'Buffer', members: comps.filter(c => c.station === 'ST2').map(c => c.id), stepTag: 'ST2_STEP' },
      { id: 'ST3', name: 'CE eject', members: comps.filter(c => c.station === 'ST3').map(c => c.id), stepTag: 'ST3_STEP' },
    ],
    cycle: { exitTag: 'PH_ST3_SHT1_FLW_OUT', autoTag: 'AUTO_RUN', countTag: 'CYCLE_CNT', exclude: 1, avgN: 10 },
    components: comps,
  };
  mountAll(scene);
  return scene;
}

/** @param {number} v */
function r3(v) { return Math.round(v * 1000) / 1000; }
/**
 * Reed switches for a 10 mm stroke: 0.3 mm from each end, 0.6 mm band. The type's default band
 * (6 mm) is right for a 100 mm cylinder and wrong here: measured on this scene, FWD came on at
 * 6.3 of 10 mm, the sequence reversed the pusher there, and it never got the pins over the pipes.
 * The 0.5 mm hysteresis keeps FWD on while a cylinder that is reversed the moment it arrives backs
 * off again: with 0.1 it read FWD for 14-18 ms, under the 20 ms the plant holds a pulse for.
 * @param {number} stroke
 */
const shortSw = stroke => ({ reedBand: 0.6, reedHyst: 0.5, switches: [{ id: 'ret', pos: 0.3 }, { id: 'ext', pos: stroke - 0.3 }] });

/**
 * Components that ride something (the shuttle on the slide) are given the WORLD pose they must
 * have with the machine at its CENTER/home dofs (`want`), and their at/rot on the parent socket
 * are solved here, so the numbers above stay the ones read off the CAD.
 * @param {any} scene
 */
function mountAll(scene) {
  const dof = homeDof();
  for (const c of scene.components) {
    if (!c.want) continue;
    const w = c.want;
    delete c.want;
    c.at = [0, 0, 0]; c.rot = [0, 0, 0];
    const tmp = { ...scene, components: scene.components.map((/** @type {any} */ x) => ({ ...x })) };
    const W = worldPoses(tmp, dof), { defs } = compile(tmp), pd = defs.get(c.parent), s = pd.sockets[c.socket];
    const base = compose(W[c.parent][s.link], pose(s.at, s.rot));
    const m = mountFrom(base, pose(w.p, w.rot));
    c.at = m.at; c.rot = m.rot;
  }
}
/** The dofs the scene's `want` poses are given at: shuttle CENTER, everything else retracted. */
const homeDof = () => ({ st2Srv: SV.CENTER });
/** The dofs the CAD model is drawn at: shuttle RIGHT, ST2's stopper down, the dividers forward. */
export const CAD_DOF = { st2Srv: SV.RIGHT, st2Stp1: 10, st1DivL: 10, st1DivR: 10 };

// ---------------------------------------------------------------------------- CAD shells
/**
 * Which CAD bodies go where. `simplify` is Denny's rule for this scene: it is a motion simulation,
 * so fasteners, fittings, sensor heads and bought-in actuators are not drawn from the CAD - the
 * actuators are drawn as the simulator's own primitives, which move - and the pneumatic box and
 * the levelling feet are plain boxes.
 */
const DROP = /^(CB|CBSST|MS|KQ2|AS1|AN|LSBG|RSCB|RCB|SEPN|FJX|FJR|C-FJR|LHFC|PSCDJ|PSFJC|C-SHH|C-NUWU|ISE|AW|AV|AR|FJFN|light|new|LAYOUT|30-1\.9|30-2\.3|E32|CDQ2A20|CDUJB6|MGJ10|E-RBM6|D4GS)/;
const COVER = /COVER/;
const BOXIFY = /^(M-6210063-000|BGOMAC35260|MHZ2-6D-M9BL)$/;
/** Moving groups: [shell id, parent component, socket, link, node names (with an x filter)]. */
const MOVING = [
  ['shlShuttle', 'st2Srv', 'carriage', 'carriage', ['M-2800017-030', 'M-2800017-040', 'M-2800017-050', 'M-2800017-060', 'M-2800017-070',
    'M-2800017-080', 'M-2800017-090', 'M-2800017-100', 'M-2800017-110', 'M-2800017-120']],
  ['shlStp1', 'st2Stp1', 'rodEnd', 'rod', ['M-2800017-130']],
  ['shlSep', 'st3Sep', 'rodEnd', 'rod', ['M-2800017-170', 'M-2800017-280', 'M-2800017-290']],
  ['shlPsh', 'st3Psh', 'rodEnd', 'rod', ['M-2800017-220', 'M-2800017-230']],
  ['shlDivL', 'st1DivL', 'rodEnd', 'rod', ['M-6600078-160', 'M-6600078-150<', 'M-6600078-180']],
  ['shlDivR', 'st1DivR', 'rodEnd', 'rod', ['M-6600078-170', 'M-6600078-150>', 'M-6600078-190']],
];

/**
 * Vertex clustering on a grid: every corner snaps to its cell, triangles that collapse go. A
 * machined plate keeps its outline and loses its chamfers and threads. @param {number[][]} tris @param {number} cell
 */
function cluster(tris, cell) {
  /** @type {number[][]} */
  const out = [];
  const q = (/** @type {number[]} */ p) => p.map(v => Math.round(v / cell) * cell);
  for (let i = 0; i < tris.length; i += 3) {
    const a = q(tris[i]), b = q(tris[i + 1]), c = q(tris[i + 2]);
    const k = (/** @type {number[]} */ p) => p.join();
    if (k(a) === k(b) || k(b) === k(c) || k(a) === k(c)) continue;
    out.push(a, b, c);
  }
  return out;
}
/** 12 triangles of an axis-aligned box. @param {number[]} b [minx, miny, minz, maxx, maxy, maxz] */
function boxTris(b) {
  const v = (/** @type {number} */ i) => [i & 1 ? b[3] : b[0], i & 2 ? b[4] : b[1], i & 4 ? b[5] : b[2]];
  const F = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];
  return F.flatMap(f => [v(f[0]), v(f[1]), v(f[2]), v(f[0]), v(f[2]), v(f[3])]);
}

/** Rebuild the shells from the iCAD export: STLs under assets/cad/ce-insert, the list in tools/ce_insert.cad.json. @param {string} file */
async function buildCad(file) {
  const { readIcad, stlBinary } = await import('./icad.js');
  const m = readIcad(file);
  // The decoder's layout is inferred, so check it on this very file: every body's triangles must
  // land inside the box the file stores for it.
  let worst = 0;
  for (const b of m.bodies) {
    for (const p of m.tris(b.id)) for (let d = 0; d < 3; d++) worst = Math.max(worst, b.box[d] - p[d], p[d] - b.box[d + 3]);
  }
  if (worst > 0.2) throw new Error('iCAD decode does not match the file\'s own boxes (off by ' + worst.toFixed(2) + ' mm): not this layout');
  const path2 = (/** @type {any} */ b) => { const out = []; for (let n = m.nodes[b.node]; n && n.id > 1; n = m.nodes[n.parent]) out.push(n); return out; };
  const scene0 = build([]);
  const W = worldPoses(scene0, { ...homeDof(), ...CAD_DOF }), { defs } = compile(scene0);
  /** @type {Map<string, {tris: number[][], boxes: number[][], label: string, color: string, parent?: string, socket?: string, link?: string}>} */
  const groups = new Map();
  const hex = (/** @type {number[]} */ c) => '#' + c.map(v => v.toString(16).padStart(2, '0')).join('');
  let kept = 0, raw = 0;
  for (const b of m.bodies) {
    const pth = path2(b), names = pth.map(n => n.name);
    const diag = Math.hypot(b.box[3] - b.box[0], b.box[4] - b.box[1], b.box[5] - b.box[2]);
    if (names.some(n => DROP.test(n)) || pth.some(n => COVER.test(n.desc)) || diag < 12) continue;
    const cx = (b.box[0] + b.box[3]) / 2;
    const mv = MOVING.find(g => /** @type {string[]} */ (g[4]).some(spec => {
      const side = spec.endsWith('<') ? -1 : spec.endsWith('>') ? 1 : 0, nm = side ? spec.slice(0, -1) : spec;
      return names.includes(nm) && (!side || Math.sign(cx) === side);
    }));
    const boxify = names.some(n => BOXIFY.test(n));
    let key, grp;
    if (mv) {
      key = /** @type {string} */ (mv[0]);
      grp = groups.get(key) || groups.set(key, { tris: [], boxes: [], label: key, color: '#d8d0b0', parent: /** @type {string} */ (mv[1]), socket: /** @type {string} */ (mv[2]), link: /** @type {string} */ (mv[3]) }).get(key);
    } else {
      const col = boxify ? [190, 190, 190] : b.color;
      key = 'shl_' + hex(col).slice(1);
      grp = groups.get(key) || groups.set(key, { tris: [], boxes: [], label: 'CAD ' + hex(col), color: hex(col) }).get(key);
    }
    const t = boxify ? boxTris(b.box) : m.tris(b.id);
    raw += t.length / 3;
    const c = boxify ? t : cluster(t, 0.8);
    kept += c.length / 3;
    grp.tris.push(...c);
    if (diag > 40 || grp.parent) grp.boxes.push(b.box);
  }
  fs.mkdirSync(path.join(ROOT, ASSET_DIR), { recursive: true });
  const shells = [];
  for (const [id, g] of groups) {
    // Moving shells are written in their parent link's frame at the CAD dofs.
    let F = IDENTITY;
    if (g.parent) {
      const s = defs.get(g.parent).sockets[/** @type {string} */ (g.socket)];
      F = compose(W[g.parent][s.link], pose(s.at, s.rot));
    }
    const inv = invert(F);
    const pts = F === IDENTITY ? g.tris : g.tris.map(p => apply(inv, p));
    const asset = ASSET_DIR + '/' + id + '.stl';
    fs.writeFileSync(path.join(ROOT, asset), stlBinary(pts));
    // Fallback boxes, in the same frame, largest first, at most 60 a shell.
    const bx = g.boxes.map(b => {
      const c = apply(inv, [(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2]);
      // The boxes of a moving shell are axis-aligned in the world; every socket used here is only
      // turned in multiples of 90 degrees, so the size is the same box with its axes permuted.
      const e = [b[3] - b[0], b[4] - b[1], b[5] - b[2]];
      const ax = [0, 1, 2].map(i => { const u = qrot(inv.q, [i === 0 ? 1 : 0, i === 1 ? 1 : 0, i === 2 ? 1 : 0]); return u.map(Math.abs); });
      const size = [0, 1, 2].map(k => ax[0][k] * e[0] + ax[1][k] * e[1] + ax[2][k] * e[2]);
      return [...c, ...size].map(v => Math.round(v * 10) / 10);
    }).sort((a, b) => b[3] * b[4] * b[5] - a[3] * a[4] * a[5]).slice(0, 60);
    shells.push({ id, label: g.label, asset, color: g.color, boxes: bx, ...(g.parent ? { parent: g.parent, socket: g.socket } : {}), tris: pts.length / 3 });
  }
  shells.sort((a, b) => (a.id < b.id ? -1 : 1));
  fs.writeFileSync(CAD_JSON, JSON.stringify({ source: path.basename(file), note: 'written by tools/gen_ce_insert.js --cad; the STLs are gitignored', shells }, null, 1) + '\n');
  console.log('CAD: ' + raw + ' triangles in, ' + kept + ' kept in ' + shells.length + ' shells (decode checked to ' + worst.toFixed(3) + ' mm)');
}

// ---------------------------------------------------------------------------- outputs
function outputs() {
  const cad = fs.existsSync(CAD_JSON) ? JSON.parse(fs.readFileSync(CAD_JSON, 'utf8')) : { shells: [] };
  const scene = build(cad.shells);
  const errs = validate(scene);
  if (errs.length) throw new Error('generated scene is invalid:\n  ' + errs.join('\n  '));
  const block = '// BEGIN GENERATED - tools/gen_ce_insert.js\n'
    + '/** Shuttle positions on the E-RBM6 slide, mm: CENTER lines the shuttle up with the feeder and chute lanes. */\n'
    + 'export const SV = ' + JSON.stringify(SV) + ';\n// END GENERATED';
  const stBlock = '// BEGIN GENERATED - tools/gen_ce_insert.js\n'
    + 'SV_CENTER := ' + SV.CENTER.toFixed(1) + ';\nSV_LEFT := ' + SV.LEFT.toFixed(1) + ';\nSV_RIGHT := ' + SV.RIGHT.toFixed(1) + ';\n// END GENERATED';
  return { scene: stringify(scene), block, stBlock };
}

/** Replace the generated block in a controller file. @param {string} txt @param {string} block */
function splice(txt, block) {
  const re = /\/\/ BEGIN GENERATED - tools\/gen_ce_insert\.js[\s\S]*?\/\/ END GENERATED/;
  if (!re.test(txt)) throw new Error('no GENERATED block');
  return txt.replace(re, block);
}

async function main() {
  const args = process.argv.slice(2);
  const ci = args.indexOf('--cad');
  if (ci >= 0) await buildCad(args[ci + 1]);
  const o = outputs();
  const files = [
    [path.join(ROOT, 'scenes', NAME + '.json'), () => o.scene],
    [path.join(ROOT, 'scenes', NAME + '.ctl.js'), (/** @type {string} */ t) => splice(t, o.block)],
    [path.join(ROOT, 'scenes', NAME + '.st'), (/** @type {string} */ t) => splice(t, o.stBlock)],
  ];
  let stale = 0;
  for (const [f, make] of files) {
    const have = fs.existsSync(/** @type {string} */ (f)) ? fs.readFileSync(/** @type {string} */ (f), 'utf8') : null;
    if (have == null && f !== files[0][0]) { console.log('skip (missing) ' + path.relative(ROOT, /** @type {string} */ (f))); continue; }
    const want = /** @type {Function} */ (make)(have);
    if (have === want) continue;
    if (args.includes('--check')) { console.log('STALE ' + path.relative(ROOT, /** @type {string} */ (f))); stale++; continue; }
    fs.writeFileSync(/** @type {string} */ (f), want);
    console.log('wrote ' + path.relative(ROOT, /** @type {string} */ (f)));
  }
  if (stale) process.exit(1);
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) main().catch(e => { console.error(e.stack || e.message); process.exit(1); });
