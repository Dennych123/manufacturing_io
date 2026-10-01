#!/usr/bin/env node
// @ts-check
// The final-caulking scene: the add-on unit that feeds a Final Caulking machine (horn assembly:
// a diaphragm and an M&B are stacked, the existing machine caulks them), run by the machine's OWN
// CX-Programmer program in the ladder soft-PLC (server/ladder.js) - the NEW design's project
// (plc/final-caulking-mdf.cxp, "newwww mdf"; gitignored, it is the company's program).
//
//   node tools/gen_final_caulking.js               write scenes/final-caulking.json
//   node tools/gen_final_caulking.js --cad FILE    also rebuild the shells from the iCAD export
//                                                  (MT0190133-0002-OT-3.html)
//   node tools/gen_final_caulking.js --check       exit 1 when the committed scene is stale
//
// Sources, and what is a choice:
//   - The target is the NEW design (Denny, 2026-09-30): the CAD plus the newer program, as a digital
//     twin to find where the cycle time goes. The line machine still has two cover cylinders and no
//     MXQ8 load slide; this one has the MXQ8 (LOAD FWD/BWD 3211.15/.14), a light curtain where the
//     outer cover was, and shutter 1 still WIRED (the program runs it) but taken off mechanically.
//   - IO: every address below is the program's own (its symbol table). The unit each one drives
//     was read from which program section writes it (P11..P17) and from the sequence comments.
//   - Flow (Denny): diaphragms come in by hand at the LEFT end through the light curtain, M&Bs by
//     hand at the RIGHT end onto the turntable; both are confirmed with a "nagara" start switch
//     (3301.15, 3304.07). The finished horn leaves on the conveyor to the left, NG to a box.
//   - Layout and heights are the CAD's: the line runs along X at y 222, diaphragm stations S0..S2 at
//     x -145, 55, 205 (floor 977), the transfer slide from 355 to the station at 475 (floor 938), the
//     M&B vacuum area at (675, 222) and the turntable at (875, 285) (floor 900), the rotary transfer's
//     axis at (475, 351) between the station and Final Caulking ST1 (475, 485).
//   - Everything that moves is drawn by the CAD: each moving body rides the component that moves it,
//     and where a slide's body is fixed and its table travels, the table's bodies are named one by one
//     (BODY_RIDES) - a whole MXQ riding its gripper left the gripper floating 125 mm off the slide.
//   - Not in this CAD, so modelled as IO only (hidden): shutter 1 (not installed) and the oil-apply
//     unit (3207/3307). The Final Caulking machine (existing, next door) is a simple model: its
//     two-station table and its press, drawn as primitives.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { stringify, validate, worldPoses, compile, mountFrom } from '../lib/scene.js';
import { compose, invert, apply, pose, qrot, IDENTITY } from '../lib/math.js';
import { cylGeometry } from '../lib/components.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const NAME = 'final-caulking';
const CAD_JSON = path.join(ROOT, 'tools', 'final_caulking.cad.json');
const ASSET_DIR = 'assets/cad/final-caulking';

// ---------------------------------------------------------------------------- geometry (mm)
const Y = 222;                                   // the line
/** Diaphragm stations: feed-in (S0), vacuum buffer (S1), buffer/oiling (S2), transfer slide at RET (S3). */
const XS = [-145, 55, 205, 355];
const X_ST = 475, X_M2 = 675;                    // station, M&B vacuum area
const ROT = [875, 285];                          // M&B turntable axis (CAD shaft MT1250374-0061)
const AX = [X_ST, 351];                          // rotary transfer axis
const Y_ST1 = 485, Y_TRAY = 93;                  // Final Caulking ST1, finish tray (the rotary gripper's ADV)
const Z_D = 977;                                 // diaphragm station floors
const Z_S = 938;                                 // transfer slide nest, ST1, finish tray (up)
const Z_M = 900;                                 // M&B floors
const T_D = 12, T_M = 27, T_U = T_D + T_M;       // diaphragm, M&B, unit: the stack is 938..977 in the CAD
const Z_CV = 796, CV_Y = 175.5;                  // the line conveyor (CAD MMX2): belt top, centre line
const EC4_STROKE = 150, EC3_STROKE = 50, SLIDE_STROKE = 120, MBS_STROKE = 200;
const GU_STROKE = 100;                           // CDQ2LC40-100
const LOAD_STROKE = ROT[1] - Y;                  // MXQ8: the LOAD IN head from the line to the turntable
const Z_IN = Z_M + T_M + 30;                     // LOAD IN magnet face, up (CAD 957)
const Z_OUT = 1003;                              // LOAD OUT magnet face, up (CAD)
/** LOAD OUT: the small stage (down at M2 and the station) and the UNIT stage (down at M2 only). */
const OUT_STROKE = Z_OUT - (Z_S + T_U), OUTU_STROKE = (Z_S + T_U) - (Z_M + T_M);
const LIFT = 120;                                // rotary transfer lift (1EC1)
const GP_SLIDE = 125;                            // rotary gripper slides (MXQ16-125): RET radius 133 -> ADV 258
const TRAY_LIFT = 100;                           // MGPM16-100
const HINGE = [X_ST, 57, 824];                   // tray tilt hinge (HCDG10-40), tray DOWN
const TILT = 28;                                 // CDQ2B12-20 on a 41 mm lever
const DG = { span: 90, fingerLen: 30, fingerW: 8 };   // diaphragm grippers
const RG = { span: 124, fingerLen: 40, fingerW: 10 }; // rotary grippers
/** A place for what the program drives but the CAD does not have: out of the machine's way. */
const OFFSTAGE = [-900, -700, 60];

// ---------------------------------------------------------------------------- IO (the program's addresses)
/** tag -> address, filled as components are added. */
const MAP = /** @type {Record<string, string>} */ ({});
/** @param {Record<string, [string, string]>} io key -> [tag, address] @returns {Record<string, string>} */
const bind = io => Object.fromEntries(Object.entries(io).map(([k, [tag, addr]]) => { if (addr) MAP[tag] = addr; return [k, tag]; }));

/**
 * Memory the program reads but never writes, as the CPU on the line holds it. H415.07 'FAULT MOTION'
 * is ANDed into ALL FAULT(OFF) and nothing sets it: on the machine it is ON (retained), and without
 * it the cell never reaches START CONDITION 1-14.
 */
const INIT = { 'H415.07': 1 };

/**
 * Rungs the simulation runs differently from the file (server/ladderctl.js: in memory only, shown
 * as [SIM PATCH] on the ladder page, never written back by Save).
 *
 * The new program has two bench-test hooks (sim_mch, E0_3.10) that write straight into the data
 * link's RECEIVE area from the Final Caulking machine - the line's program has none:
 *   P10 INPUT R28      'LD sim_mch  AND 3303.13  OUT E0_3300.12'  (ST1 photo-eye from a spare input)
 *   P16 Condition R6   'LD sim_mch  OUT E0_3301.06'               ('Loading OK' forced on)
 * OUT writes its bit every scan, so with sim_mch OFF both CLEAR the machine's own signal, and every
 * rung after them in the scan reads zero. The link is refreshed at the end of the scan, so on the
 * machine it is the same. Measured here: P16 R31 (1EC1 UP after loading ST1) never confirms, and
 * 'Take Out / Take In Finish Work' (P16 Condition R5, R7 - after R6) are never true, so the rotary
 * keeps a finished horn in gripper 2 for ever. No alarm either way. The patches keep the hooks and
 * pass the link's own value through when they are off.
 */
const PATCHES = [{
  prog: 'P10_MAIN', sec: 'INPUT', rung: 28,
  expect: ['LD sim_mch', 'AND 3303.13', 'OUT E0_3300.12'],
  il: ['LD sim_mch', 'AND 3303.13', 'LDNOT sim_mch', 'AND E0_3300.12', 'ORLD', 'OUT E0_3300.12'],
  why: 'with sim_mch off this OUT cleared the Final Caulking ST1 detect (E0_3300.12) every scan, so P16 R31 never confirmed',
}, {
  prog: 'P16_ROTARY_TRANSFER', sec: 'Condition', rung: 6,
  expect: ['LD sim_mch', 'OUT E0_3301.06'],
  il: ['LD sim_mch', 'OR E0_3301.06', 'OUT E0_3301.06'],
  why: 'with sim_mch off this OUT cleared Loading OK (E0_3301.06) every scan, so Take Out / Take In Finish Work (R5, R7) were never true',
}];

/** Cylinder directions: the rod leaves along this world axis. */
const DIR = /** @type {Record<string, {rot: number[], v: number[]}>} */ ({
  down: { rot: [180, 0, 0], v: [0, 0, -1] }, up: { rot: [0, 0, 0], v: [0, 0, 1] },
  '+x': { rot: [0, 90, 0], v: [1, 0, 0] }, '-x': { rot: [0, -90, 0], v: [-1, 0, 0] },
  '+y': { rot: [-90, 0, 0], v: [0, 1, 0] }, '-y': { rot: [90, 0, 0], v: [0, -1, 0] },
});

// ---------------------------------------------------------------------------- the scene
/** @param {any[]} shells @param {Record<string, Array<{asset: string, color: string}>>} [products] */
export function build(shells, products = {}) {
  for (const k of Object.keys(MAP)) delete MAP[k];
  /** @type {any[]} */
  const comps = [];
  const add = (/** @type {any} */ c) => { comps.push(c); return c; };
  const r3 = (/** @type {number[]} */ v) => v.map(x => Math.round(x * 1000) / 1000);
  /**
   * A cylinder given where its ROD END is when retracted and which way the rod goes out. With a
   * parent the pose is a `want` (world, at home) that mountAll turns into a mount.
   */
  const cyl = (/** @type {string} */ id, /** @type {string} */ label, /** @type {string} */ station, /** @type {number[]} */ rodEnd, /** @type {string} */ dir,
               /** @type {any} */ params, /** @type {Record<string, [string, string]>} */ io, /** @type {string} */ [parent, socket] = ['', '']) => {
    const g = cylGeometry({ bore: params.bore, stroke: params.stroke });
    const d = DIR[dir], foot = rodEnd.map((v, i) => v - d.v[i] * (g.Lb + 27));
    const c = { id, type: 'cylinder', label, station, params: { valve: '5/2-double', cushionMm: 2, valveMs: 12, ...params }, io: bind(io) };
    if (parent) Object.assign(c, { parent, socket, want: { p: r3(foot), rot: d.rot } });
    else Object.assign(c, { at: r3(foot), rot: d.rot });
    return add(c);
  };
  /** @param {string} id @param {string} parent @param {string} socket @param {number[]} p @param {number[]} rot @param {any} rest */
  const on = (id, parent, socket, p, rot, rest) => add({ id, ...rest, parent, socket, want: { p: r3(p), rot } });
  let off = 0;
  /** Somewhere nothing else is, for an actuator the program drives and the CAD does not draw. */
  const offstage = () => { off++; return [OFFSTAGE[0], OFFSTAGE[1] - off * 150, OFFSTAGE[2]]; };

  // --------------------------------------------------------------- parts
  // The products are drawn from the CAD when it is there (products, from --cad): the M&B's
  // connector is what the turntable orients, so a plain disc would hide the point of the station.
  const pm = (/** @type {string} */ k, /** @type {string} [main] */ main, /** @type {number[]} [rot] */ rot) =>
    (products[k] || []).map((x, i) => ({ asset: x.asset, color: i === 0 && main ? main : x.color, ...(rot ? { rot } : {}) }));
  // The CAD draws its unit dummy with the connector towards -Y and its M&B dummy with it towards +Y
  // (measured on the STLs: the far points of prod_mb sit at +90 deg, of prod_unit at -90). The unit
  // is MADE from the M&B where it lands on the diaphragm, at the same yaw, so drawn as the CAD has
  // it the connector jumped half a turn the moment the loader set the M&B down (Denny saw it).
  const UNIT_ROT = [0, 0, 180];
  add({ id: 'diaph', type: 'workpiece', label: 'DIAPHRAGM TEMPLATE', at: [0, -2000, 0], params: { kind: 'cyl', size: [72, 72, T_D], color: '#c9cfd6', material: 'steel', meshes: pm('diaph', '#c9cfd6') } });
  add({ id: 'mb', type: 'workpiece', label: 'M&B TEMPLATE', at: [200, -2000, 0], params: { kind: 'cyl', size: [100, 100, T_M], color: '#3b4048', material: 'steel', meshes: pm('mb') } });
  add({ id: 'unit', type: 'workpiece', label: 'UNIT (NOT CAULKED) TEMPLATE', at: [400, -2000, 0], params: { kind: 'cyl', size: [100, 100, T_U], color: '#7d8894', material: 'steel', meshes: pm('unit', undefined, UNIT_ROT) } });
  add({ id: 'unitOk', type: 'workpiece', label: 'HORN OK TEMPLATE', at: [600, -2000, 0], params: { kind: 'cyl', size: [100, 100, T_U], color: '#5fa27a', material: 'steel', meshes: pm('unit', '#5fa27a', UNIT_ROT) } });
  add({ id: 'unitNg', type: 'workpiece', label: 'HORN NG TEMPLATE', at: [800, -2000, 0], params: { kind: 'cyl', size: [100, 100, T_U], color: '#c0504d', material: 'steel', meshes: pm('unit', '#c0504d', UNIT_ROT) } });

  // --------------------------------------------------------------- P13/P14 diaphragm: covers, stations, walking beam
  // Shutter 1 (the OUTER cover) is off the machine - a light curtain guards that opening now - but
  // it is still wired and the program still runs it (and 3205.07 is also the oil dispenser's relay).
  // So it is here as what it is electrically: a cylinder with its two reed switches, not drawn.
  cyl('shut1', 'SHUTTER 1 (wired, not installed) / OIL DISPENSER', 'DIAPH', offstage(), 'up',
    { bore: 16, stroke: 200, extendMs: 700, retractMs: 700, extWord: 'UP', retWord: 'DOWN', hidden: true },
    { solExt: ['CR_OIL_DISP_SHUT1_UP', '3205.07'], solRet: ['SOL_SHUT1_DN', '3205.08'], 'sw.ext': ['AS_SHUT1_UP', '3301.00'], 'sw.ret': ['AS_SHUT1_DN', '3301.01'] });
  // Shutter 2, the INNER cover (MGPM16-200 hanging from the cover frame, door MT6701103-0551 at
  // x -63 between the feed-in and S1). Its rod goes DOWN to close the door; retracted it is open.
  cyl('shut2', 'SHUTTER 2 INNER COVER (MGPM16-200)', 'DIAPH', [-40, 214, 1440], 'down',
    { bore: 16, stroke: 200, extendMs: 700, retractMs: 700, extWord: 'DOWN', retWord: 'UP', start: 'ext', hidden: true },
    { solExt: ['SOL_SHUT2_DN', '3205.10'], solRet: ['SOL_SHUT2_UP', '3205.09'], 'sw.ext': ['AS_SHUT2_DN', '3301.03'], 'sw.ret': ['AS_SHUT2_UP', '3301.02'] });
  // The light curtains where the outer cover was are the CAD's own GL-R bars; no beams drawn (Denny).
  XS.slice(0, 3).forEach((x, i) => {
    const tag = ['PH_DIAPH_LOADIN', 'PH_ST_DIAPH_VAC', 'PH_ST_BUFFER_DIAPH'][i], addr = ['3304.10', '3304.11', '3304.12'][i];
    add({ id: 'dS' + i, type: 'nest', label: ['DIAPH FEED IN (S0)', 'DIAPH VACUUM STATION (S1)', 'DIAPH BUFFER / OILING (S2)'][i], station: 'DIAPH',
      at: [x, Y, Z_D], params: { size: [80, 80, 16], wall: 6, hidden: true }, io: bind({ present: [tag, addr] }) });
  });
  // The oil-apply unit (ADV/RET + DOWN/UP, 3207/3307) oils the S2 pad while S2 is empty. It is in
  // the program and not in this CAD, so it is IO only until its drawing comes.
  cyl('oilAR', 'OIL APPLY ADV/RET (not in CAD)', 'DIAPH', offstage(), 'up',
    { bore: 16, stroke: 100, extendMs: 500, retractMs: 500, extWord: 'ADV', retWord: 'RET', hidden: true },
    { solExt: ['SOL_OIL_ADV', '3207.01'], solRet: ['SOL_OIL_RET', '3207.00'], 'sw.ext': ['RS_OIL_ADV', '3307.02'], 'sw.ret': ['RS_OIL_RET', '3307.03'] });
  cyl('oilUD', 'OIL APPLY DOWN/UP (not in CAD)', 'DIAPH', offstage(), 'up',
    { bore: 16, stroke: 30, extendMs: 250, retractMs: 250, extWord: 'DOWN', retWord: 'UP', hidden: true },
    { solExt: ['SOL_OIL_DN', '3207.02'], solRet: ['SOL_OIL_UP', '3207.03'], 'sw.ext': ['RS_OIL_DN', '3307.00'], 'sw.ret': ['RS_OIL_UP', '3307.01'] });

  // 1EC4 (EC-S8HR-200, low): carries the whole gripper unit one pitch. At rest it is FORWARD.
  // The inputs after 4.07 moved up one when 1EC3's MIDDLE went: the program's INPUT section reads
  // 1EC3 ALARM from 4.08 and 1EC4 FWD / BWD / ALARM from 4.09 / 4.10 / 4.11 (its symbol table
  // still has the old labels - the rungs are what the machine does).
  add({ id: 'ec4', type: 'elecylinder', label: '1EC4 LOADER UNIT (EC-S8HR)', station: 'DIAPH', at: [78, 63.5, 568],
    params: { kind: 'linear', stroke: EC4_STROKE, vmax: 400, acc: 3000, body: [470, 68, 90], start: 'fwd', hidden: true },
    io: bind({ fwd: ['EC4_FWD', '3.12'], bwd: ['EC4_BWD', '3.13'], fwdEnd: ['EC4_FWD_END', '4.09'], bwdEnd: ['EC4_BWD_END', '4.10'], alarm: ['EC4_ALM', '4.11'] }) });
  // Gripper unit up/down (CDQ2LC40-100 on the EC4 bracket): its rod goes UP and lifts the beam.
  const zDn = Z_D + T_D / 2 + DG.fingerLen * 0.6;                     // gripper origin with the unit DOWN
  cyl('gu', 'DIAPH GRIPPER UNIT (CDQ2LC40-100)', 'DIAPH', [51.5, -20, 1023], 'up',
    { bore: 40, stroke: GU_STROKE, extendMs: 400, retractMs: 400, extWord: 'UP', retWord: 'DOWN', start: 'ext', hidden: true },
    { solExt: ['SOL_GU_UP', '3215.01'], solRet: ['SOL_GU_DN', '3215.00'], 'sw.ext': ['AS_GU_UP', '3301.05'], 'sw.ret': ['AS_GU_DN', '3301.04'] }, ['ec4', 'slider']);
  const grip = (/** @type {string} */ id, /** @type {string} */ label, /** @type {string} */ parent, /** @type {string} */ socket, /** @type {number[]} */ p, /** @type {any} */ g, /** @type {Record<string, [string, string]>} */ io, /** @type {string} */ station) =>
    on(id, parent, socket, p, [180, 0, 0], { type: 'gripper', label, station,
      params: { ...g, closeMs: 180, openMs: 180, band: 1, valve: 'double', confirm: 'part', hidden: true }, io: bind(io) });
  // 1EC3 (EC-TC5M, on the beam): takes gripper 1 out to the feed-in and back. The program runs
  // it FORWARD to pick and BACKWARD to place (GSB002 "1EC3 MID ELIMINATED"), so gripper 1 goes
  // S0 -> S1 while the beam goes one pitch.
  add({ id: 'ec3', type: 'elecylinder', label: '1EC3 GRIPPER 1 (EC-TC5M)', station: 'DIAPH', parent: 'gu', socket: 'rodEnd',
    params: { kind: 'linear', stroke: EC3_STROKE, vmax: 250, acc: 2500, body: [120, 40, 30], hidden: true },
    io: bind({ fwd: ['EC3_FWD', '3.08'], bwd: ['EC3_BWD', '3.09'], fwdEnd: ['EC3_FWD_END', '4.06'], bwdEnd: ['EC3_BWD_END', '4.07'], alarm: ['EC3_ALM', '4.08'] }),
    want: { p: r3([XS[0] + EC3_STROKE - 30, Y, zDn + 46]), rot: [0, 0, 180] } });
  // Gripper 1 (MHF2) closes along Y, grippers 2 and 3 (MHZ2, MRHQ) along X - as the CAD's jaws do.
  grip('g1', 'DIAPH GRIPPER 1 (MHF2)', 'ec3', 'slider', [XS[0] + EC3_STROKE, Y, zDn], DG,
    { close: ['SOL_G1_CHK', '3215.02'], unclose: ['SOL_G1_UCHK', '3215.03'], closed: ['AS_G1_CHK', '3301.06'], open: ['AS_G1_UCHK', '3301.07'] }, 'DIAPH');
  grip('g2', 'DIAPH GRIPPER 2 (MHZ2)', 'gu', 'rodEnd', [XS[1], Y, zDn], DG,
    { close: ['SOL_G2_CHK', '3215.04'], unclose: ['SOL_G2_UCHK', '3215.05'], closed: ['AS_G2_CHK', '3301.08'], open: ['AS_G2_UCHK', '3301.09'] }, 'DIAPH');
  comps[comps.length - 1].want.rot = [180, 0, 90];                   // MHZ2 jaws close along X (the CAD's)
  // Gripper 3 is a rotary gripper (MRHQ25): its body lies along Y on the beam and the head turns
  // about Y - through the jaws' centre (x 205) and z 969.5 with the unit down, 19 mm under the
  // diaphragm's centre, which is what lands the turned-over part on the slide 38 mm lower.
  add({ id: 'flip', type: 'elecylinder', label: 'FLIP DIAPH 0/180 (MRHQ25)', station: 'DIAPH', parent: 'gu', socket: 'rodEnd',
    params: { kind: 'rotary', stroke: 180, vmax: 540, acc: 5000, band: 1, body: [36, 36, 40], hidden: true },
    io: bind({ fwd: ['SOL_FLIP_180', '3215.06'], bwd: ['SOL_FLIP_0', '3215.07'], fwdEnd: ['AS_FLIP_180', '3301.12'], bwdEnd: ['AS_FLIP_0', '3301.13'] }),
    want: { p: [XS[2], 107 - 40, 969.5], rot: [-90, 0, 0] } });
  grip('g3', 'DIAPH GRIPPER 3 (MRHQ25)', 'flip', 'slider', [XS[2], Y, zDn], DG,
    { close: ['SOL_G3_CHK', '3215.08'], unclose: ['SOL_G3_UCHK', '3215.09'], closed: ['AS_G3_CHK', '3301.10'], open: ['AS_G3_UCHK', '3301.11'] }, 'DIAPH');
  comps[comps.length - 1].want.rot = [180, 0, 90];                   // MRHQ jaws close along X

  // --------------------------------------------------------------- P15 transfer slide (CY3R20-150)
  cyl('tslide', 'TRANSFER SLIDE (CY3R20)', 'SLIDE', [XS[3] - 55, 300, 850], '+x',
    { bore: 20, stroke: SLIDE_STROKE, extendMs: 700, retractMs: 700, cushionMm: 8, extWord: 'ADV', retWord: 'RET', hidden: true },
    { solExt: ['SOL_TSLIDE_ADV', '3213.02'], solRet: ['SOL_TSLIDE_RET', '3213.03'], 'sw.ext': ['AS_TSLIDE_ADV', '3303.10'], 'sw.ret': ['AS_TSLIDE_RET', '3303.11'] });
  on('tsNest', 'tslide', 'rodEnd', [XS[3], Y, Z_S], [0, 0, 0], { type: 'nest', label: 'TRANSFER SLIDE NEST', station: 'SLIDE', params: { size: [104, 104, 30], wall: 6, hidden: true } });
  // The M&B lands on the diaphragm here and the two become one unit (the rotary gripper carries
  // them together; the CAD's parts nest into each other, rigid bodies stack).
  add({ id: 'jStack', type: 'joiner', label: 'STACK DIAPH + M&B', station: 'SLIDE', at: [X_ST, Y, Z_S], params: { size: [110, 110, 70], template: 'unit', mode: 'auto', n: 2, settleMs: 120 } });
  const eye = (/** @type {string} */ id, /** @type {string} */ label, /** @type {string} */ st, /** @type {number[]} */ p, /** @type {number[]} */ rot, /** @type {number} */ range, /** @type {[string, string]} */ io, offDelayMs = 30) =>
    add({ id, type: 'photoEye', label, station: st, at: r3(p), rot, params: { range, offDelayMs }, io: bind({ out: io }) });
  eye('phSide', 'PH TRANSFER STATION DIAPH SIDE', 'SLIDE', [XS[3], Y - 75, Z_S + 6], [0, 0, 90], 150, ['PH_TS_DIAPH_SIDE', '3302.10']);
  // The two beams at the station, as the program reads them (AL46 is 'slide has work, at ADV, and
  // 3302.12 off'; the M&B head drops only with 3302.12 on and 3302.11 off): 3302.12 is the LOW beam
  // that sees any work on the slide, 3302.11 the HIGH one that sees an M&B on top of it.
  eye('phStD', 'PH TRANSFER STATION WORK (LOW)', 'SLIDE', [X_ST, Y - 75, Z_S + 6], [0, 0, 90], 150, ['PH_TS_WORK_LOW', '3302.12']);
  eye('phStM', 'PH TRANSFER STATION DIAPH+M&B (HIGH)', 'SLIDE', [X_ST, Y - 75, Z_S + T_U - 6], [0, 0, 90], 150, ['PH_TS_WORK_HIGH', '3302.11']);

  // --------------------------------------------------------------- P11/P12 M&B: turntable, vacuum area, two magnet heads
  add({ id: 'rot', type: 'turntable', label: 'M&B ROTATE UNIT (1MTD1)', station: 'MB', at: [ROT[0], ROT[1], Z_M - 60],
    params: { size: [100, 100, 40], height: 60, speed: 120, accMs: 80, feature: 0, preDeg: 25, band: 2, hidden: true, cw: true },   // FWD turns clockwise (Denny)
    io: bind({ fwd: ['MTD1_FWD', '3205.00'], rev: ['MTD1_REV', '3205.01'], lock: ['ROT_PIN_LOCK', ''], pre: ['PH_ROT_PRE_END', '3304.08'], inPos: ['PH_TERM_IN_POS', '3304.09'], present: ['PH_WORK_IN_ROT', '3302.15'] }) });
  // The positioning pin (MGJ10-20 lifting a PSFTCA10 pin at the M&B's rim): up it stops the terminal.
  cyl('pin', 'PIN POSITIONING (MGJ10-20)', 'MB', [884, 333, Z_M], 'up',
    { bore: 10, stroke: 20, extendMs: 150, retractMs: 150, extWord: 'UP', retWord: 'DOWN', reedBand: 1, reedHyst: 0.3, switches: [{ id: 'ret', pos: 0.5 }, { id: 'ext', pos: 19.5 }], hidden: true },
    { solExt: ['SOL_PIN_UP', '3211.08'], solRet: ['SOL_PIN_DN', '3211.09'], 'sw.ext': ['AS_PIN_UP', '3303.08'], 'sw.ret': ['AS_PIN_DN', '3303.09'] });
  add({ id: 'm2', type: 'nest', label: 'M&B VACUUM AREA (M2)', station: 'MB', at: [X_M2, Y, Z_M], params: { size: [106, 106, 16], wall: 6, hidden: true }, io: bind({ present: ['PH_MB_VAC', '3302.08'] }) });
  // M&B load unit slide (CY3RG20-250): both heads go one pitch to -X.
  cyl('mbs', 'M&B LOAD UNIT SLIDE (CY3RG20)', 'MB', [844, -8, 700], '-x',
    { bore: 20, stroke: MBS_STROKE, extendMs: 900, retractMs: 900, cushionMm: 10, extWord: 'ADV', retWord: 'RET', hidden: true },
    { solExt: ['SOL_MBS_ADV', '3211.00'], solRet: ['SOL_MBS_RET', '3211.01'], 'sw.ext': ['AS_MBS_ADV', '3303.06'], 'sw.ret': ['AS_MBS_RET', '3303.07'] });
  // LOAD IN: an MXQ16L-50 lowers the head, and on its table the MXQ8-75 takes the magnet between the
  // turntable (FWD, at home) and the line (BWD, for the slide's pitch to M2).
  cyl('mbIn', 'M&B LOAD IN (MXQ16L-50)', 'MB', [887, 92, 982], 'down',
    { bore: 16, stroke: Z_IN - (Z_M + T_M), extendMs: 300, retractMs: 300, extWord: 'DOWN', retWord: 'UP', hidden: true },
    { solExt: ['SOL_MBIN_DN', '3211.02'], solRet: ['SOL_MBIN_UP', '3211.03'], 'sw.ext': ['AS_MBIN_DN', '3303.00'], 'sw.ret': ['AS_MBIN_UP', '3303.01'] }, ['mbs', 'rodEnd']);
  cyl('mbLoad', 'M&B LOAD FWD/BWD (MXQ8-75)', 'MB', [875, 187, 1003], '+y',
    { bore: 10, stroke: LOAD_STROKE, extendMs: 350, retractMs: 350, cushionMm: 4, extWord: 'FWD', retWord: 'BWD', start: 'ext', hidden: true },
    { solExt: ['SOL_LOAD_FWD', '3211.15'], solRet: ['SOL_LOAD_BWD', '3211.14'], 'sw.ext': ['AS_LOAD_FWD', '3303.15'], 'sw.ret': ['AS_LOAD_BWD', '3303.14'] }, ['mbIn', 'rodEnd']);
  on('mag1', 'mbLoad', 'rodEnd', [ROT[0], Y, Z_IN], [180, 0, 0], { type: 'vacuumCup', label: 'MAGNET 1 (KE-4E)', station: 'MB', params: { d: 60, reach: 4, buildMs: 60, dropMs: 60, hidden: true },
    io: bind({ on: ['MAG1_ON', ''], vac: ['MAG1_HOLDS', ''] }) });
  // PH 3302.13/.14 'M&B LOADER WORK CONF' are photo-eyes ON the heads (PZ-V11): they see the M&B as
  // soon as the head is down on it, before the magnet is on. The program relies on it - FAULT R37
  // clears 'loader 2 has work' when the eye has been dark 1.5 s, which a magnet-holding signal lost.
  on('phMag1', 'mbLoad', 'rodEnd', [ROT[0] - 65, Y, Z_IN - 12], [0, 0, 0], { type: 'photoEye', label: 'PH M&B LOADER WORK CONF 1', station: 'MB', params: { range: 130, offDelayMs: 30 },
    io: bind({ out: ['PH_MB_LOADER_1', '3302.13'] }) });
  // LOAD OUT: an MXQ20L-125 (the UNIT stage, down at M2 only) carries an MXQ16L-40 (down at M2 and
  // at the station, where only it goes down: the unit stage is ANDNOT GSB000 there).
  cyl('mbOutU', 'M&B LOAD OUT UNIT (MXQ20L-125)', 'MB', [797, 84, 1109], 'down',
    { bore: 20, stroke: OUTU_STROKE, extendMs: 350, retractMs: 350, extWord: 'DOWN', retWord: 'UP', hidden: true },
    { solExt: ['SOL_MBOUTU_DN', '3211.04'], solRet: ['SOL_MBOUTU_UP', '3211.05'], 'sw.ext': ['AS_MBOUTU_DN', '3303.04'], 'sw.ret': ['AS_MBOUTU_UP', '3303.05'] }, ['mbs', 'rodEnd']);
  cyl('mbOut', 'M&B LOAD OUT (MXQ16L-40)', 'MB', [748, 84, 1028], 'down',
    { bore: 16, stroke: OUT_STROKE, extendMs: 250, retractMs: 250, extWord: 'DOWN', retWord: 'UP', hidden: true },
    { solExt: ['SOL_MBOUT_DN', '3211.06'], solRet: ['SOL_MBOUT_UP', '3211.07'], 'sw.ext': ['AS_MBOUT_DN', '3303.02'], 'sw.ret': ['AS_MBOUT_UP', '3303.03'] }, ['mbOutU', 'rodEnd']);
  on('mag2', 'mbOut', 'rodEnd', [X_M2, Y, Z_OUT], [180, 0, 0], { type: 'vacuumCup', label: 'MAGNET 2 (KE-4E)', station: 'MB', params: { d: 60, reach: 4, buildMs: 60, dropMs: 60, hidden: true },
    io: bind({ on: ['MAG2_ON', ''], vac: ['MAG2_HOLDS', ''] }) });
  on('phMag2', 'mbOut', 'rodEnd', [X_M2 - 65, Y, Z_OUT - 12], [0, 0, 0], { type: 'photoEye', label: 'PH M&B LOADER WORK CONF 2', station: 'MB', params: { range: 130, offDelayMs: 30 },
    io: bind({ out: ['PH_MB_LOADER_2', '3302.14'] }) });

  // --------------------------------------------------------------- P16 rotary transfer: lift, rotary, two slides, two grippers
  const zGripDown = Z_S + T_U / 2, oGrip = RG.fingerLen * 0.6;
  const zLiftUp = zGripDown + LIFT + oGrip + 60;                    // the rotary table face (it hangs), lift UP
  // This program reads 1EC1 FORWARD COMPLETE from 4.01 and BACKWARD from 4.00, the other way round
  // from the line's program; FORWARD START (3.00) is DOWN. So the lift's down end is wired to 4.01.
  add({ id: 'ec1', type: 'elecylinder', label: '1EC1 ROTARY LIFT (EC-S13L)', station: 'ROTARY', at: [340 - 60, 540, zLiftUp + 60],
    params: { kind: 'linear', stroke: LIFT, vmax: 400, acc: 3000, body: [300, 60, 60], hidden: true }, rot: [0, 90, 0],
    io: bind({ fwd: ['EC1_DN', '3.00'], bwd: ['EC1_UP', '3.01'], fwdEnd: ['EC1_DN_END', '4.01'], bwdEnd: ['EC1_UP_END', '4.00'], alarm: ['EC1_ALM', '4.02'] }) });
  add({ id: 'ec2', type: 'elecylinder', label: '1EC2 ROTARY (EC-RTC18M)', station: 'ROTARY', parent: 'ec1', socket: 'slider',
    // FWD 3.04 -> FORWARD COMPLETE 4.03, BWD 3.05 -> 4.04, as this program's sequence pairs them (the
    // line's program read the two inputs the other way round). HOME POS.1 is BACKWARD: gripper 1
    // over the station, at 0 - where it rests.
    params: { kind: 'rotary', stroke: 180, vmax: 360, acc: 1500, band: 0.5, body: [120, 120, 60], hidden: true },
    io: bind({ fwd: ['EC2_FWD', '3.04'], bwd: ['EC2_BWD', '3.05'], fwdEnd: ['EC2_FWD_END', '4.03'], bwdEnd: ['EC2_BWD_END', '4.04'], alarm: ['EC2_ALM', '4.05'] }),
    want: { p: [AX[0], AX[1], zLiftUp + 60], rot: [180, 0, 0] } });
  const zSlide = zGripDown + LIFT + oGrip + 30;
  const gp = (/** @type {number} */ n, /** @type {number} */ yGrip, /** @type {string} */ dir, /** @type {number} */ xOff, /** @type {string[]} */ a, /** @type {number} */ radialDeg) => {
    const out = dir === '-y' ? -1 : 1, rodEndY = yGrip - out * 26;
    cyl('gp' + n + 's', 'GRIPPER TRANSFER UNIT ' + n + ' (MXQ16-125)', 'ROTARY', [AX[0] + xOff, rodEndY, zSlide], dir,
      { bore: 16, stroke: GP_SLIDE, extendMs: 450, retractMs: 450, cushionMm: 6, extWord: 'ADV', retWord: 'RET', hidden: true },
      { solExt: ['SOL_GP' + n + 'S_ADV', a[0]], solRet: ['SOL_GP' + n + 'S_RET', a[1]], 'sw.ext': ['AS_GP' + n + 'S_ADV', a[2]], 'sw.ret': ['AS_GP' + n + 'S_RET', a[3]] }, ['ec2', 'slider']);
    // MHSL3: three jaws, at these angles in the gripper frame (read off the CAD's jaws at rest).
    grip('gp' + n, 'ROTARY GRIPPER ' + n + ' (MHSL3)', 'gp' + n + 's', 'rodEnd', [AX[0], yGrip, zGripDown + LIFT + oGrip], { ...RG, fingers3: true, radialDeg },
      { close: ['SOL_GP' + n + '_CHK', a[4]], unclose: ['SOL_GP' + n + '_UCHK', a[5]], closed: ['AS_GP' + n + '_CHK', a[6]], open: ['AS_GP' + n + '_UCHK', a[7]] }, 'ROTARY');
  };
  // At rest (0, POS.1) gripper 1 is over the station and gripper 2 over ST1, as in the CAD.
  gp(1, Y - 4, '-y', -26, ['3213.04', '3213.05', '3302.04', '3302.05', '3213.08', '3213.09', '3302.00', '3302.01'], 110);
  gp(2, Y_ST1, '+y', 26, ['3213.06', '3213.07', '3302.06', '3302.07', '3213.10', '3213.11', '3302.02', '3302.03'], 49);

  // --------------------------------------------------------------- P17 discharge: finish tray lift + tilt, NG chute and box
  // Denny: the MGPM16-100 carries the tray straight up and down - up to take the finished unit,
  // down, then it TILTS: OK onto the line conveyor behind it, NG into the NG box out front. The tilt
  // is two CDQ2B12-20 stacked on the lift: OK PUSH UP lifts the front edge (the tray tips back to the
  // conveyor), NG PUSH DOWN pulls it down (the tray tips forward into the chute). Home is OK DOWN,
  // NG UP: level. Stacked cylinders ADD, so they are two rotary axes about the one hinge, in series.
  cyl('tray', 'WORK FINISH TRAY LIFT (MGPM16-100)', 'DISCH', [X_ST, 66, 792], 'up',
    { bore: 16, stroke: TRAY_LIFT, extendMs: 500, retractMs: 500, cushionMm: 6, extWord: 'UP', retWord: 'DOWN', start: 'ext', hidden: true },
    { solExt: ['SOL_TRAY_UP', '3213.01'], solRet: ['SOL_TRAY_DN', '3213.00'], 'sw.ext': ['AS_TRAY_UP', '3304.01'], 'sw.ret': ['AS_TRAY_DN', '3304.00'] });
  const tilt = (/** @type {string} */ id, /** @type {string} */ label, /** @type {string} */ parent, /** @type {string} */ socket, /** @type {number} */ sx, /** @type {Record<string, [string, string]>} */ io) =>
    add({ id, type: 'elecylinder', label, station: 'DISCH', parent, socket,
      // A thin body ON the hinge line: it still collides, and a 30 mm one stood 8 mm proud of the
      // tray floor once the tray tipped, so the NG horn stuck on it halfway (measured: 21 mm of 57).
      params: { kind: 'rotary', stroke: TILT, vmax: 240, acc: 4000, band: 1, body: [12, 12, 20], hidden: true },
      io: bind(io), want: { p: [HINGE[0] - sx * 20, HINGE[1], HINGE[2]], rot: [0, sx * 90, 0] } });
  // +TILT about -X tips the +Y edge down (OK); +TILT about +X tips the -Y edge down (NG).
  tilt('okTilt', 'OK PUSH (CDQ2B12, tilts to the conveyor)', 'tray', 'rodEnd', -1,
    { fwd: ['SOL_OK_UP', '3211.12'], bwd: ['SOL_OK_DN', '3211.13'], fwdEnd: ['AS_OK_UP', '3304.02'], bwdEnd: ['AS_OK_DN', '3304.03'] });
  tilt('ngTilt', 'NG PUSH (CDQ2B12, tilts to the NG box)', 'okTilt', 'slider', 1,
    { fwd: ['SOL_NG_DN', '3211.10'], bwd: ['SOL_NG_UP', '3211.11'], fwdEnd: ['AS_NG_DN', '3304.04'], bwdEnd: ['AS_NG_UP', '3304.05'] });
  on('trayNest', 'ngTilt', 'slider', [X_ST, Y_TRAY, Z_S - TRAY_LIFT], [0, 0, 0], { type: 'nest', label: 'FINISH TRAY', station: 'DISCH', params: { size: [104, 104, 30], wall: 5, hidden: true },
    io: bind({ clamp: ['TRAY_CLAMP', ''], present: ['PH_WORK_IN_TRAY', '3303.12'] }) });
  // The tray itself is the CAD's 150 mm sheet (y -11..139), longer than the pocket towards the NG
  // side: with only the pocket's floor, a tipped horn dropped off its edge into the gap in front of
  // the chute, hung on the lift and fell out of the world when the tray came down (measured).
  on('trayFloor', 'ngTilt', 'slider', [X_ST, 64, Z_S - TRAY_LIFT - 6], [0, 0, 0], { type: 'plate', label: 'FINISH TRAY SHEET', station: 'DISCH', params: { size: [100, 150, 6], hidden: true } });
  // NG: the horn leaves the tipped tray's front end into the chute (MT1160002-0640) and the NG box.
  // The chute is drawn by the CAD and the zone takes the horn as it drops into it: a solid chute
  // meeting a tipped tray is the positive-distance trap twice over (its top edge stands 18 mm above
  // the tray's end with this hinge), and nothing about a horn in the NG chute is still in the machine.
  add({ id: 'ngBox', type: 'remover', label: 'NG CHUTE + BOX', station: 'DISCH', at: [X_ST, -165, 560], params: { size: [170, 330, 240] }, io: bind({ count: ['NG_BOX_CNT', ''] }) });
  eye('phNg', 'PH WORK NG CONFIRM (PZ-M51)', 'DISCH', [X_ST - 100, -30, 805], [0, 0, 0], 200, ['PH_WORK_NG', '3304.13'], 1500);

  // --------------------------------------------------------------- the line conveyor and the Final Caulking machine
  // The existing line's conveyor (CAD "##Current MC", MMX2): the OK horn drops onto it off the tray.
  const cvLen = 2500, cvX = 545 - cvLen / 2;
  add({ id: 'cv', type: 'conveyor', label: 'LINE CONVEYOR (MMX2)', station: 'FINAL', at: [cvX, CV_Y, 0], rot: [0, 0, 180],
    params: { length: cvLen, width: 150, height: Z_CV, speed: 200, guides: 0, centering: 150, hidden: true }, io: { run: 'FC_CV_RUN' } });
  eye('phCv', 'PH CONVEYOR (FINAL CAULKING)', 'FINAL', [X_ST - 20, CV_Y - 90, Z_CV + 20], [0, 0, 90], 180, ['FC_CV_PH', ''], 300);
  add({ id: 'cvEnd', type: 'remover', label: 'TO NEXT PROCESS', station: 'FINAL', at: [cvX - cvLen / 2 - 60, CV_Y, Z_CV - 100], params: { size: [160, 260, 300] }, io: bind({ count: ['OUT_CNT', ''] }) });
  // The existing machine, as a simple model: its two-station table and its press.
  add({ id: 'fcTable', type: 'indexTable', label: 'FINAL CAULKING TABLE (model)', station: 'FINAL', at: [X_ST, Y_ST1 + 200, Z_S - 140], rot: [0, 0, -90],
    params: { stations: 2, camMs: 1600, indexFrac: 0.6, diameter: 526, height: 120 }, io: { run: 'FC_INDEX', inPos: 'FC_TBL_INPOS' } });
  // Every nest a 39 mm unit sits in is 30 deep: a nest takes a part whose CENTRE is inside the
  // pocket (CLAUDE.md), and with 18 the unit was never taken, so the table flung it off when it turned.
  for (const i of [0, 1]) add({ id: 'fcN' + i, type: 'nest', label: 'FINAL CAULKING JIG ' + (i + 1), station: 'FINAL', parent: 'fcTable', socket: 's' + i, params: { size: [106, 106, 30], wall: 6, hidden: true } });
  eye('phSt1', 'PH FINAL CAULKING ST1', 'FINAL', [X_ST - 75, Y_ST1, Z_S + 10], [0, 0, 0], 150, ['FC_ST1_PH', '']);
  eye('phSt2', 'PH FINAL CAULKING ST2', 'FINAL', [X_ST - 75, Y_ST1 + 400, Z_S + 10], [0, 0, 0], 150, ['FC_ST2_PH', '']);
  cyl('fcPress', 'CAULKING PRESS (model)', 'FINAL', [X_ST, Y_ST1 + 400, Z_S + T_U + 3 + 10 + 60], 'down',
    { bore: 40, stroke: 60, extendMs: 400, retractMs: 400, extWord: 'DOWN', retWord: 'UP', head: 'plate', headSize: [80, 80, 10] },
    { solExt: ['FC_PRESS_DN', ''], solRet: ['FC_PRESS_UP', ''], 'sw.ext': ['FC_PRESS_DN_END', ''], 'sw.ret': ['FC_PRESS_UP_END', ''] });
  add({ id: 'jCaulk', type: 'joiner', label: 'CAULKING (OK / NG)', station: 'FINAL', at: [X_ST, Y_ST1 + 400, Z_S], params: { size: [110, 110, 70], template: 'unitOk', templateAlt: 'unitNg', mode: 'tag', n: 1 },
    io: { join: 'FC_CAULK', alt: 'FC_NG', count: 'FC_CAULK_CNT' } });

  // --------------------------------------------------------------- the operator: panel and hands
  const pb = (/** @type {string} */ id, /** @type {string} */ label, /** @type {any} */ params, /** @type {Record<string, [string, string]>} */ io) => add({ id, type: 'pushbutton', label, params, io: bind(io) });
  pb('pbEstop', 'EMERGENCY STOP', { kind: 'alternate', color: 'red', lamp: false }, { pb: ['PB_ESTOP', ''] });
  add({ id: 'sel', type: 'selector', label: 'AUTO / INDIVIDUAL', io: bind({ sel: ['SS_AUTO', '0.02'] }) });
  pb('pbMaster', 'MASTER ON', { kind: 'momentary', color: 'white', lamp: true }, { pb: ['PB_MASTER', '0.03'], lamp: ['PL_MASTER', '1.02'] });
  pb('pbAuto', 'AUTO RUN', { kind: 'momentary', color: 'green', lamp: true }, { pb: ['PB_AUTO', '0.04'], lamp: ['PL_AUTO', '1.01'] });
  pb('pbCstop', 'CYCLE STOP', { kind: 'momentary', color: 'yellow', lamp: false }, { pb: ['PB_CSTOP', '0.05'] });
  pb('pbDummy', 'DUMMY DOOR OPEN REQ', { kind: 'momentary', color: 'blue', lamp: true }, { pb: ['PB_DUMMY', '0.06'], lamp: ['PL_DUMMY_EN', '1.04'] });
  add({ id: 'lampR', type: 'lamp', label: 'SIGNAL TOWER RED', params: { color: 'red' }, io: bind({ lamp: ['PL_TOWER_R', '1.13'] }) });
  add({ id: 'lampY', type: 'lamp', label: 'SIGNAL TOWER YELLOW', params: { color: 'yellow' }, io: bind({ lamp: ['PL_TOWER_Y', '1.14'] }) });
  add({ id: 'lampG', type: 'lamp', label: 'SIGNAL TOWER GREEN', params: { color: 'green' }, io: bind({ lamp: ['PL_TOWER_G', '1.15'] }) });
  add({ id: 'lampBz', type: 'lamp', label: 'BUZZER', params: { color: 'amber', buzzer: true }, io: bind({ lamp: ['BZ', '1.06'] }) });
  add({ id: 'lampFeed', type: 'lamp', label: 'OK TO FEED DIAPHRAGM', params: { color: 'white' }, io: bind({ lamp: ['PL_FEED_DIAPH', '1.03'] }) });
  add({ id: 'lampLoad', type: 'lamp', label: 'LOAD WORK ENABLE', params: { color: 'white' }, io: bind({ lamp: ['PL_LOAD_EN', '3205.15'] }) });
  add({ id: 'lampMute', type: 'lamp', label: 'LIGHT CURTAIN MUTED (1.10)', params: { color: 'white' }, io: bind({ lamp: ['CR_SAFETY_MUTE', '1.10'] }) });
  // The touch panel's buttons the cell cannot run without (the program reads them as W bits).
  pb('hmiReset', 'HMI: FAULT RESET', { kind: 'momentary', color: 'blue', lamp: false }, { pb: ['HMI_FAULT_RESET', 'W450.08'] });
  pb('hmiBz', 'HMI: BUZZER RESET', { kind: 'momentary', color: 'blue', lamp: false }, { pb: ['HMI_BZ_RESET', 'W450.09'] });
  pb('hmiHome', 'HMI: ALL UNIT HOME', { kind: 'momentary', color: 'blue', lamp: false }, { pb: ['HMI_ALL_HOME', 'W478.15'] });
  // The operator's hands (not PLC IO): a part put down, the nagara switch that says so, a hand in
  // the light curtain; or keep both fed; and the Final Caulking result for the next unit.
  pb('pbFeedD', 'PUT DIAPHRAGM (S0)', { kind: 'momentary', color: 'blue', lamp: false }, { pb: ['OP_PUT_DIAPH', ''] });
  pb('pbNagaraD', 'NAGARA START: DIAPHRAGM', { kind: 'momentary', color: 'green', lamp: false }, { pb: ['OP_NAGARA_D', ''] });
  pb('pbFeedM', 'PUT M&B (TURNTABLE)', { kind: 'momentary', color: 'blue', lamp: false }, { pb: ['OP_PUT_MB', ''] });
  pb('pbNagaraM', 'NAGARA START: M&B', { kind: 'momentary', color: 'green', lamp: false }, { pb: ['OP_NAGARA_M', ''] });
  pb('pbAutoFeed', 'OPERATOR KEEPS FEEDING', { kind: 'alternate', color: 'white', lamp: false }, { pb: ['OP_AUTO_FEED', ''] });
  pb('pbLc', 'HAND IN LIGHT CURTAIN', { kind: 'alternate', color: 'yellow', lamp: false }, { pb: ['OP_LC_BREAK', ''] });
  // sim_mch (E0_3.10) is ON by default (Denny); in it the program judges every horn NG. OFF is the
  // line: the Final Caulking model's OK / NG results are read.
  pb('pbSimOff', 'SIM_MCH OFF (line mode)', { kind: 'alternate', color: 'white', lamp: false }, { pb: ['SIM_MCH_OFF', ''] });
  pb('pbNg', 'FINAL CAULKING: NEXT = NG', { kind: 'alternate', color: 'red', lamp: false }, { pb: ['FC_FORCE_NG', ''] });
  add({ id: 'emD', type: 'emitter', label: 'OPERATOR: DIAPHRAGM', station: 'DIAPH', at: [XS[0], Y, Z_D + 8], params: { template: 'diaph', mode: 'tag', dropOnto: true }, io: { emit: 'EM_DIAPH', count: 'EM_DIAPH_CNT' } });
  add({ id: 'emM', type: 'emitter', label: 'OPERATOR: M&B', station: 'MB', at: [ROT[0], ROT[1], Z_M + 6], params: { template: 'mb', mode: 'tag', jitterDeg: 180, dropOnto: true }, io: { emit: 'EM_MB', count: 'EM_MB_CNT' } });

  // --------------------------------------------------------------- the maker's CAD
  for (const s of shells) {
    const c = add({ id: s.id, type: 'shell', label: s.label, params: { asset: s.asset, scale: 1, color: s.color, boxes: s.boxes } });
    // A moving shell's STL is written in its socket's frame (--cad), so it mounts with no offset.
    if (s.parent) Object.assign(c, { parent: s.parent, socket: s.socket });
  }

  const st = (/** @type {string} */ id, /** @type {string} */ name) => ({ id, name, members: comps.filter(c => c.station === id).map(c => c.id) });
  const scene = {
    format: 'mio-scene/1', name: NAME,
    sim: { dtMs: 4 },
    // One cycle per horn caulked (OK or NG): the machine's output rate.
    cycle: { countTag: 'CYCLE_CNT', avgN: 10 },
    io: { driver: 'ladder', mode: 'sim', minPulseMs: 20, ladder: { cxp: 'plc/final-caulking-mdf.cxp', hmi: 'plc/final-caulking-mdf.vs4', map: MAP, init: INIT, patches: PATCHES } },
    stations: [st('DIAPH', 'P13/P14 Diaphragm shutter + loader'), st('MB', 'P11/P12 M&B loader + positioning'), st('SLIDE', 'P15 Transfer slide'),
      st('ROTARY', 'P16 Rotary transfer'), st('DISCH', 'P17 OK/NG discharge'), st('FINAL', 'Final Caulking MC (model)')],
    components: comps,
  };
  mountAll(scene);
  for (const c of scene.components) if (!c.station) delete c.station;
  return scene;
}

/**
 * Components that ride something are given the WORLD pose they have with the machine at home
 * (`want`: every cylinder retracted, every axis at 0), and their mount on the parent socket is
 * solved here - so the numbers above stay world numbers, as read off the CAD.
 * @param {any} scene
 */
function mountAll(scene) {
  for (const c of scene.components) {
    if (!c.want) continue;
    const w = c.want;
    delete c.want;
    c.at = [0, 0, 0]; c.rot = [0, 0, 0];
    const tmp = { ...scene, components: scene.components.filter((/** @type {any} */ x) => !x.want).map((/** @type {any} */ x) => ({ ...x })) };
    // At ZERO dofs, not at each component's power-up position: a tray that starts up or a beam
    // that starts forward would otherwise put everything riding it one stroke out.
    const zero = Object.fromEntries(tmp.components.map((/** @type {any} */ x) => [x.id, 0]));
    const W = worldPoses(tmp, zero), { defs } = compile(tmp), pd = defs.get(c.parent), s = pd.sockets[c.socket];
    if (!s) throw new Error(c.id + ': ' + c.parent + ' has no socket ' + c.socket);
    const base = compose(W[c.parent][s.link], pose(s.at, s.rot));
    const m = mountFrom(base, pose(w.p, w.rot));
    c.at = m.at.map((/** @type {number} */ v) => Math.round(v * 1000) / 1000); c.rot = m.rot.map((/** @type {number} */ v) => Math.round(v * 1000) / 1000);
  }
}

// ---------------------------------------------------------------------------- CAD shells
/**
 * What of the iCAD model is drawn, and what it rides. Denny's rules: a motion simulation, so
 * fasteners, fittings and the pneumatic panel go, and the covers that block the view go; the
 * products (the CAD's dummy horns) are PARTS, never machine; and everything that moves is the CAD's
 * own part riding the component that moves it. The simulator's primitives of those components
 * still collide and still carry the switches; they are just not drawn.
 *
 * `classify` says, for one CAD body, 'drop', 'static', or the component (and socket) it rides.
 * BODY_RIDES comes first: the moving TABLE of a slide whose body stays put, named body by body
 * from the assembly (an MXQ16-125 is eleven bodies, two of them the long table and its end plate).
 * Part numbers are this machine's (MT1140723/-0724/1160002/1250374).
 */
const DROP = /^(CB|CBSST|MS|KQ2|AS1|AS2|AN|ACB|BTR|HC|LB|SFB|STB|STE|STS|STM|WSSB|KNTR|TA-|STNS|NJSB|C-|FJ|LHF|HB|HF|B7001|JLNK|PSF|NKJ|OP-|ETKGR|BSJF|PETR|B17i|dummy|G21_housing|\*WORK|\*\*|\*EL|\[FENB|003-|02$|MT1160002-0510)/;
/** Kept even though DROP matches a name on their chain: they move, or they are what is moved. */
const KEEP = /^(PSFTCA|LHFC)/;
/** @type {Record<number, [string, string]>} */
const BODY_RIDES = {
  // MXQ16-125 on the rotary head: slide 1 (towards the station in the CAD) and slide 2. Table + end plate.
  229: ['gp1s', 'rodEnd'], 901: ['gp1s', 'rodEnd'], 1819: ['gp1s', 'rodEnd'], 3035: ['gp1s', 'rodEnd'],
  224: ['gp2s', 'rodEnd'], 889: ['gp2s', 'rodEnd'], 1800: ['gp2s', 'rodEnd'], 2932: ['gp2s', 'rodEnd'],
  507: ['ec2', 'slider'], 508: ['ec2', 'slider'],                  // EC-RTC18M output flange
  220: ['ec1', 'slider'],                                             // EC-S13L slider
  129: ['ec4', 'slider'],                                             // EC-S8HR slider
  192: ['ec3', 'slider'],                                             // EC-TC5M table (gripper 1 hangs from it)
  380: ['gu', 'rodEnd'], 648: ['gu', 'rodEnd'], 1270: ['gu', 'rodEnd'],   // CDQ2LC40 rod, floating joint
  393: ['flip', 'slider'], 1328: ['g3', 'jaw'], 1341: ['g3', 'jaw'],   // MRHQ25 head; its fingers chuck
  633: ['g1', 'jaw'], 635: ['g1', 'jaw'], 1238: ['g2', 'jaw'], 1320: ['g2', 'jaw'],   // MHF2 / MHZ2 fingers
  664: ['tslide', 'rodEnd'],                                          // CY3R20 carriage
  768: ['mbs', 'rodEnd'],                                             // CY3RG20 carriage
  473: ['mbIn', 'rodEnd'], 474: ['mbLoad', 'rodEnd'],                // MXQ16L-50 table, MXQ8 table
  190: ['mbOutU', 'rodEnd'], 467: ['mbOut', 'rodEnd'],               // MXQ20L-125 table, MXQ16L-40 table
  409: ['tray', 'rodEnd'],                                            // MGPM16-100 plate and rods
  210: ['shut2', 'rodEnd'], 212: ['shut2', 'rodEnd'],                // MGPM16-200 plate and rods
  411: ['ngTilt', 'slider'], 673: ['ngTilt', 'slider'],              // the finish tray and its hinge ear
  742: ['pin', 'rodEnd'],                                             // PSFTCA10 positioning pin
};
/** The CAD's product models: a diaphragm at S0, an M&B on the turntable, a unit at ST1 (tree node ids). */
const PRODUCT_NODES = { diaph: 26, mb: 1975, unit: 1971 };
/** The finish tray is drawn in the layout's UP position; everything else about the lift is down. */
const PRE_SHIFT = /** @type {Record<number, number[]>} */ ({ 411: [0, 0, -TRAY_LIFT], 673: [0, 0, -TRAY_LIFT] });
/** The dofs the CAD is drawn at: the diaphragm beam back and down with gripper 1 out at the feed-in,
 * shutter 2 closed, the transfer slide at the station, the M&B heads up at RET with LOAD BWD, the
 * rotary at rest (gripper 1 over the station) and up, the tray lift down and level. */
const CAD_DOF = { ec4: 0, ec3: 50, gu: 0, flip: 0, tslide: 120, mbs: 0, mbIn: 0, mbLoad: 0, mbOutU: 0, mbOut: 0, ec1: 0, ec2: 0,
  gp1s: 0, gp2s: 0, tray: 0, g1: 36, g2: 36, g3: 36, gp1: 50, gp2: 50, okTilt: 0, ngTilt: 0, shut2: 200, pin: 0, rot: 0 };
/**
 * @param {string} unit @param {string} top the unit's direct child (the part) @param {string[]} names the whole chain
 * @param {number[]} c the body's centre @param {number[]} box @returns {'drop'|'static'|[string, string]}
 */
function classify(unit, top, names, c, box) {
  const [x, y, z] = c;
  if (/Current MC/.test(unit)) return /^MMX2/.test(names[0]) || names.some(n => /^MMX2/.test(n)) ? 'static' : 'drop';   // the line conveyor
  // The Final Caulking machine is the existing machine next door, and the CAD has it as grey blocks
  // the size of a cabinet (its column alone is 600 x 650 x 900) plus its electrical cart a metre
  // behind. Denny: the big grey blocks go. Its table is the simple model below, drawn.
  if (unit === 'final_caulking_mc') return 'drop';
  if (/SAFETY_COVER/.test(unit)) {
    if (names.some(n => /^light$/i.test(n))) return 'drop';          // a drawn sensor BEAM, not a part
    if (/^(GL-R|LR-W500|MGPM16-200)/.test(top)) return 'static';     // light curtains, the S0 sensor, shutter 2's body
    if (/^MT6701103-0(551|610|710)/.test(top)) return ['shut2', 'rodEnd'];
    return 'drop';                                                    // covers, doors and what hangs on them
  }
  if (/STRUCTURE/.test(unit)) return /^(FRL|SS5Y5|VQ|ARM5SB|DIN RAIL|AN)/.test(top) ? 'drop' : 'static';   // the valve panel went
  if (/TRANSFER_DIAPHRAGM/.test(unit)) {
    // The station jigs sit in the layout group with the gripper unit's guide blocks: by NAME, or they
    // ride the beam (measured: the S0/S1 jig plates went up and down with the grippers).
    if (names.some(n => /^\*(Jig|XFHTD)/.test(n))) return 'static';
    if (/^SHS20V/.test(top)) {                                       // guide rails: the rail is fixed to what carries it, the block rides
      const block = names.some(n => /^\[SHS/.test(n));
      return /460L/.test(top) ? (block ? ['ec4', 'slider'] : 'static') : (block ? ['gu', 'rodEnd'] : ['ec4', 'slider']);
    }
    if (/^(EC-S8HR|MT1140724-0(270|280|360|370|380|340|351|220|261)|FU-|TKP45|RCB)/.test(top)) return 'static';
    if (/^MT1140724-0(420|430)/.test(top)) return ['g3', 'jaw'];      // the flip gripper's jaws
    if (/^MT1140724-0240/.test(top)) return 'static';                 // S2's side guides, not the gripper
    if (/^MT1140724-0(161|170)/.test(top)) return ['g1', 'jaw'];      // gripper 1's jaws and pads
    if (/^MT1140724-0(181|190)/.test(top)) return ['g2', 'jaw'];
    if (/^(MHF2|MT1140724-0150)/.test(top)) return ['ec3', 'slider'];
    if (/^(MRHQ25|EC-TC5M|TN-Q)/.test(top)) return ['gu', 'rodEnd'];
    if (/^(CDQ2LC40|MT1140724-0(040|061|071|390|400)|RB14|SBFHM)/.test(top) || (y < 30 && z > 640)) return ['ec4', 'slider'];
    if (z > 925) return ['gu', 'rodEnd'];
    return box[5] < 700 ? 'static' : 'drop';
  }
  if (/TRANSFER_M&B/.test(unit)) {
    if (z < 662 || /^(CY3RG20|SHS15C2|MT1140723-0(470|580|390|570)|MSTH|FU-57TZ)/.test(top)) return names.some(n => /^\[SHS/.test(n)) ? ['mbs', 'rodEnd'] : 'static';
    if (/^MXQ8/.test(top)) return ['mbIn', 'rodEnd'];
    if (/^MXQ16L-50/.test(top) || /^MXQ20L/.test(top)) return ['mbs', 'rodEnd'];
    if (/^MXQ16L-40/.test(top)) return ['mbOutU', 'rodEnd'];
    if (/^NETHS/.test(top)) return z > 950 && x > 830 ? ['mbLoad', 'rodEnd'] : z > 950 ? ['mbOut', 'rodEnd'] : ['mbs', 'rodEnd'];   // the magnets' locating pins
    if (x > 830) {
      if (/^(MT1140723-0(460|600|161|260|340|400|410|240)|PZ-V11)/.test(top)) return ['mbLoad', 'rodEnd'];
      if (/^MT1140723-0(510|420|430|440|450)/.test(top)) return ['mbIn', 'rodEnd'];
      return ['mbs', 'rodEnd'];
    }
    if (/^MT1140723-0(121|540|290|300)/.test(top)) return ['mbOutU', 'rodEnd'];
    if (/^(MT1140723-0(520|240|161|600|331|320|340)|PZ-V11)/.test(top)) return ['mbOut', 'rodEnd'];
    return ['mbs', 'rodEnd'];
  }
  if (/LOADER_WORK/.test(unit)) {
    if (/^MT1160002-0(171|180|192|331|700)/.test(top)) return ['tslide', 'rodEnd'];
    if (/^MT1160002-0(461|540|600|610|620)/.test(top)) return y < AX[1] ? ['gp1', 'jaw'] : ['gp2', 'jaw'];   // the MHSL3 jaws chuck
    if (/^(\*MHSL3|MT1160002-0(314|520)|MT1140723-0340)/.test(top)) return y < AX[1] ? ['gp1s', 'rodEnd'] : ['gp2s', 'rodEnd'];
    if (/^(MXQ16-125|MT1160002-0(710|720|440))/.test(top)) return ['ec2', 'slider'];
    if (/^(EC-RTC18M|MT1160002-0(220|251|261|271|281|730|740))/.test(top)) return ['ec1', 'slider'];
    if (/^SHS20V/.test(top)) return names.some(n => /^\[SHS/.test(n)) ? ['ec1', 'slider'] : 'static';
    if (/^SHS15C1/.test(top)) return names.some(n => /^\[SHS/.test(n)) ? ['tslide', 'rodEnd'] : 'static';
    if (/^(MT1160002-0(041|051|071|351|400|410)|CDQ2B12|HCDG10)/.test(top)) return ['tray', 'rodEnd'];
    if (/^\*RGHTGT/.test(top)) return 'drop';                          // (its bodies ride the tray, above)
    return 'static';                                                    // stand, lift, chute, NG box, the lift's own body
  }
  if (/ROTATE_WORK/.test(unit)) {
    // Denny: the PRODUCT turns, not the jig - only the shaft under it goes round with the motor.
    if (/^MT1250374-0(061|051)/.test(top)) return ['rot', 'top'];
    if (/^(PSFTCA|LHFC)/.test(top)) return ['pin', 'rodEnd'];
    if (/^MT1250374-0(330|340)/.test(top)) return 'drop';             // feet of a profile frame that is not drawn
    return 'static';
  }
  return 'static';
}
/**
 * One body's fate: null (dropped), or what it rides ('static' or [component, socket]).
 * @param {any} m the iCAD model @param {any} b a body
 */
function decide(m, b) {
  const ch = []; for (let n = m.nodes[b.node]; n && n.id > 1; n = m.nodes[n.parent]) ch.push(n);
  const names = ch.map(n => n.name), unit = ch[ch.length - 1]?.name ?? '';
  const top = ch.length > 1 ? ch[ch.length - 2].name : '';         // the unit's direct child ('' for a body right under the unit)
  const topId = ch.length > 1 ? ch[ch.length - 2].id : b.node;
  const ride = BODY_RIDES[b.id];
  if (names.some(n => /^light$/i.test(n))) return null;               // a drawn sensor BEAM, not a part
  if (!ride && !KEEP.test(top) && names.some(n => DROP.test(n))) return null;
  const diag = Math.hypot(b.box[3] - b.box[0], b.box[4] - b.box[1], b.box[5] - b.box[2]);
  if (diag < 12 && !ride) return null;
  const ctr = [0, 1, 2].map(d => (b.box[d] + b.box[d + 3]) / 2);
  const cls = ride ?? classify(unit, top, names, ctr, b.box);
  if (cls === 'drop') return null;
  return { cls, diag, top, unit, topId };
}
export { decide as classifyForDebug };

/** @param {number[][]} tris @param {number} cell */
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

/** @param {string} file */
async function buildCad(file) {
  const { readIcad, stlBinary } = await import('./icad.js');
  const m = readIcad(file);
  let worst = 0;
  for (const b of m.bodies) for (const p of m.tris(b.id)) for (let d = 0; d < 3; d++) worst = Math.max(worst, b.box[d] - p[d], p[d] - b.box[d + 3]);
  if (worst > 0.2) throw new Error('iCAD decode does not match the file\'s own boxes (off by ' + worst.toFixed(2) + ' mm): not this layout');
  /** @param {any} b */
  const chain = b => { const out = []; for (let n = m.nodes[b.node]; n && n.id > 1; n = m.nodes[n.parent]) out.push(n); return out; };
  // The scene at the CAD's pose: each moving shell is written in the frame of the socket it rides.
  const scene0 = build([]);
  const W = worldPoses(scene0, CAD_DOF), { defs } = compile(scene0);
  /** @type {Map<string, {tris: number[][], boxes: number[][], label: string, color: string, parent?: string, socket?: string}>} */
  const groups = new Map();
  const hex = (/** @type {number[]} */ c) => '#' + c.map(v => v.toString(16).padStart(2, '0')).join('');
  let kept = 0, raw = 0;
  for (const b of m.bodies) {
    const dec = decide(m, b);
    if (!dec) continue;
    let { cls, diag } = dec;
    // A jaw rides the gripper finger on ITS side, so a chuck moves it: the side (or, on a 3-jaw
    // gripper, the nearest radial finger) from where the jaw is in the gripper's frame.
    if (cls !== 'static' && cls[1] === 'jaw') {
      const g = cls[0], gc = scene0.components.find((/** @type {any} */ x) => x.id === g), gp = { ...gc.params };
      const Fg = W[g][defs.get(g).root], l = apply(invert(Fg), [0, 1, 2].map(d => (b.box[d] + b.box[d + 3]) / 2));
      if (gp.fingers3) {
        const ang = Math.atan2(l[1], l[0]) * 180 / Math.PI;
        let best = 1, bd = 1e9;
        for (let k = 0; k < 3; k++) { const d = Math.abs(((ang - (gp.radialDeg + k * 120)) % 360 + 540) % 360 - 180); if (d < bd) { bd = d; best = k + 1; } }
        cls = [g, 'f' + best];
      } else cls = [g, l[1] >= 0 ? 'fingerR' : 'fingerL'];
    }
    const sh = PRE_SHIFT[b.id];
    const bb = sh ? [0, 1, 2, 3, 4, 5].map(i => b.box[i] + sh[i % 3]) : b.box;
    // Each body in the colours the iCAD page draws it in: one per mesh (tools/icad.js parts()).
    const bodyParts = m.parts(b.id);
    const nTri = bodyParts.reduce((s, p) => s + p.tris.length / 3, 0);
    bodyParts.forEach((bp, k) => {
      const col = hex(bp.color);
      const key = cls === 'static' ? 'shl_' + col.slice(1) : 'mv_' + cls[0] + (/rodEnd|slider|top/.test(cls[1]) ? '' : '_' + cls[1]) + '_' + col.slice(1);
      const grp = groups.get(key) || groups.set(key, { tris: [], boxes: [], label: cls === 'static' ? 'CAD ' + col : 'CAD ' + cls[0] + ' ' + col, color: col,
        ...(cls === 'static' ? {} : { parent: cls[0], socket: cls[1] }) }).get(key);
      const t = sh ? bp.tris.map(p => [p[0] + sh[0], p[1] + sh[1], p[2] + sh[2]]) : bp.tris;
      raw += t.length / 3;
      // Decimation: a body of many triangles (a slotted profile, a caster, a motor) on a coarser grid.
      // 285k triangles in at 1 mm was twice what an integrated GPU draws smoothly (ce-insert: 102k).
      const c = cluster(t, nTri > 1500 ? 4 : nTri > 400 ? 2.5 : 1.2);
      kept += c.length / 3;
      grp.tris.push(...c);
      if (k === 0 && (diag > 60 || grp.parent)) grp.boxes.push(bb);
    });
  }
  fs.mkdirSync(path.join(ROOT, ASSET_DIR), { recursive: true });
  for (const f of fs.readdirSync(path.join(ROOT, ASSET_DIR))) if (f.endsWith('.stl')) fs.unlinkSync(path.join(ROOT, ASSET_DIR, f));
  const shells = [];
  for (const [id, g] of groups) {
    let F = IDENTITY;
    if (g.parent) {
      const s = defs.get(g.parent).sockets[/** @type {string} */ (g.socket)];
      if (!s) throw new Error('CAD shell ' + id + ': ' + g.parent + ' has no socket ' + g.socket);
      F = compose(W[g.parent][s.link], pose(s.at, s.rot));
    }
    const inv = invert(F);
    const pts = g.parent ? g.tris.map(p => apply(inv, p)) : g.tris;
    const asset = ASSET_DIR + '/' + id + '.stl';
    fs.writeFileSync(path.join(ROOT, asset), stlBinary(pts));
    // Fallback boxes in the same frame. A socket turned by a multiple of 90 degrees keeps an
    // axis-aligned box one; the flip and the tilt are drawn at 0, so every socket here is.
    const bx = g.boxes.map(b => {
      const c = apply(inv, [(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2]);
      const e = [b[3] - b[0], b[4] - b[1], b[5] - b[2]];
      const ax = [0, 1, 2].map(i => qrot(inv.q, [i === 0 ? 1 : 0, i === 1 ? 1 : 0, i === 2 ? 1 : 0]).map(Math.abs));
      const size = [0, 1, 2].map(k => ax[0][k] * e[0] + ax[1][k] * e[1] + ax[2][k] * e[2]);
      return [...c, ...size].map(v => Math.round(v * 10) / 10);
    }).sort((a, b) => b[3] * b[4] * b[5] - a[3] * a[4] * a[5]).slice(0, 60);
    shells.push({ id, label: g.label, asset, color: g.color, boxes: bx, ...(g.parent ? { parent: g.parent, socket: g.socket } : {}), tris: pts.length / 3 });
  }
  shells.sort((a, b) => (a.id < b.id ? -1 : 1));
  // The products, as the CAD draws them (Denny: the M&B's connector must be seen turning), each in
  // its part frame - origin at the underside centre - one STL per colour.
  /** @type {Record<string, Array<{asset: string, color: string}>>} */
  const products = {};
  for (const [tpl, node] of Object.entries(PRODUCT_NODES)) {
    /** @type {number[]} */
    const bs = [];
    const walk = (/** @type {number} */ nd) => { bs.push(...m.nodes[nd].bodies); for (const k of m.nodes[nd].kids) walk(k); };
    walk(node);
    const lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
    for (const b of bs) for (let d = 0; d < 3; d++) { lo[d] = Math.min(lo[d], m.bodies[b - 1].box[d]); hi[d] = Math.max(hi[d], m.bodies[b - 1].box[d + 3]); }
    const o = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]];
    /** @type {Map<string, number[][]>} */
    const byCol = new Map();
    for (const b of bs) for (const bp of m.parts(b)) {
      const col = hex(bp.color);
      (byCol.get(col) || byCol.set(col, []).get(col))?.push(...cluster(bp.tris.map(p => [p[0] - o[0], p[1] - o[1], p[2] - o[2]]), 0.6));
    }
    products[tpl] = [...byCol].sort((a, b) => b[1].length - a[1].length).map(([col, t]) => {
      const asset = ASSET_DIR + '/prod_' + tpl + '_' + col.slice(1) + '.stl';
      fs.writeFileSync(path.join(ROOT, asset), stlBinary(t));
      return { asset, color: col };
    });
  }
  fs.writeFileSync(CAD_JSON, JSON.stringify({ source: path.basename(file), note: 'written by tools/gen_final_caulking.js --cad; the STLs are gitignored', products, shells }, null, 1) + '\n');
  console.log('CAD: ' + raw + ' triangles in, ' + kept + ' kept in ' + shells.length + ' shells (decode checked to ' + worst.toFixed(3) + ' mm)');
}

// ---------------------------------------------------------------------------- outputs
function outputs() {
  const cad = fs.existsSync(CAD_JSON) ? JSON.parse(fs.readFileSync(CAD_JSON, 'utf8')) : { shells: [] };
  const scene = build(cad.shells, cad.products);
  const errs = validate(scene);
  if (errs.length) throw new Error('generated scene is invalid:\n  ' + errs.join('\n  '));
  return stringify(scene);
}

async function main() {
  const args = process.argv.slice(2);
  const ci = args.indexOf('--cad');
  if (ci >= 0) await buildCad(args[ci + 1]);
  const f = path.join(ROOT, 'scenes', NAME + '.json');
  const want = outputs(), have = fs.existsSync(f) ? fs.readFileSync(f, 'utf8') : null;
  if (have === want) return;
  if (args.includes('--check')) { console.log('STALE ' + path.relative(ROOT, f)); process.exit(1); }
  fs.writeFileSync(f, want);
  console.log('wrote ' + path.relative(ROOT, f));
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) main().catch(e => { console.error(e.stack || e.message); process.exit(1); });
