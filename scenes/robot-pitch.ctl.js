// INTERNAL CONTROLLER - not a PLC. The same sequence as robot-pitch.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.
//
// The cell: a FANUC LR Mate 200iD (joint chain, limits and speeds from the ROS-Industrial xacro,
// which takes them from Fanuc's mechanical unit manual) carrying a cam-driven pitch-change head.
// Five cups take a ROW of five plugs from a 5 x 5 pallet at 100 mm pitch, the camshaft closes the
// cups to 60 mm, the row goes into a five-pocket jig, the jig does its process, and the row is
// picked again at 60 and dropped into the bin. One cycle = one row.
//
// The poses are joint angles solved ONCE by IK against this scene's own kinematics
// (scratchpad/makerobot.mjs, lib/ik.js) and pinned in tests/lib.test.js: every one lands within
// 0.05 mm with at least 30 degrees of margin to every joint limit. The program commands joint
// targets, as the real one does; nothing here solves kinematics at run time.
//
// Moves are joint-space: all six axes get their target and Execute together and the step waits
// for every Done. Execute is a LEVEL latched on its rising edge, so every move is followed by a
// step that drops Execute and waits for Done to clear before the next one (CLAUDE.md).

/** J1..J6 in degrees, pallet rows 0-4 (up = 100 mm above the pick), the jig, the bin, home. */
export const POSE = {"home":[0,0,0,0,-90,0],
 "pal0Up":[-26.56,11.48,-4.65,0.01,-73.87,26.55],
 "pal0At":[-26.57,16.99,-15.72,-0.01,-57.29,26.58],
 "pal1Up":[-14.03,5.51,-11.49,0.01,-73,14.02],
 "pal1At":[-14.04,11.47,-22.74,-0.01,-55.79,14.05],
 "pal2Up":[0,3.41,-13.75,0,-72.85,0],
 "pal2At":[0,9.49,-25.11,0,-55.4,0],
 "pal3Up":[14.03,5.51,-11.49,-0.01,-73,-14.02],
 "pal3At":[14.04,11.47,-22.74,0.01,-55.79,-14.05],
 "pal4Up":[26.56,11.48,-4.65,-0.01,-73.87,-26.55],
 "pal4At":[26.57,16.99,-15.72,0.01,-57.29,-26.58],
 "jigUp":[77.47,13.83,-1.79,0,-74.38,-77.47],
 "jigAt":[77.47,19.16,-12.81,0,-58.04,-77.48],
 "binUp":[46.97,46.74,55.89,0,-99.14,-46.97]};
const CAM_WIDE = 0, CAM_NARROW = 90;
const N = 5;
const AX = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
/** A step that has not moved for this long is stuck: the longest normal step is a ~2 s move. */
const WD_MS = 15000;
const FAULT = 900;
const HOME = 800;
const ESTOP = 910;
/** The loader has stopped delivering, so the pallet is full: it drops one every 40 ms when it can. */
const FILL_SETTLE = 600;
/** How long the cups are given to get hold of what is under them before the head carries on. */
const PICK_MS = 600;
/** Vacuum off to parts released. */
const DROP_MS = 300;
/** The jig's own process. */
const PROCESS_MS = 1000;
const TOGGLES = ['IND_VAC'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, stepLast = -1, stepFrom = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  let row = 0, needFill = true, palBase = 0, fillFrom = -1, fillQ = false, picked = false;
  let pickFrom = -1, pickQ = false, dropFrom = -1, dropQ = false, procFrom = -1, procQ = false;
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; stepLast = -1; stepFrom = 0;
      masterOn = false; homed = false; masterLast = false; homeLast = false;
      row = 0; needFill = true; palBase = 0; fillFrom = -1; fillQ = false; picked = false;
      pickFrom = -1; pickQ = false; dropFrom = -1; dropQ = false; procFrom = -1; procQ = false;
      for (const b of TOGGLES) { indMem[b] = false; indLast[b] = false; }
    },
    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_CSTOP) stopReq = true;
      // Selector AUTO / INDIVIDUAL (rb4axis panel): START only in AUTO, the individual buttons only
      // in INDIVIDUAL, and a change while the sequence runs stops it - FAULT, not a pause.
      const auto = io.SEL_AUTO !== false, selChanged = auto !== selLast;
      selLast = auto;
      // The master circuit, as on the cell panel: E-STOP is a latching mushroom, MASTER ON
      // energises the machine, and nothing runs without it. An E-STOP also loses the home position.
      const estop = !!io.PB_ESTOP;
      const masterEdge = io.PB_MASTER && !masterLast;
      masterLast = !!io.PB_MASTER;
      const homeEdge = io.PB_HOME && !homeLast;
      homeLast = !!io.PB_HOME;
      if (estop) { masterOn = false; homed = false; }
      else if (masterEdge) masterOn = true;
      const ready = masterOn && !estop;

      const allDone = AX.every(a => io[a + '_DONE']);
      const allClear = AX.every(a => !io[a + '_DONE']);
      const anyVac = [0, 1, 2, 3, 4].some(i => io['VAC_C' + i]);
      const noVac = !anyVac;
      /** Command a pose: targets and Execute held. True once every axis reports Done. */
      const go = pose => { AX.forEach((a, i) => { io[a + '_TGT'] = pose[i]; io[a + '_EXEC'] = true; }); return allDone; };
      /** Drop Execute; true once every Done has cleared, so the next Execute is a fresh edge. */
      const drop = () => { for (const a of AX) io[a + '_EXEC'] = false; return allClear; };
      const camGo = deg => { io.CAM_TGT = deg; io.CAM_EXEC = true; return !!io.CAM_DONE; };
      const camDrop = () => { io.CAM_EXEC = false; return !io.CAM_DONE; };
      const off = () => { for (const a of AX) io[a + '_EXEC'] = false; io.CAM_EXEC = false; io.EM_PAL_EN = false; };

      switch (io.ST1_STEP) {
        case 0:
          off(); io.VAC_ON = false; io.PAL_CLAMP = true; io.JIG_CLAMP = true;
          if (startEdge && auto && ready && homed) {
            stopReq = false;
            if (needFill) { palBase = io.EM_PAL_CNT; fillFrom = t; io.ST1_STEP = 5; } else io.ST1_STEP = 10;
          }
          break;
        case 5:
          // A fresh pallet. It is FULL when the loader stops delivering, not after a count of 25:
          // a pocket that still had a plug in it would make a count wait for ever.
          io.PAL_CLAMP = true; io.EM_PAL_EN = true;
          if (io.EM_PAL_CNT !== palBase) { palBase = io.EM_PAL_CNT; fillFrom = t; }
          else if (fillQ) { io.EM_PAL_EN = false; needFill = false; row = 0; io.ST1_STEP = 10; }
          break;
        case 10: { const a = camGo(CAM_WIDE), b = go(POSE['pal' + row + 'Up']); if (a && b) io.ST1_STEP = 11; } break;
        case 11: { const a = camDrop(), b = drop(); if (a && b) io.ST1_STEP = 12; } break;
        case 12: if (go(POSE['pal' + row + 'At'])) io.ST1_STEP = 13; break;
        case 13: if (drop()) io.ST1_STEP = 14; break;
        case 14:
          // The pallet lets go while the cups take what is there. A pocket may be empty - a plug
          // was taken out by hand - and the head carries on with what it got.
          io.PAL_CLAMP = false; io.VAC_ON = true;
          if (pickQ) { picked = anyVac; io.ST1_STEP = 15; }
          break;
        case 15: io.PAL_CLAMP = true; if (go(POSE['pal' + row + 'Up'])) io.ST1_STEP = 16; break;
        case 16: if (drop()) io.ST1_STEP = picked ? 17 : 36; break;
        case 17: { const a = camGo(CAM_NARROW), b = go(POSE.jigUp); if (a && b) io.ST1_STEP = 18; } break;
        case 18: { const a = camDrop(), b = drop(); if (a && b) io.ST1_STEP = 19; } break;
        case 19: if (go(POSE.jigAt)) io.ST1_STEP = 20; break;
        case 20: if (drop()) io.ST1_STEP = 21; break;
        case 21: io.JIG_CLAMP = true; io.VAC_ON = false; if (dropQ) io.ST1_STEP = 22; break;
        case 22: if (go(POSE.jigUp)) io.ST1_STEP = 23; break;
        case 23: if (drop()) io.ST1_STEP = 24; break;
        case 24: if (procQ) io.ST1_STEP = 25; break;                 // the jig does its work
        case 25: if (go(POSE.jigAt)) io.ST1_STEP = 26; break;
        case 26: if (drop()) io.ST1_STEP = 27; break;
        case 27: io.JIG_CLAMP = false; io.VAC_ON = true; if (pickQ) { picked = anyVac; io.ST1_STEP = 28; } break;
        case 28: io.JIG_CLAMP = true; if (go(POSE.jigUp)) io.ST1_STEP = 29; break;
        case 29: if (drop()) io.ST1_STEP = 30; break;
        case 30: if (go(POSE.binUp)) io.ST1_STEP = 31; break;
        case 31: if (drop()) io.ST1_STEP = 32; break;
        case 32: io.VAC_ON = false; if (dropQ) io.ST1_STEP = 33; break;
        case 33: if (camGo(CAM_WIDE)) io.ST1_STEP = 34; break;
        case 34: if (camDrop()) io.ST1_STEP = 35; break;
        case 35:
          io.CYCLE_CNT += 1; row += 1;
          if (row >= N) { row = 0; needFill = true; }
          if (stopReq) io.ST1_STEP = 0;
          else if (needFill) { palBase = io.EM_PAL_CNT; fillFrom = t; io.ST1_STEP = 5; }
          else io.ST1_STEP = 10;
          break;
        case 36:
          // Nothing came up from this row: not a cycle. Next row, or a fresh pallet.
          row += 1;
          if (row >= N) { row = 0; needFill = true; }
          if (stopReq) io.ST1_STEP = 0;
          else if (needFill) { palBase = io.EM_PAL_CNT; fillFrom = t; io.ST1_STEP = 5; }
          else io.ST1_STEP = 10;
          break;
        case HOME:
          // HOME: the arm to its travel pose, the cam wide. The cups keep what they hold.
          io.EM_PAL_EN = false;
          { const a = camGo(CAM_WIDE), b = go(POSE.home); if (a && b) { camDrop(); drop(); homed = true; io.ST1_STEP = 0; } }
          break;
        case ESTOP:
          // The master circuit is open: every drive stops where it is. The cups KEEP their plugs -
          // five plugs let go over the pallet is not what a real head does when the power drops.
          off();
          if (ready) io.ST1_STEP = 0;                  // MASTER ON after the mushroom is released
          break;
        case FAULT:
          // Stuck, or the selector was turned while running. Drives and the loader off; the cups
          // and the pockets keep what they hold.
          off();
          if (startEdge && auto) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }

      // TONs after the CASE, as in the ST: their Q is read by the NEXT scan.
      if (io.ST1_STEP === 14 || io.ST1_STEP === 27) { if (pickFrom < 0) pickFrom = t; } else pickFrom = -1;
      pickQ = pickFrom >= 0 && t - pickFrom >= PICK_MS;
      if ((io.ST1_STEP === 21 || io.ST1_STEP === 32) && noVac) { if (dropFrom < 0) dropFrom = t; } else dropFrom = -1;
      dropQ = dropFrom >= 0 && t - dropFrom >= DROP_MS;
      if (io.ST1_STEP === 24) { if (procFrom < 0) procFrom = t; } else procFrom = -1;
      procQ = procFrom >= 0 && t - procFrom >= PROCESS_MS;
      fillQ = io.ST1_STEP === 5 && fillFrom >= 0 && t - fillFrom >= FILL_SETTLE;

      // Watchdog: a running step that stops moving is a jam, not patience. A selector change while
      // running trips the same way.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP && (t - stepFrom >= WD_MS || selChanged)) { io.ST1_STEP = FAULT; off(); }

      // E-STOP at any moment, and the HOME button when the machine is energised and idle.
      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        off();
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      // Individual operation: jog buttons drive the joints straight (the axis model holds the
      // limits), the cam buttons command an end, the vacuum toggles. Nothing stays latched into AUTO.
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      for (let i = 1; i <= 6; i++) { io['J' + i + '_JOG_P'] = individual && !!io['IND_J' + i + '_P']; io['J' + i + '_JOG_N'] = individual && !!io['IND_J' + i + '_N']; }
      if (individual) {
        io.VAC_ON = !!indMem.IND_VAC;
        if (io.IND_CAM_WIDE) { io.CAM_TGT = CAM_WIDE; io.CAM_EXEC = true; }
        else if (io.IND_CAM_NARROW) { io.CAM_TGT = CAM_NARROW; io.CAM_EXEC = true; }
        else io.CAM_EXEC = false;
      }
      // The speed override the operator dialled in, passed on to the axes.
      io.OVR = io.OVR_SET;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
