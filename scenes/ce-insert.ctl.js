// INTERNAL CONTROLLER - not a PLC. The same sequence as ce-insert.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.
//
// Denny's CE INSERT machine (tools/gen_ce_insert.js has the machine), ported from his NX1P2
// program "Ce Insert Track" (Prg010_ST1_Supply_Feeder, Prg011_ST2_Buffer, Prg012_ST3_CE_Eject).
// Three stations run at once and meet on shared state, as his three programs meet on GB010/011/012:
//
//   ST1  the gripper-stoppers of the active type's lane pair open and pins run down to the divider
//        until each lane's FULL beam holds (then that gripper chucks); when the shuttle is waiting
//        at a receiving position the divider opens, the batch runs off, and it shuts again once the
//        INSERT FLOW OUT beam has been dark for 500 ms. "Buffering" and "Discharging" in his program.
//   ST2  the shuttle (E-RBM6 slide) takes two batches onto its four lanes: type 1 (left feeder,
//        lanes 1-2) fills shuttle lanes 1-2 at CENTER and 3-4 at LEFT; type 2 (right feeder, lanes
//        3-4) fills 3-4 at CENTER and 1-2 at RIGHT. Full, it goes to CENTER, lifts STOPPER-1 and lets
//        all four lanes run onto the chute - but only while the chute's FULL beam is dark.
//   ST3  when all four lanes have a pin at the end (SHUTTER EXIST 1-4), the separator takes those
//        four out of the queue, goes back, the pusher pushes them over the receivers and they drop
//        down the pipes; the cycle ends once every FLOW OUT beam has seen its pin and gone dark.
//
// Two things his program does that this one does NOT copy, both found while porting:
//   - ST3 drives the EJECTOR valve from its PUSHER symbols and the other way round (Device_Output
//     CR_ST3_PSH_FWD -> CH5_00 "EJECTOR FORWARD"). Denny confirmed the swap; here the separator
//     (CE EJECTOR plate) goes first and the pusher second, under their own names.
//   - ST2's Memory rung sets "shutter 3, 4 full" whenever the axis is at POS4, with no batch
//     delivered, so type 1 could never take its second batch. Here a shuttle lane pair is full
//     only when ST1 has discharged into it.

// BEGIN GENERATED - tools/gen_ce_insert.js
/** Shuttle positions on the E-RBM6 slide, mm: CENTER lines the shuttle up with the feeder and chute lanes. */
export const SV = {"CENTER":110,"LEFT":30,"RIGHT":190};
// END GENERATED

/** A step that has not moved for this long is stuck. The longest normal move is well under 2 s. */
const WD_MS = 15000;
const HOME = 800, FAULT = 900, ESTOP = 910;
/** Debounce of the queue beams: a lane is FULL when its beam has held this long (his LT on-delays). */
const FULL_MS = 300;
/** A batch has run off when the flow-out beam has been dark this long. */
const FLOW_OFF_MS = 500;
/** All four EXIST beams must hold this long before the separator goes. */
const EXS_MS = 200;
/** The drop is over when the four FLOW OUT beams have been dark this long. */
const DROP_MS = 100;
/** Steps that WAIT for material, not for a move: the watchdog leaves them alone. */
const WAITING = [0, 10, 100, 120, 200];
const TOGGLES = ['IND_STP2', 'IND_STP3', 'IND_STP4', 'IND_STP5', 'IND_DIVL', 'IND_DIVR', 'IND_STP1', 'IND_SEP', 'IND_PSH', 'IND_FDR1', 'IND_FDR2'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, masterOn = false, homed = false, masterLast = false, homeLast = false;
  /** Where each actuator is told to be. The solenoids are written from these every scan. */
  const chk = [true, true, true, true];          // ST1 STOPPER-2..5: chucked
  const divF = [true, true];                     // ST1 LEFT / RIGHT DIVIDER: forward (closed)
  let stpDn = true, sepF = false, pshF = false;
  const fdr = [false, false];
  /** ST2's memory of which shuttle lanes hold a batch, and ST1's "I have discharged into it". */
  const mem = [false, false, false, false];
  let deliv = false;
  /** ST2's move: target and which lane pair the batch will land on. */
  let svTgt = SV.CENTER, half = 0;
  /** ST3: the FLOW OUT beams seen during this drop. */
  const flw = [false, false, false, false];
  /** A batch has been SEEN in the flow-out beam: waiting for "dark" before the pins even got there
   * would close the gate on them (CLAUDE.md: a step that waits for a level must first see it clear). */
  let seen1 = false, seen2 = false;
  /** On-delay / off-delay timers: [start time or -1]. */
  const onT = {}, wd = { ST1_STEP: [-1, 0], ST2_STEP: [-1, 0], ST3_STEP: [-1, 0] };
  const indMem = {}, indLast = {};
  let indMove = false, indTgt = 0, indMoveLast = { c: false, l: false, r: false };

  /** TON: true once `cond` has held for `ms`. @param {string} k @param {boolean} cond @param {number} ms @param {number} t */
  const ton = (k, cond, ms, t) => {
    if (!cond) { onT[k] = -1; return false; }
    if (onT[k] == null || onT[k] < 0) onT[k] = t;
    return t - onT[k] >= ms;
  };

  return {
    reset() {
      pbLast = false; stopReq = false; selLast = true; masterOn = false; homed = false; masterLast = false; homeLast = false;
      chk.fill(true); divF.fill(true); stpDn = true; sepF = false; pshF = false; fdr.fill(false);
      mem.fill(false); deliv = false; svTgt = SV.CENTER; half = 0; flw.fill(false); seen1 = false; seen2 = false;
      for (const k of Object.keys(onT)) delete onT[k];
      for (const k of Object.keys(wd)) wd[k] = [-1, 0];
      for (const b of TOGGLES) { indMem[b] = false; indLast[b] = false; }
      indMove = false; indTgt = 0; indMoveLast = { c: false, l: false, r: false };
    },

    /** One PLC scan. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_CSTOP) stopReq = true;
      const auto = io.SEL_AUTO !== false, selChanged = auto !== selLast;
      selLast = auto;
      const estop = !!io.PB_ESTOP;
      const masterEdge = io.PB_MASTER && !masterLast;
      masterLast = !!io.PB_MASTER;
      const homeEdge = io.PB_HOME && !homeLast;
      homeLast = !!io.PB_HOME;
      if (estop) { masterOn = false; homed = false; }
      else if (masterEdge) masterOn = true;
      const ready = masterOn && !estop;
      const type1 = !io.SEL_TYPE2;
      /** The active type's lane pair on ST1. */
      const a = type1 ? 0 : 2, b = a + 1;
      const full = [1, 2, 3, 4].map(n => ton('full' + n, !!io['PH_ST1_SHT' + n + '_FULL'], FULL_MS, t));
      const st1FlowOff = ton('st1off', !io.PH_ST1_INS_FLW_OUT, FLOW_OFF_MS, t);
      const st2FlowOff = ton('st2off', !io.PH_ST2_INS_FLW_OUT, FLOW_OFF_MS, t);
      const st3Full = ton('st3full', !!io.PH_ST3_SHT_FULL, FULL_MS, t);
      const exs = ton('exs', !!(io.PH_ST3_SHT1_EXS && io.PH_ST3_SHT2_EXS && io.PH_ST3_SHT3_EXS && io.PH_ST3_SHT4_EXS), EXS_MS, t);
      const dropOff = ton('drop', !(io.PH_ST3_SHT1_FLW_OUT || io.PH_ST3_SHT2_FLW_OUT || io.PH_ST3_SHT3_FLW_OUT || io.PH_ST3_SHT4_FLW_OUT), DROP_MS, t);
      /** ST2 is standing at a receiving position, waiting for this type's batch (his GB011_11 / 12). */
      const st2Ready = io.ST2_STEP === 120;

      // ---------------------------------------------------------- ST1: supply feeder
      switch (io.ST1_STEP) {
        case 0:
          fdr[0] = false; fdr[1] = false;
          if (startEdge && auto && ready && homed && io.AS_ST2_STP1_DN && io.AS_ST3_EJC_BWD && io.AS_ST3_PSH_BWD) {
            stopReq = false;
            io.ST2_STEP = 100; io.ST3_STEP = 200;
            io.ST1_STEP = 10;
          }
          break;
        case 10:
          // The bowl of the running type feeds all the time; its lanes back up to the grippers.
          fdr[0] = type1; fdr[1] = !type1;
          if (!(full[a] && full[b])) io.ST1_STEP = 20;
          else if (st2Ready) io.ST1_STEP = 40;
          break;
        case 20:
          // Buffering: both grippers of the pair open.
          chk[a] = false; chk[b] = false;
          if (io['AS_ST1_STP' + (a + 2) + '_UCHK'] && io['AS_ST1_STP' + (b + 2) + '_UCHK']) io.ST1_STEP = 30;
          break;
        case 30:
          // Each gripper chucks as soon as its own lane is full.
          if (full[a]) chk[a] = true;
          if (full[b]) chk[b] = true;
          if (chk[a] && chk[b] && io['AS_ST1_STP' + (a + 2) + '_CHK'] && io['AS_ST1_STP' + (b + 2) + '_CHK']) io.ST1_STEP = 10;
          break;
        case 40:
          // Discharging: the pair's divider opens.
          divF[type1 ? 0 : 1] = false;
          seen1 = false;
          if (io[type1 ? 'AS_ST1_LFT_DIV_BWD' : 'AS_ST1_RGT_DIV_BWD']) io.ST1_STEP = 45;
          break;
        case 45:
          if (io.PH_ST1_INS_FLW_OUT) seen1 = true;
          if (seen1 && st1FlowOff) io.ST1_STEP = 50;
          break;
        case 50:
          divF[0] = true; divF[1] = true;
          if (io.AS_ST1_LFT_DIV_FWD && io.AS_ST1_RGT_DIV_FWD) {
            deliv = true;                          // ST2 books the batch onto the lanes it stands at
            io.ST1_STEP = stopReq ? 0 : 10;
          }
          break;
        case HOME:
          // Grippers chucked, dividers and STOPPER-1 closed, separator and pusher back, shuttle CENTER.
          chk.fill(true); divF.fill(true); stpDn = true; sepF = false; pshF = false; fdr.fill(false);
          io.SV_ST2_TGT = SV.CENTER; io.SV_ST2_EXEC = true;
          if (io.SV_ST2_DONE && io.AS_ST1_STP2_CHK && io.AS_ST1_STP3_CHK && io.AS_ST1_STP4_CHK && io.AS_ST1_STP5_CHK
              && io.AS_ST1_LFT_DIV_FWD && io.AS_ST1_RGT_DIV_FWD && io.AS_ST2_STP1_DN && io.AS_ST3_EJC_BWD && io.AS_ST3_PSH_BWD) {
            io.SV_ST2_EXEC = false;
            mem.fill(false); deliv = false;
            homed = true; io.ST1_STEP = 0;
          }
          break;
        case ESTOP:
          fdr.fill(false); io.SV_ST2_EXEC = false;
          if (ready) io.ST1_STEP = 0;
          break;
        case FAULT:
          fdr.fill(false); io.SV_ST2_EXEC = false;
          if (startEdge && auto) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }
      const running = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;

      // ---------------------------------------------------------- ST2: the shuttle
      if (!running) io.ST2_STEP = 0;
      else switch (io.ST2_STEP) {
        case 100: {
          const n = mem.filter(Boolean).length;
          if (n === 4) { if (!st3Full && st2FlowOff) { svTgt = SV.CENTER; io.ST2_STEP = 160; } }
          else if (n === 0) { svTgt = SV.CENTER; half = type1 ? 0 : 2; io.ST2_STEP = 110; }
          else if (type1 && mem[0] && mem[1] && !mem[2] && !mem[3]) { svTgt = SV.LEFT; half = 2; io.ST2_STEP = 110; }
          else if (!type1 && mem[2] && mem[3] && !mem[0] && !mem[1]) { svTgt = SV.RIGHT; half = 0; io.ST2_STEP = 110; }
          // Half a load of the other type (the type was changed): run it off rather than stand here.
          else if (!st3Full && st2FlowOff) { svTgt = SV.CENTER; io.ST2_STEP = 160; }
          break;
        }
        case 110:
          io.SV_ST2_TGT = svTgt; io.SV_ST2_EXEC = true;
          if (io.SV_ST2_DONE) { io.SV_ST2_EXEC = false; io.ST2_STEP = 115; }
          break;
        case 115:
          if (!io.SV_ST2_DONE) io.ST2_STEP = 120;
          break;
        case 120:
          // Standing at a receiving position: ST1 discharges into the lanes in front of it.
          if (deliv) { deliv = false; mem[half] = true; mem[half + 1] = true; io.ST2_STEP = 100; }
          break;
        case 160:
          io.SV_ST2_TGT = SV.CENTER; io.SV_ST2_EXEC = true;
          if (io.SV_ST2_DONE) { io.SV_ST2_EXEC = false; io.ST2_STEP = 165; }
          break;
        case 165:
          if (!io.SV_ST2_DONE) io.ST2_STEP = 170;
          break;
        case 170:
          stpDn = false;
          seen2 = false;
          if (io.AS_ST2_STP1_UP) io.ST2_STEP = 175;
          break;
        case 175:
          if (io.PH_ST2_INS_FLW_OUT) seen2 = true;
          if (seen2 && st2FlowOff) io.ST2_STEP = 180;
          break;
        case 180:
          stpDn = true;
          if (io.AS_ST2_STP1_DN) { mem.fill(false); io.ST2_STEP = 100; }
          break;
        default:
          io.ST2_STEP = 100;
      }

      // ---------------------------------------------------------- ST3: CE eject
      if (!running) io.ST3_STEP = 0;
      else switch (io.ST3_STEP) {
        case 200:
          sepF = false; pshF = false;
          if (exs) io.ST3_STEP = 210;
          break;
        case 210:
          sepF = true;
          if (io.AS_ST3_EJC_FWD) io.ST3_STEP = 215;
          break;
        case 215:
          // The four separated pins are still in the beams, in the slots.
          if (io.PH_ST3_SHT1_EXS && io.PH_ST3_SHT2_EXS && io.PH_ST3_SHT3_EXS && io.PH_ST3_SHT4_EXS) io.ST3_STEP = 220;
          break;
        case 220:
          sepF = false;
          if (io.AS_ST3_EJC_BWD) { flw.fill(false); io.ST3_STEP = 230; }
          break;
        case 230:
          pshF = true;
          if (io.AS_ST3_PSH_FWD) io.ST3_STEP = 240;
          break;
        case 240:
          pshF = false;
          if (io.AS_ST3_PSH_BWD) io.ST3_STEP = 250;
          break;
        case 250:
          // Every pin went down its pipe, and none is still in a beam.
          if (flw.every(Boolean) && dropOff) { io.CYCLE_CNT += 1; io.ST3_STEP = 200; }
          break;
        default:
          io.ST3_STEP = 200;
      }
      if (io.ST3_STEP >= 230 && io.ST3_STEP <= 250) for (let i = 0; i < 4; i++) if (io['PH_ST3_SHT' + (i + 1) + '_FLW_OUT']) flw[i] = true;

      // Watchdog, one per station, on steps that move something.
      const stuck = (/** @type {'ST1_STEP'|'ST2_STEP'|'ST3_STEP'} */ tag) => {
        const w = wd[tag];
        if (io[tag] !== w[0]) { w[0] = io[tag]; w[1] = t; }
        return running && !WAITING.includes(io[tag]) && t - w[1] >= WD_MS;
      };
      const jam = stuck('ST1_STEP') || stuck('ST2_STEP') || stuck('ST3_STEP');
      if (running && (jam || selChanged)) { io.ST1_STEP = FAULT; io.ST2_STEP = 0; io.ST3_STEP = 0; io.SV_ST2_EXEC = false; fdr.fill(false); }
      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        io.ST2_STEP = 0; io.ST3_STEP = 0; io.SV_ST2_EXEC = false; fdr.fill(false);
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;

      // Individual operation: momentary buttons, PLC toggle memory cleared when INDIVIDUAL ends.
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const k of TOGGLES) { if (individual && io[k] && !indLast[k]) indMem[k] = !indMem[k]; if (!individual) indMem[k] = false; indLast[k] = !!io[k]; }
      if (individual) {
        chk[0] = !indMem.IND_STP2; chk[1] = !indMem.IND_STP3; chk[2] = !indMem.IND_STP4; chk[3] = !indMem.IND_STP5;
        divF[0] = !indMem.IND_DIVL; divF[1] = !indMem.IND_DIVR;
        stpDn = !indMem.IND_STP1; sepF = !!indMem.IND_SEP; pshF = !!indMem.IND_PSH;
        fdr[0] = !!indMem.IND_FDR1; fdr[1] = !!indMem.IND_FDR2;
        // Shuttle to a position: one press, one move; Execute held until Done, then dropped.
        const pc = !!io.IND_SV_CTR, pl = !!io.IND_SV_LFT, pr = !!io.IND_SV_RGT;
        if (!indMove && ((pc && !indMoveLast.c) || (pl && !indMoveLast.l) || (pr && !indMoveLast.r))) {
          indMove = true; indTgt = pc ? SV.CENTER : pl ? SV.LEFT : SV.RIGHT;
        }
        indMoveLast = { c: pc, l: pl, r: pr };
        io.SV_ST2_TGT = indTgt;
        io.SV_ST2_EXEC = indMove;
        if (indMove && io.SV_ST2_DONE) indMove = false;
      } else { indMove = false; indMoveLast = { c: false, l: false, r: false }; }

      // Solenoids from where each actuator is told to be. Without MASTER ON (and on E-STOP) the
      // master circuit is open and every coil drops: a double-solenoid valve then holds where it is.
      const on = ready;
      for (let i = 0; i < 4; i++) {
        io['SOL_ST1_STP' + (i + 2) + '_CHK'] = on && chk[i];
        io['SOL_ST1_STP' + (i + 2) + '_UCHK'] = on && !chk[i];
      }
      io.SOL_ST1_LFT_DIV_FWD = on && divF[0]; io.SOL_ST1_LFT_DIV_BWD = on && !divF[0];
      io.SOL_ST1_RGT_DIV_FWD = on && divF[1]; io.SOL_ST1_RGT_DIV_BWD = on && !divF[1];
      io.SOL_ST2_STP1_DN = on && stpDn; io.SOL_ST2_STP1_UP = on && !stpDn;
      io.SOL_ST3_EJC_FWD = on && sepF; io.SOL_ST3_EJC_BWD = on && !sepF;
      io.SOL_ST3_PSH_FWD = on && pshF; io.SOL_ST3_PSH_BWD = on && !pshF;
      io.CR_ST1_PART_FDR1_STR = on && fdr[0];
      io.CR_ST1_PART_FDR2_STR = on && fdr[1];

      io.OVR = io.OVR_SET;
      io.SV_ST2_JOG_P = individual && !!io.IND_SV_P;
      io.SV_ST2_JOG_N = individual && !!io.IND_SV_N;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
      io.PL_SV_CTR = Math.abs(io.SV_ST2_POS - SV.CENTER) < 0.5;
      io.PL_SV_LFT = Math.abs(io.SV_ST2_POS - SV.LEFT) < 0.5;
      io.PL_SV_RGT = Math.abs(io.SV_ST2_POS - SV.RIGHT) < 0.5;
    },
  };
}
