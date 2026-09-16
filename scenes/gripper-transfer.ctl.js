// INTERNAL CONTROLLER - not a PLC. The same sequence as gripper-transfer.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the ~4 s belt run. */
const WD_MS = 15000;
/** Fault step: belts and feeding off, cylinders and gripper hold, AUTO_RUN off, START acknowledges. */
const FAULT = 900;
/** Home step: every actuator is driven to its home position. AUTO will not start until it ends. */
const HOME = 800;
/** E-STOP step: the master circuit is open and every solenoid is de-energised. */
const ESTOP = 910;
/** Individual buttons that toggle an actuator (the button is momentary, the memory is the PLC's). */
const TOGGLES = ['IND_CV1', 'IND_CV2', 'IND_LIFT', 'IND_TRAV', 'IND_GRIP'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, emLast = 0;
  let settleFrom = -1, settleQ = false, gripFrom = -1, gripQ = false, stepLast = -1, stepFrom = 0, gone = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; emLast = 0;
      settleFrom = -1; settleQ = false; gripFrom = -1; gripQ = false; stepLast = -1; stepFrom = 0; gone = 0;
      masterOn = false; homed = false; masterLast = false; homeLast = false;
      for (const b of TOGGLES) { indMem[b] = false; indLast[b] = false; }
    },

    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_CSTOP) stopReq = true;
      // Selector AUTO / INDIVIDUAL: START only in AUTO, individual buttons only in INDIVIDUAL, a change
      // while running is a FAULT.
      const auto = io.SEL_AUTO !== false, selChanged = auto !== selLast;
      selLast = auto;
      // The master circuit, as on the cell panel (rb4axis): E-STOP is a latching mushroom, MASTER
      // ON energises the machine, and nothing runs without it. An E-STOP also loses the home
      // position: the machine must be homed again before AUTO will start.
      const estop = !!io.PB_ESTOP;
      const masterEdge = io.PB_MASTER && !masterLast;
      masterLast = !!io.PB_MASTER;
      const homeEdge = io.PB_HOME && !homeLast;
      homeLast = !!io.PB_HOME;
      if (estop) { masterOn = false; homed = false; }
      else if (masterEdge) masterOn = true;
      const ready = masterOn && !estop;

      // A write-off goes STALE when the part turns up after all: the hand drops it back, or it
      // reaches the unloader later. RM catches up, RM + gone runs PAST EM, and the step waiting
      // on that invariant stops waiting at all - the pipelining bug again, silently.
      if (io.RM_CNT + gone > io.EM_CNT) gone = Math.max(0, io.EM_CNT - io.RM_CNT);
      switch (io.ST1_STEP) {
        case 0:
          io.CV1_RUN = false; io.CV2_RUN = false; io.EM_EMIT = false; io.GRIP_CLOSE = false;
          io.SOL_Z_DN = false; io.SOL_Z_UP = true; io.SOL_Y_OUT = false; io.SOL_Y_IN = true;
          if (startEdge && auto && ready && homed && io.AS_Z_UP && io.AS_Y_IN && io.GRIP_OPEN) { stopReq = false; emLast = io.EM_CNT; io.ST1_STEP = 10; }
          break;
        case 10:
          io.CV2_RUN = true; io.EM_EMIT = true;
          if (io.EM_CNT !== emLast) { io.EM_EMIT = false; io.ST1_STEP = 15; }
          break;
        case 15:
          io.CV1_RUN = true;
          if (!io.PE_PICK) io.ST1_STEP = 20;
          break;
        case 20:
          io.CV1_RUN = true;
          if (io.PE_PICK) { io.CV1_RUN = false; io.ST1_STEP = 30; }
          break;
        case 30:
          if (settleQ) io.ST1_STEP = 40;
          break;
        case 40:
          io.SOL_Z_UP = false; io.SOL_Z_DN = true;
          if (io.AS_Z_DN) io.ST1_STEP = 50;
          break;
        case 50:
          // A gripper has no "part gripped" switch. After the closing time the fingers are either
          // stopped by the part (neither switch) or fully closed on nothing: a missed grip is a fault.
          io.GRIP_CLOSE = true;
          if (gripQ) io.ST1_STEP = io.GRIP_CLOSED ? FAULT : 60;
          break;
        case 60:
          io.SOL_Z_DN = false; io.SOL_Z_UP = true;
          if (io.AS_Z_UP) io.ST1_STEP = 70;
          break;
        case 70:
          io.SOL_Y_IN = false; io.SOL_Y_OUT = true;
          if (io.AS_Y_OUT) io.ST1_STEP = 80;
          break;
        case 80:
          io.SOL_Z_UP = false; io.SOL_Z_DN = true;
          if (io.AS_Z_DN) io.ST1_STEP = 90;
          break;
        case 90:
          io.GRIP_CLOSE = false;
          if (io.GRIP_OPEN) io.ST1_STEP = 100;
          break;
        case 100:
          io.SOL_Z_DN = false; io.SOL_Z_UP = true;
          if (io.AS_Z_UP) io.ST1_STEP = 110;
          break;
        case 110:
          io.SOL_Y_OUT = false; io.SOL_Y_IN = true;
          if (io.AS_Y_IN) io.ST1_STEP = 120;
          break;
        case 120:
          // the invariant, not "a count moved": every part loaded has left the outfeed belt
          if (io.RM_CNT + gone >= io.EM_CNT) {
            io.CYCLE_CNT += 1;
            if (stopReq) io.ST1_STEP = 0; else { emLast = io.EM_CNT; io.ST1_STEP = 10; }
          }
          break;
        case HOME:
          // HOME: lift up and traverse in. The gripper is left alone: a real machine does not drop the part it holds.
          io.CV1_RUN = false; io.CV2_RUN = false; io.EM_EMIT = false;
          io.SOL_Z_DN = false; io.SOL_Z_UP = true; io.SOL_Y_OUT = false; io.SOL_Y_IN = true;
          if (io.AS_Z_UP && io.AS_Y_IN) { homed = true; io.ST1_STEP = 0; }
          break;
        case ESTOP:
          // The master circuit is open, so every solenoid de-energises - which is what really
          // happens: a 5/2 single-solenoid valve springs back, a double one holds where it is. The holders keep their parts: a real machine does not let go when the power drops.
          io.CV1_RUN = false; io.EM_EMIT = false; io.SOL_Y_OUT = false; io.SOL_Y_IN = false;
          io.SOL_Z_DN = false; io.SOL_Z_UP = false; io.CV2_RUN = false;
          if (ready) io.ST1_STEP = 0;                  // MASTER ON after the mushroom is released
          break;
        case FAULT:
          // Stuck, a missed grip, or the selector turned while running: belts and feeding off.
          // The gripper and the cylinders stay where they are - a real machine does not drop the
          // part it is holding.
          io.CV1_RUN = false; io.CV2_RUN = false; io.EM_EMIT = false;
          // Acknowledging writes off whatever never reached the outfeed. Only HERE (docs/PLAN.md §13).
          if (startEdge && auto) { stopReq = false; gone = io.EM_CNT - io.RM_CNT; io.ST1_STEP = 0; }
          break;
      }

      // TONs after the CASE, as in the ST: their Q is read by the NEXT scan.
      if (io.ST1_STEP === 30) { if (settleFrom < 0) settleFrom = t; } else settleFrom = -1;
      settleQ = settleFrom >= 0 && t - settleFrom >= 300;
      if (io.ST1_STEP === 50) { if (gripFrom < 0) gripFrom = t; } else gripFrom = -1;
      gripQ = gripFrom >= 0 && t - gripFrom >= 400;

      // Watchdog: a running step that stops moving is a jam, not patience. A selector change
      // while running trips the same way.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP && (t - stepFrom >= WD_MS || selChanged)) {
        io.ST1_STEP = FAULT; io.CV1_RUN = false; io.CV2_RUN = false; io.EM_EMIT = false;
      }

      // E-STOP at any moment, and the HOME button when the machine is energised and idle.
      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        io.CV1_RUN = false; io.EM_EMIT = false; io.SOL_Y_OUT = false; io.SOL_Y_IN = false;
        io.SOL_Z_DN = false; io.SOL_Z_UP = false; io.CV2_RUN = false;
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      // Individual operation: momentary buttons, PLC toggle memory cleared when INDIVIDUAL ends.
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      if (individual) {
        io.CV1_RUN = !!indMem.IND_CV1; io.CV2_RUN = !!indMem.IND_CV2; io.EM_EMIT = !!io.IND_FEED;
        io.SOL_Z_DN = !!indMem.IND_LIFT; io.SOL_Z_UP = !indMem.IND_LIFT;
        io.SOL_Y_OUT = !!indMem.IND_TRAV; io.SOL_Y_IN = !indMem.IND_TRAV;
        io.GRIP_CLOSE = !!indMem.IND_GRIP;
      }
      // The speed override the operator dialled in, passed on to the axes and the belts.
      io.OVR = io.OVR_SET;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
