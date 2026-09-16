// INTERNAL CONTROLLER - not a PLC. The same sequence as sort-by-material.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the ~5 s belt run. */
const WD_MS = 15000;
/** Fault step: feeding, belt and pusher off, AUTO_RUN off, waits for START to be acknowledged. */
const FAULT = 900;
/** Home step: every actuator is driven to its home position. AUTO will not start until it ends. */
const HOME = 800;
/** E-STOP step: the master circuit is open and every solenoid is de-energised. */
const ESTOP = 910;
/** Individual buttons that toggle an actuator (the button is momentary, the memory is the PLC's). */
const TOGGLES = ['IND_CV', 'IND_PUSH'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, metalTurn = true, metal = false, emMLast = 0, emPLast = 0;
  let settleFrom = -1, settleQ = false, stepLast = -1, stepFrom = 0, goneM = 0, goneP = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; metalTurn = true; metal = false; emMLast = 0; emPLast = 0;
      settleFrom = -1; settleQ = false; stepLast = -1; stepFrom = 0; goneM = 0; goneP = 0;
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
      if (io.RM_M_CNT + goneM > io.EM_M_CNT) goneM = Math.max(0, io.EM_M_CNT - io.RM_M_CNT);
      if (io.RM_P_CNT + goneP > io.EM_P_CNT) goneP = Math.max(0, io.EM_P_CNT - io.RM_P_CNT);
      switch (io.ST1_STEP) {
        case 0:
          io.CV_RUN = false; io.EM_M_EMIT = false; io.EM_P_EMIT = false; io.SOL_PUSH = false;
          if (startEdge && auto && ready && homed && io.AS_PUSH_RET) { stopReq = false; emMLast = io.EM_M_CNT; emPLast = io.EM_P_CNT; io.ST1_STEP = 10; }
          break;
        case 10:
          // feed one part, steel and plastic by turns; the metal latch starts clean
          metal = false;
          if (metalTurn) {
            io.EM_M_EMIT = true;
            if (io.EM_M_CNT !== emMLast) { io.EM_M_EMIT = false; io.ST1_STEP = 15; }
          } else {
            io.EM_P_EMIT = true;
            if (io.EM_P_CNT !== emPLast) { io.EM_P_EMIT = false; io.ST1_STEP = 15; }
          }
          break;
        case 15:
          io.CV_RUN = true;
          if (!io.PE_STOP) io.ST1_STEP = 20;
          break;
        case 20:
          io.CV_RUN = true;
          if (io.PE_STOP) { io.CV_RUN = false; io.ST1_STEP = 30; }
          break;
        case 30:
          if (settleQ) io.ST1_STEP = 40;
          break;
        case 40:
          // the inductive sensor upstream saw this part pass (or not): sort by the latch
          io.ST1_STEP = metal ? 50 : 60;
          break;
        case 50:
          io.SOL_PUSH = true;
          if (io.AS_PUSH_EXT) io.ST1_STEP = 55;
          break;
        case 55:
          // the invariant, not "a count moved": every metal part fed has reached the bin
          io.SOL_PUSH = false;
          if (io.AS_PUSH_RET && io.RM_M_CNT + goneM >= io.EM_M_CNT) io.ST1_STEP = 70;
          break;
        case 60:
          io.CV_RUN = true;
          if (io.RM_P_CNT + goneP >= io.EM_P_CNT) { io.CV_RUN = false; io.ST1_STEP = 70; }
          break;
        case 70:
          io.CYCLE_CNT += 1;
          metalTurn = !metalTurn;
          if (stopReq) io.ST1_STEP = 0; else { emMLast = io.EM_M_CNT; emPLast = io.EM_P_CNT; io.ST1_STEP = 10; }
          break;
        case HOME:
          // HOME: pusher back, belt and feeders off.
          io.CV_RUN = false; io.EM_M_EMIT = false; io.EM_P_EMIT = false; io.SOL_PUSH = false;
          if (io.AS_PUSH_RET) { homed = true; io.ST1_STEP = 0; }
          break;
        case ESTOP:
          // The master circuit is open, so every solenoid de-energises - which is what really
          // happens: a 5/2 single-solenoid valve springs back, a double one holds where it is.
          io.CV_RUN = false; io.EM_M_EMIT = false; io.EM_P_EMIT = false; io.SOL_PUSH = false;
          if (ready) io.ST1_STEP = 0;                  // MASTER ON after the mushroom is released
          break;
        case FAULT:
          // Stuck: a part was taken, jammed, lost, sorted the wrong way, or the selector was turned
          // while running. Hold everything.
          io.CV_RUN = false; io.EM_M_EMIT = false; io.EM_P_EMIT = false; io.SOL_PUSH = false;
          // Acknowledging writes off whatever never reached its bin. Only HERE (docs/PLAN.md §13).
          if (startEdge && auto) { stopReq = false; goneM = io.EM_M_CNT - io.RM_M_CNT; goneP = io.EM_P_CNT - io.RM_P_CNT; io.ST1_STEP = 0; }
          break;
      }

      // The metal latch: the part passes the inductive sensor on its way to the stop beam, so
      // the sensor is read while the belt carries it, not when it stands at the pusher.
      if ((io.ST1_STEP === 15 || io.ST1_STEP === 20) && io.PX_METAL) metal = true;

      // TON after the CASE, as in the ST: its Q is read by the NEXT scan.
      if (io.ST1_STEP === 30) { if (settleFrom < 0) settleFrom = t; } else settleFrom = -1;
      settleQ = settleFrom >= 0 && t - settleFrom >= 300;

      // Watchdog: a running step that stops moving is a jam, not patience. A selector change
      // while running trips the same way.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP && (t - stepFrom >= WD_MS || selChanged)) {
        io.ST1_STEP = FAULT; io.CV_RUN = false; io.EM_M_EMIT = false; io.EM_P_EMIT = false; io.SOL_PUSH = false;
      }

      // E-STOP at any moment, and the HOME button when the machine is energised and idle.
      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        io.CV_RUN = false; io.EM_M_EMIT = false; io.EM_P_EMIT = false; io.SOL_PUSH = false;
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      // Individual operation: momentary buttons, PLC toggle memory cleared when INDIVIDUAL ends.
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      if (individual) { io.CV_RUN = !!indMem.IND_CV; io.EM_M_EMIT = !!io.IND_FEED_M; io.EM_P_EMIT = !!io.IND_FEED_P; io.SOL_PUSH = !!indMem.IND_PUSH; }
      // The speed override the operator dialled in, passed on to the axes and the belts.
      io.OVR = io.OVR_SET;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
