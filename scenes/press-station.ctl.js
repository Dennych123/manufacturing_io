// INTERNAL CONTROLLER - not a PLC. The same sequence as press-station.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the ~2 s unload. */
const WD_MS = 15000;
/** Fault step: feeding off, press up, ejector back, AUTO_RUN off, waits for START to be acknowledged. */
const FAULT = 900;
/** Home step: every actuator is driven to its home position. AUTO will not start until it ends. */
const HOME = 800;
/** E-STOP step: the master circuit is open and every solenoid is de-energised. */
const ESTOP = 910;
/** Individual buttons that toggle an actuator (the button is momentary, the memory is the PLC's). */
const TOGGLES = ['IND_CLAMP', 'IND_PRESS', 'IND_EJECT'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, emLast = 0;
  let pressFrom = -1, pressQ = false, stepLast = -1, stepFrom = 0, gone = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; emLast = 0; pressFrom = -1; pressQ = false; stepLast = -1; stepFrom = 0; gone = 0;
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

      switch (io.ST1_STEP) {
        case 0:
          io.CLAMP = false; io.EM_EMIT = false; io.SOL_EJECT = false;
          io.SOL_P_DN = false; io.SOL_P_UP = true;
          if (startEdge && auto && ready && homed && io.AS_P_UP && io.AS_EJ_BACK) { stopReq = false; emLast = io.EM_CNT; io.ST1_STEP = 10; }
          break;
        case 10:
          // clamp first, so the nest LOCATES the part the feeder drops (docs/PLAN.md §3)
          io.CLAMP = true; io.EM_EMIT = true;
          if (io.EM_CNT !== emLast) { io.EM_EMIT = false; io.ST1_STEP = 20; }
          break;
        case 20:
          if (io.NEST_P) io.ST1_STEP = 30;
          break;
        case 30:
          io.SOL_P_UP = false; io.SOL_P_DN = true;
          if (io.AS_P_DN) io.ST1_STEP = 40;
          break;
        case 40:
          // press time
          if (pressQ) io.ST1_STEP = 50;
          break;
        case 50:
          io.SOL_P_DN = false; io.SOL_P_UP = true;
          if (io.AS_P_UP) io.ST1_STEP = 60;
          break;
        case 60:
          // unclamp and push the part off the nest onto the chute
          io.CLAMP = false; io.SOL_EJECT = true;
          if (io.AS_EJ_OUT) io.ST1_STEP = 70;
          break;
        case 70:
          // the invariant, not "a count moved": every part fed has reached the bin
          io.SOL_EJECT = false;
          if (io.AS_EJ_BACK && io.RM_CNT + gone >= io.EM_CNT) io.ST1_STEP = 80;
          break;
        case 80:
          io.CYCLE_CNT += 1;
          if (stopReq) io.ST1_STEP = 0; else { emLast = io.EM_CNT; io.ST1_STEP = 10; }
          break;
        case HOME:
          // HOME: press up, ejector back. The clamp is left alone: a real machine does not drop the part it holds.
          io.EM_EMIT = false; io.SOL_EJECT = false;
          io.SOL_P_DN = false; io.SOL_P_UP = true;
          if (io.AS_P_UP && io.AS_EJ_BACK) { homed = true; io.ST1_STEP = 0; }
          break;
        case ESTOP:
          // The master circuit is open, so every solenoid de-energises - which is what really
          // happens: a 5/2 single-solenoid valve springs back, a double one holds where it is. The holders keep their parts: a real machine does not let go when the power drops.
          io.EM_EMIT = false; io.SOL_P_DN = false; io.SOL_P_UP = false; io.SOL_EJECT = false;
          if (ready) io.ST1_STEP = 0;                  // MASTER ON after the mushroom is released
          break;
        case FAULT:
          // Stuck: the part was taken out of the nest, never arrived, never reached the bin, or
          // the selector was turned while running. Feeding off, press up, ejector back; the clamp
          // keeps whatever it holds.
          io.EM_EMIT = false; io.SOL_EJECT = false; io.SOL_P_DN = false; io.SOL_P_UP = true;
          // Acknowledging writes off whatever never reached the bin. Only HERE (docs/PLAN.md §13).
          if (startEdge && auto) { stopReq = false; gone = io.EM_CNT - io.RM_CNT; io.ST1_STEP = 0; }
          break;
      }

      // TON after the CASE, as in the ST: its Q is read by the NEXT scan.
      if (io.ST1_STEP === 40) { if (pressFrom < 0) pressFrom = t; } else pressFrom = -1;
      pressQ = pressFrom >= 0 && t - pressFrom >= 500;

      // Watchdog: a running step that stops moving is a jam, not patience. A selector change
      // while running trips the same way.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP && (t - stepFrom >= WD_MS || selChanged)) {
        io.ST1_STEP = FAULT; io.EM_EMIT = false; io.SOL_EJECT = false; io.SOL_P_DN = false; io.SOL_P_UP = true;
      }

      // E-STOP at any moment, and the HOME button when the machine is energised and idle.
      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        io.EM_EMIT = false; io.SOL_P_DN = false; io.SOL_P_UP = false; io.SOL_EJECT = false;
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      // Individual operation: momentary buttons, PLC toggle memory cleared when INDIVIDUAL ends.
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      if (individual) {
        io.EM_EMIT = !!io.IND_FEED; io.CLAMP = !!indMem.IND_CLAMP;
        io.SOL_P_DN = !!indMem.IND_PRESS; io.SOL_P_UP = !indMem.IND_PRESS; io.SOL_EJECT = !!indMem.IND_EJECT;
      }
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
