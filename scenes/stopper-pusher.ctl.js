// INTERNAL CONTROLLER - not a PLC. The same sequence as stopper-pusher.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the ~2.5 s feed wait. */
const WD_MS = 15000;
/** Fault step: belt, feeder, stopper and pusher off, AUTO_RUN off, waits for START to be acknowledged. */
const FAULT = 900;
/** Home step: every actuator is driven to its home position. AUTO will not start until it ends. */
const HOME = 800;
/** E-STOP step: the master circuit is open and every solenoid is de-energised. */
const ESTOP = 910;
/** Individual buttons that toggle an actuator (the button is momentary, the memory is the PLC's). */
const TOGGLES = ['IND_CV', 'IND_FEED', 'IND_STOP', 'IND_PUSH'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, n = 0;
  let settleFrom = -1, settleQ = false, gapFrom = -1, gapQ = false, stepLast = -1, stepFrom = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  return {
    /** The plant was reset: start the sort count and the timers over. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; n = 0; settleFrom = -1; settleQ = false; gapFrom = -1; gapQ = false; stepLast = -1; stepFrom = 0;
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
          io.CV1_RUN = false; io.EM1_EN = false; io.SOL_STOP = false; io.SOL_PUSH = false;
          if (startEdge && auto && ready && homed && io.AS_PUSH_RET) { stopReq = false; io.ST1_STEP = 10; }
          break;
        case 10:
          io.SOL_STOP = true; io.CV1_RUN = true; io.EM1_EN = true;
          if (io.AS_STOP_DN) io.ST1_STEP = 20;
          break;
        case 20:
          if (settleQ) io.ST1_STEP = 30;
          break;
        case 30:
          n += 1;
          io.ST1_STEP = n % 2 === 1 ? 40 : 60;
          break;
        case 40:
          io.SOL_PUSH = true;
          if (io.AS_PUSH_EXT) io.ST1_STEP = 50;
          break;
        case 50:
          io.SOL_PUSH = false;
          if (io.AS_PUSH_RET && !io.PE_STOP) io.ST1_STEP = 80;
          break;
        case 60:
          io.SOL_STOP = false;
          if (!io.PE_STOP) io.ST1_STEP = 70;
          break;
        case 70:
          if (gapQ) { io.SOL_STOP = true; if (io.AS_STOP_DN) io.ST1_STEP = 80; }
          break;
        case 80:
          io.CYCLE_CNT += 1;
          io.ST1_STEP = stopReq ? 0 : 20;
          break;
        case HOME:
          // HOME: pusher back, stopper up, belt and loader off.
          io.CV1_RUN = false; io.EM1_EN = false; io.SOL_STOP = false; io.SOL_PUSH = false;
          if (io.AS_PUSH_RET && io.AS_STOP_UP) { homed = true; io.ST1_STEP = 0; }
          break;
        case ESTOP:
          // The master circuit is open, so every solenoid de-energises - which is what really
          // happens: a 5/2 single-solenoid valve springs back, a double one holds where it is.
          io.CV1_RUN = false; io.EM1_EN = false; io.SOL_STOP = false; io.SOL_PUSH = false;
          if (ready) io.ST1_STEP = 0;                  // MASTER ON after the mushroom is released
          break;
        case FAULT:
          // Stuck: a part never came, never left, or the selector was turned while running. Stop
          // feeding and the pusher, but LEAVE THE STOPPER DOWN - it is holding the queue back, and
          // a fault that releases it runs the whole queue into a machine that has just stopped.
          // (E-STOP is the other case: there the master circuit really does drop every solenoid.)
          io.CV1_RUN = false; io.EM1_EN = false; io.SOL_PUSH = false;
          if (startEdge && auto) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }

      // TONs after the CASE, as in the ST: their Q is read by the NEXT scan.
      if (io.ST1_STEP === 20 && io.PE_STOP) { if (settleFrom < 0) settleFrom = t; } else settleFrom = -1;
      settleQ = settleFrom >= 0 && t - settleFrom >= 300;
      if (io.ST1_STEP === 70) { if (gapFrom < 0) gapFrom = t; } else gapFrom = -1;
      gapQ = gapFrom >= 0 && t - gapFrom >= 400;

      // Watchdog: a running step that stops moving is a jam, not patience. A selector change
      // while running trips the same way.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP && (t - stepFrom >= WD_MS || selChanged)) {
        io.ST1_STEP = FAULT; io.CV1_RUN = false; io.EM1_EN = false; io.SOL_PUSH = false;   // stopper stays down
      }

      // E-STOP at any moment, and the HOME button when the machine is energised and idle.
      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        io.CV1_RUN = false; io.EM1_EN = false; io.SOL_STOP = false; io.SOL_PUSH = false;
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      // Individual operation: momentary buttons, PLC toggle memory cleared when INDIVIDUAL ends.
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      if (individual) { io.CV1_RUN = !!indMem.IND_CV; io.EM1_EN = !!indMem.IND_FEED; io.SOL_STOP = !!indMem.IND_STOP; io.SOL_PUSH = !!indMem.IND_PUSH; }
      // The speed override the operator dialled in, passed on to the axes and the belts.
      io.OVR = io.OVR_SET;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
