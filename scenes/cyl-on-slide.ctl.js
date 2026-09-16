// INTERNAL CONTROLLER - not a PLC. The same sequence as cyl-on-slide.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the ~2 s slide. */
const WD_MS = 15000;
/** Fault step: servo stopped, cylinder up, AUTO_RUN off, waits for START to be acknowledged. */
const FAULT = 900;
/** Home step: every actuator is driven to its home position. AUTO will not start until it ends. */
const HOME = 800;
/** E-STOP step: the master circuit is open and every solenoid is de-energised. */
const ESTOP = 910;
/** Individual buttons that toggle an actuator (the button is momentary, the memory is the PLC's). */
const TOGGLES = ['IND_CYL'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, dwellFrom = -1, dwellQ = false, stepLast = -1, stepFrom = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  return {
    /** The plant was reset: start the dwell timer and the edge memories over. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; dwellFrom = -1; dwellQ = false; stepLast = -1; stepFrom = 0;
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
          io.SOL_ST1_PRSS_CYL_DN = false;
          io.SOL_ST1_PRSS_CYL_UP = true;
          io.SV1_EXEC = false;
          if (startEdge && auto && ready && homed && io.AS_ST1_PRSS_CYL_UP && io.SV1_INPOS && !io.SV1_DONE) { stopReq = false; io.ST1_STEP = 10; }
          break;
        case 10:
          io.SV1_TGT = 400;
          io.SV1_EXEC = true;
          if (io.SV1_DONE) { io.SV1_EXEC = false; io.ST1_STEP = 20; }
          break;
        case 20:
          if (!io.SV1_DONE) io.ST1_STEP = 30;
          break;
        case 30:
          io.SOL_ST1_PRSS_CYL_UP = false;
          io.SOL_ST1_PRSS_CYL_DN = true;
          if (io.AS_ST1_PRSS_CYL_DN) io.ST1_STEP = 40;
          break;
        case 40:
          if (dwellQ) io.ST1_STEP = 50;
          break;
        case 50:
          io.SOL_ST1_PRSS_CYL_DN = false;
          io.SOL_ST1_PRSS_CYL_UP = true;
          if (io.AS_ST1_PRSS_CYL_UP) io.ST1_STEP = 60;
          break;
        case 60:
          io.SV1_TGT = 0;
          io.SV1_EXEC = true;
          if (io.SV1_DONE) { io.SV1_EXEC = false; io.ST1_STEP = 70; }
          break;
        case 70:
          if (!io.SV1_DONE) { io.CYCLE_CNT += 1; io.ST1_STEP = stopReq ? 0 : 10; }
          break;
        case HOME:
          // HOME: press cylinder up, slide back to zero.
          io.SOL_ST1_PRSS_CYL_DN = false; io.SOL_ST1_PRSS_CYL_UP = true;
          io.SV1_TGT = 0; io.SV1_EXEC = true;
          if (io.AS_ST1_PRSS_CYL_UP && io.SV1_DONE) { io.SV1_EXEC = false; homed = true; io.ST1_STEP = 0; }
          break;
        case ESTOP:
          // The master circuit is open, so every solenoid de-energises - which is what really
          // happens: a 5/2 single-solenoid valve springs back, a double one holds where it is.
          io.SV1_EXEC = false; io.SOL_ST1_PRSS_CYL_DN = false; io.SOL_ST1_PRSS_CYL_UP = false;
          if (ready) io.ST1_STEP = 0;                  // MASTER ON after the mushroom is released
          break;
        case FAULT:
          // Stuck: the servo never reported, or the selector was turned while running. Servo
          // stopped, cylinder up, wait for START in AUTO.
          io.SV1_EXEC = false;
          io.SOL_ST1_PRSS_CYL_DN = false;
          io.SOL_ST1_PRSS_CYL_UP = true;
          if (startEdge && auto) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }

      // TON after the CASE, as in the ST: its Q is read by the NEXT scan.
      if (io.ST1_STEP === 40) { if (dwellFrom < 0) dwellFrom = t; } else dwellFrom = -1;
      dwellQ = dwellFrom >= 0 && t - dwellFrom >= 300;

      // Watchdog: a running step that stops moving is a jam, not patience. A selector change
      // while running trips the same way.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP && (t - stepFrom >= WD_MS || selChanged)) {
        io.ST1_STEP = FAULT; io.SV1_EXEC = false; io.SOL_ST1_PRSS_CYL_DN = false; io.SOL_ST1_PRSS_CYL_UP = true;
      }

      // E-STOP at any moment, and the HOME button when the machine is energised and idle.
      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        io.SV1_EXEC = false; io.SOL_ST1_PRSS_CYL_DN = false; io.SOL_ST1_PRSS_CYL_UP = false;
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      // Individual operation: momentary buttons, PLC toggle memory cleared when INDIVIDUAL ends. The
      // servo buttons are held: Execute is a level, and the model finishes the move on its own.
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      if (individual) {
        io.SOL_ST1_PRSS_CYL_DN = !!indMem.IND_CYL;
        io.SOL_ST1_PRSS_CYL_UP = !indMem.IND_CYL;
      }
      // The speed override the operator dialled in, passed on to the axes and the belts.
      io.OVR = io.OVR_SET;
      // Jog: the axis creeps while the button is held, and only on INDIVIDUAL.
      io.SV1_JOG_P = individual && !!io.IND_SV1_P;
      io.SV1_JOG_N = individual && !!io.IND_SV1_N;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
