// INTERNAL CONTROLLER - not a PLC. The same sequence as assembler.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the ~1.2 s index. */
const WD_MS = 15000;
/** Fault step: table and feeders off, press up, AUTO_RUN off, waits for START to be acknowledged. */
const FAULT = 900;
/** Home step: every actuator is driven to its home position. AUTO will not start until it ends. */
const HOME = 800;
/** E-STOP step: the master circuit is open and every solenoid is de-energised. */
const ESTOP = 910;
/** Individual buttons that toggle an actuator (the button is momentary, the memory is the PLC's). */
const TOGGLES = ['IND_PRESS'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, emBLast = 0, emLLast = 0, nextSt = 0, pressFrom = -1, pressQ = false;
  let stepLast = -1, stepFrom = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; emBLast = 0; emLLast = 0; nextSt = 0; pressFrom = -1; pressQ = false; stepLast = -1; stepFrom = 0;
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
          io.TBL_RUN = false; io.EM_B_EMIT = false; io.EM_L_EMIT = false;
          io.SOL_PRESS_DN = false; io.SOL_PRESS_UP = true;
          if (startEdge && auto && ready && homed && io.TBL_INPOS && io.AS_PRESS_UP) { stopReq = false; emBLast = io.EM_B_CNT; io.ST1_STEP = 10; }
          break;
        case 10:
          io.EM_B_EMIT = true;
          if (io.EM_B_CNT !== emBLast) { io.EM_B_EMIT = false; emLLast = io.EM_L_CNT; io.ST1_STEP = 20; }
          break;
        case 20:
          io.EM_L_EMIT = true;
          if (io.EM_L_CNT !== emLLast) { io.EM_L_EMIT = false; io.ST1_STEP = 30; }
          break;
        case 30:
          io.SOL_PRESS_UP = false; io.SOL_PRESS_DN = true;
          if (io.AS_PRESS_DN) io.ST1_STEP = 40;
          break;
        case 40:
          if (pressQ) io.ST1_STEP = 50;
          break;
        case 50:
          io.SOL_PRESS_DN = false; io.SOL_PRESS_UP = true;
          if (io.AS_PRESS_UP) { nextSt = (io.TBL_STATION + 1) % 4; io.ST1_STEP = 60; }
          break;
        case 60:
          // the invariant: this index has landed on the station it was sent to
          io.TBL_RUN = true;
          if (io.TBL_INPOS && io.TBL_STATION === nextSt) {
            io.TBL_RUN = false;
            io.CYCLE_CNT += 1;
            if (stopReq) io.ST1_STEP = 0; else { emBLast = io.EM_B_CNT; io.ST1_STEP = 10; }
          }
          break;
        case HOME:
          // HOME: press up, and the table turns until its origin station is back under the loader: a real home.
          io.EM_B_EMIT = false; io.EM_L_EMIT = false;
          io.SOL_PRESS_DN = false; io.SOL_PRESS_UP = true;
          io.TBL_RUN = !io.TBL_ORIGIN;
          if (io.AS_PRESS_UP && io.TBL_ORIGIN) { io.TBL_RUN = false; homed = true; io.ST1_STEP = 0; }
          break;
        case ESTOP:
          // The master circuit is open, so every solenoid de-energises - which is what really
          // happens: a 5/2 single-solenoid valve springs back, a double one holds where it is. The holders keep their parts: a real machine does not let go when the power drops.
          io.TBL_RUN = false; io.EM_B_EMIT = false; io.EM_L_EMIT = false; io.SOL_PRESS_DN = false;
          io.SOL_PRESS_UP = false;
          if (ready) io.ST1_STEP = 0;                  // MASTER ON after the mushroom is released
          break;
        case FAULT:
          // Stuck: a feeder never answered, the table never landed, or the selector was turned
          // while running. Table and feeders off, press up. The nests keep what they hold.
          io.TBL_RUN = false; io.EM_B_EMIT = false; io.EM_L_EMIT = false;
          io.SOL_PRESS_DN = false; io.SOL_PRESS_UP = true;
          if (startEdge && auto) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }

      // TON after the CASE, as in the ST: its Q is read by the NEXT scan.
      if (io.ST1_STEP === 40) { if (pressFrom < 0) pressFrom = t; } else pressFrom = -1;
      pressQ = pressFrom >= 0 && t - pressFrom >= 300;

      // Watchdog: a running step that stops moving is a jam, not patience. A selector change
      // while running trips the same way.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP && (t - stepFrom >= WD_MS || selChanged)) {
        io.ST1_STEP = FAULT; io.TBL_RUN = false; io.EM_B_EMIT = false; io.EM_L_EMIT = false; io.SOL_PRESS_DN = false; io.SOL_PRESS_UP = true;
      }

      // E-STOP at any moment, and the HOME button when the machine is energised and idle.
      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        io.TBL_RUN = false; io.EM_B_EMIT = false; io.EM_L_EMIT = false; io.SOL_PRESS_DN = false;
        io.SOL_PRESS_UP = false;
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      // Individual operation: momentary buttons, PLC toggle memory cleared when INDIVIDUAL ends. The
      // index button is held: the cam turns while it is down, as on a jog.
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      if (individual) {
        io.EM_B_EMIT = !!io.IND_FEED_B; io.EM_L_EMIT = !!io.IND_FEED_L;
        io.SOL_PRESS_DN = !!indMem.IND_PRESS; io.SOL_PRESS_UP = !indMem.IND_PRESS;
        io.TBL_RUN = !!io.IND_INDEX;
      }
      // The speed override the operator dialled in, passed on to the axes and the belts.
      io.OVR = io.OVR_SET;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
