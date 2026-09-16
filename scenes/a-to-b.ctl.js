// INTERNAL CONTROLLER - not a PLC. The same sequence as a-to-b.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the ~5 s belt run. */
const WD_MS = 15000;
/** Fault step: outputs off, AUTO_RUN off, waits for START to be acknowledged. */
const FAULT = 900;
/** Home step: every actuator is driven to its home position. AUTO will not start until it ends. */
const HOME = 800;
/** E-STOP step: the master circuit is open and every solenoid is de-energised. */
const ESTOP = 910;
/** Individual buttons that toggle an actuator (the button is momentary, the memory is the PLC's). */
const TOGGLES = ['IND_CV'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, dwellFrom = -1, dwellQ = false, emLast = 0;
  let stepLast = -1, stepFrom = 0, gone = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; dwellFrom = -1; dwellQ = false; emLast = 0; stepLast = -1; stepFrom = 0; gone = 0;
      masterOn = false; homed = false; masterLast = false; homeLast = false;
      for (const b of TOGGLES) { indMem[b] = false; indLast[b] = false; }
    },
    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_CSTOP) stopReq = true;
      // Selector AUTO / INDIVIDUAL (rb4axis panel): START only in AUTO, the individual buttons only in
      // INDIVIDUAL, and a change while the sequence runs stops it - FAULT, not a pause.
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
          io.CV1_RUN = false;
          io.EM1_EMIT = false;
          if (startEdge && auto && ready && homed) { stopReq = false; emLast = io.EM1_CNT; io.ST1_STEP = 10; }
          break;
        case 10:
          io.EM1_EMIT = true;
          if (io.EM1_CNT !== emLast) { io.EM1_EMIT = false; io.ST1_STEP = 15; }
          break;
        case 15:
          io.CV1_RUN = true;
          if (!io.PE_END) io.ST1_STEP = 20;
          break;
        case 20:
          io.CV1_RUN = true;
          if (io.PE_END) { io.CV1_RUN = false; io.ST1_STEP = 30; }
          break;
        case 30:
          if (dwellQ) io.ST1_STEP = 40;
          break;
        case 40:
          // the invariant, not "a count moved": every part loaded has left the belt, counting the
          // ones written off at START
          io.CV1_RUN = true;
          if (io.RM1_CNT + gone >= io.EM1_CNT) { io.CV1_RUN = false; io.ST1_STEP = 50; }
          break;
        case HOME:
          // HOME: nothing here is positioned, so home only has to leave the line quiet.
          io.CV1_RUN = false; io.EM1_EMIT = false;
          homed = true; io.ST1_STEP = 0;
          break;
        case ESTOP:
          // The master circuit is open, so every solenoid de-energises - which is what really
          // happens: a 5/2 single-solenoid valve springs back, a double one holds where it is.
          io.CV1_RUN = false; io.EM1_EMIT = false;
          if (ready) io.ST1_STEP = 0;                  // MASTER ON after the mushroom is released
          break;
        case FAULT:
          // Stuck: a part was taken, jammed or lost. Hold everything and wait for the operator.
          io.CV1_RUN = false;
          io.EM1_EMIT = false;
          // Acknowledging the fault writes off whatever never reached the unloader. Only HERE:
          // doing it at every START writes off parts that are still legitimately on the belt, and
          // then an older part's removal ends this cycle - the pipelining bug that cost two live
          // rounds on the simulator (docs/PLAN.md §13).
          if (startEdge && auto) { stopReq = false; gone = io.EM1_CNT - io.RM1_CNT; io.ST1_STEP = 0; }
          break;
        case 50:
          io.CYCLE_CNT += 1;
          if (stopReq) io.ST1_STEP = 0; else { emLast = io.EM1_CNT; io.ST1_STEP = 10; }
          break;
      }

      // TON after the CASE, as in the ST: its Q is read by the NEXT scan.
      if (io.ST1_STEP === 30) { if (dwellFrom < 0) dwellFrom = t; } else dwellFrom = -1;
      dwellQ = dwellFrom >= 0 && t - dwellFrom >= 500;

      // Watchdog: a running step that stops moving is a jam, not patience. Without it the
      // sequence waits for ever and the machine only LOOKS alive - belt running, AUTO on.
      // A selector change while running trips the same way.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP && (t - stepFrom >= WD_MS || selChanged)) { io.ST1_STEP = FAULT; io.CV1_RUN = false; io.EM1_EMIT = false; }

      // E-STOP at any moment, and the HOME button when the machine is energised and idle.
      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        io.CV1_RUN = false; io.EM1_EMIT = false;
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      // Individual operation: the buttons are momentary; the PLC keeps a toggle
      // memory per actuator and clears it when INDIVIDUAL ends, so nothing stays latched into AUTO.
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      if (individual) { io.CV1_RUN = !!indMem.IND_CV; io.EM1_EMIT = !!io.IND_FEED; }
      // The speed override the operator dialled in, passed on to the axes and the belts.
      io.OVR = io.OVR_SET;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
