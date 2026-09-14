// INTERNAL CONTROLLER - not a PLC. The same sequence as a-to-b.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the ~5 s belt run. */
const WD_MS = 15000;
/** Fault step: outputs off, AUTO_RUN off, waits for START to be acknowledged. */
const FAULT = 900;

export function create() {
  let pbLast = false, stopReq = false, dwellFrom = -1, dwellQ = false, emLast = 0;
  let stepLast = -1, stepFrom = 0, gone = 0;
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() { pbLast = false; stopReq = false; dwellFrom = -1; dwellQ = false; emLast = 0; stepLast = -1; stepFrom = 0; gone = 0; },
    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_STOP) stopReq = true;

      switch (io.ST1_STEP) {
        case 0:
          io.CV1_RUN = false;
          io.EM1_EMIT = false;
          if (startEdge) { stopReq = false; emLast = io.EM1_CNT; io.ST1_STEP = 10; }
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
        case FAULT:
          // Stuck: a part was taken, jammed or lost. Hold everything and wait for the operator.
          io.CV1_RUN = false;
          io.EM1_EMIT = false;
          // Acknowledging the fault writes off whatever never reached the unloader. Only HERE:
          // doing it at every START writes off parts that are still legitimately on the belt, and
          // then an older part's removal ends this cycle - the pipelining bug that cost two live
          // rounds on the simulator (docs/PLAN.md §13).
          if (startEdge) { stopReq = false; gone = io.EM1_CNT - io.RM1_CNT; io.ST1_STEP = 0; }
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
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && t - stepFrom >= WD_MS) { io.ST1_STEP = FAULT; io.CV1_RUN = false; io.EM1_EMIT = false; }

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT;
      io.PL_START = io.AUTO_RUN;
    },
  };
}
