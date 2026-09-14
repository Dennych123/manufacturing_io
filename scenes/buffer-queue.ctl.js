// INTERNAL CONTROLLER - not a PLC. The same sequence as buffer-queue.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/**
 * A step that has not moved for this long is stuck. Longer than the other scenes: metering waits
 * for a part to travel the whole buffer belt, and the buffer can legitimately start empty.
 */
const WD_MS = 25000;
/** Fault step: the buffer belt stops and AUTO_RUN drops, which also holds the feeder off. */
const FAULT = 900;

export function create() {
  let pbLast = false, stopReq = false, demandFrom = -1, demandQ = false;
  let stepLast = -1, stepFrom = 0;
  return {
    /** The plant was reset: start the demand timer and the edge memory over. */
    reset() { pbLast = false; stopReq = false; demandFrom = -1; demandQ = false; stepLast = -1; stepFrom = 0; },

    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_STOP) stopReq = true;

      switch (io.ST1_STEP) {
        case 0:
          io.CV_ACC_RUN = false;
          if (startEdge) { stopReq = false; io.ST1_STEP = 10; }
          break;
        case 10:
          io.CV_ACC_RUN = false;
          if (demandQ) io.ST1_STEP = 15;
          break;
        case 15:
          io.CV_ACC_RUN = true;
          if (!io.PE_EXIT) io.ST1_STEP = 20;
          break;
        case 20:
          io.CV_ACC_RUN = true;
          if (io.PE_EXIT) io.ST1_STEP = 30;
          break;
        case 30:
          io.CV_ACC_RUN = true;
          if (!io.PE_EXIT) {
            io.CV_ACC_RUN = false;
            io.CYCLE_CNT += 1;
            io.ST1_STEP = stopReq ? 0 : 10;
          }
          break;
        case FAULT:
          // Stuck: the metered part was taken off the belt, or the buffer never filled.
          io.CV_ACC_RUN = false;
          if (startEdge) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }

      // TON after the CASE, as in the ST: its Q is read by the NEXT scan.
      if (io.ST1_STEP === 10) { if (demandFrom < 0) demandFrom = t; } else demandFrom = -1;
      demandQ = demandFrom >= 0 && t - demandFrom >= 2000;

      // Watchdog: a step that stops moving is a jam, not patience (CLAUDE.md).
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && t - stepFrom >= WD_MS) { io.ST1_STEP = FAULT; io.CV_ACC_RUN = false; }

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT;
      io.EM_EN = io.AUTO_RUN && !io.PE_FULL;                    // the feeder stops with AUTO_RUN
      io.PL_START = io.AUTO_RUN;
    },
  };
}
