// INTERNAL CONTROLLER - not a PLC. The same sequence as stopper-pusher.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the belt run. */
const WD_MS = 15000;
/** Fault step: feeding and the pusher off, AUTO_RUN off, waits for START to acknowledge. */
const FAULT = 900;

export function create() {
  let pbLast = false, stopReq = false, n = 0;
  let settleFrom = -1, settleQ = false, gapFrom = -1, gapQ = false;
  let stepLast = -1, stepFrom = 0;
  return {
    /** The plant was reset: start the sort count and the timers over. */
    reset() { pbLast = false; stopReq = false; n = 0; settleFrom = -1; settleQ = false; gapFrom = -1; gapQ = false; stepLast = -1; stepFrom = 0; },
    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_STOP) stopReq = true;

      switch (io.ST1_STEP) {
        case 0:
          io.CV1_RUN = false; io.EM1_EN = false; io.SOL_STOP = false; io.SOL_PUSH = false;
          if (startEdge && io.AS_PUSH_RET) { stopReq = false; io.ST1_STEP = 10; }
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
        case FAULT:
          // Stuck: the part at the stopper was taken, or never arrived. Stop feeding and the
          // pusher, but LEAVE the stopper down - it is holding the queue back.
          io.CV1_RUN = false;
          io.EM1_EN = false;
          io.SOL_PUSH = false;
          if (startEdge) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }

      // TONs after the CASE, as in the ST: their Q is read by the NEXT scan.
      if (io.ST1_STEP === 20 && io.PE_STOP) { if (settleFrom < 0) settleFrom = t; } else settleFrom = -1;
      settleQ = settleFrom >= 0 && t - settleFrom >= 300;
      if (io.ST1_STEP === 70) { if (gapFrom < 0) gapFrom = t; } else gapFrom = -1;
      gapQ = gapFrom >= 0 && t - gapFrom >= 400;

      // Watchdog: a step that stops moving is a jam, not patience (CLAUDE.md).
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && t - stepFrom >= WD_MS) {
        io.ST1_STEP = FAULT; io.CV1_RUN = false; io.EM1_EN = false; io.SOL_PUSH = false;
      }

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT;
      io.PL_START = io.AUTO_RUN;
    },
  };
}
