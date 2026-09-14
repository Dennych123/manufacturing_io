// INTERNAL CONTROLLER - not a PLC. The same sequence as a-to-b.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

export function create() {
  let pbLast = false, stopReq = false, dwellFrom = -1, dwellQ = false, emLast = 0, rmLast = 0;
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() { pbLast = false; stopReq = false; dwellFrom = -1; dwellQ = false; emLast = 0; rmLast = 0; },
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
          // snapshot the unloader's count on ENTERING the wait, not when it moves
          if (dwellQ) { rmLast = io.RM1_CNT; io.ST1_STEP = 40; }
          break;
        case 40:
          io.CV1_RUN = true;
          if (io.RM1_CNT !== rmLast) { io.CV1_RUN = false; io.ST1_STEP = 50; }
          break;
        case 50:
          io.CYCLE_CNT += 1;
          if (stopReq) io.ST1_STEP = 0; else { emLast = io.EM1_CNT; io.ST1_STEP = 10; }
          break;
      }

      // TON after the CASE, as in the ST: its Q is read by the NEXT scan.
      if (io.ST1_STEP === 30) { if (dwellFrom < 0) dwellFrom = t; } else dwellFrom = -1;
      dwellQ = dwellFrom >= 0 && t - dwellFrom >= 500;
      io.AUTO_RUN = io.ST1_STEP !== 0;
      io.PL_START = io.AUTO_RUN;
    },
  };
}
