// INTERNAL CONTROLLER - not a PLC. The same sequence as assembler.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

export function create() {
  let pbLast = false, stopReq = false, emBLast = 0, emLLast = 0, nextSt = 0, pressFrom = -1, pressQ = false;
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() { pbLast = false; stopReq = false; emBLast = 0; emLLast = 0; nextSt = 0; pressFrom = -1; pressQ = false; },

    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_STOP) stopReq = true;

      switch (io.ST1_STEP) {
        case 0:
          io.TBL_RUN = false; io.EM_B_EMIT = false; io.EM_L_EMIT = false;
          io.SOL_PRESS_DN = false; io.SOL_PRESS_UP = true;
          if (startEdge && io.TBL_INPOS && io.AS_PRESS_UP) { stopReq = false; emBLast = io.EM_B_CNT; io.ST1_STEP = 10; }
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
      }

      // TON after the CASE, as in the ST: its Q is read by the NEXT scan.
      if (io.ST1_STEP === 40) { if (pressFrom < 0) pressFrom = t; } else pressFrom = -1;
      pressQ = pressFrom >= 0 && t - pressFrom >= 300;
      io.AUTO_RUN = io.ST1_STEP !== 0;
      io.PL_START = io.AUTO_RUN;
    },
  };
}
