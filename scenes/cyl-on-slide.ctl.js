// INTERNAL CONTROLLER - not a PLC. The same sequence as cyl-on-slide.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

export function create() {
  let pbLast = false, stopReq = false, dwellFrom = -1, dwellQ = false;
  return {
    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_STOP) stopReq = true;

      switch (io.ST1_STEP) {
        case 0:
          io.SOL_ST1_PRSS_CYL_DN = false;
          io.SOL_ST1_PRSS_CYL_UP = true;
          io.SV1_EXEC = false;
          if (startEdge && io.AS_ST1_PRSS_CYL_UP && io.SV1_INPOS && !io.SV1_DONE) { stopReq = false; io.ST1_STEP = 10; }
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
      }

      // TON after the CASE, as in the ST: its Q is read by the NEXT scan.
      if (io.ST1_STEP === 40) { if (dwellFrom < 0) dwellFrom = t; } else dwellFrom = -1;
      dwellQ = dwellFrom >= 0 && t - dwellFrom >= 300;
      io.AUTO_RUN = io.ST1_STEP !== 0;
      io.PL_START = io.AUTO_RUN;
    },
  };
}
