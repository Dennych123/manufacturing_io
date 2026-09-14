// INTERNAL CONTROLLER - not a PLC. The same sequence as pick-place.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

export function create() {
  let pbLast = false, stopReq = false, emLast = 0;
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() { pbLast = false; stopReq = false; emLast = 0; },

    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_STOP) stopReq = true;

      switch (io.ST1_STEP) {
        case 0:
          io.CV_RUN = false; io.EM_EMIT = false; io.VAC_ON = false;
          io.SOL_Z_DN = false; io.SOL_Z_UP = true; io.SV_EXEC = false;
          if (startEdge && io.AS_Z_UP && io.SV_INPOS && !io.SV_DONE) { stopReq = false; emLast = io.EM_CNT; io.ST1_STEP = 10; }
          break;
        case 10:
          io.CV_RUN = true; io.CLAMP_A = true; io.EM_EMIT = true;
          if (io.EM_CNT !== emLast) { io.EM_EMIT = false; io.ST1_STEP = 20; }
          break;
        case 20:
          if (io.NEST_A_P) io.ST1_STEP = 30;
          break;
        case 30:
          io.SOL_Z_UP = false; io.SOL_Z_DN = true;
          if (io.AS_Z_DN) io.ST1_STEP = 40;
          break;
        case 40:
          io.CLAMP_A = false; io.VAC_ON = true;
          if (io.VAC_SW) io.ST1_STEP = 50;
          break;
        case 50:
          io.SOL_Z_DN = false; io.SOL_Z_UP = true;
          if (io.AS_Z_UP) io.ST1_STEP = 60;
          break;
        case 60:
          io.SV_TGT = 600;
          io.SV_EXEC = true;
          if (io.SV_DONE) { io.SV_EXEC = false; io.ST1_STEP = 70; }
          break;
        case 70:
          if (!io.SV_DONE) io.ST1_STEP = 80;
          break;
        case 80:
          io.SOL_Z_UP = false; io.SOL_Z_DN = true;
          if (io.AS_Z_DN) io.ST1_STEP = 90;
          break;
        case 90:
          io.VAC_ON = false;
          if (!io.VAC_SW) io.ST1_STEP = 100;
          break;
        case 100:
          io.SOL_Z_DN = false; io.SOL_Z_UP = true;
          if (io.AS_Z_UP) io.ST1_STEP = 110;
          break;
        case 110:
          io.SV_TGT = 0;
          io.SV_EXEC = true;
          if (io.SV_DONE) { io.SV_EXEC = false; io.ST1_STEP = 120; }
          break;
        case 120:
          if (!io.SV_DONE) io.ST1_STEP = 130;
          break;
        case 130:
          // the invariant, not "a count moved": every part loaded has left
          if (io.RM_CNT === io.EM_CNT) {
            io.CYCLE_CNT += 1;
            if (stopReq) io.ST1_STEP = 0; else { emLast = io.EM_CNT; io.ST1_STEP = 10; }
          }
          break;
      }

      io.AUTO_RUN = io.ST1_STEP !== 0;
      io.PL_START = io.AUTO_RUN;
    },
  };
}
