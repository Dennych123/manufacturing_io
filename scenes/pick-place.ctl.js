// INTERNAL CONTROLLER - not a PLC. The same sequence as pick-place.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the ~4 s discharge. */
const WD_MS = 15000;
/** Fault step: feeding and motion off, AUTO_RUN off, waits for START to be acknowledged. */
const FAULT = 900;

export function create() {
  let pbLast = false, stopReq = false, emLast = 0;
  let stepLast = -1, stepFrom = 0, gone = 0;
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() { pbLast = false; stopReq = false; emLast = 0; stepLast = -1; stepFrom = 0; gone = 0; },

    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_STOP) stopReq = true;

      // A write-off goes STALE when the part turns up after all: RM catches up, RM + gone runs
      // PAST EM, and step 130 stops waiting at all. Clamp it every scan.
      if (io.RM_CNT + gone > io.EM_CNT) gone = Math.max(0, io.EM_CNT - io.RM_CNT);

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
          // the invariant, not "a count moved": every part loaded has left, counting write-offs
          if (io.RM_CNT + gone >= io.EM_CNT) {
            io.CYCLE_CNT += 1;
            if (stopReq) io.ST1_STEP = 0; else { emLast = io.EM_CNT; io.ST1_STEP = 10; }
          }
          break;
        case FAULT:
          // Stuck: the part was taken out of the nest or the cup, or never arrived. Stop feeding
          // and stop the servo, but do NOT drop vacuum - a real machine keeps hold of its part.
          io.EM_EMIT = false;
          io.CV_RUN = false;
          io.SV_EXEC = false;
          // Acknowledging writes off whatever never reached the unloader. Only HERE: at every
          // START it would write off parts still legitimately in the machine, and an older part's
          // removal would end this cycle (the pipelining bug, docs/PLAN.md §13).
          if (startEdge) { stopReq = false; gone = io.EM_CNT - io.RM_CNT; io.ST1_STEP = 0; }
          break;
      }

      // Watchdog: a step that stops moving is a jam, not patience. Measured before this existed:
      // taking the part out of the nest left ST1_STEP at 20 for as long as you care to watch,
      // with AUTO_RUN still on.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && t - stepFrom >= WD_MS) {
        io.ST1_STEP = FAULT; io.EM_EMIT = false; io.CV_RUN = false; io.SV_EXEC = false;
      }

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT;
      io.PL_START = io.AUTO_RUN;
    },
  };
}
