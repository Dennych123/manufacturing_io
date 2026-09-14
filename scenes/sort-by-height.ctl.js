// INTERNAL CONTROLLER - not a PLC. The same sequence as sort-by-height.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the belt run. */
const WD_MS = 15000;
/** Fault step: feeders, belt and pusher off, AUTO_RUN off, waits for START to acknowledge. */
const FAULT = 900;

export function create() {
  let pbLast = false, stopReq = false, tallTurn = false, emTLast = 0, emSLast = 0, settleFrom = -1, settleQ = false;
  let stepLast = -1, stepFrom = 0, goneT = 0, goneS = 0;
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() { pbLast = false; stopReq = false; tallTurn = false; emTLast = 0; emSLast = 0; settleFrom = -1; settleQ = false; stepLast = -1; stepFrom = 0; goneT = 0; goneS = 0; },

    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_STOP) stopReq = true;

      // A write-off goes STALE when the part turns up after all: RM catches up, RM + gone runs
      // PAST EM, and the step waiting on that invariant stops waiting at all. Clamp both.
      if (io.RM_T_CNT + goneT > io.EM_T_CNT) goneT = Math.max(0, io.EM_T_CNT - io.RM_T_CNT);
      if (io.RM_S_CNT + goneS > io.EM_S_CNT) goneS = Math.max(0, io.EM_S_CNT - io.RM_S_CNT);

      switch (io.ST1_STEP) {
        case 0:
          io.CV_RUN = false; io.EM_T_EMIT = false; io.EM_S_EMIT = false; io.SOL_PUSH = false;
          if (startEdge && io.AS_PUSH_RET) { stopReq = false; emTLast = io.EM_T_CNT; emSLast = io.EM_S_CNT; io.ST1_STEP = 10; }
          break;
        case 10:
          if (tallTurn) {
            io.EM_T_EMIT = true;
            if (io.EM_T_CNT !== emTLast) { io.EM_T_EMIT = false; io.ST1_STEP = 15; }
          } else {
            io.EM_S_EMIT = true;
            if (io.EM_S_CNT !== emSLast) { io.EM_S_EMIT = false; io.ST1_STEP = 15; }
          }
          break;
        case 15:
          io.CV_RUN = true;
          if (!io.PE_LOW) io.ST1_STEP = 20;
          break;
        case 20:
          io.CV_RUN = true;
          if (io.PE_LOW) { io.CV_RUN = false; io.ST1_STEP = 30; }
          break;
        case 30:
          if (settleQ) io.ST1_STEP = 40;
          break;
        case 40:
          // the high beam only sees the tall part
          io.ST1_STEP = io.PE_HIGH ? 50 : 60;
          break;
        case 50:
          io.SOL_PUSH = true;
          if (io.AS_PUSH_EXT) io.ST1_STEP = 55;
          break;
        case 55:
          io.SOL_PUSH = false;
          if (io.AS_PUSH_RET && io.RM_T_CNT + goneT >= io.EM_T_CNT) io.ST1_STEP = 70;
          break;
        case 60:
          io.CV_RUN = true;
          if (io.RM_S_CNT + goneS >= io.EM_S_CNT) { io.CV_RUN = false; io.ST1_STEP = 70; }
          break;
        case 70:
          io.CYCLE_CNT += 1;
          tallTurn = !tallTurn;
          if (stopReq) io.ST1_STEP = 0; else { emTLast = io.EM_T_CNT; emSLast = io.EM_S_CNT; io.ST1_STEP = 10; }
          break;
        case FAULT:
          // Stuck: a part was taken off the belt or out of the chute, so one of the two invariants
          // below can never hold again. Acknowledging writes off what never arrived - only HERE,
          // never at every START, or a straggler's removal ends the next cycle (docs/PLAN.md §13).
          io.CV_RUN = false;
          io.EM_T_EMIT = false;
          io.EM_S_EMIT = false;
          io.SOL_PUSH = false;
          if (startEdge) {
            stopReq = false;
            goneT = io.EM_T_CNT - io.RM_T_CNT;
            goneS = io.EM_S_CNT - io.RM_S_CNT;
            io.ST1_STEP = 0;
          }
          break;
      }

      // TON after the CASE, as in the ST: its Q is read by the NEXT scan.
      if (io.ST1_STEP === 30) { if (settleFrom < 0) settleFrom = t; } else settleFrom = -1;
      settleQ = settleFrom >= 0 && t - settleFrom >= 300;

      // Watchdog: a step that stops moving is a jam, not patience (CLAUDE.md).
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && t - stepFrom >= WD_MS) {
        io.ST1_STEP = FAULT; io.CV_RUN = false; io.EM_T_EMIT = false; io.EM_S_EMIT = false; io.SOL_PUSH = false;
      }

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT;
      io.PL_START = io.AUTO_RUN;
    },
  };
}
