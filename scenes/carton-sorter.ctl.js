// INTERNAL CONTROLLER - not a PLC. The same sequence as carton-sorter.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.
//
// A carton sortation line, built on the Open Industry Project's own numbers (MIT, Automation
// Standard LLC): a 1.524 m belt at 2 m/s, cartons 600 x 400 x 400 at 10 kg fed 45 a minute, two
// swing-blade diverters whose 2.127 m blade sweeps 1504 mm across the belt at 45 degrees - the
// belt itself drives the carton along the blade and off the far side onto a take-away at 1 m/s.
//
//   infeed scan -> destination A, B or straight through, round robin
//   arming beam A -> if this carton is for A the blade swings out, else the carton is handed on
//   take-away beam A -> the carton is clear, the blade comes back
//   the same again at B; anything not diverted runs off the end of the line
//
// TRACKING is the whole job of this program, and it is a SHIFT REGISTER, not a timer: each
// diverter owns a FIFO of the destinations of the cartons between it and the scanner upstream.
// A carton arriving at a blade pops the queue it belongs to, and one that is not for this blade
// is pushed onto the next blade's queue. Timing the belt instead would mean a jam, a hand taking
// a carton off, or a speed override change silently sorting the wrong box - the queue moves only
// when a beam says a carton actually went past.

/** Destinations. THRU is "no read": it runs off the end of the line. */
const DEST = { A: 0, B: 1, THRU: 2 };
const NDEST = 3;
/** The FIFO between the scanner and a blade: a ring, as it is in the ST twin. */
const QMAX = 16;
const FAULT = 900, HOME = 800, ESTOP = 910;
/** A step that has not moved for this long is stuck. A carton crosses the whole line in ~7 s. */
const WD_MS = 20000;
/** From the blade going out to the carton clearing the take-away beam. */
const DIVERT_MS = 6000;
const TOGGLES = ['IND_CV', 'IND_FEED', 'IND_SPA', 'IND_SPB'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, stepLast = -1, stepFrom = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  let scanLast = false, upLast = [false, false], next = 0;
  /** One ring per blade: q[0] is what is between the scanner and blade A, q[1] between A and B. */
  let q = [[], []];
  let divFrom = [-1, -1], divQ = [false, false];
  let stLast = [-1, -1], stFrom = [0, 0];
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; stepLast = -1; stepFrom = 0;
      masterOn = false; homed = false; masterLast = false; homeLast = false;
      scanLast = false; upLast = [false, false]; next = 0; q = [[], []];
      divFrom = [-1, -1]; divQ = [false, false];
      stLast = [-1, -1]; stFrom = [0, 0];
      for (const b of TOGGLES) { indMem[b] = false; indLast[b] = false; }
    },

    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_CSTOP) stopReq = true;
      const auto = io.SEL_AUTO !== false, selChanged = auto !== selLast;
      selLast = auto;
      const estop = !!io.PB_ESTOP;
      const masterEdge = io.PB_MASTER && !masterLast;
      masterLast = !!io.PB_MASTER;
      const homeEdge = io.PB_HOME && !homeLast;
      homeLast = !!io.PB_HOME;
      if (estop) { masterOn = false; homed = false; }
      else if (masterEdge) masterOn = true;
      const ready = masterOn && !estop;

      const D = k => (k === 0 ? 'A' : 'B');
      /** Swing a blade: OUT is toward the belt, which is -45 for A and +45 for B. */
      const blade = (k, deg) => { io['DV' + D(k) + '_TGT'] = deg; io['DV' + D(k) + '_EXEC'] = true; return !!io['DV' + D(k) + '_DONE']; };
      const bladeDrop = k => { io['DV' + D(k) + '_EXEC'] = false; return !io['DV' + D(k) + '_DONE']; };
      const OUT = [-45, 45];
      const allOff = () => {
        io.DVA_EXEC = false; io.DVB_EXEC = false;
        io.CV_RUN = false; io.SPA_RUN = false; io.SPB_RUN = false; io.EM_EN = false;
      };

      switch (io.ST1_STEP) {
        case 0:
          allOff();
          q = [[], []]; next = 0;
          if (startEdge && auto && ready && homed) { stopReq = false; io.ST2_STEP = 100; io.ST3_STEP = 100; io.ST1_STEP = 10; }
          break;
        case 10:
          // The line runs and the scanner reads every carton that goes by. The destination is round
          // robin here; on a real sorter it is whatever the barcode says.
          io.CV_RUN = true; io.SPA_RUN = true; io.SPB_RUN = true; io.EM_EN = true;
          if (io.PE_IN && !scanLast) {
            if (q[0].length >= QMAX) { io.ST1_STEP = FAULT; break; }   // the line has run away from the sorter
            q[0].push(next);
            next = (next + 1) % NDEST;
            io.CYCLE_CNT += 1;
          }
          if (stopReq && !q[0].length && !q[1].length) { io.EM_EN = false; io.ST1_STEP = 0; }
          break;
        case HOME:
          // HOME on a sorter is the blades parked along the line, out of the path.
          allOff();
          { const a = blade(0, 0), b = blade(1, 0); if (a && b) { bladeDrop(0); bladeDrop(1); homed = true; io.ST1_STEP = 0; } }
          break;
        case ESTOP:
          allOff();
          if (ready) io.ST1_STEP = 0;
          break;
        case FAULT:
          allOff();
          if (startEdge && auto) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }
      scanLast = !!io.PE_IN;

      const running = io.ST1_STEP === 10;

      // ---------------------------------------------------------- ST2: diverter A
      // The two diverters run at the same time as each other and as the line, so they are their
      // own state machines (CLAUDE.md). Written out per diverter rather than as a loop, so the
      // CASE labels here and in the .st twin line up one for one.
      if (!running) io.ST2_STEP = 0;
      else switch (io.ST2_STEP) {
        case 100:
          // Waiting for a carton. The queue tells us what the next one is for, and it is popped on
          // the beam's EDGE: one carton past the beam is one carton out of the queue.
          if (io.PE_UP_A && !upLast[0]) {
            const d = q[0].length ? q[0].shift() : DEST.THRU;
            if (d === DEST.A) io.ST2_STEP = 110;
            else { q[1].push(d); io.ST2_STEP = 130; }
          }
          break;
        case 110: if (blade(0, OUT[0])) io.ST2_STEP = 111; break;
        case 111: if (bladeDrop(0)) io.ST2_STEP = 112; break;
        case 112:
          // The carton is on its way along the blade. It is CLEAR when the take-away beam has seen
          // it and let go again: a beam that is still made means the carton is still on the blade.
          if (io.PE_A) io.ST2_STEP = 113;
          else if (divQ[0]) io.ST2_STEP = FAULT;                       // it never arrived
          break;
        case 113: if (!io.PE_A) io.ST2_STEP = 120; break;
        case 120: if (blade(0, 0)) io.ST2_STEP = 121; break;
        case 121: if (bladeDrop(0)) io.ST2_STEP = 100; break;
        case 130: if (!io.PE_UP_A) io.ST2_STEP = 100; break;           // not for A: let it run on
        default: io.ST2_STEP = 100;
      }

      // ---------------------------------------------------------- ST3: diverter B
      if (!running) io.ST3_STEP = 0;
      else switch (io.ST3_STEP) {
        case 100:
          if (io.PE_UP_B && !upLast[1]) {
            const d = q[1].length ? q[1].shift() : DEST.THRU;
            io.ST3_STEP = d === DEST.B ? 110 : 130;
          }
          break;
        case 110: if (blade(1, OUT[1])) io.ST3_STEP = 111; break;
        case 111: if (bladeDrop(1)) io.ST3_STEP = 112; break;
        case 112:
          if (io.PE_B) io.ST3_STEP = 113;
          else if (divQ[1]) io.ST3_STEP = FAULT;
          break;
        case 113: if (!io.PE_B) io.ST3_STEP = 120; break;
        case 120: if (blade(1, 0)) io.ST3_STEP = 121; break;
        case 121: if (bladeDrop(1)) io.ST3_STEP = 100; break;
        case 130: if (!io.PE_UP_B) io.ST3_STEP = 100; break;
        default: io.ST3_STEP = 100;
      }
      upLast = [!!io.PE_UP_A, !!io.PE_UP_B];

      // TONs after the CASE, as in the ST: their Q is read by the NEXT scan.
      for (const k of [0, 1]) {
        const step = k === 0 ? io.ST2_STEP : io.ST3_STEP;
        if (step === 112) { if (divFrom[k] < 0) divFrom[k] = t; } else divFrom[k] = -1;
        divQ[k] = divFrom[k] >= 0 && t - divFrom[k] >= DIVERT_MS;
        // A diverter step that stops moving is a jam. Its waiting step (100) is not: on a quiet
        // line the next carton is as far away as the feed rate says.
        if (step !== stLast[k]) { stLast[k] = step; stFrom[k] = t; }
        if (running && step !== 100 && t - stFrom[k] >= WD_MS) { io.ST1_STEP = FAULT; allOff(); }
        if ((k === 0 ? io.ST2_STEP : io.ST3_STEP) === FAULT) { io.ST1_STEP = FAULT; allOff(); }
      }

      // Watchdog on the line itself, and the selector rule: turning it while running is a FAULT.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (running && selChanged) { io.ST1_STEP = FAULT; allOff(); }

      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        allOff();
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      for (const k of [0, 1]) {
        io['DV' + D(k) + '_JOG_P'] = individual && !!io['IND_DV' + D(k) + '_P'];
        io['DV' + D(k) + '_JOG_N'] = individual && !!io['IND_DV' + D(k) + '_N'];
      }
      if (individual) {
        io.CV_RUN = !!indMem.IND_CV;
        io.EM_EN = !!indMem.IND_FEED;
        io.SPA_RUN = !!indMem.IND_SPA;
        io.SPB_RUN = !!indMem.IND_SPB;
      }
      io.OVR = io.OVR_SET;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
