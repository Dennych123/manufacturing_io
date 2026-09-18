// INTERNAL CONTROLLER - not a PLC. The same sequence as mps-sorting.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.
//
// Festo Didactic's MPS Sorting station 8046325 (manual 8046391, 11/2015), sequence and IO taken
// from the manual: a 40 mm workpiece is laid on the belt, a fork light barrier sees it, the belt
// runs it up to a stop, and three sensors at the stop say what it is:
//
//   through-beam        every workpiece            (I4 "Werkstück erkannt")
//   retro-reflective    every one that is NOT matt black   (I5 "Werkstück nicht schwarz")
//   inductive           the metallic one           (I6 "Werkstück metallisch")
//
// so black = seen but not by the retro one, red = retro but not inductive, metallic = both. The
// station then sorts onto three chutes, exactly as the manual's Ablaufbeschreibung has it:
// red to the chute at the BELT START (ejector 1), metallic to the one in the MIDDLE (ejector 2),
// black to the chute at the BELT END - it is simply let go and runs off the end.
//
// ONE difference from the real machine, and it is deliberate. Festo's Q3 is "Stopper einfahren":
// the stop is out by default and the output RETRACTS it. Here the stop pops UP out of the belt
// (`STOP_HOLD`, energised to hold), because a stop a part has to pass UNDER holds it from a
// positive distance in this simulator and no clearance tunes that away (CLAUDE.md). The logic is
// the same signal inverted.
//
// The feed is the operator laying a workpiece on the belt: the program asks for one colour at a
// time, round robin, so every run sorts all three.

/** What the sensors at the stop say, as the manual reads them. */
const BLACK = 0, RED = 1, METAL = 2;
const FAULT = 900, HOME = 800, ESTOP = 910;
/** A step that has not moved for this long is stuck. The slowest normal step is the 7 s run-out. */
const WD_MS = 20000;
/** The workpiece is held against the stop before the sensors are read (they are 40 ms off-delayed). */
const READ_MS = 400;
/** Belt off to the next workpiece being laid on. */
const FEED_MS = 600;
const TOGGLES = ['IND_BELT', 'IND_STOP', 'IND_W1', 'IND_W2'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, stepLast = -1, stepFrom = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  let kind = BLACK, nextFeed = 0;
  let readFrom = -1, readQ = false, feedFrom = -1, feedQ = false;
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; stepLast = -1; stepFrom = 0;
      masterOn = false; homed = false; masterLast = false; homeLast = false;
      kind = BLACK; nextFeed = 0;
      readFrom = -1; readQ = false; feedFrom = -1; feedQ = false;
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

      const allOff = () => {
        io.BELT_FWD = false; io.W1_EXT = false; io.W2_EXT = false;
        io.EM_BLACK = false; io.EM_RED = false; io.EM_METAL = false;
      };
      /** Everything laid on the belt has reached a bin: the invariant a discharge step waits for. */
      const clear = () => (io.EM_BLACK_CNT + io.EM_RED_CNT + io.EM_METAL_CNT) === (io.RM_1_CNT + io.RM_2_CNT + io.RM_3_CNT);

      switch (io.ST1_STEP) {
        case 0:
          // Ausgangsstellung: belt off, stop out, both ejectors in.
          allOff();
          io.STOP_HOLD = true;
          if (startEdge && auto && ready && homed) { stopReq = false; io.ST1_STEP = 10; }
          break;

        case 10:
          // The operator lays the next workpiece on the belt. One colour at a time, round robin.
          io.BELT_FWD = false;
          io.STOP_HOLD = true;
          io.EM_BLACK = nextFeed === BLACK; io.EM_RED = nextFeed === RED; io.EM_METAL = nextFeed === METAL;
          if (feedQ) { nextFeed = (nextFeed + 1) % 3; io.ST1_STEP = 15; }
          break;
        case 15:
          allOff();
          io.STOP_HOLD = true;
          // Startvoraussetzung: Werkstück am Bandanfang.
          if (io.WP_AT_START && !io.CHUTE_FULL) io.ST1_STEP = 20;
          break;
        case 20:
          // Bandmotor ein: the belt carries it up to the stop.
          io.BELT_FWD = true;
          io.STOP_HOLD = true;
          if (io.WP_DETECTED) io.ST1_STEP = 30;
          break;
        case 30:
          // Farb-/Materialidentifikation, with the workpiece held against the stop.
          io.BELT_FWD = true;
          if (readQ) {
            kind = io.WP_METALLIC ? METAL : io.WP_NOT_BLACK ? RED : BLACK;
            io.ST1_STEP = kind === RED ? 40 : kind === METAL ? 50 : 60;
          }
          break;

        // ---- red: ejector 1, the chute at the belt start. The belt keeps running through the
        // discharge: it is what carries the workpiece to the ejector and off the end.
        case 40: io.BELT_FWD = true; io.W1_EXT = true; if (io.W1_OUT) io.ST1_STEP = 41; break;
        case 41: io.BELT_FWD = true; io.STOP_HOLD = false; if (io.AS_STOP_FREE) io.ST1_STEP = 42; break;
        case 42:
          // Werkstück ausgeschleust: not "a counter moved" but the invariant that everything laid
          // on the belt has reached a bin (CLAUDE.md). It is self-healing after a fault.
          io.BELT_FWD = true;
          if (clear()) io.ST1_STEP = 43;
          break;
        case 43: io.BELT_FWD = true; io.W1_EXT = false; if (io.AS_W1_IN) io.ST1_STEP = 70; break;

        // ---- metallic: ejector 2, the chute in the middle
        case 50: io.BELT_FWD = true; io.W2_EXT = true; if (io.W2_OUT) io.ST1_STEP = 51; break;
        case 51: io.BELT_FWD = true; io.STOP_HOLD = false; if (io.AS_STOP_FREE) io.ST1_STEP = 52; break;
        case 52: io.BELT_FWD = true; if (clear()) io.ST1_STEP = 53; break;
        case 53: io.BELT_FWD = true; io.W2_EXT = false; if (io.AS_W2_IN) io.ST1_STEP = 70; break;

        // ---- black: let it go, it runs off the belt end onto the third chute
        case 60: io.BELT_FWD = true; io.STOP_HOLD = false; if (io.AS_STOP_FREE) io.ST1_STEP = 61; break;
        case 61: io.BELT_FWD = true; if (clear()) io.ST1_STEP = 70; break;

        case 70:
          // Bandmotor aus, Sperre ausfahren.
          io.BELT_FWD = false;
          io.STOP_HOLD = true;
          if (io.AS_STOP_HOLD) { io.CYCLE_CNT += 1; io.ST1_STEP = stopReq ? 0 : 10; }
          break;

        case HOME:
          // The station's home is its Ausgangsstellung: belt off, stop out, ejectors in.
          allOff();
          io.STOP_HOLD = true;
          if (io.AS_STOP_HOLD && io.AS_W1_IN && io.AS_W2_IN) { homed = true; io.ST1_STEP = 0; }
          break;
        case ESTOP:
          // The master circuit is open. The stop KEEPS the workpiece it is holding.
          allOff();
          if (ready) io.ST1_STEP = 0;
          break;
        case FAULT:
          allOff();
          if (startEdge && auto) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }

      // TONs after the CASE, as in the ST: their Q is read by the NEXT scan.
      if (io.ST1_STEP === 30) { if (readFrom < 0) readFrom = t; } else readFrom = -1;
      readQ = readFrom >= 0 && t - readFrom >= READ_MS;
      if (io.ST1_STEP === 10) { if (feedFrom < 0) feedFrom = t; } else feedFrom = -1;
      feedQ = feedFrom >= 0 && t - feedFrom >= FEED_MS;

      // Watchdog: a running step that stops moving is a jam, not patience. A workpiece taken off
      // the belt by hand leaves step 20 waiting for a stop it will never reach.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP && (t - stepFrom >= WD_MS || selChanged)) { io.ST1_STEP = FAULT; allOff(); }

      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        allOff();
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      if (individual) {
        io.BELT_FWD = !!indMem.IND_BELT;
        io.STOP_HOLD = !!indMem.IND_STOP;
        io.W1_EXT = !!indMem.IND_W1;
        io.W2_EXT = !!indMem.IND_W2;
        // The individual feed buttons lay one workpiece of that colour on the belt.
        io.EM_BLACK = !!io.IND_FEED_BLACK;
        io.EM_RED = !!io.IND_FEED_RED;
        io.EM_METAL = !!io.IND_FEED_METAL;
      }
      io.OVR = io.OVR_SET;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
