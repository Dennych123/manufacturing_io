// INTERNAL CONTROLLER - not a PLC. The same sequence as palletizing.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.
//
// THREE stations that run at the same time, which is what a cell does and what one long sequence
// could not: the gantry goes back for the next five as soon as it has put its five down, while
// the carrier indexes and the unloader empties a jig on the far side. They meet at the carrier:
// `full[i]` says which jig has parts in it, and neither head lets the table turn under it.

/** A step that has not moved for this long is stuck: the longest normal step is the ~2.4 s index. */
const WD_MS = 15000;
/** Home step: every actuator is driven to its home position. AUTO will not start until it ends. */
const HOME = 800;
/** Fault step: motion safe, AUTO_RUN off, waits for START to be acknowledged. */
const FAULT = 900;
/** E-STOP step: the master circuit is open and every solenoid is de-energised. */
const ESTOP = 910;
/** Individual buttons that toggle an actuator (the button is momentary, the memory is the PLC's). */
const TOGGLES = ['IND_VAC', 'IND_LIFT', 'IND_PAL', 'IND_JIG', 'IND_UVAC', 'IND_ULIFT', 'IND_USHIFT'];

/** Gantry positions, mm on each axis (measured from the scene: cup 3 sits at X = GX - 300, Y = GY - 600). */
const GX_LEFT = 150, GX_RIGHT = 450, GX_JIG = 1268, GY_JIG = 600;
/** Pallet row 0 is at GY 330 and the rows are 60 apart; row 9 is the last. */
const GY_ROW0 = 330, GY_ROWN = 870, ROW_PITCH = 60;
/** The loader has stopped delivering, so the tray is full: it drops one every 40 ms when it can. */
const FILL_SETTLE = 600;
/** How long a head is given to get hold of what is under it before it carries on with what it has. */
const PICK_MS = 600;
/** Stations on the carrier. */
const JIGS = 4;

export function create() {
  let pbLast = false, stopReq = false, selLast = true;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  let half = false, rowY = GY_ROW0, picked = false;
  let needFill = true, palBase = 0, fillFrom = 0, pickFrom = -1, uPickFrom = -1, nextSt = 0;
  /** Which jig has parts in it. The gantry sets one, the unloader clears one. */
  const full = new Array(JIGS).fill(false);
  const wd = { ST1_STEP: [-1, 0], ST2_STEP: [-1, 0], ST3_STEP: [-1, 0] };
  const indMem = {}, indLast = {};

  const anyVac = io => !!(io.VAC_SW1 || io.VAC_SW2 || io.VAC_SW3 || io.VAC_SW4 || io.VAC_SW5);
  const noVac = io => !(io.VAC_SW1 || io.VAC_SW2 || io.VAC_SW3 || io.VAC_SW4 || io.VAC_SW5);
  const noUVac = io => !(io.UVAC_SW1 || io.UVAC_SW2 || io.UVAC_SW3 || io.UVAC_SW4 || io.UVAC_SW5);
  /** The jig standing at the load station, and the one at the unload station. */
  const loadIdx = io => (2 - (io.TBL_STATION | 0) + JIGS) % JIGS;
  const unloadIdx = io => ((JIGS - (io.TBL_STATION | 0)) % JIGS);

  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true;
      masterOn = false; homed = false; masterLast = false; homeLast = false;
      half = false; rowY = GY_ROW0; picked = false;
      needFill = true; palBase = 0; fillFrom = 0; pickFrom = -1; uPickFrom = -1; nextSt = 0;
      full.fill(false);
      for (const k of Object.keys(wd)) wd[k] = [-1, 0];
      for (const b of TOGGLES) { indMem[b] = false; indLast[b] = false; }
    },

    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_CSTOP) stopReq = true;
      // Selector AUTO / INDIVIDUAL: START only in AUTO, individual buttons only in INDIVIDUAL, a
      // change while running is a FAULT.
      const auto = io.SEL_AUTO !== false, selChanged = auto !== selLast;
      selLast = auto;
      // The master circuit: E-STOP is a latching mushroom, MASTER ON energises the machine, and an
      // E-STOP also loses the home position.
      const estop = !!io.PB_ESTOP;
      const masterEdge = io.PB_MASTER && !masterLast;
      masterLast = !!io.PB_MASTER;
      const homeEdge = io.PB_HOME && !homeLast;
      homeLast = !!io.PB_HOME;
      if (estop) { masterOn = false; homed = false; }
      else if (masterEdge) masterOn = true;
      const ready = masterOn && !estop;
      // Neither head lets the carrier turn while it is over a jig.
      const loadBusy = io.ST1_STEP >= 57 && io.ST1_STEP <= 80;
      const unloadBusy = io.ST3_STEP >= 110 && io.ST3_STEP <= 130;

      // ---------------------------------------------------------- ST1: the depalletiser
      switch (io.ST1_STEP) {
        case 0:
          io.VAC_ON = false; io.GX_EXEC = false; io.GY_EXEC = false;
          io.SOL_Z_DN = false; io.SOL_Z_UP = true;
          io.EM_PAL_EN = false;
          if (startEdge && auto && ready && homed && io.AS_Z_UP && io.AS_U_UP) {
            stopReq = false;
            io.ST2_STEP = 10; io.ST3_STEP = 100;
            if (needFill) { palBase = io.EM_PAL_CNT; fillFrom = t; io.ST1_STEP = 5; } else io.ST1_STEP = 10;
          }
          break;
        case 5:
          // A fresh pallet. The tray is FULL when the loader stops delivering, not after a count
          // of 100: a hole that still had a plug in it would make a count wait for ever.
          io.PAL_CLAMP = true; io.EM_PAL_EN = true;
          if (io.EM_PAL_CNT !== palBase) { palBase = io.EM_PAL_CNT; fillFrom = t; }
          else if (t - fillFrom >= FILL_SETTLE) { io.EM_PAL_EN = false; needFill = false; io.ST1_STEP = 10; }
          break;
        case 10:
          io.PAL_CLAMP = true; io.JIG_CLAMP = true;
          io.GX_TGT = half ? GX_RIGHT : GX_LEFT;
          io.GY_TGT = rowY;
          io.GX_EXEC = true; io.GY_EXEC = true;
          if (io.GX_DONE && io.GY_DONE) { io.GX_EXEC = false; io.GY_EXEC = false; io.ST1_STEP = 15; }
          break;
        case 15:
          if (!io.GX_DONE && !io.GY_DONE) io.ST1_STEP = 20;
          break;
        case 20:
          io.SOL_Z_UP = false; io.SOL_Z_DN = true;
          if (io.AS_Z_DN) io.ST1_STEP = 30;
          break;
        case 30:
          // The tray lets go while the cups take what is there. A hole may be empty - a plug was
          // taken out, or the pallet is finished - and the head carries on with what it got
          // rather than standing here waiting for a plug that is not coming.
          io.PAL_CLAMP = false; io.VAC_ON = true;
          if (pickFrom < 0) pickFrom = t;
          if (t - pickFrom >= PICK_MS) { picked = anyVac(io); pickFrom = -1; io.ST1_STEP = 40; }
          break;
        case 40:
          io.PAL_CLAMP = true;                       // the rest of the pallet is held again
          io.SOL_Z_DN = false; io.SOL_Z_UP = true;
          if (io.AS_Z_UP) {
            // Nothing at all came up: the tray is empty, so call for a fresh pallet.
            if (!picked) { needFill = true; palBase = io.EM_PAL_CNT; fillFrom = t; io.ST1_STEP = 5; }
            else io.ST1_STEP = 50;
          }
          break;
        case 50:
          io.GX_TGT = GX_JIG; io.GY_TGT = GY_JIG;
          io.GX_EXEC = true; io.GY_EXEC = true;
          if (io.GX_DONE && io.GY_DONE) { io.GX_EXEC = false; io.GY_EXEC = false; io.ST1_STEP = 55; }
          break;
        case 55:
          if (!io.GX_DONE && !io.GY_DONE) io.ST1_STEP = 57;
          break;
        case 57:
          // The load station must be standing still with an empty jig under it.
          if (io.TBL_INPOS && !full[loadIdx(io)]) io.ST1_STEP = 60;
          break;
        case 60:
          io.SOL_Z_UP = false; io.SOL_Z_DN = true;
          if (io.AS_Z_DN) io.ST1_STEP = 70;
          break;
        case 70:
          io.VAC_ON = false;
          if (noVac(io)) io.ST1_STEP = 80;
          break;
        case 80:
          io.SOL_Z_DN = false; io.SOL_Z_UP = true;
          if (io.AS_Z_UP) {
            full[loadIdx(io)] = true;                // the carrier may take this jig away now
            io.CYCLE_CNT += 1;
            if (half) {
              half = false;
              if (rowY >= GY_ROWN) { rowY = GY_ROW0; needFill = true; } else rowY += ROW_PITCH;
            } else half = true;
            if (stopReq) io.ST1_STEP = 0;
            else if (needFill) { palBase = io.EM_PAL_CNT; fillFrom = t; io.ST1_STEP = 5; }
            else io.ST1_STEP = 10;
          }
          break;
        case HOME:
          // Heads up, the unload shift over the jig, both gantry axes to zero and the carrier
          // turned to its origin. The vacuum is left alone: a head keeps the plugs it is holding.
          io.TBL_RUN = !io.TBL_ORIGIN;
          io.SOL_Z_DN = false; io.SOL_Z_UP = true;
          io.SOL_U_DN = false; io.SOL_U_UP = true;
          io.SOL_U_BIN = false; io.SOL_U_JIG = true;
          io.GX_TGT = 0; io.GY_TGT = 0; io.GX_EXEC = true; io.GY_EXEC = true;
          if (io.AS_Z_UP && io.AS_U_UP && io.AS_U_JIG && io.GX_DONE && io.GY_DONE && io.TBL_ORIGIN) {
            io.GX_EXEC = false; io.GY_EXEC = false; io.TBL_RUN = false;
            homed = true; io.ST1_STEP = 0;
          }
          break;
        case ESTOP:
          // The master circuit is open, so every solenoid de-energises - which is what really
          // happens: a 5/2 single-solenoid valve springs back, a double one holds where it is.
          // The trays and the cups KEEP their parts: a hundred plugs let loose in their pockets
          // is not what a power cut does, and it costs the plant four times the work.
          io.TBL_RUN = false; io.GX_EXEC = false; io.GY_EXEC = false; io.EM_PAL_EN = false;
          io.SOL_Z_DN = false; io.SOL_Z_UP = false; io.SOL_U_DN = false; io.SOL_U_UP = false;
          io.SOL_U_BIN = false; io.SOL_U_JIG = false;
          io.PAL_CLAMP = true; io.JIG_CLAMP = true;
          if (ready) io.ST1_STEP = 0;                // MASTER ON after the mushroom is released
          break;
        case FAULT:
          // Stuck, or the selector was turned while running. Motion and feeding off; the trays and
          // the cups keep whatever they hold.
          io.TBL_RUN = false; io.GX_EXEC = false; io.GY_EXEC = false; io.EM_PAL_EN = false;
          io.PAL_CLAMP = true; io.JIG_CLAMP = true;
          if (startEdge && auto) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }

      const running = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;

      // ---------------------------------------------------------- ST2: the jig carrier
      if (!running) { io.ST2_STEP = 0; io.TBL_RUN = false; }
      else switch (io.ST2_STEP) {
        case 10:
          // Turn when there is a full jig to move on AND an empty one to bring to the unloader,
          // and never while a head is over the table.
          io.TBL_RUN = false;
          if (full[loadIdx(io)] && !full[unloadIdx(io)] && !loadBusy && !unloadBusy && io.TBL_INPOS) {
            nextSt = ((io.TBL_STATION | 0) + 1) % JIGS;
            io.ST2_STEP = 20;
          }
          break;
        case 20:
          // the invariant: this index has landed on the station it was sent to
          io.TBL_RUN = true;
          if (io.TBL_INPOS && io.TBL_STATION === nextSt) { io.TBL_RUN = false; io.ST2_STEP = 10; }
          break;
        default:
          io.ST2_STEP = 10;
      }

      // ---------------------------------------------------------- ST3: the unloader
      if (!running) { io.ST3_STEP = 0; }
      else switch (io.ST3_STEP) {
        case 100:
          if (full[unloadIdx(io)] && io.TBL_INPOS) io.ST3_STEP = 110;
          break;
        case 110:
          io.SOL_U_BIN = false; io.SOL_U_JIG = true;
          if (io.AS_U_JIG) io.ST3_STEP = 115;
          break;
        case 115:
          io.SOL_U_UP = false; io.SOL_U_DN = true;
          if (io.AS_U_DN) io.ST3_STEP = 120;
          break;
        case 120:
          // Whatever is in the jig, however many: a gap in the jig is not a reason to stop.
          io.JIG_CLAMP = false; io.UVAC_ON = true;
          if (uPickFrom < 0) uPickFrom = t;
          if (t - uPickFrom >= PICK_MS) { uPickFrom = -1; io.ST3_STEP = 130; }
          break;
        case 130:
          io.JIG_CLAMP = true;
          io.SOL_U_DN = false; io.SOL_U_UP = true;
          if (io.AS_U_UP) { full[unloadIdx(io)] = false; io.ST3_STEP = 140; }  // the carrier is free to turn
          break;
        case 140:
          io.SOL_U_JIG = false; io.SOL_U_BIN = true;
          if (io.AS_U_BIN) io.ST3_STEP = 150;
          break;
        case 150:
          io.SOL_U_UP = false; io.SOL_U_DN = true;
          if (io.AS_U_DN) io.ST3_STEP = 160;
          break;
        case 160:
          io.UVAC_ON = false;
          if (noUVac(io)) io.ST3_STEP = 170;
          break;
        case 170:
          io.SOL_U_DN = false; io.SOL_U_UP = true;
          if (io.AS_U_UP) io.ST3_STEP = 180;
          break;
        case 180:
          io.SOL_U_BIN = false; io.SOL_U_JIG = true;
          if (io.AS_U_JIG) io.ST3_STEP = 100;
          break;
        default:
          io.ST3_STEP = 100;
      }

      // Watchdog, one per station: a running step that stops moving is a jam, not patience. The
      // carrier's wait for work and the unloader's wait for a jig are not stuck, so they are
      // excluded by their own step numbers.
      const stuck = (tag, idle) => {
        const w = wd[tag];
        if (io[tag] !== w[0]) { w[0] = io[tag]; w[1] = t; }
        return io[tag] !== idle && running && t - w[1] >= WD_MS;
      };
      const jam = stuck('ST1_STEP', 0) || stuck('ST2_STEP', 10) || stuck('ST3_STEP', 100);
      if (running && (jam || selChanged)) {
        io.ST1_STEP = FAULT; io.ST2_STEP = 0; io.ST3_STEP = 0;
        io.TBL_RUN = false; io.GX_EXEC = false; io.GY_EXEC = false; io.EM_PAL_EN = false;
      }

      // E-STOP at any moment, and the HOME button when the machine is energised and idle.
      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        io.ST2_STEP = 0; io.ST3_STEP = 0;
        io.TBL_RUN = false; io.GX_EXEC = false; io.GY_EXEC = false; io.EM_PAL_EN = false;
        io.SOL_Z_DN = false; io.SOL_Z_UP = false; io.SOL_U_DN = false; io.SOL_U_UP = false;
        io.SOL_U_BIN = false; io.SOL_U_JIG = false;
        io.PAL_CLAMP = true; io.JIG_CLAMP = true;
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      // Individual operation: momentary buttons, PLC toggle memory cleared when INDIVIDUAL ends.
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      if (individual) {
        io.VAC_ON = !!indMem.IND_VAC; io.UVAC_ON = !!indMem.IND_UVAC;
        io.PAL_CLAMP = !!indMem.IND_PAL; io.JIG_CLAMP = !!indMem.IND_JIG;
        io.SOL_Z_DN = !!indMem.IND_LIFT; io.SOL_Z_UP = !indMem.IND_LIFT;
        io.SOL_U_DN = !!indMem.IND_ULIFT; io.SOL_U_UP = !indMem.IND_ULIFT;
        io.SOL_U_BIN = !!indMem.IND_USHIFT; io.SOL_U_JIG = !indMem.IND_USHIFT;
        io.TBL_RUN = !!io.IND_INDEX;
      }
      // The speed override the operator dialled in, passed on to the axes and the belts.
      io.OVR = io.OVR_SET;
      // Jog: the axis creeps while the button is held, and only on INDIVIDUAL.
      io.GX_JOG_P = individual && !!io.IND_GX_P;
      io.GX_JOG_N = individual && !!io.IND_GX_N;
      io.GY_JOG_P = individual && !!io.IND_GY_P;
      io.GY_JOG_N = individual && !!io.IND_GY_N;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
