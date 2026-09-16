// INTERNAL CONTROLLER - not a PLC. The same sequence as pallet-line.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.

/** A step that has not moved for this long is stuck: the longest normal step is the ~5 s belt run. */
const WD_MS = 15000;
/** Fault step: feeding and the belt off, AUTO_RUN off, waits for START to acknowledge. */
const FAULT = 900;
/** Home step: every actuator is driven to its home position. AUTO will not start until it ends. */
const HOME = 800;
/** E-STOP step: the master circuit is open and every solenoid is de-energised. */
const ESTOP = 910;
/** Individual buttons that toggle an actuator (the button is momentary, the memory is the PLC's). */
const TOGGLES = ['IND_CV', 'IND_STOP', 'IND_LIFT'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, emPLast = 0, emWLast = 0, dwellFrom = -1, dwellQ = false;
  let settleFrom = -1, settleQ = false, confFrom = -1, confQ = false;
  let stepLast = -1, stepFrom = 0, gone = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; emPLast = 0; emWLast = 0; dwellFrom = -1; dwellQ = false;
      settleFrom = -1; settleQ = false; confFrom = -1; confQ = false;
      stepLast = -1; stepFrom = 0; gone = 0;
      masterOn = false; homed = false; masterLast = false; homeLast = false;
      for (const b of TOGGLES) { indMem[b] = false; indLast[b] = false; }
    },

    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_CSTOP) stopReq = true;
      // Selector AUTO / INDIVIDUAL (rb4axis panel): START only in AUTO, the individual buttons only
      // in INDIVIDUAL, and a change while the sequence runs stops it - FAULT, not a pause.
      const auto = io.SEL_AUTO !== false, selChanged = auto !== selLast;
      selLast = auto;
      // The master circuit, as on the cell panel: E-STOP is a latching mushroom, MASTER ON
      // energises the machine, and nothing runs without it. An E-STOP also loses the home position.
      const estop = !!io.PB_ESTOP;
      const masterEdge = io.PB_MASTER && !masterLast;
      masterLast = !!io.PB_MASTER;
      const homeEdge = io.PB_HOME && !homeLast;
      homeLast = !!io.PB_HOME;
      if (estop) { masterOn = false; homed = false; }
      else if (masterEdge) masterOn = true;
      const ready = masterOn && !estop;

      // The unloader counts EVERYTHING it takes - the pallet AND the part riding on it - so the
      // discharge invariant is against both feeders (measured: RM_CNT went to 2 in one cycle).
      const fed = io.EM_P_CNT + io.EM_W_CNT;
      // A write-off goes STALE when what was written off turns up after all: RM + gone would run
      // PAST fed and the discharge step would stop waiting at all. Clamp it every scan.
      if (io.RM_CNT + gone > fed) gone = Math.max(0, fed - io.RM_CNT);

      switch (io.ST1_STEP) {
        case 0:
          io.CV1_RUN = false; io.EM_P_EMIT = false; io.EM_W_EMIT = false;
          io.SOL_LIFT = false; io.SOL_STOP = false;
          if (startEdge && auto && ready && homed && io.AS_LIFT_DN) { stopReq = false; io.ST1_STEP = 10; }
          break;
        case 10:
          // The pin must be UP before a pallet arrives: one that rises under a pallet already over
          // it tips the pallet off the belt (measured).
          io.SOL_STOP = true;
          if (io.AS_STOP_UP) { emPLast = io.EM_P_CNT; io.ST1_STEP = 20; }
          break;
        case 20:
          io.CV1_RUN = true;
          io.EM_P_EMIT = true;
          if (io.EM_P_CNT !== emPLast) { io.EM_P_EMIT = false; io.ST1_STEP = 30; }
          break;
        case 30:
          // The beam only says a pallet is HERE; the pin is what locates it. Stopping the belt on
          // the beam edge left the pallet 106 mm short of the pin, so the part feeder dropped its
          // load onto the belt behind the deck every cycle (measured: pallet centre 94.1 instead
          // of ~194, load offset +118.9 mm). Keep driving until it is pressed against the pin.
          io.CV1_RUN = true;
          if (io.PE_STN) io.ST1_STEP = 35;
          break;
        case 35:
          io.CV1_RUN = true;
          if (settleQ) { io.CV1_RUN = false; io.ST1_STEP = 40; }
          break;
        case 40:
          // Lift and locate. The pallet must be reported present for a CONFIRM time, not just for
          // the instant the switch first makes: a part is dropped on the strength of this signal,
          // and the timer restarts if it flickers.
          io.SOL_LIFT = true;
          if (confQ) { emWLast = io.EM_W_CNT; io.ST1_STEP = 50; }
          break;
        case 50:
          io.EM_W_EMIT = true;
          if (io.EM_W_CNT !== emWLast) { io.EM_W_EMIT = false; io.ST1_STEP = 60; }
          break;
        case 60:
          if (dwellQ) io.ST1_STEP = 70;                       // the station does its work
          break;
        case 70:
          io.SOL_LIFT = false;
          if (io.AS_LIFT_DN) io.ST1_STEP = 80;
          break;
        case 80:
          // Only now may the pin go down: it must be clear of the path before the pallet moves.
          io.SOL_STOP = false;
          if (io.AS_STOP_DN) io.ST1_STEP = 90;
          break;
        case 90:
          // the invariant, not "a count moved": every pallet and part fed has left the belt
          io.CV1_RUN = true;
          if (io.RM_CNT + gone >= fed) { io.CV1_RUN = false; io.ST1_STEP = 100; }
          break;
        case 100:
          io.CYCLE_CNT += 1;
          io.ST1_STEP = stopReq ? 0 : 10;
          break;
        case HOME:
          // HOME: the lift down and the pin down, so the belt is clear along its whole length and
          // the next pallet can be fed from a known state.
          io.CV1_RUN = false; io.EM_P_EMIT = false; io.EM_W_EMIT = false;
          io.SOL_LIFT = false; io.SOL_STOP = false;
          if (io.AS_LIFT_DN && io.AS_STOP_DN) { homed = true; io.ST1_STEP = 0; }
          break;
        case ESTOP:
          // The master circuit is open, so every solenoid de-energises - a 5/2 single-solenoid
          // valve springs back. The LIFT is the exception: it is a holder, and a real machine does
          // not drop what it is carrying when the power goes (the same rule as the assembler's
          // nests). Its pallet stays on the pins until someone deals with it.
          io.CV1_RUN = false; io.EM_P_EMIT = false; io.EM_W_EMIT = false;
          io.SOL_STOP = false;
          if (ready) io.ST1_STEP = 0;                  // MASTER ON after the mushroom is released
          break;
        case FAULT:
          // Stuck: a pallet was taken, never arrived, or never left. Stop feeding and the belt, and
          // leave the pin and the lift where they are: the pin is holding the queue back, and the
          // lift keeps whatever it is carrying, as a real machine does.
          io.CV1_RUN = false;
          io.EM_P_EMIT = false;
          io.EM_W_EMIT = false;
          // Acknowledging writes off what never reached the unloader. Only HERE: at every START it
          // would write off pallets still legitimately on the belt (docs/PLAN.md §13).
          if (startEdge && auto) { stopReq = false; gone = fed - io.RM_CNT; io.ST1_STEP = 0; }
          break;
      }

      // TON after the CASE, as in the ST: its Q is read by the NEXT scan.
      if (io.ST1_STEP === 60) { if (dwellFrom < 0) dwellFrom = t; } else dwellFrom = -1;
      dwellQ = dwellFrom >= 0 && t - dwellFrom >= 500;
      // 106 mm from the beam edge to the pin at 250 mm/s is 424 ms; 700 leaves margin to press up.
      if (io.ST1_STEP === 35) { if (settleFrom < 0) settleFrom = t; } else settleFrom = -1;
      settleQ = settleFrom >= 0 && t - settleFrom >= 700;
      // The pallet-present confirm: held 300 ms, and reset by any flicker of the switch.
      if (io.ST1_STEP === 40 && io.AS_LIFT_UP && io.PLT_PRESENT) { if (confFrom < 0) confFrom = t; } else confFrom = -1;
      confQ = confFrom >= 0 && t - confFrom >= 300;

      // Watchdog: a running step that stops moving is a jam, not patience. A selector change while
      // running trips the same way.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP && (t - stepFrom >= WD_MS || selChanged)) {
        io.ST1_STEP = FAULT; io.CV1_RUN = false; io.EM_P_EMIT = false; io.EM_W_EMIT = false;
      }

      // E-STOP at any moment, and the HOME button when the machine is energised and idle.
      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        io.CV1_RUN = false; io.EM_P_EMIT = false; io.EM_W_EMIT = false; io.SOL_STOP = false;
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      // Individual operation: the buttons are momentary; the PLC keeps a toggle memory per actuator
      // and clears it when INDIVIDUAL ends, so nothing stays latched into AUTO.
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      if (individual) {
        io.CV1_RUN = !!indMem.IND_CV;
        io.SOL_STOP = !!indMem.IND_STOP;
        io.SOL_LIFT = !!indMem.IND_LIFT;
        io.EM_P_EMIT = !!io.IND_FEED_P;
        io.EM_W_EMIT = !!io.IND_FEED_W;
      }
      // The speed override the operator dialled in, passed on to the axes and the belts.
      io.OVR = io.OVR_SET;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
