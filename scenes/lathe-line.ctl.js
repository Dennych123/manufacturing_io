// INTERNAL CONTROLLER - not a PLC. The same sequence as lathe-line.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.
//
// The cell, as the video of the real line shows it: a DENSO VS-087 hangs upside down from a
// traverse beam over the aisle and serves two TAKISAWA TCC-2000 lathes that run the SAME
// operation in PARALLEL - neither feeds the other. Raw castings arrive on the infeed conveyor and
// finished parts leave on a SEPARATE outfeed conveyor; both have a stop and a pin lift at the
// loading end of the cell, which is where the robot works them.
//
//   infeed stop -> pin up -> hand A takes the raw casting
//   a lathe that is free or finished: the door SLIDES open, hand B takes the FINISHED part,
//   hand A puts the raw one in the chuck, the door shuts and the cut starts
//   outfeed pin: hand B stands the finished part on it, the pin lowers, the belt takes it away
//
// Two machines on one operation, so a 20 s cut gives a part every ~10 s and the robot is the busy
// one. DOUBLE HAND at 90 degrees: hand A reaches along the flange axis for raw parts, hand B
// across it for finished ones, so ONE door opening serves the whole exchange - which is why the
// arm turns its wrist in front of the open door in the video at 9 s.
//
// The poses are joint angles solved ONCE by IK against this scene's own kinematics
// (tools/gen_lathe_line.js, lib/ik.js) and pinned in tests/lib.test.js. The generator also checks
// every pose against the lathes' castings: the plant cannot notice an arm drawn through a machine,
// because a kinematic link does not collide with a fixed one, and the picture then lies quietly.
//
// Moves are joint-space: every axis gets its target and Execute together and the step waits for
// every Done. Execute is a LEVEL latched on its rising edge, so every move is followed by a step
// that drops Execute and waits for Done to clear before the next one (CLAUDE.md).

// BEGIN GENERATED - tools/gen_lathe_line.js
/** J1..J6 in degrees, one entry per goal. The machine poses serve either lathe: the rail
 * carries the robot one machine pitch along and the pose is the same. */
export const POSE = {"chAtA":[-93.18,23.49,-31.11,-89.58,86.84,0],
 "chOutA":[-79.65,24.94,-33.19,-91.5,100.23,0],
 "doorA":[-76.94,-22.03,15,-91.62,102.96,0],
 "chAtB":[-105.39,13.7,-7.75,-14.8,-50.19,110.08],
 "chOutB":[-90,10.24,-4.49,-0.02,-43.43,90.02],
 "doorB":[-103.29,-33.05,33.1,-9.66,-54.59,106.37],
 "inAt":[-140.14,-52.06,32.7,0.01,-70.64,0],
 "inUp":[-140.14,-58.18,52.78,0.01,-84.6,0],
 "outAt":[88.04,-26.11,8.32,68.82,41.64,-62.6],
 "outUp":[92.76,-31.59,30.13,86.45,22.56,-86.16],
 "home":[-70.21,115.61,134.43,0.03,19.96,0]};
/** Where the carriage stands: at each machine, and at the two conveyor stations. */
export const RAIL = [-880,720];
export const RAIL_STN = -1690;
// END GENERATED

const AX = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
/** A step that has not moved for this long is stuck. The longest normal wait is the 20 s cut. */
const WD_MS = 45000;
const FAULT = 900, HOME = 800, ESTOP = 910;
/** How long a jaw is given to take hold before the sequence calls it a missed grip. */
const GRIP_MS = 400;
/** The cut, the same on both machines. */
const CUT_MS = 20000;
/** The part is driven against the stop before the pin comes up under it (CLAUDE.md: a beam says a
 * part is HERE, the STOP is what locates it). */
const SETTLE_MS = 700;
/** Pin clamp off to the part resting on the pin, before the lift takes it down. */
const DROP_MS = 200;
/** Station steps that are moving something: these have the watchdog on them (see below). */
const MOVING = [220, 230, 250, 260, 262, 265, 270];
const TOGGLES = ['IND_A', 'IND_B', 'IND_DOOR1', 'IND_DOOR2', 'IND_LIFT_IN', 'IND_LIFT_OUT'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, stepLast = -1, stepFrom = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  let tgt = 0;                       // which machine this trip serves: 0 = machine 1, 1 = machine 2
  let hasRaw = false, hasFin = false;
  let gripFrom = -1, gripQ = false, settleFrom = -1, settleQ = false;
  let dropFrom = -1, dropQ = false;
  /** what the PLC believes about each machine: 0 empty, 1 cutting, 2 finished */
  let mstate = [0, 0];
  let cutFrom = [-1, -1];
  /** the robot has stood a finished part on the outfeed pin: the station may send it away */
  let outGo = false;
  let stLast = [-1, -1], stFrom = [0, 0];
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; stepLast = -1; stepFrom = 0;
      masterOn = false; homed = false; masterLast = false; homeLast = false;
      tgt = 0; hasRaw = false; hasFin = false;
      gripFrom = -1; gripQ = false; settleFrom = -1; settleQ = false;
      dropFrom = -1; dropQ = false;
      mstate = [0, 0]; cutFrom = [-1, -1]; outGo = false;
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

      const allDone = AX.every(a => io[a + '_DONE']);
      const allClear = AX.every(a => !io[a + '_DONE']);
      const arm = p => { AX.forEach((a, i) => { io[a + '_TGT'] = p[i]; io[a + '_EXEC'] = true; }); return allDone; };
      const railTo = x => { io.RX_TGT = x; io.RX_EXEC = true; return !!io.RX_DONE; };
      const drop = () => { for (const a of AX) io[a + '_EXEC'] = false; io.RX_EXEC = false; return allClear && !io.RX_DONE; };
      const M = () => 'M' + (tgt + 1);
      const doorOpen = () => { io['SOL_' + M() + '_DOOR'] = true; return !!io['AS_' + M() + '_DOOR_OP']; };
      const doorShut = () => { io['SOL_' + M() + '_DOOR'] = false; return !!io['AS_' + M() + '_DOOR_CL']; };
      const allOff = () => {
        for (const a of AX) io[a + '_EXEC'] = false;
        io.RX_EXEC = false; io.EM_IN_EMIT = false; io.CVIN_RUN = false; io.CVOUT_RUN = false;
      };
      // A 2-finger grip is confirmed by the OPEN switch DROPPING, never by the CLOSED one: that
      // switch sits at full close, so with a part between the fingers it never comes on. Waiting
      // for it hangs for ever on a good grip (CLAUDE.md, measured on rb4axis).
      const gripped = jaw => !io['AS_' + jaw + '_OPEN'];

      switch (io.ST1_STEP) {
        case 0:
          allOff();
          io.GRIP_A = false; io.GRIP_B = false;
          io.SOL_M1_DOOR = false; io.SOL_M2_DOOR = false;
          io.M1_CHUCK = true; io.M2_CHUCK = true;
          hasRaw = false; hasFin = false; outGo = false;
          if (startEdge && auto && ready && homed) { stopReq = false; io.ST4_STEP = 200; io.ST5_STEP = 200; io.ST2_STEP = 100; io.ST3_STEP = 100; io.ST1_STEP = 10; }
          break;

        // ---- hand A takes a raw casting off the infeed pin
        case 10: { const a = railTo(RAIL_STN), b = arm(POSE.inUp); if (a && b) io.ST1_STEP = 11; } break;
        case 11: if (drop()) io.ST1_STEP = 12; break;
        case 12:
          // The infeed station presents one casting at a time and holds it up until the robot has
          // it. Waiting for the station's own READY step, not just for the switch, keeps the two
          // from racing: the pin is only ever loaded at 240.
          if (io.ST4_STEP === 240 && io.PX_IN) io.ST1_STEP = 13;
          break;
        case 13: if (arm(POSE.inAt)) io.ST1_STEP = 14; break;
        case 14: if (drop()) io.ST1_STEP = 15; break;
        case 15:
          // The pin lets go as the fingers close on the part it is holding: that is a hand-over,
          // the way a robot takes a part out of a chuck.
          io.GRIP_A = true;
          if (gripQ && gripped('A')) { hasRaw = true; io.IN_CLAMP = false; io.ST1_STEP = 16; }
          break;
        case 16: if (arm(POSE.inUp)) io.ST1_STEP = 17; break;
        case 17: if (drop()) io.ST1_STEP = 20; break;

        // ---- pick a machine: one that has FINISHED first (it is holding a part hostage), else an
        // empty one. Both run the same operation, so either will do.
        case 20:
          if (mstate[0] === 2) { tgt = 0; io.ST1_STEP = 21; }
          else if (mstate[1] === 2) { tgt = 1; io.ST1_STEP = 21; }
          else if (mstate[0] === 0) { tgt = 0; io.ST1_STEP = 21; }
          else if (mstate[1] === 0) { tgt = 1; io.ST1_STEP = 21; }
          break;
        case 21: { const a = railTo(RAIL[tgt]), b = arm(POSE.doorA); if (a && b) io.ST1_STEP = 22; } break;
        case 22: if (drop()) io.ST1_STEP = 23; break;
        case 23: if (doorOpen()) io.ST1_STEP = 24; break;      // the door must be OPEN before reaching in
        case 24: io.ST1_STEP = (mstate[tgt] === 2 ? 30 : 40); break;

        // ---- hand B takes the finished part out of the chuck
        case 30: if (arm(POSE.chOutB)) io.ST1_STEP = 31; break;
        case 31: if (drop()) io.ST1_STEP = 32; break;
        case 32: if (arm(POSE.chAtB)) io.ST1_STEP = 33; break;
        case 33: if (drop()) io.ST1_STEP = 34; break;
        case 34:
          io.GRIP_B = true;
          if (gripQ && gripped('B')) { io[M() + '_CHUCK'] = false; hasFin = true; mstate[tgt] = 0; io.ST1_STEP = 35; }
          break;
        case 35: if (arm(POSE.chOutB)) io.ST1_STEP = 36; break;
        case 36: if (drop()) io.ST1_STEP = 40; break;

        // ---- hand A puts the raw casting in the chuck
        case 40: if (arm(POSE.chOutA)) io.ST1_STEP = 41; break;
        case 41: if (drop()) io.ST1_STEP = 42; break;
        case 42: if (arm(POSE.chAtA)) io.ST1_STEP = 43; break;
        case 43: if (drop()) io.ST1_STEP = 44; break;
        case 44:
          io[M() + '_CHUCK'] = true; io.GRIP_A = false;
          if (gripQ && io.AS_A_OPEN) { hasRaw = false; mstate[tgt] = 1; cutFrom[tgt] = t; io.ST1_STEP = 45; }
          break;
        case 45: if (arm(POSE.chOutA)) io.ST1_STEP = 46; break;
        case 46: if (drop()) io.ST1_STEP = 60; break;

        // ---- out of the machine and shut the door on the cut
        case 60: if (arm(POSE.doorA)) io.ST1_STEP = 61; break;
        case 61: if (drop()) io.ST1_STEP = 62; break;
        case 62: if (doorShut()) io.ST1_STEP = (hasFin ? 70 : 80); break;

        // ---- hand B stands the finished part on the outfeed pin
        case 70: { const a = railTo(RAIL_STN), b = arm(POSE.outUp); if (a && b) io.ST1_STEP = 71; } break;
        case 71: if (drop()) io.ST1_STEP = 72; break;
        case 72: if (io.ST5_STEP === 240 && !io.PX_OUT) io.ST1_STEP = 73; break;   // the pin is up and empty
        case 73: if (arm(POSE.outAt)) io.ST1_STEP = 74; break;
        case 74: if (drop()) io.ST1_STEP = 75; break;
        case 75:
          io.OUT_CLAMP = true; io.GRIP_B = false;
          if (gripQ && io.AS_B_OPEN && io.PX_OUT) { hasFin = false; io.ST1_STEP = 76; }
          break;
        case 76: if (arm(POSE.outUp)) io.ST1_STEP = 77; break;
        case 77:
          // The station is told the pin may go down only once the hand is CLEAR of it. Told at the
          // moment the fingers opened instead, the pin let go while the jaw was still retreating
          // and the open finger flicked the part off the pin and across the cell (measured).
          if (drop()) { outGo = true; io.ST1_STEP = 80; }
          break;

        case 80:
          io.CYCLE_CNT += 1;
          io.ST1_STEP = stopReq ? 0 : 10;
          break;

        case HOME:
          allOff();
          io.SOL_M1_DOOR = false; io.SOL_M2_DOOR = false;
          if (arm(POSE.home) && railTo(RAIL_STN)) { drop(); homed = true; io.ST1_STEP = 0; }
          break;
        case ESTOP:
          // The master circuit is open: every drive stops where it is. The JAWS, the CHUCKS and the
          // PINS keep what they hold - a machine does not drop a part when the power goes.
          allOff();
          if (ready) io.ST1_STEP = 0;
          break;
        case FAULT:
          allOff();
          if (startEdge && auto) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }

      const running = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      if (running) { io.CVIN_RUN = true; io.CVOUT_RUN = true; }

      // ---------------------------------------------------------- ST2: machine 1, the cut
      // Written out per machine rather than as a loop over both, so the CASE labels here and in the
      // .st twin line up one for one: tests/ctl.test.js compares the two files' step numbers.
      if (!running) io.ST2_STEP = 0;
      else switch (io.ST2_STEP) {
        case 100: if (mstate[0] === 1) io.ST2_STEP = 110; break;
        case 110:
          // The cut. The door stays shut and the chuck stays closed for the whole of it.
          if (cutFrom[0] >= 0 && t - cutFrom[0] >= CUT_MS) { mstate[0] = 2; cutFrom[0] = -1; io.ST2_STEP = 120; }
          break;
        case 120: if (mstate[0] !== 2) io.ST2_STEP = 100; break;      // the robot has taken it
        default: io.ST2_STEP = 100;
      }

      // ---------------------------------------------------------- ST3: machine 2, the cut
      if (!running) io.ST3_STEP = 0;
      else switch (io.ST3_STEP) {
        case 100: if (mstate[1] === 1) io.ST3_STEP = 110; break;
        case 110:
          if (cutFrom[1] >= 0 && t - cutFrom[1] >= CUT_MS) { mstate[1] = 2; cutFrom[1] = -1; io.ST3_STEP = 120; }
          break;
        case 120: if (mstate[1] !== 2) io.ST3_STEP = 100; break;
        default: io.ST3_STEP = 100;
      }

      // ---------------------------------------------------------- ST4: the infeed station
      // The stations run at the same time as the robot and as each other, so they are their own
      // state machines and meet the robot on the pin (CLAUDE.md).
      if (!running) { io.ST4_STEP = 0; io.EM_IN_EMIT = false; }
      else switch (io.ST4_STEP) {
        case 200:
          // Down, stop up, nothing on the pin: call for one raw casting.
          io.SOL_IN_UP = false; io.SOL_IN_DN = true; io.SOL_IN_STOP = true; io.IN_CLAMP = false;
          if (io.AS_IN_STOP_UP && !io.PE_IN) { io.EM_IN_EMIT = true; io.ST4_STEP = 210; }
          break;
        case 210:
          io.EM_IN_EMIT = false;
          if (io.PE_IN) io.ST4_STEP = 220;                            // it has arrived at the beam
          break;
        case 220:
          // The beam says the part is HERE; the STOP is what locates it, so keep driving it against
          // the pin for a settle time before the lift comes up (CLAUDE.md).
          if (settleQ) { io.SOL_IN_DN = false; io.SOL_IN_UP = true; io.IN_CLAMP = true; io.ST4_STEP = 230; }
          break;
        case 230: if (io.AS_IN_UP) io.ST4_STEP = 240; break;
        case 240:
          // Presented to the robot. The casting is gone when the pin reports empty.
          if (!io.PX_IN) io.ST4_STEP = 270;
          break;
        case 270:
          io.IN_CLAMP = false; io.SOL_IN_UP = false; io.SOL_IN_DN = true;
          if (io.AS_IN_DN) io.ST4_STEP = 200;
          break;
        default: io.ST4_STEP = 200;
      }

      // ---------------------------------------------------------- ST5: the outfeed station
      if (!running) io.ST5_STEP = 0;
      else switch (io.ST5_STEP) {
        case 200:
          // The outfeed pin waits UP and EMPTY: here it is the robot that loads it.
          io.SOL_OUT_STOP = true; io.SOL_OUT_DN = false; io.SOL_OUT_UP = true; io.OUT_CLAMP = false;
          if (io.AS_OUT_UP && io.AS_OUT_STOP_UP) io.ST5_STEP = 240;
          break;
        case 240:
          if (outGo && io.PX_OUT) { outGo = false; io.ST5_STEP = 250; }
          break;
        case 250:
          // Let go FIRST, so the part rides the pin down and the pin drops away under it.
          io.OUT_CLAMP = false;
          if (dropQ) io.ST5_STEP = 260;
          break;
        case 260:
          // The STOP goes down BEFORE the pin does: with the stop still up, the part comes off the
          // pin onto the 30 mm stop head instead of the belt, stands on its rim and topples
          // (CLAUDE.md, measured).
          io.SOL_OUT_STOP = false;
          if (io.AS_OUT_STOP_DN) io.ST5_STEP = 262;
          break;
        case 262:
          io.SOL_OUT_UP = false; io.SOL_OUT_DN = true;
          if (io.AS_OUT_DN) io.ST5_STEP = 265;
          break;
        case 265:
          // It has gone down the outfeed belt; the stop comes back up for the next one.
          if (!io.PE_OUT) { io.SOL_OUT_STOP = true; io.ST5_STEP = 200; }
          break;
        default: io.ST5_STEP = 200;
      }

      // TONs after the CASE, as in the ST: their Q is read by the NEXT scan.
      if (io.ST1_STEP === 15 || io.ST1_STEP === 34 || io.ST1_STEP === 44 || io.ST1_STEP === 75) { if (gripFrom < 0) gripFrom = t; } else gripFrom = -1;
      gripQ = gripFrom >= 0 && t - gripFrom >= GRIP_MS;
      if (io.ST4_STEP === 220) { if (settleFrom < 0) settleFrom = t; } else settleFrom = -1;
      settleQ = settleFrom >= 0 && t - settleFrom >= SETTLE_MS;
      if (io.ST5_STEP === 250) { if (dropFrom < 0) dropFrom = t; } else dropFrom = -1;
      dropQ = dropFrom >= 0 && t - dropFrom >= DROP_MS;
      // A station step that MOVES something and stops moving is a jam: a casting taken off the belt
      // by hand leaves the lift waiting for a beam that never comes. The steps that wait for the
      // line (200, 210) and for the robot (240) are not jams - the robot is 45 s away whenever it
      // stands at the other machine through a 20 s cut.
      for (const k of [0, 1]) {
        const step = k === 0 ? io.ST4_STEP : io.ST5_STEP;
        if (step !== stLast[k]) { stLast[k] = step; stFrom[k] = t; }
        if (running && MOVING.includes(step) && t - stFrom[k] >= WD_MS) { io.ST1_STEP = FAULT; allOff(); }
      }

      // Watchdog: a running step that stops moving is a jam, not patience. A selector change while
      // running trips the same way.
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP && (t - stepFrom >= WD_MS || selChanged)) { io.ST1_STEP = FAULT; allOff(); }

      if (estop) {
        if (io.ST1_STEP !== ESTOP) io.ST1_STEP = ESTOP;
        allOff();
      } else if (homeEdge && ready && io.ST1_STEP === 0) io.ST1_STEP = HOME;

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== HOME && io.ST1_STEP !== FAULT && io.ST1_STEP !== ESTOP;
      const individual = !auto && !io.AUTO_RUN && ready && io.ST1_STEP !== HOME;
      for (const b of TOGGLES) { if (individual && io[b] && !indLast[b]) indMem[b] = !indMem[b]; if (!individual) indMem[b] = false; indLast[b] = !!io[b]; }
      for (let i = 1; i <= 6; i++) { io['J' + i + '_JOG_P'] = individual && !!io['IND_J' + i + '_P']; io['J' + i + '_JOG_N'] = individual && !!io['IND_J' + i + '_N']; }
      io.RX_JOG_P = individual && !!io.IND_RX_P;
      io.RX_JOG_N = individual && !!io.IND_RX_N;
      if (individual) {
        io.GRIP_A = !!indMem.IND_A;
        io.GRIP_B = !!indMem.IND_B;
        io.SOL_M1_DOOR = !!indMem.IND_DOOR1;
        io.SOL_M2_DOOR = !!indMem.IND_DOOR2;
        io.SOL_IN_UP = !!indMem.IND_LIFT_IN; io.SOL_IN_DN = !indMem.IND_LIFT_IN;
        io.SOL_OUT_UP = !!indMem.IND_LIFT_OUT; io.SOL_OUT_DN = !indMem.IND_LIFT_OUT;
      }
      io.OVR = io.OVR_SET;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
