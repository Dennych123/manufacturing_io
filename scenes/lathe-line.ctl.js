// INTERNAL CONTROLLER - not a PLC. The same sequence as lathe-line.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.
//
// The cell, as the video of the real line shows it: a DENSO VS-087 hangs upside down from a
// traverse beam over the aisle and serves a row of TAKISAWA TCC-2000 lathes. ONE flat conveyor
// runs along their fronts. At each machine a stopper pops up out of the belt and a pin lift
// raises the part that stopped on it up to the robot; the robot works over the pin, never over
// the belt.
//
//   feed -> belt -> stop 1 -> pin 1 up -> hand A takes the raw part
//   machine 1 door opens: hand B takes the FINISHED part, hand A puts the raw one in the chuck
//   door shuts, the cut starts, hand B puts the finished part back on pin 1
//   pin 1 down, stop 1 down -> the belt carries it to station 2, which feeds machine 2 (OP20)
//   off the end of the belt into the next-process bin
//
// So the line is OP10 then OP20 on one conveyor, which is why the same pin both gives the robot
// its raw part and takes back the finished one - exactly what the video shows at 5 s and 14 s.
//
// DOUBLE HAND at 90 degrees: hand A reaches along the flange axis, hand B across it. One door
// opening serves both halves of the exchange - take the finished part with B, put the raw one in
// with A - and presenting the other jaw is a WRIST move, which is what the arm does in front of
// the open door in the video at 9 s.
//
// The poses are joint angles solved ONCE by IK against this scene's own kinematics
// (tools/gen_lathe_line.js, lib/ik.js) and pinned in tests/lib.test.js. The program commands joint
// targets, as the real one does; nothing here solves kinematics at run time.
//
// Moves are joint-space: every axis gets its target and Execute together and the step waits for
// every Done. Execute is a LEVEL latched on its rising edge, so every move is followed by a step
// that drops Execute and waits for Done to clear before the next one (CLAUDE.md).

// BEGIN GENERATED - tools/gen_lathe_line.js
/** J1..J6 in degrees, one entry per goal. The rail owns X, so a pose serves either machine. */
export const POSE = {"chAtA":[-93.18,7.66,-37.91,91.6,-87.24,0],
 "chOutA":[-74.48,10.03,-40.73,81.92,-103.29,0],
 "doorA":[-124.77,-54.34,-17.86,123.46,-79.95,0],
 "chAtB":[-108.8,-1.09,-12.69,-7.37,-57.72,111.71],
 "chOutB":[-88.59,-4.61,-9.64,0.66,-53.28,88.31],
 "pinAtA":[66.86,-42.35,8.5,0.01,-56.15,0],
 "pinUpA":[66.82,-51.18,29.7,0,-68.51,0],
 "pinAtB":[71.46,-41.47,-14.24,-3.7,55.75,2.09],
 "pinUpB":[71.08,-54.82,10.71,-4.03,44.19,2.9],
 "home":[4.43,-47.26,42.85,0.01,-85.59,0]};
/** Where the carriage stands to work at machine n (0-based), and its parking place. */
export const RAIL = [-980,620];
// END GENERATED

const AX = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
/** A step that has not moved for this long is stuck. The longest normal wait is the 20 s cut. */
const WD_MS = 45000;
const FAULT = 900, HOME = 800, ESTOP = 910;
/** How long a jaw is given to take hold before the sequence calls it a missed grip. */
const GRIP_MS = 400;
/** The cut. */
const CUT_MS = 20000;
/** The part is driven against the stopper before the pin comes up under it (CLAUDE.md: a beam
 * says a part is HERE, the STOP is what locates it). */
const SETTLE_MS = 700;
/** Pin clamp off to the part resting on the pin, before the lift takes it down. */
const DROP_MS = 200;
/** Station steps that are moving something: these have the watchdog on them (see below). */
const MOVING = [220, 230, 250, 262, 265, 270];
const TOGGLES = ['IND_A', 'IND_B', 'IND_DOOR1', 'IND_DOOR2', 'IND_LIFT1', 'IND_LIFT2'];

export function create() {
  let pbLast = false, stopReq = false, selLast = true, stepLast = -1, stepFrom = 0;
  let masterOn = false, homed = false, masterLast = false, homeLast = false;
  const indMem = {}, indLast = {};
  let tgt = 0;                       // which machine this trip serves: 0 = machine 1, 1 = machine 2
  let hasRaw = false, hasFin = false;
  let gripFrom = -1, gripQ = false, settleFrom = [-1, -1], settleQ = [false, false];
  let dropFrom = [-1, -1], dropQ = [false, false];
  /** what the PLC believes about each machine: 0 empty, 1 cutting, 2 finished */
  let mstate = [0, 0];
  let cutFrom = [-1, -1];
  /** the robot has finished at station n and the pin may go down */
  let stGo = [false, false];
  let stLast = [-1, -1], stFrom = [0, 0];
  return {
    /** The plant was reset: its counters are back to 0, so drop the copies we compare against. */
    reset() {
      pbLast = false; stopReq = false; selLast = true; stepLast = -1; stepFrom = 0;
      masterOn = false; homed = false; masterLast = false; homeLast = false;
      tgt = 0; hasRaw = false; hasFin = false;
      gripFrom = -1; gripQ = false; settleFrom = [-1, -1]; settleQ = [false, false];
      dropFrom = [-1, -1]; dropQ = [false, false];
      mstate = [0, 0]; cutFrom = [-1, -1]; stGo = [false, false];
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
      const S = () => 'S' + (tgt + 1);
      const doorOpen = () => { io['SOL_' + M() + '_DOOR'] = true; return !!io['AS_' + M() + '_DOOR_OP']; };
      const doorShut = () => { io['SOL_' + M() + '_DOOR'] = false; return !!io['AS_' + M() + '_DOOR_CL']; };
      const allOff = () => {
        for (const a of AX) io[a + '_EXEC'] = false;
        io.RX_EXEC = false; io.EM_IN_EMIT = false; io.CV_RUN = false;
      };
      // A 2-finger grip is confirmed by the OPEN switch DROPPING, never by the CLOSED one: that
      // switch sits at full close, so with a part between the fingers it never comes on. Waiting
      // for it hangs for ever on a good grip (CLAUDE.md, measured on rb4axis).
      const gripped = jaw => !io['AS_' + jaw + '_OPEN'];
      /**
       * A machine worth a trip. The pin must be UP with a raw part on it, every time: the robot
       * gives the pin a finished part back at the end of the visit, and the only thing that
       * guarantees the pin is free then is that the robot took what was on it. Going for a
       * finished part with the pin empty put the cell in a corner it could not get out of - the
       * station cannot present an empty pin while the next part stands at the stop over it, so the
       * robot waited at the pin until the watchdog. A cut part waits in the chuck instead, which
       * is what the real cell does when the line runs dry.
       */
      const work = k => !!io['PX_S' + (k + 1)] && !!io['AS_S' + (k + 1) + '_UP'] && mstate[k] !== 1;

      switch (io.ST1_STEP) {
        case 0:
          allOff();
          io.GRIP_A = false; io.GRIP_B = false;
          io.SOL_M1_DOOR = false; io.SOL_M2_DOOR = false;
          io.M1_CHUCK = true; io.M2_CHUCK = true;
          hasRaw = false; hasFin = false;
          if (startEdge && auto && ready && homed) { stopReq = false; io.ST4_STEP = 200; io.ST5_STEP = 200; io.ST2_STEP = 100; io.ST3_STEP = 100; io.ST1_STEP = 10; }
          break;

        // ---- which machine to serve: the one with a finished part first, it is holding one hostage
        case 10:
          hasRaw = false; hasFin = false;
          // A machine with a part finished first: it is holding one hostage.
          if (work(0) && mstate[0] === 2) tgt = 0;
          else if (work(1) && mstate[1] === 2) tgt = 1;
          else if (work(0)) tgt = 0;
          else if (work(1)) tgt = 1;
          else break;
          io.ST1_STEP = 11;
          break;

        // ---- hand A takes the raw part off the pin
        case 11: { const a = railTo(RAIL[tgt]), b = arm(POSE.pinUpA); if (a && b) io.ST1_STEP = 12; } break;
        case 12:
          // The pin was up with a part on it when this trip was chosen, and the station stays at
          // its ready step until the robot says it is done, so nothing else can have appeared on
          // it. A part that has gone (the viewer's hand took it) means no exchange: the machine
          // keeps what it has and the watchdog has the last word.
          if (drop()) io.ST1_STEP = (io['PX_' + S()] ? 13 : 20);
          break;
        case 13: if (arm(POSE.pinAtA)) io.ST1_STEP = 14; break;
        case 14: if (drop()) io.ST1_STEP = 15; break;
        case 15:
          // The pin lets go as the fingers close on the part it is holding: that is a hand-over,
          // the way a robot takes a part out of a chuck.
          io.GRIP_A = true;
          if (gripQ && gripped('A')) { hasRaw = true; io[S() + '_CLAMP'] = false; io.ST1_STEP = 16; }
          break;
        case 16: if (arm(POSE.pinUpA)) io.ST1_STEP = 17; break;
        case 17: if (drop()) io.ST1_STEP = 20; break;

        // ---- in front of the machine, door open
        case 20: if (arm(POSE.doorA)) io.ST1_STEP = 21; break;
        case 21: if (drop()) io.ST1_STEP = 22; break;
        case 22: if (doorOpen()) io.ST1_STEP = 23; break;
        case 23:
          if (mstate[tgt] === 2) io.ST1_STEP = 30;
          else if (hasRaw) io.ST1_STEP = 40;
          else io.ST1_STEP = 60;
          break;

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
        case 36: if (drop()) io.ST1_STEP = (hasRaw ? 40 : 60); break;

        // ---- hand A puts the raw part in the chuck
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

        // ---- hand B stands the finished part back on the pin it came off
        case 70: if (arm(POSE.pinUpB)) io.ST1_STEP = 71; break;
        case 71: if (drop()) io.ST1_STEP = 72; break;
        case 72: if (arm(POSE.pinAtB)) io.ST1_STEP = 73; break;
        case 73: if (drop()) io.ST1_STEP = 74; break;
        case 74:
          io[S() + '_CLAMP'] = true; io.GRIP_B = false;
          if (gripQ && io.AS_B_OPEN && io['PX_' + S()]) { hasFin = false; io.ST1_STEP = 75; }
          break;
        case 75: if (arm(POSE.pinUpB)) io.ST1_STEP = 76; break;
        case 76: if (drop()) io.ST1_STEP = 80; break;

        case 80:
          // Done at this station: the pin may go down and the station is free again.
          stGo[tgt] = true;
          io.CYCLE_CNT += 1;
          io.ST1_STEP = stopReq ? 0 : 10;
          break;

        case HOME:
          allOff();
          io.SOL_M1_DOOR = false; io.SOL_M2_DOOR = false;
          if (arm(POSE.home) && railTo(RAIL[0])) { drop(); homed = true; io.ST1_STEP = 0; }
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
      if (running) io.CV_RUN = true;

      // ---------------------------------------------------------- ST2: machine 1, the cut
      // Written out per machine rather than as a loop over both, so the CASE labels here and in the
      // .st twin line up one for one: tests/ctl.test.js compares the two files' step numbers.
      if (!running) io.ST2_STEP = 0;
      else switch (io.ST2_STEP) {
        case 100: if (mstate[0] === 1) io.ST2_STEP = 110; break;
        case 110:
          // The door stays shut and the chuck stays closed for the whole of it.
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

      // ---------------------------------------------------------- ST4: station 1 and the feed
      // The stations run at the same time as the robot and as each other, so they are their own
      // state machines and meet the robot on stGo[] (CLAUDE.md).
      if (!running) { io.ST4_STEP = 0; io.EM_IN_EMIT = false; }
      else switch (io.ST4_STEP) {
        case 200:
          // Down, stop up, nothing on the pin: call for one raw part.
          io.SOL_S1_UP = false; io.SOL_S1_DN = true; io.SOL_S1_STOP = true; io.S1_CLAMP = false;
          stGo[0] = false;
          if (io.AS_S1_STOP_UP && !io.PE_S1) { io.EM_IN_EMIT = true; io.ST4_STEP = 210; }
          break;
        case 210:
          io.EM_IN_EMIT = false;
          if (io.PE_S1) io.ST4_STEP = 220;                            // it has arrived at the beam
          break;
        case 220:
          // The beam says the part is HERE; the STOP is what locates it, so keep driving it against
          // the pin for a settle time before the lift comes up (CLAUDE.md).
          if (settleQ[0]) { io.SOL_S1_DN = false; io.SOL_S1_UP = true; io.S1_CLAMP = true; io.ST4_STEP = 230; }
          break;
        case 230: if (io.AS_S1_UP) io.ST4_STEP = 240; break;
        case 240: if (stGo[0]) { stGo[0] = false; io.ST4_STEP = (io.PX_S1 ? 250 : 270); } break;
        case 250:
          // A finished part is standing on the pin. Let go of it FIRST, so it rides the pin down
          // resting on it, and the pin drops away under it onto the belt.
          io.S1_CLAMP = false;
          if (dropQ[0]) io.ST4_STEP = 260;
          break;
        case 260:
          // The STOP goes down BEFORE the pin does. Measured: with the stop still up, the part
          // came down off the pin onto the 30 mm stop head instead of the belt, stood on its rim
          // and toppled - it then rode the whole line lying down. Station 2 must be empty and
          // clear before this one is sent on: two parts at one stop cannot be metered apart again.
          if (io.ST5_STEP === 200 && !io.PE_S2) { io.SOL_S1_STOP = false; io.ST4_STEP = 262; }
          break;
        case 262: if (io.AS_S1_STOP_DN) { io.SOL_S1_UP = false; io.SOL_S1_DN = true; io.ST4_STEP = 265; } break;
        case 265: if (io.AS_S1_DN && !io.PE_S1) { io.SOL_S1_STOP = true; io.ST4_STEP = 200; } break;
        case 270:
          io.S1_CLAMP = false; io.SOL_S1_UP = false; io.SOL_S1_DN = true;
          if (io.AS_S1_DN) io.ST4_STEP = 200;
          break;
        default: io.ST4_STEP = 200;
      }

      // ---------------------------------------------------------- ST5: station 2 and the discharge
      if (!running) io.ST5_STEP = 0;
      else switch (io.ST5_STEP) {
        case 200:
          io.SOL_S2_UP = false; io.SOL_S2_DN = true; io.SOL_S2_STOP = true; io.S2_CLAMP = false;
          stGo[1] = false;
          if (io.PE_S2) io.ST5_STEP = 220;                            // station 1 has sent one on
          break;
        case 220:
          if (settleQ[1]) { io.SOL_S2_DN = false; io.SOL_S2_UP = true; io.S2_CLAMP = true; io.ST5_STEP = 230; }
          break;
        case 230: if (io.AS_S2_UP) io.ST5_STEP = 240; break;
        case 240: if (stGo[1]) { stGo[1] = false; io.ST5_STEP = (io.PX_S2 ? 250 : 270); } break;
        case 250:
          io.S2_CLAMP = false;
          if (dropQ[1]) io.ST5_STEP = 260;
          break;
        case 260: io.SOL_S2_STOP = false; io.ST5_STEP = 262; break;
        case 262: if (io.AS_S2_STOP_DN) { io.SOL_S2_UP = false; io.SOL_S2_DN = true; io.ST5_STEP = 265; } break;
        case 265: if (io.AS_S2_DN && !io.PE_S2) { io.SOL_S2_STOP = true; io.ST5_STEP = 200; } break;
        case 270:
          io.S2_CLAMP = false; io.SOL_S2_UP = false; io.SOL_S2_DN = true;
          if (io.AS_S2_DN) io.ST5_STEP = 200;
          break;
        default: io.ST5_STEP = 200;
      }

      // TONs after the CASE, as in the ST: their Q is read by the NEXT scan.
      if (io.ST1_STEP === 15 || io.ST1_STEP === 34 || io.ST1_STEP === 44 || io.ST1_STEP === 74) { if (gripFrom < 0) gripFrom = t; } else gripFrom = -1;
      gripQ = gripFrom >= 0 && t - gripFrom >= GRIP_MS;
      for (const k of [0, 1]) {
        const step = k === 0 ? io.ST4_STEP : io.ST5_STEP;
        if (step === 220) { if (settleFrom[k] < 0) settleFrom[k] = t; } else settleFrom[k] = -1;
        settleQ[k] = settleFrom[k] >= 0 && t - settleFrom[k] >= SETTLE_MS;
        if (step === 250) { if (dropFrom[k] < 0) dropFrom[k] = t; } else dropFrom[k] = -1;
        dropQ[k] = dropFrom[k] >= 0 && t - dropFrom[k] >= DROP_MS;
        // A station step that MOVES something and stops moving is a jam: a part taken off the belt
        // by hand leaves the lift waiting for a beam that never comes. The steps that wait for the
        // line - for a part to be sent on (200, 210) and for the robot (240) - are not jams: the
        // robot is 45 s away whenever it is standing over the other machine through a 20 s cut,
        // and station 2 waits as long as machine 1 takes.
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
        io.SOL_S1_UP = !!indMem.IND_LIFT1; io.SOL_S1_DN = !indMem.IND_LIFT1;
        io.SOL_S2_UP = !!indMem.IND_LIFT2; io.SOL_S2_DN = !indMem.IND_LIFT2;
      }
      io.OVR = io.OVR_SET;
      io.PL_START = io.AUTO_RUN;
      io.PL_MASTER = masterOn;
      io.PL_HOME = homed;
    },
  };
}
