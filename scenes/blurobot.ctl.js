// INTERNAL CONTROLLER - not a PLC. The same sequence as blurobot.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.
//
// This is a PORT of rb4axis PRG_SIM_ROBOT.st, not an invention. The cell, the station table, the
// job priorities, the step numbers and the timings are that program's.
//
//   station   type  rail x   surface z   process
//   WIP IN     0     -1400      250        -      stock is not simulated: always FULL
//   ICC 1      1      -700      285      17.0 s   test; the cover presses the PCB onto the probes
//   ICC 2      1      -300      285      17.0 s
//   DW 1       2       300      250      15.0 s   data writer
//   DW 2       2       700      250      15.0 s
//   WIP OUT    3      1400      250        -      the product leaves: always EMPTY
//
// Job priority, and the ORDER is the behaviour (PRG_SIM_ROBOT lines 456-509):
//   1. fill an EMPTY ICC from WIP IN - both of them - before anything else. That is what "buffer"
//      means: the 17 s tester must not idle while the robot runs errands for DW.
//   2. empty a FINISHED DW to WIP OUT, ahead of 3, so there is always a free DW. Without it two
//      full DWs plus two finished ICCs deadlock, and the deadlock looks like a robot that stopped.
//   3. move a FINISHED ICC to an EMPTY DW.
//
// Station state and the process clocks are PROGRAM variables, not machine IO - they are what the
// PLC believes, and rb4axis only publishes them so its viewer can draw them. The tags this reads
// and writes are the real ones: the axes, the gripper, each nest's clamp/present, and each cover.
//
// Poses were measured against this scene's own kinematics with the carriage at z 200, where all
// twelve (six surfaces + six 140 mm approaches) solve with 67.3 deg of worst-case joint margin.
// Only two station heights exist, so there are four arm poses; the rail carries the difference.

/** name, rail x, surface z, type (0 WIP IN, 1 ICC, 2 DW, 3 WIP OUT), process seconds */
const ST = [
  ['wipIn', -1400, 250, 0, 0], ['icc1', -700, 285, 1, 17], ['icc2', -300, 285, 1, 17],
  ['dw1', 300, 250, 2, 15], ['dw2', 700, 250, 2, 15], ['wipOut', 1400, 250, 3, 0],
];
const POSE = {
  home: [90, -90, -90],                          // axes 1-3; the rail is NOT part of home
  250: { at: [9.76, -67.25, -32.5], up: [31.86, -77.37, -44.49] },
  285: { at: [15.72, -71.33, -34.39], up: [36.55, -77.44, -49.11] },
};
const AX = ['A0', 'A1', 'A2', 'A3'];
const COVER_OPEN = 80;
/** 30 s, not 15: a station's own process is 17 s, and the arm waits at step 8/21 for that cover. */
const WD_MS = 30000;
const FAULT = 900;

/** The cover tag stem, or '' where there is none - the WIP racks are open shelves. */
const coverOf = i => (ST[i][3] === 1 || ST[i][3] === 2 ? ST[i][0].toUpperCase() + '_CV' : '');
const nestOf = i => ST[i][0].toUpperCase();

export function create() {
  let pbLast = false, stopReq = false, stepLast = -1, stepFrom = 0;
  let src = -1, dst = -1, idx = 0, holding = false;
  let gripFrom = -1, gripQ = false;
  /** what the PLC believes about each station: 0 empty, 1 processing, 2 done. */
  let state = ST.map(s => (s[3] === 0 ? 2 : 0));
  let timer = ST.map(() => 0);
  /** the cover target each station is currently latched on - a change re-pulses its Execute */
  let coverWant = ST.map(() => COVER_OPEN);
  return {
    reset() {
      pbLast = false; stopReq = false; stepLast = -1; stepFrom = 0;
      src = -1; dst = -1; idx = 0; holding = false;
      gripFrom = -1; gripQ = false;
      state = ST.map(s => (s[3] === 0 ? 2 : 0));
      timer = ST.map(() => 0);
      coverWant = ST.map(() => COVER_OPEN);
    },

    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_STOP) stopReq = true;
      const dt = 0.002;

      // ---- the machines. Each keeps its OWN clock, running together: that is the whole point of
      // a buffer - ICC 1 and ICC 2 must be able to test at once while the robot is elsewhere.
      for (let i = 0; i < ST.length; i++) {
        const type = ST[i][3], cv = coverOf(i);
        if (type === 0) state[i] = 2;                          // WIP IN never runs out
        else if (type === 3) state[i] = 0;                     // WIP OUT never fills up
        else if (state[i] === 1 && Number(io[cv + '_POS']) <= 0.5) {
          // The clock runs ONLY with the cover shut. The cover is what presses the PCB onto the
          // probes; counting before it closes means claiming a test on a board nothing touched.
          timer[i] = Math.max(0, timer[i] - dt);
          if (timer[i] <= 0) state[i] = 2;
        }
        if (cv) {
          // Closes only while processing, and REOPENS if the arm is in its sweep: a light curtain,
          // not a delay. Without it the leaf swings onto an arm that is still lifting out.
          const inZone = Math.abs(Number(io.A0_POS) - ST[i][1]) < 200 && Number(io.A1_POS) < 60;
          const want = state[i] === 1 && !inZone ? 0 : COVER_OPEN;
          // Execute is LATCHED on its rising edge, like every other axis here. Holding it true
          // from the first scan means the target is never re-read: measured, the covers latched
          // their startup 80 and sat there while _CV_TGT went to 0 and stayed 0, so no ICC clock
          // ever ran and no board ever finished. Drop exec for a scan whenever the target moves.
          if (want !== coverWant[i]) { coverWant[i] = want; io[cv + '_EXEC'] = false; }
          else { io[cv + '_TGT'] = want; io[cv + '_EXEC'] = true; }
        }
      }

      // ---- every station HOLDS what is on it, all the time. Only the one being picked from lets
      // go, and only while the gripper is taking the board. Measured with no clamp set at all: the
      // nest held nothing, WIPIN_P never came true, and the fingers closed to x = 0 straight
      // through the PCB - the plant only measures a part's width for a FREE part, so a grip on a
      // still-held board reads as a grip on air, and `closed` asserts for the wrong reason.
      const picking = io.ST1_STEP === 14 || io.ST1_STEP === 15;
      for (let i = 0; i < ST.length; i++) io[nestOf(i) + '_CLAMP'] = !(picking && i === idx);

      // WIP IN is an ENDLESS stock: the PLC pins its state to "full" because the racks either side
      // of the cell are not simulated, so the plant has to keep a board there or the cell runs dry
      // and the next grip closes on air. The command is a LEVEL held until the feeder's own count
      // answers it, never a pulse - and it only asks while that nest is empty and the arm is away.
      io.EM_IN_EMIT = !io.WIPIN_P && !(picking && idx === 0);

      const atPose = AX.every(a => io[a + '_DONE']);
      const cleared = AX.every(a => !io[a + '_DONE']);
      const armAt = () => io.A1_DONE && io.A2_DONE && io.A3_DONE;
      const pose = i => POSE[ST[i][2]];
      const coverOk = i => !coverOf(i) || Number(io[coverOf(i) + '_POS']) >= COVER_OPEN - 0.5;
      const go = arm => {
        io.A0_TGT = ST[idx][1]; io.A1_TGT = arm[0]; io.A2_TGT = arm[1]; io.A3_TGT = arm[2];
        for (const a of AX) io[a + '_EXEC'] = true;
        return atPose;
      };
      const fold = () => {                                     // axes 1-3 only; the rail stays put
        io.A1_TGT = POSE.home[0]; io.A2_TGT = POSE.home[1]; io.A3_TGT = POSE.home[2];
        for (const a of ['A1', 'A2', 'A3']) io[a + '_EXEC'] = true;
        return armAt();
      };
      const drop = () => { for (const a of AX) io[a + '_EXEC'] = false; return cleared; };

      switch (io.ST1_STEP) {
        case 0:                                                // pick a job
          src = -1; dst = -1;
          for (let i = 0; i < ST.length && dst < 0; i++) if (ST[i][3] === 1 && state[i] === 0) { src = 0; dst = i; }
          if (dst < 0) for (let i = 0; i < ST.length && dst < 0; i++) if (ST[i][3] === 2 && state[i] === 2) { src = i; dst = 5; }
          if (dst < 0) {
            for (let i = 0; i < ST.length && dst < 0; i++) {
              if (ST[i][3] === 1 && state[i] === 2) {
                for (let j = 0; j < ST.length && dst < 0; j++) if (ST[j][3] === 2 && state[j] === 0) { src = i; dst = j; }
              }
            }
          }
          if (dst >= 0) { idx = src; io.ST1_STEP = 5; }
          else if (stopReq) stopReq = false;                   // no work and asked to stop: stop now
          break;

        case 5: if (fold()) io.ST1_STEP = 7; break;
        case 7:                                                // slide the rail, folded
          io.A0_TGT = ST[idx][1]; io.A0_EXEC = true;
          io.ST1_STEP = 8;
          break;
        case 8:
          // TWO things, and the wait is HERE rather than after the approach is asked for: asking
          // while the cover is still shut gets the move refused, nothing moves, and "done" stays
          // true - so the next step would drive straight down from the travel pose, skipping the
          // approach that keeps the arm off the machines.
          if (io.A0_DONE && coverOk(idx)) io.ST1_STEP = 10;
          break;
        case 10: if (go(pose(idx).up)) io.ST1_STEP = 11; break;
        case 11: if (drop()) io.ST1_STEP = 12; break;
        case 12: if (go(pose(idx).at)) io.ST1_STEP = 13; break;
        case 13: if (drop()) io.ST1_STEP = 14; break;
        case 14:
          io.GRIP_CLOSE = true;                                // the station let go above
          io.ST1_STEP = 15;
          break;
        case 15:
          // NOT `closed`: that switch sits at FULL close, and a good grip STOPS at the product
          // width - measured, the jaws ran 90 -> 60 mm half-opening on a 120 mm board and held
          // there, with the PCB transferring to the gripper. Waiting for `closed` waits for the
          // signal that means the fingers found nothing (rb4axis SIM_GRIP_TUTUP). Confirm on the
          // open switch dropping and STAYING dropped, so a board slipping out restarts it.
          if (gripQ) {
            holding = true;
            // against SRC, for the same reason step 27 uses DST: only the station the board came
            // OUT of is emptied, never whichever station idx happens to point at later.
            if (ST[src][3] !== 0) state[src] = 0;
            io.ST1_STEP = 16;
          }
          break;
        case 16: if (go(pose(idx).up)) io.ST1_STEP = 17; break;
        case 17: if (drop()) io.ST1_STEP = 18; break;
        case 18: if (fold()) { idx = dst; io.ST1_STEP = 20; } break;
        case 20:
          io.A0_TGT = ST[idx][1]; io.A0_EXEC = true;
          io.ST1_STEP = 21;
          break;
        case 21: if (io.A0_DONE && coverOk(idx)) io.ST1_STEP = 22; break;
        case 22: if (go(pose(idx).up)) io.ST1_STEP = 23; break;
        case 23: if (drop()) io.ST1_STEP = 24; break;
        case 24: if (go(pose(idx).at)) io.ST1_STEP = 25; break;
        case 25: if (drop()) io.ST1_STEP = 26; break;
        case 26:
          io.GRIP_CLOSE = false;                               // the station is already holding
          io.ST1_STEP = 27;
          break;
        case 27:
          if (io.GRIP_OPEN) {
            holding = false;
            // WIP OUT: the product leaves the cell, and THAT is where a cycle is counted - exit to
            // exit, because that is what decides how many boards an hour leave.
            // Written against DST, never `idx`: idx is reused for source then destination, and on
            // a job whose SOURCE was this same station the step-15 clear would zero the state that
            // was just set here. Measured: ICC 1 was loaded, set to processing, then immediately
            // reported empty again, so its cover never shut and its 17 s clock never ran.
            if (ST[dst][3] === 3) io.CYCLE_CNT += 1;
            else { state[dst] = 1; timer[dst] = ST[dst][4]; }
            io.ST1_STEP = 28;
          }
          break;
        case 28: if (go(pose(idx).up)) io.ST1_STEP = 29; break;
        case 29: if (drop()) io.ST1_STEP = 30; break;
        case 30: if (fold()) io.ST1_STEP = 31; break;
        case 31:
          // Cycle stop is served HERE: the product is placed and the arm folded, so stopping is
          // controlled and START works again straight away.
          if (stopReq) stopReq = false;
          io.ST1_STEP = 0;
          break;

        case FAULT:
          io.GRIP_CLOSE = holding;                             // never open the fingers over the floor
          for (const a of AX) io[a + '_EXEC'] = false;
          if (startEdge) { stopReq = false; io.ST1_STEP = 0; }
          break;
      }

      // The grip confirm: fingers off the open switch and staying off for 300 ms.
      if (io.ST1_STEP === 15 && io.GRIP_CLOSE && !io.GRIP_OPEN) { if (gripFrom < 0) gripFrom = t; } else gripFrom = -1;
      gripQ = gripFrom >= 0 && t - gripFrom >= 300;

      // Watchdog: a step that stops moving is a jam, not patience (CLAUDE.md).
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && t - stepFrom >= WD_MS) {
        io.ST1_STEP = FAULT;
        for (const a of AX) io[a + '_EXEC'] = false;
      }

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT;
      io.PL_START = io.AUTO_RUN;
    },
  };
}
