// INTERNAL CONTROLLER - not a PLC. The same sequence as blurobot.st, for tests and demos
// without Sysmac. Keep the two in step: a change to one is a change to both.
// Loaded from disk only, never through the API.
//
// The arm is rb4axis's BLUEROBOT: a rail along X and three joints turning about X in the Y-Z
// plane, each angle relative to the one before it. Every pose below was MEASURED with the scene's
// own worldPoses() (scratchpad IK, verified against chainPoints()), not guessed:
//
//   pose      rail     j1      j2       j3     gripper tip
//   pick       -10   52.53  -70.73   -71.80   [-10, 420, 670]  20 mm under the part's centre
//   pickUp     -10   54.93  -47.00   -97.93   40 mm above it
//   place      850   54.89  -63.04   -81.85   [850, 420, 720]  just over the outfeed belt
//   placeUp    850   55.35  -59.20   -86.15   40 mm above it
//   home         0   90.00  -90.00   -90.00   folded, the rb4axis home
//
// The nest SNAPS what it catches onto its pocket floor, and that pocket is sunk 40 mm to keep it
// out of the belt's path, so the part rests at z 660 - not on the belt surface at 700. The pick
// pose aims 20 mm under the part's centre, which puts it in the middle of the grip zone.
//
// The outfeed sits at x 850, not 450: at 450 its slab reached back into the infeed's path and held
// the incoming part from 16.6 mm away, 175 mm short of the nest. Two belts must not overlap in X.
//
// The lift is 40 mm, not more. The envelope closes as the wrist rises toward the shoulder, and the
// sweep is unforgiving: +40 keeps 22.1 deg of margin on every joint at both stations, +60 leaves
// 16.7, +80 leaves 10.5, and by +120 the pick side is OUT OF REACH altogether.

/** A step that has not moved for this long is stuck: the longest normal step is the rail traverse. */
const WD_MS = 15000;
const FAULT = 900;

/** The measured poses: [rail, j1, j2, j3]. */
const POSE = {
  home: [0, 90, -90, -90],
  pickUp: [-10, 54.93, -47, -97.93],
  pick: [-10, 52.53, -70.73, -71.8],
  placeUp: [850, 55.35, -59.2, -86.15],
  place: [850, 54.89, -63.04, -81.85],
};
const AX = ['A0', 'A1', 'A2', 'A3'];

export function create() {
  let pbLast = false, stopReq = false, emLast = 0, gone = 0;
  let stepLast = -1, stepFrom = 0, settleFrom = -1, settleQ = false, gripFrom = -1, gripQ = false;
  return {
    reset() {
      pbLast = false; stopReq = false; emLast = 0; gone = 0;
      stepLast = -1; stepFrom = 0; settleFrom = -1; settleQ = false; gripFrom = -1; gripQ = false;
    },

    /** One PLC scan: reads `in` tags, writes `out` tags. @param {Record<string, any>} io @param {number} t ms */
    scan(io, t) {
      const startEdge = io.PB_START && !pbLast;
      pbLast = io.PB_START;
      if (io.PB_STOP) stopReq = true;

      // A write-off goes stale when what was written off turns up after all; clamp it every scan.
      if (io.RM_CNT + gone > io.EM_CNT) gone = Math.max(0, io.EM_CNT - io.RM_CNT);

      /** Command all four axes to a pose. Execute is a LEVEL held until every Done is in. */
      const go = name => {
        const p = POSE[name];
        for (let i = 0; i < 4; i++) { io[AX[i] + '_TGT'] = p[i]; io[AX[i] + '_EXEC'] = true; }
        return AX.every(a => io[a + '_DONE']);
      };
      /** Drop Execute and wait for the axes to report Done has fallen, as the servo demands. */
      const clear = () => {
        for (const a of AX) io[a + '_EXEC'] = false;
        return AX.every(a => !io[a + '_DONE']);
      };

      switch (io.ST1_STEP) {
        case 0:
          io.CV_IN_RUN = false; io.CV_OUT_RUN = false; io.EM_EMIT = false;
          io.GRIP_CLOSE = false; io.NEST_CLAMP = false;
          for (const a of AX) io[a + '_EXEC'] = false;
          if (startEdge) { stopReq = false; io.ST1_STEP = 10; }
          break;
        case 10:                                              // fold to home before anything moves
          if (go('home')) io.ST1_STEP = 15;
          break;
        case 15:
          if (clear()) { emLast = io.EM_CNT; io.ST1_STEP = 20; }
          break;
        case 20:                                              // feed one part onto the infeed belt
          io.CV_IN_RUN = true;
          io.EM_EMIT = true;
          if (io.EM_CNT !== emLast) { io.EM_EMIT = false; io.ST1_STEP = 30; }
          break;
        case 30:                                              // run it into the nest
          io.CV_IN_RUN = true;
          io.NEST_CLAMP = true;
          if (io.PE_IN) io.ST1_STEP = 35;
          break;
        case 35:                                              // let it settle square in the pocket
          io.CV_IN_RUN = false;
          if (settleQ) io.ST1_STEP = 40;
          break;
        case 40:                                              // above the nest, gripper open
          io.GRIP_CLOSE = false;
          if (go('pickUp')) io.ST1_STEP = 45;
          break;
        case 45:
          if (clear()) io.ST1_STEP = 50;
          break;
        case 50:                                              // down onto the part
          if (go('pick')) io.ST1_STEP = 55;
          break;
        case 55:
          if (clear()) io.ST1_STEP = 60;
          break;
        case 60:                                              // the nest lets go, the gripper takes it
          io.NEST_CLAMP = false;
          io.GRIP_CLOSE = true;
          // NOT the `closed` switch: that sits at FULL close, so with a part between the fingers
          // it never comes on - which is the point, it is how a MISSED grip stays visible to the
          // PLC (rb4axis SIM_GRIP_TUTUP). A grip is confirmed by the fingers having left the open
          // switch and stayed off it, so wait for `open` to drop and hold.
          if (gripQ) io.ST1_STEP = 70;
          break;
        case 70:                                              // lift clear
          if (go('pickUp')) io.ST1_STEP = 75;
          break;
        case 75:
          if (clear()) io.ST1_STEP = 80;
          break;
        case 80:                                              // traverse to the outfeed
          if (go('placeUp')) io.ST1_STEP = 85;
          break;
        case 85:
          if (clear()) io.ST1_STEP = 90;
          break;
        case 90:                                              // down onto the belt
          if (go('place')) io.ST1_STEP = 95;
          break;
        case 95:
          if (clear()) io.ST1_STEP = 100;
          break;
        case 100:                                             // let it go
          io.GRIP_CLOSE = false;
          // `closed` is false all through a successful grip, so waiting for NOT closed would pass
          // instantly and lift away with the part still in the fingers. Wait for OPEN.
          if (io.GRIP_OPEN) io.ST1_STEP = 110;
          break;
        case 110:                                             // lift away before the belt runs
          if (go('placeUp')) io.ST1_STEP = 115;
          break;
        case 115:
          if (clear()) io.ST1_STEP = 120;
          break;
        case 120:                                             // the invariant: everything fed has left
          io.CV_OUT_RUN = true;
          if (io.RM_CNT + gone >= io.EM_CNT) { io.CV_OUT_RUN = false; io.ST1_STEP = 130; }
          break;
        case 130:
          io.CYCLE_CNT += 1;
          // Re-snapshot the feeder count for the NEXT cycle. Taking it only at step 15, which the
          // cycle never visits again, left emLast at its pre-first-part value: step 20 then saw
          // "the count already moved", dropped EM_EMIT without ever commanding a part, and step 30
          // waited for a part that was never fed until the watchdog faulted (measured).
          emLast = io.EM_CNT;
          io.ST1_STEP = stopReq ? 0 : 20;
          break;
        case FAULT:
          // Stuck: no part arrived, the grip missed, or nothing reached the unloader. Stop the
          // belts and the feeder and drop Execute, but KEEP the grip: a real arm does not open its
          // fingers over the floor.
          io.CV_IN_RUN = false; io.CV_OUT_RUN = false; io.EM_EMIT = false;
          for (const a of AX) io[a + '_EXEC'] = false;
          if (startEdge) { stopReq = false; gone = io.EM_CNT - io.RM_CNT; io.ST1_STEP = 0; }
          break;
      }

      // TONs after the CASE, as in the ST: their Q is read by the NEXT scan.
      if (io.ST1_STEP === 35) { if (settleFrom < 0) settleFrom = t; } else settleFrom = -1;
      settleQ = settleFrom >= 0 && t - settleFrom >= 400;
      // The grip confirm: the fingers must be off the open switch and STAY off for 300 ms. A
      // flicker restarts it, so a part that slips out of the fingers is not reported as gripped.
      if (io.ST1_STEP === 60 && io.GRIP_CLOSE && !io.GRIP_OPEN) { if (gripFrom < 0) gripFrom = t; } else gripFrom = -1;
      gripQ = gripFrom >= 0 && t - gripFrom >= 300;

      // Watchdog: a step that stops moving is a jam, not patience (CLAUDE.md).
      if (io.ST1_STEP !== stepLast) { stepLast = io.ST1_STEP; stepFrom = t; }
      if (io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT && t - stepFrom >= WD_MS) {
        io.ST1_STEP = FAULT;
        io.CV_IN_RUN = false; io.CV_OUT_RUN = false; io.EM_EMIT = false;
        for (const a of AX) io[a + '_EXEC'] = false;
      }

      io.AUTO_RUN = io.ST1_STEP !== 0 && io.ST1_STEP !== FAULT;
      io.PL_START = io.AUTO_RUN;
    },
  };
}
