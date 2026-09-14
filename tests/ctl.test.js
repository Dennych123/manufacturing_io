// The scene controllers on their own, with the plant faked: the sequence discipline that only
// shows up against a real PLC, where the plant's counters move whenever physics gets round to
// it and not while the step that waits for them happens to be active.
//
// Measured on the Sysmac simulator with scenes/a-to-b, in two rounds:
//   waiting for "the unloader count moved" let an older part's removal end the cycle, so parts
//   piled up (231 loaded, 217 unloaded) and the crowd at the end sensor raised 13 pulse-stretch
//   warnings; snapshotting the count on entry balanced the parts but left a five-part pipeline
//   running at 1.1 s a cycle instead of 6.5 s, still riding other parts' removals.
// So a waiting step waits for an INVARIANT: every part loaded has left (RM = EM).
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

/** Scan loop over a plain io object; `each` runs before every scan. */
function drive(ctl, io, ms, each = () => {}) {
  for (let i = 0; i < ms / 2; i++) { io.t += 2; each(io, io.t); ctl.scan(io, io.t); }
  return io;
}

// ---------------------------------------------------------------- a-to-b
{
  const { create } = await import('../scenes/a-to-b.ctl.js');
  /**
   * Runs one part to the discharge step. `onBelt` is how many earlier parts are still out
   * there, which is what a live line looks like: the counters do not start equal.
   */
  const toDischarge = onBelt => {
    const ctl = create();
    const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0,
                 EM1_CNT: 3, RM1_CNT: 3 - onBelt, PE_END: false, CV1_RUN: false, EM1_EMIT: false };
    drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; });
    let loaded = false;
    drive(ctl, io, 3000, o => {
      if (o.ST1_STEP === 10 && o.EM1_EMIT && !loaded) { o.EM1_CNT++; loaded = true; }   // the loader answers
      if (o.ST1_STEP === 20 && o.CV1_RUN) o.PE_END = true;                              // the part reaches the eye
    });
    return { ctl, io };
  };

  {
    const { ctl, io } = toDischarge(0);
    chk('a-to-b: after the dwell the cycle waits at the discharge step', io.ST1_STEP === 40 && io.CYCLE_CNT === 0,
      'step ' + io.ST1_STEP + ', CYCLE_CNT ' + io.CYCLE_CNT);
    drive(ctl, io, 200, o => { o.RM1_CNT = o.EM1_CNT; });                                // its own part leaves
    chk('a-to-b: the cycle ends when every loaded part has left', io.CYCLE_CNT === 1 && io.ST1_STEP === 10,
      'step ' + io.ST1_STEP + ', CYCLE_CNT ' + io.CYCLE_CNT);
  }
  {
    // One older part is still on the belt. Its removal must NOT end this cycle: that is the
    // live failure, where the sequence rode other parts' removals and pipelined.
    const { ctl, io } = toDischarge(1);
    let straggled = false;
    drive(ctl, io, 200, o => { if (o.ST1_STEP === 40 && !straggled) { o.RM1_CNT++; straggled = true; } });   // ONE older part leaves
    chk('a-to-b: an older part leaving does not end this cycle', io.ST1_STEP === 40 && io.CYCLE_CNT === 0,
      'step ' + io.ST1_STEP + ', RM ' + io.RM1_CNT + ' EM ' + io.EM1_CNT + ', CYCLE_CNT ' + io.CYCLE_CNT);
    drive(ctl, io, 200, o => { o.RM1_CNT = o.EM1_CNT; });                                // now this part leaves too
    chk('a-to-b: once its own part is out the cycle completes', io.CYCLE_CNT === 1, 'CYCLE_CNT ' + io.CYCLE_CNT);
  }
  {
    // A part still in the beam belongs to the last cycle: the belt runs until the eye clears,
    // or the dwell happens with this part halfway down the belt.
    const ctl = create();
    const io = { t: 0, PB_START: false, ST1_STEP: 0, CYCLE_CNT: 0, EM1_CNT: 0, RM1_CNT: 0, PE_END: true, CV1_RUN: false, EM1_EMIT: false };
    drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; });
    let loaded = false;
    drive(ctl, io, 600, o => { if (o.ST1_STEP === 10 && o.EM1_EMIT && !loaded) { o.EM1_CNT++; loaded = true; } });
    chk('a-to-b: with the eye still blocked it runs the belt and waits for it to clear', io.ST1_STEP === 15 && io.CV1_RUN === true,
      'step ' + io.ST1_STEP + ', CV1_RUN ' + io.CV1_RUN);
    drive(ctl, io, 200, o => { o.PE_END = false; });
    chk('a-to-b: once clear it conveys this part to the eye', io.ST1_STEP === 20);
  }
}

// ---------------------------------------------------------------- pick-place
{
  const { create } = await import('../scenes/pick-place.ctl.js');
  const ctl = create();
  // One older part is still riding the outfeed belt (EM 3, RM 2).
  const io = { t: 0, PB_START: false, ST1_STEP: 0, CYCLE_CNT: 0, EM_CNT: 3, RM_CNT: 2, NEST_A_P: false,
               AS_Z_UP: true, AS_Z_DN: false, SV_INPOS: true, SV_DONE: false, VAC_SW: false, CV_RUN: false,
               EM_EMIT: false, SOL_Z_DN: false, SOL_Z_UP: true, VAC_ON: false, SV_EXEC: false, CLAMP_A: false };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; });
  let loaded = false, straggled = false;
  drive(ctl, io, 6000, o => {
    if (o.ST1_STEP === 10 && o.EM_EMIT && !loaded) { o.EM_CNT++; loaded = true; }
    if (o.ST1_STEP === 20) o.NEST_A_P = true;
    if (o.ST1_STEP === 40 && !straggled) { o.RM_CNT++; straggled = true; }      // the straggler leaves mid-pick
    o.AS_Z_DN = o.SOL_Z_DN; o.AS_Z_UP = o.SOL_Z_UP;                            // the plant answers each command
    o.VAC_SW = o.VAC_ON;
    o.SV_DONE = o.SV_EXEC;
  });
  chk('pick-place: the straggler does not end the cycle; it waits at the unloader step',
    io.ST1_STEP === 130 && io.CYCLE_CNT === 0, 'step ' + io.ST1_STEP + ', RM ' + io.RM_CNT + ' EM ' + io.EM_CNT);
  drive(ctl, io, 200, o => { o.RM_CNT = o.EM_CNT; });
  chk('pick-place: its own part arriving completes the cycle', io.CYCLE_CNT === 1 && io.ST1_STEP === 10,
    'step ' + io.ST1_STEP + ', CYCLE_CNT ' + io.CYCLE_CNT);
}

// ---------------------------------------------------------------- stopper-pusher
// Both branches must leave the beam clear before the next part is judged, or the machine sorts
// the part it has just dealt with all over again.
{
  const { create } = await import('../scenes/stopper-pusher.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV1_RUN: false, EM1_EN: false,
               SOL_STOP: false, AS_STOP_DN: false, AS_STOP_UP: true, PE_STOP: false,
               SOL_PUSH: false, AS_PUSH_EXT: false, AS_PUSH_RET: true };
  const plant = o => { o.AS_STOP_DN = o.SOL_STOP; o.AS_STOP_UP = !o.SOL_STOP; o.AS_PUSH_EXT = o.SOL_PUSH; o.AS_PUSH_RET = !o.SOL_PUSH; };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; plant(o); });
  drive(ctl, io, 200, plant);
  chk('stopper-pusher: with no part at the stopper it waits, it does not sort thin air', io.ST1_STEP === 20 && io.CYCLE_CNT === 0,
    'step ' + io.ST1_STEP);
  drive(ctl, io, 400, o => { o.PE_STOP = true; plant(o); });              // a part settles on the pin
  chk('stopper-pusher: the first part is pushed (odd) and the beam must clear before it is done',
    io.ST1_STEP === 50 && io.SOL_PUSH === false, 'step ' + io.ST1_STEP);
  drive(ctl, io, 200, o => { o.PE_STOP = true; plant(o); });              // the pushed part still in the beam
  chk('stopper-pusher: it will not finish the push while the beam is still blocked', io.ST1_STEP === 50, 'step ' + io.ST1_STEP);
  drive(ctl, io, 200, o => { o.PE_STOP = false; plant(o); });
  chk('stopper-pusher: once clear the cycle completes and waits for a new part', io.CYCLE_CNT === 1 && io.ST1_STEP === 20,
    'step ' + io.ST1_STEP + ', CYCLE_CNT ' + io.CYCLE_CNT);
}

// ---------------------------------------------------------------- assembler
{
  const { create } = await import('../scenes/assembler.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, TBL_RUN: false, TBL_INPOS: true, TBL_STATION: 0,
               EM_B_EMIT: false, EM_B_CNT: 5, EM_L_EMIT: false, EM_L_CNT: 5,
               SOL_PRESS_DN: false, SOL_PRESS_UP: true, AS_PRESS_DN: false, AS_PRESS_UP: true };
  const plant = o => { o.AS_PRESS_DN = o.SOL_PRESS_DN; o.AS_PRESS_UP = o.SOL_PRESS_UP; if (o.EM_B_EMIT) o.EM_B_CNT++; if (o.EM_L_EMIT) o.EM_L_CNT++; };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; plant(o); });
  drive(ctl, io, 2000, plant);
  chk('assembler: it reaches the index step with the press back up', io.ST1_STEP === 60 && io.AS_PRESS_UP === true, 'step ' + io.ST1_STEP);
  // The table has not moved yet, so the station it is sent to is not the one it is on.
  drive(ctl, io, 400, plant);
  chk('assembler: the index waits for the station it was sent to, not for any station',
    io.ST1_STEP === 60 && io.TBL_RUN === true && io.CYCLE_CNT === 0, 'step ' + io.ST1_STEP + ', station ' + io.TBL_STATION);
  drive(ctl, io, 100, o => { o.TBL_STATION = 1; plant(o); });
  chk('assembler: arriving at that station ends the index', io.CYCLE_CNT === 1 && io.TBL_RUN === false, 'CYCLE_CNT ' + io.CYCLE_CNT);
}

// ---------------------------------------------------------------- sort-by-height
{
  const { create } = await import('../scenes/sort-by-height.ctl.js');
  const ctl = create();
  // A tall part from an earlier cycle is still on its way to the bin: RM_T is behind EM_T.
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV_RUN: false,
               EM_T_EMIT: false, EM_T_CNT: 4, EM_S_EMIT: false, EM_S_CNT: 4, RM_T_CNT: 3, RM_S_CNT: 4,
               PE_LOW: false, PE_HIGH: false, SOL_PUSH: false, AS_PUSH_EXT: false, AS_PUSH_RET: true };
  const plant = o => { o.AS_PUSH_EXT = o.SOL_PUSH; o.AS_PUSH_RET = !o.SOL_PUSH; if (o.EM_S_EMIT) o.EM_S_CNT++; if (o.EM_T_EMIT) o.EM_T_CNT++; };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; plant(o); });
  drive(ctl, io, 200, plant);                                              // short part first (TALL_TURN starts false)
  drive(ctl, io, 600, o => { o.PE_LOW = true; plant(o); });                 // it arrives at the beams, short
  chk('sort-by-height: a short part is sent to the outfeed, not to the chute', io.ST1_STEP === 60 && io.SOL_PUSH === false,
    'step ' + io.ST1_STEP);
  drive(ctl, io, 200, plant);
  chk('sort-by-height: it waits until every short part emitted has reached the outfeed',
    io.ST1_STEP === 60 && io.CYCLE_CNT === 0, 'RM_S ' + io.RM_S_CNT + ' EM_S ' + io.EM_S_CNT);
  drive(ctl, io, 100, o => { o.RM_S_CNT = o.EM_S_CNT; plant(o); });
  chk('sort-by-height: the cycle completes when its own part is out', io.CYCLE_CNT === 1, 'CYCLE_CNT ' + io.CYCLE_CNT);
}

// ---------------------------------------------------------------- buffer-queue
{
  const { create } = await import('../scenes/buffer-queue.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV_ACC_RUN: false,
               EM_EN: false, PE_EXIT: true, PE_FULL: true, AUTO_RUN: false };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; });
  drive(ctl, io, 2200, () => {});                                          // the demand timer is 2 s
  chk('buffer-queue: with the exit beam still blocked it runs the belt and waits for it to clear',
    io.ST1_STEP === 15 && io.CV_ACC_RUN === true, 'step ' + io.ST1_STEP);
  chk('buffer-queue: the feeder is held off while the queue is full', io.EM_EN === false);
  drive(ctl, io, 100, o => { o.PE_EXIT = false; });
  chk('buffer-queue: once clear it meters the next part out', io.ST1_STEP === 20 && io.CV_ACC_RUN === true, 'step ' + io.ST1_STEP);
  drive(ctl, io, 100, o => { o.PE_EXIT = true; });
  drive(ctl, io, 100, o => { o.PE_EXIT = false; });
  chk('buffer-queue: one part past the beam is one cycle', io.CYCLE_CNT === 1 && io.CV_ACC_RUN === false,
    'CYCLE_CNT ' + io.CYCLE_CNT + ', belt ' + io.CV_ACC_RUN);
}

process.exit(fail ? 1 : 0);
