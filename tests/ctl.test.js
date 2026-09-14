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

// ---------------------------------------------------------------- watchdog and write-off
// A part can leave the machine without reaching the unloader: the viewer's hand takes it, or it
// falls off. Then RM = EM can never hold again. Measured before this existed: a-to-b sat at step
// 40 with the belt running and AUTO_RUN on for as long as anyone watched, and pick-place sat at
// step 20 waiting for a nest that would never report a part. A machine that waits for ever looks
// alive and is not.
{
  const { create } = await import('../scenes/a-to-b.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, EM1_CNT: 0, RM1_CNT: 0,
               PE_END: false, CV1_RUN: false, EM1_EMIT: false, AUTO_RUN: false, PL_START: false };
  const press = () => { drive(ctl, io, 8, o => { o.PB_START = true; }); drive(ctl, io, 8, o => { o.PB_START = false; }); };
  const feed = ms => {
    let loaded = false;
    drive(ctl, io, ms, o => {
      if (o.ST1_STEP === 10 && o.EM1_EMIT && !loaded) { o.EM1_CNT++; loaded = true; }
      if (o.ST1_STEP === 20 && o.CV1_RUN) o.PE_END = true;
    });
  };
  press();
  feed(3000);                                                              // the part never reaches the unloader
  chk('a-to-b: the discharge step waits for the part that was taken away', io.ST1_STEP === 40 && io.CV1_RUN === true,
    'step ' + io.ST1_STEP + ', RM ' + io.RM1_CNT + ' EM ' + io.EM1_CNT);
  drive(ctl, io, 16000, () => {});
  chk('a-to-b: a step that stops moving faults instead of waiting for ever',
    io.ST1_STEP === 900 && io.AUTO_RUN === false && io.CV1_RUN === false && io.EM1_EMIT === false,
    'step ' + io.ST1_STEP + ', AUTO ' + io.AUTO_RUN + ', belt ' + io.CV1_RUN);
  press();
  chk('a-to-b: START acknowledges the fault and leaves it idle', io.ST1_STEP === 0 && io.AUTO_RUN === false, 'step ' + io.ST1_STEP);
  io.PE_END = false;
  press();
  let removed = false;
  drive(ctl, io, 6000, o => {
    if (o.ST1_STEP === 10 && o.EM1_EMIT && o.EM1_CNT < 2) o.EM1_CNT++;
    if (o.ST1_STEP === 20 && o.CV1_RUN) o.PE_END = true;
    if (o.ST1_STEP === 40 && !removed) { o.RM1_CNT++; removed = true; }    // only THIS part is unloaded
  });
  chk('a-to-b: the lost part is written off when the fault is acknowledged, so it cycles again',
    io.CYCLE_CNT === 1 && io.ST1_STEP === 10, 'CYCLE_CNT ' + io.CYCLE_CNT + ', step ' + io.ST1_STEP + ', RM ' + io.RM1_CNT + ' EM ' + io.EM1_CNT);
}

{
  const { create } = await import('../scenes/pick-place.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV_RUN: false, EM_EMIT: false,
               EM_CNT: 0, RM_CNT: 0, NEST_A_P: false, CLAMP_A: false, VAC_ON: false, VAC_SW: false,
               SOL_Z_UP: false, SOL_Z_DN: false, AS_Z_UP: true, AS_Z_DN: false, SV_TGT: 0, SV_EXEC: false,
               SV_DONE: false, SV_INPOS: true, AUTO_RUN: false, PL_START: false };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; });
  let loaded = false;
  drive(ctl, io, 500, o => { if (o.ST1_STEP === 10 && o.EM_EMIT && !loaded) { o.EM_CNT++; loaded = true; } });
  chk('pick-place: it waits for the nest to report the part', io.ST1_STEP === 20, 'step ' + io.ST1_STEP);
  drive(ctl, io, 16000, () => {});                                         // the part was taken out of the nest
  chk('pick-place: a part that never arrives faults instead of waiting for ever',
    io.ST1_STEP === 900 && io.AUTO_RUN === false && io.EM_EMIT === false && io.CV_RUN === false,
    'step ' + io.ST1_STEP + ', AUTO ' + io.AUTO_RUN);
  chk('pick-place: the fault does not drop a part the cup is holding', io.VAC_ON === false || io.VAC_SW === false);
}

// A written-off part can still turn up: the hand drops it back on the line, or it rolls into the
// unloader later. Then RM catches up with EM and the write-off is stale. If GONE is left as it
// was, RM + GONE runs PAST EM and the discharge step stops waiting altogether - the pipelining
// bug again, silently. In ST it is worse: GONE is a UDINT, so EM - RM with RM ahead underflows to
// ~4 billion and the invariant is true for ever.
{
  const { create } = await import('../scenes/a-to-b.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, EM1_CNT: 0, RM1_CNT: 0,
               PE_END: false, CV1_RUN: false, EM1_EMIT: false, AUTO_RUN: false, PL_START: false };
  const press = () => { drive(ctl, io, 8, o => { o.PB_START = true; }); drive(ctl, io, 8, o => { o.PB_START = false; }); };
  press();
  drive(ctl, io, 3000, o => {
    if (o.ST1_STEP === 10 && o.EM1_EMIT && o.EM1_CNT < 1) o.EM1_CNT++;
    if (o.ST1_STEP === 20 && o.CV1_RUN) o.PE_END = true;
  });
  drive(ctl, io, 16000, () => {});                                         // the part was taken: fault
  press();                                                                 // acknowledge: GONE = 1 - 0 = 1
  io.RM1_CNT = 1;                                                          // and now the strays turn up after all
  io.PE_END = false;
  press();
  drive(ctl, io, 6000, o => {
    if (o.ST1_STEP === 10 && o.EM1_EMIT && o.EM1_CNT < 2) o.EM1_CNT++;
    if (o.ST1_STEP === 20 && o.CV1_RUN) o.PE_END = true;
  });
  chk('a-to-b: a write-off that turns up again does not end the next cycle early',
    io.ST1_STEP === 40 && io.CYCLE_CNT === 0, 'step ' + io.ST1_STEP + ', CYCLE_CNT ' + io.CYCLE_CNT + ', RM ' + io.RM1_CNT + ' EM ' + io.EM1_CNT);
  drive(ctl, io, 200, o => { o.RM1_CNT = 2; });                            // its own part really leaves
  chk('a-to-b: and it still completes once its own part has left', io.CYCLE_CNT === 1, 'CYCLE_CNT ' + io.CYCLE_CNT);
}

// ---------------------------------------------------------------- watchdog on the other four
{
  const { create } = await import('../scenes/stopper-pusher.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV1_RUN: false, EM1_EN: false,
               SOL_STOP: false, SOL_PUSH: false, AS_STOP_DN: false, AS_PUSH_EXT: false, AS_PUSH_RET: true,
               PE_STOP: false, AUTO_RUN: false, PL_START: false };
  const plant = o => { o.AS_STOP_DN = o.SOL_STOP; o.AS_PUSH_EXT = o.SOL_PUSH; o.AS_PUSH_RET = !o.SOL_PUSH; };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; plant(o); });
  drive(ctl, io, 500, plant);                                              // no part ever settles at the pin
  chk('stopper-pusher: it waits at the settle step for a part that never comes', io.ST1_STEP === 20, 'step ' + io.ST1_STEP);
  drive(ctl, io, 16000, plant);
  chk('stopper-pusher: the missing part faults instead of waiting for ever',
    io.ST1_STEP === 900 && io.AUTO_RUN === false && io.CV1_RUN === false && io.EM1_EN === false, 'step ' + io.ST1_STEP);
  chk('stopper-pusher: the fault leaves the stopper down, holding the queue', io.SOL_STOP === true);
}

{
  const { create } = await import('../scenes/sort-by-height.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV_RUN: false,
               EM_T_EMIT: false, EM_T_CNT: 4, EM_S_EMIT: false, EM_S_CNT: 4, RM_T_CNT: 4, RM_S_CNT: 4,
               PE_LOW: false, PE_HIGH: false, SOL_PUSH: false, AS_PUSH_EXT: false, AS_PUSH_RET: true, AUTO_RUN: false };
  const plant = o => { o.AS_PUSH_EXT = o.SOL_PUSH; o.AS_PUSH_RET = !o.SOL_PUSH; if (o.EM_S_EMIT) o.EM_S_CNT++; if (o.EM_T_EMIT) o.EM_T_CNT++; };
  const press = () => { drive(ctl, io, 8, o => { o.PB_START = true; plant(o); }); drive(ctl, io, 8, o => { o.PB_START = false; plant(o); }); };
  press();
  drive(ctl, io, 400, plant);                                              // a short part is fed, then taken off the belt
  chk('sort-by-height: it waits at the beam for a part that was taken', io.ST1_STEP === 20, 'step ' + io.ST1_STEP);
  drive(ctl, io, 16000, plant);
  chk('sort-by-height: the missing part faults instead of waiting for ever',
    io.ST1_STEP === 900 && io.AUTO_RUN === false && io.CV_RUN === false && io.SOL_PUSH === false, 'step ' + io.ST1_STEP);
  const emS = io.EM_S_CNT;
  press();                                                                 // acknowledge: the fed part is written off
  chk('sort-by-height: START acknowledges the fault', io.ST1_STEP === 0, 'step ' + io.ST1_STEP);
  press();
  let unloaded = false;
  drive(ctl, io, 4000, o => {
    plant(o);
    if (o.ST1_STEP === 20 && o.CV_RUN) o.PE_LOW = true;
    if (o.ST1_STEP === 60 && !unloaded) { o.RM_S_CNT++; unloaded = true; }  // only THIS part reaches the outfeed
  });
  chk('sort-by-height: the written-off part does not block the next cycle',
    io.CYCLE_CNT === 1 && io.EM_S_CNT === emS + 1, 'CYCLE_CNT ' + io.CYCLE_CNT + ', RM_S ' + io.RM_S_CNT + ' EM_S ' + io.EM_S_CNT);
}

{
  const { create } = await import('../scenes/buffer-queue.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV_ACC_RUN: false,
               EM_EN: false, PE_EXIT: true, PE_FULL: false, AUTO_RUN: false };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; });
  drive(ctl, io, 2500, () => {});                                          // the exit beam never clears
  chk('buffer-queue: it runs the belt waiting for the exit beam to clear', io.ST1_STEP === 15 && io.CV_ACC_RUN === true,
    'step ' + io.ST1_STEP);
  drive(ctl, io, 26000, () => {});
  chk('buffer-queue: a beam that never clears faults instead of waiting for ever',
    io.ST1_STEP === 900 && io.AUTO_RUN === false && io.CV_ACC_RUN === false, 'step ' + io.ST1_STEP);
  chk('buffer-queue: the fault holds the feeder off too', io.EM_EN === false);
}

{
  const { create } = await import('../scenes/assembler.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, TBL_RUN: false, TBL_INPOS: true,
               TBL_STATION: 0, EM_B_EMIT: false, EM_B_CNT: 0, EM_L_EMIT: false, EM_L_CNT: 0,
               SOL_PRESS_DN: false, SOL_PRESS_UP: true, AS_PRESS_UP: true, AS_PRESS_DN: false, AUTO_RUN: false };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; });
  drive(ctl, io, 500, () => {});                                           // the base feeder is blocked: its count never moves
  chk('assembler: it holds the feed command waiting for the counter', io.ST1_STEP === 10 && io.EM_B_EMIT === true,
    'step ' + io.ST1_STEP);
  drive(ctl, io, 16000, () => {});
  chk('assembler: a feeder that cannot drop faults instead of waiting for ever',
    io.ST1_STEP === 900 && io.AUTO_RUN === false && io.TBL_RUN === false && io.EM_B_EMIT === false, 'step ' + io.ST1_STEP);
  chk('assembler: the fault lifts the press off the work', io.SOL_PRESS_UP === true && io.SOL_PRESS_DN === false);
}

// ---------------------------------------------------------------- pallet-line
// The unloader counts the pallet AND the part riding on it, so the discharge step waits on both
// feeders. The stop pin must be UP before a pallet is fed (one that rises under a pallet already
// over it tips it off the belt) and may only go DOWN once the lift is down.
{
  const { create } = await import('../scenes/pallet-line.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV1_RUN: false,
               EM_P_EMIT: false, EM_P_CNT: 3, EM_W_EMIT: false, EM_W_CNT: 3, RM_CNT: 5, PE_STN: false,
               SOL_STOP: false, AS_STOP_UP: false, AS_STOP_DN: true,
               SOL_LIFT: false, AS_LIFT_UP: false, AS_LIFT_DN: true, PLT_PRESENT: false, AUTO_RUN: false };
  // RM 5 against 6 fed: one earlier pallet is still on its way out, as a live line looks.
  const plant = o => {
    o.AS_STOP_UP = o.SOL_STOP; o.AS_STOP_DN = !o.SOL_STOP;
    o.AS_LIFT_UP = o.SOL_LIFT; o.AS_LIFT_DN = !o.SOL_LIFT;
    o.PLT_PRESENT = o.SOL_LIFT;
    if (o.EM_P_EMIT) o.EM_P_CNT++;
    if (o.EM_W_EMIT) o.EM_W_CNT++;
  };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; plant(o); });
  drive(ctl, io, 100, plant);
  chk('pallet-line: the stop pin is up before a pallet is fed', io.ST1_STEP >= 20 && io.SOL_STOP === true,
    'step ' + io.ST1_STEP + ', SOL_STOP ' + io.SOL_STOP);
  drive(ctl, io, 300, o => { plant(o); if (o.ST1_STEP === 30 && o.CV1_RUN) o.PE_STN = true; });
  // The beam only says a pallet is HERE. The belt keeps running through step 35 so the pallet is
  // pressed against the pin: stopping on the beam edge left it 106 mm short and the part feeder
  // dropped its load onto the belt behind the deck (measured).
  chk('pallet-line: the beam does not stop the belt; the pin does', io.ST1_STEP === 35 && io.CV1_RUN === true,
    'step ' + io.ST1_STEP + ', belt ' + io.CV1_RUN);
  drive(ctl, io, 900, plant);                                              // the 700 ms settle against the pin
  chk('pallet-line: it lifts once the pallet is at the stop', io.ST1_STEP >= 40 && io.SOL_LIFT === true, 'step ' + io.ST1_STEP);
  drive(ctl, io, 900, plant);                                              // load, dwell, lift down, pin down
  chk('pallet-line: the pin only goes down after the lift is down',
    io.ST1_STEP === 90 && io.SOL_STOP === false && io.SOL_LIFT === false, 'step ' + io.ST1_STEP);
  const fed = io.EM_P_CNT + io.EM_W_CNT;
  drive(ctl, io, 200, o => { plant(o); o.RM_CNT = fed - 1; });             // the straggler leaves, ours has not
  chk('pallet-line: an older pallet leaving does not end this cycle', io.ST1_STEP === 90 && io.CYCLE_CNT === 0,
    'RM ' + io.RM_CNT + ' fed ' + fed + ', CYCLE_CNT ' + io.CYCLE_CNT);
  drive(ctl, io, 200, o => { plant(o); o.RM_CNT = fed; });                 // pallet and part both out
  chk('pallet-line: the cycle completes when both the pallet and its part are out', io.CYCLE_CNT === 1,
    'CYCLE_CNT ' + io.CYCLE_CNT + ', step ' + io.ST1_STEP);
  // Nothing reaches the unloader from here on, so the discharge step is where it gets stuck. Let
  // it settle there FIRST: the watchdog counts time on ONE step, and the sequence still had a few
  // steps to walk through.
  io.PE_STN = false;
  drive(ctl, io, 4000, plant);
  chk('pallet-line: with nothing unloaded it waits at the discharge step', io.ST1_STEP === 90, 'step ' + io.ST1_STEP);
  drive(ctl, io, 16000, plant);
  chk('pallet-line: a step that stops moving faults instead of waiting for ever',
    io.ST1_STEP === 900 && io.AUTO_RUN === false && io.CV1_RUN === false, 'step ' + io.ST1_STEP);
}

// A part is dropped on the strength of PLT_PRESENT, so that signal has to be CONFIRMED - held for
// 300 ms - and not merely seen once. A switch that flickers must not load a pallet that is not
// properly on the pins.
{
  const { create } = await import('../scenes/pallet-line.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV1_RUN: false,
               EM_P_EMIT: false, EM_P_CNT: 0, EM_W_EMIT: false, EM_W_CNT: 0, RM_CNT: 0, PE_STN: false,
               SOL_STOP: false, AS_STOP_UP: false, AS_STOP_DN: true,
               SOL_LIFT: false, AS_LIFT_UP: false, AS_LIFT_DN: true, PLT_PRESENT: false, AUTO_RUN: false };
  const base = o => {
    o.AS_STOP_UP = o.SOL_STOP; o.AS_STOP_DN = !o.SOL_STOP;
    o.AS_LIFT_UP = o.SOL_LIFT; o.AS_LIFT_DN = !o.SOL_LIFT;
    if (o.EM_P_EMIT) o.EM_P_CNT++;
    if (o.EM_W_EMIT) o.EM_W_CNT++;
  };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; base(o); o.PLT_PRESENT = false; });
  drive(ctl, io, 1400, o => { base(o); o.PLT_PRESENT = false; if (o.ST1_STEP === 30 && o.CV1_RUN) o.PE_STN = true; });
  // at the stop, lifting, but the present switch chatters: 100 ms on, 40 ms off
  drive(ctl, io, 3000, o => { base(o); o.PLT_PRESENT = o.SOL_LIFT && (o.t % 140) < 100; });
  chk('pallet-line: a flickering pallet-present switch does not load the pallet',
    io.ST1_STEP === 40 && io.EM_W_CNT === 0, 'step ' + io.ST1_STEP + ', parts loaded ' + io.EM_W_CNT);
  drive(ctl, io, 400, o => { base(o); o.PLT_PRESENT = o.SOL_LIFT; });       // steady: the confirm can run
  chk('pallet-line: a steady pallet-present signal loads it after the confirm time',
    io.ST1_STEP >= 50 && io.EM_W_CNT === 1, 'step ' + io.ST1_STEP + ', parts loaded ' + io.EM_W_CNT);
}

process.exit(fail ? 1 : 0);
