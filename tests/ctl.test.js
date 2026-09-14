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

process.exit(fail ? 1 : 0);
