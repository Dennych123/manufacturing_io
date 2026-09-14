// The scene controllers on their own, with the plant faked: the sequence discipline that only
// shows up against a real PLC, where the plant's counters move whenever physics gets round to
// it and not while the step that waits for them happens to be active.
//
// Measured on the Sysmac simulator with scenes/a-to-b: a part removed while the sequence was
// still loading the next one made the discharge step see "the count moved" immediately, so the
// cycle never waited for its own part. 231 parts went in and 217 came out, and the crowd of
// parts crossing the end sensor raised 13 pulse-stretch warnings.
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

/** A scan loop over a plain io object; `each` runs before every scan. */
function drive(ctl, io, ms, each = () => {}) {
  for (let i = 0; i < ms / 2; i++) { io.t = (io.t || 0) + 2; each(io, io.t); ctl.scan(io, io.t); }
  return io;
}

// ---------------------------------------------------------------- a-to-b
{
  const { create } = await import('../scenes/a-to-b.ctl.js');
  /** Runs one loaded part through the cycle. `straggler` fires an unloader count during step 10. */
  const cycle = straggler => {
    const ctl = create();
    const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0, EM1_CNT: 0, RM1_CNT: 0, PE_END: false, CV1_RUN: false, EM1_EMIT: false };
    drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; });
    let loaded = false, straggled = false;
    drive(ctl, io, 4000, o => {
      if (o.ST1_STEP === 10 && o.EM1_EMIT && !loaded) { o.EM1_CNT++; loaded = true; }         // the loader answers
      if (o.ST1_STEP === 10 && straggler && !straggled) { o.RM1_CNT++; straggled = true; }    // an older part clears now
      if (o.ST1_STEP === 20 && o.CV1_RUN) o.PE_END = true;                                    // the part reaches the eye
    });
    return io;
  };
  const clean = cycle(false);
  chk('a-to-b: after the dwell the cycle waits at the discharge step for its own part', clean.ST1_STEP === 40 && clean.CYCLE_CNT === 0,
    'step ' + clean.ST1_STEP + ', CYCLE_CNT ' + clean.CYCLE_CNT);
  const raced = cycle(true);
  chk('a-to-b: an unloader count from an EARLIER part does not end this cycle', raced.ST1_STEP === 40 && raced.CYCLE_CNT === 0,
    'step ' + raced.ST1_STEP + ', CYCLE_CNT ' + raced.CYCLE_CNT);

  // A part still sitting in the beam belongs to the last cycle: the belt must clear it before
  // this part is conveyed, or the dwell happens with the part halfway down the belt.
  const ctl = create();
  const io = { t: 0, PB_START: false, ST1_STEP: 0, CYCLE_CNT: 0, EM1_CNT: 0, RM1_CNT: 0, PE_END: true, CV1_RUN: false, EM1_EMIT: false };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; });
  let loaded = false;
  drive(ctl, io, 600, o => { if (o.ST1_STEP === 10 && o.EM1_EMIT && !loaded) { o.EM1_CNT++; loaded = true; } });
  chk('a-to-b: with the eye still blocked, the sequence runs the belt and waits for it to clear', io.ST1_STEP === 15 && io.CV1_RUN === true,
    'step ' + io.ST1_STEP + ', CV1_RUN ' + io.CV1_RUN);
  drive(ctl, io, 200, o => { o.PE_END = false; });
  chk('a-to-b: once clear it conveys this part to the eye', io.ST1_STEP === 20);
}

// ---------------------------------------------------------------- pick-place
{
  const { create } = await import('../scenes/pick-place.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, ST1_STEP: 0, CYCLE_CNT: 0, EM_CNT: 0, RM_CNT: 0, NEST_A_P: false, AS_Z_UP: true, AS_Z_DN: false,
               SV_INPOS: true, SV_DONE: false, VAC_SW: false, CV_RUN: false, EM_EMIT: false, SOL_Z_DN: false, SOL_Z_UP: true, VAC_ON: false, SV_EXEC: false, CLAMP_A: false };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; });
  let loaded = false;
  // Walk the whole sequence with the plant answering each command, and an unloader count from an
  // earlier part arriving while the part is still being picked.
  drive(ctl, io, 6000, o => {
    if (o.ST1_STEP === 10 && o.EM_EMIT && !loaded) { o.EM_CNT++; loaded = true; }
    if (o.ST1_STEP === 20) { o.NEST_A_P = true; o.RM_CNT++; }          // a straggler clears during the pick
    o.AS_Z_DN = o.SOL_Z_DN; o.AS_Z_UP = o.SOL_Z_UP;
    o.VAC_SW = o.VAC_ON;
    o.SV_DONE = o.SV_EXEC;
  });
  chk('pick-place: the cycle ends up waiting for ITS part at the unloader', io.ST1_STEP === 130 && io.CYCLE_CNT === 0,
    'step ' + io.ST1_STEP + ', CYCLE_CNT ' + io.CYCLE_CNT);
  drive(ctl, io, 100, o => { o.RM_CNT++; });
  chk('pick-place: its own part arriving completes the cycle', io.CYCLE_CNT === 1 && io.ST1_STEP === 10, 'step ' + io.ST1_STEP + ', CYCLE_CNT ' + io.CYCLE_CNT);
}

process.exit(fail ? 1 : 0);
