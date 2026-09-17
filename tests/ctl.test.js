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

/** One panel button pressed and released. */
function tap(ctl, io, tag, each = () => {}) {
  const t0 = io.t;
  drive(ctl, io, 10, (o, t) => { o[tag] = t - t0 <= 4; each(o, t); });
  io[tag] = false;
}

/**
 * The panel start-up, in the order an operator does it: MASTER ON energises the machine, HOME
 * drives every actuator to its home position, and only then does START run the cycle. Without
 * the home the sequence refuses to start, which is the point of the Home button.
 */
function powerUp(ctl, io, each = () => {}) {
  tap(ctl, io, 'PB_MASTER', each);
  tap(ctl, io, 'PB_HOME', each);
  drive(ctl, io, 2000, each);
  tap(ctl, io, 'PB_START', each);
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
    const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0,
                 EM1_CNT: 3, RM1_CNT: 3 - onBelt, PE_END: false, CV1_RUN: false, EM1_EMIT: false };
    powerUp(ctl, io);
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
    powerUp(ctl, io);
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
  powerUp(ctl, io, o => { o.AS_Z_DN = o.SOL_Z_DN; o.AS_Z_UP = o.SOL_Z_UP; o.SV_DONE = o.SV_EXEC; });
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
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV1_RUN: false, EM1_EN: false,
               SOL_STOP: false, AS_STOP_DN: false, AS_STOP_UP: true, PE_STOP: false,
               SOL_PUSH: false, AS_PUSH_EXT: false, AS_PUSH_RET: true };
  const plant = o => { o.AS_STOP_DN = o.SOL_STOP; o.AS_STOP_UP = !o.SOL_STOP; o.AS_PUSH_EXT = o.SOL_PUSH; o.AS_PUSH_RET = !o.SOL_PUSH; };
  powerUp(ctl, io, plant);
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
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0, TBL_RUN: false, TBL_INPOS: true, TBL_STATION: 0, TBL_ORIGIN: true,
               EM_B_EMIT: false, EM_B_CNT: 5, EM_L_EMIT: false, EM_L_CNT: 5,
               SOL_PRESS_DN: false, SOL_PRESS_UP: true, AS_PRESS_DN: false, AS_PRESS_UP: true };
  const plant = o => {
    o.AS_PRESS_DN = o.SOL_PRESS_DN; o.AS_PRESS_UP = o.SOL_PRESS_UP;
    o.TBL_ORIGIN = o.TBL_INPOS && o.TBL_STATION === 0;                       // as the index table reports it
    if (o.EM_B_EMIT) o.EM_B_CNT++; if (o.EM_L_EMIT) o.EM_L_CNT++;
  };
  powerUp(ctl, io, plant);
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
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV_RUN: false,
               EM_T_EMIT: false, EM_T_CNT: 4, EM_S_EMIT: false, EM_S_CNT: 4, RM_T_CNT: 3, RM_S_CNT: 4,
               PE_LOW: false, PE_HIGH: false, SOL_PUSH: false, AS_PUSH_EXT: false, AS_PUSH_RET: true };
  const plant = o => { o.AS_PUSH_EXT = o.SOL_PUSH; o.AS_PUSH_RET = !o.SOL_PUSH; if (o.EM_S_EMIT) o.EM_S_CNT++; if (o.EM_T_EMIT) o.EM_T_CNT++; };
  powerUp(ctl, io, plant);
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
// The escapement's two pins must never be open at once, or the whole line runs through the gate.
{
  const { create } = await import('../scenes/buffer-queue.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV_RUN: false,
               EM_EMIT: false, EM_CNT: 0, RM_CNT: 0, PE_HOLD: false, PE_GATE: false,
               SOL_HOLD: false, SOL_GATE: false, AS_HOLD_UP: true, AS_HOLD_DN: false,
               AS_GATE_UP: true, AS_GATE_DN: false, AUTO_RUN: false };
  // The pins answer their own solenoids, and the loader answers the feed command.
  const plant = o => {
    o.AS_HOLD_DN = o.SOL_HOLD; o.AS_HOLD_UP = !o.SOL_HOLD;
    o.AS_GATE_DN = o.SOL_GATE; o.AS_GATE_UP = !o.SOL_GATE;
    if (o.EM_EMIT) o.EM_CNT++;
  };
  powerUp(ctl, io, plant);
  drive(ctl, io, 500, plant);
  chk('buffer-queue: it arms both pins, feeds one part and waits for it at the hold pin',
    io.ST1_STEP === 25 && io.SOL_HOLD === true && io.SOL_GATE === true && io.EM_CNT === 1,
    'step ' + io.ST1_STEP + ', EM ' + io.EM_CNT);
  let bothOpen = 0;
  const watch = o => { plant(o); if (!o.SOL_HOLD && !o.SOL_GATE && o.AUTO_RUN) bothOpen++; };
  drive(ctl, io, 400, o => { o.PE_HOLD = true; watch(o); });
  chk('buffer-queue: the part settles, then the hold pin lifts to let it through', io.ST1_STEP === 40 || io.ST1_STEP === 45,
    'step ' + io.ST1_STEP + ', hold ' + io.SOL_HOLD);
  drive(ctl, io, 200, o => { o.PE_HOLD = false; watch(o); });
  chk('buffer-queue: the hold pin comes down again behind it', io.ST1_STEP === 60 && io.SOL_HOLD === true, 'step ' + io.ST1_STEP);
  drive(ctl, io, 1800, o => { o.PE_GATE = true; watch(o); });
  chk('buffer-queue: it waits in the pocket for the demand, then opens the gate',
    io.ST1_STEP === 80 && io.SOL_GATE === false, 'step ' + io.ST1_STEP + ', gate ' + io.SOL_GATE);
  drive(ctl, io, 200, o => { o.PE_GATE = false; watch(o); });
  chk('buffer-queue: with the part gone the gate closes again', io.ST1_STEP === 95 && io.SOL_GATE === true, 'step ' + io.ST1_STEP);
  chk('buffer-queue: the two pins are never open at the same time while it runs', bothOpen === 0, bothOpen + ' scans with both pins up');
  drive(ctl, io, 100, o => { o.RM_CNT = o.EM_CNT; watch(o); });
  chk('buffer-queue: the part reaching the outfeed ends the cycle and the next one is let in',
    io.CYCLE_CNT === 1 && io.ST1_STEP >= 20 && io.ST1_STEP < 40, 'CYCLE_CNT ' + io.CYCLE_CNT + ', step ' + io.ST1_STEP);
}

// ---------------------------------------------------------------- palletizing
// A pallet holds 100 plugs and one cycle takes five, so the twentieth cycle empties it. The
// machine must then call for a fresh pallet instead of standing there with nothing to pick - the
// twenty cycles are too slow to drive through the physics, so the sequence is checked here.
{
  const { create } = await import('../scenes/palletizing.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0,
               EM_PAL_EN: false, EM_PAL_CNT: 0, PAL_CLAMP: false, JIG_CLAMP: false,
               AS_Z_UP: true, AS_Z_DN: false, AS_U_UP: true, AS_U_DN: false, AS_U_JIG: true, AS_U_BIN: false,
               GX_DONE: false, GY_DONE: false, GX_INPOS: true, GY_INPOS: true,
               TBL_RUN: false, TBL_INPOS: true, TBL_STATION: 0, TBL_ORIGIN: true,
               VAC_ON: false, UVAC_ON: false, AUTO_RUN: false };
  // The faked plant: every axis answers its own command, and the loader counts what it is asked for.
  let inTray = 0, lastStep = -1;
  const plant = o => {
    o.AS_Z_DN = o.SOL_Z_DN; o.AS_Z_UP = o.SOL_Z_UP;
    o.AS_U_DN = o.SOL_U_DN; o.AS_U_UP = o.SOL_U_UP;
    o.AS_U_JIG = o.SOL_U_JIG; o.AS_U_BIN = o.SOL_U_BIN;
    o.GX_DONE = o.GX_EXEC; o.GY_DONE = o.GY_EXEC;
    for (let k = 1; k <= 5; k++) { o['VAC_SW' + k] = o.VAC_ON; o['UVAC_SW' + k] = o.UVAC_ON; }
    // The tray holds 100 and the loader stops when it is full, which is how the sequence knows.
    if (o.EM_PAL_EN && inTray < 100) { o.EM_PAL_CNT += 1; inTray++; }
    if (o.ST1_STEP === 40 && lastStep !== 40) inTray = Math.max(0, inTray - 5);   // the head took five
    lastStep = o.ST1_STEP;
    if (o.TBL_RUN) { o.TBL_STATION = (o.TBL_STATION + 1) % 4; o.TBL_ORIGIN = o.TBL_STATION === 0; }
  };
  powerUp(ctl, io, plant);
  drive(ctl, io, 1600, plant);                                             // the loader drops one a scan
  chk('palletizing: START calls for a pallet before it picks anything', io.EM_PAL_CNT >= 100 && io.ST1_STEP >= 10,
    'EM_PAL_CNT ' + io.EM_PAL_CNT + ', step ' + io.ST1_STEP);
  chk('palletizing: the loader is switched off once the tray is full', io.EM_PAL_EN === false);
  const afterFirst = io.EM_PAL_CNT;
  // Twenty cycles of five empty the tray. Drive them and watch for the next pallet.
  drive(ctl, io, 20000, plant);
  chk('palletizing: it works through the whole tray, five at a time', io.CYCLE_CNT >= 20, 'CYCLE_CNT ' + io.CYCLE_CNT);
  chk('palletizing: an empty tray calls for a fresh pallet instead of standing still',
    io.EM_PAL_CNT >= afterFirst + 100 && io.ST1_STEP !== 900,
    'loaded ' + io.EM_PAL_CNT + ' plugs in ' + io.CYCLE_CNT + ' cycles, step ' + io.ST1_STEP);
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
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0, EM1_CNT: 0, RM1_CNT: 0,
               PE_END: false, CV1_RUN: false, EM1_EMIT: false, AUTO_RUN: false, PL_START: false };
  const press = () => { drive(ctl, io, 8, o => { o.PB_START = true; }); drive(ctl, io, 8, o => { o.PB_START = false; }); };
  const feed = ms => {
    let loaded = false;
    drive(ctl, io, ms, o => {
      if (o.ST1_STEP === 10 && o.EM1_EMIT && !loaded) { o.EM1_CNT++; loaded = true; }
      if (o.ST1_STEP === 20 && o.CV1_RUN) o.PE_END = true;
    });
  };
  powerUp(ctl, io);
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
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV_RUN: false, EM_EMIT: false,
               EM_CNT: 0, RM_CNT: 0, NEST_A_P: false, CLAMP_A: false, VAC_ON: false, VAC_SW: false,
               SOL_Z_UP: false, SOL_Z_DN: false, AS_Z_UP: true, AS_Z_DN: false, SV_TGT: 0, SV_EXEC: false,
               SV_DONE: false, SV_INPOS: true, AUTO_RUN: false, PL_START: false };
  powerUp(ctl, io, o => { o.SV_DONE = o.SV_EXEC; });
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
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0, EM1_CNT: 0, RM1_CNT: 0,
               PE_END: false, CV1_RUN: false, EM1_EMIT: false, AUTO_RUN: false, PL_START: false };
  const press = () => { drive(ctl, io, 8, o => { o.PB_START = true; }); drive(ctl, io, 8, o => { o.PB_START = false; }); };
  powerUp(ctl, io);
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
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV1_RUN: false, EM1_EN: false,
               SOL_STOP: false, SOL_PUSH: false, AS_STOP_DN: false, AS_STOP_UP: true, AS_PUSH_EXT: false, AS_PUSH_RET: true,
               PE_STOP: false, AUTO_RUN: false, PL_START: false };
  const plant = o => { o.AS_STOP_DN = o.SOL_STOP; o.AS_STOP_UP = !o.SOL_STOP; o.AS_PUSH_EXT = o.SOL_PUSH; o.AS_PUSH_RET = !o.SOL_PUSH; };
  powerUp(ctl, io, plant);
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
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV_RUN: false,
               EM_T_EMIT: false, EM_T_CNT: 4, EM_S_EMIT: false, EM_S_CNT: 4, RM_T_CNT: 4, RM_S_CNT: 4,
               PE_LOW: false, PE_HIGH: false, SOL_PUSH: false, AS_PUSH_EXT: false, AS_PUSH_RET: true, AUTO_RUN: false };
  const plant = o => { o.AS_PUSH_EXT = o.SOL_PUSH; o.AS_PUSH_RET = !o.SOL_PUSH; if (o.EM_S_EMIT) o.EM_S_CNT++; if (o.EM_T_EMIT) o.EM_T_CNT++; };
  const press = () => { drive(ctl, io, 8, o => { o.PB_START = true; plant(o); }); drive(ctl, io, 8, o => { o.PB_START = false; plant(o); }); };
  powerUp(ctl, io, plant);
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
  // The escapement feeds a part and waits for it at the hold pin. If that part is taken off the
  // belt it never arrives, and step 25 would wait for ever.
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0, CV_RUN: false,
               EM_EMIT: false, EM_CNT: 0, RM_CNT: 0, PE_HOLD: false, PE_GATE: false,
               SOL_HOLD: false, SOL_GATE: false, AS_HOLD_UP: true, AS_HOLD_DN: false,
               AS_GATE_UP: true, AS_GATE_DN: false, AUTO_RUN: false };
  const plant = o => {
    o.AS_HOLD_DN = o.SOL_HOLD; o.AS_HOLD_UP = !o.SOL_HOLD;
    o.AS_GATE_DN = o.SOL_GATE; o.AS_GATE_UP = !o.SOL_GATE;
    if (o.EM_EMIT) o.EM_CNT++;
  };
  powerUp(ctl, io, plant);
  drive(ctl, io, 500, plant);                                              // the part never reaches the hold pin
  chk('buffer-queue: it waits at the hold pin for a part that was taken', io.ST1_STEP === 25, 'step ' + io.ST1_STEP);
  drive(ctl, io, 16000, plant);
  chk('buffer-queue: a part that never arrives faults instead of waiting for ever',
    io.ST1_STEP === 900 && io.AUTO_RUN === false && io.CV_RUN === false, 'step ' + io.ST1_STEP);
  chk('buffer-queue: the fault holds the feeder off too', io.EM_EMIT === false);
}

{
  const { create } = await import('../scenes/assembler.ctl.js');
  const ctl = create();
  const io = { t: 0, PB_START: false, PB_CSTOP: false, ST1_STEP: 0, CYCLE_CNT: 0, TBL_RUN: false, TBL_INPOS: true,
               TBL_STATION: 0, TBL_ORIGIN: true, EM_B_EMIT: false, EM_B_CNT: 0, EM_L_EMIT: false, EM_L_CNT: 0,
               SOL_PRESS_DN: false, SOL_PRESS_UP: true, AS_PRESS_UP: true, AS_PRESS_DN: false, AUTO_RUN: false };
  // The press answers its solenoids and the table reports its origin; the base FEEDER never does,
  // which is the jam under test.
  const plant = o => { o.AS_PRESS_DN = o.SOL_PRESS_DN; o.AS_PRESS_UP = o.SOL_PRESS_UP; o.TBL_ORIGIN = o.TBL_INPOS && o.TBL_STATION === 0; };
  powerUp(ctl, io, plant);
  drive(ctl, io, 500, plant);                                              // the base feeder is blocked: its count never moves
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
  const io = { t: 0, PB_START: false, PB_CSTOP: false, PB_MASTER: false, PB_ESTOP: false, PB_HOME: false,
               SEL_AUTO: true, OVR_SET: 100, PL_MASTER: false, PL_HOME: false,
               ST1_STEP: 0, CYCLE_CNT: 0, CV1_RUN: false,
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
  powerUp(ctl, io, plant);
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
  const io = { t: 0, PB_START: false, PB_CSTOP: false, PB_MASTER: false, PB_ESTOP: false, PB_HOME: false,
               SEL_AUTO: true, OVR_SET: 100, PL_MASTER: false, PL_HOME: false,
               ST1_STEP: 0, CYCLE_CNT: 0, CV1_RUN: false,
               EM_P_EMIT: false, EM_P_CNT: 0, EM_W_EMIT: false, EM_W_CNT: 0, RM_CNT: 0, PE_STN: false,
               SOL_STOP: false, AS_STOP_UP: false, AS_STOP_DN: true,
               SOL_LIFT: false, AS_LIFT_UP: false, AS_LIFT_DN: true, PLT_PRESENT: false, AUTO_RUN: false };
  const base = o => {
    o.AS_STOP_UP = o.SOL_STOP; o.AS_STOP_DN = !o.SOL_STOP;
    o.AS_LIFT_UP = o.SOL_LIFT; o.AS_LIFT_DN = !o.SOL_LIFT;
    if (o.EM_P_EMIT) o.EM_P_CNT++;
    if (o.EM_W_EMIT) o.EM_W_CNT++;
  };
  powerUp(ctl, io, o => { base(o); o.PLT_PRESENT = false; });
  drive(ctl, io, 1400, o => { base(o); o.PLT_PRESENT = false; if (o.ST1_STEP === 30 && o.CV1_RUN) o.PE_STN = true; });
  // at the stop, lifting, but the present switch chatters: 100 ms on, 40 ms off
  drive(ctl, io, 3000, o => { base(o); o.PLT_PRESENT = o.SOL_LIFT && (o.t % 140) < 100; });
  chk('pallet-line: a flickering pallet-present switch does not load the pallet',
    io.ST1_STEP === 40 && io.EM_W_CNT === 0, 'step ' + io.ST1_STEP + ', parts loaded ' + io.EM_W_CNT);
  drive(ctl, io, 400, o => { base(o); o.PLT_PRESENT = o.SOL_LIFT; });       // steady: the confirm can run
  chk('pallet-line: a steady pallet-present signal loads it after the confirm time',
    io.ST1_STEP >= 50 && io.EM_W_CNT === 1, 'step ' + io.ST1_STEP + ', parts loaded ' + io.EM_W_CNT);
}

// ---------------------------------------------------------------- blurobot (the rb4axis cell)
// Three things this cell taught, each measured before it was fixed:
//   - a 2-finger grip is confirmed by the OPEN switch dropping, never by `closed`: `closed` sits
//     at full close, so with a part between the fingers it never comes on;
//   - the cycle must re-snapshot the feeder count when it restarts, or step 20 sees "the count
//     already moved" and drops the emit command without ever feeding a part;
//   - every waiting step still needs the watchdog.
{
  const { create } = await import('../scenes/blurobot.ctl.js');
  const ctl = create();
  const AX = ['A0', 'A1', 'A2', 'A3'];
  const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0,
               CV_IN_RUN: false, CV_OUT_RUN: false, EM_EMIT: false, EM_CNT: 0, RM_CNT: 0,
               PE_IN: false, NEST_CLAMP: false, GRIP_CLOSE: false, GRIP_OPEN: true, GRIP_CLOSED: false,
               AUTO_RUN: false, PL_START: false };
  for (const a of AX) { io[a + '_TGT'] = 0; io[a + '_EXEC'] = false; io[a + '_DONE'] = false; io[a + '_BUSY'] = false; io[a + '_INPOS'] = true; }
  // The axes answer Execute at once, and Done falls with it. The gripper's fingers leave the open
  // switch as soon as it is told to close, and `closed` NEVER comes on: a part is between them.
  const plant = o => {
    for (const a of AX) o[a + '_DONE'] = o[a + '_EXEC'];
    o.GRIP_OPEN = !o.GRIP_CLOSE;
    o.GRIP_CLOSED = false;
    if (o.EM_EMIT) o.EM_CNT++;
    if (o.CV_IN_RUN && o.EM_CNT > 0) o.PE_IN = true;           // the fed part reaches the nest
    if (o.CV_OUT_RUN) o.RM_CNT = o.EM_CNT;                      // the outfeed clears what was fed
  };
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; plant(o); });
}

// ---------------------------------------------------------------- blurobot (the rb4axis cell)
// A port of PRG_SIM_ROBOT: six stations, three job priorities, covers that gate the test clocks.
// Four things this port taught, each measured before it was fixed.
{
  const { create } = await import('../scenes/blurobot.ctl.js');
  const AX = ['A0', 'A1', 'A2', 'A3'];
  const NEST = ['WIPIN', 'ICC1', 'ICC2', 'DW1', 'DW2', 'WIPOUT'];
  const CV = ['ICC1', 'ICC2', 'DW1', 'DW2'];
  const fresh = () => {
    const io = { t: 0, PB_START: false, PB_STOP: false, ST1_STEP: 0, CYCLE_CNT: 0,
                 GRIP_CLOSE: false, GRIP_OPEN: true, GRIP_CLOSED: false,
                 EM_IN_EMIT: false, EM_IN_CNT: 0, AUTO_RUN: false, PL_START: false };
    for (const a of AX) { io[a + '_TGT'] = 0; io[a + '_EXEC'] = false; io[a + '_DONE'] = false; io[a + '_POS'] = 0; }
    for (const n of NEST) { io[n + '_CLAMP'] = false; io[n + '_P'] = n === 'WIPIN'; }
    for (const c of CV) { io[c + '_CV_TGT'] = 80; io[c + '_CV_EXEC'] = false; io[c + '_CV_POS'] = 80; io[c + '_CV_DONE'] = false; }
    return io;
  };
  // The plant: axes answer Execute at once, the covers only move when Execute is RE-PULSED (the
  // joint latches its target on the rising edge), and the jaws stop at the board's width so
  // `closed` never comes on - which is exactly what a good grip looks like.
  const plant = o => {
    for (const a of AX) { o[a + '_DONE'] = o[a + '_EXEC']; if (o[a + '_EXEC']) o[a + '_POS'] = o[a + '_TGT']; }
    for (const c of CV) {
      if (!o[c + '_CV_EXEC']) o[c + '_CV_ARMED'] = true;
      else if (o[c + '_CV_ARMED']) { o[c + '_CV_POS'] = o[c + '_CV_TGT']; o[c + '_CV_ARMED'] = false; }
    }
    o.GRIP_OPEN = !o.GRIP_CLOSE;
    o.GRIP_CLOSED = false;
    // Taking the board EMPTIES the rack: the nest releases (clamp off) and the fingers close on
    // it. Without this the rack reads full for ever, the controller never asks for a refill, and
    // the "restock" it is supposed to do can never be observed.
    if (o.GRIP_CLOSE && !o.WIPIN_CLAMP) o.WIPIN_P = false;
    if (o.EM_IN_EMIT) { o.EM_IN_CNT++; o.WIPIN_P = true; }
  };
  const ctl = create();
  const io = fresh();
  drive(ctl, io, 10, (o, t) => { o.PB_START = t <= 4; plant(o); });
  // Long enough for the first job to matter: the arm has to fold, rail 1400 mm to WIP IN, descend,
  // grip, lift, fold, rail back to an ICC and place. 2 s only got as far as the first traverse.
  drive(ctl, io, 12000, plant);

  // WIP IN is an endless stock: the PLC pins it full, so the plant has to be asked to refill it
  // every time a board is taken, or the next grip closes on air.
  chk('blurobot: taking a board from WIP IN makes it restock', io.EM_IN_CNT >= 1, 'restocks ' + io.EM_IN_CNT);

  // The cover gates the test clock, so a cover that never moves means a cell that never finishes.
  // Both ICCs get filled first - that is priority 1, and it is what "buffer" means - after which
  // the cell correctly IDLES at step 0 while two 17 s tests run. Idling there is not a stall.
  chk('blurobot: both ICCs are loaded and testing with their covers shut',
    io.ICC1_CV_POS === 0 && io.ICC2_CV_POS === 0, `ICC1 ${io.ICC1_CV_POS}, ICC2 ${io.ICC2_CV_POS}, step ${io.ST1_STEP}`);

  // Long enough for both ICCs (17 s) and a DW (15 s) to finish and for boards to leave.
  drive(ctl, io, 90000, plant);
  chk('blurobot: boards reach WIP OUT', io.CYCLE_CNT >= 1, 'cycles ' + io.CYCLE_CNT);
  chk('blurobot: it never faults during normal running', io.ST1_STEP !== 900, 'step ' + io.ST1_STEP);

  // A grip that keeps slipping must not be confirmed, and must fault rather than wait for ever.
  const ctl2 = create();
  const io2 = fresh();
  const slipping = o => { plant(o); o.GRIP_OPEN = true; };     // the fingers never stay closed
  drive(ctl2, io2, 10, (o, t) => { o.PB_START = t <= 4; slipping(o); });
  drive(ctl2, io2, 4000, slipping);
  chk('blurobot: a grip that never holds is not confirmed', io2.CYCLE_CNT === 0, 'cycles ' + io2.CYCLE_CNT);
  drive(ctl2, io2, 32000, slipping);
  chk('blurobot: and it faults instead of waiting for ever', io2.ST1_STEP === 900 && io2.AUTO_RUN === false,
    'step ' + io2.ST1_STEP);
}

// ---------------------------------------------------------------- robot-pitch (LR Mate + cam head)
// One cycle = one pallet row: cam wide, pick five at 100, cam narrow, set into the jig at 60, the
// jig's process, pick again, drop in the bin, cam wide. The fake plant answers Execute with Done
// after 40 ms, builds vacuum 100 ms after VAC_ON, and delivers a plug every 40 ms while the loader
// is enabled, 25 to a pallet.
{
  const { create } = await import('../scenes/robot-pitch.ctl.js');
  const fresh = () => {
    const io = { t: 0, PB_START: false, PB_CSTOP: false, PB_MASTER: false, PB_ESTOP: false, PB_HOME: false, SEL_AUTO: true, OVR_SET: 100,
                 ST1_STEP: 0, CYCLE_CNT: 0, EM_PAL_CNT: 0, EM_PAL_EN: false, RM_BIN_CNT: 0, VAC_ON: false, PAL_CLAMP: true, JIG_CLAMP: true,
                 CAM_TGT: 0, CAM_EXEC: false, CAM_DONE: false, CAM_POS: 0, AUTO_RUN: false };
    for (let i = 1; i <= 6; i++) Object.assign(io, { ['J' + i + '_TGT']: 0, ['J' + i + '_EXEC']: false, ['J' + i + '_DONE']: false, ['J' + i + '_POS']: 0 });
    for (let i = 0; i < 5; i++) io['VAC_C' + i] = false;
    return io;
  };
  const fake = (vacWorks = true) => {
    let vacFrom = -1, fillLast = 0, filled = 0, enLast = false;
    /** @type {Record<string, number>} */
    const execFrom = {};
    return (/** @type {any} */ o, /** @type {number} */ t) => {
      for (const a of ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'CAM']) {
        if (o[a + '_EXEC']) { if (execFrom[a] == null) execFrom[a] = t; o[a + '_DONE'] = t - execFrom[a] >= 40; }
        else { delete execFrom[a]; o[a + '_DONE'] = false; }
      }
      if (o.VAC_ON) { if (vacFrom < 0) vacFrom = t; } else vacFrom = -1;
      for (let i = 0; i < 5; i++) o['VAC_C' + i] = vacWorks && vacFrom >= 0 && t - vacFrom >= 100;
      if (o.EM_PAL_EN && !enLast) filled = 0;
      enLast = o.EM_PAL_EN;
      if (o.EM_PAL_EN && filled < 25 && t - fillLast >= 40) { o.EM_PAL_CNT++; filled++; fillLast = t; }
    };
  };
  const ctl = create(), io = fresh(), plant = fake();
  const seen = { camAtJig: null, camAtPallet: null, palClampAtPick: null, jigClampAtPlace: null, vacAtPlace: null };
  powerUp(ctl, io, plant);
  drive(ctl, io, 40000, o => {
    plant(o, o.t);
    // Sampled at the steps that USE the pitch, not at the step that commands it: `each` runs
    // before the scan, so at the commanding step the target is still the previous one.
    if (o.ST1_STEP === 21 && seen.camAtJig == null) seen.camAtJig = o.CAM_TGT;
    if (o.ST1_STEP === 14 && o.CYCLE_CNT > 0 && seen.camAtPallet == null) seen.camAtPallet = o.CAM_TGT;
    if (o.ST1_STEP === 14) seen.palClampAtPick = o.PAL_CLAMP;
    if (o.ST1_STEP === 21) { seen.jigClampAtPlace = o.JIG_CLAMP; seen.vacAtPlace = o.VAC_ON; }
  });
  chk('robot-pitch: rows complete as cycles', io.CYCLE_CNT >= 3, 'CYCLE_CNT ' + io.CYCLE_CNT + ', step ' + io.ST1_STEP);
  chk('robot-pitch: the cam is narrow (60 mm) on the way to the jig and wide (100 mm) back at the pallet', seen.camAtJig === 90 && seen.camAtPallet === 0, JSON.stringify(seen));
  chk('robot-pitch: the pallet lets go while the cups take the row', seen.palClampAtPick === false);
  chk('robot-pitch: the jig holds and the vacuum is off when the row is set down', seen.jigClampAtPlace === true && seen.vacAtPlace === false);
  chk('robot-pitch: it never faults during normal running', io.ST1_STEP !== 900, 'step ' + io.ST1_STEP);
  // A pallet is filled in whole 5 x 5 loads, never part of one: the loader is asked for a fresh
  // pallet when the fifth row has gone, and it delivers 25 before the arm picks again.
  chk('robot-pitch: pallets are loaded whole, 25 at a time', io.EM_PAL_CNT >= 25 && io.EM_PAL_CNT % 25 === 0, 'EM_PAL_CNT ' + io.EM_PAL_CNT);

  // A row where nothing comes up (the plugs were taken out by hand) is not a cycle: the head
  // carries on to the next row instead of standing over an empty one for ever.
  const ctl2 = create(), io2 = fresh(), plant2 = fake(false);
  powerUp(ctl2, io2, plant2);
  drive(ctl2, io2, 20000, o => plant2(o, o.t));
  chk('robot-pitch: an empty row is skipped, not counted, and does not fault', io2.CYCLE_CNT === 0 && io2.ST1_STEP !== 900 && io2.AUTO_RUN === true,
    'CYCLE_CNT ' + io2.CYCLE_CNT + ', step ' + io2.ST1_STEP);
}

// ---------------------------------------------------------------- .st and .ctl.js in step
// Every .ctl.js says "keep the two in step". Nothing checked it: the watchdog step (900) can be
// added to one file and not the other, and neither the plant nor the simulator would notice.
// The step numbers each CASE handles are the cheapest thing that must agree.
{
  const fs = await import('node:fs');
  const path = await import('node:path');
  const { fileURLToPath } = await import('node:url');
  const dir = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'scenes');
  const NAMED = { HOME: 800, FAULT: 900, ESTOP: 910 };
  const steps = (txt, re) => [...txt.matchAll(re)].map(m => NAMED[m[1]] ?? +m[1]).sort((a, b) => a - b).join(' ');
  // Only the SEQUENCE case block counts. blurobot.st also carries small LOOKUP case blocks (rail
  // position by station, cover-OK by station), and `^\s*(\d+):` cannot tell one of those labels
  // from a step: they came through as steps 0-5 twice over and failed a pair of files that were
  // perfectly in step. So take every CASE ST<n>_STEP OF ... END_CASE block and nothing else:
  // palletizing runs THREE stations at once (CASE ST2_STEP, CASE ST3_STEP), and taking only the
  // first block would drop two thirds of it. Measured: these blocks never nest - palletizing's run
  // 107-290, 299-315 and 322-384, and blurobot's two lookups both close before its sequence opens.
  const seq = txt => {
    const blocks = [...txt.matchAll(/CASE\s+ST\d*_STEP\s+OF([\s\S]*?)END_CASE/gi)].map(m => m[1]);
    return blocks.length ? blocks.join('\n') : txt;
  };
  for (const f of fs.readdirSync(dir).filter(f => f.endsWith('.ctl.js')).sort()) {
    const name = f.slice(0, -7), stFile = path.join(dir, name + '.st');
    if (!fs.existsSync(stFile)) { console.log('  SKIP  ' + name + ': no .st beside the .ctl.js'); continue; }
    const js = steps(fs.readFileSync(path.join(dir, f), 'utf8'), /\bcase (HOME|FAULT|ESTOP|\d+):/g);
    const st = steps(seq(fs.readFileSync(stFile, 'utf8')), /^\s*(\d+):/gm);
    chk(name + ': .st and .ctl.js handle the same steps', js === st, 'st [' + st + '] js [' + js + ']');
  }
}

process.exit(fail ? 1 : 0);
