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
  for (const f of fs.readdirSync(dir).filter(f => f.endsWith('.ctl.js')).sort()) {
    const name = f.slice(0, -7), stFile = path.join(dir, name + '.st');
    if (!fs.existsSync(stFile)) { console.log('  SKIP  ' + name + ': no .st beside the .ctl.js'); continue; }
    const js = steps(fs.readFileSync(path.join(dir, f), 'utf8'), /\bcase (HOME|FAULT|ESTOP|\d+):/g);
    const st = steps(fs.readFileSync(stFile, 'utf8'), /^\s*(\d+):/gm);
    chk(name + ': .st and .ctl.js handle the same steps', js === st, 'st [' + st + '] js [' + js + ']');
  }
}

process.exit(fail ? 1 : 0);
