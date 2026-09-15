# Working notes for Claude: `manufacturing_io`

The plan is in [docs/PLAN.md](docs/PLAN.md). This file holds the rules that are **invisible
in the code but break things silently** when violated. Most were paid for once in
[rb4axis](https://github.com/Dennych123/rb4axis).

## Design rules

- **The plant runs in Node. The browser only renders, edits and analyses.** Rapier never loads in
  the page, and a second sequence never runs in the page.
- **One source for numbers and names: the scene JSON.** Tag names, dimensions and stroke times
  live there and nowhere else. A copied number drifts, and the drift is silent.
- **`lib/` is imported by both Node and the browser, so it never imports three or Rapier.**
  Component geometry is plain data: Node builds colliders from it and the browser builds meshes.
- **Moving-part poses come from `worldPoses()` in `lib/scene.js`, on both sides.** The server
  streams DOF values, not transforms. A pose computed in `web/` makes the picture lie while still
  looking right.
- **Z-up in mm everywhere** (`Object3D.DEFAULT_UP = (0,0,1)`). Rapier uses metres through
  exactly one constant, `SK = 0.001`. No axis remapping anywhere.
- **Actuators are kinematic models. Only free workpieces are Rapier-dynamic.** Physics never
  gates safety or sequence, and nothing in physics writes to the PLC.
- **Workpieces never sleep** (`setCanSleep(false)`). Measured on Rapier 0.20: a kinematic
  pusher sweeps straight through a part that fell asleep while resting. It does not wake the
  part, the part stays put, and there is no error. `tests/rapier.test.js` pins both halves:
  the trap and the rule.
- **A held part is kinematic and follows its holder at a stored relative pose.** No fixed joints
  between dynamic bodies: they fight the solver.
- **The viewer's hand is that same mechanism with the WORLD as the holder.** Clicking and holding
  a loose part pins it where it is (`holdPart`, `pt.pin`), so the belt slips under it and a queue
  builds behind it: a deliberate jam, which is the test a real machine has to survive. Dragging
  moves the part instead (`at` in mm, world), in the plane facing the camera. The hand DOES take a
  part out of a gripper, a cup or a nest, and the holder's own switch then goes false: that is the
  point, because the machine must notice the part it thinks it has. It releases at zero velocity,
  and its edges are queued like button presses so a run still replays identically. Forcing a tag
  lies to the PLC; the hand breaks the material flow, which is a different failure and finds
  different bugs.
- **A pile of parts on a belt is STABLE, not slow.** Measured with 32 parts fed against a wall on
  a running belt: 350 µs/step free-running, 510–590 µs/step through the real-time pacer with sim
  time exactly level with wall time, 0 overruns, 0 warnings. The pile's speed falls from 982 to
  5 mm/s in about 15 s and stays there: the belt wedges the parts, it does not scatter them. So
  "the pile takes a long time to break up" is the physics, not a performance problem, and neither
  the step cost nor the pacer is the thing to look at. `compile()` in the render loop was the
  obvious suspect and was measured at ~1 µs, i.e. innocent (it is cached anyway).
- **Sensors about the machine (reed switch, in-position, origin) are analytic, from DOF values.**
  Only sensors about parts use Rapier queries.
- **Part sensors query PARTS only** (collision groups `G_PART`/`PART_RAYS` in `server/plant.js`).
  A ray that could hit the machine trips on the belt guide it looks through.
- **Loose parts stream transforms; machine links stream DOF values.** Parts are Rapier-dynamic,
  so no DOF describes them. Everything with a DOF still goes through `worldPoses()`.
- **The belt drives parts by friction-clamped slip plus a friction torque** (`beltDv`,
  `beltSpin`), and the belt collider has friction 0 (combine `Min`). This was measured:
  - a velocity override stacks a queue on top of itself;
  - a frictionless belt with no torque lets a landing spin grow to 9° of yaw in 4 s.
- **Zone tests use a part's shape centre, not its origin.** The origin is the part's
  underside, which rests a hair inside the belt, so an origin test in a remover box misses
  every part.
- **Templates** (workpieces an emitter copies) are never simulated and never drawn.
- **An emitter checks the COLUMN below it, not the spawn point.** A part falls onto whatever is
  under the feeder. Measured on `buffer-queue`: with the check at spawn height only, the ball
  tested 845–905 mm while the part already on the belt sat at 800–830, so the emitter called it
  clear and dropped part onto part, three stacks of two. A feeder that MEANS to stack sets
  `dropOnto` (the assembler's lid feeder), and then only its own spot must be free — the strict
  check had stopped the lids from ever reaching their bases.
- **Nothing solid may stand ahead of a part's path at its own level.** A fixed collider holds a
  driven part through a contact at a POSITIVE distance, and no geometry tunes it away. Measured
  three times while building `buffer-queue`, each time seizing the queue with the belt still
  running and the parts creeping backwards:
  - a second belt slab butted to the first held a part from 3.76 mm away. Sweeping the drop gave
    stuck at 0, clear at 0.5–1, stuck at 1.5–4, clear from 5; changing the overlap flipped the
    answers again (a 6 mm drop clears at 0/20/40/60 mm of overlap but sticks at 10);
  - a chute plate set just past the belt end held it from 14.43 mm away.

  A part must leave a belt over its END into free space, and be caught by a remover ZONE, which
  has no collider and cannot jam. A chute only works where the part **falls onto** it from
  above, well clear of the edge, as in `stopper-pusher`. `tests/rapier.test.js` pins the trap.
- **The same trap catches a RETRACTED overhead stopper, so a pallet must not pass under one.**
  Measured on a 200 mm pallet driven at 250 mm/s past a stopper whose pin had lifted clear and
  come to rest (contact refresh done): it was held from 7.9 and 15.1 mm away, creeping backwards
  at 1.7 mm/s. Sweeping the mounting height gave clearance 5.1 mm HELD, 8.1 free, 11.1 free,
  **14.1 HELD**, 17.1 / 20.1 / 24.1 / 28.1 free — non-monotonic, exactly like the belt seam, so
  there is no clearance to design to. A stop a pallet has to drive under is the wrong structure;
  use one that leaves the path entirely.

  **So a pallet stop pops UP from under the belt.** Retracted, its head sits below the belt
  surface and nothing stands in the path at all. Measured at 5, 15 and 30 mm of sink, and with
  60 and 100 mm strokes: every one blocks the pallet at the same place and then releases it, with
  the pallet flat at z = 800 all the way down the belt. The sink depth does not matter, which is
  the point — it is structure, not tuning. The cylinder foot must sit `Lb + 52 + sink` below the
  belt (`Lb = stroke + bore + 20`, rod end 27, head 25), and the pin must be UP before the pallet
  arrives: a pin that rises under a pallet already over it tips the pallet off (measured).
- **Two belts must not overlap in X, and a nest on a belt must be SUNK.** Both are the
  positive-distance trap again, met twice while building `blurobot`: an outfeed belt whose slab
  reached back into the infeed's path held the incoming part from 16.6 mm away, dead still, 175 mm
  short of its target; and a nest whose floor sat at belt level stopped the part on its leading
  edge from 3.1 mm away, creeping backwards at 1.4 mm/s. Sink the nest instead — but then its
  catch ZONE goes down with it and the part sails off the belt end (measured: lost at z −1009), so
  deepen the pocket until the zone reaches belt level again. The walls are drawn without colliding,
  so a deeper pocket puts nothing new in the path.
- **A 2-finger grip is confirmed by the OPEN switch DROPPING, never by `closed`.** `closed` sits at
  full close, so with a part between the fingers it never comes on — that is the whole point, it is
  how a missed grip stays visible (rb4axis `SIM_GRIP_TUTUP`). Waiting for `closed` hangs for ever
  on a successful grip; waiting for `NOT closed` passes instantly and lifts away with the part
  still in the fingers. Wait for `open` to drop and HOLD for a confirm time, so a part slipping out
  restarts it. `tests/ctl.test.js` pins both halves.
- **A count snapshotted in a step the cycle never revisits is stale on the second cycle.** Measured
  on `blurobot`: the feeder count was taken at a start-up step, so on cycle 2 the emit step saw
  "the count already moved", dropped the command without feeding anything, and the next step waited
  for a part that never came. Re-snapshot it at the cycle-complete step, as `a-to-b` does.
- **A held Execute is LATCHED on its rising edge, so a target that changes later is never read.**
  The axis model takes its target when `exec` goes true and not again. Holding `exec` true from the
  first scan therefore freezes the axis on its startup target: measured on `blurobot`, every
  station cover latched 80° open while the program commanded 0°, so no test clock ever ran, no
  board ever finished, and the cell looked busy while producing nothing. Drop `exec` for one scan
  whenever the target moves. This is the same rule as "commands are levels held until answered" —
  the level is held, but its EDGE is what arms it.
- **A step that both empties a source and fills a destination must not share one index.** `idx` was
  reused for source then destination, so on a job whose source was the station just loaded, the
  pick step zeroed the state the place step had set. Measured: ICC 1 was loaded, marked processing,
  then immediately reported empty again. Keep `src` and `dst` separate, as rb4axis does.
- **A beam says a part is HERE; the STOP is what locates it.** Cutting the belt on the beam edge
  left the pallet 106 mm short of the pin on every cycle — centre at 94.1 mm instead of 190.6 —
  so the part feeder dropped its load onto the belt behind the deck, +118.9 mm out. The sequence
  must keep driving after the beam until the part is pressed against the stop (a settle step:
  106 mm at 250 mm/s is 424 ms, so 700 ms), and only then stop the belt. With that, the load lands
  0.2 mm from the deck centre, cycle after cycle. Denny saw this one in the 3D view first.
- **A seized machine still balances its parts and raises no warning.** Ask for PROGRESS: cycles
  that keep completing. The 30-minute soak reported OK on a `buffer-queue` that had stopped
  after 15 minutes, because every part was still accounted for. Both the soak script and the
  scene test now check that cycles keep coming.
- **A nest LOCATES the part it catches** (`snap` on the type): the part is seated square on the
  pocket floor, not frozen wherever it was when its centre entered the pocket. Measured: a base
  caught mid-fall hung 11.6 mm high, which then put it inside the press's stroke.
- **A holder's catch volume is not always `params.size`.** `candidate()` in `server/plant.js` read
  `r.p.size` for every non-vacuum holder, but a gripper has no `size` — its catch volume is
  `zone(p)`. A gripper only reaches that line as the holders loop's FALLBACK (`cand ?? candidate`),
  which needs a free part sitting in its zone at the moment it is asked, so the crash
  (`undefined is not iterable`, and the plant dies mid-run) sat in shared code from 34be524 on
  12 Sept until the `blurobot` cell happened to arrange exactly that. Every scene with a gripper
  was one coincidence away from it. Fall back to `zone(p).size`, and return null when a holder has
  neither: a holder that cannot say where it catches should catch nothing, not throw.
- **A machine cover is a `joint` (`arm: 'plate'`), and its leaf does not collide.** A cover is a
  revolute axis with an angle the sequence waits on, so it is the same type as any other axis, not
  a new one. The leaf sweeps the whole machine top, which is exactly where the nest and the board
  it holds are: a solid leaf would hand physics a question the interlock already answers, and
  closing it onto a board held by a kinematic nest is the eject case below. It is drawn, not solid.
- **Hinge a cover at the edge AWAY from the robot.** Hinged at the near edge it opens across the
  side the arm comes in from; hinged at the far edge it stands up behind the nest. rb4axis hinges
  at the back for this reason, and the leaf is offset half its thickness off the hinge line so it
  lies ON the hinge when shut instead of straddling it.
- **`frame` has two styles.** `table` is legs plus a top plate; `solid` is one coloured cabinet
  whose 16 mm plate overhangs 10 mm a side and is counted INSIDE `size[2]`, so `top` stays at
  `size[2]` and whatever is mounted on it keeps its height. Six tables in one cell read as a
  thicket of legs, when what has to be readable is which machine the arm is over.
- **A kinematic tool never closes ONTO a part resting on another kinematic body.** The solver
  has nowhere to put the part and ejects it. Stop at the part's surface, as the gripper does
  with `blockAt`, or leave a few mm (the assembler's press stops 3 mm above the lid).
- **A part only carried by friction limits how fast a table may index.** A cycloid over an arc
  `h` in time `T` peaks at `2πh/T²`, and the part slides when that passes `μg`. Measured on the
  assembler: a 418 mm station arc in 0.6 s gives 7.3 m/s² against μg = 4.9, and all 13 lids slid
  off the table and fell out of the world. At 1.2 s it is 1.8 m/s² tangential plus 1.8
  centripetal, and they ride.
- **A part must not have to drop into a tight pocket.** Measured: a 70×60×20 part dropped
  between 25 mm walls hangs on speculative contacts at the wall top edges (45° normals) with
  anything under 12 mm of clearance per side; at 5 mm it hung 20 mm above the floor. So a
  `nest` draws its walls without colliding, and any guide a part drops between leaves ≥ 12 mm
  per side (or is shorter than the part). `tests/rapier.test.js` pins it.
- **A kinematic link that comes to rest takes its colliders out for one step** (the contact
  refresh in `server/plant.js`). This was measured on Rapier 0.20: a part that was pressed
  against a stopper keeps a stale blocking contact after the stopper lifts clear. The part stays
  stuck even with the stopper 20 mm above it, whether the stopper is a cylinder or a box, and
  with CCD on or off. With the refresh, a box releases the part at any clearance, but a cylinder
  needs about 12 mm, so stoppers are square blocks. `tests/rapier.test.js` pins both halves.
- **One motion model**, the trapezoid ported from rb4axis `langkahSumbu`. A second copy will
  disagree one day.
- **Time scale is 1× whenever a PLC is connected.** Sysmac timers run on wall time.
- **Twin mode is read-only inside the driver's `write()`**, not in the UI. Never write to a real
  machine.

## OPC UA and Sysmac Studio (each cost a round once)

| | |
|---|---|
| start the simulator FIRST | *Simulation → Use the OPC UA Server* stays grey until Run (F5). Needs Studio ≥ 1.62 |
| security **None** + anonymous **Permit** | a certificate rejection looks like a wrong password |
| `networkPublish="PublishOnly"` | a project built from scratch exposes zero tags without it |
| `OPCUACertificateManager` needs an explicit `rootFolder` | otherwise it hangs forever on "Creating default certificate". The class is in `node-opcua-certificate-manager`, not re-exported by the client |
| browse MUST follow `browseNext` | without it nodes vanish silently and look like "tag missing" |
| `server/pki/` is gitignored | it contains a private key |
| typed arrays → `Array` in `plain()` | `Float64Array` stringifies to `{"0":..}` and the page silently gets NaN |
| the program must be **assigned to a task** | XML import cannot do it; `tools/smc2.js` can (proven in Studio). An unassigned program does not run and Studio does not complain. The heartbeat check exists for this |
| a tag with a coil in the PLC cannot be forced from OPC UA | the PLC overwrites it every scan. The plant's overwrite detection warns about it |
| no POU names starting with `P_` | Studio silently renames them to `PR_...` |
| never index an FB instance's array output | copy the whole array first |
| arrays in XML = `InstantlyDefinedType`/`ArrayTypeSpec` | `<TypeName>ARRAY..` passes the XSD and fails in Studio |
| importing GlobalVars adds; importing an existing POU name replaces | |
| `<ST>` in XML uses LF; `.smc2` entries use CRLF | XML normalises line endings; the ZIP does not |
| no `ATAN2` in generated ST | not in the W560 list of 353 instructions |
| `.smc2` edits need the project CLOSED in Studio | Studio rewrites the whole file on Save. `tools/smc2.js` backs up and verifies every entry before writing |
| the solution id (the `.smc2` root folder) changes on every Save | find it through the manifest, never hard-code it |
| a program missing from `<task>.xml` never runs, silently | the task file, the `.oem` node, the AssociatedProgramModel and the OPC UA node are written together (docs/SMC2.md) |

## IO timing

- **PLC → plant commands are levels or counters, never one-scan pulses.** Servo Execute is held
  until Done. 50 ms sampling misses a 4 ms pulse.
- **Plant → PLC blips are held** (`offDelayMs` per sensor, `minPulseMs` per scene), and every
  stretch is recorded as a `warn`. Do not silence it.
- **Replies to PLC commands are `hold: false`** in the io schema (servo `done`/`busy`/`inPos`).
  Their short pulses are caused by the PLC itself. Holding them reports "in position" while
  the axis already moves, and warns on every move.
- One-shot events (counts, drops, rejects) are published as **counters**.
- **A waiting step waits for an INVARIANT, never for a counter to move.** A change cannot say
  which part moved. Measured live on `a-to-b`, in two rounds:
  - waiting for "the unloader count moved" let an older part's removal end the cycle: 231 parts
    in, 217 out, and the crowd at the end sensor raised 13 pulse-stretch warnings;
  - snapshotting the count on entry balanced the parts and silenced the warnings, but the cycle
    then ran at 1.1 s instead of 6.5 s (388 cycles in 423 s) with five parts pipelined on the
    belt, still riding other parts' removals.

  The discharge step now waits for `RM1_CNT = EM1_CNT`: everything loaded has left. An invariant
  is also self-healing, since a pipeline drains back to one part. The internal controller never
  showed any of it, because there the counters only ever moved during the waiting step.
  `tests/ctl.test.js` pins it.
- **A waiting step needs a WATCHDOG, because a part can leave without reaching the unloader.**
  The viewer's hand takes one, or it falls off the line. Then `RM = EM` can never hold again.
  Measured right after the hand shipped: dragging a part off `a-to-b` left `ST1_STEP` at 40 with
  the belt running and `AUTO_RUN` on for as long as anyone watched (EM 2 / RM 1), and taking
  `pick-place`'s part out of the nest left step 20 the same way. **A machine that waits for ever
  looks alive and is not.** A step that has not moved for 15 s goes to FAULT (step 900): feeding
  and motion off, `AUTO_RUN` off, START acknowledges it. Nothing is dropped on the way: the
  vacuum keeps its part, as a real machine does.
- **A write-off goes STALE when the part turns up after all**, and a stale one is silent. The hand
  drops it back on the line, or it rolls into the unloader later; `RM` catches up, `RM + GONE` runs
  past `EM`, and the discharge step stops waiting altogether — the pipelining bug again. In ST it
  is worse: `GONE` is a `UDINT`, so `EM - RM` with `RM` ahead underflows to about 4 billion and the
  invariant is true for ever. So clamp it every scan: `GONE` never exceeds `EM - RM`, and never
  goes below zero. Found by reviewing my own diff, and `tests/ctl.test.js` pins it.
- **An invariant must stay reachable, so write off what left the machine — but only when the
  FAULT is acknowledged.** Acknowledging takes `GONE = EM - RM`, and the discharge step tests
  `RM + GONE >= EM`. Without the write-off a single part removed by hand stops the machine for
  good, even after a restart. Writing off at every START instead breaks the opposite way, and I
  did exactly that first: it writes off parts still legitimately on the belt, so an older part's
  removal ends this cycle again — the pipelining bug that cost two live rounds on the simulator.
  `tests/ctl.test.js` pins both halves.
- **Sampling hides fast steps, so count cycles, not step sightings.** The plant samples
  `ST1_STEP` at 10 ms: a step that should last seconds but is seen a handful of times in 388
  cycles is the bug report.
- **A step that waits for a sensor level must first see that level clear.** A part still in the
  beam belongs to the last cycle.
- **A counter the controller compares against must be reset with the plant.** `reset()` zeroes
  the plant's counters, so a controller still holding the old value sees "it changed" and waits
  for a part that never comes: the sequence stalls with the plant timer still running.
  `plant.reset()` calls `controller.reset?.()`, and every `.ctl.js` has one. A real PLC keeps
  its own copies: restart its program after a plant Reset.
- The browser sends button **edges**, and the PLC enforces the conditions. Conditions enforced in
  the browser do not apply when the same tag is written from anywhere else.

## Browser

- **Interpolate** between plant frames, rendering 50 ms behind. Velocity prediction (capped at
  120 ms) is only for mirrored DOFs, and it lives in the plant.
- Build panels once, then update text only, throttled to about 8 per second. Rebuilding
  `innerHTML` for each SSE message stutters the 3D view.
- Sliders send on `change`, not `input`.
- Redraw labels only when their text changes.
- Since three r169, `TransformControls` is not an Object3D: `scene.add(tc.getHelper())`.
- **A WebGL canvas cannot be hit-tested from the DOM, and sweeping it with clicks is no
  substitute.** A blind sweep looking for a part pressed the machine's own pushbuttons: it hit
  STOP, the line stopped feeding, and 1380 clicks over 62 s then found nothing left to grab. The
  browser test asks the page where a part is (`window.mioPartScreen`, read-only, used by nothing
  else) and clicks that point.

## Repo hygiene

- **Never point a junction at the real `node_modules` from a directory you intend to delete.**
  `git worktree remove --force` followed the junction and emptied the repo's own `node_modules`
  (measured 2026-09-14; the symptom is `Cannot find package '@dimforge/rapier3d-deterministic-compat'`,
  and `npm install` restores it from the lockfile). `tests/browser.test.js` already knew this and
  unlinks its junctions first, non-recursively; a throwaway worktree needs the same care, or its
  own `npm install`.
- `.gitattributes` = `* -text`. Generated files are compared byte for byte.
- Generators get `--check`, which exits 1 when committed output is stale.
- Dependencies are pinned exactly (no `^`). three is served from `node_modules`, never from a CDN.
- Tests: `node tests/run.js`, no framework. **A SKIP must print a message**; a silent skip looks
  like a pass.
- If a test fails, first prove whether the TEST is wrong.
