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
- **A machine drawn all in grey cannot be read.** Every component family takes its own material
  (`MAT` in `web/app.js`, `mat` on each shape in `lib/components.js`): blue `tube` is pneumatic
  power, bronze `motion` is what drives (servo rails, index tables), orange `tool` is what touches
  the part (pusher plates, stopper pins, press heads), green `holder` is what holds it (nests,
  gripper fingers, vacuum cups), teal `sensor` is what senses it, and the structure stays grey so
  the working parts stand out against it. A `mat` the viewer does not know falls back to grey
  SILENTLY, so `tests/web.test.js` checks that the palette covers every material the types use.
- **A beam needs columns under its ends, and a long stroke needs a supported rail.** Measured by
  looking at `palletizing`: the gantry rail's columns were nowhere near its ends, the cross beam
  hung 850 mm from the carriage that carried it, and the unload transfer was a 900 mm rod
  cantilevered off one post. None of it is wrong to the solver and all of it is wrong to the eye,
  which is the only thing that reviews a scene's mechanics. A transfer longer than about 400 mm
  is a slide on two columns, not a rod on a bracket.
- **Every servo can be jogged** (`jogP`/`jogN` on `servoLinear`): the axis creeps while the button
  is held, at the speed override. Jog is REFUSED while Execute is held, because two sources of
  motion for one axis is how a machine gets broken, and both buttons at once is a stop, as on a
  real pendant.
- **A magazine that runs out must be refilled, or the machine just stands there.** An emitter with
  `gridCols`/`gridRows`/`gridPitch` fills a whole tray from ONE component: part n goes to hole n.
  `palletizing` starts with an empty pallet and loads its own 100 plugs, and calls for a fresh
  pallet when the twentieth cycle empties it. Twenty cycles are too slow to drive through the
  physics, so the pallet change is pinned in `tests/ctl.test.js` against a faked plant.
- **The viewer's hand WALKS a part to the pointer, it does not teleport it** (`sim.handMmS`,
  1200 mm/s by default). A pointer jumps half a metre between frames, and a kinematic body dropped
  into a queue of resting parts scatters them across the hall. Measured on a nine-part queue with
  the belt stopped: dragging the front one out now moves the rest by 0.2 mm.
- **An E-STOP never makes a holder let go.** Measured live on `palletizing`: the mushroom was hit
  while the pallet was unclamped for a pick, a hundred plugs were left loose in their pockets, and
  the plant went from 355 to over 2800 µs a step - the machine came back stalling at 2x world
  speed. Loose parts are Rapier-dynamic and never sleep; held ones are kinematic and nearly free.
  The E-STOP and FAULT steps therefore SET the tray clamps, they do not clear them.
- **A head takes what is there.** A hole may be empty because a plug was lifted out by hand or the
  tray is finished, and a station that waits for all five stands there until the watchdog. It
  picks for a fixed time and carries on with what it got; nothing at all means the tray is empty,
  which is how `palletizing` knows to call for a fresh pallet.
- **Stations that can run at once must be separate state machines**, one step tag each, meeting on
  shared state. `palletizing` ran pick, index and unload as one sequence and the gantry stood
  still for both: as three stations sharing `FULL[i]` per jig, with each head blocking the index
  while it is over the table, the cycle went from about 9.5 s to 5.8 s.
- **The status block above the operator panel has a fixed height.** Its hints come and go, and
  every line it gained or lost moved the panel under the pointer. Buttons that move get pressed by
  mistake.
- **A part parked on furniture is written to Rapier ONCE** (`parked` in `server/plant.js`): a part
  held by a component that never moves cannot have gone anywhere, so it needs no kinematic target
  and no fresh world centre each step. Ninety-five plugs standing in pallet pockets were costing
  300 allocations and 200 boundary calls a step to be told where they already were.
- **A feeder that is switched off has nothing pending, and one that is blocked does not remember
  what it missed** (the emitter in `lib/components.js`). Measured live on `palletizing`: the tray
  loader banked a backlog while the tray was full, then refilled every hole the machine emptied -
  1700 plugs loaded, a pallet that never ran out, and a plant slow enough to stall at 2x. A tray
  loader also steps PAST a hole that is already full (`slot` in the emitter loop), or a
  half-full tray can never be topped up.
- **The mount offset is built once per scene object** (`mount` in `compile()`), not from three
  Euler angles on every step, and `Math.hypot` is not used in the hot path: it rescales to guard
  against overflow, which quaternions and unit axes never need.
- **A holder only ever looks at the parts that are FREE** (the `free` set in `server/plant.js`),
  and every part's world centre is read from Rapier ONCE a step into `pt.cw`. Profiled on
  `palletizing` at 4x world speed: each of 120 nests scanned all 100 parts every step, two Rapier
  boundary calls apiece, about 4000 of them a step. That alone was 1469 µs of the step; with both
  fixes it is 355 µs and the scene holds 4.00x with a viewer attached. The physics was never the
  problem: the profile put `world.step()` at about a tenth of the cost.
- **Most of a scene never moves, and `worldPoses()` caches it** (`still` in `lib/scene.js`). Frames,
  plates, pallet pockets and the parts standing in them are computed once per scene object.
  Measured on `palletizing` (265 components): 2276 -> 663 µs a step. A component is cached only
  when no link of it has a DOF AND nothing it is mounted on moves, so anything riding a carriage
  is still recomputed. The cached poses are SHARED between calls: never write to a pose you were
  given. `tests/lib.test.js` pins both halves.
- **The first pacer tick after `start()` is not an overrun** (`firstTick` in `server/plant.js`).
  Measured on every scene: one stall of 240-440 ms lands exactly on the first tick - a major GC
  right after setup - and counting it left a permanent "overruns 171" on a plant that then held
  99% of real time for the next 40 s.
- **A vacuum cup's bar must clear the parts it picks.** Measured on `palletizing`: the cup bar sat
  2 mm into the plug tops, and the moment the pallet unclamped, the solver had nowhere to put the
  plug and flung it out of the world. The cups hang 20 mm below the bar. This is the same rule as
  a kinematic tool closing onto a part, seen from above.
- **A remover ZONE must stand clear of anything that sweeps past it.** Measured on `palletizing`:
  the "next process" bin overlapped the rotary carrier's swept circle, and an outer jig pocket
  passed through the zone during an index, so the bin quietly ate plugs off the moving carrier.
- **A pin cannot come down between parts that TOUCH**, so a stop-and-go escapement only works
  where the parts arrive with a gap. `buffer-queue` therefore meters ONE part into the line at a
  time, and its HOLD pin always lands on free belt.
- **A nest only takes a part whose CENTRE is inside the pocket depth** (`candidate` in
  `server/plant.js`). Measured while building `press-station`: a 40 mm part over an 18 mm pocket
  was never taken, and nothing said why. The pocket must be at least half the part's height.
- **The IO image starts from what the components say, not from zeros** (`settle()` in
  `server/plant.js`, at creation and after Reset). Measured: with zeros, the first scan after
  Reset saw the selector off and the next scan saw it "turn to AUTO" on the same scan as START,
  and the controller faulted.
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
- **A part must leave a surface into FREE AIR, never across the top of a solid box flush with
  it.** Rapier 0.20 can keep a box-top contact manifold after the part has slid off the box.
  Measured on `sort-by-material`: the conveyor's side members were flush with the belt, so a
  pushed part slid belt → rail top → off the rail's far edge, and steel #43 of 43 came to rest
  12 mm BESIDE the rail at belt height with 4 contacts at −0.03 mm, normal +Z, nothing under it,
  velocity zero — it never fell, the discharge invariant never held, and the machine faulted
  for good after 84 cycles. The other 42 identical pushes fell at once, so this is a landing
  that is sensitive, not a landing that is wrong; the cure is structural: the side members now
  sit `T` below the belt surface, so the part leaves the belt edge with nothing beside it.
  `tests/lib.test.js` pins the geometry; the 30-minute soak is what found it.
- **A kinematic link that comes to rest takes its colliders out for one step** (the contact
  refresh in `server/plant.js`). This was measured on Rapier 0.20: a part that was pressed
  against a stopper keeps a stale blocking contact after the stopper lifts clear. The part stays
  stuck even with the stopper 20 mm above it, whether the stopper is a cylinder or a box, and
  with CCD on or off. With the refresh, a box releases the part at any clearance, but a cylinder
  needs about 12 mm, so stoppers are square blocks. `tests/rapier.test.js` pins both halves.
- **A six-axis arm is a chain of `joint`s copied from the maker's URDF, and its poses are solved
  ONCE.** `robot-pitch` is the FANUC LR Mate 200iD straight from the ROS-Industrial xacro (which
  takes its numbers from Fanuc's manual): each joint's `to` is the next joint's origin and the
  axes are signed (`-y`, `-x`), so nothing is re-derived and the flange lands at (465, 0, 695)
  from the base at all-zero, as the arm does. The controller commands joint ANGLES from a table,
  as the real program does; `lib/ik.js` (damped least squares on a finite-difference Jacobian of
  `worldPoses()`, so it cannot disagree with the picture) runs only in the build script and the
  tests, never in the plant. The table and the ST twin are GENERATED from one IK output - 78
  angles typed twice would drift - and `tests/lib.test.js` pins every pose against the scene:
  13 poses within 0.05 mm, ≥ 30° from every limit. The IK had its update sign wrong once and
  walked the arm 1191 mm the other way until the limits stopped it; the test that says
  "unreachable is reported, not pretended" is from that.
- **An IK that stops short with every joint far from its limit is a BOUNDARY, not a bug.**
  Measured on the hanging VS-060: five of six poses failed by 2–126 mm with 73–80° of margin to
  every limit, and the residual was bit-identical (126.10 mm) from both seeds, at 400 and at 2000
  iterations, with damping loosened and the step cap doubled — position-only, with no orientation
  demanded at all. A local minimum moves when you reseed it; a sphere does not. The tool-down
  reach from the J1 origin is **918 mm** (swept over the real chain, not summed from the link
  lengths), and the chuck was 1026 mm away. Before touching the solver, compute the distance from
  the first joint to the target and compare it with a SWEPT reach.
- **A sweep that holds J1 measures a slice, not an envelope.** On the VS-060 (and most 6-axis
  arms) J2, J3 and J5 all turn about the same axis, so with J1 at zero the tool moves only in
  that plane — for a rail-mounted robot, the rail's own plane. An envelope map that pins J1 says
  every reachable point is directly under the rail, which is true and useless. Reaching sideways
  is J1's job, and the envelope has to be measured with it free.
- **A part leaves a belt as a PROJECTILE, so a discharge zone starts at the belt end and is as
  long as the flight.** 0.38 s to fall 700 mm is 380 mm of flight off a 1 m/s take-away and 760 off
  a 2 m/s line. Measured on `carton-sorter` with the zones 600 mm long at the belt end: 61 of 68
  cartons flew clean over them and were reported lost, with the sorter itself working perfectly.
- **A diverter does not push: it STANDS IN THE PATH at an angle and the belt does the work.**
  A blade 2127 mm long at 45 degrees sweeps 1504 mm across, which is a 1.524 m belt's full width;
  the carton meets it, slides along it and leaves over the side onto a take-away. Measured on
  `carton-sorter`: 18 cartons deflected in 29 s with none lost and no warnings. This is the
  exception to "nothing solid may stand ahead of a part's path" - what seizes a part is a face
  SQUARE to its travel; a face at 45 degrees gives it somewhere to go.
- **Sorter tracking is a SHIFT REGISTER, not a timer.** Each blade owns a FIFO of the destinations
  of the cartons between it and the scanner upstream; a beam edge at the blade pops the queue, and
  a carton that is not for this blade is pushed onto the next blade's. A timer from the scanner
  would sort the wrong box the first time the line jams, a hand takes a carton off, or the speed
  override is turned down - all three of which this simulator lets a viewer do. `carton-sorter`
  splits 21/21/21 over 63 cartons because the queue moves only when a beam says one went past.
- **A GRIPPER may take a part a nest is still holding; a chuck that opens first drops it.**
  `inZone()` in `server/plant.js` lets a gripper's fingers close on a part held by a `nest`, and the
  nest gives it up on that step (its `present` drops at once, which is the point - the machine has
  to notice). That is the hand-over every tending robot lives on: grip, THEN open the chuck. With
  the old rule - holders take FREE parts only - the sequence had to unclamp first, and a horizontal
  chuck then drops the part before the fingers are anywhere near it. Nothing else changes: a holder
  still never takes a part another GRIPPER has.
- **A nest on a cylinder's rod end must sit clear of the rod-end block.** Every rod carries a 12 mm
  steel block at its end, and a nest mounted flat on the `rodEnd` socket puts its 6 mm floor
  straight through it. Measured on `lathe-line`: the part stood on the 19 mm block instead of the
  62 mm floor, slid off it, sank 11 mm and toppled - and then rode the whole line lying down, which
  no sensor in the scene reports. Mount the nest `pad` (6 mm) above the socket.
- **A pop-up stop goes DOWN before the pin that lifted the part comes down.** Measured on the same
  scene: with the stop still up, the part came off the pin onto the 30 mm stop head instead of the
  belt, stood on its rim and fell over. The release order is stop down, then pin down, then wait
  for the beam to clear before the stop comes back up.
- **A station that both GIVES the robot a part and TAKES one back must be full when the robot
  arrives.** The pin is the same pocket for both halves, so the only thing that guarantees it is
  free for the finished part is that the robot took the raw one off it. Measured on `lathe-line`
  twice: going to a machine because it had finished, with the pin still empty, put the robot at a
  pin the station could not present empty (the next part stands at the stop directly over it) and
  the visit hung until the watchdog; and letting the station raise a part during the visit put two
  parts in one pocket, 40 mm apart, one of them off the end of the pin. A cut part waits in the
  chuck instead, which is what the real cell does when the line runs dry.
- **A beam a part crosses more than once needs an OFF-DELAY on the sensor, not a hold in the
  plant.** The part crosses the station beam three times - in, up with the pin, down again - and a
  part rocking as it lands off the pin flickers the ray for tens of milliseconds (measured: an 8 ms
  and a 20 ms gap in three minutes), which the plant then reports as a stretched pulse. 150 ms of
  off-delay is what the setting on a real beam is for.
- **A pitch-change head is ONE dof with a `scale` per slot link.** Five cups on a camshaft:
  slot i sits at (i − c)·pitchMax and slides by (i − c)·(pitchMin − pitchMax)/camDeg per
  degree, and the cam link turns by the same dof, so the picture shows the shaft turning as
  the cups close from 100 to 60. A second dof per slot would be five axes for a mechanism that
  has one motor.
- **A model is DRAWN and never collided.** A `mesh` shape names an asset under `assets/` (a
  `shell` component for something that does not move, a `joint`'s `mesh` for a link that does).
  The primitives stay as the colliders and carry `draw: false` so the viewer does not put a grey
  box through the middle of the robot — a trimesh does not collide with a trimesh in Rapier, and
  a part sensor would have to ray-trace thousands of triangles to answer "is something there".
  `colliderDesc()` now THROWS on a kind it cannot build instead of falling through to a ball,
  where `s.r` would be undefined and Rapier would take the NaN without complaint and break
  contacts somewhere else entirely. A URDF mesh is in metres and its visual is expressed in its
  own link frame, so it needs `scale: 1000` and no offset — and the arm's first joint is the
  scene ROOT, because a pedestal box under it lifts every shell by its own height.
- **Jog is a FRACTION of the cycle rate and stops at the axis limits.** `jogPct` (10% by default,
  5% on the robot) scales `vmax` for jogging only: at full rate J1 crosses its whole range in
  0.75 s and slide1 crosses its stroke in under two seconds, which cannot be placed by hand and
  reads as a fault. The jog clamps to `min`/`max` exactly as a move does — a pendant cannot drive
  an axis past a soft limit. `tests/plant.test.js` pins the creep rate and both end stops.
- **One motion model**, the trapezoid ported from rb4axis `langkahSumbu`. A second copy will
  disagree one day.
- **A step cost measured next to a software renderer is not the scene's cost.** The screenshot
  harness runs headless Chrome on SwiftShader, rendering 1440×860 at 30 fps on the same box, and
  its shots of `robot-pitch` showed `plant stalled 534 ms … step 5702 us vs dt 2 ms` — the plant
  apparently three times over its budget. Measured headless and alone, the same scene at the same
  point in its cycle costs **198 µs average, p50 161, 1 step of 4000 over dt: 10% of a 2 ms
  budget**, against `palletizing` at 13% of its 4 ms. Nothing was wrong with the scene; the OS was
  taking the CPU and `performance.now()` around the step loop billed it to the step. Measure
  headless before making a scene lighter, or you tune away a problem that was never there.
- **A hiccup is not a stall: the pacer catches up, it does not drop.** Measured on `palletizing`
  while Denny jogged a servo with the PLC on the same laptop: "plant stalled" warnings, yet the
  plant itself was never the limit - a step costs 0.3–1.1 ms against a 4 ms dt in every mode
  (jog held: avg 0.95 ms, p95 1.5, max 2.4, 0 of 1000 steps over dt), and through the real
  server with SSE and jog POSTs it held sim/wall 1.000 at 1× and 4.004 at 4× with 0 overruns.
  What tripped it was the pacer's cliff: owe more than 50 steps (200 ms) after any event-loop
  gap - a GC, or Windows scheduling Sysmac Studio and Chrome ahead of Node - and the excess was
  dropped and reported as a stall. Now anything under `DEBT_MAX_MS` (500 ms × world speed) is
  repaid within `CATCHUP_MS` (10 ms) of wall time per tick with the rest carried, so nothing is
  dropped and nothing warns; only a debt past that is a stall, and the warning now says the step
  cost against dt so "the box hiccuped" and "the plant cannot keep up" read differently. The
  status carries `behindMs`. `tests/plant.test.js` pins a 300 ms hiccup (caught up, 0 overruns,
  no warning) beside the 1 s stall (50 steps run, 450 dropped).
- **Time scale is 1× whenever a PLC is connected.** Sysmac timers run on wall time. `setScale()`
  in `server/plant.js` enforces it and warns; the viewer's picker only asks. Slow motion is
  0.05..4×: below 1 it is slow motion for watching, above it the plant runs ahead and pays for it
  in CPU, and an overrun is the honest report when it cannot. **The pacer's step cap scales with
  it** (`MAX_STEPS * scale` in `tick()`): the cap is on SIM time, so at 4× one 15 ms tick
  legitimately owes 60 ms of plant and a tick that slips to 30 ms owes 120 ms. Left fixed at 50 it
  warned "plant stalled" several times a second at 2×-4× on a plant that was holding 4.00× exactly
  with a viewer attached. The viewer's render clock must be
  scaled with it (`simScale` in `web/app.js`), or the
  interpolation offset drifts and the picture jumps.
- **The panel's speed override is a different thing from the time scale.** It is the MACHINE's
  own percentage dial (`speedDial`, `ovrK()` in `lib/components.js`): it scales servo velocity,
  belt speed and the index cam, never acceleration and never the pneumatics. It survives a PLC
  connection because it is part of the machine, and it reaches the axes only through the PLC
  (`OVR_SET` in, `OVR` out), never directly.
- **Twin mode is read-only inside the driver's `write()`**, not in the UI. Never write to a real
  machine.

## The operator panel

Every machine carries the same cell panel, drawn as HTML beside the 3D view and hideable. It is
NOT 3D buttons: a WebGL canvas cannot be hit-tested from the DOM, and the panel is read far more
often than it is looked at.

- **The panel devices are scene components** (`group: 'operator'` in `lib/components.js`): they own
  their tags like any other component, and `web/app.js` skips them in 3D and draws them in the
  panel instead. The browser sends button EDGES and dial VALUES; the PLC program enforces every
  condition. A condition enforced in the browser does not apply when the same tag is written from
  anywhere else.
- **The start-up order is the machine's own: energise, home, start.** MASTER ON closes the master
  circuit, HOME POS drives every actuator to its home position, and only then will START run the
  cycle. A machine that has not been homed refuses to start, which is the point of the button.
- **E-STOP is a latching mushroom** (`kind: 'alternate'`), so its `pb` tag IS its state: pressed in
  stays pressed in until it is twisted out. The panel shows that state on the button itself, not
  through a lamp - an operator who cannot tell whether the mushroom is in has no way to work out
  why the machine will not start. An E-STOP also LOSES the home position: the axes must be homed
  again before AUTO will run.
- **CYCLE STOP finishes the cycle**, it does not stop the machine where it stands. That is what
  the red button on a cell does, and it is why the machine comes back to a known state.
- **INDIVIDUAL is the other half of the selector.** Each actuator has its own button; the button
  is MOMENTARY and the PLC keeps the toggle memory, cleared when INDIVIDUAL ends, so nothing stays
  latched into AUTO. Servos are JOGGED rather than sent to positions (see the jog rule). Turning
  the selector while the sequence runs is a FAULT, not a pause.
- **The speed dial is an operator INPUT** (`speedDial`, dir `in`): the plant tells the PLC what the
  dial says and the PLC passes it to the axes (`OVR_SET` in, `OVR` out). It never reaches an axis
  directly, because on a real machine it does not either.
- **The plant times the cycle** from the tag the scene names in `cycle.countTag`, and averages the
  last `cycle.avgN`. The viewer shows the last cycle, the average, and two clocks side by side -
  sim and wall - because that is how you see at a glance whether the world is running fast or slow.

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
=======
- **A scene written by a generator loses every hand edit the generator does not know about.**
  `palletizing` was regenerated after its speed dial, its `ovr` bindings and its jog buttons had
  been added by hand, and all three vanished without a single error: the machine simply ran with
  three fewer controls than the others. Anything a generated scene must have belongs IN the
  generator. `tests/lib.test.js` now fails when a scene has a motor with no override, a motor and
  no dial, or a servo with no jog buttons.
- **`.st` and `.ctl.js` say "keep the two in step" and nothing checked it.** `tests/ctl.test.js`
  compares the step numbers each CASE handles, per scene. It is the cheapest thing that must agree,
  and it caught the watchdog being added to one file and not the other.
- **A parameter that can make the plant loop for ever is a validate() error, not a comment.** A
  workpiece of size zero has no collider radius, and the emitter's column check steps by that
  radius: the server hangs on a scene the editor was happy to save.
- **POST is refused unless the Host is this PC** (`postAllowed` in `server/http.js`). A page on
  another site whose name now resolves to 127.0.0.1 arrives from the local browser with Origin and
  Host equal to each other, so the Origin check alone lets it drive the machine.

>>>>>>> origin/operator-panel-and-more-scenes
- `.gitattributes` = `* -text`. Generated files are compared byte for byte.
- Generators get `--check`, which exits 1 when committed output is stale.
- Dependencies are pinned exactly (no `^`). three is served from `node_modules`, never from a CDN.
- Tests: `node tests/run.js`, no framework. **A SKIP must print a message**; a silent skip looks
  like a pass.
- If a test fails, first prove whether the TEST is wrong.
