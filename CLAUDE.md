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
- **A belt-to-belt transfer overlaps and steps DOWN.** Flush and merely touching, a part parks on
  the seam: it settles a hair into the belt it is on, and its leading face meets the vertical
  edge of the next slab. Measured on `buffer-queue`: the queue jammed with the front part dead
  at the exit beam. The outfeed now overlaps the last 20 mm and sits 1 mm lower.
- **A nest LOCATES the part it catches** (`snap` on the type): the part is seated square on the
  pocket floor, not frozen wherever it was when its centre entered the pocket. Measured: a base
  caught mid-fall hung 11.6 mm high, which then put it inside the press's stroke.
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

## Repo hygiene

- `.gitattributes` = `* -text`. Generated files are compared byte for byte.
- Generators get `--check`, which exits 1 when committed output is stale.
- Dependencies are pinned exactly (no `^`). three is served from `node_modules`, never from a CDN.
- Tests: `node tests/run.js`, no framework. **A SKIP must print a message**; a silent skip looks
  like a pass.
- If a test fails, first prove whether the TEST is wrong.
