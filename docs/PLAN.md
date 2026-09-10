# manufacturing_io — plan

## 0. Decisions

| | |
|---|---|
| plant host | **headless Node** owns the Rapier world, component models, IO image and recorder. The browser renders, edits and analyses. A hidden tab never pauses the plant a PLC is controlling, and one plant can have several viewers. |
| code | plain JavaScript, ES modules, **no bundler**, JSDoc types. One root `package.json`. |
| browser libs | three **0.186.0**, served from `node_modules` by the server (works offline on factory PCs), imported through an importmap. The browser does not load Rapier. |
| physics | `@dimforge/rapier3d-deterministic-compat` **0.20.0** in Node, so a run can be replayed bit for bit. |
| OPC UA | `node-opcua-client` **2.182.2** + `node-opcua-certificate-manager` **2.182.1** |
| transport | **SSE + POST** (same as rb4axis). Add `ws` 8.21.3 only if a measured stream is too heavy for SSE. |
| units / axes | scene in **mm**, **Z-up everywhere**: plant, Rapier (gravity −Z) and three (`camera.up = Z`). No axis remapping anywhere, so there is no mapping bug. Rapier works in metres through one constant `SK = 0.001`. |
| port | **7660** (rb4axis and ceinsert use 7656, sysmac uses 7655) |
| language | English |
| tests | `node tests/run.js`, one `chk()` per suite, no framework, loud SKIP (rb4axis style) |

## 1. Architecture

```
┌─────────────────────────────┐      OPC UA (opc.tcp://127.0.0.1:4840)
│ Sysmac Studio NX simulator  │◄──── subscribe PLC outputs (actuators)
│ PLC program = the controller│────► write PLC inputs (sensors), batched, on change
└─────────────────────────────┘
               ▲
┌──────────────┴──────────────────────────────────────────────────┐
│ server/ (Node, headless)                                        │
│  opcua.js   one session, browse+browseNext, subscribe, write    │
│  plant.js   fixed-step loop: IO image → component.step() →      │
│             Rapier step → sensors → IO image                    │
│  record.js  every IO edge + component state → ring buffer/NDJSON│
│  sysmac.js  scene → Sysmac global-variable XML + IO list TSV    │
│  main.js    HTTP: static, SSE /api/stream, POST /api/*, CLI     │
└──────────────┬──────────────────────────────────────────────────┘
               │ SSE state (~30 Hz) + POST commands
┌──────────────┴──────────────────────────────────────────────────┐
│ web/ (browser)                                                  │
│  app.js       three scene from scene JSON + joint values        │
│  editor.js    palette, gizmo, attach, properties, undo, save    │
│  analyzer.js  time chart, Gantt, cycle time (plain canvas)      │
└─────────────────────────────────────────────────────────────────┘
        shared/  imported by BOTH sides (no three, no Rapier inside)
          scene.js, motion.js, stats.js, components/*.js
```

**Why `shared/` matters.** A component file describes its geometry as a primitive list (box,
cylinder, sphere with size and offset). The browser turns that list into meshes. Node turns the
same list into colliders.

The browser resolves kinematic transforms itself from the scene JSON plus the joint values the
plant publishes, using the same `shared/scene.js`. So the stream only carries **joint values and
free-part poses**, not every mesh. The picture cannot disagree with the plant: it is the same
function with the same inputs. This is the rb4axis rule "draw from `chainPoints()`", made general.

## 2. Scene model

A scene is a **flat list** of component instances. Each instance names its parent. A flat list is
easier to diff, undo and edit than a nested tree.

```json
{
  "name": "servo-with-cylinder",
  "components": [
    { "id": "frame1", "type": "frame", "params": { "w": 800, "d": 400, "h": 900 },
      "pos": [0, 0, 0], "rot": [0, 0, 0] },

    { "id": "ax1", "type": "servoLinear", "parent": "frame1", "mount": "top",
      "params": { "stroke": 600, "vmax": 500, "acc": 2000, "mode": "drive" },
      "pos": [0, 0, 0], "rot": [0, 0, 0],
      "tags": { "target": "AX1_TARGET", "exec": "AX1_EXEC", "pos": "AX1_POS", "inPos": "AX1_INPOS" } },

    { "id": "cyl1", "type": "cylinder", "parent": "ax1", "mount": "carriage",
      "params": { "bore": 20, "stroke": 100, "acting": "double", "extendTime": 0.40, "retractTime": 0.35 },
      "pos": [0, 0, 40], "rot": [0, 90, 0],
      "tags": { "extend": "CYL1_EXT_SOL", "retract": "CYL1_RET_SOL" } },

    { "id": "rs1", "type": "reedSwitch", "parent": "cyl1", "mount": "tube",
      "params": { "at": 0, "band": 3 }, "tags": { "on": "CYL1_RET_RS" } },
    { "id": "rs2", "type": "reedSwitch", "parent": "cyl1", "mount": "tube",
      "params": { "at": 100, "band": 3 }, "tags": { "on": "CYL1_EXT_RS" } },

    { "id": "grip1", "type": "gripper", "parent": "cyl1", "mount": "rodEnd",
      "params": { "stroke": 10, "closeWidth": 24 }, "tags": { "close": "GRIP1_SOL", "closed": "GRIP1_CL" } }
  ]
}
```

**Attaching** (the core builder feature). A child's world transform is the parent's **mount** world
transform times the child's local `pos`/`rot`. Mounts can sit on a moving link. Here:

- `ax1.carriage` moves with the servo;
- `cyl1.rodEnd` moves with the rod.

So in this example the gripper rides the rod, the rod rides the cylinder, and the cylinder rides
the servo carriage. `scene.js` resolves the mounts in parent-first order each step. Cycles and
unknown mounts are rejected at load time.

**Tag direction** follows the Factory I/O convention, seen **from the PLC**:

- `out` = PLC output, which drives an actuator;
- `in` = PLC input, which is a sensor.

Tag names are generated as `<ID>_<KEY>` and can be edited. The scene file is the **only** place
names and numbers live.

## 3. Component contract

One file per type in `shared/components/`:

```js
export default {
  type: 'cylinder',
  params: {                       // drives the editor property panel AND validation
    bore:   { enum: [6,10,16,20,25,32,40,50,63,80,100], default: 20, unit: 'mm' },
    stroke: { min: 5, max: 1000, default: 100, unit: 'mm' },
    acting: { enum: ['double', 'single'], default: 'double' },
    extendTime:  { min: 0.05, max: 10, default: 0.4, unit: 's' },
    retractTime: { min: 0.05, max: 10, default: 0.4, unit: 's' },
  },
  io: p => [ { key: 'extend', dir: 'out', type: 'BOOL' },
             ...(p.acting === 'double' ? [{ key: 'retract', dir: 'out', type: 'BOOL' }] : []) ],
  links: p => [                   // rigid parts; a link with a joint moves
    { name: 'tube', shapes: [{ cyl: [p.bore * 1.4, p.stroke + 40], at: [..] }] },
    { name: 'rod',  parent: 'tube', joint: { prismatic: [1, 0, 0], min: 0, max: p.stroke },
      shapes: [{ cyl: [p.bore * 0.4, p.stroke + 20], at: [..] }] },
  ],
  mounts: p => ({ base: {...}, tube: {...}, rodEnd: { link: 'rod', at: [..] } }),
  states: ['retracted', 'extending', 'extended', 'retracting'],   // feeds the Gantt
  init: p => ({ x: 0 }),
  step(s, p, io, dt) { /* valve logic → s.x toward 0/stroke at stroke/extendTime */ },
}
```

- **Sensors that read their parent** (reed switch, servo in-position, index-table home) read the
  parent's joint value directly. They need no physics, which matches reality: a reed switch
  senses the piston magnet, not an object.
- **Sensors that see parts** (photoelectric, fiber, proximity, light curtain) use Rapier ray casts
  or intersection tests in Node.
- **Holders** are one mechanism shared by gripper, vacuum cup, nest/fixture, index-table pocket
  and pallet. A part is either **free** (dynamic Rapier body) or **held**. A held part is
  kinematic and follows the holder with the offset captured when it was taken. Releasing it
  makes it dynamic again, starting at the holder's velocity. Fixed joints between dynamic
  bodies jitter; switching the part to kinematic does not.
- **Kinematic links** are Rapier `kinematicPositionBased` bodies moved with
  `setNextKinematicTranslation/Rotation`, so pushers and stoppers really push parts.
- **Conveyor drive**: a part touching the belt's contact sensor gets its velocity along the belt
  pulled toward belt speed. This is velocity matching, not friction. It is a `ponytail:` choice:
  switch to true contact modification if slip ever matters. **Spike this first in Phase 2.**

## 4. SPM component kit (target ≈ 20)

| group | components |
|---|---|
| pneumatics | cylinder (double/single, bore/stroke/rod/speed), guided cylinder, rotary actuator (90/180°), 2-finger gripper, vacuum cup + vacuum switch, stopper, pusher |
| sensors | reed/auto switch (on a cylinder, adjustable `at` + hysteresis `band`), photoelectric (diffuse / through-beam / retro), fiber, inductive/capacitive proximity, light curtain, safety door switch, QR/ID reader (returns the workpiece ID) |
| motion | servo linear axis (stroke, carriage), servo rotary, **index table** (N stations, cam-profile index, in-position + locked signals), robot arm (rb4axis 1P+3R chain from `kin.js`) |
| process | press unit (position + pass/fail load window), drill/screw unit (spindle + feed), process station (timer + hinged cover, the rb4axis ICC/DW), marking/inspection (OK/NG result tag) |
| material flow | belt conveyor, roller conveyor, pallet conveyor with stopper/lifter, chute/gravity feeder, bowl-feeder output, emitter, remover |
| operator | pushbutton (NO/NC, momentary/alternate, with lamp), selector 2/3-position, e-stop, lamp, tower lamp, buzzer, numeric display |
| structure | frame, plate, profile, fixture/nest, guard (visual + collider) |
| items | workpiece (box/cylinder/plate, size, colour, material for inductive sensors, ID) |

**Servo has two modes:**

1. **`drive`**: the PLC writes target, velocity and execute. The plant runs the trapezoid profile
   (`langkahSumbu` from rb4axis, the single JS copy, moved to `shared/motion.js`) and publishes
   position, in-position and busy.
2. **`follow`**: the PLC runs real `MC_*` function blocks on a simulator axis and copies
   `Act.Pos` into a published LREAL global. The plant just follows that value, smoothed by
   velocity prediction (`haluskan`). This is the important mode for Sysmac users: the motion
   program being tested is the real one.

## 5. Plant loop and IO timing

- The plant uses a fixed step, starting at **2 ms**, paced to the wall clock. Catch-up is capped
  (rb4axis rule) so a stall never turns into a burst.
- **With a PLC connected, time scale is locked to 1×.** Sysmac timers run on wall time.
- **Time scale 0.1–10× only works in internal mode** (manual forcing or a scripted test
  controller). Factory I/O allows scaling and warns that sensors get missed; this plan forbids it.
- **IO exchange:**
  - PLC outputs arrive through one subscription (sampling rate from the Phase 0 measurement,
    target 10 ms).
  - Changed sensor values are written in **one batched `write`** per exchange tick.
- **Heartbeat.** The PLC template increments `MIO_HEARTBEAT`. If it stops, the UI says "program
  not running or not assigned to a task", which is the top cause of "tags read but nothing
  moves".
- **Pulse guard.** The recorder flags any sensor pulse shorter than 2× the measured round trip as
  one the PLC may not see.
- **Counters.** Events shorter than sampling (part counts, drops, rejects) are published as
  **counters**, never pulses, in both directions.
- **Forcing and failure injection** (Factory I/O parity) are applied to the plant's IO image, so
  the PLC sees them too. Forcing a sensor sends the forced value to the PLC.

## 6. Recording and analysis (what Factory I/O does not have)

**Recording.** `record.js` logs `{t, kind, id, v}` for every IO edge and every component state
change, using plant time in ms. Events go to an in-memory ring buffer and, when recording, to
`runs/<date>_<scene>.ndjson` (gitignored).

**Analysis** is computed from events by pure functions in `shared/stats.js` and drawn on plain
canvas (no chart library):

| view | content |
|---|---|
| **time chart** | a logic analyzer: BOOL tags as digital traces, numeric tags as analog traces. Two cursors show Δt between edges (e.g. `CYL1_EXT_SOL↑ → CYL1_EXT_RS↑ = 0.42 s`), with zoom and pan. |
| **Gantt** | one row per actuator or station, coloured by `states` (moving, waiting, working, blocked, starved) |
| **cycle time** | the cycle marker is a chosen tag edge (the part leaving) |
| **line balance** | stacked bar per station, working / waiting; the bottleneck is the largest working share |
| **OEE-lite** | availability (run vs stopped), performance (ideal vs actual cycle), quality (NG counter) |
| **compare** | two runs overlaid (sim vs sim after a change, or sim vs real twin) |
| **export** | CSV |

The cycle-time rules are carried over from rb4axis:

- measured **out-to-out**;
- the first part is excluded;
- the clock runs only while the machine runs;
- the average is divided by the sample count.

The rb4axis rule also stays: **anything shorter than bridge sampling is recorded on the PLC side**
(array + index, or counters).

## 7. Digital twin mode

The scene is the same; only the binding direction changes.

- **Every tag is read-only**, and `opcua.js` enforces that with a session flag, not with the UI.
  **Never write to a real machine.**
- Actuators are animated from real outputs and snapped to real sensors. Cylinder travel between
  valve and sensor is interpolated from the measured time.
- Servos in `follow` mode show real positions.
- The value is **drift detection**: per-actuator extend and retract times across shifts. A
  cylinder going from 0.42 s to 0.61 s over a week points to an air leak or cushion problem,
  seen before the machine stops. Real runs use the same analyzer and the same NDJSON as sim runs.

## 8. Sysmac integration

1. **Export tags.** Scene → `MIO_Globals.xml`: Sysmac global variables with
   `networkPublish="PublishOnly"`, arrays written as `ArrayTypeSpec`, LF line endings, no `P_`
   names. Also exported as a paste-able TSV. The functions are ported from
   `rb4axis/tools/gen_xml.js` (`tipeXml`, `varXml`, `globalXml`).
2. **Test program templates.** A loopback/latency program (Phase 0) and a heartbeat rung.
3. **IO list export** in the sysmac generator format (`ADDR\tTYPE\tIN|OUT\tCOMMENT`, types PB CR LS
   SS PH PL BZ AS SOL). Then: scene → IO list → sysmac generator → full program skeleton
   (Device_Input / AutoRunning / Fault …) → import → Sysmac simulator runs it against the same
   scene. **This closes the loop from 3D machine design to a tested PLC program.**
4. **Validation.** XML is checked against the official XSD with sysmac's `scripts/validate_xml.ps1`.
   The check is skipped **loudly** when that script is not present.
5. **Documented manual steps** (they cannot be automated): import → Build → assign the program to
   the primary task → Run (F5) → *Simulation → Use the OPC UA Server* → security **None** +
   anonymous **Permit** → start the plant.

## 9. Editor

- **Palette.** Drag a component onto the floor or onto a **mount socket**. Sockets highlight under
  the cursor. Dropping on a socket sets `parent` and `mount`.
- **Gizmo.** three `TransformControls` with a 5 mm / 15° snap; hold Shift for free movement.
- **Also:** `OrbitControls` and saved camera views.
- **Property panel** is generated from `params`. Editing a value rebuilds that component, and
  its children re-resolve through their mounts.
- **Hierarchy tree**, rename, tag editor (with a uniqueness check), copy/paste of a subtree.
- **Undo/redo**: snapshots of the scene JSON. This is the sysmac editor pattern: no mutation path
  can be missed.
- **Edit vs Run.** The scene can only be edited while stopped. Run rebuilds the plant from the
  saved JSON.
- **Save/load**: `POST /api/scene` writes `scenes/*.json`.
- **UI performance rules from rb4axis:**
  - build panels once and update only text, throttled to about 8 per second;
  - sliders send on `change`, not on `input`;
  - redraw labels only when their text changes.

## 10. Scene library

| priority | scene | shows |
|---|---|---|
| 1 | **Cylinder & reed** (hello world) | button → PLC → valve → cylinder → reed → PLC → lamp |
| 2 | **2-axis pneumatic pick & place** | X/Z cylinders + gripper, feeder chute → nest |
| 3 | **Index table 4-station** | load, press, inspect, unload; per-station Gantt |
| 4 | **Press-fit station** | servo press with a pass/fail window, NG reject |
| 5 | **CE Insert Track** (ceinsert) | supply feeder → buffer → CE eject, servo, QR/ID reader |
| 6 | **Blurobot cell** (rb4axis) | rail + 3R arm in follow mode, ICC/DW process stations with covers |
| 7 | **Drill / seaming / gel press** | spindle + feed units, rotary seaming |
| later | Factory I/O classics | From A to B, Sorting by Height, Buffer Station, Pick & Place XYZ, Separating Station, Production Line |
| skip | Filling Tank, Level Control, Warehouse, Palletizer, Elevator | fluids and heavy logistics are not SPM |

**Skipped on purpose (YAGNI):** a Factory I/O `.factoryio` importer (the format is plain XML, so it
can come later), a first-person camera, VR, and CAD/STEP import.

## 11. Repo layout (grows only when a phase needs it)

```
manufacturing_io/
  package.json            deps pinned exactly
  server/  main.js opcua.js plant.js record.js sysmac.js
  shared/  scene.js motion.js stats.js components/*.js
  web/     index.html app.js editor.js analyzer.js
  scenes/  *.json
  plc/     loopback + heartbeat templates (generated XML)
  tests/   run.js *.test.js
  docs/    PLAN.md SETUP.md (Studio steps + symptom → cause table)
```

## 12. Phases

Each phase ends with something runnable and a written exit criterion.

### Phase 0 — bootstrap, OPC UA and latency
- Port `rb4axis/bridge/bridge.js` into `server/opcua.js` + `server/main.js`, keeping:
  - certificate manager with an explicit `rootFolder`;
  - browse with `browseNext`, and path+suffix lookup;
  - read-once type metadata;
  - write whitelist;
  - `polos()` for typed arrays;
  - reconnect loop;
  - CLI `--tree/--list/--write/--watch`.

  This becomes the **one** copy. The ceinsert and sysmac browse code lack `browseNext`.
- `plc/Loopback.xml`: `MIO_LOOP_OUT := MIO_LOOP_IN` every scan, plus `MIO_HEARTBEAT`.
- `node server/main.js --latency`: toggle `IN`, time the echo on `OUT`, over 1000 samples at
  sampling 50 / 20 / 10 ms. Report p50/p95/max.
- Test whether OPC UA can write variables that have an **AT** address in the simulator. The
  answer decides whether the sim project can reuse the real machine's AT-mapped IO names.
- **Exit:** measured numbers written in `docs/SETUP.md`; the CLI reads and writes the simulator.

### Phase 1 — plant core, first component, round trip
- `shared/scene.js` (mount chain, validation, tag list), `shared/motion.js`.
- Components: `frame`, `cylinder`, `reedSwitch`, `pushbutton`, `lamp`.
- `server/plant.js`: fixed-step loop, IO image, force/release. **No Rapier yet**, because nothing
  is free-moving.
- `web/app.js`: render the scene from JSON plus the streamed joint values, with velocity-predicted
  smoothing capped at 120 ms. Force panel.
- `server/sysmac.js`: scene → globals XML.
- Scene 1, plus a 3-rung PLC program.
- **Exit:** a button pressed in the browser reaches the PLC, the cylinder extends in 3D, the reed
  switch reaches the PLC and the lamp lights. The event log shows the timings.

### Phase 2 — physics and material flow
- Rapier (deterministic build) in Node:
  - parts: workpiece, emitter, remover;
  - flow: belt conveyor (spike the velocity-matching drive first), stopper, pusher;
  - sensing: photoelectric (ray cast);
  - handling: holders (gripper, vacuum, nest).
- Scene 2, plus Factory I/O's From A to B as a sanity check.
- **Exit:**
  - a 1000-part soak run with no leaked bodies;
  - the same inputs replayed twice give the same event-log hash;
  - 200 free parts at 2 ms steps use less than 50% of one core.

### Phase 3 — editor
- Palette, sockets, gizmo, property panel, hierarchy, tag editor, undo/redo, save/load, edit/run.
- **Exit:** scene 2 rebuilt from an empty scene in the browser in under 10 minutes, without
  touching JSON.

### Phase 4 — analyzer
- Recorder NDJSON, time chart with cursors, Gantt, cycle time, line balance, OEE-lite, run
  comparison, CSV, pulse guard.
- **Exit:** scene 3 shows a per-station Gantt, and its cycle time matches a PLC-side counter to
  within one sampling interval.

### Phase 5 — motion and full SPM kit
- Servo linear and rotary (drive and follow), index table, rotary actuator, press unit, drill
  unit, light curtain, safety door, tower lamp, selector, e-stop, robot arm (`kin.js`
  `chainPoints` reused).
- Scenes 3–7.
- **Exit:** the ceinsert program running on the Sysmac simulator drives scene 5, and the
  rb4axis PLC program drives scene 6.

### Phase 6 — digital twin
- Read-only mirror mode, drift report per actuator, real runs in the analyzer.
- **Exit:** side-by-side real vs sim cycle for one machine.

### Phase 7 — Sysmac codegen loop
- IO list export into the sysmac generator, and a one-click "Sysmac pack" (globals XML + IO list +
  loopback template + setup checklist).
- **Exit:** empty scene → IO list → generated program → import → the program runs the scene.

### Phase 8 — FUXA and more drivers
- **FUXA → Sysmac simulator OPC UA directly.** The HMI talks to the PLC, as in reality, so there
  is nothing to build; document it.
- For plant-only data (cycle stats, twin drift): a Factory-I/O-compatible **Web API**
  (`GET /api/tags`, `PUT /api/tag/values`) that FUXA's WebAPI device can poll. Bind it to
  127.0.0.1, because FUXA's own API had an unauthenticated tag disclosure (CVE-2026-43946).
- A Modbus TCP server driver for non-Omron PLCs, and the Factory I/O classic scenes.

## 13. Tests (`node tests/run.js`)

| suite | checks |
|---|---|
| `opcua.test.js` | `polos`, path/suffix lookup, value parsing; live part SKIPs loudly without a simulator |
| `scene.test.js` | mount chain: moving the servo moves the reed switch's world position by the same amount; rejects cycles, unknown mounts and duplicate tags |
| `components.test.js` | every component's defaults validate; cylinder travel time matches `extendTime`; reed switches at `at ± band`; servo matches `motion.js` |
| `physics.test.js` | part on a conveyor reaches the sensor at the expected time; replay hash is stable; holder take/release |
| `sysmac.test.js` | `ArrayTypeSpec`, `networkPublish`, no `P_` names, LF in `<ST>`; XSD via `validate_xml.ps1` or loud SKIP |
| `stats.test.js` | cycle-time rules (first part excluded, stopped time excluded, avg / N), Gantt intervals |
| `web.test.js` | source guards: no second motion model or scene resolver in `web/`; panels not rebuilt from SSE messages |

## 14. Risks

| risk | handling |
|---|---|
| OPC UA sampling floor of the Sysmac simulator (unknown) | measure in Phase 0; pulse guard; counters; PLC-side traces |
| writing AT-bound variables through OPC UA in the simulator | test in Phase 0, before scene naming conventions are fixed |
| Sysmac simulator needs **Studio ≥ 1.62** for its OPC UA server | check the version first (SETUP.md) |
| Sysmac simulator cannot be time-scaled | 1× whenever a PLC is connected |
| Rapier determinism across machines | the deterministic build; the replay hash test |
| conveyor friction model | velocity matching, spiked early in Phase 2 |
| scope creep toward all of Factory I/O | the SPM kit first; the Factory I/O catalogue in Phase 8 |
| program not assigned to a task (silent) | heartbeat check shown in the UI |
| writing to a real machine by accident | the twin session is read-only in code, not in the UI |
| licence | **undecided**: pick one before the first external contributor (MIT matches FUXA and Open Industry Project) |
