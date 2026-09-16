# manufacturing_io — plan

## 0. Decisions

| | |
|---|---|
| plant host | **headless Node** owns the Rapier world, component models, IO image and recorder. The browser renders, edits and analyses. A hidden tab never pauses the plant a PLC is controlling, and one plant can have several viewers. |
| code | plain JavaScript, `"type": "module"`, **no bundler**, `// @ts-check` + JSDoc in `lib/`. One root `package.json`, exact pins (`npm i -E`). |
| Rapier | **Node only.** `@dimforge/rapier3d-deterministic-compat` **0.20.0**, so a run can be replayed bit for bit. The browser never loads WASM. |
| three | **0.186.0**, served from `node_modules` by the server through the importmap: offline on factory PCs, version pinned by the lockfile, no CDN. |
| OPC UA | `node-opcua-client` **2.181.1** + `node-opcua-certificate-manager` **2.181.0**, the combination proven against the Sysmac simulator in rb4axis. node-opcua is CommonJS; load it with `createRequire`. |
| transport | **SSE + POST, JSON.** No `ws`. |
| units / axes | scene in **mm**, **Z-up everywhere**. three uses `Object3D.DEFAULT_UP = (0,0,1)`; Rapier uses gravity −Z in metres through one constant `SK = 0.001`. rb4axis's `ke3()` swap (a mirror, not only a rotation) disappears. |
| port | **7660** (rb4axis and ceinsert use 7656, sysmac uses 7655) |
| language | English |
| tests | `node tests/run.js`, one `chk()` per suite, no framework, loud SKIP (rb4axis style) |
| licence | **undecided** (MIT would match FUXA and Open Industry Project) |

## 1. Architecture

```
┌─────────────────────────────┐      OPC UA (opc.tcp://127.0.0.1:4840)
│ Sysmac Studio NX simulator  │◄──── subscribe PLC outputs (actuator commands)
│ PLC program = the controller│────► write PLC inputs (sensors): batched, on change
└─────────────────────────────┘
               ▲
┌──────────────┴──────────────────────────────────────────────────┐
│ server/  (Node, headless)                                       │
│  opcua.js  THE single copy: session, browse+browseNext,         │
│            subscribe, batched write, reconnect, CLI             │
│  plant.js  Rapier world, fixed-step loop, attach/grip,          │
│            part sensors, IO exchange, recording                 │
│  main.js   CLI + HTTP: static, SSE /api/stream, POST /api/*     │
│ tools/gen_sysmac.js  scene → Sysmac XML, IO list, probe program │
└──────────────┬──────────────────────────────────────────────────┘
               │ SSE (30 Hz deltas) + POST
┌──────────────┴──────────────────────────────────────────────────┐
│ web/  index.html  app.js (viewer + IO panel)                    │
│       editor.js (builder)  charts.js (time chart, Gantt)        │
└─────────────────────────────────────────────────────────────────┘
 lib/  pure ES modules shared by BOTH sides, never import three or Rapier
       math.js  scene.js  components.js  analysis.js
```

**Why `lib/` matters.** A component describes its geometry as plain data (`box`, `cyl` and
`sphere` with size and offset). Node turns that into Rapier colliders and the browser turns it
into meshes.

Poses of moving machine parts come from **one** function, `worldPoses(scene, dof)`, called on both
sides. So the server streams **DOF values** (one number per cylinder), plus full poses only for
loose workpieces. The picture cannot disagree with the plant, because it is the same function with
the same inputs. This is the rb4axis rule "draw from `chainPoints()`", made general.

## 2. Scene model

A scene is a **flat list** of component instances, each naming its parent. A flat list is easier
to diff, undo and edit than a nested tree. Saves use a stable key order, so saving twice gives
byte-identical files.

```json
{
  "format": "mio-scene/1",
  "name": "cyl-on-slide",
  "sim": { "dtMs": 2 },
  "io": { "driver": "opcua", "endpoint": "opc.tcp://127.0.0.1:4840", "prefix": "GlobalVars.",
          "mode": "sim", "minPulseMs": 100 },
  "stations": [ { "id": "ST1", "name": "Press", "members": ["slide1", "cyl1"], "stepTag": "ST1_STEP" } ],
  "cycle": { "exitTag": "AS_ST1_PRSS_CYL_UP", "autoTag": "AUTO_RUN", "exclude": 1, "avgN": 10 },
  "components": [
    { "id": "base", "type": "frame", "params": { "size": [900, 600, 800] } },

    { "id": "slide1", "type": "servoLinear", "parent": "base", "socket": "top",
      "params": { "mode": "plant", "stroke": 500, "vmax": 300, "acc": 1500, "body": [640, 90, 70] },
      "io": { "target": "SV1_TGT", "exec": "SV1_EXEC", "actPos": "SV1_ACT_POS", "inPos": "SV1_INPOS" } },

    { "id": "cyl1", "type": "cylinder", "label": "PRESS CYLINDER", "station": "ST1",
      "parent": "slide1", "socket": "carriage", "at": [0, 60, 40], "rot": [180, 0, 0],
      "params": { "bore": 32, "stroke": 100, "rod": 12, "valve": "5/2-double",
                  "extendMs": 450, "retractMs": 380, "cushionMm": 8, "valveMs": 15,
                  "extWord": "DOWN", "retWord": "UP",
                  "switches": [ { "id": "ret", "pos": 2 }, { "id": "ext", "pos": 97, "band": 6 } ] },
      "io": { "solExt": "SOL_ST1_PRSS_CYL_DN", "solRet": "SOL_ST1_PRSS_CYL_UP",
              "sw.ret": "AS_ST1_PRSS_CYL_UP", "sw.ext": "AS_ST1_PRSS_CYL_DN" } }
  ]
}
```

**Attaching** is the core builder feature. A component mounts on a parent **link**, through a
named **socket** (a link plus a pose) with an optional `at`/`rot` offset on top. Leaving out the
socket means "relative to the parent's root link". Sockets are also the editor's snap targets.

Sockets can sit on a moving link. Here:
- `slide1.carriage` moves with the servo;
- `cyl1.rodEnd` moves with the rod.

So the cylinder rides the carriage, and a gripper on `cyl1.rodEnd` would ride the rod.
`worldPoses()` resolves the chain in parent-first order. `validate()` rejects missing parents,
cycles, unknown types or sockets, duplicate ids, and **two components writing the same tag**.

`rot` is in degrees, applied X then Y then Z. That rule is written once, in `lib/math.js`.

**Reed switches are part of the cylinder** (`params.switches`). They are not separate
components, because a reed switch only exists on a cylinder tube. The editor shows them as
markers you drag along the tube.

**Tag direction and type come from the component type's schema**, not from each instance:

- `out` = PLC → plant (actuator command);
- `in` = plant → PLC (sensor).

In twin mode the direction flips for the whole scene. The scene file is the **only** place names
and numbers live.

## 3. Component contract (`lib/components.js`)

Each type is a plain object in one `TYPES` map. There are no classes and no registry. Split the
file per family once it passes about 1000 lines.

```js
export const cylinder = {
  label: 'Air cylinder',
  params: [{ k: 'bore', type: 'enum', of: [6,10,16,20,25,32,40,50,63,80,100], def: 32, unit: 'mm' }, /* … */],
  io: p => ({ solExt: { dir: 'out', type: 'BOOL', dev: 'SOL', words: p.extWord },
              /* solRet if 5/2-double or 5/3, and 'sw.<id>': { dir: 'in', dev: 'AS' } for each switch */ }),
  links:   p => ({ body: {}, rod: { dof: 'prismatic', axis: [0,0,1], min: 0, max: p.stroke } }),
  sockets: p => ({ foot: { link: 'body', at: [0,0,0] }, rodEnd: { link: 'rod', at: [0,0,/*…*/] } }),
  shapes:  p => [{ link: 'body', kind: 'cyl', r: /*…*/, h: /*…*/, at: [/*…*/], mat: 'alu', collide: true } /* … */],
  states: ['retracted', 'extending', 'extended', 'retracting'],          // feeds the Gantt
  init: p => ({ x: 0, v: 0, spool: 0 }),
  step(s, p, io, dt) { /* valve → s.x; switches → io['sw.<id>'] */ },
};
```

### Actuator models
All of them are analytic and deterministic.

**Cylinder**
- Valve types:
  - `5/2-single`: spring return;
  - `5/2-double`: holds its last position;
  - `5/3-closed`: stops mid-stroke;
  - `single-acting`.
- **You enter the stroke times you measure on the machine with a stopwatch.** The model works out
  the speed that gives that time, using `v = ((stroke − c) + c/k) / (T − valveMs)`, where `c` is
  the cushion length and the cushion runs at `k·v`. Those times are the calibration knob, and twin
  mode can write measured times back into them.
- A reed switch is ON while `|x − pos| ≤ band/2`, with hysteresis.
- **Stopper, pusher, clamp, lifter and pneumatic rotary are presets** (parameter bundles plus a
  shape), not new code.

**Servo** (linear or rotary) has three modes:

| mode | behaviour |
|---|---|
| `plant` | The PLC writes target, velocity and execute. The plant runs the trapezoid profile (`langkahSumbu` from rb4axis `kin.js:656`, ported once) and publishes actual position, in-position and busy. **Execute is a level held until Done**, because OPC UA sampling misses one-scan pulses. |
| `positions` | A named position table `[{name, mm}]` with `cmd.<name>` / `ls.<name>` keys. These map 1:1 to the sysmac generator's `SRV_CMD` / `SRV_LS`. |
| `mirror` | The DOF is read from a tag. The PLC runs real `MC_*` function blocks on a simulator axis, and a generated shim copies `Act.Pos` into a published LREAL global. **This is the important mode for Sysmac users**: the motion program being tested is the real one, and the axis is never simulated twice. The generic `joint` component uses the same mode for rb4axis's `SIM_JOINT_POS[i]` and its hinged covers. |

In mirror mode the plant smooths the irregular PLC samples by predicting from velocity, capped at
120 ms (rb4axis `haluskan`).

**Index table** (drive `cam`)
- While `run` is held, the camshaft turns.
- The table angle is `pitch·(k + cyc(frac))`, using the cycloidal profile `s = h(τ − sin 2πτ / 2π)`.
- `inPos` is true only during dwell; `origin` is true at station 0.
- Stopping mid-index leaves the table where it stopped, as on a real indexer.
- Drive `servo` reuses the servo `positions` mode.

**Gripper**
- The fingers stop at the part's width when a part is inside the grip zone.
- The closed switch sits at *full* close, so a missed grip is visible to the PLC as it is on a
  real machine (rb4axis `SIM_GRIP_TUTUP`).

**Vacuum cup:** ON with a part within 2 mm → the vacuum switch turns on after `buildMs` and the
part attaches.

**Conveyor: friction-clamped slip** (spike A0, 2026-09-12). The belt collider has friction 0
(combine rule `Min`), so Rapier's own friction never brakes a part against the static belt.
Each step, a part in contact with the belt (`contactPair`, at least one contact) gets an
impulse. The impulse pulls its in-plane velocity toward the belt's by at most `μ·g·dt`
(`beltDv()` in `server/plant.js`), so a blocked part **slips**, as it does on a real belt.

Measured on a 3 m belt at 0.3 m/s, with 60×40×30 mm alu parts, μ 0.5, and 5 parts against a
stopper for 10 s:

| model | 1 m arrival | queue gaps (60 mm parts) | jitter | parts climb |
|---|---|---|---|---|
| velocity override (the first plan) | 3.336 s | 0.6 / 49.8 / 45.3 / 52.7 mm | 0.005 mm | yes, zmax 42.8 mm |
| **friction-clamped slip** | 3.366 s = d/v + v/(2μg) | 59.3 / 59.5 / 59.7 / 59.8 mm | 0.000 mm | no, z 15.08 mm |

Override keeps shoving blocked parts (residual 128 mm/s) and stacks them. Slip costs 76 µs
per step with 5 parts. Both runs are deterministic. `tests/rapier.test.js` pins these numbers.

**Others**
- emitter and remover;
- `process`: start → busy for T seconds → done (the rb4axis ICC 17 s tester, the DW writer);
- motor: spinning DOF plus an at-speed output;
- panel parts: pushbutton (momentary or alternate, with lamp), selector, e-stop (latching NC),
  lamp, tower lamp, buzzer. Panel parts are clickable in 3D.

### Sensors come in two kinds
- **About the machine, analytic from DOF values:** reed switch, servo in-position, table origin.
  These need no physics, which matches reality: a reed switch senses the piston magnet.
- **About parts, Rapier queries in Node:**
  - photoelectric (diffuse, retro, through-beam), fiber and light curtain use `castRay`;
  - proximity uses `intersectionsWithShape`, with an optional metal-only filter.

  All of them have NO/NC and `offDelayMs` settings. There are no sensor colliders and no event
  queues.

### Holding (one mechanism)
Gripper, vacuum cup, nest/fixture, index-table pocket and pallet all hold a part the same way:

1. **Take:** set the part's Rapier body to kinematic and store its pose relative to the holder.
2. **Each step:** move the part to `holder world × relative pose`.
3. **Release:** set the body back to dynamic, starting at the holder link's velocity (rb4axis
   `fisikaJatuhkan`).

This is exact and deterministic. A fixed joint between a kinematic and a dynamic body fights the
solver.

**The hand (jam testing, "ijiwaru").** The viewer can click and hold any loose part: `POST
/api/hold {uid, down}` → `plant.holdPart()`, which is take/follow/release again with the **world**
as the holder, so the part stays exactly where it was grabbed. The belt then slips under it and
the parts behind it queue up, which is what a real jam looks like. Dragging sends `at` (mm, world)
and moves the part instead, in the plane facing the camera.

The hand **does** take a part out of a gripper, a cup or a nest, and clears that holder's `s.uid`,
so its vacuum or present switch goes false. That is the interesting test: the machine carries on
believing it holds a part, and the PLC has to raise the alarm. It releases with zero velocity, and
its edges are queued and applied at the start of a step, so a recording still replays identically.
The plant reports the held uids in `snapshot().pins`, and the viewer highlights them. This is failure injection on the
**material flow**; forcing (§5) is failure injection on the **IO image**. They find different
bugs: forcing asks "does the PLC handle a lying sensor", the hand asks "does the sequence handle a
part that stopped moving".

Moving machine links are `kinematicPositionBased` bodies, driven by
`setNextKinematicTranslation/Rotation`, so pushers and stoppers really push parts. **Actuators are
never dynamic bodies, and nothing in physics writes to the PLC.**

## 4. SPM component kit (≈ 20 types plus presets)

| group | components |
|---|---|
| pneumatics | cylinder (bore, stroke, rod, valve, cushion, switches), presets for guided cylinder, stopper, pusher, clamp, lifter, rotary 90/180°; 2-finger gripper; vacuum cup + vacuum switch |
| sensors | photoelectric (diffuse / through-beam / retro), fiber, inductive/capacitive proximity, light curtain, safety door switch, QR/ID reader (returns the workpiece ID, like `CE_SIM_QR` in ceinsert) |
| motion | servo linear/rotary (plant / positions / mirror), **index table** (N stations, cam or servo drive), generic `joint` (mirror), robot arm (rb4axis 1P+3R `chainPoints` chain) |
| process | `process` station (timer + optional hinged cover), press unit (pass/fail window), drill/screw unit (spindle + feed), inspection/marking (OK/NG tag) |
| material flow | belt conveyor, roller conveyor, pallet with stopper/lifter, chute/gravity feeder, emitter, remover |
| operator | pushbutton, selector 2/3-position, e-stop, lamp, tower lamp, buzzer, numeric display |
| structure | frame, plate, profile, fixture/nest, guard |
| items | workpiece: box/cylinder/plate, size, colour, material (for inductive sensors), ID |

## 5. Plant loop and IO timing

**Step size and pacing**
- Fixed `dt = 2 ms` (`sim.dtMs`, 1–10). A part at 500 mm/s crossing a 10 mm beam gives 20 ms, i.e.
  10 samples.
- Windows timers fire about every 15 ms, so each tick runs `floor(elapsed/dt)` steps from an
  accumulator, **capped at 50**. Anything past the cap increments `overruns`, which the UI shows.

**Step order**
1. Apply the PLC outputs that have arrived.
2. `component.step()` for each component.
3. `worldPoses()`, then set the kinematic targets.
4. Conveyor drive and held parts.
5. `world.step()`.
6. Part sensors.
7. Diff the IO image and record the edges.

**Time scale**
- **1× whenever a PLC is in the loop**, because Sysmac timers run on wall time and the simulator
  cannot be scaled.
- `internal` and `replay` modes allow 0.1–10× or as fast as possible. Factory I/O allows scaling
  with a PLC and warns that sensors get missed; this plan forbids it.

**IO exchange (OPC UA)**
- PLC outputs come through one subscription at 10 ms sampling. **Measured on the Studio 1.66
  simulator** (docs/SETUP.md):
  - the effective sampling floor is ~16 ms, the Windows timer tick;
  - publishing is floored at 50 ms, so outputs arrive in batches every ~50 ms, about 3
    samples per batch.

  The recorder stamps `out` edges with each sample's PLC source timestamp, which gives ~16 ms
  resolution instead of 50 ms.
- Changed sensor values go out in **one batched `write`** per exchange, with at most one batch in
  flight.

**Short sensor pulses.** A blip that falls between two write batches never reaches the PLC. Two
layers stop that:
- the sensor's own `offDelayMs`, a real setting on Omron and Keyence photo-eyes;
- a scene-level `minPulseMs` hold, whose default comes from the Phase 0 measurement:
  **20 ms**. The simulator counted every pulse from 10 ms up, so the hold covers the plant's own
  exchange tick, not the PLC (docs/SETUP.md §4).

Each stretch is recorded as a `warn` event, e.g. `PH_ST1_EXIST 12 ms → 100 ms`. Do not silence it.

The hold is for **physical events the PLC does not cause**: reed switches, photo-eyes,
pushbuttons. Replies to PLC commands are exempt with `hold: false` in the type's io schema
(servo `done`, `busy`, `inPos`). Every short pulse of theirs is caused by the PLC: Done falls
because Execute dropped, InPos falls because the next move started. Found in P1: stretching them
slowed every handshake by 100 ms, warned on every move, and reported "in position" while the
axis was already moving.

**PLC → plant commands must be levels or counters, never one-scan pulses.** Events shorter than
sampling (part counts, drops, rejects) are published as **counters** in both directions.

**Overwrite detection.** The plant also subscribes to the tags it writes. If the PLC keeps a
different value for more than two sampling intervals, it warns "overwritten by the PLC (coil on
this tag?)". That is the ceinsert `NX_SerialRcv_FB_done` lesson, detected automatically.

**Heartbeat.** The probe program increments `MIO_HEARTBEAT`. If it stops, the UI says "program not
running or not assigned to a task", which is the top cause of "tags read but nothing moves".

**Forcing and failure injection** (always on / always off; Factory I/O parity) act on the plant's
IO image, so the PLC sees them too.

**Replay.** `--replay run.ndjson --fast` feeds the recorded PLC outputs back in and reports the
first sensor edge that differs. It relies on the deterministic Rapier build, a fixed insertion
order and a fixed dt.

## 6. Transport, messages, security

SSE on `/api/stream`. Each broadcast is serialised once and written to every viewer.

```
event: scene   {"v":7,"scene":{...}}                          on connect and after each save
event: state   {"t":123456,"dof":{"slide1":250.31,"cyl1":87.2},
                "parts":[["p17",x,y,z,qx,qy,qz,qw]],"gone":["p12"],"io":{"SOL_ST1_PRSS_CYL_DN":true}}
               30 Hz deltas, rounded to 0.01 mm / 1e-4; full snapshot on connect and every 5 s
event: status  {"io":{"driver":"opcua","ok":true,"msg":"41 tags","samplingMs":50,"rttMs":38,"heartbeat":true},
                "plant":{"mode":"run","t":123456,"overruns":0,"stepUs":180}}   1 Hz
event: warn    {"t":123456,"msg":"pulse stretched PH_ST1_EXIST 12->100 ms"}
```

Requests:
- `POST /api/cmd {op: run|stop|reset}`
- `POST /api/press {id, key, down}`
- `POST /api/force {tag, value|null}`
- `PUT /api/scene/:name {baseVersion, scene}`, which returns 409 when `baseVersion` is stale
- `GET /api/scenes`, `/api/runs`, `/api/run/:id` (NDJSON), `/api/tags` (browse cache for
  autocomplete), `/api/ping`

**Smoothing.** The browser renders at sim time minus 50 ms and **interpolates** between frames.
Frames carry sim timestamps and arrive at a steady rate, so interpolation is exact and never
overshoots. Velocity prediction lives in the plant, only for mirrored DOFs (§3).

**Security**
- Bind to 127.0.0.1 by default.
- `--lan` allows GET from the LAN. POST stays localhost-only unless `--lan-control` is given.
- Keep rb4axis's origin check.
- Scene names must match `^[a-z0-9_-]+$`.
- Static files are served only from fixed prefixes (`/web /lib /vendor/three`), and `..` is
  rejected.
- Internal controllers (`scenes/<name>.ctl.js`) load only from disk, never through the API.

## 7. Recording and analysis (what Factory I/O does not have)

**Recording**
- Events go to a 200k in-memory ring and to `runs/<iso>_<scene>.ndjson` (gitignored).
- The first line is a header with the scene hash, driver and dt.
- Event kinds: `{t,k:"out"|"in",tag,v}`, `{t,k:"step",st,v}`, `{t,k:"part",id,ev}`,
  `{t,k:"warn",msg}`, `{t,k:"mark",label}`.

**`lib/analysis.js`** is used by the browser, by `node server/main.js --report run.ndjson`, and by
the tests.

| view | content |
|---|---|
| **time chart** | a logic analyzer: BOOL traces, analog traces, A/B cursors with Δt (e.g. `SOL_…_DN↑ → AS_…_DN↑ = 0.45 s`), wheel zoom, drag pan, per-pixel min/max decimation |
| **Gantt** | built **without touching the PLC program**. Per actuator, from the command edge to the switch arriving: `ext-moving / extended / ret-moving / retracted`. Per station, from `stepTag` values when one is configured. |
| **cycle time** | out-to-out on `cycle.exitTag` rising edges or on remover `exit` events; per station from its exit edge |
| **line balance** | busy vs waiting per station; the bottleneck is the station with the highest busy % |
| **OEE-lite** | availability = auto time / session time; performance = ideal CT × count / auto time; quality = 1 unless an `ngTag` counter is set |
| **compare** | two runs aligned at the Nth cycle start, B dashed, plus a per-actuator table of mean durations with Δ |
| **export** | CSV of events and of the per-cycle table |

The cycle-time rules are carried over from rb4axis:
- measured **out-to-out**;
- the first `exclude` parts are skipped;
- the clock runs only while `autoTag` is true;
- the average is divided by the **number of samples**, never by a fixed N.

Charts are drawn on plain canvas, like rb4axis `gambarGrafik`, with colours taken from CSS
variables.

**PLC-side trace** (Phase 4+, optional). The generator emits `MIO_TR_T/V[0..255]` plus an index
counter, for when ±1 sampling interval per edge is too coarse. That is the rb4axis rule: anything
shorter than sampling is recorded on the PLC side.

## 8. Digital twin mode

The scene is the same, with `io.mode: "twin"`:
- **Every binding is read-only, enforced inside the driver's `write()`**, not in the UI. Never
  write to a real machine.
- Servos and joints are mirrored from the real tags.
- Cylinder DOFs are still **modelled from the SOL outputs**, because a real machine only reports
  its AS switches, not rod position.
- The model's switch outputs are compared with the real AS inputs, which gives **divergence
  markers**.
- **"Apply measured stroke times"** writes the measured SOL→AS delays back into
  `extendMs`/`retractMs`.
- The main use is **drift detection**: per-actuator times across shifts. A cylinder going from
  0.45 s to 0.61 s in a week means an air leak or a cushion problem, seen before the machine stops.
- Real runs use the same NDJSON and analyzer as simulated ones.
- Workpieces are optional, since the real machine reports no part pose.

## 9. Sysmac integration (`tools/gen_sysmac.js`)

- **Ported code.** `esc/tipeXml/varXml/globalXml` and the program wrapper come from rb4axis
  `tools/gen_xml.js`. Arrays use `ArrayTypeSpec`, `<ST>` uses LF, and the copy has its own tests.
- **Scene → `<name>.sysmac.xml`.** Every bound tag becomes a GlobalVar with
  `networkPublish="PublishOnly"` and the type from its schema (BOOL / LREAL / UDINT / INT /
  STRING). If `scenes/<name>.st` exists, it is added as `PRG_<NAME>`, never with a `P_` prefix.
  Importing GlobalVars *adds* variables (proven in ceinsert); importing a POU with an existing name
  *replaces* it.
- **`--probe`** generates `PRG_MIO_PROBE`:
  - `MIO_HEARTBEAT`, +1 every scan;
  - `MIO_ECHO_OUT := MIO_ECHO_IN`;
  - `MIO_PULSE_CNT`, which counts rising edges of `MIO_PULSE_IN`.
- **`--shim`** generates `MIO_AXn_POS := <axis>.Act.Pos` for mirror-mode servos.
- **`--iolist` → `<name>.io.tsv`**, in the sysmac generator's format
  `ADDR\tTYPE\tIN|OUT\tCOMMENT` (types PB CR LS SS PH PL BZ AS SOL):
  - Comments follow `ST<n> <label> <words>`, e.g. `ST1 PRESS CYLINDER DOWN`. A SOL and its AS share
    the same comment stem, so sysmac's `findLsc` pairs them.
  - Inputs fill `CH0_00` upward, and outputs start on the next channel. A re-export keeps the
    address of any comment that did not change.
  - A test rejects words like "(virtual)" in comments, because the generator turns every word into
    part of the name.
- **`--bind <GlobalVariables.tsv>`** fills the scene's `io` names by matching the comment column
  exactly, so there is no second copy of the name generator.
- **Result:** scene → IO list → sysmac generator → program skeleton (Device_Input / AutoRunning /
  Fault …) → import → the Sysmac simulator runs it against the same scene. **3D machine design to
  a tested PLC program, in one loop.**
- **Upstream change in the sysmac repo:** `js/gen_all.js` `tsvRow` (about line 2134) hardcodes
  `"Do not publish"`. Add a project flag `networkPublishIo` so device IO is `PublishOnly`; `gvr()`
  already supports `e.publish`. Until then, bulk-edit that column in Studio.
- **`--check`** compares all generated files byte for byte. The XSD check uses sysmac's
  `scripts/validate_xml.ps1`, with a **loud** SKIP when it is absent.
- **Documented individual Studio steps** (they cannot be automated):
  1. import;
  2. Build;
  3. **assign the program to the primary task**;
  4. Run (F5);
  5. *Simulation → Use the OPC UA Server*;
  6. security None + anonymous Permit;
  7. start the plant.

  The simulator's OPC UA server needs Studio ≥ 1.62.

## 10. Editor (`web/editor.js`)

- **Modes.** Edit mode stops physics and keeps the scene local to the browser. Save validates
  with the same `lib/scene.js validate()` the server uses, then sends `PUT` with `baseVersion`. The
  server rebuilds the plant and broadcasts the new scene.
- **Placing and selecting.** A palette lists every type and preset. Click to place; raycast to
  select.
- **Moving.** three `TransformControls` edits the mount offset: the world-space move is converted
  into the parent link's frame. Grid snap is 1/5/10 mm and rotation snap 15°/90°. Since r169
  `TransformControls` is not an Object3D, so add it with `scene.add(tc.getHelper())`.
- **Attaching.** Pick a target and its sockets show as spheres; clicking one sets `parent` and
  `socket`. Shift-drag snaps to any socket within 20 mm.
- **Properties.** The panel is generated from `params`, and cylinder switches are draggable
  markers on the tube. Tag fields **autocomplete from the PLC browse cache** (`/api/tags`), which
  removes typos, the top silent failure. Two components writing the same tag are flagged.
- **Other tools.** Hierarchy tree, rename, copy/paste of a subtree with new ids, and undo/redo as
  up to 100 JSON snapshots (the sysmac editor pattern, so no mutation path can be missed).
- **Cameras.** `OrbitControls` from `three/addons/` plus saved views.
- **UI rules from rb4axis:**
  - build panels once and update only text, about 8 times a second;
  - sliders write on `change`, not on `input`;
  - redraw labels only when their text changes;
  - fit the shadow camera to the scene's bounding box.

## 11. Scene library

| when | scene | shows |
|---|---|---|
| P1 | **cyl-on-slide** (hello world) | button → PLC → valve → cylinder on servo slide → reed → PLC → lamp |
| P3 | From A to B | conveyor, photo-eye, emitter/remover |
| P3 | Sorting by Height | height photo-eyes plus a pusher cylinder with reed switches |
| P3 | Separating Station | escapement stoppers, a core SPM pattern |
| P3 | Buffer Station / Queue of Items | stoppers and pallets |
| P3 | Pick & Place (XZ pneumatic) | two-axis pneumatic pick and place with vacuum or gripper |
| P3 | Assembler | lid onto base on an **index table** |
| P3 | sort-by-material | inductive proximity (metalOnly) latched upstream of the stop beam; steel and plastic templates |
| P3 | gripper-transfer | 2-finger gripper on a pneumatic lift on a pneumatic traverse, cylinder mounted on a rod end, a cross belt; a missed grip is a FAULT |
| P3 | press-station | process station without a belt: feeder into a clamped nest, press with a dwell, ejector onto a chute |
| P3 | palletizing | the heaviest scene: a 10 x 10 pallet of 100 spark plugs, a five-up vacuum gantry on two servo axes, a rotary carrier with five-slot jigs, an unload head. It is what pins the pose cache and the step budget |
| P5 | **CE Insert Track** (ceinsert) | ST1 stoppers/dividers, ST2 buffer with servo in mirror mode, ST3 ejector/pusher, QR reader |
| P6 | **Blurobot cell** (rb4axis) | rail + 3R arm (mirror joints), process stations with covers, physical PCBs |
| P6 | press-fit, drill, seaming | lifter/radial cylinders, motor, servo, nest, index table; press-fit adds a nest that captures a pin at seat depth |
| P6 | Converge Station, Production Line, Sorting by Weight, Elevator | the same primitives |
| skip | Filling Tank, Level Control, Batching | fluids and analogue processes, not SPM |
| later | Palletizer, Automated Warehouse | need a gantry or stacker; possible later with servo mode |

Each scene can include `scenes/<name>.st`, a PLC demo program, and `scenes/<name>.ctl.js`, an
internal controller used for tests and demos without a PLC. The UI marks the internal controller
in yellow ("INTERNAL CONTROLLER — not a PLC"). Scene tests check numbers and tags against the
source repos (rb4axis `robot.config.json`, ceinsert `extract/variables.tsv`), with a loud SKIP
when those repos are absent.

**Left out on purpose (YAGNI):**
- Factory I/O `.factoryio` import (it is plain XML, so it can come later);
- first-person camera and VR;
- STL/STEP import (until box/cyl/sphere is not enough);
- cylinder force/stall modelling (until press or clamp checks need it);
- Modbus (until a non-Omron PLC needs it).

## 12. Repo layout (grows only when a phase needs it)

```
manufacturing_io/
  package.json  package-lock.json  .gitattributes (* -text)  .gitignore
  README.md  CLAUDE.md  docs/PLAN.md  docs/SETUP.md (Studio steps + symptom → cause)
  server/  main.js  opcua.js  plant.js
  lib/     math.js  scene.js  components.js  analysis.js
  web/     index.html  app.js  editor.js  charts.js
  tools/   gen_sysmac.js
  scenes/  <name>.json  [<name>.st]  [<name>.ctl.js]  <name>.sysmac.xml  <name>.io.tsv
  tests/   run.js  lib / plant / opcua / sysmac / analysis / web .test.js
```

## 13. Phases

Each phase ends runnable, with a written exit criterion.

### P0 — bootstrap, OPC UA port, latency
- `package.json` with exact pins, and the test harness.
- `server/opcua.js`, ported from rb4axis `bridge/bridge.js` and translated to English:
  - certificate manager with an explicit `rootFolder`;
  - browse with `browseNext`, and path-then-suffix lookup;
  - read-once type metadata;
  - `plain()` for typed arrays;
  - whole-array writes;
  - reconnect with re-browse;
  - the empty-tree diagnostic;
  - CLI `--tree/--list/--write/--watch`.

  This becomes **the** copy. The ceinsert and sysmac browse loops lack `browseNext`.
- `gen_sysmac.js --probe`.
- `main.js --latency` measures:
  - 200 echo round trips with random spacing (p50/p95/max);
  - the revised sampling and publishing intervals;
  - tick jitter;
  - a pulse table for 10/20/50/100/150/200 ms × 20 each, giving the detection rate per width.
- Check whether OPC UA can write **AT-assigned** variables in the simulator. The answer decides
  whether the sim can reuse the real machine's AT-mapped IO names.
- A Rapier deterministic-compat smoke test in Node: kinematic bodies, `castRay`,
  `intersectionsWithShape`. rb4axis used 0.14, and the API changed since.
- **Exit:**
  - tests green, with the live parts SKIPping loudly;
  - with the probe imported and assigned: `--tree` finds `MIO_*`;
  - `--latency` writes `runs/latency-*.json`, and the numbers are copied into `docs/SETUP.md`;
  - the default `minPulseMs` is the smallest width detected 100% of the time, plus margin.

### P1 — first SPM primitive, round trip with the Sysmac simulator
- `lib/math.js` and `lib/scene.js`.
- Components: frame, plate, cylinder with switches, servoLinear (plant mode), pushbutton, lamp,
  static workpiece.
- `plant.js`: fixed and kinematic bodies only, IO exchange, `minPulseMs`, overwrite detection,
  NDJSON recording.
- `main.js`: HTTP and SSE. `app.js`: viewer and IO/force panel.
- The `cyl-on-slide` scene with a `.st` sequence and an equivalent `.ctl.js`.
- **Exit:**
  - the 3D pushbutton starts the PLC sequence;
  - the cylinder rides the slide;
  - the PLC sees the reed switches at the configured stroke positions (checked with Watch in
    Studio);
  - moving a switch's `pos` changes when the PLC sees it;
  - stroke-time error ≤ 1 step;
  - 0 overruns in 10 minutes;
  - the NDJSON holds every edge.

### P2 — parametric builder (editor)
- `editor.js` and the rest of §4 that does not need loose parts: presets, index table, gripper,
  sensors about the machine, panel parts.
- **Exit:**
  - P1's scene rebuilt from empty in under 10 minutes without touching JSON;
  - two saves byte-identical;
  - 50 undo/redo steps work;
  - tag autocomplete lists the PLC's tags;
  - an invalid scene cannot be saved.

### P3 — parts and material flow
- **Status (2026-09-12).** Done:
  - loose parts (CCD, never sleep, lost/NaN guard, streamed as transforms);
  - the conveyor (spike A0: slip + friction torque);
  - emitter/remover;
  - photo-eye/proximity;
  - cylinder `head` plus the Stopper/Pusher/Lifter presets;
  - the `ref` param type;
  - holding (vacuum cup, 2-finger gripper, nest): one take/follow/release mechanism;
  - the hand: hold a part still from the viewer to jam the line on purpose;
  - the index table (cam drive, cycloidal profile, `inPos` only in the dwell);
  - the scenes **a-to-b**, **stopper-pusher**, **pick-place**, **assembler**, **sort-by-height**
    and **buffer-queue**, each with `.st` and `.ctl.js`;
  - the contact refresh for kinematic links that come to rest and for parts whose body type
    changes.

  Soak, 30 min of sim time with the internal controller:

  | scene | cycles | parts in = out + inside | lost / NaN / warnings | worst step |
  |---|---|---|---|---|
  | a-to-b | 271 | 272 = 271 + 1 | 0 / 0 / 0 | 69 µs |
  | stopper-pusher | 718 | 721 = 718 + 3 | 0 / 0 / 0 | 257 µs |
  | pick-place | 277 | 278 = 278 + 0 | 0 / 0 / 0 | 108 µs |
  | assembler | 539 | 1080 = 1075 + 5 | 0 / 0 / 0 | 251 µs |
  | sort-by-height | 245 | 246 = 245 + 1 | 0 / 0 / 0 | 408 µs |
  | buffer-queue | 665 | 681 = 666 + 15 | 0 / 0 / 0 | 441 µs |

  The budget is 1000 µs (50 % of dt).

  **Live on the simulator (2026-09-14, a-to-b), two rounds.** The sequence raced, and the fix
  took two goes:

  1. Waiting for "the unloader count moved" let a part removed during the load satisfy the
     discharge step at once, so the cycle never waited for its own part: 231 parts in, 217 out,
     and the crowd crossing the end sensor raised 13 pulse-stretch warnings.
  2. Snapshotting the count on entry balanced the parts (388 in, 383 out, 5 reset) and silenced
     the warnings, but the recording showed the cycle running at **1.09 s instead of 6.5 s**
     (388 cycles in 423 s) with about five parts pipelined on the belt: the sequence was still
     riding other parts' removals, and step 20 was seen 5 times in 388 cycles.

  A waiting step now waits for an **invariant** — `RM1_CNT = EM1_CNT`, everything loaded has
  left — which is also self-healing, since a pipeline drains back to one part. The internal
  controller never showed any of it, because there the counters only moved during the waiting
  step. `tests/ctl.test.js` drives each controller against a faked plant to pin it.

  **All six controllers are now audited that way.** Driving the other four against a faked plant
  whose counters and beams move at awkward moments found no further races: stopper-pusher clears
  the beam on both branches before judging the next part, assembler waits for the station its
  index was sent to, and sort-by-height and buffer-queue wait on invariants and on a clear beam.

  **Read step times from a headless run with nothing else on the box.** The plant's accumulator
  measures wall time, so anything sharing the CPU shows up as plant cost:
  - with headless Chrome on SwiftShader beside the server, pick-place read 839 µs against the
    108 µs it soaks at;
  - buffer-queue soaked at 1232 µs (over budget) while the test suite and a browser shot ran
    alongside, and at 441 µs alone — same 665 cycles and same part counts both times, because
    the simulation is deterministic and only the clock moved.

  A soak that shares the machine measures the machine, not the scene.

  **A pile of parts is cheap, and it is meant to sit still** (measured 2026-09-14, 32 parts fed
  against a wall on a running 300 mm/s belt, nothing else on the box):

  | | free-running | through the real-time pacer |
  |---|---|---|
  | step | 350 µs | 510–590 µs |
  | sim behind wall | — | 0.0 s at every 5 s sample |
  | overruns / warnings | 0 / 0 | 0 / 0 |

  The pile's top speed falls from 982 to 5 mm/s in about 15 s and stays there, because the belt
  wedges the parts against each other. "It takes a long time to break up" is that, not a
  performance problem. `compile()` per rendered frame looked like the culprit and measured ~1 µs;
  it is cached per build now, but it was never the cost.

  **The hand found a real hole in the sequences (2026-09-14).** Denny reported "banyak yang
  stall" after the hand shipped. The plant timer was innocent — a-to-b idle with a browser
  attached ran 120 s with 0 stalls, and 150 s of grabbing, dragging and burying parts in
  pick-place cost 128 µs/step with 0 overruns. What stalled was the MACHINE:

  | scene | what the hand did | where it stuck |
  |---|---|---|
  | a-to-b | dragged a part through the floor | step 40 for ever, belt running, AUTO_RUN on, EM 2 / RM 1 |
  | pick-place | took the part out of the nest | step 20 for ever, waiting for `NEST_A_P` |

  Two fixes, both in the controller and its `.st`:
  - a **watchdog**: a step that has not moved for 15 s goes to FAULT (step 900), outputs safe,
    `AUTO_RUN` off, START acknowledges. The cup keeps its part;
  - a **write-off**: acknowledging the fault takes `GONE = EM − RM`, and the discharge invariant
    becomes `RM + GONE >= EM`, so a part removed by hand does not stop the machine for good.

  The write-off belongs to the fault acknowledgement and nowhere else. Doing it at every START
  was the first attempt and `tests/ctl.test.js` caught it: it writes off parts that are still
  legitimately on the belt, so an older part's removal ends this cycle — the pipelining bug that
  cost two live rounds on the simulator.

  **A write-off also goes stale, and that is silent** (found by reviewing the rollout diff with
  the `code-review` skill, and reproduced in `tests/ctl.test.js` before fixing). A part written
  off can still turn up — the hand puts it back, or it reaches the unloader later. `RM` then
  catches up, `RM + GONE` runs past `EM`, and the discharge step stops waiting at all:

  | | EM | RM | GONE | `RM + GONE >= EM` |
  |---|---|---|---|---|
  | after the acknowledgement | 1 | 0 | 1 | — |
  | the stray part turns up | 1 | 1 | 1 | — |
  | next cycle loads its own part | 2 | 1 | 1 | **true at once**, so the cycle ended with its part still on the belt |

  In ST it is worse: `GONE` is a `UDINT`, so `EM - RM` with `RM` ahead underflows to about 4
  billion and the invariant holds for ever. The write-off is therefore clamped every scan: never
  more than `EM - RM`, never below zero.

  All six scenes carry the watchdog now (15 s per step; 25 s on buffer-queue, where metering waits
  for a part to travel the whole buffer belt and the buffer can legitimately start empty).
  sort-by-height writes off both counters on the acknowledgement, since it has two invariants.
  The fault stops what feeds and moves and leaves what holds: the stopper stays down, the vacuum
  keeps its part, the press lifts off the work.

  **Pallets (2026-09-14).** A `pallet` is a PART, not a holder: it rides the belt on friction and
  what it carries rides it the same way. Its rails DO collide (they are shorter than the load, the
  measured exception to the pocket rule) — without them a pallet stopping against a stop let its
  part slide 70 mm along the deck. A `palletLift` holds at its `lift` link (`holdLink`) so the
  pallet rises with it, and filters on `holdOnly: 'pallet'` so it takes the carrier and not the
  load. The scene is `pallet-line`: feed a pallet, stop it, lift it, load a part onto it, put it
  back down and release it. Three things were measured the hard way:

  | tried | what happened |
  |---|---|
  | overhead stop pin | holds the pallet 7.9–15.1 mm after retracting; the clearance sweep is non-monotonic (5.1 held, 8.1 free, 11.1 free, 14.1 HELD, 17.1+ free) |
  | lift deck flush with the belt | blocks the path outright: the pallet stopped dead with its front edge on the deck edge |
  | lift deck sunk, with `snap` | snapping teleports the pallet down to the holder frame, so it fell out of the station beam and back in: `pulse stretched PE_STN 8 -> 20 ms`, once per cycle |

  So the stop POPS UP from under the belt, the deck stays below the belt line, and the lift does
  not snap — a pallet that arrived flat is already square. `tests/plant.test.js` pins the overhead
  trap against the pop-up rule.

  A fourth one showed up only in the 3D view, and only because Denny looked: the sequence cut the
  belt on the station beam, which stopped the pallet **106 mm short of the pin** (centre 94.1
  instead of 190.6), so the part feeder dropped its load onto the belt behind the deck, 118.9 mm
  out. A beam says a part is here; the stop is what locates it. Step 35 keeps the belt running for
  700 ms after the beam (106 mm at 250 mm/s is 424 ms) so the pallet is pressed against the pin,
  and the load then lands 0.2 mm from the deck centre on every cycle.

  Done since (2026-09-16): the watchdog and FAULT step are in all thirteen controllers, and the
  operator panel (selector AUTO / INDIVIDUAL, individual buttons with PLC-side toggle memory,
  selector change while running = FAULT) is on twelve of them. `tests/plant.test.js` drives the
  panel end to end. Pallets are done: `pallet-line` got the panel as part of the merge.

  Open:
  - **blurobot has no operator panel.** It passed the "every motor takes the override" and "the
    panel has the speed dial" checks only because those are gated on motors and on `servoLinear`,
    and its axes are `joint`s — so neither rule ever looked at it. Giving it the panel means
    giving `joint` an `ovr` and a jog pair first, or it is a half panel again;
  - the live PLC runs of the newer scenes.

  **buffer-queue meters, it does not accumulate.** The belt itself is the stop, because a pin
  coming down between parts that touch lands on a part (the escapement that stopper-pusher
  replaced). So the line moves as a block and parts stay 80–440 mm apart instead of packing.
  Accumulation against a stopper, with the belt slipping underneath, is proven separately in
  `tests/rapier.test.js`: five parts queue 59.3–59.8 mm apart with no jitter.

  stopper-pusher replaced the planned two-stopper escapement. Parts queue touching, so a stopper
  coming down between them lands on a part.
- Dynamic parts with CCD, conveyor (spike first), emitter/remover, holding (gripper/vacuum/nest),
  part sensors, pallets.
- The six P3 scenes, each with a `.ctl.js`; the first two also get `.st` programs.
- **Exit:**
  - each scene runs 30 minutes in internal mode;
  - the part count balances (in = out + inside) with no NaN;
  - step time is under 50% of dt;
  - Pick & Place completes a round trip with the PLC;
  - two runs give identical event logs.

### P4 — analyzer
- Time chart, Gantt, cycle time, line balance, OEE-lite, compare, CSV, `--report`, `--replay`.
- **Exit:**
  - cycle time on the P3 scenes matches the PLC's own counter within ±1 sampling interval;
  - a deliberately slowed cylinder shows up as the bottleneck;
  - the CSV opens in Excel.

### P5 — Sysmac IO list and ceinsert
- `--iolist`, `--bind`, `--shim`, and the upstream sysmac `networkPublishIo` flag.
- The CE Insert Track scene.
- **Exit:**
  - the IO list goes through the sysmac generator into a program;
  - `--bind` fills 100% of the names;
  - a copy of the real ceinsert project cycles its stations in the simulator.

### P6 — more machines
- Blurobot cell, press-fit, drill, seaming, then the remaining library scenes.
- **Exit:** the rb4axis PLC program drives the Blurobot scene.

**The Blurobot CELL runs (2026-09-15).** The first cut was a simplification — one belt in, one belt
out — and Denny rejected it: the real machine is a buffered cell with two ICC testers and two DW
writers. It is now a port of the actual project, taken from `sim/robot.config.json` and
`PRG_SIM_ROBOT.st` rather than re-derived from a description:

| station | type | rail x | surface z | process |
|---|---|---|---|---|
| WIP IN | 0 | −1400 | 250 | — (stock not simulated: always full) |
| ICC 1 / ICC 2 | 1 | −700 / −300 | 285 | 17.0 s, cover presses the PCB onto the probes |
| DW 1 / DW 2 | 2 | 300 / 700 | 250 | 15.0 s |
| WIP OUT | 3 | 1400 | 250 | — (always empty) |

Job priority, and the order IS the behaviour: fill an empty ICC from WIP IN first (both — that is
what "buffer" means, the 17 s tester must not idle); then empty a finished DW to WIP OUT, ahead of
the third rule, because two full DWs plus two finished ICCs otherwise deadlock and the deadlock
looks like a robot that simply stopped; then move a finished ICC to a free DW. Machines 300×320
with hinged covers that open 80° and close only while processing, reopening if the arm enters the
sweep — a light curtain, not a delay. PCB 120×80×8, gripped on its 120 mm side, jaws opening along
the rail.

The arm is unchanged: a rail along X (±1500 mm, 900 mm/s) and three joints turning about X in the
Y–Z plane (−90..180, −150..0, −120..120 at 90/90/120 °/s), links L1 400, L2 300, L3 250, L4 100,
home [0, 90, −90, −90]. Because the plant keeps ONE dof per component, it is a **chain of `joint`
components**, each mounted on the previous one's `end` socket — which is also how `chainPoints()`
composes it, since every angle is relative to its parent. `lib.test.js` pins the closed form
against hand-computed points, at home and at a second pose.

Carriage height 200 was measured, not chosen: all twelve poses (six surfaces plus six 140 mm
approaches — the real cell's approach) solve there with 67.3° of worst-case joint margin, and the
reachable band is −100 to 350. Only two station heights exist, so there are four arm poses and the
rail carries the rest.

New type: `joint` (revolute or prismatic, min/max/home, `trapStep` for motion, target/exec/done
like the servo). Every pose in `blurobot.st` was measured with the scene's own kinematics:

| pose | rail | j1 | j2 | j3 | tip |
|---|---|---|---|---|---|
| pick | −10 | 52.53 | −70.73 | −71.80 | [−10, 420, 670] |
| pickUp | −10 | 54.93 | −47.00 | −97.93 | +40 mm |
| place | 850 | 54.89 | −63.04 | −81.85 | [850, 420, 720] |
| placeUp | 850 | 55.35 | −59.20 | −86.15 | +40 mm |

The lift is 40 mm because the envelope closes as the wrist rises toward the shoulder: +40 keeps
22.1° of margin on every joint at both stations, +80 keeps 10.5°, and by +120 the pick side is out
of reach. Three traps were paid for on the way, and all three are now rules in CLAUDE.md: belts
that overlap in X, a nest that is not sunk (and then a sunk nest whose catch zone went down with
it), and confirming a finger grip on `closed` instead of on `open` dropping.

Open in P6: press-fit, drill and seaming cells; the rb4axis PLC program itself driving this scene
(that needs `mirror` mode, which is still P5 work).

### P7 — digital twin and FUXA
- Read-only twin, divergence markers, apply-measured-times, drift report.
- **FUXA → Sysmac simulator OPC UA directly.** The HMI talks to the PLC, as in reality, so there
  is nothing to build; document it. Add `/?view=cam1&hud=0` so the 3D view can be embedded in a
  FUXA iframe.
- Only if plant-only data (cycle stats, drift) is needed in FUXA: a Factory-I/O-style Web API
  (`GET /api/tags`, `PUT /api/tag/values`) for FUXA's WebAPI device, bound to 127.0.0.1. FUXA's
  own API had an unauthenticated tag disclosure (CVE-2026-43946).
- **Exit:** twin runs against a real or simulated PLC with zero writes, proven by the driver test.

## 14. Tests (`node tests/run.js`)

| suite | checks |
|---|---|
| `lib` | math compose/invert; `validate()` rejects missing parent, cycles, unknown type, duplicate ids and two writers; the rod-end world pose of the cylinder on the slide equals hand-computed numbers; stroke time ±1 step; cushion and each valve type; reed band edges and hysteresis; servo matches test vectors copied from rb4axis `langkahSumbu`; the cycloid reaches the pitch exactly and `inPos` only in dwell |
| `plant` | kinematic bodies line up with links; a conveyor part reaches the photo-eye on time; grip take/release; a cylinder pushes a part; the hand holds a part still (0.00 mm in 4 s over a running belt), the queue builds behind it, the run still replays, and the hand never takes a part a holder has; two runs give identical logs; a 12 ms blip becomes 100 ms plus a warning; static check that physics never calls `driver.write`. Loud SKIP if Rapier is not installed |
| `opcua` | a fake session returning 3 browse batches is fully mapped; `plain()`; the whitelist; twin rejects every write; suffix path matching; live echo only when the simulator answers, else loud SKIP |
| `sysmac` | PublishOnly on every bound tag; `ArrayTypeSpec`, never `<TypeName>ARRAY`; no `P_` POUs; LF in `<ST>`; `--check`; IO list has 4 columns, unique addresses, allowed types, `ST<n>` present, every SOL has an AS with the same stem; `--bind` against a fixture; XSD via the sysmac script or loud SKIP |
| `analysis` | cycle-time rules on synthetic logs, Gantt intervals, bottleneck, OEE formulas, compare alignment, CSV quoting |
| `web` | static checks: no CDN in the importmap; exact dependency pins; `lib/` never imports three or Rapier; `web/` builds geometry only from `lib` shapes and poses only from `worldPoses`; panel updates throttled; sliders use `onchange` |
| `browser` | headless Chrome over CDP, only with `MIO_BROWSER=1`: the editor end to end (tree, properties, undo/redo, Add, Save to disk, canonical text, 3D click), then the hand on `a-to-b` — a real mouse press on a part jams it in the plant, it holds still over the running belt, and the release lets it go. No page errors on either page |

## 15. Risks

| risk | handling |
|---|---|
| Sysmac OPC UA revises sampling to ≥ 50 ms | measured in P0; commands as levels or counters; `minPulseMs` / `offDelayMs`; PLC-side trace |
| simulator OPC UA server needs Studio ≥ 1.62 | check first (SETUP.md) |
| AT-assigned variables not writable in the simulator | tested in P0, before naming conventions are fixed |
| MC axis variables are not publishable | generated `--shim` copies them into published globals |
| PLC coil overwrites a sensor tag | overwrite detection warning |
| program not assigned to a task (silent) | `MIO_HEARTBEAT` check in the UI |
| Windows timer granularity (~15 ms) | accumulator + 50-step cap + overrun counter |
| Rapier 0.14 → 0.20 API changes | P0 smoke test |
| determinism | deterministic build; tests still allow ±1 step on edge times |
| many parts | part cap with recycling, deltas only, `dtMs` knob. Parts must NOT sleep: Rapier 0.20 kinematic links do not wake them (measured, `rapier.test.js`). If CPU matters, wake parts that touch a moving link |
| three `TransformControls` API change | exact pin + `getHelper()` note |
| two people editing at once | `baseVersion` → 409 |
| writing to a real machine by accident | twin read-only inside the driver |
| scope creep toward all of Factory I/O | SPM kit first; Factory I/O classics interleaved as smoke tests |
