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
| the program must be **assigned to a task** by hand | XML cannot do it. An unassigned program does not run and Studio does not complain. The heartbeat check exists for this |
| a tag with a coil in the PLC cannot be forced from OPC UA | the PLC overwrites it every scan. The plant's overwrite detection warns about it |
| no POU names starting with `P_` | Studio silently renames them to `PR_...` |
| never index an FB instance's array output | copy the whole array first |
| arrays in XML = `InstantlyDefinedType`/`ArrayTypeSpec` | `<TypeName>ARRAY..` passes the XSD and fails in Studio |
| importing GlobalVars adds; importing an existing POU name replaces | |
| `<ST>` in XML uses LF; `.smc2` entries use CRLF | XML normalises line endings; the ZIP does not |
| no `ATAN2` in generated ST | not in the W560 list of 353 instructions |

## IO timing

- **PLC → plant commands are levels or counters, never one-scan pulses.** Servo Execute is held
  until Done. 50 ms sampling misses a 4 ms pulse.
- **Plant → PLC blips are held** (`offDelayMs` per sensor, `minPulseMs` per scene), and every
  stretch is recorded as a `warn`. Do not silence it.
- **Replies to PLC commands are `hold: false`** in the io schema (servo `done`/`busy`/`inPos`).
  Their short pulses are caused by the PLC itself. Holding them reports "in position" while
  the axis already moves, and warns on every move.
- One-shot events (counts, drops, rejects) are published as **counters**.
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
