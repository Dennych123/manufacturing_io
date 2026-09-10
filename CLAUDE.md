# Working notes for Claude: `manufacturing_io`

The plan is in [docs/PLAN.md](docs/PLAN.md). This file holds the rules that are **invisible
in the code but break things silently** when violated. Most were paid for once in
[rb4axis](https://github.com/Dennych123/rb4axis).

## Design rules

- **The plant runs in Node. The browser only renders, edits and analyses.** Do not run a
  second physics world or a second sequence in the page.
- **One source for numbers and names: the scene JSON.** Tag names, dimensions and times live
  there and nowhere else. A copied number drifts, and the drift is silent.
- **`shared/` is imported by both Node and the browser.** Never import three or Rapier inside
  `shared/`. Component geometry is a primitive list: the browser builds meshes from it and Node
  builds colliders from it.
- **The browser draws kinematics through `shared/scene.js`** from joint values, never by
  recomputing chains in `web/`. If the picture and the plant use different functions, the
  picture lies while still looking right.
- **Z-up in mm everywhere.** Rapier uses metres through exactly one constant (`SK = 0.001`).
  No axis remapping anywhere.
- **Actuators are kinematic models. Only free workpieces are Rapier-dynamic.** Physics never
  gates safety or sequence, and never writes to the PLC.
- **A held part is kinematic and follows its holder.** Do not use fixed joints between dynamic
  bodies: they jitter.
- **One motion model** (`shared/motion.js`, the trapezoid ported from rb4axis `langkahSumbu`).
  A second copy will disagree one day.
- **Time scale is 1× whenever a PLC is connected.** Sysmac timers run on wall time.
- **Twin mode is read-only in `opcua.js`**, not in the UI. Never write to a real machine.

## OPC UA and Sysmac Studio (each cost a round once)

| | |
|---|---|
| start the simulator FIRST | *Simulation → Use the OPC UA Server* stays grey until Run (F5). Needs Studio ≥ 1.62 |
| security **None** + anonymous **Permit** | a certificate rejection shows up looking like a wrong password |
| `networkPublish="PublishOnly"` | a project created from scratch exposes zero tags without it |
| `OPCUACertificateManager` needs an explicit `rootFolder` | otherwise it hangs forever on "Creating default certificate". The class is in `node-opcua-certificate-manager`, not re-exported by the client |
| browse MUST follow `browseNext` | without it nodes vanish silently and look like "tag missing" |
| `server/pki/` is gitignored | it contains a private key |
| typed arrays → `Array` in `polos()` | `Float64Array` stringifies to `{"0":..}` and the page silently gets NaN |
| program must be **assigned to a task** by hand | XML cannot do it. An unassigned program does not run and Studio does not complain. The heartbeat check exists for this |
| no POU names starting with `P_` | Studio silently renames them to `PR_...` |
| never index an FB instance's array output | copy the whole array first |
| arrays in XML = `InstantlyDefinedType`/`ArrayTypeSpec` | `<TypeName>ARRAY..` passes the XSD and fails in Studio |
| `<ST>` in XML uses LF; `.smc2` entries use CRLF | XML normalises line endings; the ZIP does not |
| no `ATAN2` in generated ST | not in the W560 list of 353 instructions |

## IO timing

- **One-shot events are counters, not pulses**, in both directions. 50 ms sampling misses a 4 ms
  pulse.
- A sensor pulse shorter than 2× the measured round trip gets flagged by the recorder. Do not
  silence that flag.
- The browser sends button **edges**, and the PLC enforces the conditions. Conditions enforced in
  the browser do not apply when the same tag is written from anywhere else.

## Browser performance (from rb4axis)

- Build panels once, then update text only, throttled to about 8 per second. Rebuilding
  `innerHTML` for each SSE message stutters the 3D view.
- Sliders send on `change`, not `input`.
- Redraw labels only when their text changes.
- Smooth the drawing by **predicting from velocity**, capped at 120 ms. Never smooth the numbers
  shown in panels.

## Repo hygiene

- `.gitattributes` = `* -text`. Generated files are compared byte for byte.
- Generators get `--check`, which exits 1 when committed output is stale.
- Tests: `node tests/run.js`, no framework. **A SKIP must print a message**; a silent skip looks
  like a pass.
- If a test fails, first prove whether the TEST is wrong.
