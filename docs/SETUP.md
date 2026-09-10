# Setup: Sysmac Studio simulator ⇄ manufacturing_io

These Studio steps cannot be automated. Guessing here is expensive, because each mistake
only shows up after a Build or as "nothing moves".

## 0. Requirements

| | |
|---|---|
| Sysmac Studio | **≥ 1.62** (the simulator got its OPC UA server in 1.62). This PC: **1.66.0** |
| Node | ≥ 22 |
| packages | `npm install` once in the repo root |

```bash
node tests/run.js                 # must be green before going to Studio
```

## 1. Generate and import the probe

```bash
node tools/gen_sysmac.js --probe  # -> plc/MioProbe.xml (committed; --check verifies it)
```

`PRG_MIO_PROBE` is the smallest program that answers the Phase 0 questions:

- `MIO_HEARTBEAT` goes up by 1 every scan;
- `MIO_ECHO_OUT := MIO_ECHO_IN`;
- `MIO_PULSE_CNT` counts rising edges of `MIO_PULSE_IN`.

In Studio:

1. New project (NX102, or the controller you use) or an existing sim project.
2. **Multiview Explorer → right-click Programming → Import** (or File → Import), then pick
   `plc/MioProbe.xml`. Importing global variables *adds* them. Importing a POU whose name
   already exists *replaces* it.
3. **Build (F8)** must be clean.
4. **Task Settings → PrimaryTask → Program Assignment → add `PRG_MIO_PROBE`.** XML cannot do
   this. An unassigned program **does not run, and Studio does not complain**.

## 2. Simulator + OPC UA server

The order matters: the OPC UA menu stays grey until the simulator runs.

1. **Simulation → Run (F5)**, and wait until it is really RUN.
2. **Simulation → Use the OPC UA Server for the simulator**.
3. In the OPC UA server settings, tick **Security policy `None`** and set **Anonymous = Permit**.
4. **Transfer to simulator**.

The endpoint is `opc.tcp://127.0.0.1:4840`.

## 3. Prove it runs

```bash
node server/main.js --list MIO_          # run twice: MIO_HEARTBEAT must have moved
node server/main.js --write MIO_ECHO_IN=42
node server/main.js --list MIO_ECHO      # MIO_ECHO_OUT = 42
node server/main.js --watch MIO_HEARTBEAT
```

## 4. Measure (Phase 0 exit)

```bash
node server/main.js --latency            # ~2 min; writes runs/latency-<date>.json
```

It reports:

- Node's timer granularity on this PC;
- the task period, from the heartbeat;
- the echo round trip at requested sampling of 50, 20 and 10 ms, with what the server actually
  granted;
- which pulse widths the PLC counted, 20 of each;
- a suggested `minPulseMs`.

**Results so far.** Measured 2026-09-10 on Studio 1.66.0 with the NX simulator. The probe was
not imported yet, so these were taken **read-only** from rb4axis's running `SIM_HEARTBEAT`,
3 s per row:

| asked sampling | granted sampling / publishing | samples arriving | publishes |
|---|---|---|---|
| 50 ms | 50 / 50 | 16/s (one every ~62 ms) | ~16/s |
| 20 ms | 20 / 50 | 32/s (one every ~31 ms) | ~19/s |
| 10 ms | 10 / 50 | 63/s (one every ~16 ms) | ~19/s |
| 5 ms | 5 / 50 | 64/s (one every ~16 ms) | ~19/s |

- **The effective sampling floor is ≈ 16 ms**, the Windows timer tick, whatever is granted on
  paper. Asking for less than 10 ms buys nothing.
- **Publishing is floored at 50 ms.** Anything lower is revised to 50, so PLC outputs reach the
  plant in batches every ~50 ms, with about 3 queued samples per batch.
- **PLC → plant:** an output pulse shorter than ~16 ms can fall between samples, so commands
  must be levels or counters. The reaction delay is up to ~50 ms plus one sample.
- **Queued samples carry the PLC's source timestamp.** The recorder stamps `out` edges with it,
  which gives ~16 ms resolution in the time chart instead of 50 ms.

**Measured with the probe.** Taken 2026-09-10 on Studio 1.66.0 with the NX simulator:
`--latency`, 200 echoes per row with random spacing, raw data in
`runs/latency-2026-09-10T15-04-36-295Z.json`.

| | |
|---|---|
| Node timer (asked 1 ms) | p50 2 / p95 2.4 / max 5 ms |
| task period (heartbeat) | ~1.1 ms (876 scans/s) |
| echo round trip, sampling 10 ms | p50 **38.7** / p95 **63.9** / max 79.6 ms, 0 of 200 lost |
| echo round trip, sampling 20 ms | p50 49.2 / p95 75.9 / max 83.6 ms |
| echo round trip, sampling 50 ms | p50 61.9 / p95 106.2 / max 135.7 ms |
| legs at 10 ms sampling, write / read (p50) | 11 / 27 ms |
| write-call latency | p50 ~2.5 ms |
| pulses counted | every width tested, 10 to 200 ms: 20/20 |
| `minPulseMs` | **20** (scenes and the plant default) |

What it means:

- **Plant → PLC is fast.** A write lands about 11 ms after it is sent, and the PLC reads it on
  its next ~1 ms scan. Even a 10 ms sensor pulse, written as two separate writes, was counted
  every time.
- **PLC → plant is the slow leg** (~27 ms p50), set by the 50 ms publishing floor. An output
  pulse shorter than ~16 ms can still fall between samples, so commands stay levels or counters.
- **Asking for 10 ms sampling is worth it.** It is what the driver asks for, and it cuts the round
  trip from 62 to 39 ms at p50 and from 106 to 64 ms at p95.
- **The risk for a short sensor pulse is the plant's own batching**, not the PLC: one batch in
  flight, one exchange per tick. `minPulseMs = 20` covers one exchange tick plus the write, with
  margin. The old placeholder, 100 ms, would have stretched real sensor pulses for nothing.
- **A PLC reaction** (sensor edge in the plant → PLC output back in the plant) takes ~40 ms
  typically and ~65 ms at worst. That delay is inside every time the PLC measures. The analyzer's
  ±1 sampling interval tolerance (P4) rests on it.

## 5. Manual check: are AT-assigned variables writable?

The answer decides whether a sim project can reuse the real machine's AT-mapped IO names, or needs
unmapped copies.

1. In a project that has an IO unit configured, give one BOOL global an **AT** address and set
   Network Publish to `Publish Only`.
2. Transfer, then `node server/main.js --write THAT_TAG=true`, then `--list THAT_TAG`.
3. Record here whether the value stuck, was rejected, or was overwritten on the next scan.

**Result:** not tested yet.

## 6. Run a scene against the simulator (Phase 1)

```bash
node server/main.js --internal                    # first WITHOUT a PLC: the scene's .ctl.js runs the sequence
node tools/gen_sysmac.js --scene cyl-on-slide     # -> scenes/cyl-on-slide.sysmac.xml (committed; --check verifies it)
```

The XML holds one `PublishOnly` global per tag the scene binds, typed from the component
schema, plus `PRG_CYL_ON_SLIDE` built from `scenes/cyl-on-slide.st`. That file starts with a
`VAR ... END_VAR` block of program locals; the generator adds the `MIO_HEARTBEAT` line.

1. Import `scenes/cyl-on-slide.sysmac.xml`, Build (F8).
2. **Assign `PRG_CYL_ON_SLIDE` to the primary task.**
3. Run (F5), OPC UA server on, Transfer (§2).
4. `node server/main.js` (the scene's `io.driver` is `opcua`), then open http://127.0.0.1:7660/.
   The header must say `opcua: 16 tags`. A `MISSING` count names the tags Studio does not publish.
5. Click the green START button in 3D. Watch `ST1_STEP`, `AS_ST1_PRSS_CYL_UP/DN` in Studio.

The probe and a scene program both declare `MIO_HEARTBEAT`. Import a scene into a project
without the probe, or delete the probe's globals first. Which way Studio resolves the duplicate
is not tested yet.

**Phase 1 exit, with the simulator** (not run yet):

| check | result |
|---|---|
| 3D START starts the PLC sequence | |
| the PLC sees the reed switches at the configured positions (Watch) | |
| moving a switch's `pos` changes when the PLC sees it | |
| 0 overruns in 10 minutes (status line) | |
| the NDJSON in `runs/` holds every edge | |

The internal-controller versions of these checks are in `tests/plant.test.js`.

## When it goes wrong

| symptom | almost always |
|---|---|
| tags exist, nothing moves, `MIO_HEARTBEAT` stays constant | the program is not assigned to a task |
| ZERO tags found although Studio shows a client connected | run `--tree MIO_`: it prints the REAL OPC UA tree. If the names are missing, Transfer was not done or Network Publish is not `Publish Only`. If they exist under another path, pass `--prefix` |
| OPC UA menu is grey | the simulator is not running (F5 first) |
| client rejected with what looks like a wrong password | security policy `None` is not ticked |
| hangs at "Creating default certificate" | the certificate manager has no explicit `rootFolder` (fixed in `server/opcua.js`; do not remove it) |
| `(Import failed)` with no line number | run the XSD check (`node tests/run.js` does it when sysmac's validator is present); it names the element and line, Studio does not |
| program name in the error list is not the one you imported | Studio renamed a POU that started with `P_`, without saying so |
| `Cannot use an element of array ... function block instance variables` | an FB instance's array output was indexed; copy the whole array first |
| a written sensor tag snaps back on the next scan | the PLC program has a coil or assignment on that tag; a sensor tag must only be read by the PLC |
