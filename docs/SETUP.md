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

**Results:** not measured yet. Paste the summary here with the date and the Studio version.

| | |
|---|---|
| timer granularity | |
| task period | |
| granted sampling / publishing | |
| round trip p50 / p95 / max | |
| smallest pulse counted 20/20 | |
| suggested `minPulseMs` | |

## 5. Manual check: are AT-assigned variables writable?

The answer decides whether a sim project can reuse the real machine's AT-mapped IO names, or needs
unmapped copies.

1. In a project that has an IO unit configured, give one BOOL global an **AT** address and set
   Network Publish to `Publish Only`.
2. Transfer, then `node server/main.js --write THAT_TAG=true`, then `--list THAT_TAG`.
3. Record here whether the value stuck, was rejected, or was overwritten on the next scan.

**Result:** not tested yet.

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
