# manufacturing_io

A browser-based 3D plant simulator for **special purpose machines (SPM)**, in the spirit of
Factory I/O, built on **three.js** (rendering) and **Rapier** (physics). The controller is a real
PLC program: first target is the **Omron Sysmac Studio NX/NJ simulator over OPC UA**.

Status: **Phase 1**: the first SPM primitives run end to end. The `cyl-on-slide` scene has a
servo slide carrying a double-acting cylinder with reed switches, plus a start button and lamps.
It runs in a headless Node plant (Rapier, fixed 2 ms step, recording). The browser viewer
renders it and drives it through an IO/force panel. The scene runs either with its internal
controller, or against the Sysmac simulator through generated XML. The IO timing is measured:
~39 ms round trip at p50, and the PLC counts 10 ms pulses (docs/SETUP.md §4). Still open from
Phase 0: whether AT-assigned variables are writable. The full plan is in
[docs/PLAN.md](docs/PLAN.md); the Studio steps are in [docs/SETUP.md](docs/SETUP.md).

```bash
npm install
node tests/run.js
node server/main.js --internal          # plant + viewer at http://127.0.0.1:7660/, no PLC needed
node tools/gen_sysmac.js --scene cyl-on-slide   # scenes/cyl-on-slide.sysmac.xml, import into Studio
node server/main.js                     # the same scene against the simulator (opc.tcp://127.0.0.1:4840)
node tools/gen_sysmac.js --probe        # plc/MioProbe.xml, for:
node server/main.js --latency           # IO timing, with the simulator running
```

## What it is for

- **Build a machine from parametric parts.** An air cylinder takes bore, stroke, acting type and
  speed. Reed switches sit on the tube at adjustable positions. The cylinder can be mounted on a
  servo slide, a gripper on the cylinder's rod end, and everything moves with whatever it is
  attached to. Index tables, conveyors, stoppers, sensors, buttons, lamps and fixtures are built
  the same way.
- **Test the PLC program before the machine exists.** Sysmac simulator tags drive the actuators.
  The plant computes the sensors and writes them back.
- **Measure it.** Cycle time, a per-station Gantt chart, and a logic-analyzer-style time chart of
  every IO edge. None of these exist in Factory I/O.
- **Digital twin.** Point the same scene at a real machine's PLC in read-only mode, then compare
  real and simulated timing and watch actuator times drift.
- **Later:** FUXA HMI, Modbus TCP, and Factory I/O's classic training scenes.

## Honest assessment

**Worth building.** The gap is real:

- Factory I/O is Windows-only.
- It cannot import or resize parts.
- It has no servo or PLCopen motion and no Omron driver.
- OPC UA tags are mapped one by one by hand, with a 256-node browse limit.
- It has no cycle-time, Gantt or signal-trace tooling.

For Sysmac users building SPMs, a browser tool with parametric pneumatic and servo parts, native
Sysmac tag export and timing analysis does not exist yet. Most of the plumbing is already proven
in [rb4axis](https://github.com/Dennych123/rb4axis): the OPC UA bridge, the Sysmac XML generator,
the three.js smoothing and the test harness.

**The risks, and how the plan handles them:**

1. **Scope.** "Everything Factory I/O has" means about 60 part types, 21 scenes and a full
   editor. The plan builds an SPM kit of about 20 parametric parts first. The Factory I/O
   catalogue comes afterwards, since conveyors and sensors are the same primitives.
2. **IO latency.** OPC UA sampling (tens of ms) is slower than the PLC scan (1–4 ms). A sensor
   pulse shorter than one round trip can be missed, and PLC-measured times include that latency.
   Phase 0 **measures** it before anything is built on top. The analyzer flags pulses too short
   for the PLC to see.
3. **Physics honesty.** Actuators are kinematic, driven by their own models. Only free
   workpieces are simulated by Rapier. Physics never decides safety or sequence. That boundary
   was learned in rb4axis.
4. **Manual Sysmac Studio steps.** Task assignment and enabling the OPC UA server cannot be
   automated. The plant checks a PLC heartbeat tag and says so when nothing is running, instead
   of looking broken.

**Closest existing projects:** realvirtual WEB (three.js, AGPL) and Open Industry Project (Godot,
MIT). Neither targets SPM parametric pneumatics or Sysmac.

## Architecture (one line)

```
Sysmac NX simulator (OPC UA :4840) <-> Node plant (Rapier + component models + recorder) <-> browser (three.js editor/viewer/analyzer)
```
