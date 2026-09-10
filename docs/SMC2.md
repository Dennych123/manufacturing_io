# What Studio writes into a `.smc2`: import and task assignment

This file is the reference for automating "import XML + assign the program to a task". It is
measured, not guessed. `plc/test_mio.smc2` was diffed before and after one import of
`scenes/cyl-on-slide.sysmac.xml` plus the manual assignment of `PRG_CYL_ON_SLIDE` to PrimaryTask.
The setup was Studio 1.66 with an NJ501-1500 1.70 project, on 2026-09-10.

## Layout

A `.smc2` is a ZIP with one folder, named after the **solution id**:

| entry | what |
|---|---|
| `<solution>.manifest` | solution name, **id**, trackingId, features (`SetUnit NJ501-1500 Ver. 1.70`) |
| `<solution>.oem` | the **index**: a tree of `<Entity type=… id=… trackingId=… name=… DN=…>` for every object |
| `<entity-id>.xml` | one file per entity, in the entity's own format (see below) |
| `b9cb2acf….zip` + `.NexBuild*` / `.NexTransferInformation` | build and transfer output (`LV_*.cxil2`, obfuscated names). **Studio regenerates it on Build.** Leave it alone. |
| `StorageData/SysmacStudio.stsdb4` | Studio's own store; changes on every save |

**The solution id changes on every Save.** Before: `c1a35a90-…`. After: `03e3b355-…`. The folder
name, the manifest `id` and the root `Entity type="Solution"` all changed together, while the
solution `trackingId` stayed. So a tool must find the root folder from the manifest and never
hard-code it.

Entity ids for tasks and programs stayed stable across the save. PrimaryTask is
`cc41ec56-…`, the OPC UA simulation settings are `6d891f78-…`, and the global variables are
`2840705b-…`.

## Formats

**Variables** (globals, and each program's locals and externals) are the `SLWD` text format,
CRLF, tab-separated:

```
[SLWD version=1.0]
_EN=Variables
+GN=VAR_GLOBAL	GVT=GlobalNamespaceGroup
++D=LREAL	N=SV1_TGT	G=VAR_GLOBAL	Com=ST1 PRESS SLIDE TARGET
```

For a program:

```
+GN=VAR	GVT=DefaultGroup
++D=TON	N=T_DWELL	G=VAR
+GN=VAR_EXTERNAL	GA=External	GVT=ExternalGroup
++D=LREAL	N=SV1_TGT	G=VAR_EXTERNAL
```

The globals file lists **no publish attribute** at all, although the XML import said
`networkPublish="PublishOnly"`. See the note on OPC UA below.

Importing a global that already exists (`MIO_HEARTBEAT`, from the probe) **merges** it: it stays
listed once.

**ST body**: a `StructuredTextModel` with `<Text>…</Text>`, line breaks as `&#xD;` + LF, the whole
file on one physical line (no CRLF).

## What one import adds

Under the controller's POU folder in `.oem`, one `Entity type="Program" subtype="StructuredText"`
(name and DN = the program; its `trackingId` matters, see below), with these children:

| child entity | file |
|---|---|
| `DebugProgramSetting` | `<data><DebugProgramSetting Flag="False" /></data>` |
| `Variables` (+ a `SourceHolder` child) | SLWD locals + externals |
| `PouBody` version `0.2` (+ `SourceHolder`, `PouBodySourceHolder`) | ST body; the PouBodySourceHolder file references `Program_<trackingId-without-dashes>` (Cxil link, rebuilt by Build) |

Plus the new lines in the global variables file.

## What one task assignment adds (four places)

1. **`.oem`**: under the `Entity type="NexTask" … name="PrimaryTask"`, a child
   ```
   <Entity type="NexAssociatedProgram" subtype="StructuredText" id="<new guid>" name="PRG_X" version="0" trackingId="<new guid>" DN="PRG_X">
   ```
2. **`<that id>.xml`**:
   ```
   <AssociatedProgramModel xmlns="http://schemas.datacontract.org/2004/07/Omron.Cxap.Modules.TaskConfiguration.Models" xmlns:i="http://www.w3.org/2001/XMLSchema-instance"><PouInstanceName>PRG_X</PouInstanceName></AssociatedProgramModel>
   ```
3. **The task file** (`cc41ec56-….xml`, which also holds the period in `TaskExecutionCondition="1000"` µs):
   ```
   <AssociatedProgramData ProgramName="PRG_X" InstanceName="PRG_X" IniFileTrackingId="<Program entity trackingId, no dashes>" StartupSetting="TRUE" SequenceNumber="<next>" IsDebugProgram="false" />
   ```
4. **OPC UA simulation settings** (`6d891f78-….xml`), under `Primary Periodic Task / PrimaryTask`:
   ```
   <Node Name="PRG_X" IsPublished="false" />
   ```

## OPC UA note

These are the simulation settings of this project:

- `OnlyNetworkPublishVariablesFlag Selected="False"`
- `Global Variables` listed as a published node

This most likely means **the simulator publishes every global**, whatever its Network Publish
attribute, which fits the globals file carrying no publish attribute. On a real controller,
`PublishOnly` is still what exposes a tag. So the generator keeps emitting it. This is not tested
on hardware.

## Automation plan (not built yet)

- **Task assignment** is four small, textual edits. It is safe to automate:
  1. with Studio's project closed;
  2. after a byte-exact backup (`../smc2-backup/`);
  3. keeping the ZIP entry order and each file's line endings.
- **Import** looks feasible the same way: Program entity plus three children, plus lines in the
  globals file. It touches more entities, and Build must regenerate the Cxil output. Try it on a
  copy first, then open it in Studio and Build.
- **Run (F5), OPC UA on, Transfer** stay UI actions: those are not stored in the project.
