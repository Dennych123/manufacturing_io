#!/usr/bin/env node
// manufacturing_io server entry. Phase 0 is CLI only; the HTTP/SSE plant server arrives in P1.
//
//   node server/main.js --tree [filter]             the OPC UA tree as it really is
//   node server/main.js --list [filter]             GlobalVars whose name contains filter, with values
//   node server/main.js --write NAME=v "ARR[i]=v"   one batched write
//   node server/main.js --watch NAME ...            print changes until Ctrl+C
//   node server/main.js --latency [--samples 200]   IO timing against plc/MioProbe.xml
//
//   options: --endpoint opc.tcp://127.0.0.1:4840  --prefix GlobalVars.  --user U --pass P
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { connect, parseValue, diagnoseEmpty, globalName } from './opcua.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const MODES = ['--tree', '--list', '--write', '--watch', '--latency'];
const WITH_VALUE = ['--endpoint', '--prefix', '--user', '--pass', '--samples'];

const argv = process.argv.slice(2);
const opt = (name, dflt) => { const i = argv.indexOf(name); return i >= 0 && argv[i + 1] ? argv[i + 1] : dflt; };
let mode = null;
const rest = [];
for (let i = 0; i < argv.length; i++) {
  const a = argv[i];
  if (WITH_VALUE.includes(a)) { i++; continue; }
  if (MODES.includes(a)) mode = a;
  else if (a.startsWith('--')) die('unknown option ' + a);
  else rest.push(a);
}
const ENDPOINT = opt('--endpoint', 'opc.tcp://127.0.0.1:4840');
const PREFIX = opt('--prefix', 'GlobalVars.');

function die(msg, code = 1) { console.error(msg); process.exit(code); }

async function open(list) {
  try {
    const c = await connect({ endpoint: ENDPOINT, prefix: PREFIX, user: opt('--user', null), pass: opt('--pass', null), list });
    if (c.truncated) console.error('WARNING: browse stopped at the node cap; tags past it are invisible (' + c.map.size + ' variables mapped).');
    return c;
  } catch (e) {
    console.error('FAILED to connect to ' + ENDPOINT + ': ' + String(e.message || e).split('\n')[0]);
    console.error('Run the simulation FIRST (F5), then Simulation -> Use the OPC UA Server for the simulator,');
    console.error('with security policy None and anonymous Permit. See docs/SETUP.md.');
    process.exit(2);
  }
}

// --tree stands ALONE: it prints the tree without going through any tag list. When tags
// are not found, the answer must not depend on our own guess about the path; a diagnostic
// that reuses the suspect map only repeats the same mistake.
async function tree(c, list) {
  const f = (rest[0] || '').toLowerCase();
  const hits = f ? list.filter(d => d.path.toLowerCase().includes(f)) : list;
  for (const d of hits.slice(0, 400)) console.log('  ' + d.kind.padEnd(9) + d.path);
  if (hits.length > 400) console.log('  ... ' + (hits.length - 400) + ' more');
  console.log(hits.length + ' nodes' + (f ? ' matching "' + rest[0] + '"' : '') + ' of ' + list.length + ' visible.');
  const v = f && hits.find(d => d.kind === 'Variable');
  if (v) {
    const real = v.path.slice(0, v.path.lastIndexOf('.') + 1);
    console.log('\nreal path prefix : "' + real + '"\n--prefix in use  : "' + PREFIX + '"');
    console.log(real === PREFIX || globalName(v.path, PREFIX)
      ? 'found (full path or "' + PREFIX + '" segment) - if tags still fail, the cause is not the path.'
      : 'NOT FOUND under "' + PREFIX + '" - pass --prefix "' + real + '".');
  }
}

async function list(c) {
  const f = rest[0] || '';
  const names = [...new Set([...c.map.keys()].map(p => globalName(p, PREFIX)))]
    .filter(n => n && n.includes(f))
    .sort();
  if (!names.length) { for (const l of diagnoseEmpty(c.map, PREFIX)) console.log('  ' + l); return; }
  await c.resolve(names);
  for (const n of names.slice(0, 400)) console.log('  ' + n.padEnd(24) + ' = ' + JSON.stringify(c.values[n]));
  if (names.length > 400) console.log('  ... ' + (names.length - 400) + ' more');
  console.log(names.length + ' tags' + (f ? ' matching "' + f + '"' : ''));
}

async function write(c) {
  const changes = rest.map(pair => {
    const k = pair.indexOf('=');
    if (k < 0) die('not NAME=value: ' + pair);
    const left = pair.slice(0, k).trim();
    const m = /^([A-Za-z_]\w*)\s*\[\s*(\d+)\s*\]$/.exec(left);
    return { name: m ? m[1] : left, index: m ? +m[2] : undefined, value: parseValue(pair.slice(k + 1).trim()) };
  });
  if (!changes.length) die('nothing to write: --write NAME=value');
  const { missing } = await c.resolve([...new Set(changes.map(x => x.name))]);
  if (missing.length) die('tag not found: ' + missing.join(' ') + '  (try --tree ' + missing[0] + ')', 2);
  await c.write(changes);
  console.log('  written: ' + rest.join(' '));
}

async function watch(c) {
  if (!rest.length) die('name the tags: --watch NAME ...');
  const { missing } = await c.resolve(rest);
  if (missing.length) die('tag not found: ' + missing.join(' '), 2);
  const stamp = () => new Date().toISOString().slice(11, 23);
  for (const n of rest) console.log(stamp() + '  ' + n.padEnd(24) + ' = ' + JSON.stringify(c.values[n]));
  const g = await c.subscribe(rest, (n, v) => console.log(stamp() + '  ' + n.padEnd(24) + ' = ' + JSON.stringify(v)));
  console.log('watching ' + rest.length + ' tags (sampling granted ' + g.samplingMs.join('/') + ' ms). Ctrl+C to stop.');
  await new Promise(() => {});                              // keep the session open
}

// ------------------------------------------------------------------------ latency
// Measures the numbers every IO-timing decision in docs/PLAN.md §5 rests on, against the
// probe program (node tools/gen_sysmac.js --probe):
//   heartbeat -> task period, and proof that the program is assigned to a task
//   echo      -> write MIO_ECHO_IN = k, time until MIO_ECHO_OUT = k arrives, per sampling
//   pulses    -> which pulse widths written over OPC UA the PLC counts, 20 of each
//   tick      -> how late Node's timers fire on this PC (Windows: ~15 ms granularity)
const PROBE = ['MIO_HEARTBEAT', 'MIO_ECHO_IN', 'MIO_ECHO_OUT', 'MIO_PULSE_IN', 'MIO_PULSE_CNT'];
const sleep = ms => new Promise(r => setTimeout(r, ms));
const now = () => performance.now();
const r1 = x => Math.round(x * 10) / 10;
function pct(a, p) {
  if (!a.length) return NaN;
  const s = [...a].sort((x, y) => x - y);
  return s[Math.min(s.length - 1, Math.floor(p / 100 * s.length))];
}
const stats = a => ({ n: a.length, p50: r1(pct(a, 50)), p95: r1(pct(a, 95)), max: r1(Math.max(...a)) });

async function tickJitter() {
  const d = [];
  let t = now();
  for (let i = 0; i < 200; i++) { await sleep(1); const n = now(); d.push(n - t); t = n; }
  return { requestedMs: 1, ...stats(d) };
}

async function latency(c) {
  const N = +opt('--samples', 200);
  const out = { endpoint: ENDPOINT, date: new Date().toISOString(), tick: await tickJitter() };
  console.log('node timer (asked 1 ms)  p50 ' + out.tick.p50 + '  p95 ' + out.tick.p95 + '  max ' + out.tick.max + ' ms');

  const { missing } = await c.resolve(PROBE);
  if (missing.length) {
    die('probe tags missing: ' + missing.join(' ') + '\nimport plc/MioProbe.xml, Build, assign PRG_MIO_PROBE to the'
      + ' primary task, Transfer to simulator (docs/SETUP.md).', 2);
  }

  const [h0] = await c.read(['MIO_HEARTBEAT']);
  const t0 = now();
  await sleep(1000);
  const [h1] = await c.read(['MIO_HEARTBEAT']);
  const scans = h1 - h0;
  if (!(scans > 0)) {
    die('MIO_HEARTBEAT did not move in 1 s: PRG_MIO_PROBE exists but is not running. Assign it to the primary'
      + ' task (Task Settings -> Program Assignment), then Transfer. Studio does not complain about this.', 2);
  }
  out.taskPeriodMs = r1((now() - t0) / scans);
  console.log('task period             ~' + out.taskPeriodMs + ' ms  (' + scans + ' scans in 1 s)');

  out.echo = [];
  for (const samplingMs of [50, 20, 10]) {
    const waiters = new Set();
    const g = await c.subscribe(['MIO_ECHO_OUT'], (n, v, dv) => { for (const w of waiters) w(v, dv); }, { samplingMs });
    let k = Number(c.values.MIO_ECHO_OUT) || 0;
    const rtt = [], call = [], wLeg = [], rLeg = [];
    let lost = 0;
    for (let i = 0; i < N; i++) {
      const target = k = (k + 1) % 1e9;
      const arrived = new Promise(res => {
        const w = (v, dv) => { if (v === target) { waiters.delete(w); res({ dv, at: now(), wall: Date.now() }); } };
        waiters.add(w);
        setTimeout(() => { waiters.delete(w); res(null); }, 2000);
      });
      const wall = Date.now(), ts = now();
      await c.write([{ name: 'MIO_ECHO_IN', value: target }]);
      call.push(now() - ts);
      const a = await arrived;
      if (!a) { lost++; continue; }
      rtt.push(a.at - ts);
      // Split into legs using the PLC's source timestamp. Only meaningful if the simulator
      // clock is the PC clock, so a leg outside [0, rtt] is dropped rather than reported.
      const src = a.dv.sourceTimestamp && a.dv.sourceTimestamp.getTime();
      if (src && src - wall >= 0 && a.wall - src >= 0) { wLeg.push(src - wall); rLeg.push(a.wall - src); }
      await sleep(20 + Math.random() * 100);          // random spacing: no phase lock with the scan
    }
    await g.sub.terminate();
    const row = { requestedSamplingMs: samplingMs, grantedSamplingMs: g.samplingMs[0], grantedPublishingMs: g.publishingMs,
                  roundTrip: stats(rtt), writeCall: stats(call), lost,
                  writeLeg: wLeg.length ? stats(wLeg) : null, readLeg: rLeg.length ? stats(rLeg) : null };
    out.echo.push(row);
    console.log(`echo  sampling ${samplingMs} -> granted ${row.grantedSamplingMs}/${row.grantedPublishingMs} ms`
      + `  round trip p50 ${row.roundTrip.p50}  p95 ${row.roundTrip.p95}  max ${row.roundTrip.max} ms`
      + `  write call p50 ${row.writeCall.p50}  lost ${lost}/${N}`
      + (row.writeLeg ? `  legs w/r p50 ${row.writeLeg.p50}/${row.readLeg.p50}` : ''));
  }

  out.pulses = [];
  for (const widthMs of [10, 20, 50, 100, 150, 200]) {
    let seen = 0;
    const held = [];
    for (let i = 0; i < 20; i++) {
      const [c0] = await c.read(['MIO_PULSE_CNT']);
      await c.write([{ name: 'MIO_PULSE_IN', value: true }]);
      const ts = now();
      await sleep(widthMs);
      held.push(now() - ts);
      await c.write([{ name: 'MIO_PULSE_IN', value: false }]);
      await sleep(60);
      const [c1] = await c.read(['MIO_PULSE_CNT']);
      if (c1 - c0 === 1) seen++;
    }
    out.pulses.push({ widthMs, heldP50Ms: r1(pct(held, 50)), detected: seen, of: 20 });
    console.log(`pulse ${String(widthMs).padStart(3)} ms (held ~${r1(pct(held, 50))})  PLC counted ${seen}/20`);
  }
  const full = out.pulses.find(p => p.detected === p.of);
  // Margin: the plant's own write batching adds up to one exchange tick on top.
  out.suggestedMinPulseMs = full ? Math.ceil(full.widthMs * 2 / 10) * 10 : null;
  console.log('suggested minPulseMs    ' + (out.suggestedMinPulseMs ?? 'none - no width was counted 20/20'));

  fs.mkdirSync(path.join(ROOT, 'runs'), { recursive: true });
  const file = path.join(ROOT, 'runs', 'latency-' + out.date.replace(/[:.]/g, '-') + '.json');
  fs.writeFileSync(file, JSON.stringify(out, null, 2) + '\n');
  console.log('\nsaved ' + path.relative(ROOT, file) + '  - copy the numbers into docs/SETUP.md');
}

// ---------------------------------------------------------------------------- run
if (!mode) {
  die('usage: node server/main.js --tree [filter] | --list [filter] | --write NAME=v ... | --watch NAME ... | --latency\n'
    + '       [--endpoint ' + ENDPOINT + '] [--prefix ' + PREFIX + '] [--user U --pass P] [--samples 200]');
}
const collected = mode === '--tree' ? [] : undefined;
const c = await open(collected);
try {
  if (mode === '--tree') await tree(c, collected);
  else if (mode === '--list') await list(c);
  else if (mode === '--write') await write(c);
  else if (mode === '--watch') await watch(c);
  else await latency(c);
} catch (e) {
  console.error('FAILED: ' + (e.message || e));
  process.exitCode = 2;
} finally {
  await c.close();
}
process.exit();
