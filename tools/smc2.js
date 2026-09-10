#!/usr/bin/env node
// @ts-check
// Import a scene's globals and program straight into a Sysmac Studio project (.smc2), and
// assign the program to a task, without clicking through Studio. docs/SMC2.md is the reference:
// every edit here is one Studio itself was seen making.
//
//   node tools/smc2.js plc/test_mio.smc2 --scene cyl-on-slide [--task PrimaryTask] [--dry-run]
//   node tools/smc2.js plc/test_mio.smc2 --probe
//   node tools/smc2.js plc/test_mio.smc2 --list
//
// The project must be CLOSED in Studio: Studio rewrites the whole file on Save, so an edit made
// while it is open is lost. A byte-exact backup is written next to the file first. Then open the
// project, Build, and Run / OPC UA / Transfer as usual (those are not stored in the project).
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { execSync } from 'node:child_process';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { readZip, writeZip, content, makeEntry } from './zip.js';
import { sceneProject, externalVars, stText, PROBE } from './gen_sysmac.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const BOM = '﻿', NL = '\r\n';

// ------------------------------------------------------------------ project access

/**
 * Opens a .smc2. The root folder is the solution id, which Studio changes on EVERY Save, so
 * it is found through the manifest, never assumed.
 * @param {Buffer} buf
 */
export function openSmc2(buf) {
  const zip = readZip(buf);
  const man = zip.entries.map(e => /^([0-9a-f-]{36})\/\1\.manifest$/i.exec(e.name)).find(Boolean);
  if (!man) throw new Error('no <solution>/<solution>.manifest: not a Sysmac Studio project');
  const root = man[1];
  const at = (/** @type {string} */ f) => zip.entries.find(e => e.name === root + '/' + f);
  const changed = new Set();
  const p = {
    zip, root, changed,
    /** @param {string} f */
    has: f => !!at(f),
    /** @param {string} f */
    get(f) { const e = at(f); if (!e) throw new Error('missing entry ' + f); return content(e).toString('utf8'); },
    /** Replace or add a file. Unchanged content is left alone, so a second run is a no-op. @param {string} f @param {string} text @param {Date} when */
    put(f, text, when) {
      const bytes = Buffer.from(text, 'utf8'), e = at(f);
      if (e) {
        if (content(e).equals(bytes)) return false;
        zip.entries[zip.entries.indexOf(e)] = makeEntry(e, e.name, bytes, when);
      } else {
        // New entries go where Studio keeps them: sorted by name, ignoring case.
        const name = root + '/' + f, key = name.toLowerCase();
        const tpl = zip.entries.find(x => x.central.readUInt16LE(10) === 8 && /\.xml$/i.test(x.name)) || null;
        let i = zip.entries.findIndex((x, k) => k > 0 && x.name.toLowerCase() > key);
        if (i < 0) i = zip.entries.length;
        zip.entries.splice(i, 0, makeEntry(tpl, name, bytes, when));
      }
      changed.add(f);
      return true;
    },
    save: () => writeZip(zip),
  };
  return p;
}

// ------------------------------------------------------------------ the .oem index

const attrs = (/** @type {string} */ line) => Object.fromEntries([...line.matchAll(/(\w+)="([^"]*)"/g)].map(m => [m[1], m[2]]));
const indentOf = (/** @type {string} */ line) => /^ */.exec(line)[0];
const xesc = (/** @type {string} */ s) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');

/** @param {string[]} lines @param {(a: Record<string, string>) => boolean} pred @param {number} [from] @param {number} [to] */
function findEntity(lines, pred, from = 0, to = lines.length) {
  for (let i = from; i < to; i++) if (/^\s*<Entity /.test(lines[i]) && pred(attrs(lines[i]))) return i;
  return -1;
}
/** Index of the `</Entity>` closing the entity that starts at line i. @param {string[]} lines @param {number} i */
function entityEnd(lines, i) {
  const close = indentOf(lines[i]) + '</Entity>';
  for (let k = i + 1; k < lines.length; k++) if (lines[k] === close) return k;
  throw new Error('.oem: entity at line ' + (i + 1) + ' is not closed');
}
/** Where a new child of the entity at line i goes (expands `<ChildEntities />`). @param {string[]} lines @param {number} i */
function childSlot(lines, i) {
  const ind = indentOf(lines[i]) + '  ', end = entityEnd(lines, i);
  for (let k = i + 1; k < end; k++) {
    if (lines[k] === ind + '<ChildEntities />') { lines.splice(k, 1, ind + '<ChildEntities>', ind + '</ChildEntities>'); return k + 1; }
    if (lines[k] === ind + '</ChildEntities>') return k;
  }
  throw new Error('.oem: entity at line ' + (i + 1) + ' has no ChildEntities');
}
/** @param {Date} d */
const stamp = d => [d.getMonth() + 1, d.getDate()].map(n => String(n).padStart(2, '0')).join('/') + '/' + d.getFullYear() + ' '
  + [d.getHours(), d.getMinutes(), d.getSeconds()].map(n => String(n).padStart(2, '0')).join(':');
/** @param {string[]} lines @param {number} i @param {string} date */
const touch = (lines, i, date) => { lines[i] = lines[i].replace(/dateLastModified="[^"]*"/, 'dateLastModified="' + date + '"'); };

/**
 * One entity, in the exact attribute order Studio writes. `kids` are already-indented lines.
 * @param {string} ind @param {{type: string, subtype?: string, id: string, name: string, version?: string, date: string, trackingId: string, DN?: string}} a
 * @param {string[]|null} kids null = `<ChildEntities />`
 */
function entity(ind, a, kids) {
  const head = ind + '<Entity type="' + a.type + '" subtype="' + (a.subtype || '') + '" id="' + a.id + '" name="' + xesc(a.name) + '" version="' + (a.version || '0')
    + '" dateCreated="' + a.date + '" dateLastModified="' + a.date + '" trackingId="' + a.trackingId + '"' + (a.DN ? ' DN="' + xesc(a.DN) + '"' : '') + '>';
  return [head, ind + '  <AccessInfos />', ...(kids ? [ind + '  <ChildEntities>', ...kids, ind + '  </ChildEntities>'] : [ind + '  <ChildEntities />']), ind + '</Entity>'];
}

// ------------------------------------------------------------------ file formats (docs/SMC2.md)

/** SLWD fields of one `++` line. @param {string} line */
const slwdFields = line => Object.fromEntries(line.slice(2).split('\t').map(kv => { const i = kv.indexOf('='); return [kv.slice(0, i), kv.slice(i + 1)]; }));
const oneLine = (/** @type {string} */ s) => s.replace(/[\t\r\n]+/g, ' ').trim();

/** @param {import('./gen_sysmac.js').Var} v */
function supported(v) {
  if (/^ARRAY/i.test(v.type) || v.init || v.at || v.retain || v.constant) {
    throw new Error(v.name + ': arrays, initial values, AT, retain and constant are not written to .smc2 yet - import the XML in Studio for this one');
  }
}

/** Studio's program Variables file: locals, then externals. @param {import('./gen_sysmac.js').Var[]} locals @param {import('./gen_sysmac.js').Var[]} ext */
export function programVars(locals, ext) {
  locals.forEach(supported);
  return ['[SLWD version=1.0]', '_EN=Variables', '+GN=VAR\tGVT=DefaultGroup', ...locals.map(v => '++D=' + v.type + '\tN=' + v.name + '\tG=VAR'),
          '+GN=VAR_EXTERNAL\tGA=External\tGVT=ExternalGroup', ...ext.map(v => '++D=' + v.type + '\tN=' + v.name + '\tG=VAR_EXTERNAL')].join(NL) + NL;
}

/** Studio's ST body: line breaks as `&#xD;` + LF, no trailing newline, quotes left as they are. @param {string} st */
export function stBody(st) {
  const t = stText(st).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/\n/g, '&#xD;\n');
  return '<StructuredTextModel xmlns="http://schemas.datacontract.org/2004/07/Omron.Cxap.Modules.StructuredText.Core" xmlns:i="http://www.w3.org/2001/XMLSchema-instance"><Text>'
    + t + '</Text></StructuredTextModel>';
}

const DEBUG_SETTING = BOM + '<?xml version="1.0" encoding="utf-8"?>' + NL + '<data>' + NL + '  <DebugProgramSetting Flag="False" />' + NL + '</data>';
const BOOKMARKS = '<BookmarkGroup xmlns="http://schemas.datacontract.org/2004/07/Omron.Cxap.Modules.Programming.Core" xmlns:i="http://www.w3.org/2001/XMLSchema-instance"><Bookmarks/></BookmarkGroup>';
/** @param {string} tid program trackingId without dashes */
const bodySourceHolder = tid => BOM + '<?xml version="1.0" encoding="utf-8"?><data><SectionUsingMCOrMcr>false</SectionUsingMCOrMcr><CxilVariableGroup POUTrackingId="00000000-0000-0000-0000-000000000000" />'
  + '<CxilLinkInformationListTable><CxilLinkInformationList Offset="6" FileNameWithoutExtension="Program_' + tid + '" /></CxilLinkInformationListTable><CxilDivisionInformation /></data>';
/** @param {string} name */
const assocModel = name => '<AssociatedProgramModel xmlns="http://schemas.datacontract.org/2004/07/Omron.Cxap.Modules.TaskConfiguration.Models" xmlns:i="http://www.w3.org/2001/XMLSchema-instance"><PouInstanceName>'
  + name + '</PouInstanceName></AssociatedProgramModel>';

// ------------------------------------------------------------------ import

/**
 * Upserts globals, creates or updates each program, assigns it to `task`. Returns what it did.
 * @param {ReturnType<typeof openSmc2>} p
 * @param {{globals: import('./gen_sysmac.js').Var[], programs: Array<{name: string, locals?: import('./gen_sysmac.js').Var[], st: string}>}} proj
 * @param {{task?: string, now?: Date, uuid?: () => string}} [opt]
 */
export function importProject(p, proj, { task = 'PrimaryTask', now = new Date(), uuid = () => crypto.randomUUID() } = {}) {
  const date = stamp(now), log = [];
  const oemFile = p.root + '.oem';
  const oem = p.get(oemFile);
  const lines = oem.split(NL);

  // 1. globals (SLWD, CRLF). An existing name is updated in place: import MERGES, never duplicates.
  const gi = findEntity(lines, a => a.type === 'Variables' && a.subtype === 'Global');
  if (gi < 0) throw new Error('.oem: no Global Variables entity');
  const gFile = attrs(lines[gi]).id + '.xml';
  const g = p.get(gFile).split(NL);
  const grp = g.findIndex(l => /^\+GN=VAR_GLOBAL(\t|$)/.test(l));
  if (grp < 0) throw new Error(gFile + ': no VAR_GLOBAL group');
  let gEnd = grp + 1;
  while (gEnd < g.length && g[gEnd].startsWith('++')) gEnd++;
  let gAdded = 0, gUpdated = 0;
  for (const v of proj.globals) {
    supported(v);
    const com = v.comment ? oneLine(v.comment) : '';
    const k = g.findIndex(l => l.startsWith('++') && slwdFields(l).N === v.name);
    if (k >= 0 && (k < grp || k >= gEnd)) throw new Error(v.name + ' exists in another variable group of this project (constant/retain?): not touched');
    if (k < 0) {
      g.splice(gEnd++, 0, '++D=' + v.type + '\tN=' + v.name + '\tG=VAR_GLOBAL' + (com ? '\tCom=' + com : ''));
      gAdded++;
    } else {
      const f = slwdFields(g[k]);
      if (f.D === v.type && (f.Com || '') === com) continue;
      f.D = v.type;
      if (com) f.Com = com; else delete f.Com;
      g[k] = '++' + Object.entries(f).map(([a, b]) => a + '=' + b).join('\t');
      gUpdated++;
    }
  }
  if (p.put(gFile, g.join(NL), now)) { touch(lines, gi, date); log.push('globals: ' + gAdded + ' added, ' + gUpdated + ' updated'); }
  else log.push('globals: all ' + proj.globals.length + ' already there');

  // 2. programs
  const pgi = findEntity(lines, a => a.type === 'Group' && a.subtype === 'IecPrograms');
  if (pgi < 0) throw new Error('.oem: no Programs group');
  const ti = findEntity(lines, a => a.type === 'NexTask' && a.name === task);
  if (ti < 0) throw new Error('.oem: no task named ' + task + ' (Task Settings in Studio)');
  let taskChanged = false;

  for (const prog of proj.programs) {
    if (/^P_/i.test(prog.name)) throw new Error(prog.name + ': starts with P_, Studio renames it');
    const vars = programVars(prog.locals || [], externalVars(prog, proj.globals));
    const body = stBody(prog.st);
    const pgs = findEntity(lines, () => true, pgi) >= 0 ? pgi : pgi;
    let pi = findEntity(lines, a => a.type === 'Program' && a.name === prog.name, pgs, entityEnd(lines, pgi));
    let tracking;
    if (pi >= 0) {
      // Re-import: the same ids, new Variables and body. (Studio's import REPLACES a POU too.)
      const end = entityEnd(lines, pi);
      const vi = findEntity(lines, a => a.type === 'Variables', pi, end), bi = findEntity(lines, a => a.type === 'PouBody', pi, end);
      if (vi < 0 || bi < 0) throw new Error(prog.name + ': program without Variables or Program Body: not an ST program?');
      tracking = attrs(lines[pi]).trackingId;
      const a = p.put(attrs(lines[vi]).id + '.xml', vars, now), b = p.put(attrs(lines[bi]).id + '.xml', body, now);
      if (a) touch(lines, vi, date);
      if (b) touch(lines, bi, date);
      if (a || b) { touch(lines, pi, date); log.push(prog.name + ': updated (' + [a && 'variables', b && 'body'].filter(Boolean).join(' + ') + ')'); }
      else log.push(prog.name + ': already up to date');
    } else {
      const id = { prog: uuid(), dbg: uuid(), vars: uuid(), varsSrc: uuid(), body: uuid(), bodySrc: uuid(), holder: uuid() };
      tracking = uuid();
      const slot = childSlot(lines, pgi), ind = indentOf(lines[pgi]) + '    ';
      const t = () => uuid();
      const block = entity(ind, { type: 'Program', subtype: 'StructuredText', id: id.prog, name: prog.name, date, trackingId: tracking, DN: prog.name }, [
        ...entity(ind + '    ', { type: 'DebugProgramSetting', id: id.dbg, name: 'DebugProgramSetting0,00', date, trackingId: t() }, null),
        ...entity(ind + '    ', { type: 'Variables', subtype: 'StructuredText', id: id.vars, name: 'Variables', date, trackingId: t() },
          entity(ind + '        ', { type: 'SourceHolder', id: id.varsSrc, name: 'Source', date, trackingId: t() }, null)),
        ...entity(ind + '    ', { type: 'PouBody', subtype: 'StructuredText', id: id.body, name: 'Program Body', version: '0.2', date, trackingId: t() }, [
          ...entity(ind + '        ', { type: 'SourceHolder', id: id.bodySrc, name: 'Source', date, trackingId: t() }, null),
          ...entity(ind + '        ', { type: 'PouBodySourceHolder', id: id.holder, name: 'PouBodySourceHolder', date, trackingId: t() }, null),
        ]),
      ]);
      lines.splice(slot, 0, ...block);
      touch(lines, pgi, date);
      p.put(id.dbg + '.xml', DEBUG_SETTING, now);
      p.put(id.vars + '.xml', vars, now);
      p.put(id.body + '.xml', body, now);
      p.put(id.body + '.BookmarkGroup', BOOKMARKS, now);
      p.put(id.holder + '.xml', bodySourceHolder(tracking.replace(/-/g, '')), now);
      log.push(prog.name + ': created');
    }

    // 3. task assignment, in the four places Studio writes it
    const tNow = findEntity(lines, a => a.type === 'NexTask' && a.name === task);
    if (findEntity(lines, a => a.type === 'NexAssociatedProgram' && a.name === prog.name, tNow, entityEnd(lines, tNow)) >= 0) {
      log.push(prog.name + ': already assigned to ' + task);
      continue;
    }
    const aid = uuid(), slot = childSlot(lines, tNow);
    lines.splice(slot, 0, ...entity(indentOf(lines[tNow]) + '    ', { type: 'NexAssociatedProgram', subtype: 'StructuredText', id: aid, name: prog.name, date, trackingId: uuid(), DN: prog.name }, null));
    touch(lines, tNow, date);
    p.put(aid + '.xml', assocModel(prog.name), now);

    const taskFile = attrs(lines[tNow]).id + '.xml';
    const tl = p.get(taskFile).split(NL);
    const seq = Math.max(0, ...tl.map(l => +(/SequenceNumber="(\d+)"/.exec(l) || [0, 0])[1])) + 1;
    const row = '    <AssociatedProgramData ProgramName="' + prog.name + '" InstanceName="' + prog.name + '" IniFileTrackingId="' + tracking.replace(/-/g, '')
      + '" StartupSetting="TRUE" SequenceNumber="' + seq + '" IsDebugProgram="false" />';
    const close = tl.indexOf('  </Programs>'), empty = tl.indexOf('  <Programs />');
    if (close >= 0) tl.splice(close, 0, row);
    else if (empty >= 0) tl.splice(empty, 1, '  <Programs>', row, '  </Programs>');
    else throw new Error(taskFile + ': no <Programs> list');
    p.put(taskFile, tl.join(NL), now);

    const oi = findEntity(lines, a => a.type === 'OpcUaServerSimulationSettings');
    if (oi >= 0) {
      const oFile = attrs(lines[oi]).id + '.xml', ol = p.get(oFile).split(NL);
      const k = ol.findIndex(l => l.trim() === '<Node Name="' + task + '">' || l.trim() === '<Node Name="' + task + '" />');
      if (k >= 0) {
        const ind = indentOf(ol[k]);
        if (ol[k].trim().endsWith('/>')) ol.splice(k, 1, ind + '<Node Name="' + task + '">', ind + '</Node>');
        const end = ol.indexOf(ind + '</Node>', k);
        ol.splice(end, 0, ind + '  <Node Name="' + prog.name + '" IsPublished="false" />');
        if (p.put(oFile, ol.join(NL), now)) touch(lines, oi, date);
      } else log.push('note: no ' + task + ' node in the OPC UA simulation settings; Studio adds it when they are opened');
    }
    taskChanged = true;
    log.push(prog.name + ': assigned to ' + task + ' (sequence ' + seq + ')');
  }

  // 4. a configuration change gets a new comparison id, as Studio does
  if (taskChanged) {
    const ci = findEntity(lines, a => a.type === 'SourceCompareId' && a.subtype === 'Configuration');
    if (ci >= 0) {
      const cf = attrs(lines[ci]).id + '.xml';
      p.put(cf, p.get(cf).replace(/<ConfigurationSourceCompareId>[^<]*</, '<ConfigurationSourceCompareId>' + uuid() + '<'), now);
      touch(lines, ci, date);
    }
  }
  p.put(oemFile, lines.join(NL), now);
  return log;
}

/** Programs, and which task runs them. @param {ReturnType<typeof openSmc2>} p */
export function listProject(p) {
  const lines = p.get(p.root + '.oem').split(NL);
  const programs = lines.filter(l => /<Entity type="Program" /.test(l)).map(l => attrs(l).name);
  /** @type {Record<string, string[]>} */
  const tasks = {};
  for (let i = 0; i < lines.length; i++) {
    if (!/<Entity type="NexTask" /.test(lines[i])) continue;
    const end = entityEnd(lines, i);
    tasks[attrs(lines[i]).name] = lines.slice(i, end).filter(l => /<Entity type="NexAssociatedProgram" /.test(l)).map(l => attrs(l).name);
  }
  const gi = findEntity(lines, a => a.type === 'Variables' && a.subtype === 'Global');
  const globals = gi < 0 ? [] : p.get(attrs(lines[gi]).id + '.xml').split(NL).filter(l => l.startsWith('++')).map(l => slwdFields(l).N);
  const device = (/SetUnit ([^"]+)"/.exec(p.get(p.root + '.manifest')) || [])[1] || '?';
  return { solution: p.root, device, programs, tasks, globals };
}

/**
 * The new container is unpacked again and compared entry by entry BEFORE the original is
 * touched (the sysmac repo's rule, smc2_comment.js): a broken ZIP only announces itself when
 * Studio refuses to open the project, and by then the file is overwritten. Every entry this run
 * did not change must also still be byte-identical to the original.
 * @param {Buffer} orig @param {Buffer} out @param {ReturnType<typeof openSmc2>} p
 */
export function verify(orig, out, p) {
  const back = readZip(out).entries, packed = p.zip.entries;
  if (back.length !== packed.length) throw new Error('self-check: ' + back.length + ' entries read back, ' + packed.length + ' packed. Nothing written');
  back.forEach((e, i) => {
    if (e.name !== packed[i].name || !content(e).equals(content(packed[i]))) throw new Error('self-check: ' + e.name + ' differs after packing. Nothing written');
  });
  const now = new Map(back.map(e => [e.name, e]));
  for (const e of readZip(orig).entries) {
    const f = e.name.slice(p.root.length + 1), n = now.get(e.name);
    if (!n) throw new Error('self-check: ' + e.name + ' disappeared. Nothing written');
    if (!p.changed.has(f) && !(n.data.equals(e.data) && n.local.equals(e.local))) throw new Error('self-check: untouched ' + e.name + ' changed. Nothing written');
  }
}

// ------------------------------------------------------------------ CLI

function studioRunning() {
  try { return /sysmac/i.test(execSync('tasklist /FO CSV /NH', { encoding: 'utf8', stdio: ['ignore', 'pipe', 'ignore'] })); } catch { return false; }
}

function main() {
  const args = process.argv.slice(2);
  const file = args.find(a => !a.startsWith('--') && /\.smc2$/i.test(a));
  const opt = (/** @type {string} */ n) => { const i = args.indexOf(n); return i >= 0 ? args[i + 1] : undefined; };
  if (!file) {
    console.error('usage: node tools/smc2.js PROJECT.smc2 --scene NAME | --probe | --list  [--task PrimaryTask] [--dry-run]');
    process.exit(1);
  }
  const buf = fs.readFileSync(file);
  const p = openSmc2(buf);
  if (args.includes('--list')) { console.log(JSON.stringify(listProject(p), null, 2)); return; }

  let proj;
  const sceneName = opt('--scene');
  if (sceneName) {
    const scene = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', sceneName + '.json'), 'utf8'));
    const st = path.join(ROOT, 'scenes', sceneName + '.st');
    proj = sceneProject(scene, fs.existsSync(st) ? fs.readFileSync(st, 'utf8') : null);
  } else if (args.includes('--probe')) proj = PROBE;
  else { console.error('say what to import: --scene NAME or --probe'); process.exit(1); }

  const log = importProject(p, proj, { task: opt('--task') || 'PrimaryTask' });
  for (const l of log) console.log('  ' + l);
  if (!p.changed.size) { console.log('nothing to change: ' + file + ' already has all of it'); return; }
  if (args.includes('--dry-run')) { console.log('dry run: ' + p.changed.size + ' entries would change, nothing written'); return; }
  if (studioRunning()) {
    console.log('\n  WARNING: Sysmac Studio is running. This project must be CLOSED in it, or Studio overwrites these changes on its next Save.\n');
  }

  const out = p.save();
  verify(buf, out, p);
  const check = listProject(openSmc2(out));
  for (const prog of proj.programs) {
    if (!check.programs.includes(prog.name) || !Object.values(check.tasks).some(t => t.includes(prog.name))) throw new Error('self-check failed: ' + prog.name + ' missing after write, nothing written');
  }
  const d = new Date(), ts = d.getFullYear() + String(d.getMonth() + 1).padStart(2, '0') + String(d.getDate()).padStart(2, '0') + '-'
    + [d.getHours(), d.getMinutes(), d.getSeconds()].map(n => String(n).padStart(2, '0')).join('');
  const backup = file.replace(/\.smc2$/i, '') + '.' + ts + '.bak';
  fs.copyFileSync(file, backup);
  fs.writeFileSync(file + '.tmp', out);
  fs.renameSync(file + '.tmp', file);
  console.log('OK  ' + file + '  (' + p.changed.size + ' entries changed; backup ' + path.basename(backup) + ')');
  console.log('    now: open the project in Studio, Build (F8), then Run (F5) / OPC UA / Transfer');
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  try { main(); } catch (e) { console.error('FAILED: ' + (/** @type {any} */ (e).message || e)); process.exit(2); }
}
