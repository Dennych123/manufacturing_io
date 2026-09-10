#!/usr/bin/env node
// @ts-check
// Sysmac Studio import XML. Phase 0 only needs the probe program; scene -> globals and the
// IO list come in P1/P5.
//
//   node tools/gen_sysmac.js --probe            write plc/MioProbe.xml
//   node tools/gen_sysmac.js --probe --check    exit 1 if the committed file is stale
//
// The XML shape is ported from rb4axis tools/gen_xml.js (1e2b998), which copied it from
// Omron's own Sample.xml rather than inventing it. The rules that make the difference:
//   - ARRAY is an InstantlyDefinedType/ArrayTypeSpec. As <TypeName> it passes the XSD
//     (TypeName is any xsd:string) and only Studio rejects it.
//   - <Variable> children are an xsd:sequence: Documentation, AddData, Type, InitialValue,
//     Address. That is not the column order of Studio's table.
//   - networkPublish="PublishOnly" on every global the plant touches. A project built from
//     scratch otherwise exposes ZERO tags over OPC UA.
//   - <ST> uses LF. An XML reader normalises CRLF anyway (XML 1.0 §2.11); CRLF is only
//     required on the .smc2 path, which is a different route.
//   - No POU name starting with P_. Studio renames it to PR_... without a message, and task
//     assignment then points at nothing.
// The XSD only checks SHAPE. Tests also run sysmac's scripts/validate_xml.ps1 when present.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const OUT = path.join(ROOT, 'plc', 'MioProbe.xml');
const SMC = 'https://www.ia.omron.com/Smc IEC61131_10_Ed1_0_SmcExt1_0_Spc1_0.xsd';
// ponytail: fixed device, the one rb4axis imported into successfully. Add a --device flag
// when an import into another controller (NX1P2) complains about it.
const DEVICE = { modelName: 'NX102', version: '1.40' };

/** @param {any} s */
export function esc(s) {
  return String(s == null ? '' : s)
    .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
}

/** @param {string} t IEC type, e.g. BOOL or ARRAY[0..3] OF LREAL @param {string} ind */
export function typeXml(t, ind) {
  const m = /^ARRAY\s*\[\s*(-?\d+)\s*\.\.\s*(-?\d+)\s*\]\s*OF\s+(.+)$/i.exec(String(t).trim());
  if (!m) return ind + '<Type><TypeName>' + esc(t) + '</TypeName></Type>';
  return ind + '<Type>\n'
    + ind + '  <InstantlyDefinedType xsi:type="ArrayTypeSpec">\n'
    + ind + '    <BaseType><TypeName>' + esc(m[3].trim()) + '</TypeName></BaseType>\n'
    + ind + '    <DimensionSpec dimensionNumber="1"><IndexRange lower="' + esc(m[1]) + '" upper="' + esc(m[2]) + '" /></DimensionSpec>\n'
    + ind + '  </InstantlyDefinedType>\n'
    + ind + '</Type>';
}

/**
 * @typedef {{name: string, type: string, init?: string, at?: string, retain?: boolean, constant?: boolean,
 *            publish?: 'PublishOnly'|'Input'|'Output', comment?: string}} Var
 * @param {Var} v @param {string} ind @param {string} [extra] extra attributes
 */
export function varXml(v, ind, extra = '') {
  const b = [ind + '<Variable name="' + esc(v.name) + '"' + extra + '>'];
  if (v.comment) b.push(ind + '  <Documentation xsi:type="SimpleText">' + esc(v.comment) + '</Documentation>');
  if (v.publish) {
    b.push(ind + '  <AddData><Data name="' + SMC + '" handleUnknown="discard">'
      + '<smcext:GlobalVariableAdditionalProperties networkPublish="' + v.publish + '" /></Data></AddData>');
  }
  b.push(typeXml(v.type, ind + '  '));
  if (v.init) b.push(ind + '  <InitialValue><SimpleValue value="' + esc(v.init) + '" /></InitialValue>');
  if (v.at) b.push(ind + '  <Address address="' + esc(v.at) + '" />');
  b.push(ind + '</Variable>');
  return b.join('\n');
}

/**
 * Retain and Constant are attributes of the CONTAINER, not of the variable, so each
 * combination gets its own <GlobalVars>, as in Omron's Sample.xml.
 * @param {Var[]} list
 */
export function globalsXml(list) {
  const out = [];
  for (const [constant, retain] of [[false, false], [true, false], [false, true], [true, true]]) {
    const vars = list.filter(v => !!v.constant === constant && !!v.retain === retain);
    if (!vars.length) continue;
    out.push('        <GlobalVars' + (constant ? ' constant="true"' : '') + (retain ? ' retain="true"' : '') + '>');
    vars.forEach(v => out.push(varXml(v, '          ')));
    out.push('        </GlobalVars>');
  }
  return out.join('\n');
}

const stText = (/** @type {string} */ s) => s.replace(/\r\n?/g, '\n').replace(/\s+$/, '');

/**
 * ExternalVars are PER PROGRAM, not inherited. A global the program uses but does not
 * declare passes the XSD, passes import, and shows up red in Studio. So they are derived
 * from the identifiers the ST body actually uses.
 * @param {{name: string, locals?: Var[], st: string}} p @param {Var[]} globals
 */
export function programXml(p, globals) {
  if (/^P_/i.test(p.name)) throw new Error('POU name "' + p.name + '" starts with P_: Studio silently renames it to PR_');
  const locals = p.locals || [];
  const code = stText(p.st).split('\n').map(l => l.replace(/\/\/.*$/, '')).join('\n');
  const used = new Set(code.match(/[A-Za-z_]\w*/g) || []);
  const localNames = new Set(locals.map(v => v.name));
  const ext = globals.filter(v => used.has(v.name) && !localNames.has(v.name));
  const b = ['      <Program name="' + esc(p.name) + '">', '        <ExternalVars>'];
  ext.forEach(v => b.push(varXml({ name: v.name, type: v.type }, '          ')));
  b.push('        </ExternalVars>', '        <Vars accessSpecifier="private">');
  locals.forEach(v => b.push(varXml(v, '          ')));
  b.push('        </Vars>', '        <MainBody>', '          <BodyContent xsi:type="ST">');
  b.push('            <ST>' + esc(stText(p.st)) + '</ST>');
  b.push('          </BodyContent>', '        </MainBody>', '      </Program>');
  return b.join('\n');
}

/** @param {{name: string, programs?: Array<{name: string, locals?: Var[], st: string}>, globals?: Var[]}} proj */
export function projectXml({ name, programs = [], globals = [] }) {
  const b = [
    '<?xml version="1.0" encoding="utf-8"?>',
    '<Project xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"',
    '         xmlns:smcext="https://www.ia.omron.com/Smc"',
    '         xsi:schemaLocation="https://www.ia.omron.com/Smc IEC61131_10_Ed1_0_SmcExt1_0_Spc1_0.xsd"',
    '         schemaVersion="1"',
    '         xmlns="www.iec.ch/public/TC65SC65BWG7TF10">',
    '  <FileHeader companyName="manufacturing_io" productName="tools/gen_sysmac.js" productVersion="1.0.0.0" />',
    '  <ContentHeader name="' + esc(name) + '" creationDateTime="2026-01-01T00:00:00">',
    '    <AddData><Data name="' + SMC + '" handleUnknown="discard">'
      + '<smcext:DeviceInfo modelName="' + DEVICE.modelName + '" version="' + DEVICE.version + '" /></Data></AddData>',
    '  </ContentHeader>',
    '  <Types>',
    '    <GlobalNamespace>',
    ...programs.map(p => programXml(p, globals)),
    '    </GlobalNamespace>',
    '  </Types>',
    '  <Instances>',
    '    <Configuration name="' + esc(name) + '">',
    '      <Resource name="MainResource" resourceTypeName="">',
    globalsXml(globals),
    '      </Resource>',
    '    </Configuration>',
    '  </Instances>',
    '</Project>',
  ];
  return b.join('\n') + '\n';
}

// The probe: the smallest PLC program that answers the Phase 0 questions (docs/SETUP.md).
// Counters are UDINT with `+ 1`, the form rb4axis's SIM_HEARTBEAT already built in Studio.
export const PROBE = {
  name: 'MioProbe',
  globals: /** @type {Var[]} */ ([
    { name: 'MIO_HEARTBEAT', type: 'UDINT', comment: '+1 every scan: proves PRG_MIO_PROBE is assigned to a task and running' },
    { name: 'MIO_ECHO_IN', type: 'DINT', comment: 'written by the plant, copied to MIO_ECHO_OUT every scan' },
    { name: 'MIO_ECHO_OUT', type: 'DINT', comment: 'MIO_ECHO_IN as of the last scan: the delay until the plant sees it is the IO round trip' },
    { name: 'MIO_PULSE_IN', type: 'BOOL', comment: 'pulsed by the plant for the pulse-width test' },
    { name: 'MIO_PULSE_CNT', type: 'UDINT', comment: 'rising edges of MIO_PULSE_IN the PLC saw' },
  ].map(v => ({ ...v, publish: 'PublishOnly' }))),
  programs: [{
    name: 'PRG_MIO_PROBE',
    locals: [{ name: 'PULSE_LAST', type: 'BOOL' }],
    st: [
      '// Generated by manufacturing_io tools/gen_sysmac.js --probe. Regenerate, do not edit.',
      '',
      '// Heartbeat: if this does not move, the program is not assigned to a task.',
      'MIO_HEARTBEAT := MIO_HEARTBEAT + 1;',
      '',
      '// Echo: the plant writes MIO_ECHO_IN; the time until MIO_ECHO_OUT follows is the round trip.',
      'MIO_ECHO_OUT := MIO_ECHO_IN;',
      '',
      '// Pulse counter: a pulse the PLC saw is counted, so it cannot be lost to OPC UA sampling.',
      'IF MIO_PULSE_IN AND NOT PULSE_LAST THEN',
      '\tMIO_PULSE_CNT := MIO_PULSE_CNT + 1;',
      'END_IF;',
      'PULSE_LAST := MIO_PULSE_IN;',
    ].join('\n'),
  }],
};

function main() {
  const args = process.argv.slice(2);
  if (!args.includes('--probe')) { console.error('usage: node tools/gen_sysmac.js --probe [--check]'); process.exit(1); }
  const xml = projectXml(PROBE);
  const rel = path.relative(ROOT, OUT).replace(/\\/g, '/');
  if (args.includes('--check')) {
    if (!fs.existsSync(OUT) || fs.readFileSync(OUT, 'utf8') !== xml) {
      console.error('STALE: ' + rel + '  -> run: node tools/gen_sysmac.js --probe');
      process.exit(1);
    }
    console.log('OK  ' + rel + ' is up to date');
    return;
  }
  fs.mkdirSync(path.dirname(OUT), { recursive: true });
  fs.writeFileSync(OUT, xml);
  console.log('OK  ' + PROBE.programs.length + ' program + ' + PROBE.globals.length + ' globals -> ' + rel);
  console.log('    then in Studio: import, Build, assign PRG_MIO_PROBE to the primary task (docs/SETUP.md)');
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) main();
