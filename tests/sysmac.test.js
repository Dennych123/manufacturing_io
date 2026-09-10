// tools/gen_sysmac.js: the XML rules Studio enforces only at import or Build time, plus the
// official XSD through sysmac's scripts/validate_xml.ps1 when that repo is on this machine.
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { typeXml, programXml, projectXml, PROBE, parseSt, sceneProject, programName } from '../tools/gen_sysmac.js';
import { tags } from '../lib/scene.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

const arr = typeXml('ARRAY[0..3] OF LREAL', '');
chk('ARRAY -> ArrayTypeSpec with its bounds', /ArrayTypeSpec/.test(arr) && /lower="0" upper="3"/.test(arr) && /<TypeName>LREAL</.test(arr));
chk('ARRAY never written as <TypeName>ARRAY (passes XSD, fails in Studio)', !/<TypeName>ARRAY/i.test(arr));
chk('scalar stays a TypeName', typeXml('BOOL', '') === '<Type><TypeName>BOOL</TypeName></Type>');

const xml = projectXml(PROBE);
const globals = (xml.match(/<GlobalVars[\s\S]*?<\/GlobalVars>/g) || []).join('\n');
const nVars = (globals.match(/<Variable /g) || []).length;
const nPub = (globals.match(/networkPublish="PublishOnly"/g) || []).length;
chk('every global is PublishOnly (else ZERO tags over OPC UA)', nVars === PROBE.globals.length && nPub === nVars, nPub + '/' + nVars);
chk('no CR anywhere (LF inside <ST>)', !xml.includes('\r'));

const firstVar = globals.match(/<Variable [\s\S]*?<\/Variable>/)[0];
const order = ['<Documentation', '<AddData', '<Type>'].map(t => firstVar.indexOf(t));
chk('Variable children in xsd:sequence order (Documentation, AddData, Type)', order.every((p, i) => p >= 0 && (i === 0 || p > order[i - 1])), order.join(','));

const ext = (xml.match(/<ExternalVars>([\s\S]*?)<\/ExternalVars>/) || [])[1] || '';
const extNames = [...ext.matchAll(/name="(\w+)"/g)].map(m => m[1]).sort();
chk('ExternalVars = exactly the globals the ST uses', extNames.join() === PROBE.globals.map(v => v.name).sort().join(), extNames.join(' '));
chk('a program local is not an ExternalVar', !extNames.includes('PULSE_LAST'));
chk('ExternalVars carry no publish/comment (the global owns them)', !/networkPublish|Documentation/.test(ext));

let threw = '';
try { programXml({ name: 'P_TEST', st: '' }, []); } catch (e) { threw = e.message; }
chk('P_ program name refused (Studio silently renames it)', /P_/.test(threw));
chk('probe program name has no P_ prefix', PROBE.programs.every(p => !/^P_/i.test(p.name)));

const check = spawnSync(process.execPath, [path.join(ROOT, 'tools', 'gen_sysmac.js'), '--probe', '--check'], { encoding: 'utf8' });
chk('plc/MioProbe.xml is up to date (--check)', check.status === 0, (check.stdout + check.stderr).trim());

// ---------------------------------------------------------------- scenes
const pv = parseSt('VAR\r\n\tA : BOOL;\n\tT1 : TON; // timer\n\tN : INT := 5;\nEND_VAR\nA := TRUE;\n');
chk('.st VAR block -> program locals (CRLF tolerated)', JSON.stringify(pv.locals) === '[{"name":"A","type":"BOOL"},{"name":"T1","type":"TON"},{"name":"N","type":"INT","init":"5"}]'
  && pv.st === 'A := TRUE;\n', JSON.stringify(pv));
let bad = '';
try { parseSt('VAR\n\tA BOOL;\nEND_VAR\n'); } catch (e) { bad = e.message; }
chk('a VAR line it cannot read is an error, not a silently missing local', /not understood/.test(bad));
chk('scene program name: PRG_ + upper snake, never P_', programName('cyl-on-slide') === 'PRG_CYL_ON_SLIDE');

const scene = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'cyl-on-slide.json'), 'utf8'));
const st = fs.readFileSync(path.join(ROOT, 'scenes', 'cyl-on-slide.st'), 'utf8');
const proj = sceneProject(scene, st);
const sxml = projectXml(proj);
const sglob = (sxml.match(/<GlobalVars[\s\S]*?<\/GlobalVars>/g) || []).join('\n');
const declared = new Map([...sglob.matchAll(/<Variable name="(\w+)">[\s\S]*?<TypeName>(\w+)<\/TypeName>/g)].map(m => [m[1], m[2]]));
const want = tags(scene);
chk('every bound tag is a global with its schema type', [...want].every(([n, i]) => declared.get(n) === i.type), [...want].filter(([n, i]) => declared.get(n) !== i.type).map(([n]) => n).join(' '));
chk('every scene global is PublishOnly', (sglob.match(/<Variable /g) || []).length === (sglob.match(/networkPublish="PublishOnly"/g) || []).length);
chk('the scene program bumps MIO_HEARTBEAT (the "not assigned to a task" check)', declared.get('MIO_HEARTBEAT') === 'UDINT' && /MIO_HEARTBEAT := MIO_HEARTBEAT \+ 1;/.test(sxml));
const sext = (sxml.match(/<ExternalVars>([\s\S]*?)<\/ExternalVars>/) || [])[1] || '';
chk('ExternalVars include the tags the ST uses', ['SV1_EXEC', 'SV1_DONE', 'AS_ST1_PRSS_CYL_DN', 'ST1_STEP', 'MIO_HEARTBEAT'].every(n => sext.includes('"' + n + '"')));
chk('program locals are Vars, not ExternalVars', !/"T_DWELL"|"STOP_REQ"/.test(sext) && /<Variable name="T_DWELL">\s*<Type><TypeName>TON<\/TypeName>/.test(sxml));
chk('scene XML has no CR (LF inside <ST>)', !sxml.includes('\r'));
chk('comments follow ST<n> <label> <words>', /<Variable name="SOL_ST1_PRSS_CYL_DN">\s*<Documentation xsi:type="SimpleText">ST1 PRESS CYLINDER DOWN</.test(sxml));
const scheck = spawnSync(process.execPath, [path.join(ROOT, 'tools', 'gen_sysmac.js'), '--scenes', '--check'], { encoding: 'utf8' });
chk('scenes/*.sysmac.xml are up to date (--scenes --check)', scheck.status === 0, (scheck.stdout + scheck.stderr).trim().replace(/\s+/g, ' '));

// Official XSD: sysmac's validator uses the schemas Sysmac Studio installs. One copy of that
// validator; this repo does not keep a second one.
const candidates = [process.env.MIO_SYSMAC_REPO, path.join(ROOT, '..', 'sysmacgen', 'sysmac'), path.join(ROOT, '..', 'sysmac')]
  .filter(Boolean).map(d => path.join(d, 'scripts', 'validate_xml.ps1'));
const script = candidates.find(f => fs.existsSync(f));
if (!script) {
  console.log('  SKIP  XSD: sysmac scripts/validate_xml.ps1 not found (set MIO_SYSMAC_REPO). Tried: ' + candidates.join(' | '));
} else {
  for (const rel of ['plc/MioProbe.xml', 'scenes/cyl-on-slide.sysmac.xml']) {
    const v = spawnSync('powershell', ['-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', script, path.join(ROOT, ...rel.split('/'))], { encoding: 'utf8' });
    const out = ((v.stdout || '') + (v.stderr || '')).trim().replace(/\s+/g, ' ');
    if (v.error) console.log('  SKIP  XSD: powershell not available (' + v.error.message + ')');
    else if (v.status === 2) console.log('  SKIP  XSD: Sysmac Studio schemas not installed here: ' + out);
    else chk(rel + ' passes the official Sysmac XSD', v.status === 0, out);
  }
}

process.exit(fail ? 1 : 0);
