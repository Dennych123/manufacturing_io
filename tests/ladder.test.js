// The Omron ladder soft-PLC (server/ladder.js), its IL grammar (lib/ladder.js) and the .cxp
// codec (server/cxp.js). Semantics are pinned on small programs; the real CX-Programmer project
// a scene runs is company code and not in git, so its checks run only when it is on disk.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createLadder } from '../server/ladder.js';
import { explode, implode, readCxp, writeCxp, parseText, serializeText } from '../server/cxp.js';
import { parseLine, checkRung, network, parseOperand, addrText } from '../lib/ladder.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

/** One program, one section; each rung is mnemonic lines separated by '/'. */
const proj = (rungs, globals = [], locals = []) => ({
  name: 't', device: 'CJ2M', cpu: 'CPU33', globals,
  programs: [{ name: 'P', comment: '', taskId: 256, locals, sections: [{ name: 'S', rungs: rungs.map(r => ({ comment: '', il: r.split('/').map(s => s.trim()).filter(Boolean), notes: [] })) }] }],
});
const plcOf = (rungs, globals, locals) => { const p = createLadder(proj(rungs, globals, locals)); chk('compiles: ' + rungs[0].slice(0, 50), p.errors.length === 0, JSON.stringify(p.errors)); return p; };

// ---------------------------------------------------------------- contacts and blocks
{
  const p = plcOf(['LD 0.00 / AND 0.01 / OR 0.02 / OUT 100.00',
                   'LD 0.00 / OR 0.01 / LD 0.02 / OR 0.03 / ANDLD / OUT 100.01',
                   'LD 0.00 / AND 0.01 / LD 0.02 / AND 0.03 / ORLD / OUT 100.02',
                   'LDNOT 0.00 / ANDNOT 0.01 / OUT 100.03']);
  let ok = true;
  for (let v = 0; v < 16; v++) {
    const [a, b, c, d] = [0, 1, 2, 3].map(i => (v >> i) & 1);
    p.set('0', v);
    p.scan(v);
    ok &&= p.bit('100.00') === ((a & b) | c) && p.bit('100.01') === ((a | b) & (c | d))
        && p.bit('100.02') === ((a & b) | (c & d)) && p.bit('100.03') === (+!a & +!b);
  }
  chk('LD/AND/OR, ANDLD, ORLD and NOT contacts give the truth tables they should (16 cases)', ok);
}
{
  // CX-Programmer's branch: OUT TR0 remembers the condition, LD TR0 brings it back.
  const p = plcOf(['LD 0.00 / OUT TR0 / AND 0.01 / OUT 100.00 / LD TR0 / AND 0.02 / OUT 100.01',
                   'LD 0.00 / OUT 100.02 / AND 0.01 / OUT 100.03']);
  p.set('0.00', 1); p.set('0.02', 1); p.scan(1);
  chk('OUT TR0 / LD TR0 branch: each branch sees the shared condition', p.bit('100.00') === 0 && p.bit('100.01') === 1);
  chk('an AND after an output narrows only what follows it', p.bit('100.02') === 1 && p.bit('100.03') === 0);
}
{
  const p = plcOf(['LD 0.00 / LD 0.01 / KEEP(011) 100.00', 'LD 0.02 / SET 100.01', 'LD 0.03 / RSET 100.01']);
  p.set('0.00', 1); p.scan(1); p.set('0.00', 0); p.scan(2);
  const held = p.bit('100.00');
  p.set('0.00', 1); p.set('0.01', 1); p.scan(3);
  chk('KEEP latches on set and RESET wins when both are on', held === 1 && p.bit('100.00') === 0);
  p.set('0.02', 1); p.scan(4); p.set('0.02', 0); p.scan(5);
  const s = p.bit('100.01');
  p.set('0.03', 1); p.scan(6);
  chk('SET holds after its condition drops, RSET clears', s === 1 && p.bit('100.01') === 0);
}
{
  const p = plcOf(['@LD 0.00 / OUT 100.00', '%LD 0.00 / OUT 100.01', 'LD 0.00 / UP(521) / OUT 100.02',
                   'LD 0.00 / DOWN(522) / OUT 100.03', 'LD 0.00 / NOT(520) / OUT 100.04',
                   'LD 0.00 / @++(590) D0', 'LD 0.00 / ++(590) D1']);
  const seen = [];
  for (const [t, v] of [[1, 1], [2, 1], [3, 1], [4, 0], [5, 0]]) { p.set('0.00', v); p.scan(t); seen.push([p.bit('100.00'), p.bit('100.01'), p.bit('100.02'), p.bit('100.03'), p.bit('100.04')].join('')); }
  chk('@LD / UP are ON for exactly the scan the bit rises, %LD / DOWN the scan it falls', seen.join(' ') === '10100 00000 00000 01011 00001', seen.join(' '));
  chk('@++ runs once per rising edge, ++ every scan the condition is on', p.word('D0') === 1 && p.word('D1') === 3, p.word('D0') + ' ' + p.word('D1'));
}
{
  // @ANDNOT X is NOT(rising edge of X): a self-hold that it breaks holds until X RISES. Read as
  // the edge of NOT X it is ON only for the scan X falls and the self-hold never holds - the
  // final-caulking program's AutoRunning never latched that way.
  const p = plcOf(['LD 0.00 / OR 100.00 / @ANDNOT 0.01 / OUT 100.00', 'LD 0.00 / OR 100.01 / %ANDNOT 0.01 / OUT 100.01']);
  p.set('0.01', 0); p.set('0.00', 1); p.scan(1); p.set('0.00', 0); p.scan(2); p.scan(3);
  const held = [p.bit('100.00'), p.bit('100.01')];
  p.set('0.01', 1); p.scan(4);
  const rose = [p.bit('100.00'), p.bit('100.01')];
  p.set('0.01', 0); p.scan(5);
  chk('@ANDNOT holds a self-hold and breaks it on the rising edge; %ANDNOT on the falling edge',
      held.join('') === '11' && rose.join('') === '01' && p.bit('100.00') === 0 && p.bit('100.01') === 0,
      'held ' + held.join('') + ' rose ' + rose.join('') + ' fell ' + p.bit('100.00') + p.bit('100.01'));
}

// ---------------------------------------------------------------- interlock and END
{
  const p = plcOf(['LD 0.00 / IL(002)', 'LD 0.01 / OUT 100.00', 'LD 0.01 / SET 100.01', 'LD 0.01 / TIM 0001 #0100', 'ILC(003)', 'LD 0.01 / OUT 100.02',
                   'END(001)', 'LD P_On / OUT 100.03']);
  p.set('0.00', 1); p.set('0.01', 1);
  p.scan(0); p.scan(3000);
  const on = [p.bit('100.00'), p.bit('100.01'), p.word('T0001')];
  p.set('0.00', 0); p.scan(3002);
  chk('IL off: OUT goes OFF, SET keeps, a timer resets to its SV', on[0] === 1 && on[1] === 1 && p.bit('100.00') === 0 && p.bit('100.01') === 1
      && p.word('T0001') === 0x100 && p.bit('T0001') === 0, JSON.stringify(on) + ' pv ' + p.word('T0001').toString(16));
  chk('after ILC the rung runs normally', p.bit('100.02') === 1);
  chk('END(001) ends the program: the rung after it never runs', p.bit('100.03') === 0);
}

// ---------------------------------------------------------------- timers, counters
{
  const p = plcOf(['LD 0.00 / TIM 0010 #0015', 'LD 0.00 / TIMX(550) 0011 &15', 'LD 0.00 / TIM LT1 #0']
    , [{ name: 'LT1', addr: 'T0012', type: 'TIMER', comment: '' }]);
  p.set('0.00', 1);
  p.scan(1000); const pv0 = p.word('T0010');
  p.scan(2498); const early = [p.bit('T0010'), p.bit('T0011'), p.word('T0010')];
  p.scan(2500);
  chk('TIM #0015 is 1.5 s in BCD, its PV counts down in BCD', pv0 === 0x15 && early[0] === 0 && early[2] === 0x1 && p.bit('T0010') === 1 && p.word('T0010') === 0,
      'pv0 ' + pv0.toString(16) + ' early ' + JSON.stringify(early));
  chk('TIMX &15 is the same 1.5 s in binary', early[1] === 0 && p.bit('T0011') === 1);
  chk('TIM #0 is done the scan it starts (a symbol naming the timer works)', p.bit('T0012') === 1);
  p.set('0.00', 0); p.scan(2502);
  chk('a timer whose condition drops resets: flag OFF, PV back to SV', p.bit('T0010') === 0 && p.word('T0010') === 0x15);
}
{
  const p = plcOf(['LD 0.00 / LD 0.01 / LD 0.02 / CNTR(012) 0005 #3', 'LD 0.00 / LD 0.02 / CNT 0006 #0002']);
  const pulse = (bit, t) => { p.set(bit, 1); p.scan(t); p.set(bit, 0); p.scan(t + 1); };
  pulse('0.02', 0);
  const seq = [];
  for (let i = 0; i < 4; i++) { pulse('0.00', 10 + i * 2); seq.push(p.word('C0005') + (p.bit('C0005') ? '*' : '')); }
  chk('CNTR counts up to SV, then wraps to 0 with the completion flag', seq.join(' ') === '1 2 3 0*', seq.join(' '));
  pulse('0.01', 30);
  chk('CNTR counting down from 0 goes to SV with the flag', p.word('C0005') === 3 && p.bit('C0005') === 1);
  chk('CNT counts DOWN from SV and flags at 0', p.bit('C0006') === 1 && p.word('C0006') === 0, p.word('C0006') + '');
  p.set('0.00', 1); p.set('0.01', 1); p.scan(40);
  chk('CNTR with both inputs rising does not change', p.word('C0005') === 3);
}

// ---------------------------------------------------------------- data instructions
{
  const p = plcOf([
    'LD P_On / MOV(021) #1234 D10', 'LD P_On / BIN(023) D10 D11', 'LD P_On / MOVL(498) #00123456 D12', 'LD P_On / BINL(058) D12 D14',
    'LD P_On / MOV(021) &7 D20 / MOV(021) &8 D21 / MOV(021) &9 D22', 'LD P_On / XFER(070) &3 D20 D30', 'LD P_On / BSET(071) #00FF D40 D42',
    'LD P_On / BCNT(067) &3 D40 D43', 'LD 0.00 / @WSFT(016) &5 D50 D52', 'LD P_On / SETA(530) D60 &14 &4', 'LD P_On / MOV(021) #FFFF D62 / RSTA(531) D62 &0 &4',
    'LD P_On / +(400) D20 D21 D70', 'LD P_On / -(410) D20 D21 D71', 'LD P_On / *U(422) #FFFF &2 D72', 'LD P_On / MOV(021) &60 D80',
    'LD P_On / MOV(021) &42 D60 / XFER(070) &1 @D80 D81', 'LD P_On / ++L(591) D90', 'LD P_On / MOV(021) #8001 D92 / ASL(025) D92',
    'LD=(300) D20 &7 / AND<>L(306) D12 #0 / AND>(320) D21 D20 / OUT 100.00', 'LD>S(322) #FFFF &1 / OUT 100.01', 'LD P_On / CMP(020) D20 D21 / LD P_LT / OUT 100.02',
  ]);
  p.set('0.00', 1); p.scan(1); p.set('0.00', 0); p.scan(2); p.set('0.00', 1); p.scan(3);
  chk('MOV/BIN: #1234 BCD is 1234', p.word('D10') === 0x1234 && p.word('D11') === 1234);
  chk('MOVL/BINL: #00123456 BCD is 123456 over two words', p.long('D12') === 0x123456 && p.long('D14') === 123456, p.long('D14') + '');
  chk('XFER copies a block', [0, 1, 2].map(i => p.word('D' + (30 + i))).join() === '7,8,9');
  chk('BSET fills a range, BCNT counts its ON bits', p.word('D40') === 0xff && p.word('D42') === 0xff && p.word('D43') === 24, p.word('D43') + '');
  chk('@WSFT shifts once per edge', p.word('D50') === 5 && p.word('D51') === 5 && p.word('D52') === 0, [p.word('D50'), p.word('D51'), p.word('D52')].join());
  chk('SETA sets bits across a word boundary, RSTA clears', p.word('D61') === 0x3 && p.word('D62') === 0xfff0, p.word('D60').toString(16) + ' ' + p.word('D61').toString(16) + ' ' + p.word('D62').toString(16));
  chk('+ and - are binary, wrapping in 16 bits', p.word('D70') === 15 && p.word('D71') === 0xffff);
  chk('*U gives a 32-bit product', p.long('D72') === 0x1fffe);
  chk('@D is indirect: D80 holds 60, so @D80 reads D60', p.word('D81') === 42, p.word('D81') + '');
  chk('++L increments a double word each scan', p.long('D90') === 3);
  chk('ASL shifts left, bit 15 goes to the carry', p.word('D92') === 2);
  chk('compare contacts: = on words, <>L on double words, >', p.bit('100.00') === 1);
  chk('>S compares signed: #FFFF is -1, not greater than 1', p.bit('100.01') === 0);
  chk('CMP sets P_LT', p.bit('100.02') === 1);
}

// ---------------------------------------------------------------- flags, forcing, editing
{
  const p = plcOf(['LD P_First_Cycle / @++(590) D0', 'LD P_1s / OUT 100.00', 'LD 0.00 / OUT 100.01']);
  p.scan(0); p.scan(2); p.scan(4);
  chk('P_First_Cycle is ON for the first scan only', p.word('D0') === 1);
  const a = p.bit('100.00'); p.scan(600);
  chk('P_1s is a 1 s clock on SIM time', a === 1 && p.bit('100.00') === 0);
  p.force('100.01', 1); p.scan(602);
  chk('a forced output stays ON against the program writing it OFF', p.bit('100.01') === 1);
  p.force('0.00', 0); p.set('0.00', 1); p.scan(604);
  chk('a forced input ignores what the plant writes', p.bit('0.00') === 0 && p.forced().length === 2, JSON.stringify(p.forced()));
  p.force('100.01', null); p.force('0.00', null); p.set('0.00', 1); p.scan(606);
  chk('released, both follow program and plant again', p.bit('100.01') === 1 && p.forced().length === 0);
  const bad = p.edit({ prog: 0, sec: 0, at: 1, del: 1, rungs: [{ il: ['LD 0.00', 'FOO(999) D0'] }] });
  chk('an online edit that does not compile is refused, by name, and changes nothing', !bad.ok && /FOO/.test(bad.errors[0]) && p.project.programs[0].sections[0].rungs[1].il[0] === 'LD P_1s', bad.errors.join());
  const good = p.edit({ prog: 0, sec: 0, at: 1, del: 1, rungs: [{ il: ['LDNOT 0.00', 'OUT 100.00'], comment: 'edited' }] });
  p.set('0.00', 0); p.scan(608);
  chk('an online edit that compiles takes effect on the next scan', good.ok && p.bit('100.00') === 1 && p.project.programs[0].sections[0].rungs[1].comment === 'edited');
  const ins = p.edit({ prog: 0, sec: 0, at: 3, rungs: [{ il: ['LD P_On', 'OUT 100.05'] }] });
  p.scan(610);
  chk('a rung inserted online runs', ins.ok && p.bit('100.05') === 1 && p.project.programs[0].sections[0].rungs.length === 4);
  const unk = createLadder(proj(['LD 0.00 / PID(190) D0 D1 D2']));
  chk('an instruction the soft-PLC cannot execute is a compile error naming it, not a silent skip', unk.errors.length === 1 && /PID/.test(unk.errors[0].msg), JSON.stringify(unk.errors));
  const und = createLadder(proj(['LD NOSUCH / OUT 100.00']));
  chk('an undeclared symbol is a compile error', und.errors.length === 1 && /NOSUCH/.test(und.errors[0].msg));
}
{
  // Symbols: locals shadow globals, arrays index words, TIMER symbols name timers.
  const g = [{ name: 'A', addr: 'W10.00', type: 'BOOL', comment: '' }, { name: 'HIST', addr: 'D2300', type: 'WORD[11]', comment: '' }];
  const l = [{ name: 'A', addr: 'E0_3.10', type: 'BOOL', comment: '' }];
  const scope = { find: x => l.find(s => s.name === x) ?? g.find(s => s.name === x) };
  chk('a local symbol shadows a global of the same name', addrText(parseOperand('A', 'b', scope)) === 'E0_3.10');
  chk('an array element is base + index', addrText(parseOperand('HIST[4]', 'w', scope)) === 'D2304');
  let threw = false; try { parseOperand('HIST[11]', 'w', scope); } catch { threw = true; }
  chk('an array index past the end is refused', threw);
  chk('a bare number is a CIO address, except as a timer number', addrText(parseOperand('1541', 'w')) === '1541' && addrText(parseOperand('0090', 't')) === 'T0090');
}

// ---------------------------------------------------------------- the drawn network
{
  const net = network(checkRung(['LD 0.00', 'OR 0.01', 'AND 0.02', 'OUT TR0', 'AND 0.03', 'OUT 100.00', 'LD TR0', 'TIM 0001 #10']).lines);
  const r = net.roots[0];
  chk('a rung with a TR branch draws as one node with an output branch and a timer', !net.error && net.roots.length === 1 && r.outs.length === 2
      && r.cond?.t === 's' && r.outs[0].cond?.t === 'c', JSON.stringify(net));
  const k = network(checkRung(['LD 0.00', 'OR 100.00', 'LD 0.01', 'KEEP(011) 100.00']).lines);
  chk('KEEP draws with its reset input beside the set logic', !k.error && k.roots[0].outs[0].ins?.length === 1);
  const bad = network(['LD 0.00', 'OUT 100.00', 'OR 0.01', 'OUT 100.01'].map(parseLine));
  chk('an OR after an output is reported as undrawable (it still runs)', !!bad.error);
}

// ---------------------------------------------------------------- the .cxp codec
{
  const rnd = Buffer.alloc(20000); let x = 12345;
  for (let i = 0; i < rnd.length; i++) { x = (x * 1103515245 + 12345) >>> 0; rnd[i] = i % 3 ? (x >>> 24) : 65 + (i % 7); }
  chk('explode(implode(x)) is x on mixed data', explode(implode(rnd)).equals(rnd));
  const txt = Buffer.from('CXProgVer:="2.3";\r\n'.repeat(400), 'latin1');
  const z = implode(txt);
  chk('implode compresses repetitive text', explode(z).equals(txt) && z.length < txt.length / 5, z.length + ' bytes');
}
const SYNTH = [
  'CXProgVer:="2.3";', 'Name:="T";', 'Resource[0]:=', 'BEGIN', ' Name:="R";',
  ' PLC:=', ' BEGIN', '  Name:="M";', '  Config:="[DEV]DEV:CJ2M;CPU:CPU33;";', ' END;',
  ' GlobalVariables:=', ' BEGIN', '  VariableList:=', '  BEGIN_LIST_$#[4]', '   GO,W10.00,BOOL,,,,"start, go";', '  END_LIST_$#[4];', ' END;',
  ' Programs:=', ' BEGIN', '  Program[0]:=', '  BEGIN', '   Name:="P1";', '   Comment:="c";',
  '   LocalVariables:=', '   BEGIN', '    VariableList:=', '    BEGIN_LIST_$#[8]', '    END_LIST_$#[8];', '   END;',
  '   TaskID:=256;', '   Sections:=', '   BEGIN', '    SC:=1;', '    Sec[0]:=', '    BEGIN', '     SecName:="MAIN";', '     ProgramData:=', '     BEGIN', '      RC:=2;',
  '      R[0]:=', '      BEGIN', '       Com:=', '$?St$Bk?_#[10]', 'first\n==$?St$Bk?_#[10]', '       Flags:="1,0";', '       SL:=', '$?St$Bk?_#[11]', "'  R0\nLD GO\nOUT 100.00\n$?St$Bk?_#[11]",
  '       AtchCmts:=', '       BEGIN', '        CC:=0;', '       END;', '      END;',
  '      R[1]:=', '      BEGIN', '       Com:="";', '       Flags:="1,0";', '       SL:=', '$?St$Bk?_#[12]', 'END(001)\n$?St$Bk?_#[12]',
  '       AtchCmts:=', '       BEGIN', '        CC:=0;', '       END;', '      END;',
  '     END;', '    END;', '   END;', '  END;', '  ProgramCount:=1;', ' END;', 'END;', ''].join('\r\n');
{
  chk('the CXP text parser reproduces its input byte for byte', serializeText(parseText(SYNTH)) === SYNTH);
  const file = implode(Buffer.from(SYNTH, 'latin1'));
  const { tree, project } = readCxp(file);
  const r0 = project.programs[0].sections[0].rungs[0];
  chk('the project model: device, symbols with quoted commas, rung comment and mnemonic', project.device === 'CJ2M' && project.cpu === 'CPU33'
      && project.globals[0].comment === 'start, go' && r0.comment === 'first\n==' && r0.il.join('|') === 'LD GO|OUT 100.00' && r0.notes[0] === "'  R0");
  chk('writing back an unchanged project gives the same text', explode(writeCxp(tree, project)).toString('latin1') === SYNTH);
  project.programs[0].sections[0].rungs.splice(1, 0, { comment: 'added', il: ['LDNOT GO', 'OUT 100.01'], notes: [] });
  const again = readCxp(writeCxp(tree, project)).project.programs[0].sections[0];
  chk('an inserted rung is written back and reads back; the others keep their text', again.rungs.length === 3 && again.rungs[1].il.join('|') === 'LDNOT GO|OUT 100.01'
      && again.rungs[1].comment === 'added' && again.rungs[0].notes[0] === "'  R0" && again.rungs[2].il[0] === 'END(001)');
  const p = createLadder(project);
  p.set('W10.00', 1); p.scan(0);
  chk('the round-tripped project runs', p.errors.length === 0 && p.bit('100.00') === 1 && p.bit('100.01') === 0);
}

// ---------------------------------------------------------------- the real projects, when present
// The line's program ("11.10.2025") and the new design's ("newwww mdf"), which the scene runs.
for (const name of ['final-caulking.cxp', 'final-caulking-mdf.cxp']) {
  const REAL = path.join(ROOT, 'plc', name);
  if (!fs.existsSync(REAL)) { console.log('  SKIP  ' + name + ' checks: plc/' + name + ' is not on disk (company program, not in git)'); continue; }
  const buf = fs.readFileSync(REAL);
  const { tree, project } = readCxp(buf);
  const n = project.programs.reduce((a, p) => a + p.sections.reduce((b, s) => b + s.rungs.length, 0), 0);
  const plc = createLadder(project);
  chk(name + ': every one of ' + n + ' rungs compiles', plc.errors.length === 0, JSON.stringify(plc.errors.slice(0, 3)));
  chk(name + ': written back unchanged, the text is byte-identical', explode(writeCxp(tree, project)).equals(explode(buf)));
  let undrawable = 0;
  const G = new Map(project.globals.filter(g => g.name).map(g => [g.name, g]));
  for (const pr of project.programs) {
    const L = new Map(pr.locals.filter(g => g.name).map(g => [g.name, g]));
    for (const s of pr.sections) for (const r of s.rungs) if (r.il.length && network(checkRung(r.il, { find: x => L.get(x) ?? G.get(x) }).lines).error) undrawable++;
  }
  chk(name + ': every rung draws as a ladder network', undrawable === 0, undrawable + ' undrawable');
  // V8 optimises a chunk after about 6500 calls, and only what has RUN: 3000 idle scans measured
  // the unoptimised plateau (247-566 us on this laptop, over the limit on a slow day), which is
  // also what stalled the plant through every first cycle. warm() is what the controller does at
  // load, and it must leave no trace: the memory afterwards is a cold start's.
  const io = Object.values(JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'final-caulking.json'), 'utf8')).io.ladder.map).map(String);
  plc.warm(7000, io);
  const cold = createLadder(project);
  chk(name + ': warm() leaves the memory of a cold start', plc.M.every((/** @type {number} */ v, /** @type {number} */ i) => v === cold.M[i])
    && plc.TF.every((/** @type {number} */ v) => v === 0) && plc.CF.every((/** @type {number} */ v) => v === 0) && plc.stats.scans === 0);
  let t = 0;
  const t0 = performance.now();
  for (let i = 0; i < 2000; i++) plc.scan(t += 2);
  const us = (performance.now() - t0) / 2000 * 1000;
  chk(name + ': a warm scan is well inside a 2 ms plant step', us < 400, us.toFixed(1) + ' us/scan');
}

const scene = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'final-caulking.json'), 'utf8'));
const SCENE_CXP = path.join(ROOT, scene.io.ladder.cxp);
if (fs.existsSync(SCENE_CXP)) {
  // The machine's own program runs the scene: MASTER ON, FAULT RESET, ALL UNIT HOME, AUTO RUN, the
  // operator keeps it fed, and finished horns leave on the conveyor; then the Final Caulking machine
  // reports NG for a while and back. Every model fix in scenes/final-caulking.ctl.js and
  // tools/gen_final_caulking.js was found by this run stalling.
  const { createPlant } = await import('../server/plant.js');
  const { create } = await import('../scenes/final-caulking.ctl.js');
  /** @type {string[]} */
  const logs = [];
  const ctl = await create(scene, { root: ROOT, log: s => logs.push(s) });
  const lad = ctl.ladder;
  // The simulation patches: applied in memory, named on the ladder page, never written to the file.
  const want = (scene.io.ladder.patches || []).length;
  chk('final-caulking: every sim patch applies to the rung it was written for', logs.filter(s => /sim patch applied/.test(s)).length === want && want > 0,
    logs.filter(s => /sim patch/.test(s)).join(' | '));
  chk('final-caulking: a patched rung says so in its comment', lad.project.programs.some((/** @type {any} */ p) => p.sections.some((/** @type {any} */ s) => s.rungs.some((/** @type {any} */ r) => /\[SIM PATCH/.test(r.comment)))));
  chk('final-caulking: Save / Download write the ORIGINAL rungs, not the patches', explode(lad.bytes()).equals(explode(fs.readFileSync(SCENE_CXP))));
  const plant = await createPlant(scene, { controller: ctl });
  /** @type {string[]} */
  const warns = [];
  plant.warnListeners.push((/** @type {string} */ m) => warns.push(m));
  const press = (/** @type {string} */ id, ms = 300) => { plant.press(id, 'pb', true); plant.run(ms); plant.press(id, 'pb', false); plant.run(50); };
  // The scene starts in the program's bench mode (sim_mch on: every horn NG); the line is sim_mch OFF.
  press('pbSimOff');
  plant.run(500); press('hmiReset'); press('pbMaster'); plant.run(3500); press('hmiHome', 500); plant.run(8000); press('pbAuto'); plant.run(500);
  const lp = lad.plc;
  chk('final-caulking: AUTO RUN latches after FAULT RESET, MASTER ON and ALL UNIT HOME', lp.bit('W75.03') === 1 && lp.bit('1.01') === 1);
  press('pbAutoFeed');
  plant.run(120000);
  const alarms = () => { const a = []; for (let w = 400; w <= 414; w++) for (let b = 0; b < 16; b++) if (lp.bit('H' + w + '.' + String(b).padStart(2, '0'))) a.push('H' + w + '.' + b); return a; };
  chk('final-caulking: 120 s of AUTO puts finished horns on the conveyor', plant.io.OUT_CNT >= 10, plant.io.OUT_CNT + ' out, ' + plant.io.EM_DIAPH_CNT + ' diaphragms in');
  chk('final-caulking: no alarm is raised on the way', alarms().length === 0, alarms().join(' '));
  // The twin reads the program's own units: every one of P11..P17 that has a cycle has done several,
  // and the busiest is named.
  const ct = lad.twin.ct();
  const cycled = ct.units.filter((/** @type {any} */ u) => u.cycles >= 2).map((/** @type {any} */ u) => u.unit);
  chk('final-caulking twin: the units P11..P17 are found by their own symbol comments', ct.units.length === 7, ct.units.map((/** @type {any} */ u) => u.unit).join(' '));
  chk('final-caulking twin: each unit that works in AUTO has cycles and a busy time', ['P11', 'P12', 'P14', 'P15', 'P16', 'P17'].every(u => cycled.includes(u))
    && ct.units.every((/** @type {any} */ u) => u.cycles < 2 || (u.busyMs > 0 && u.periodMs >= u.busyMs * 0.5)), cycled.join(' '));
  chk('final-caulking twin: a bottleneck is named', !!ct.bottleneck, String(ct.bottleneck));
  const u14 = ct.units.find((/** @type {any} */ u) => u.unit === 'P14');
  chk('final-caulking twin: a unit\'s last cycle is broken into its own steps, in order', !!u14 && u14.steps.length > 5 && u14.steps.every((/** @type {any} */ s, /** @type {number} */ i, /** @type {any[]} */ a) => i === 0 || s.at >= a[i - 1].at),
    u14 ? u14.steps.length + ' steps' : '');
  const d = lad.twin.diag();
  chk('final-caulking twin: while AUTO runs, every master condition is ON', d.master.every((/** @type {any} */ x) => x.value), d.master.filter((/** @type {any} */ x) => !x.value).map((/** @type {any} */ x) => x.comment).join(' | '));
  const why = lad.twin.why('W482.00');
  chk('final-caulking twin: why() names the rung that drives a condition', why.length > 0 && /P10_MAIN/.test(why[0].where), JSON.stringify(why[0] ?? null).slice(0, 120));
  // NG: the Final Caulking machine says NG; the tray tips forward and the horns go to the NG box.
  const out0 = plant.io.OUT_CNT, ng0 = plant.io.NG_BOX_CNT;
  press('pbNg');
  plant.run(60000);
  chk('final-caulking: NG horns tip off the tray into the NG box', plant.io.NG_BOX_CNT - ng0 >= 3, (plant.io.NG_BOX_CNT - ng0) + ' NG');
  press('pbNg');
  const out1 = plant.io.OUT_CNT;
  plant.run(60000);                           // the horns already caulked NG leave first: ~20 s
  chk('final-caulking: after NG, OK horns go to the conveyor again', plant.io.OUT_CNT - out1 >= 3, (plant.io.OUT_CNT - out1) + ' OK after, ' + (out1 - out0) + ' during NG');
  chk('final-caulking: still no alarm', alarms().length === 0, alarms().join(' '));
  chk('final-caulking: no part is lost', !warns.some(w => /part lost/.test(w)), warns.filter(w => /lost/.test(w)).slice(0, 3).join(' | '));
  await plant.close();
} else console.log('  SKIP  final-caulking scene run: ' + scene.io.ladder.cxp + ' is not on disk (company program, not in git)');

console.log(fail ? fail + ' FAILED' : 'all ladder checks pass');
process.exit(fail ? 1 : 0);
