// server/opcua.js without a simulator: browse with continuation points, path lookup,
// typed arrays, batched writes, write guards. The live check runs only with MIO_LIVE=1.
import { plain, parseValue, browseAll, makeLookup, createConn, globalName, SKIP_TOP } from '../server/opcua.js';

let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };
const throws = async f => { try { await f(); return false; } catch { return true; } };

// plain(): typed arrays must become real arrays before JSON sees them
chk('plain(Float64Array) -> Array', Array.isArray(plain(new Float64Array([1, 2]))));
chk('plain(Float64Array) stringifies as [..]', JSON.stringify(plain(new Float64Array([1, 2]))) === '[1,2]');
chk('plain(scalar) unchanged', plain(5) === 5 && plain(true) === true);

chk('parseValue true/FALSE', parseValue('true') === true && parseValue('FALSE') === false);
chk('parseValue number', parseValue('-12.5') === -12.5);
chk('parseValue rejects junk and empty', await throws(() => parseValue('abc')) && await throws(() => parseValue(' ')));

// browseAll(): a folder served in THREE batches must be mapped completely
const ref = (name, cls, id) => ({ browseName: { name }, nodeClass: cls, nodeId: { toString: () => id } });
const tree = {
  ObjectsFolder: [[ref('Server', 1, 'srv'), ref('GlobalVars', 1, 'gv')]],
  srv: [[ref('ServerStatus', 2, 'ss')]],
  gv: [[ref('A', 2, 'a'), ref('B', 2, 'b')], [ref('C', 2, 'c'), ref('D', 2, 'd')], [ref('E', 2, 'e'), ref('Sub', 1, 'sub')]],
  sub: [[ref('Server', 2, 'deep')]],
  a: [[ref('A[0]', 2, 'a0'), ref('A[1]', 2, 'a1')]],          // array elements under variable A
};
let nexts = 0;
const fakeBrowse = {
  browse: async id => {
    const b = tree[String(id)] || [[]];
    return { references: b[0], continuationPoint: b.length > 1 ? Buffer.from(String(id) + ':1') : null };
  },
  browseNext: async cp => {
    nexts++;
    const [id, i] = cp.toString().split(':');
    const b = tree[id];
    return { references: b[+i], continuationPoint: +i + 1 < b.length ? Buffer.from(id + ':' + (+i + 1)) : null };
  },
};
const list = [];
const { map, truncated } = await browseAll(fakeBrowse, list);
chk('browseNext followed (2 continuation calls)', nexts === 2, 'calls ' + nexts);
chk('variables are not descended into (array elements would flood the cap)', !map.has('GlobalVars.A.A[0]'));
chk('small tree is not truncated', truncated === false);
chk('all 3 batches mapped (A..E)', ['A', 'B', 'C', 'D', 'E'].every(n => map.has('GlobalVars.' + n)), [...map.keys()].join(' '));
chk('top-level Server branch skipped', !map.has('Server.ServerStatus'));
chk('a deep variable called Server is kept', map.get('GlobalVars.Sub.Server') === 'deep');
chk('--tree collector lists folders too', list.some(d => d.kind === 'Object' && d.path === 'GlobalVars.Sub'));
chk('SKIP_TOP is anchored', !SKIP_TOP.test('ServerX') && SKIP_TOP.test('Types'));

// makeLookup(): full path first, then suffix (extra controller level)
const look = makeLookup(new Map([['GlobalVars.X', '1'], ['Ctrl.GlobalVars.Y', '2'], ['GlobalVars.S.X', '3']]), 'GlobalVars.');
chk('full path wins over a struct member with the same last name', look('X') === '1');
chk('suffix match for an extra controller level', look('Y') === '2');
chk('unknown -> null', look('Z') === null);

// globalName(): the Sysmac simulator's real layout, seen live on Studio 1.66
const SIMPATH = 'DeviceSet.Configuration.Resources.new_Controller_0.GlobalVars.';
chk('globalName finds GlobalVars as an inner segment', globalName(SIMPATH + 'SIM_HEARTBEAT', 'GlobalVars.') === 'SIM_HEARTBEAT');
chk('globalName: root layout too', globalName('GlobalVars.X', 'GlobalVars.') === 'X');
chk('globalName: struct member / array element is not a global', globalName(SIMPATH + 'S.M', 'GlobalVars.') === null);
chk('globalName: segment must match whole ("MyGlobalVars.")', globalName('MyGlobalVars.X', 'GlobalVars.') === null);

// createConn(): resolve + batched write with a fake session
const writes = [];
const ok = { isGood: () => true, name: 'Good' };
const sess = {
  read: async nodes => nodes.map(n => ({
    statusCode: { isGood: () => n.nodeId !== 'bad' },
    value: { dataType: 11, arrayType: n.nodeId === 'arr' ? 1 : 0, value: n.nodeId === 'arr' ? new Float64Array([1, 2, 3]) : false },
  })),
  write: async wvs => { writes.push(wvs); return wvs.map(() => ok); },
};
const cmap = new Map([['GlobalVars.ON', 'on'], ['GlobalVars.ARR', 'arr'], ['GlobalVars.BAD', 'bad']]);
const c = createConn(sess, { map: cmap, writable: new Set(['ON', 'ARR']) });
const r = await c.resolve(['ON', 'ARR', 'BAD', 'NOPE']);
chk('resolve finds good tags', r.found.join() === 'ON,ARR', r.found.join());
chk('resolve reports bad status and unknown as missing', r.missing.sort().join() === 'BAD,NOPE', r.missing.join());
chk('resolved array value is plain', Array.isArray(c.values.ARR) && c.values.ARR.join() === '1,2,3');

await c.write([{ name: 'ARR', index: 1, value: 9 }, { name: 'ARR', index: 2, value: 8 }, { name: 'ON', value: true }]);
chk('one batch per write() call', writes.length === 1);
chk('two elements of one array merge into ONE whole-array write', writes[0].length === 2
  && JSON.stringify(writes[0][0].value.value.value) === '[1,9,8]', JSON.stringify(writes[0].map(w => w.value.value.value)));
chk('write keeps the read-once type', writes[0][0].value.value.dataType === 11 && writes[0][0].value.value.arrayType === 1);
chk('shadow updated after a good write', c.values.ARR.join() === '1,9,8' && c.values.ON === true);
chk('write outside the whitelist refused', await throws(() => c.write([{ name: 'BAD', value: 1 }])));
chk('index out of range refused', await throws(() => c.write([{ name: 'ARR', index: 3, value: 0 }])));

const twin = createConn(sess, { map: cmap, readOnly: true });
await twin.resolve(['ON']);
const before = writes.length;
chk('twin (readOnly) refuses every write', await throws(() => twin.write([{ name: 'ON', value: true }])) && writes.length === before);

const rej = createConn({ ...sess, write: async wvs => wvs.map(() => ({ isGood: () => false, name: 'BadTypeMismatch' })) }, { map: cmap });
await rej.resolve(['ON']);
chk('a rejected write throws with the tag name', await rej.write([{ name: 'ON', value: 1 }]).then(() => '', e => e.message).then(m => /ON BadTypeMismatch/.test(m)));

// live: only with a running simulator + imported probe
if (!process.env.MIO_LIVE) {
  console.log('  SKIP  live simulator check: set MIO_LIVE=1 with the simulator running and plc/MioProbe.xml imported');
} else {
  const { connect } = await import('../server/opcua.js');
  const live = await connect({});
  try {
    const [h0] = await live.read(['MIO_HEARTBEAT']);
    await new Promise(res => setTimeout(res, 300));
    const [h1] = await live.read(['MIO_HEARTBEAT']);
    chk('live: MIO_HEARTBEAT moves (program assigned to a task)', h1 > h0, h0 + ' -> ' + h1);
  } finally { await live.close(); }
}

process.exit(fail ? 1 : 0);
