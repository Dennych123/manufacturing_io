// @ts-check
// OPC UA session to the Sysmac NX simulator. THIS is the single copy of connect, browse,
// read, subscribe and write in manufacturing_io: the plant, the CLI and the tests all go
// through it. Ported from rb4axis bridge/bridge.js (1e2b998).
//
// The ceinsert and sysmac copies of the browse loop do NOT follow browseNext. Do not copy
// from them.
import path from 'node:path';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

const require = createRequire(import.meta.url);
const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');

// OPC UA spec constants, so the pure parts below run without loading node-opcua.
const ATTR_VALUE = 13;          // AttributeIds.Value
const TS_BOTH = 2;              // TimestampsToReturn.Both

// node-opcua is CommonJS and heavy (~1 s to load). It is loaded only when a real
// connection is made.
let ua = null;
function loadUa() {
  if (ua) return ua;
  try {
    ua = { ...require('node-opcua-client'),
           // The class lives in its own package; node-opcua-client does NOT re-export it.
           OPCUACertificateManager: require('node-opcua-certificate-manager').OPCUACertificateManager };
  } catch (e) {
    throw new Error('OPC UA packages missing: run `npm install` in ' + ROOT);
  }
  return ua;
}

// OPC UA standard branches hold thousands of diagnostic nodes that have nothing to do with
// the machine program. They are skipped ONLY in the top two levels. A program variable or
// folder that happens to be called `Server` or `Types` deeper down must still be found;
// skipping at every depth made such tags vanish in rb4axis.
export const SKIP_TOP = /^(Types|Views|Server|Aliases|Locations|DataTypes|EventTypes|ObjectTypes|ReferenceTypes|VariableTypes)$/;
const MAX_DEPTH = 8, MAX_NODES = 20000;

/**
 * Typed arrays (Float64Array for LREAL[], Float32Array for REAL[]) become plain arrays
 * HERE, before anything else sees them. JSON turns a typed array into {"0":..,"1":..} with
 * no length, and a page reading that gets NaN without a single error.
 * @param {any} v
 */
export function plain(v) {
  return ArrayBuffer.isView(v) ? Array.prototype.slice.call(v) : v;
}

/** CLI text -> BOOL or number. @param {string} text */
export function parseValue(text) {
  if (/^(true|false)$/i.test(text)) return /^true$/i.test(text);
  const n = Number(text);
  if (text.trim() === '' || !Number.isFinite(n)) throw new Error('unknown value: ' + text);
  return n;
}

/**
 * Walks the address space from ObjectsFolder and maps "Path.To.Var" -> nodeId string.
 *
 * browse() returns only the FIRST batch of a large folder, and the rest has to be fetched
 * with browseNext and the continuation point. Skipping that step truncates folders
 * silently, and the tags that vanish are the ones at the end of the list.
 *
 * Variables are NOT descended into. Array elements are child nodes too (one LREAL[0..49]
 * is 50 more nodes), and walking them made a real rb4axis sim project hit the node cap,
 * with every tag past the cap silently missing. Hitting the cap is now reported as
 * `truncated` instead.
 * ponytail: struct members are unreachable; add a depth-limited member walk when a scene needs one.
 *
 * @param {{browse: Function, browseNext: Function}} session
 * @param {Array<{path: string, kind: string}>} [list] optional collector for --tree
 * @returns {Promise<{map: Map<string, string>, truncated: boolean}>}
 */
export async function browseAll(session, list) {
  const map = new Map();
  let visited = 0;

  async function children(node) {
    const out = [];
    let r;
    try { r = await session.browse(node); } catch { return out; }
    for (;;) {
      out.push(...(r.references || []));
      const cp = r.continuationPoint;
      if (!cp || !cp.length) break;
      try { r = await session.browseNext(cp, false); } catch { break; }
    }
    return out;
  }

  async function walk(node, prefix, depth) {
    if (depth > MAX_DEPTH || visited > MAX_NODES) return;
    for (const ref of await children(node)) {
      if (visited++ > MAX_NODES) return;
      const name = ref.browseName.name;
      if (depth <= 1 && SKIP_TOP.test(name)) continue;
      const p = prefix ? prefix + '.' + name : name;
      if (ref.nodeClass === 2) {                    // Variable
        map.set(p, ref.nodeId.toString());
        list?.push({ path: p, kind: 'Variable' });
      } else if (ref.nodeClass === 1) {             // Object (folder)
        list?.push({ path: p, kind: 'Object' });
        await walk(ref.nodeId, p, depth + 1);
      }
    }
  }

  await walk('ObjectsFolder', '', 0);
  return { map, truncated: visited > MAX_NODES };
}

/**
 * "…new_Controller_0.GlobalVars.X" -> "X" when `prefix` ("GlobalVars.") appears as a
 * whole path segment ANYWHERE. The Sysmac simulator publishes globals under
 * DeviceSet.Configuration.Resources.<controller>.GlobalVars, not at the root.
 * Returns null for anything that is not a direct child.
 * @param {string} p @param {string} prefix
 */
export function globalName(p, prefix) {
  const q = '.' + p;
  const i = q.lastIndexOf('.' + prefix);
  if (i < 0) return null;
  const n = q.slice(i + 1 + prefix.length);
  return n && !n.includes('.') ? n : null;
}

/**
 * name -> nodeId. Tries the full path `prefix + name` first, then any path ending in
 * `.name`. A controller whose tree has one extra level (<Controller>.GlobalVars.X) would
 * otherwise fail for EVERY tag at once, which looks exactly like variables that do not exist.
 * @param {Map<string, string>} map
 * @param {string} prefix
 */
export function makeLookup(map, prefix) {
  const bySuffix = new Map();
  for (const [p, id] of map) {
    const n = p.split('.').pop();
    if (!bySuffix.has(n)) bySuffix.set(n, id);
  }
  return (/** @type {string} */ name) => map.get(prefix + name) || bySuffix.get(name) || null;
}

/**
 * Used when NOT ONE tag is found. It prints the tree that really exists instead of
 * guessing, because "tag not found" matches four different causes and each wrong guess
 * costs a round trip to Studio.
 * @param {Map<string, string>} map
 * @param {string} prefix
 */
export function diagnoseEmpty(map, prefix) {
  const paths = [...map.keys()].filter(p => !/^Server\b/.test(p));
  const lines = ['NOT ONE tag found. Variable nodes visible: ' + map.size, 'sample paths:'];
  for (const p of paths.slice(0, 25)) lines.push('  ' + p);
  if (paths.length > 25) lines.push('  ... ' + (paths.length - 25) + ' more');
  if (paths.length && !paths.some(p => globalName(p, prefix))) {
    lines.push('no path contains a "' + prefix + '" segment: pass --prefix with the real one (see --tree).');
  }
  lines.push('otherwise check, in order: Transfer to simulator done; Network Publish = Publish Only;');
  lines.push('the program is assigned to a task (docs/SETUP.md).');
  return lines;
}

/**
 * Tag access over one session. Split from connect() so tests can drive it with a fake
 * session.
 * @param {any} session node-opcua ClientSession, or a fake with read/write
 * @param {{prefix?: string, readOnly?: boolean, writable?: Set<string>|null, map?: Map<string,string>}} [opts]
 *   writable: null = any resolved tag (CLI only); the plant passes its scene's `in` tags.
 *   readOnly: twin mode. Enforced HERE, not in the UI: never write to a real machine.
 */
export function createConn(session, { prefix = 'GlobalVars.', readOnly = false, writable = null, map = new Map() } = {}) {
  const lookup = makeLookup(map, prefix);
  /** @type {Record<string, {nodeId: string, dataType: any, arrayType: any}>} */
  const meta = {};
  /** @type {Record<string, any>} */
  const values = {};
  const subs = [];

  const conn = {
    session, map, meta, values, readOnly, prefix,
    truncated: false,           // set by connect() when the browse hit the node cap

    /**
     * Reads each tag once, in batches. That read is what tells us each tag's data type and
     * shape. The type could be guessed from its IEC name, but a wrong guess is rejected as
     * BadTypeMismatch, and that message does not say which tag.
     * @param {string[]} names
     */
    async resolve(names) {
      const todo = [], missing = [];
      for (const n of names) {
        const id = lookup(n);
        if (id) todo.push([n, id]); else missing.push(n);
      }
      for (let i = 0; i < todo.length; i += 500) {
        const chunk = todo.slice(i, i + 500);
        const res = await session.read(chunk.map(([, nodeId]) => ({ nodeId, attributeId: ATTR_VALUE })));
        chunk.forEach(([n, nodeId], k) => {
          const d = res[k];
          if (!d || !d.statusCode.isGood()) { missing.push(n); return; }
          meta[n] = { nodeId, dataType: d.value.dataType, arrayType: d.value.arrayType };
          values[n] = plain(d.value.value);
        });
      }
      return { found: todo.map(t => t[0]).filter(n => meta[n]), missing };
    },

    /** Fresh values, in the order asked. @param {string[]} names */
    async read(names) {
      const { missing } = await conn.resolve(names);
      if (missing.length) throw new Error('tag not found: ' + missing.join(' '));
      return names.map(n => values[n]);
    },

    /**
     * One batched write. An array element is written by sending the WHOLE array: the last
     * known copy with that element replaced. An IndexRange mistake is rejected with a message
     * that does not say which index.
     * @param {Array<{name: string, value: any, index?: number}>} changes
     */
    async write(changes) {
      if (readOnly) throw new Error('read-only session (twin mode): write refused');
      /** @type {Map<string, any>} */
      const merged = new Map();
      for (const { name, value, index } of changes) {
        if (writable && !writable.has(name)) throw new Error('tag not writable: ' + name);
        if (!meta[name]) throw new Error('tag not resolved: ' + name);
        if (index == null) { merged.set(name, value); continue; }
        const base = merged.has(name) ? merged.get(name) : values[name];
        const arr = Array.isArray(base) ? base.slice() : [];
        if (index < 0 || index >= arr.length) throw new Error(`index out of range: ${name}[${index}]`);
        arr[index] = value;
        merged.set(name, arr);
      }
      const names = [...merged.keys()];
      const codes = await session.write(names.map(n => ({
        nodeId: meta[n].nodeId, attributeId: ATTR_VALUE,
        value: { value: { dataType: meta[n].dataType, arrayType: meta[n].arrayType, value: merged.get(n) } }
      })));
      const bad = [];
      names.forEach((n, i) => {
        if (codes[i] && !codes[i].isGood()) bad.push(n + ' ' + codes[i].name);
        else values[n] = merged.get(n);
      });
      if (bad.length) throw new Error('write rejected: ' + bad.join(', '));
    },

    /**
     * Subscribes to value changes. The queue holds more than one value, so every
     * intermediate value arrives: an edge that toggles back within one publishing interval
     * must still be seen. Returns the intervals the SERVER actually granted.
     * @param {string[]} names
     * @param {(name: string, value: any, dataValue: any) => void} onChange
     * @param {{samplingMs?: number, publishingMs?: number}} [opt]
     */
    async subscribe(names, onChange, { samplingMs = 50, publishingMs = samplingMs } = {}) {
      const sub = await session.createSubscription2({
        requestedPublishingInterval: publishingMs, requestedLifetimeCount: 1000,
        requestedMaxKeepAliveCount: 20, maxNotificationsPerPublish: 0,
        publishingEnabled: true, priority: 10
      });
      subs.push(sub);
      const granted = [];
      for (const n of names) {
        if (!meta[n]) throw new Error('tag not resolved: ' + n);
        const item = await sub.monitor({ nodeId: meta[n].nodeId, attributeId: ATTR_VALUE },
          { samplingInterval: samplingMs, queueSize: 16, discardOldest: true }, TS_BOTH);
        granted.push(item.result?.revisedSamplingInterval);
        item.on('changed', dv => { const v = plain(dv.value.value); values[n] = v; onChange(n, v, dv); });
      }
      return { sub, publishingMs: sub.publishingInterval, samplingMs: granted };
    },

    async close() {
      for (const s of subs.splice(0)) await s.terminate().catch(() => {});
    },

    /** @param {(event: string) => void} cb */
    onLost(cb) {}
  };
  return conn;
}

/**
 * Connects, browses the whole tree once, and returns a conn (see createConn).
 * @param {{endpoint?: string, user?: string|null, pass?: string|null, list?: Array<{path: string, kind: string}>,
 *          prefix?: string, readOnly?: boolean, writable?: Set<string>|null}} [opts]
 */
export async function connect({ endpoint = 'opc.tcp://127.0.0.1:4840', user = null, pass = null, list, ...opts } = {}) {
  const { OPCUAClient, MessageSecurityMode, SecurityPolicy, UserTokenType, OPCUACertificateManager } = loadUa();
  // The certificate folder MUST be explicit. Left implicit, node-opcua waits forever at
  // "Creating default certificate". server/pki/ is gitignored because it holds a private key.
  const cm = new OPCUACertificateManager({
    rootFolder: path.join(ROOT, 'server', 'pki'),
    automaticallyAcceptUnknownCertificate: true
  });
  await cm.initialize();
  const client = OPCUAClient.create({
    endpointMustExist: false,
    connectionStrategy: { maxRetry: 1 },
    clientCertificateManager: cm,
    // None = what is ticked in the simulator's Security Settings. Without it the client
    // certificate must be trusted first, and a rejected certificate reports something that
    // looks like a wrong password.
    securityMode: MessageSecurityMode.None,
    securityPolicy: SecurityPolicy.None
  });
  await client.connect(endpoint);
  let session;
  try {
    session = user
      ? await client.createSession({ type: UserTokenType.UserName, userName: user, password: pass || '' })
      : await client.createSession();
  } catch (e) {
    await client.disconnect().catch(() => {});
    throw e;
  }

  // NodeIds are BROWSED, never made up. The namespace index ("ns=4") differs between
  // controllers and Studio versions, and a wrong one reads exactly like "variable not there".
  const { map, truncated } = await browseAll(session, list);
  const conn = createConn(session, { ...opts, map });
  conn.truncated = truncated;
  const closeSubs = conn.close;
  conn.close = async () => {
    await closeSubs();
    await session.close().catch(() => {});
    await client.disconnect().catch(() => {});
  };
  // A simulator that is stopped does not always close its socket cleanly. It is the CLIENT
  // events that declare the link lost; a dead session still looks connected from outside.
  conn.onLost = cb => { for (const ev of ['connection_lost', 'close', 'abort']) client.on(ev, () => cb(ev)); };
  return conn;
}
