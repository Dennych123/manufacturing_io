// @ts-check
// Just enough ZIP for Sysmac Studio's .smc2: read every entry, replace or add a few, and write
// the rest back BYTE FOR BYTE (headers and compressed data untouched). No zip64, no data
// descriptors: a .smc2 uses neither, and anything else is refused rather than guessed.
import zlib from 'node:zlib';

const CRC = new Int32Array(256).map((_, n) => {
  let c = n;
  for (let k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320 ^ (c >>> 1) : c >>> 1;
  return c;
});
/** @param {Uint8Array} buf */
export function crc32(buf) {
  let c = -1;
  for (let i = 0; i < buf.length; i++) c = CRC[(c ^ buf[i]) & 0xff] ^ (c >>> 8);
  return (c ^ -1) >>> 0;
}

/** @typedef {{name: string, central: Buffer, local: Buffer, data: Buffer}} Entry */

/** @param {Buffer} buf @returns {{entries: Entry[], comment: Buffer}} */
export function readZip(buf) {
  let e = buf.length - 22;
  while (e >= 0 && buf.readUInt32LE(e) !== 0x06054b50) e--;
  if (e < 0) throw new Error('not a ZIP: no end of central directory');
  const n = buf.readUInt16LE(e + 10), cdOff = buf.readUInt32LE(e + 16);
  const comment = Buffer.from(buf.subarray(e + 22, e + 22 + buf.readUInt16LE(e + 20)));
  /** @type {Entry[]} */
  const entries = [];
  let p = cdOff;
  for (let i = 0; i < n; i++) {
    if (buf.readUInt32LE(p) !== 0x02014b50) throw new Error('bad central directory entry #' + i);
    const fn = buf.readUInt16LE(p + 28), xl = buf.readUInt16LE(p + 30), cl = buf.readUInt16LE(p + 32);
    const central = Buffer.from(buf.subarray(p, p + 46 + fn + xl + cl));
    const lo = buf.readUInt32LE(p + 42);
    if (buf.readUInt32LE(lo) !== 0x04034b50) throw new Error('bad local header #' + i);
    if (buf.readUInt16LE(lo + 6) & 8) throw new Error('data descriptors are not supported');
    const lfn = buf.readUInt16LE(lo + 26), lxl = buf.readUInt16LE(lo + 28);
    const local = Buffer.from(buf.subarray(lo, lo + 30 + lfn + lxl));
    const start = lo + 30 + lfn + lxl;
    const data = Buffer.from(buf.subarray(start, start + central.readUInt32LE(20)));
    entries.push({ name: central.subarray(46, 46 + fn).toString('utf8'), central, local, data });
    p += 46 + fn + xl + cl;
  }
  return { entries, comment };
}

/** Uncompressed bytes of an entry. @param {Entry} e */
export function content(e) {
  const m = e.central.readUInt16LE(10);
  if (m === 0) return e.data;
  if (m === 8) return zlib.inflateRawSync(e.data);
  throw new Error(e.name + ': compression method ' + m + ' not supported');
}

/** @param {{entries: Entry[], comment: Buffer}} zip */
export function writeZip({ entries, comment }) {
  const parts = [], cds = [];
  let off = 0;
  for (const e of entries) {
    const c = Buffer.from(e.central);
    c.writeUInt32LE(off, 42);
    cds.push(c);
    parts.push(e.local, e.data);
    off += e.local.length + e.data.length;
  }
  const cd = Buffer.concat(cds);
  const eocd = Buffer.alloc(22);
  eocd.writeUInt32LE(0x06054b50, 0);
  eocd.writeUInt16LE(entries.length, 8);
  eocd.writeUInt16LE(entries.length, 10);
  eocd.writeUInt32LE(cd.length, 12);
  eocd.writeUInt32LE(off, 16);
  eocd.writeUInt16LE(comment.length, 20);
  return Buffer.concat([...parts, cd, eocd, comment]);
}

/** @param {Date} d */
function dos(d) {
  return { time: (d.getHours() << 11) | (d.getMinutes() << 5) | (d.getSeconds() >> 1),
           date: ((d.getFullYear() - 1980) << 9) | ((d.getMonth() + 1) << 5) | d.getDate() };
}

/** Extra field with its NTFS (0x000a) times set to `when`; other blocks (Studio's 0x9999 padding) kept. @param {Buffer} extra @param {Date} when */
function stampNtfs(extra, when) {
  const x = Buffer.from(extra);
  const ft = (BigInt(when.getTime()) + 11644473600000n) * 10000n;
  for (let i = 0; i + 4 <= x.length;) {
    const tag = x.readUInt16LE(i), size = x.readUInt16LE(i + 2);
    if (tag === 0x000a && size >= 32 && x.readUInt16LE(i + 8) === 1) for (const k of [12, 20, 28]) x.writeBigUInt64LE(ft, i + k);
    i += 4 + size;
  }
  return x;
}

// What Studio 1.66 writes: version made by 4.5, needed 2.0, local extra = 16-byte 0x9999
// padding + NTFS times, central extra = NTFS times, file attribute ARCHIVE.
const NTFS = Buffer.from('0a00200000000000010018000000000000000000000000000000000000000000000000000000', 'hex').subarray(0, 36);
const DEFAULT = { vmade: 0x2d, vneed: 20, flags: 0, intAttr: 0, extAttr: 0x20,
                  lextra: Buffer.concat([Buffer.from('99991000', 'hex'), Buffer.alloc(16), NTFS]), cextra: NTFS };

/**
 * A new deflated entry. With a template, its version/flag/attribute fields and extra-field
 * layout are copied, so a replaced file looks like one Studio wrote.
 * @param {Entry|null} t @param {string} name @param {Buffer} bytes @param {Date} [when]
 * @returns {Entry}
 */
export function makeEntry(t, name, bytes, when = new Date()) {
  const f = t ? {
    vmade: t.central.readUInt16LE(4), vneed: t.central.readUInt16LE(6), flags: t.central.readUInt16LE(8),
    intAttr: t.central.readUInt16LE(36), extAttr: t.central.readUInt32LE(38),
    lextra: t.local.subarray(30 + t.local.readUInt16LE(26)),
    cextra: t.central.subarray(46 + t.central.readUInt16LE(28), 46 + t.central.readUInt16LE(28) + t.central.readUInt16LE(30)),
  } : DEFAULT;
  const data = zlib.deflateRawSync(bytes);
  const nm = Buffer.from(name, 'utf8'), { time, date } = dos(when), crc = crc32(bytes);
  const lx = stampNtfs(f.lextra, when), cx = stampNtfs(f.cextra, when);
  const local = Buffer.alloc(30 + nm.length + lx.length);
  local.writeUInt32LE(0x04034b50, 0); local.writeUInt16LE(f.vneed, 4); local.writeUInt16LE(f.flags, 6); local.writeUInt16LE(8, 8);
  local.writeUInt16LE(time, 10); local.writeUInt16LE(date, 12); local.writeUInt32LE(crc, 14);
  local.writeUInt32LE(data.length, 18); local.writeUInt32LE(bytes.length, 22);
  local.writeUInt16LE(nm.length, 26); local.writeUInt16LE(lx.length, 28);
  nm.copy(local, 30); lx.copy(local, 30 + nm.length);
  const central = Buffer.alloc(46 + nm.length + cx.length);
  central.writeUInt32LE(0x02014b50, 0); central.writeUInt16LE(f.vmade, 4); central.writeUInt16LE(f.vneed, 6);
  central.writeUInt16LE(f.flags, 8); central.writeUInt16LE(8, 10); central.writeUInt16LE(time, 12); central.writeUInt16LE(date, 14);
  central.writeUInt32LE(crc, 16); central.writeUInt32LE(data.length, 20); central.writeUInt32LE(bytes.length, 24);
  central.writeUInt16LE(nm.length, 28); central.writeUInt16LE(cx.length, 30);
  central.writeUInt16LE(f.intAttr, 36); central.writeUInt32LE(f.extAttr, 38);
  nm.copy(central, 46); cx.copy(central, 46 + nm.length);
  return { name, central, local, data };
}
