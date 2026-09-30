// @ts-check
// A reader for Keyence VT STUDIO screen files (.vs4, "EditorB_ed7"): the touch panel's screens and
// their parts, as far as a simulator needs them - where each part is, what it says, which PLC bit it
// lights from or writes, and which screen a switch goes to. Drawn from this, the machine's own HMI
// runs against the ladder soft-PLC (web/hmi.html).
//
// There is no published format. What is below was read out of the add-on's own file and checked
// against its program (a lamp labelled NORMAL/FAULT reads W405.00 'NORMAL E FAULT', the ALARM
// RESET switch writes W450.09 'PB_BZ.RESET', the MENU's MASTER button goes to the screen titled
// MASTER). The layout, all little-endian u16:
//
//   screen   [7008] x y w h flags name\0            '@0002_B' = base screen 2, '@G002_W' a window
//   part     <prefix> [7089] ? ? 0 0 ? [7030] size 7 name\0 <items>
//     prefix   devices and actions, just BEFORE the [7089]:
//                0009 0000 <bit> <area>              a bit device: bit = word*16 + bit, area 1 = W
//                1101 0000 0000 0000 0000 0000       a switch action (momentary) - its device follows
//                ... 8000 <screen> ffff 0000         touching it goes to <screen>; 4000 0000 = back
//     item     [7007] ... 0005 "Arial"\0 <len> <text>\0 ...   then a geometry record:
//              [70b0|70b1|70b6|70bc|7081|...] x y w h            the item's box
//
// Everything is in panel dots (the VT3-W4 is 320 x 128).

/** @typedef {{rect: number[], labels: string[], tag: number, fg?: string, bg?: string}} Item */
/** @typedef {{kind: string, name: string, items: Item[], lamp?: string, write?: string, action?: number, go?: number|'back'}} Part */
/** @typedef {{no: number, id: string, title: string, x: number, y: number, w: number, h: number, parts: Part[]}} Screen */

const GEOM = new Set([0x70b0, 0x70b1, 0x70b6, 0x70b7, 0x70bc, 0x70bd, 0x70be, 0x70bf, 0x7080, 0x7081, 0x7082, 0x7083]);
const AREA = ['', 'W', 'H', 'D', 'A'];

/**
 * @param {Uint8Array} bytes the .vs4 file
 * @returns {{screens: Screen[], warnings: string[]}}
 */
export function readVs4(bytes) {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const n = bytes.byteLength;
  const u16 = (/** @type {number} */ o) => (o >= 0 && o + 1 < n ? dv.getUint16(o, true) : 0);
  const zstr = (/** @type {number} */ o, max = 80) => { let s = ''; for (let k = 0; k < max; k++) { const c = u16(o + 2 * k); if (!c) break; s += String.fromCharCode(c); } return s; };
  const head = String.fromCharCode(...bytes.subarray(0, 11));
  /** @type {string[]} */
  const warnings = [];
  if (head !== 'EditorB_ed7') warnings.push('not a VT STUDIO file this reader knows (header "' + head + '")');
  // Screens: [7008] records whose name looks like one.
  /** @type {Array<{o: number, end: number, x: number, y: number, w: number, h: number, name: string}>} */
  const heads = [];
  for (let o = 0; o + 16 < n; o += 2) {
    if (u16(o) !== 0x7008) continue;
    const name = zstr(o + 12, 40);
    if (!/^(@[0-9A-Z]\d{3}_[A-Z0-9]|[A-Z][A-Za-z ]{3,})$/.test(name)) continue;
    const w = u16(o + 6), h = u16(o + 8);
    if (!w || !h || w > 2000 || h > 2000) continue;
    heads.push({ o, end: 0, x: u16(o + 2), y: u16(o + 4), w, h, name });
  }
  heads.forEach((s, i) => { s.end = heads[i + 1]?.o ?? Math.min(n, s.o + 0x10000); });
  /** @type {Screen[]} */
  const screens = [];
  for (const s of heads) {
    // Parts: [7089] with a [7030] name 12 bytes on.
    /** @type {number[]} */
    const at = [];
    for (let o = s.o; o + 20 < s.end; o += 2) if (u16(o) === 0x7089 && u16(o + 12) === 0x7030) at.push(o);
    /** @type {Part[]} */
    const parts = [];
    let prevEnd = s.o + 12;
    at.forEach((o, i) => {
      const name = zstr(o + 18, 16);
      const end = at[i + 1] ?? s.end;
      /** @type {Part} */
      const p = { kind: (/^([A-Za-z]+)_/.exec(name)?.[1] ?? '?'), name, items: [] };
      // The prefix, between the last part's geometry and this header.
      const pre = Math.max(prevEnd, o - 64);
      /** @type {string[]} */
      const devs = [];
      for (let q = pre; q + 8 <= o; q += 2) {
        if (u16(q) === 0x0009 && u16(q + 2) === 0 && u16(q + 6) > 0 && u16(q + 6) < AREA.length) {
          const bit = u16(q + 4);
          devs.push(AREA[u16(q + 6)] + Math.floor(bit / 16) + '.' + String(bit % 16).padStart(2, '0'));
        }
        if ((u16(q) & 0xf0ff) === 0x1001 && u16(q + 2) === 0 && u16(q + 12) === 0x0009) p.action = u16(q);
      }
      if (u16(o - 8) === 0x8000 && u16(o - 4) === 0xffff) p.go = u16(o - 6);
      if (u16(o - 8) === 0x4000 && u16(o - 4) === 0xffff) p.go = 'back';
      if (p.action && devs.length) { p.write = devs[devs.length - 1]; if (devs.length > 1) p.lamp = devs[0]; }
      else if (devs.length) p.lamp = devs[devs.length - 1];
      // Items: texts after a font name, each closed by the next geometry record.
      /** @type {string[]} */
      let pending = [];
      let lastGeomEnd = o;
      for (let q = o + 18 + 2 * (name.length + 1); q + 10 < end; q += 2) {
        const w = u16(q);
        if (GEOM.has(w)) {
          const rect = [u16(q + 2), u16(q + 4), u16(q + 6), u16(q + 8)];
          if (rect[2] > 0 && rect[3] > 0 && rect[0] + rect[2] <= s.w + 2 && rect[1] + rect[3] <= s.h + 2) {
            // Colours are 0x8RGB (4 bits a channel) after the box: text first, then background.
            /** @type {string[]} */
            const cols = [];
            for (let k = 5; k < 14 && cols.length < 2; k++) { const c = u16(q + 2 * k); if ((c & 0xf000) === 0x8000) cols.push('#' + ((c >> 8) & 15).toString(16) + ((c >> 4) & 15).toString(16) + (c & 15).toString(16)); if (GEOM.has(c) || c === 0x7007) break; }
            p.items.push({ rect, labels: pending, tag: w, ...(cols.length ? { fg: cols[0], bg: cols[1] } : {}) });
            pending = [];
            lastGeomEnd = q + 10;
          }
          continue;
        }
        // A font: <len> "Arial" \0, then the text: <len> <chars> \0.
        if (w >= 3 && w <= 20 && /^[A-Za-z][A-Za-z ]+$/.test(zstr(q + 2, w)) && u16(q + 2 + 2 * w) === 0 && /^(Arial|MS |Tahoma|Courier|Times|Segoe|Verdana)/.test(zstr(q + 2, w))) {
          const t = q + 4 + 2 * w, len = u16(t);
          if (len >= 1 && len <= 120 && u16(t + 2 + 2 * len) === 0) {
            let txt = '';
            for (let k = 0; k < len; k++) txt += String.fromCharCode(u16(t + 2 + 2 * k));
            if (!/[\x00-\x09\x0b\x0c\x0e-\x1f￿]/.test(txt)) pending.push(txt.replace(/\r\n/g, '\n'));
            q = t + 2 * len;
          }
        }
      }
      prevEnd = lastGeomEnd;
      parts.push(p);
    });
    const m = /^@(\d{4})_B$/.exec(s.name);
    // The screen's title is the text in its top-left corner, where every screen of this panel has it.
    const tl = parts.flatMap(p => p.items).filter(it => it.labels.length && it.rect[0] <= 4 && it.rect[1] <= 4).map(it => it.labels[it.labels.length - 1]);
    screens.push({ no: m ? +m[1] : -1, id: s.name, title: tl[0] ?? s.name, x: s.x, y: s.y, w: s.w, h: s.h, parts });
  }
  if (!screens.length) warnings.push('no screens found');
  // Screen names: a table of 38-byte entries before the screens, [ffff ffff ffff][no][name, 13 chars].
  // The number in front of a name belongs to the entry BEFORE it (checked on the menu: its MASTER,
  // FAULT CFRM, MONITOR and AUX. OPERATION buttons go to 2, 6, 9 and 10, the entries after MENU,
  // MASTER... carry 1, 2... ).
  const first = heads[0]?.o ?? n;
  /** @type {Array<{p: number, no: number, name: string}>} */
  const tab = [];
  for (let o = 8; o + 30 < first; o += 2) {
    if (u16(o - 4) !== 0xffff || u16(o - 2) > 4000) continue;
    const name = zstr(o, 14);
    if (name.length >= 2 && /^[A-Z0-9][A-Za-z0-9 ._/()\-]+$/.test(name)) tab.push({ p: o, no: u16(o - 2), name });
  }
  // The table is the longest run of entries 38 bytes apart (an empty slot is a gap of 76).
  let best = /** @type {typeof tab} */ ([]), run = /** @type {typeof tab} */ ([]);
  for (const e of tab) {
    const prev = run[run.length - 1];
    run = prev && (e.p - prev.p) % 38 === 0 && e.p - prev.p <= 38 * 3 ? [...run, e] : [e];
    if (run.length > best.length) best = run;
  }
  tab.length = 0; tab.push(...best);
  /** @type {Map<number, string>} */
  const names = new Map();
  if (tab.length) {
    const before = zstr(tab[0].p - 38, 14);
    if (/^[A-Z0-9][A-Za-z0-9 ._/()\-]+$/.test(before)) names.set(tab[0].no, before);
    tab.forEach((e, i) => { if (tab[i + 1]) names.set(tab[i + 1].no, e.name); });
  }
  for (const s of screens) if (names.has(s.no)) s.title = /** @type {string} */ (names.get(s.no));
  return { screens, warnings };
}
