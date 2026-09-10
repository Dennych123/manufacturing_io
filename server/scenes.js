// @ts-check
// Scene files on disk: scenes/<name>.json. The editor saves through here only (docs/PLAN.md §10).
// A version is a hash of the file's bytes, so a stale editor (another tab, another person, a
// hand edit) gets 409 instead of silently overwriting what it never saw.
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { validate, stringify, NAME_RE } from '../lib/scene.js';

export class SceneError extends Error {
  /** @param {number} code HTTP status @param {string} msg @param {any} [extra] */
  constructor(code, msg, extra) { super(msg); this.code = code; this.extra = extra; }
}

/** @param {Buffer|string} bytes */
export const versionOf = bytes => crypto.createHash('sha256').update(bytes).digest('hex').slice(0, 16);

/** @param {string} root @param {string} name */
function fileOf(root, name) {
  if (typeof name !== 'string' || !NAME_RE.test(name)) throw new SceneError(400, 'scene name must match ' + NAME_RE);
  return path.join(root, 'scenes', name + '.json');
}

/** @param {string} root @param {string} name @returns {{version: string, scene: any}} */
export function readScene(root, name) {
  const f = fileOf(root, name);
  if (!fs.existsSync(f)) throw new SceneError(404, 'no scene ' + name);
  const bytes = fs.readFileSync(f);
  return { version: versionOf(bytes), scene: JSON.parse(bytes.toString('utf8')) };
}

/**
 * Validates, writes the canonical text (so two saves of the same scene are byte-identical) via
 * .tmp + rename, and returns the new version. `baseVersion` null means "create": refused when
 * the file exists.
 * @param {string} root @param {string} name @param {string|null} baseVersion @param {any} scene
 * @returns {{version: string, changed: boolean}}
 */
export function saveScene(root, name, baseVersion, scene) {
  const f = fileOf(root, name);
  const errs = validate(scene);
  if (errs.length) throw new SceneError(422, 'scene is invalid', { errors: errs });
  if (scene.name !== name) throw new SceneError(422, 'scene.name "' + scene.name + '" must equal the file name "' + name + '"');
  const cur = fs.existsSync(f) ? versionOf(fs.readFileSync(f)) : null;
  if (cur !== (baseVersion ?? null)) {
    throw new SceneError(409, cur ? 'scene changed since you loaded it (reload, then redo the edit)' : 'scene no longer exists', { version: cur });
  }
  const text = stringify(scene);
  const version = versionOf(text);
  if (version === cur) return { version, changed: false };
  fs.writeFileSync(f + '.tmp', text);
  fs.renameSync(f + '.tmp', f);
  return { version, changed: true };
}
