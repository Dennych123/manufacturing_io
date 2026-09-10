// Repo rules that break silently: exact pins, no line-ending conversion, no private key in git.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const read = f => fs.readFileSync(path.join(ROOT, f), 'utf8');
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

const pkg = JSON.parse(read('package.json'));
for (const [n, v] of Object.entries(pkg.dependencies || {})) chk('exact pin ' + n, /^\d+\.\d+\.\d+$/.test(v), v);
chk('package-lock.json present', fs.existsSync(path.join(ROOT, 'package-lock.json')));
chk('.gitattributes is "* -text" (generated files are compared byte for byte)', read('.gitattributes').trim() === '* -text');
chk('server/pki/ ignored (holds a private key)', /^server\/pki\/$/m.test(read('.gitignore')));

process.exit(fail ? 1 : 0);
