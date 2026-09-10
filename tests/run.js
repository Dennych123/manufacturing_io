// node tests/run.js: every tests/*.test.js in its own process. No framework.
// Each suite prints one line per check ('  OK  ' / '>>BAD ' / '  SKIP ') and exits non-zero on failure.
import { spawnSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const dir = path.dirname(fileURLToPath(import.meta.url));
const suites = fs.readdirSync(dir).filter(f => f.endsWith('.test.js')).sort();

let failed = 0;
for (const f of suites) {
  const r = spawnSync(process.execPath, [path.join(dir, f)], { encoding: 'utf8' });
  const out = (r.stdout || '') + (r.stderr || '');
  console.log('=== ' + f.replace(/\.test\.js$/, ''));
  process.stdout.write(out.endsWith('\n') || !out ? out : out + '\n');
  if (r.status !== 0) failed++;
}
console.log(failed ? '\nFAILED: ' + failed + ' suite(s)' : '\nALL SUITES PASS (' + suites.length + ')');
process.exit(failed ? 1 : 0);
