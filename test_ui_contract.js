/* test_ui_contract.js — 检查显控页面与 simulator_app.js 的控件契约 */
'use strict';
const fs = require('fs');
const assert = require('assert');

const html = fs.readFileSync('./radar_simulator.html', 'utf8');
const app = fs.readFileSync('./simulator_app.js', 'utf8');

const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(m => m[1]);
const dup = ids.filter((id, i) => ids.indexOf(id) !== i);
assert.deepStrictEqual([...new Set(dup)], [], `Duplicate HTML ids: ${[...new Set(dup)].join(', ')}`);
const idSet = new Set(ids);

const refs = [...app.matchAll(/(?:\$|setText|setVal|setChip|on|bindRange|bindTargetRange)\(\s*'([^']+)'/g)].map(m => m[1]);
const missing = [...new Set(refs)].filter(id => !idSet.has(id));
assert.deepStrictEqual(missing, [], `simulator_app.js references missing ids: ${missing.join(', ')}`);

for (const file of ['radar_engine.js', 'simulator_app.js']) {
  assert(html.includes(`src="${file}"`), `radar_simulator.html must load ${file}`);
}
assert(html.includes('</body>') && html.includes('</html>'), 'HTML must be properly closed');

// 页面必须覆盖核心画布与参数编辑器
for (const id of ['mfdCanvas', 'scanCanvas', 'arrayCanvas', 'patternCanvas', 'waveCanvas', 'ascopeCanvas', 'rdCanvas',
  'inpFGHz', 'inpWaveform', 'inpTau', 'inpBw', 'inpTrW', 'inpDiam', 'inpGrid', 'inpTgR', 'inpTgSpd', 'inpTgHdg',
  'inpTgRcs', 'inpScanAz0', 'inpScanBars', 'inpJamErp', 'inpJamRng']) {
  assert(idSet.has(id), `Missing required UI control #${id}`);
}

console.log(`  [PASS] UI 契约: ${ids.length} 个控件 id，${new Set(refs).size} 个引用，无缺失、无重复`);
