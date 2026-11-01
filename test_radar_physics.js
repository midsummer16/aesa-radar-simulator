/**
 * test_radar_physics.js
 *
 * 直接测试 radar_engine.js (模拟器实际使用的物理与信号处理引擎，无任何公式复制)
 * 覆盖全部 5 项审查缺陷修复验证与物理公式精度验证
 */
'use strict';

const assert = require('assert');
const fs = require('fs');
const RE = require('./radar_engine.js');

let total = 0, passed = 0;
function test(name, fn) {
  total++;
  try {
    fn();
    passed++;
    console.log(`  [PASS] ${total}. ${name}`);
  } catch (err) {
    console.error(`  [FAIL] ${total}. ${name}`);
    console.error(`         ${err.message}`);
  }
}

console.log('========================================================================');
console.log('  AESA 火控雷达物理引擎 (radar_engine.js) 自动化验证测试套件');
console.log('========================================================================\n');

// ---------------------------------------------------------------------------
// 1. 全 X 波段 (8.0 ~ 12.0 GHz) 阵面构建与方向图解算验证
// ---------------------------------------------------------------------------
test('千级阵元三种栅格拓扑 (正三角形/矩形/同心圆环) 与全X波段无栅瓣间距', () => {
  const tri = RE.buildArray({ diameter: 0.70, fDesignGHz: 12.0, spacingLambda: 0.61, grid: 'triangular' });
  const rect = RE.buildArray({ diameter: 0.70, fDesignGHz: 12.0, spacingLambda: 0.53, grid: 'rectangular' });
  const conc = RE.buildArray({ diameter: 0.70, fDesignGHz: 12.0, spacingLambda: 0.61, grid: 'concentric' });

  assert(tri.N > 1500 && tri.N < 2200, `Triangular N=${tri.N} should be >1500`);
  assert(rect.N > 1500, `Rectangular N=${rect.N} should be >1500`);
  assert(conc.N > 1200, `Concentric N=${conc.N} should be >1200`);

  // 正三角形栅格在 12 GHz 扫描 ±60° 无栅瓣条件: dx <= (2/√3)*λ/(1+sin60°)
  const lam12 = RE.wavelength(12.0);
  const maxDx = (2 / Math.sqrt(3)) * lam12 / (1 + Math.sin(60 * RE.DEG));
  assert(tri.dx <= maxDx + 1e-6, `dx=${tri.dx} exceeds grating lobe limit ${maxDx}`);
});

test('全X波段 (8.0 / 10.0 / 12.0 GHz) 阵面方向图峰值增益 G(f)∝f² 与波束宽度 HPBW(f)∝1/f', () => {
  const arr = RE.buildArray({ diameter: 0.70, fDesignGHz: 12.0, spacingLambda: 0.61, grid: 'triangular' });
  const w8 = RE.buildWeights(arr, { fGHz: 8.0, az: 0, el: 0, taper: 'uniform' });
  const w10 = RE.buildWeights(arr, { fGHz: 10.0, az: 0, el: 0, taper: 'uniform' });
  const w12 = RE.buildWeights(arr, { fGHz: 12.0, az: 0, el: 0, taper: 'uniform' });

  const g8 = RE.dB(RE.gainEval(arr, w8, 8.0, 0, 0));
  const g10 = RE.dB(RE.gainEval(arr, w10, 10.0, 0, 0));
  const g12 = RE.dB(RE.gainEval(arr, w12, 12.0, 0, 0));

  // 20*log10(12/8) = 3.5218 dB
  assert(Math.abs((g12 - g8) - 3.522) < 0.05, `Expected G(12GHz)-G(8GHz) ≈ 3.52 dB, got ${(g12 - g8).toFixed(3)} dB`);
  assert(g10 > g8 && g12 > g10, 'Gain must increase monotonically with frequency');

  const c8 = RE.analyzeCut(...Object.values(RE.cutAz(arr, w8, 8.0, 0, -20, 20, 401)));
  const c12 = RE.analyzeCut(...Object.values(RE.cutAz(arr, w12, 12.0, 0, -20, 20, 401)));
  assert(Math.abs(c8.hpbw / c12.hpbw - 1.5) < 0.06, `HPBW(8GHz)/HPBW(12GHz) expected ~1.50, got ${(c8.hpbw / c12.hpbw).toFixed(3)}`);
});

test('圆口径赋形方向图：均匀加权 (-17.6dB)、低副瓣锥削 (<-33dB)、差波束零深、唯相展宽与 MVDR 自适应零陷', () => {
  const arr = RE.buildArray({ diameter: 0.70, fDesignGHz: 12.0, spacingLambda: 0.61, grid: 'triangular' });
  const f = 10.0;

  const wUni = RE.buildWeights(arr, { fGHz: f, az: 0, el: 0, taper: 'uniform' });
  const aUni = RE.analyzeCut(...Object.values(RE.cutAz(arr, wUni, f, 0, -40, 40, 721)));
  // 连续圆口径 Airy 第一副瓣理论值 -17.57 dB
  assert(Math.abs(aUni.sll - (-17.6)) < 0.8, `Uniform circular SLL expected ~-17.6 dB, got ${aUni.sll.toFixed(2)} dB`);

  const wLow = RE.buildWeights(arr, { fGHz: f, az: 0, el: 0, taper: 'lowsll', taperN: 2.2, pedestal: 0.05 });
  const aLow = RE.analyzeCut(...Object.values(RE.cutAz(arr, wLow, f, 0, -40, 40, 721)));
  assert(aLow.sll < -33.0, `Low-SLL taper expected SLL < -33 dB, got ${aLow.sll.toFixed(2)} dB`);
  assert(aLow.hpbw > aUni.hpbw, 'Low-SLL taper must broaden mainlobe HPBW relative to uniform');

  // Bayliss / 差波束在轴向 (az=0) 形成深零陷
  const wDiff = RE.buildWeights(arr, { fGHz: f, az: 0, el: 0, taper: 'diffAz' });
  const cDiff = RE.cutAz(arr, wDiff, f, 0, -20, 20, 401);
  const pkDiff = Math.max(...cDiff.g);
  const nullDb = RE.dB(RE.gainEval(arr, wDiff, f, 0, 0) / pkDiff);
  assert(nullDb < -40, `Difference pattern boresight null expected < -40 dB, got ${nullDb.toFixed(1)} dB`);

  // 四象限单脉冲鉴角曲线奇对称与线性斜率验证
  const mp0 = RE.monopulseEval(arr, wLow, f, 0, 0);
  const mpPos = RE.monopulseEval(arr, wLow, f, 0.6, 0);
  const mpNeg = RE.monopulseEval(arr, wLow, f, -0.6, 0);
  assert(Math.abs(mp0.errAz) < 1e-6, `Monopulse error at boresight must be 0, got ${mp0.errAz}`);
  assert(mpPos.errAz > 0 && mpNeg.errAz < 0 && Math.abs(mpPos.errAz + mpNeg.errAz) < 1e-4, 'Monopulse discriminator must be odd-symmetric');

  // 唯相二次展宽波束 (ACM/GM)
  const wSpoil = RE.buildWeights(arr, { fGHz: f, az: 0, el: 0, taper: 'spoil', spoilBeta: 5.0 });
  const aSpoil = RE.analyzeCut(...Object.values(RE.cutAz(arr, wSpoil, f, 0, -40, 40, 721)));
  assert(aSpoil.hpbw > aUni.hpbw * 2.0, `Spoiled beam HPBW (${aSpoil.hpbw.toFixed(2)}°) must be >2x uniform (${aUni.hpbw.toFixed(2)}°)`);

  // MVDR 空域自适应零陷 (不含量化误差时零深 <-60dB，含6-bit移相量化时零深 <-35dB)
  const jamAz = -22.0;
  const wMvdrIdeal = RE.buildWeights(arr, { fGHz: f, az: 10, el: 0, taper: 'lowsll', nulls: [{ az: jamAz, el: 0 }], phaseBits: 0 });
  const gMain = RE.gainEval(arr, wMvdrIdeal, f, 10, 0);
  const gJam = RE.gainEval(arr, wMvdrIdeal, f, jamAz, 0);
  const jamNullDb = RE.dB(gJam / gMain);
  assert(jamNullDb < -60, `MVDR null at jammer expected < -60 dB (unquantized), got ${jamNullDb.toFixed(1)} dB`);
});

// ---------------------------------------------------------------------------
// 2. 缺陷3修复验证：天线方向图与雷达方程增益严格同源
// ---------------------------------------------------------------------------
test('天线方向图与雷达方程增益同源性验证 (消除两套增益缺陷)', () => {
  const sim = new RE.RadarSim();
  const tg = sim.addTarget({ name: 'TestTarget', rangeKm: 60, azDeg: 18, altM: 8000, speed: 250, headingDeg: 180, rcs: 5 });
  const diag = sim.analyzeTarget(tg);
  const s = sim.targetState(tg, sim.time);

  // 诊断必须使用最近扫描波束的真实权值，与 dwell 同源
  let nearest = sim.scanList[0], best = 1e9;
  const uT = Math.sin(s.az * RE.DEG) * Math.cos(s.el * RE.DEG), vT = Math.sin(s.el * RE.DEG);
  const wT0 = Math.sqrt(Math.max(0, 1 - uT * uT - vT * vT));
  for (const p of sim.scanList) {
    const uP = Math.sin(p.az * RE.DEG) * Math.cos(p.el * RE.DEG), vP = Math.sin(p.el * RE.DEG);
    const wP = Math.sqrt(Math.max(0, 1 - uP * uP - vP * vP));
    const off = Math.acos(RE.clamp(uT * uP + vT * vP + wT0 * wP, -1, 1));
    if (off < best) { best = off; nearest = p; }
  }
  const wT = RE.buildWeights(sim.array, sim._wOpts(sim.cfg.fGHz, nearest.az, nearest.el, false));
  const wR = RE.buildWeights(sim.array, sim._wOpts(sim.cfg.fGHz, nearest.az, nearest.el, true));
  const directGtDb = RE.dB(RE.gainEval(sim.array, wT, sim.cfg.fGHz, s.az, s.el));
  const directGrDb = RE.dB(RE.gainEval(sim.array, wR, sim.cfg.fGHz, s.az, s.el));

  assert(Math.abs(diag.gTdb - directGtDb) < 1e-9, `Tx gain mismatch: ${diag.gTdb} vs ${directGtDb}`);
  assert(Math.abs(diag.gRdb - directGrDb) < 1e-9, `Rx gain mismatch: ${diag.gRdb} vs ${directGrDb}`);
  assert(Math.abs(diag.scanOffsetDeg - best * RE.RAD) < 1e-9, 'Diagnostic scan offset must equal nearest-beam angular offset');
});

// ---------------------------------------------------------------------------
// 3. 发射波形、FFT 脉冲压缩与 Costas 抗 DRFM 互相关测试
// ---------------------------------------------------------------------------
test('LFM / Barker-13 / Welch-Costas 脉冲压缩 FFT 匹配滤波与旁瓣特性', () => {
  const wfLfm = RE.makeWaveform({ type: 'lfm', tauUs: 20, bwMHz: 20 });
  const mfRect = RE.matchedFilter(wfLfm, 'none');
  const mfHam = RE.matchedFilter(wfLfm, 'hamming');

  // 未加窗 LFM sinc 第一旁瓣 ≈ -13.26 dB；加 Hamming 窗后 PSLR <= -40 dB，失配损耗 ≈ 1.34 dB
  assert(Math.abs(mfRect.pslrDb - (-13.26)) < 0.6, `Unwindowed LFM PSLR expected ~-13.3 dB, got ${mfRect.pslrDb.toFixed(2)} dB`);
  assert(mfHam.pslrDb < -39.0, `Hamming LFM PSLR expected < -39 dB, got ${mfHam.pslrDb.toFixed(2)} dB`);
  assert(Math.abs(mfHam.mismatchLossDb - 1.34) < 0.15, `Hamming mismatch loss expected ~1.34 dB, got ${mfHam.mismatchLossDb.toFixed(2)} dB`);

  // Barker-13 理论峰值旁瓣比 -20*log10(13) = -22.28 dB
  const wfBarker = RE.makeWaveform({ type: 'barker', tauUs: 13, bwMHz: 10 });
  const mfBarker = RE.matchedFilter(wfBarker, 'none');
  assert(Math.abs(mfBarker.pslrDb - (-22.28)) < 0.5, `Barker-13 PSLR expected ~-22.28 dB, got ${mfBarker.pslrDb.toFixed(2)} dB`);

  // Costas 序列合法性与跨脉冲正交跳频对 DRFM 重放的失配抑制
  const c0 = RE.welchCostas(0), c1 = RE.welchCostas(3);
  assert(RE.isCostas(c0) && RE.isCostas(c1), 'Welch construction must produce valid Costas arrays');
  const wfC0 = RE.makeWaveform({ type: 'costas', tauUs: 20, bwMHz: 20, costasShift: 0 });
  const wfC1 = RE.makeWaveform({ type: 'costas', tauUs: 20, bwMHz: 20, costasShift: 3 });
  const crossRatio = RE.crossMatch(wfC0, wfC1);
  assert(RE.dB(crossRatio) < -9.5, `Costas cross-correlation rejection expected < -9.5 dB, got ${RE.dB(crossRatio).toFixed(2)} dB`);
});

// ---------------------------------------------------------------------------
// 4. 传播模型、雷达方程、CA-CFAR 检测统计与 CRT 解距离模糊
// ---------------------------------------------------------------------------
test('传播损耗 (R^4、ITU-R P.676/P.838、STC、遮蔽) 与 CA-CFAR / CRT 解模糊精度', () => {
  // R^4 自由空间损耗：距离翻倍严格下降 12.0412 dB
  const eq40 = RE.radarEquation({ fGHz: 10, PtW: 15000, tauS: 20e-6, nCoh: 48, GtLin: 5000, GrLin: 3500, rcs: 5, Rm: 40000, nfDb: 3.2, lossDb: 3.8 });
  const eq80 = RE.radarEquation({ fGHz: 10, PtW: 15000, tauS: 20e-6, nCoh: 48, GtLin: 5000, GrLin: 3500, rcs: 5, Rm: 80000, nfDb: 3.2, lossDb: 3.8 });
  assert(Math.abs((eq40.snrDb - eq80.snrDb) - 40 * Math.log10(2)) < 1e-6, 'Distance doubling must reduce free-space SNR by 40*log10(2) dB');

  // ITU-R P.838 频变降雨衰减：12 GHz > 10 GHz > 8 GHz
  const r8 = RE.rainLossTwoWayDb(50, 8.0, 25, 40, 5000, 5000, 10);
  const r10 = RE.rainLossTwoWayDb(50, 10.0, 25, 40, 5000, 5000, 10);
  const r12 = RE.rainLossTwoWayDb(50, 12.0, 25, 40, 5000, 5000, 10);
  assert(r12 > r10 && r10 > r8 && r12 / r8 > 2.5, 'Rain attenuation must increase steeply across 8~12 GHz');

  // 3-of-8 参差 PRF 中国剩余定理 (CRT) 解算 85 km 真实距离
  const trueR = 85320;
  const prfs = [11000, 13000, 17000];
  const apps = prfs.map(prf => ({ prf, r: trueR % (RE.C / (2 * prf)), sig: 15 }));
  const resolved = RE.resolveRange(apps, 120, 300000);
  assert(resolved.ok && Math.abs(resolved.R - trueR) < 1.0, `CRT range resolution expected ${trueR}, got ${resolved.R}`);
});

// ---------------------------------------------------------------------------
// 5. 缺陷1与缺陷2修复验证：运动目标 + 扫描图形匹配 + 纯雷达方程能量检测闭环
// ---------------------------------------------------------------------------
test('运动目标运动学更新与扫描图形 (Scan Pattern) 空间匹配约束验证', () => {
  const sim = new RE.RadarSim({ seed: 42 });
  // 目标位于方位 +42°，以 300 m/s 迎头飞行
  const tg = sim.addTarget({ name: 'FlankFighter', rangeKm: 60, azDeg: 42, altM: 8000, speed: 300, headingDeg: 200, rcs: 5.0 });

  const s0 = sim.targetState(tg, 0);
  const s5 = sim.targetState(tg, 5.0);
  assert(s5.R < s0.R && Math.abs(s5.px - s0.px) > 100, 'Target must move in 3D space over time');

  // 配置 1：扫描扇区设为 ±15° (中心 0°) -> 目标 (+42°) 在扫描图形外
  sim.setConfig({ scan: { azCenter: 0, azHalf: 15, bars: 2, elCenter: 0 } });
  const diagOut = sim.analyzeTarget(tg);
  assert.strictEqual(diagOut.verdict, 'OUT_OF_SCAN', 'Target outside scan sector must be diagnosed as OUT_OF_SCAN');
  sim.step(2.5);
  const detOut = sim.plots.filter(p => !p.isFalse && p.truth === tg.id);
  assert.strictEqual(detOut.length, 0, 'Target outside scan sector must produce 0 detections during scan');

  // 配置 2：将扫描中心转向 +40° (扇区 ±15° 覆盖 +25°~+55°) -> 波束扫过目标，成功检出！
  sim.setConfig({ scan: { azCenter: 40, azHalf: 15, bars: 2, elCenter: 0 } });
  const diagIn = sim.analyzeTarget(tg);
  assert.strictEqual(diagIn.verdict, 'OK', `Target inside steered scan sector must be OK, got ${diagIn.verdict}`);
  sim.step(2.5);
  const detIn = sim.plots.filter(p => !p.isFalse && p.truth === tg.id);
  assert(detIn.length >= 1, 'Target inside scan sector must be detected when beam sweeps across it');
});

test('纯雷达方程波形能量与抗干扰/杂波物理闭环 (无任何魔法常数与硬编码分支)', () => {
  // 确认 radar_engine.js 中不含任何 "-= 26" 或 "-= 16.5" 等魔法扣减常数
  const engineSrc = fs.readFileSync('./radar_engine.js', 'utf8');
  assert(!/snrDb\s*-=\s*(26|16\.5)/.test(engineSrc), 'radar_engine.js must not contain hardcoded SNR magic subtractions');

  // (A) 隐身弱目标 (RCS = 0.015 m², R = 75 km)：短脉冲小积累无法探测，长脉冲大相参积累跨过 CFAR 门限检出
  const sim = new RE.RadarSim({ seed: 7 });
  const stealth = sim.addTarget({ name: 'Stealth', rangeKm: 75, azDeg: 5, altM: 8500, speed: 320, headingDeg: 180, rcs: 0.015, swerling: 0 });
  sim.setConfig({ scan: { azCenter: 5, azHalf: 10, bars: 1, elCenter: 0 }, tx: { tauUs: 2, bwMHz: 20, nCpi: 12 } });
  const diagWeak = sim.analyzeTarget(stealth);
  assert.strictEqual(diagWeak.verdict, 'LOW_SNR', `Short pulse weak echo must fail by radar equation SNR, got ${diagWeak.verdict}`);

  sim.setConfig({ fGHz: 11.5, tx: { tauUs: 20, bwMHz: 20, nCpi: 256 }, array: { trPeakW: 16, rxTaper: 'uniform' } });
  const diagStrong = sim.analyzeTarget(stealth);
  assert.strictEqual(diagStrong.verdict, 'OK', `High pulse energy & coherent integration must detect stealth target, got ${diagStrong.verdict}: ${diagStrong.reasons.join('; ')}`);

  // (B) 远距支援瞄准干扰 (Spot Jammer)：由干扰方程 J/N0 抬高噪底致盲 -> 开启 MVDR 空间零陷或全频段捷变恢复探测
  const simJ = new RE.RadarSim({ seed: 11 });
  const fighter = simJ.addTarget({ name: 'Fighter', rangeKm: 70, azDeg: 10, altM: 8000, speed: 280, headingDeg: 180, rcs: 3.0 });
  simJ.addJammer({ type: 'spot', azDeg: -22, elDeg: 0, rangeKm: 90, erpDbW: 75, fGHz: 10.0, bwMHz: 25 });
  const diagJammed = simJ.analyzeTarget(fighter);
  assert.strictEqual(diagJammed.verdict, 'JAMMED', `Spot jammer must raise noise floor via jammer equation, got ${diagJammed.verdict}`);

  simJ.setConfig({ eccm: { adbf: true } });
  const diagMvdr = simJ.analyzeTarget(fighter);
  assert.strictEqual(diagMvdr.verdict, 'OK', `MVDR spatial nulling at jammer angle must suppress J/N0 and restore detection, got ${diagMvdr.verdict}`);

  // (C) 下视慢速地面目标 (GMTI)：未开 DPCA 被地杂波淹没 -> 开启 DPCA 对消 38 dB 主瓣杂波后检出
  const simG = new RE.RadarSim({ seed: 19 });
  simG.applyMode('GMTI');
  simG.setConfig({ eccm: { dpca: false } });
  const convoy = simG.addTarget({ name: 'Convoy', rangeKm: 35, azDeg: 0, altM: 100, speed: 18, headingDeg: 0, rcs: 25, isGround: true });
  // 将扫描俯仰中心对准地面目标的俯角
  const stC = simG.targetState(convoy, 0);
  simG.setConfig({ scan: { azCenter: 0, azHalf: 20, bars: 1, elCenter: stC.el } });
  const diagClutter = simG.analyzeTarget(convoy);
  assert.strictEqual(diagClutter.verdict, 'CLUTTER', `Ground convoy without DPCA must be masked by mainlobe clutter, got ${diagClutter.verdict}`);

  simG.setConfig({ eccm: { dpca: true } });
  const diagDpca = simG.analyzeTarget(convoy);
  assert.strictEqual(diagDpca.verdict, 'OK', `Ground convoy with DPCA must be detected, got ${diagDpca.verdict}: ${diagDpca.reasons.join('; ')}`);
});

test('实时波束方向图接口 beamPattern/arrayMap 与阵面权值同源', () => {
  const sim = new RE.RadarSim();
  const p = sim.beamPattern(15, 2, { points: 401, spanDeg: 20 });
  assert.strictEqual(p.cutAz.ang.length, 401);
  assert(p.gTdb > 30 && p.gRdb > 30, 'Beam pattern must report realistic peak gains');
  const directT = RE.dB(RE.gainEval(sim.array, RE.buildWeights(sim.array, sim._wOpts(sim.cfg.fGHz, 15, 2, false)), sim.cfg.fGHz, 15, 2));
  assert(Math.abs(p.gTdb - directT) < 1e-9, `beamPattern gain must equal gainEval: ${p.gTdb} vs ${directT}`);

  // 切面峰值必须落在波束指向附近
  let pk = 0; for (let i = 1; i < p.cutAz.twoDb.length; i++) if (p.cutAz.twoDb[i] > p.cutAz.twoDb[pk]) pk = i;
  assert(Math.abs(p.cutAz.ang[pk] - 15) < 0.6, `Az cut peak expected near 15°, got ${p.cutAz.ang[pk].toFixed(2)}°`);
  assert(p.hpbwAz > 0.5 && p.hpbwAz < 8, `HPBW out of range: ${p.hpbwAz}`);

  const p2 = sim.beamPattern(0, 0, { cut: '2d', nu: 31, nv: 31, spanDeg: 16 });
  let mi = 0; for (let i = 1; i < p2.map.db.length; i++) if (p2.map.db[i] > p2.map.db[mi]) mi = i;
  const ui = mi % p2.map.nu, vi = Math.floor(mi / p2.map.nu);
  assert(Math.abs(p2.map.uMin + ui * p2.map.du) < 0.05 && Math.abs(p2.map.vMin + vi * p2.map.dv) < 0.05, '2D pattern peak must be at beam center');

  const am = sim.arrayMap(15, 2);
  assert.strictEqual(am.N, sim.array.N);
  assert.strictEqual(am.health.length, sim.array.N);
  assert(am.amp.length === sim.array.N && am.phase.length === sim.array.N, 'arrayMap must expose per-element amplitude/phase');
});

test('7 个内置想定均由真实雷达方程给出预期物理结论', () => {
  const verdict = k => {
    const sim = RE.createScenario(k);
    return sim.analyzeTarget(sim.targets[0]).verdict;
  };
  assert.strictEqual(verdict('default'), 'OK', 'Default scenario targets must be detectable');
  assert.strictEqual(verdict('stealth'), 'LOW_SNR', 'VLO scenario must fail by radar-equation SNR');
  assert.strictEqual(verdict('jam_adbf'), 'JAMMED', 'Spot jammer scenario must be jammed');
  assert.strictEqual(verdict('drfm_rgpo'), 'OK', 'DRFM scenario real target must be detectable');
  assert.strictEqual(verdict('gmti_convoy'), 'CLUTTER', 'Ground convoy must be masked by mainlobe clutter');
  assert.strictEqual(verdict('acm_dogfight'), 'OK', 'ACM close target must be detectable');
  assert.strictEqual(verdict('rain_xband'), 'LOW_SNR', '12 GHz heavy rain must attenuate the target below threshold');

  const j = RE.createScenario('jam_adbf'); j.setConfig({ eccm: { adbf: true } });
  assert.strictEqual(j.analyzeTarget(j.targets[0]).verdict, 'OK', 'ADBF must restore detection');
  const g = RE.createScenario('gmti_convoy'); g.setConfig({ eccm: { dpca: true } });
  assert.strictEqual(g.analyzeTarget(g.targets[0]).verdict, 'OK', 'DPCA must restore ground target detection');
});

test('TWS 扫描航迹关联：多目标不被重复建批', () => {
  const sim = RE.createScenario('default');
  sim.applyMode('TWS');
  for (let i = 0; i < 3000; i++) sim.step(0.02, 0);
  const conf = sim.tracks.filter(t => t.confirmed);
  assert(conf.length >= 2, `Expected at least 2 confirmed tracks, got ${conf.length}`);
  assert(conf.length <= sim.targets.length, `Duplicate tracks detected: ${conf.length} tracks for ${sim.targets.length} targets`);
  for (const tg of sim.targets) assert(conf.some(t => t.truth === tg.id), `Missing confirmed track for ${tg.id}`);
});

console.log('\n========================================================================');
console.log(`  测试汇总: ${passed} / ${total} 项全部通过 (${passed === total ? '100% PASS' : 'FAILED'})`);
console.log('========================================================================');
if (passed !== total) process.exit(1);
