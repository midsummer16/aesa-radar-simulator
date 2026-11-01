/**
 * radar_engine.js — AESA 火控雷达物理/信号引擎 (浏览器与 Node 共用，测试直接测本文件)
 *
 * 核心原则：目标能否被探测，完全由
 *   阵面几何+权值 → 方向图增益(目标方向) → 雷达方程(含大气/降雨/STC/遮蔽/杂波/干扰) → 检测统计(CA-CFAR, M-of-N)
 * 计算得出，不存在任何"按场景写死"的判决。
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.RadarEngine = factory();
})(typeof self !== 'undefined' ? self : this, function () {
'use strict';

const C = 299792458, KB = 1.380649e-23, T0 = 290;
const DEG = Math.PI / 180, RAD = 180 / Math.PI;
const dB = x => 10 * Math.log10(x);
const unDb = x => Math.pow(10, x / 10);
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const wavelength = fGHz => C / (fGHz * 1e9);

/* ------------------------------------------------------------------ RNG */
function makeRng(seed) {
  let s = (seed >>> 0) || 1;
  function next() {
    s = (s + 0x6D2B79F5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }
  return {
    next,
    gauss() { let u = 0; while (u === 0) u = next(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * next()); },
    exp() { let u = 0; while (u === 0) u = next(); return -Math.log(u); },
    range(a, b) { return a + (b - a) * next(); },
    poisson(l) { if (l <= 0) return 0; if (l > 30) return Math.max(0, Math.round(l + Math.sqrt(l) * this.gauss())); const L = Math.exp(-l); let k = 0, p = 1; do { k++; p *= next(); } while (p > L); return k - 1; }
  };
}

/* ------------------------------------------------------------------ FFT */
function fft(re, im, inverse) {
  const n = re.length;
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { let t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = 2 * Math.PI / len * (inverse ? 1 : -1);
    const wr = Math.cos(ang), wi = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let cr = 1, ci = 0;
      for (let k = 0; k < len / 2; k++) {
        const a = i + k, b = a + len / 2;
        const xr = re[b] * cr - im[b] * ci, xi = re[b] * ci + im[b] * cr;
        re[b] = re[a] - xr; im[b] = im[a] - xi; re[a] += xr; im[a] += xi;
        const t = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = t;
      }
    }
  }
  if (inverse) for (let i = 0; i < n; i++) { re[i] /= n; im[i] /= n; }
}
const nextPow2 = n => { let p = 1; while (p < n) p <<= 1; return p; };

/* ------------------------------------------------------- AESA geometry */
function buildArray(o) {
  o = Object.assign({ diameter: 0.70, fDesignGHz: 12, spacingLambda: 0.61, grid: 'triangular', edge: 0.985 }, o || {});
  const lam = wavelength(o.fDesignGHz), R = o.diameter / 2;
  const dx = o.spacingLambda * lam;
  const dy = o.grid === 'triangular' ? dx * Math.sqrt(3) / 2 : dx;
  const xs = [], ys = [];
  if (o.grid === 'concentric') {
    const step = dx * 0.94, nRings = Math.floor((R * o.edge) / step);
    xs.push(0); ys.push(0);
    for (let m = 1; m <= nRings; m++) {
      const r = m * step, cnt = Math.max(6, Math.round(2 * Math.PI * r / step));
      for (let k = 0; k < cnt; k++) {
        const phi = (2 * Math.PI * k) / cnt + (m % 2) * (Math.PI / cnt);
        xs.push(r * Math.cos(phi)); ys.push(r * Math.sin(phi));
      }
    }
  } else {
    const rows = Math.ceil(R / dy) + 1, cols = Math.ceil(R / dx) + 1;
    for (let m = -rows; m <= rows; m++) {
      const off = (o.grid === 'triangular' && Math.abs(m) % 2 === 1) ? dx / 2 : 0;
      for (let n = -cols; n <= cols; n++) {
        const x = n * dx + off, y = m * dy;
        if (Math.hypot(x, y) <= R * o.edge) { xs.push(x); ys.push(y); }
      }
    }
  }
  const N = xs.length;
  let mx = 0, my = 0;
  for (let i = 0; i < N; i++) { mx += xs[i]; my += ys[i]; }
  mx /= N; my /= N;
  const x = new Float64Array(N), y = new Float64Array(N), rho = new Float64Array(N), quad = new Uint8Array(N);
  for (let i = 0; i < N; i++) {
    x[i] = xs[i] - mx; y[i] = ys[i] - my; rho[i] = Math.hypot(x[i], y[i]) / R;
    quad[i] = (x[i] >= 0 && y[i] >= 0) ? 1 : (x[i] < 0 && y[i] >= 0) ? 2 : (x[i] < 0 && y[i] < 0) ? 3 : 4;
  }
  const effCellArea = o.grid === 'concentric' ? (Math.PI * R * R * o.edge * o.edge) / N : dx * dy;
  return { N, x, y, rho, quad, R, diameter: o.diameter, dx, dy, cellArea: effCellArea, grid: o.grid, fDesignGHz: o.fDesignGHz };
}

function makeHealth(N, failRate, seed) {
  const r = makeRng((seed || 1) + 777), h = new Uint8Array(N);
  for (let i = 0; i < N; i++) h[i] = r.next() >= failRate ? 1 : 0;
  return h;
}

/**
 * 阵元复权值。taper: uniform | lowsll | diffAz | diffEl | spoil
 * 返回 {re,im,amp,phase,sumSq,healthy}
 */
function buildWeights(arr, o) {
  const N = arr.N, k = 2 * Math.PI * o.fGHz * 1e9 / C;
  const u0 = Math.sin(o.az * DEG) * Math.cos(o.el * DEG), v0 = Math.sin(o.el * DEG);
  const taper = o.taper || 'uniform', nn = o.taperN != null ? o.taperN : 2.2, ped = o.pedestal != null ? o.pedestal : 0.05;
  const amp = new Float64Array(N), ph = new Float64Array(N);
  let healthy = 0;
  for (let i = 0; i < N; i++) {
    const rho = arr.rho[i], ok = o.health ? o.health[i] : 1;
    const t = ped + (1 - ped) * Math.pow(Math.max(0, 1 - rho * rho), nn);
    let a = 1, extra = 0;
    if (taper === 'lowsll') a = t;
    else if (taper === 'diffAz') a = (arr.x[i] / arr.R) * t;
    else if (taper === 'diffEl') a = (arr.y[i] / arr.R) * t;
    else if (taper === 'spoil') extra = (o.spoilBeta != null ? o.spoilBeta : 5.0) * rho * rho;
    if (!ok) a = 0; else healthy++;
    let p = -k * (arr.x[i] * u0 + arr.y[i] * v0) + extra;
    if (a < 0) { a = -a; p += Math.PI; }
    amp[i] = a; ph[i] = p;
  }
  // 6-bit 移相器 / 数控衰减器量化 (模拟 TR 组件射频量化)
  if (o.phaseBits > 0) {
    const step = 2 * Math.PI / Math.pow(2, o.phaseBits);
    for (let i = 0; i < N; i++) ph[i] = Math.round(ph[i] / step) * step;
  }
  if (o.ampBits > 0) {
    const stepDb = 31.5 / (Math.pow(2, o.ampBits) - 1);
    let mx = 0; for (let i = 0; i < N; i++) if (amp[i] > mx) mx = amp[i];
    for (let i = 0; i < N; i++) {
      if (amp[i] <= 0) continue;
      const q = Math.round(20 * Math.log10(amp[i] / mx) / stepDb) * stepDb;
      amp[i] = q < -31.5 ? 0 : mx * Math.pow(10, q / 20);
    }
  }
  let re = new Float64Array(N), im = new Float64Array(N);
  for (let i = 0; i < N; i++) { re[i] = amp[i] * Math.cos(ph[i]); im[i] = amp[i] * Math.sin(ph[i]); }

  // 数字接收端空域自适应零陷 (ADBF / MVDR 正交投影，令 AF(u_j,v_j)=Σ a_n w_n = 0)
  if (o.nulls && o.nulls.length) {
    const basis = [];
    for (const nl of o.nulls) {
      const uj = Math.sin(nl.az * DEG) * Math.cos(nl.el * DEG), vj = Math.sin(nl.el * DEG);
      const br = new Float64Array(N), bi = new Float64Array(N);
      for (let i = 0; i < N; i++) {
        if (o.health && !o.health[i]) continue;
        const p = k * (arr.x[i] * uj + arr.y[i] * vj);
        br[i] = Math.cos(p); bi[i] = -Math.sin(p);   // b = conj(a)
      }
      for (const q of basis) {
        let cr = 0, ci = 0;
        for (let i = 0; i < N; i++) { cr += q.r[i] * br[i] + q.i[i] * bi[i]; ci += q.r[i] * bi[i] - q.i[i] * br[i]; }
        for (let i = 0; i < N; i++) { br[i] -= cr * q.r[i] - ci * q.i[i]; bi[i] -= cr * q.i[i] + ci * q.r[i]; }
      }
      let nr = 0; for (let i = 0; i < N; i++) nr += br[i] * br[i] + bi[i] * bi[i];
      nr = Math.sqrt(nr); if (nr < 1e-9) continue;
      for (let i = 0; i < N; i++) { br[i] /= nr; bi[i] /= nr; }
      basis.push({ r: br, i: bi });
    }
    for (const q of basis) {
      let cr = 0, ci = 0;
      for (let i = 0; i < N; i++) { cr += q.r[i] * re[i] + q.i[i] * im[i]; ci += q.r[i] * im[i] - q.i[i] * re[i]; }
      for (let i = 0; i < N; i++) { re[i] -= cr * q.r[i] - ci * q.i[i]; im[i] -= cr * q.i[i] + ci * q.r[i]; }
    }
    let mx = 0;
    for (let i = 0; i < N; i++) { amp[i] = Math.hypot(re[i], im[i]); ph[i] = Math.atan2(im[i], re[i]); if (amp[i] > mx) mx = amp[i]; }
    if (mx > 0) for (let i = 0; i < N; i++) { amp[i] /= mx; re[i] /= mx; im[i] /= mx; }
  }
  let sumSq = 0;
  for (let i = 0; i < N; i++) sumSq += re[i] * re[i] + im[i] * im[i];
  return { re, im, amp, phase: ph, sumSq: Math.max(sumSq, 1e-12), healthy };
}

/* ---------------------------------------------------- pattern / gain */
function elementGain0(arr, fGHz) { const l = wavelength(fGHz); return 4 * Math.PI * arr.cellArea / (l * l); }

/** 方向增益(线性)：G = Ge(θ)·|AF|²/Σ|w|²，Ge=4πA_cell/λ²·cos^1.2θ。均匀口径峰值 = 4πA/λ² */
function gainEval(arr, w, fGHz, az, el) {
  const u = Math.sin(az * DEG) * Math.cos(el * DEG), v = Math.sin(el * DEG);
  const cosT = Math.cos(az * DEG) * Math.cos(el * DEG);
  if (cosT <= 0.001) return 1e-12;
  const k = 2 * Math.PI * fGHz * 1e9 / C;
  let re = 0, im = 0;
  for (let i = 0; i < arr.N; i++) {
    const a = w.re[i], b = w.im[i]; if (a === 0 && b === 0) continue;
    const p = k * (arr.x[i] * u + arr.y[i] * v), c = Math.cos(p), s = Math.sin(p);
    re += a * c - b * s; im += a * s + b * c;
  }
  return elementGain0(arr, fGHz) * Math.pow(cosT, 1.2) * (re * re + im * im) / w.sumSq;
}

/** 四象限单脉冲和差复响应与鉴角误差比：Σ = Q1+Q2+Q3+Q4, Δ_Az = (Q1+Q4)-(Q2+Q3), Δ_El = (Q1+Q2)-(Q3+Q4) */
function monopulseEval(arr, w, fGHz, az, el) {
  const u = Math.sin(az * DEG) * Math.cos(el * DEG), v = Math.sin(el * DEG);
  const k = 2 * Math.PI * fGHz * 1e9 / C;
  const qr = [0, 0, 0, 0], qi = [0, 0, 0, 0];
  for (let i = 0; i < arr.N; i++) {
    const a = w.re[i], b = w.im[i]; if (a === 0 && b === 0) continue;
    const p = k * (arr.x[i] * u + arr.y[i] * v), c = Math.cos(p), s = Math.sin(p);
    const q = arr.quad[i] - 1;
    qr[q] += a * c - b * s; qi[q] += a * s + b * c;
  }
  const sr = qr[0] + qr[1] + qr[2] + qr[3], si = qi[0] + qi[1] + qi[2] + qi[3];
  const dar = (qr[0] + qr[3]) - (qr[1] + qr[2]), dai = (qi[0] + qi[3]) - (qi[1] + qi[2]);
  const der = (qr[0] + qr[1]) - (qr[2] + qr[3]), dei = (qi[0] + qi[1]) - (qi[2] + qi[3]);
  const sMag2 = Math.max(sr * sr + si * si, 1e-18);
  // Im{Δ/Σ} 因阵列空间相位 e^{jkx(u-u0)} 对奇数对称分布表现为虚部奇函数
  const errAz = (dai * sr - dar * si) / sMag2;
  const errEl = (dei * sr - der * si) / sMag2;
  return { sumMag2: sMag2, diffAzMag2: dar * dar + dai * dai, diffElMag2: der * der + dei * dei, errAz, errEl };
}

function powerCutLine(arr, w, fGHz, u0, v0, du, dv, n) {
  const k = 2 * Math.PI * fGHz * 1e9 / C, re = new Float64Array(n), im = new Float64Array(n);
  for (let i = 0; i < arr.N; i++) {
    const a = w.re[i], b = w.im[i]; if (a === 0 && b === 0) continue;
    const p0 = k * (arr.x[i] * u0 + arr.y[i] * v0), p1 = k * (arr.x[i] * du + arr.y[i] * dv);
    const c0 = Math.cos(p0), s0 = Math.sin(p0);
    let pr = a * c0 - b * s0, pi = a * s0 + b * c0;
    const cr = Math.cos(p1), ci = Math.sin(p1);
    for (let j = 0; j < n; j++) { re[j] += pr; im[j] += pi; const t = pr * cr - pi * ci; pi = pr * ci + pi * cr; pr = t; }
  }
  const ge0 = elementGain0(arr, fGHz), g = new Float64Array(n);
  for (let j = 0; j < n; j++) {
    const u = u0 + j * du, v = v0 + j * dv, c2 = 1 - u * u - v * v, cosT = c2 > 0 ? Math.sqrt(c2) : 0;
    g[j] = cosT > 0.001 ? ge0 * Math.pow(cosT, 1.2) * (re[j] * re[j] + im[j] * im[j]) / w.sumSq : 1e-12;
  }
  return g;
}
/** 方位切面(固定仰角)，u 等间隔 → 返回角度与线性增益 */
function cutAz(arr, w, fGHz, el, azMin, azMax, n) {
  const ce = Math.cos(el * DEG), u0 = Math.sin(azMin * DEG) * ce, u1 = Math.sin(azMax * DEG) * ce, du = (u1 - u0) / (n - 1);
  const g = powerCutLine(arr, w, fGHz, u0, Math.sin(el * DEG), du, 0, n), ang = new Float64Array(n);
  for (let j = 0; j < n; j++) ang[j] = Math.asin(clamp((u0 + j * du) / ce, -1, 1)) * RAD;
  return { ang, g };
}
/** 俯仰切面(固定 u = 波束 u0) */
function cutEl(arr, w, fGHz, u, elMin, elMax, n) {
  const v0 = Math.sin(elMin * DEG), v1 = Math.sin(elMax * DEG), dv = (v1 - v0) / (n - 1);
  const g = powerCutLine(arr, w, fGHz, u, v0, 0, dv, n), ang = new Float64Array(n);
  for (let j = 0; j < n; j++) ang[j] = Math.asin(clamp(v0 + j * dv, -1, 1)) * RAD;
  return { ang, g };
}
function patternMap(arr, w, fGHz, u0, du, nu, v0, dv, nv) {
  const out = new Float64Array(nu * nv);
  for (let j = 0; j < nv; j++) out.set(powerCutLine(arr, w, fGHz, u0, v0 + j * dv, du, 0, nu), j * nu);
  return out;
}
/** 分析切面：峰值、3dB 波束宽度、第一零点、最大副瓣(相对峰值 dB) */
function analyzeCut(ang, g) {
  const n = ang.length; let pk = 0;
  for (let i = 1; i < n; i++) if (g[i] > g[pk]) pk = i;
  const rel = new Float64Array(n);
  for (let i = 0; i < n; i++) rel[i] = dB(g[i] / g[pk] + 1e-30);
  let l = pk; while (l > 0 && rel[l] > -3) l--;
  let r = pk; while (r < n - 1 && rel[r] > -3) r++;
  const lAng = l < pk ? ang[l] + (ang[l + 1] - ang[l]) * (-3 - rel[l]) / (rel[l + 1] - rel[l]) : ang[0];
  const rAng = r > pk ? ang[r - 1] + (ang[r] - ang[r - 1]) * (-3 - rel[r - 1]) / (rel[r] - rel[r - 1]) : ang[n - 1];
  let rn = pk; while (rn < n - 1 && rel[rn + 1] <= rel[rn]) rn++;
  let ln = pk; while (ln > 0 && rel[ln - 1] <= rel[ln]) ln--;
  let sll = -200;
  for (let i = rn; i < n; i++) sll = Math.max(sll, rel[i]);
  for (let i = 0; i <= ln; i++) sll = Math.max(sll, rel[i]);
  return { peakIdx: pk, peakAng: ang[pk], peakDb: dB(g[pk]), hpbw: rAng - lAng, sll, rel, firstNullR: ang[rn], firstNullL: ang[ln] };
}

/* ---------------------------------------------------- waveforms */
const COSTAS_ROOTS = [2, 6, 7, 8];
function welchCostas(idx, p, g) {
  p = p || 11;
  const k = idx || 0;
  g = g || COSTAS_ROOTS[k % COSTAS_ROOTS.length];
  const a = []; let v = 1;
  for (let i = 0; i < p - 1; i++) { a.push(v); v = (v * g) % p; }
  if (k >= COSTAS_ROOTS.length) a.reverse();
  return a;
}
function isCostas(perm) {
  const n = perm.length;
  for (let d = 1; d < n; d++) {
    const seen = new Set();
    for (let i = 0; i + d < n; i++) { const df = perm[i + d] - perm[i]; if (seen.has(df)) return false; seen.add(df); }
  }
  return new Set(perm).size === n;
}
const BARKER13 = [1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1];

function makeWaveform(p) {
  const tau = p.tauUs * 1e-6, type = p.type || 'lfm';
  let fs, n, B, re, im;
  if (type === 'barker') {
    const spc = 8; n = 13 * spc; fs = n / tau; B = 13 / tau; re = new Float64Array(n); im = new Float64Array(n);
    for (let i = 0; i < n; i++) re[i] = BARKER13[Math.floor(i / spc)];
  } else {
    B = p.bwMHz * 1e6; fs = Math.max(3 * B, 4 / tau); n = Math.round(tau * fs);
    if (n > 16384) { n = 16384; fs = n / tau; }
    re = new Float64Array(n); im = new Float64Array(n);
    if (type === 'costas') {
      const cIdx = p.costasShift || 0;
      const perm = welchCostas(cIdx), M = perm.length, tSub = tau / M, df = B / M;
      const muSub = ((cIdx % 2 === 0 ? 1 : -1) * df) / tSub;
      for (let i = 0; i < n; i++) {
        const t = i / fs, m = Math.min(M - 1, Math.floor(t / tSub)), tl = t - m * tSub;
        const f = (perm[m] - (M + 1) / 2) * df;
        const ph = 2 * Math.PI * f * tl + Math.PI * muSub * tl * tl;
        re[i] = Math.cos(ph); im[i] = Math.sin(ph);
      }
    } else {
      const mu = B / tau;
      for (let i = 0; i < n; i++) { const t = i / fs - tau / 2, ph = Math.PI * mu * t * t; re[i] = Math.cos(ph); im[i] = Math.sin(ph); }
    }
  }
  return { type, tau, B, fs, n, re, im };
}
function windowFn(name, n) {
  const w = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const x = n > 1 ? i / (n - 1) : 0;
    w[i] = name === 'hamming' ? 0.54 - 0.46 * Math.cos(2 * Math.PI * x)
      : name === 'blackman' ? 0.42 - 0.5 * Math.cos(2 * Math.PI * x) + 0.08 * Math.cos(4 * Math.PI * x) : 1;
  }
  return w;
}
/** 匹配滤波(脉压)输出核：y[m]=Σ s[n+m]·conj(h[n])，h=窗·s。返回以峰值归一的包络、PSLR、分辨率、失配损耗 */
function matchedFilter(wf, win) {
  const n = wf.n, nf = nextPow2(2 * n), w = windowFn(wf.type === 'lfm' ? (win || 'none') : 'none', n);
  const ar = new Float64Array(nf), ai = new Float64Array(nf), hr = new Float64Array(nf), hi = new Float64Array(nf);
  let Es = 0, Eh = 0;
  for (let i = 0; i < n; i++) { ar[i] = wf.re[i]; ai[i] = wf.im[i]; hr[i] = wf.re[i] * w[i]; hi[i] = wf.im[i] * w[i]; Es += wf.re[i] ** 2 + wf.im[i] ** 2; Eh += hr[i] ** 2 + hi[i] ** 2; }
  fft(ar, ai, false); fft(hr, hi, false);
  for (let i = 0; i < nf; i++) { const r = ar[i] * hr[i] + ai[i] * hi[i], im = ai[i] * hr[i] - ar[i] * hi[i]; ar[i] = r; ai[i] = im; }
  fft(ar, ai, true);
  const L = 2 * n - 1, mag = new Float64Array(L);
  for (let m = -(n - 1); m <= n - 1; m++) { const idx = (m + nf) % nf; mag[m + n - 1] = Math.hypot(ar[idx], ai[idx]); }
  const pkIdx = n - 1, pk = mag[pkIdx];
  let peak = 0; for (let i = 0; i < L; i++) if (mag[i] > peak) peak = mag[i];
  const env = new Float64Array(L); for (let i = 0; i < L; i++) env[i] = mag[i] / peak;
  const pi = Array.prototype.indexOf.call(mag, peak);
  let r = pi; while (r < L - 1 && env[r] > Math.SQRT1_2) r++;
  let l = pi; while (l > 0 && env[l] > Math.SQRT1_2) l--;
  const wR = (r - 1) + (env[r - 1] - Math.SQRT1_2) / (env[r - 1] - env[r]);
  const wL = (l + 1) - (env[l + 1] - Math.SQRT1_2) / (env[l + 1] - env[l]);
  let rn = pi; while (rn < L - 1 && env[rn + 1] <= env[rn]) rn++;
  let ln = pi; while (ln > 0 && env[ln - 1] <= env[ln]) ln--;
  let sl = 0; for (let i = rn; i < L; i++) sl = Math.max(sl, env[i]); for (let i = 0; i <= ln; i++) sl = Math.max(sl, env[i]);
  const width3 = (wR - wL) / wf.fs; // 1/√2 幅度点 = -3dB(功率)
  return {
    env, centerIdx: pi, dt: 1 / wf.fs, pslrDb: dB(sl * sl + 1e-30), width3dBs: width3,
    rangeRes: C / 2 * width3, mismatchLossDb: -dB(pk * pk / (Es * Eh)), compression: wf.tau * wf.B,
    window: wf.type === 'lfm' ? (win || 'none') : 'none'
  };
}
/** DRFM 重放(上一脉冲波形)经当前脉冲匹配滤波器的峰值功率比 (1=完全匹配) */
function crossMatch(a, b) {
  const n = Math.max(a.n, b.n), nf = nextPow2(2 * n);
  const ar = new Float64Array(nf), ai = new Float64Array(nf), br = new Float64Array(nf), bi = new Float64Array(nf);
  let Ea = 0, Eb = 0;
  for (let i = 0; i < a.n; i++) { ar[i] = a.re[i]; ai[i] = a.im[i]; Ea += ar[i] ** 2 + ai[i] ** 2; }
  for (let i = 0; i < b.n; i++) { br[i] = b.re[i]; bi[i] = b.im[i]; Eb += br[i] ** 2 + bi[i] ** 2; }
  fft(ar, ai, false); fft(br, bi, false);
  for (let i = 0; i < nf; i++) { const r = br[i] * ar[i] + bi[i] * ai[i], im = bi[i] * ar[i] - br[i] * ai[i]; ar[i] = r; ai[i] = im; }
  fft(ar, ai, true);
  let mx = 0; for (let i = 0; i < nf; i++) mx = Math.max(mx, ar[i] ** 2 + ai[i] ** 2);
  return mx / (Ea * Eb);
}

/* ------------------------------------------------- propagation */
function gasLossTwoWayDb(Rkm, fGHz, h1m, h2m) {
  const steps = 24, dr = Rkm / steps;
  const gO = 0.0072 * (1 + 0.015 * (fGHz - 10)), gW = 0.0085 * Math.pow(fGHz / 10, 2);
  let s = 0;
  for (let i = 0; i < steps; i++) {
    const h = Math.max(0, (h1m + (h2m - h1m) * (i + 0.5) / steps) / 1000);
    s += (gO * Math.exp(-h / 6.0) + gW * Math.exp(-h / 2.1)) * dr;
  }
  return 2 * s;
}
function rainCoeff(fGHz) {
  const P = [[8, 0.0045, 1.32], [10, 0.0101, 1.276], [12, 0.0188, 1.217]];
  const f = clamp(fGHz, 8, 12), i = f <= 10 ? 0 : 1, t = (f - P[i][0]) / (P[i + 1][0] - P[i][0]);
  return { k: Math.exp(Math.log(P[i][1]) + t * (Math.log(P[i + 1][1]) - Math.log(P[i][1]))), alpha: P[i][2] + t * (P[i + 1][2] - P[i][2]) };
}
/** ITU-R P.838 幂律双程降雨衰减；雨区限制：水平范围 extentKm、高度 < rainTopKm */
function rainLossTwoWayDb(Rkm, fGHz, rainMmH, extentKm, h1m, h2m, rainTopKm) {
  if (!(rainMmH > 0)) return 0;
  const top = (rainTopKm != null ? rainTopKm : 10) * 1000;
  let frac = 0; const steps = 24;
  for (let i = 0; i < steps; i++) if (Math.max(0, h1m + (h2m - h1m) * (i + 0.5) / steps) < top) frac++;
  frac /= steps;
  const Reff = Math.min(Rkm * frac, extentKm == null ? 1e9 : extentKm);
  const c = rainCoeff(fGHz);
  return 2 * c.k * Math.pow(rainMmH, c.alpha) * Reff;
}
function stcLossDb(Rkm, stcKm, nu) { return Rkm >= stcKm ? 0 : -nu * 10 * Math.log10(Math.max(Rkm, 0.5) / stcKm); }
function eclipseLossFraction(Rm, tau, trec, pri) {
  const t = (2 * Rm / C) % pri, txEnd = tau + trec;
  const ov = (a0, a1, b0, b1) => Math.max(0, Math.min(a1, b1) - Math.max(a0, b0));
  return Math.min(1, (ov(t, t + tau, 0, txEnd) + ov(t, t + tau, pri, pri + txEnd)) / tau);
}
function foldDoppler(fd, prf) { let x = fd % prf; if (x > prf / 2) x -= prf; if (x < -prf / 2) x += prf; return x; }
const horizonKm = (h1, h2) => 4.12 * (Math.sqrt(Math.max(0, h1)) + Math.sqrt(Math.max(0, h2)));

/** 相参脉冲雷达方程(能量形式)：SNR = Pt·τ·N·Gt·Gr·λ²·σ / ((4π)³R⁴·kT0·F·L)  (脉压增益已含于 Pt·τ) */
function radarEquation(p) {
  const lam = wavelength(p.fGHz);
  const num = p.PtW * p.tauS * p.nCoh * p.GtLin * p.GrLin * lam * lam * p.rcs;
  const den = Math.pow(4 * Math.PI, 3) * Math.pow(p.Rm, 4) * KB * T0 * unDb(p.nfDb);
  const loss = p.lossDb + (p.atmDb || 0) + (p.rainDb || 0) + (p.stcDb || 0) + (p.eclipseDb || 0) + (p.extraDb || 0);
  return {
    snrIdealDb: dB(num / den), snrDb: dB(num / den) - loss, lossTotalDb: loss,
    terms: {
      PtDb: dB(p.PtW), tauNDb: dB(p.tauS * p.nCoh), GtDb: dB(p.GtLin), GrDb: dB(p.GrLin), lam2Db: dB(lam * lam), sigmaDb: dB(p.rcs),
      fourPi3Db: dB(Math.pow(4 * Math.PI, 3)), R4Db: 40 * Math.log10(p.Rm), kT0Db: dB(KB * T0), nfDb: p.nfDb,
      lossDb: p.lossDb, atmDb: p.atmDb || 0, rainDb: p.rainDb || 0, stcDb: p.stcDb || 0, eclipseDb: p.eclipseDb || 0, extraDb: p.extraDb || 0
    }
  };
}

/* ------------------------------------------- detection statistics */
function i0e(z) {
  z = Math.abs(z);
  if (z < 3.75) { const t = (z / 3.75) ** 2; return (1 + t * (3.5156229 + t * (3.0899424 + t * (1.2067492 + t * (0.2659732 + t * (0.0360768 + t * 0.0045813)))))) * Math.exp(-z); }
  const t = 3.75 / z;
  return (0.39894228 + t * (0.01328592 + t * (0.00225319 + t * (-0.00157565 + t * (0.00916281 + t * (-0.02057706 + t * (0.02635537 + t * (-0.01647633 + t * 0.00392377)))))))) / Math.sqrt(z);
}
/** 非起伏目标固定门限 Pd (平方律，噪声功率=1，信号功率=S)：Marcum Q 数值积分 */
function pdFixedThreshold(S, T) {
  if (S <= 1e-12) return Math.exp(-T);
  const sq = Math.sqrt(S), xmax = Math.max((sq + 8) ** 2, T + 20);
  if (T >= xmax) return 0;
  const n = 4000, h = (xmax - T) / n;
  const f = x => Math.exp(-((Math.sqrt(x) - sq) ** 2)) * i0e(2 * sq * Math.sqrt(x));
  let s = f(T) + f(xmax);
  for (let i = 1; i < n; i++) s += f(T + i * h) * (i % 2 ? 4 : 2);
  return clamp(s * h / 3, 0, 1);
}
const cfarAlpha = (n, pf) => isFinite(n) ? n * (Math.pow(pf, -1 / n) - 1) : -Math.log(pf);
const cfarLossDb = (n, pf) => dB(cfarAlpha(n, pf) / -Math.log(pf));
function albersheimDb(pfa, pd, N) {
  N = N || 1; const A = Math.log(0.62 / pfa), B = Math.log(pd / (1 - pd));
  return -5 * Math.log10(N) + (6.2 + 4.54 / Math.sqrt(N + 0.44)) * Math.log10(A + 0.12 * A * B + 1.7 * B);
}
function pdBurst(S, pf, nRef, swerling) {
  if (S <= 0) return pf;
  if (swerling === 1 || swerling === 2) {
    if (!isFinite(nRef)) return Math.pow(pf, 1 / (1 + S));
    return Math.pow(1 + cfarAlpha(nRef, pf) / (nRef * (1 + S)), -nRef);
  }
  return pdFixedThreshold(S / unDb(cfarLossDb(nRef, pf)), -Math.log(pf));
}
function requiredSnrDb(pf, pd, nRef, swerling) {
  let lo = -20, hi = 70;
  for (let i = 0; i < 50; i++) { const m = (lo + hi) / 2; if (pdBurst(unDb(m), pf, nRef, swerling) < pd) lo = m; else hi = m; }
  return (lo + hi) / 2;
}
function binomTail(N, M, p) {
  let s = 0, c = 1;
  for (let k = 0; k <= N; k++) { if (k > 0) c = c * (N - k + 1) / k; if (k >= M) s += c * Math.pow(p, k) * Math.pow(1 - p, N - k); }
  return s;
}
function pbTail(ps, M) {
  const n = ps.length, dp = new Float64Array(n + 1); dp[0] = 1;
  for (const p of ps) for (let k = n; k >= 0; k--) dp[k] = dp[k] * (1 - p) + (k > 0 ? dp[k - 1] * p : 0);
  let s = 0; for (let k = M; k <= n; k++) s += dp[k]; return s;
}
function solvePerBurstPfa(N, M, pfaSys) {
  if (N <= 1) return pfaSys;
  let lo = Math.log(1e-16), hi = Math.log(0.5);
  for (let i = 0; i < 80; i++) { const m = (lo + hi) / 2; if (binomTail(N, M, Math.exp(m)) > pfaSys) hi = m; else lo = m; }
  return Math.exp((lo + hi) / 2);
}
/** 蒙特卡洛单元检测：测试单元 |√S+n|²  vs  α·(n_ref 个参考单元均值)；噪声功率放大 noiseScale(干扰/杂波) */
function mcDetect(rng, S, noiseScale, alpha, nRef) {
  const sd = Math.sqrt(noiseScale / 2), re = Math.sqrt(S) + sd * rng.gauss(), im = sd * rng.gauss();
  let ref;
  if (!isFinite(nRef)) ref = noiseScale;
  else { ref = 0; for (let i = 0; i < nRef; i++) ref += rng.exp(); ref = ref * noiseScale / nRef; }
  return re * re + im * im > alpha * ref;
}
/** 多 PRF 距离解模糊 (中国剩余定理的搜索实现) */
function resolveRange(apps, tol, rMax) {
  const Ru = f => C / (2 * f);
  if (apps.length === 1) return { R: apps[0].r, ok: true, nCand: 1, single: true };
  const a0 = apps[0], c = [];
  for (let k = 0; a0.r + k * Ru(a0.prf) <= rMax; k++) {
    const Rc = a0.r + k * Ru(a0.prf); let resid = 0, ok = true;
    for (let j = 1; j < apps.length; j++) {
      const ru = Ru(apps[j].prf); let d = Math.abs((Rc % ru) - apps[j].r); d = Math.min(d, ru - d);
      if (d > tol) { ok = false; break; } resid += d * d;
    }
    if (ok) c.push({ R: Rc, resid });
  }
  if (!c.length) return { R: NaN, ok: false, nCand: 0 };
  c.sort((a, b) => a.resid - b.resid);
  // 多个 CRT 候选时优先最近距离(首个模糊区间)；远处目标由 amb 标记暴露
  const rmin = c[0].resid, near = c.filter(q => q.resid <= rmin + tol * tol * 0.01);
  near.sort((a, b) => a.R - b.R);
  return { R: near[0].R, ok: true, nCand: c.length, amb: c.length > 1 };
}
/** DPCA：双子孔径时延相消，动目标响应 4sin²(2π v Δt/λ)，Δt = d_sub/(2Va) */
function dpcaResponse(vGroundRadial, lam, dSub, Va) {
  const dphi = 4 * Math.PI * vGroundRadial * (dSub / (2 * Va)) / lam;
  return 2 * (1 - Math.cos(dphi));
}
function sarResolution(o) { return { rangeRes: C / (2 * o.bwHz), azRes: o.lambda * o.R0 / (2 * o.Va * o.Tsar) }; }

/* ------------------------------------------------- terrain / GM */
function hash2(ix, iy) { let h = (Math.imul(ix, 374761393) + Math.imul(iy, 668265263)) | 0; h = Math.imul(h ^ (h >>> 13), 1274126177); return ((h ^ (h >>> 16)) >>> 0) / 4294967296; }
function vnoise(x, y) {
  const ix = Math.floor(x), iy = Math.floor(y), fx = x - ix, fy = y - iy, s = t => t * t * (3 - 2 * t);
  const a = hash2(ix, iy), b = hash2(ix + 1, iy), c = hash2(ix, iy + 1), d = hash2(ix + 1, iy + 1);
  return a + (b - a) * s(fx) + (c - a) * s(fy) + (a - b - c + d) * s(fx) * s(fy);
}
function terrainSigma0Db(xm, ym) {
  const n = vnoise(xm / 9000, ym / 9000) * 0.65 + vnoise(xm / 2500, ym / 2500) * 0.35;
  let s = n < 0.30 ? -30 : n < 0.55 ? -18 : n < 0.75 ? -13 : -9;
  if (Math.abs(xm - 6000 + 9000 * Math.sin(ym / 25000)) < 600) s = -32;
  if (vnoise(xm / 3500 + 50, ym / 3500 + 50) > 0.80) s = -4;
  return s + 3 * (vnoise(xm / 250, ym / 250) - 0.5);
}

/* --------------------------------------------- presets / defaults */
const PRF_SETS = {
  lprf: [2.5], mprf1: [15.0],
  mprf3: [11.0, 13.0, 17.0],
  mprf8: [11.0, 12.7, 14.3, 17.0],
  hprf3: [120.0, 133.0, 145.0], acm: [8.1]
};
function defaultConfig() {
  return {
    seed: 1, mode: 'RWS', timeScale: 1, fGHz: 10, rangeMaxKm: 160,
    array: { diameter: 0.70, grid: 'triangular', spacingLambda: 0.61, trPeakW: 10, failRate: 0, phaseBits: 6, ampBits: 0, rxTaper: 'lowsll', taperN: 2.2, pedestal: 0.05, spoilBeta: 5.0 },
    tx: { waveform: 'lfm', tauUs: 18, bwMHz: 20, window: 'hamming', prfRegime: 'mprf3', customPrf: [15], nCpi: 48, mOfN: 2, dutyMax: 0.35, trecUs: 2 },
    rx: { nfDb: 3.2, lossDb: 3.8, pfa: 1e-9, nRef: 32, stc: true, stcRangeKm: 15 },
    scan: { azCenter: 0, azHalf: 60, elCenter: 0, bars: 4, barSpacingDeg: 0, stepFactor: 0.8 },
    own: { alt: 8000, speed: 250 },
    env: { rainMmH: 0, rainExtentKm: 40, rainTopKm: 10, clutterGammaDb: -15, clutterOn: true },
    eccm: { adbf: false, agility: false, agLowGHz: 8, agHighGHz: 12, dpca: false, leadingEdge: false, slb: false },
    track: { revisitS: 1.0 }, acmSub: 'hud'
  };
}
const MODE_PRESETS = {
  RWS: { tracks: false, rxTaper: 'lowsll', tx: { waveform: 'lfm', window: 'hamming', tauUs: 18, bwMHz: 20, prfRegime: 'mprf3', nCpi: 48, mOfN: 2 }, scan: { azHalf: 60, bars: 4, elCenter: 0 } },
  TWS: { tracks: true, rxTaper: 'lowsll', tx: { waveform: 'lfm', window: 'hamming', tauUs: 15, bwMHz: 20, prfRegime: 'mprf3', nCpi: 32, mOfN: 2 }, scan: { azHalf: 30, bars: 2, elCenter: 0 } },
  TAS: { tracks: true, rxTaper: 'lowsll', tx: { waveform: 'lfm', window: 'hamming', tauUs: 18, bwMHz: 20, prfRegime: 'mprf3', nCpi: 36, mOfN: 2 }, scan: { azHalf: 60, bars: 2, elCenter: 0 } },
  MTT: { tracks: true, rxTaper: 'lowsll', tx: { waveform: 'lfm', window: 'hamming', tauUs: 12, bwMHz: 50, prfRegime: 'mprf3', nCpi: 48, mOfN: 2 }, scan: { azHalf: 60, bars: 2, elCenter: 0 } },
  DTT: { tracks: true, rxTaper: 'lowsll', tx: { waveform: 'lfm', window: 'hamming', tauUs: 12, bwMHz: 50, prfRegime: 'mprf3', nCpi: 48, mOfN: 2 }, scan: { azHalf: 60, bars: 2, elCenter: 0 } },
  STT: { tracks: true, rxTaper: 'lowsll', tx: { waveform: 'lfm', window: 'hamming', tauUs: 15, bwMHz: 50, prfRegime: 'mprf3', nCpi: 64, mOfN: 2 }, scan: { azHalf: 60, bars: 2, elCenter: 0 } },
  ACM: { tracks: true, rxTaper: 'spoil', tx: { waveform: 'barker', window: 'none', tauUs: 5, bwMHz: 20, prfRegime: 'acm', nCpi: 64, mOfN: 1 }, scan: { azHalf: 10, bars: 4, elCenter: 0 } },
  GM: { tracks: false, rxTaper: 'lowsll', tx: { waveform: 'lfm', window: 'hamming', tauUs: 20, bwMHz: 50, prfRegime: 'lprf', nCpi: 16, mOfN: 1 }, scan: { azHalf: 60, bars: 1, elCenter: -8 } },
  SAR: { tracks: false, rxTaper: 'uniform', tx: { waveform: 'lfm', window: 'hamming', tauUs: 30, bwMHz: 300, prfRegime: 'lprf', nCpi: 16, mOfN: 1 }, scan: { azHalf: 5, bars: 1, elCenter: -10 } },
  GMTI: { tracks: true, rxTaper: 'lowsll', tx: { waveform: 'lfm', window: 'hamming', tauUs: 15, bwMHz: 20, prfRegime: 'mprf3', nCpi: 48, mOfN: 2 }, scan: { azHalf: 40, bars: 2, elCenter: -8 }, eccm: { dpca: true } }
};
const ACM_SUBMODES = { boresight: { azHalf: 0, bars: 1, elCenter: 0 }, vertical: { azHalf: 0, bars: 8, elCenter: 0, barSpacingDeg: 6 }, hud: { azHalf: 8, bars: 4, elCenter: 0 }, slew: { azHalf: 4, bars: 2, elCenter: 0 } };

/* ---------------------------------------------------- scenario presets */
const SCENARIOS = {
  default: {
    name: '多批次空地综合态势', seed: 1, mode: 'RWS',
    targets: [
      { name: 'Fighter-1', rangeKm: 78, azDeg: -18, altM: 8500, speed: 280, headingDeg: 175, rcs: 4.5, swerling: 1 },
      { name: 'Fighter-2', rangeKm: 62, azDeg: 14, altM: 9000, speed: 320, headingDeg: 190, rcs: 2.5, swerling: 1 },
      { name: 'Transport', rangeKm: 110, azDeg: 22, altM: 9500, speed: 180, headingDeg: 200, rcs: 35, swerling: 2 }
    ]
  },
  stealth: {
    name: '远距 VLO 隐身目标突防', seed: 7, mode: 'RWS',
    config: { scan: { azCenter: 5, azHalf: 10, bars: 1, elCenter: 0 }, tx: { tauUs: 6, bwMHz: 20, nCpi: 24 } },
    targets: [{ name: 'VLO-UCAV', rangeKm: 75, azDeg: 5, altM: 8500, speed: 320, headingDeg: 180, rcs: 0.015, swerling: 0 }]
  },
  jam_adbf: {
    name: '强噪声压制支援干扰', seed: 11, mode: 'RWS',
    config: { scan: { azCenter: 10, azHalf: 20, bars: 2, elCenter: 0 } },
    targets: [{ name: 'Fighter', rangeKm: 70, azDeg: 10, altM: 8000, speed: 280, headingDeg: 180, rcs: 3.0, swerling: 1 }],
    jammers: [{ type: 'spot', azDeg: -22, elDeg: 0, rangeKm: 90, erpDbW: 75, fGHz: 10, bwMHz: 25 }]
  },
  drfm_rgpo: {
    name: '自卫 DRFM 距离拖引假目标', seed: 23, mode: 'RWS',
    targets: [{ name: 'Strike', rangeKm: 68.5, azDeg: 0, altM: 8000, speed: 260, headingDeg: 180, rcs: 4.0, swerling: 1 }],
    jammers: [{ type: 'drfm', azDeg: 0, elDeg: 0, rangeKm: 68.5, erpDbW: 60, fGHz: 10, bwMHz: 20, targetId: 'T1', rgpoRate: 150, rgpoMaxKm: 6 }]
  },
  gmti_convoy: {
    name: '下视强地杂波 + 地面装甲车队', seed: 19, mode: 'GMTI',
    config: { eccm: { dpca: false }, scan: { azCenter: 0, azHalf: 20, bars: 1, elCenter: -13 } },
    targets: [
      { name: 'Convoy', rangeKm: 35, azDeg: 0, altM: 100, speed: 18, headingDeg: 0, rcs: 25, isGround: true, swerling: 1 },
      { name: 'Truck', rangeKm: 42, azDeg: 12, altM: 90, speed: 14, headingDeg: 20, rcs: 12, isGround: true, swerling: 1 }
    ]
  },
  acm_dogfight: {
    name: '近距大离轴角高G格斗', seed: 31, mode: 'ACM',
    config: { scan: { azCenter: 8, azHalf: 10, bars: 4, elCenter: 0 } },
    targets: [{ name: 'Bandit', rangeKm: 9, azDeg: 8, altM: 7500, speed: 300, headingDeg: 210, rcs: 4.0, swerling: 1 }]
  },
  rain_xband: {
    name: '强降雨衰减与全 X 波段频点优选', seed: 37, mode: 'RWS',
    config: { fGHz: 12, env: { rainMmH: 25, rainExtentKm: 40, rainTopKm: 10 }, scan: { azCenter: 0, azHalf: 20, bars: 1, elCenter: 0 } },
    targets: [{ name: 'RainTarget', rangeKm: 65, azDeg: 0, altM: 8000, speed: 260, headingDeg: 180, rcs: 5.0, swerling: 1 }]
  }
};
function createScenario(key) {
  const sc = SCENARIOS[key] || SCENARIOS.default;
  const sim = new RadarSim({ seed: sc.seed });
  if (sc.mode) sim.applyMode(sc.mode);
  if (sc.config) sim.setConfig(sc.config);
  (sc.targets || []).forEach(t => sim.addTarget(t));
  (sc.jammers || []).forEach(j => sim.addJammer(j));
  return sim;
}

function deepMerge(t, s) {
  for (const k in s) {
    if (s[k] && typeof s[k] === 'object' && !Array.isArray(s[k])) { if (!t[k] || typeof t[k] !== 'object') t[k] = {}; deepMerge(t[k], s[k]); }
    else t[k] = Array.isArray(s[k]) ? s[k].slice() : s[k];
  }
  return t;
}

/* ====================================================== RadarSim */
class RadarSim {
  constructor(cfg) {
    this.cfg = deepMerge(defaultConfig(), cfg || {});
    this.rng = makeRng(this.cfg.seed);
    this.time = 0; this.targets = []; this.jammers = []; this.plots = []; this.tracks = [];
    this.tid = 0; this.trid = 0; this.events = []; this.targetStats = {};
    this.scanIdx = 0; this.frames = 0; this.frameStartT = 0; this.frameTimeS = 0; this.lastDwell = null;
    this.lockId = null; this.lockIds = []; this.rr = 0; this.lastTrackDwell = {}; this.timeBudget = 0; this.slow = false;
    this._cache = {}; this.gm = null;
    this.rebuild();
  }

  setConfig(patch) { deepMerge(this.cfg, patch); this.rebuild(); }
  applyMode(mode) {
    const p = MODE_PRESETS[mode]; if (!p) return;
    this.cfg.mode = mode;
    this.cfg.array.rxTaper = p.rxTaper;
    deepMerge(this.cfg.tx, p.tx); deepMerge(this.cfg.scan, p.scan);
    this.cfg.scan.barSpacingDeg = 0;
    if (p.eccm) deepMerge(this.cfg.eccm, p.eccm);
    if (mode === 'ACM') deepMerge(this.cfg.scan, ACM_SUBMODES[this.cfg.acmSub] || {});
    if (mode === 'STT' || mode === 'DTT') { if (!this.lockId) this.autoLockStrongest(mode === 'DTT' ? 2 : 1); }
    this.rebuild();
  }

  /* ---- targets / jammers ---- */
  addTarget(s) {
    const tg = { id: s.id || ('T' + (++this.tid)) };
    this.targets.push(tg); this.placeTarget(tg, s); return tg;
  }
  placeTarget(tg, s) {
    const sp = Object.assign({ name: tg.id, rangeKm: 80, azDeg: 0, altM: 8000, speed: 250, headingDeg: 180, climb: 0, rcs: 5, swerling: 1, isGround: false }, tg.spec || {}, s);
    tg.spec = sp; tg.name = sp.name; tg.rcs = sp.rcs; tg.swerling = sp.swerling; tg.isGround = !!sp.isGround;
    const R = sp.rangeKm * 1000, dz = sp.altM - this.cfg.own.alt, gr = Math.sqrt(Math.max(0, R * R - dz * dz)), az = sp.azDeg * DEG;
    tg.x0 = gr * Math.sin(az); tg.y0 = gr * Math.cos(az); tg.z0 = sp.altM; tg.t0 = this.time;
    const h = sp.headingDeg * DEG; tg.vx = sp.speed * Math.sin(h); tg.vy = sp.speed * Math.cos(h); tg.vz = sp.climb;
  }
  updateTarget(id, patch) {
    const tg = this.targets.find(t => t.id === id);
    if (!tg) return null;
    const cur = this.targetState(tg, this.time);
    const hasPos = patch.rangeKm != null || patch.azDeg != null || patch.altM != null;
    if (!hasPos) {
      // 保持当前时刻的瞬时空间坐标，仅更新运动速度/航向/RCS/模型
      const sp = Object.assign({}, tg.spec, { rangeKm: cur.R / 1000, azDeg: cur.az, altM: cur.alt }, patch);
      tg.spec = sp; tg.name = sp.name; tg.rcs = sp.rcs; tg.swerling = sp.swerling; tg.isGround = !!sp.isGround;
      tg.x0 = cur.px; tg.y0 = cur.py; tg.z0 = cur.alt; tg.t0 = this.time;
      const h = sp.headingDeg * DEG; tg.vx = sp.speed * Math.sin(h); tg.vy = sp.speed * Math.cos(h); tg.vz = sp.climb;
    } else {
      this.placeTarget(tg, Object.assign({ rangeKm: cur.R / 1000, azDeg: cur.az, altM: cur.alt }, patch));
    }
    return tg;
  }
  removeTarget(id) { this.targets = this.targets.filter(t => t.id !== id); this.tracks = this.tracks.filter(tr => tr.truth !== id); }
  addJammer(s) {
    let n = this.jammers.length + 1;
    while (this.jammers.some(j => j.id === 'J' + n)) n++;
    const j = Object.assign({ id: 'J' + n, type: 'spot', azDeg: -22, elDeg: 0, rangeKm: 100, erpDbW: 40, fGHz: 10, bwMHz: 20, targetId: null, rgpoRate: 150, rgpoMaxKm: 6, pulseRate: 3 }, s);
    this.jammers.push(j); return j;
  }
  updateJammer(id, patch) { const j = this.jammers.find(q => q.id === id); if (j) Object.assign(j, patch); return j; }
  removeJammer(id) { this.jammers = this.jammers.filter(j => j.id !== id); }

  targetState(tg, t) {
    const dt = t - tg.t0, Va = this.cfg.own.speed, lam = wavelength(this.cfg.fGHz);
    const px = tg.x0 + tg.vx * dt, py = tg.y0 + tg.vy * dt - Va * dt, pz = tg.z0 + tg.vz * dt - this.cfg.own.alt;
    const R = Math.sqrt(px * px + py * py + pz * pz), gr = Math.hypot(px, py);
    const vrx = tg.vx, vry = tg.vy - Va, vrz = tg.vz;
    const closing = -(px * vrx + py * vry + pz * vrz) / Math.max(R, 1);
    const losx = px / R, losy = py / R, losz = pz / R;
    const vGround = tg.vx * losx + tg.vy * losy + tg.vz * losz;  // 目标相对地面的视线速度分量(远离为正)
    return {
      R, gr, az: Math.atan2(px, py) * RAD, el: Math.atan2(pz, gr) * RAD, closing, fd: 2 * closing / lam, vGround,
      alt: tg.z0 + tg.vz * dt, visible: gr / 1000 <= horizonKm(this.cfg.own.alt, Math.max(0, tg.z0 + tg.vz * dt)), px, py, pz
    };
  }

  /* ---- derived parameters ---- */
  rebuild() {
    const cfg = this.cfg, a = cfg.array, tx = cfg.tx;
    const akey = [a.diameter, a.grid, a.spacingLambda].join('|');
    if (this._akey !== akey) { this.array = buildArray({ diameter: a.diameter, spacingLambda: a.spacingLambda, grid: a.grid }); this._akey = akey; this._cache = {}; }
    const hkey = akey + '|' + a.failRate + '|' + cfg.seed;
    if (this._hkey !== hkey) { this.health = makeHealth(this.array.N, a.failRate, cfg.seed); this._hkey = hkey; }
    const d = this.derived = {};
    d.lambda = wavelength(cfg.fGHz);
    const prfs = tx.prfRegime === 'custom' ? tx.customPrf.slice() : (PRF_SETS[tx.prfRegime] || PRF_SETS.mprf8).slice();
    d.prfs = prfs.map(f => f * 1e3);
    d.nB = d.prfs.length; d.M = Math.min(tx.mOfN, d.nB);
    d.tau = tx.tauUs * 1e-6; d.trec = tx.trecUs * 1e-6;
    d.duty = d.tau * Math.max(...d.prfs);
    d.txInhibit = d.duty > tx.dutyMax ? 'duty' : (d.tau >= 0.9 / Math.max(...d.prfs) ? 'pulse>PRI' : null);
    d.dwellS = d.prfs.reduce((s, f) => s + tx.nCpi / f, 0) + 2e-4;
    d.healthy = 0; for (let i = 0; i < this.array.N; i++) d.healthy += this.health[i];
    d.PtW = d.healthy * a.trPeakW;
    d.avgPowerW = d.PtW * d.duty;
    // 波形与匹配滤波
    const wkey = [tx.waveform, tx.tauUs, tx.bwMHz, tx.window].join('|');
    if (this._wkey !== wkey) {
      this.wf = makeWaveform({ type: tx.waveform, tauUs: tx.tauUs, bwMHz: tx.bwMHz, costasShift: 0 });
      this.mf = matchedFilter(this.wf, tx.window); this._wkey = wkey; this._rho = null;
    }
    d.bwEff = this.wf.B; d.rangeRes = this.mf.rangeRes; d.mfLossDb = this.mf.mismatchLossDb;
    d.ruKm = d.prfs.map(f => C / (2 * f) / 1000);
    // 虚警控制
    d.pfaBurst = solvePerBurstPfa(d.nB, d.M, cfg.rx.pfa);
    d.alpha = cfarAlpha(cfg.rx.nRef, d.pfaBurst);
    d.nCells = Math.max(1, cfg.rangeMaxKm * 1000 / Math.max(d.rangeRes, 1)) * tx.nCpi;
    // 阵面切面(宽边波束)：HPBW 用于扫描步进与测角
    const ckey = [akey, hkey, cfg.fGHz, a.rxTaper, a.taperN, a.pedestal, a.spoilBeta, a.phaseBits, a.ampBits].join('|');
    if (this._ckey !== ckey) { this._cuts = this._computeCuts(); this._ckey = ckey; }
    Object.assign(d, this._cuts);
    // 扫描
    this.scanList = this._buildScan(); d.nPos = this.scanList.length; d.frameS = d.nPos * d.dwellS;
    if (this.scanIdx >= d.nPos) this.scanIdx = 0;
    d.trackLossS = Math.max(10, 2.5 * d.frameS);
    d.modeInfo = MODE_PRESETS[cfg.mode] || MODE_PRESETS.RWS;
    // DRFM 重放失配 (Costas 与其它随机码字)
    if (this._rho == null) this._rho = this._drfmMismatch();
    d.drfmRho = this._rho;
    return d;
  }
  _txTaper() { return this.cfg.array.rxTaper === 'spoil' ? 'spoil' : 'uniform'; }
  _wOpts(f, az, el, rx, nulls) {
    const a = this.cfg.array;
    return { fGHz: f, az, el, taper: rx ? a.rxTaper : this._txTaper(), taperN: a.taperN, pedestal: a.pedestal, spoilBeta: a.spoilBeta, phaseBits: a.phaseBits, ampBits: a.ampBits, health: this.health, nulls };
  }
  _computeCuts() {
    const f = this.cfg.fGHz, arr = this.array, wT = buildWeights(arr, this._wOpts(f, 0, 0, false)), wR = buildWeights(arr, this._wOpts(f, 0, 0, true));
    const n = 721, ca = [cutAz(arr, wT, f, 0, -30, 30, n), cutAz(arr, wR, f, 0, -30, 30, n)], ce = [cutEl(arr, wT, f, 0, -30, 30, n), cutEl(arr, wR, f, 0, -30, 30, n)];
    const tw = (c) => { const g = new Float64Array(n); for (let i = 0; i < n; i++) g[i] = c[0].g[i] * c[1].g[i]; return g; };
    const aA = analyzeCut(ca[1].ang, tw(ca)), aE = analyzeCut(ce[1].ang, tw(ce)), aR = analyzeCut(ca[1].ang, ca[1].g);
    return {
      g0Db: dB(gainEval(arr, wT, f, 0, 0)), grDb: dB(gainEval(arr, wR, f, 0, 0)),
      hpbwDeg: aA.hpbw, hpbwElDeg: aE.hpbw, hpbwRxDeg: aR.hpbw, sllRxDb: aR.sll,
      twoWayAz: { ang: ca[1].ang, db: Array.from(aA.rel) }, twoWayEl: { ang: ce[1].ang, db: Array.from(aE.rel) }
    };
  }
  /** 当前阵面/波束的实时方向图：方位切面、俯仰切面与可选二维 (u,v) 图 */
  beamPattern(az, el, o) {
    o = o || {};
    const cfg = this.cfg, arr = this.array, f = o.fGHz || cfg.fGHz;
    const nulls = o.nulls !== undefined ? o.nulls
      : (cfg.eccm.adbf ? this.jammers.filter(j => j.type !== 'drfm').map(j => ({ az: j.azDeg, el: j.elDeg })) : []);
    const wT = buildWeights(arr, this._wOpts(f, az, el, false, nulls));
    const wR = buildWeights(arr, this._wOpts(f, az, el, true, nulls));
    const n = o.points || 361, span = o.spanDeg || 40;
    const caT = cutAz(arr, wT, f, el, az - span / 2, az + span / 2, n);
    const caR = cutAz(arr, wR, f, el, az - span / 2, az + span / 2, n);
    const u0 = Math.sin(az * DEG) * Math.cos(el * DEG);
    const ceT = cutEl(arr, wT, f, u0, el - span / 2, el + span / 2, n);
    const ceR = cutEl(arr, wR, f, u0, el - span / 2, el + span / 2, n);
    const mk = (a, b) => { const g = new Float64Array(n); for (let i = 0; i < n; i++) g[i] = a[i] * b[i]; return g; };
    const aAz = analyzeCut(caT.ang, mk(caT.g, caR.g)), aEl = analyzeCut(ceT.ang, mk(ceT.g, ceR.g));
    const out = {
      fGHz: f, az, el,
      gTdb: dB(gainEval(arr, wT, f, az, el)), gRdb: dB(gainEval(arr, wR, f, az, el)),
      hpbwAz: aAz.hpbw, hpbwEl: aEl.hpbw, sllAz: aAz.sll, sllEl: aEl.sll,
      cutAz: { ang: caT.ang, txDb: caT.g.map(dB), rxDb: caR.g.map(dB), twoDb: mk(caT.g, caR.g).map(dB) },
      cutEl: { ang: ceT.ang, txDb: ceT.g.map(dB), rxDb: ceR.g.map(dB), twoDb: mk(ceT.g, ceR.g).map(dB) }
    };
    if (o.cut === '2d') {
      const nu = o.nu || 49, nv = o.nv || 49;
      const uMin = Math.sin((az - span / 2) * DEG) * Math.cos(el * DEG), uMax = Math.sin((az + span / 2) * DEG) * Math.cos(el * DEG);
      const vMin = Math.sin((el - span / 2) * DEG), vMax = Math.sin((el + span / 2) * DEG);
      const du = (uMax - uMin) / (nu - 1), dv = (vMax - vMin) / (nv - 1);
      const mT = patternMap(arr, wT, f, uMin, du, nu, vMin, dv, nv);
      const mR = patternMap(arr, wR, f, uMin, du, nu, vMin, dv, nv);
      const db = new Float64Array(nu * nv);
      for (let i = 0; i < db.length; i++) db[i] = dB(mT[i] * mR[i]);
      out.map = { uMin, vMin, du, dv, nu, nv, db };
    }
    return out;
  }

  /** 阵面激励分布快照：供阵面画布绘制幅度/相位/失效 */
  arrayMap(az, el, f) {
    const arr = this.array;
    const w = buildWeights(arr, this._wOpts(f || this.cfg.fGHz, az || 0, el || 0, true));
    const health = new Uint8Array(arr.N);
    for (let i = 0; i < arr.N; i++) health[i] = this.health ? this.health[i] : 1;
    return { N: arr.N, x: arr.x, y: arr.y, rho: arr.rho, quad: arr.quad, R: arr.R, amp: w.amp, phase: w.phase, health };
  }

  _drfmMismatch() {
    const tx = this.cfg.tx;
    if (tx.waveform !== 'costas') return { fixed: 1, hop: 1 };
    const a = makeWaveform({ type: 'costas', tauUs: tx.tauUs, bwMHz: tx.bwMHz, costasShift: 0 });
    let s = 0; for (let c = 1; c < 10; c++) s += crossMatch(a, makeWaveform({ type: 'costas', tauUs: tx.tauUs, bwMHz: tx.bwMHz, costasShift: c }));
    return { fixed: 1, hop: s / 9 };
  }
  _buildScan() {
    const c = this.cfg.scan, d = this.derived, step = c.stepFactor * d.hpbwDeg;
    const barSp = c.barSpacingDeg > 0 ? c.barSpacingDeg : c.stepFactor * d.hpbwElDeg, pos = [];
    for (let b = 0; b < c.bars; b++) {
      const el = c.elCenter + (b - (c.bars - 1) / 2) * barSp, ce = Math.cos(el * DEG);
      const umin = Math.sin((c.azCenter - c.azHalf) * DEG) * ce, umax = Math.sin((c.azCenter + c.azHalf) * DEG) * ce;
      const nAz = Math.max(1, Math.ceil((umax - umin) / (step * DEG)) + 1);
      for (let i = 0; i < nAz; i++) {
        const idx = b % 2 === 0 ? i : nAz - 1 - i, u = nAz === 1 ? (umin + umax) / 2 : umin + (umax - umin) * idx / (nAz - 1);
        pos.push({ az: Math.asin(clamp(u / ce, -1, 1)) * RAD, el, bar: b });
      }
    }
    return pos;
  }
  rangeLimitM() { return (this.cfg.mode === 'ACM' ? 18.5 : this.cfg.rangeMaxKm) * 1000; }

  /* ---- clutter geometry for a beam ---- */
  clutterInfo(beamAz, beamEl, fc, prf) {
    const cfg = this.cfg, d = this.derived, lam = wavelength(fc), env = cfg.env;
    const depr = -beamEl * DEG;
    if (!env.clutterOn || depr < 0.3 * DEG) return null;
    const h = cfg.own.alt, Rg = h / Math.sin(depr), Va = cfg.own.speed, cosPsi = Math.cos(beamAz * DEG) * Math.cos(beamEl * DEG);
    if (Rg > horizonKm(h, 0) * 1000) return null;
    const sinPsi = Math.sqrt(Math.max(0.08, 1 - cosPsi * cosPsi));
    // 主瓣零点宽度 (~2.4*HPBW) 投射的多普勒展宽 + 平台微动/杂波内运动展宽 (对应常规PD最小可检测速度 MDV ≈ 18 m/s)
    const nullBwRad = 2.4 * d.hpbwDeg * DEG;
    const mdvSpread = 2 * (2 * 18.0 / lam);
    const fcl = 2 * Va * cosPsi / lam;
    const width = Math.max(2 * Va / lam * sinPsi * nullBwRad, mdvSpread, 3 * prf / cfg.tx.nCpi);
    const sigma0 = unDb(env.clutterGammaDb) * Math.sin(depr);
    const dRg = d.rangeRes / Math.cos(depr), elFoot = Rg * d.hpbwElDeg * DEG / Math.sin(depr);
    const A = Rg * d.hpbwDeg * DEG * Math.min(dRg, elFoot);
    return { Rg, fcl, width, sigma0, area: A, footR: elFoot };
  }

  /* ---- one dwell ---- */
  dwell(beam, kind) {
    const cfg = this.cfg, d = this.derived, tx = cfg.tx, arr = this.array, t = this.time, rng = this.rng;
    const res = { t, beamAz: beam.az, beamEl: beam.el, kind, cells: [], plots: [], bursts: d.nB, illum: {} };
    this.lastDwell = res;
    if (d.txInhibit) return res;
    const states = this.targets.map(tg => this.targetState(tg, t));
    const rLim = this.rangeLimitM();
    const fluc = this.targets.map(tg => tg.swerling === 1 ? rng.exp() : 1);
    const hits = this.targets.map(() => ({ n: 0, apps: [], sig: 0, S: 0 }));
    const jamHits = {}; // 干扰产生的假目标(ghost)累计
    let cacheF = null, wT = null, wR = null, gT = [], gR = [], jInfo = [];
    const agil = cfg.eccm.agility;
    const nullsBase = [];
    if (cfg.eccm.adbf) for (const j of this.jammers) if (j.type !== 'drfm') nullsBase.push({ az: j.azDeg, el: j.elDeg });
    for (let b = 0; b < d.nB; b++) {
      const prf = d.prfs[b], pri = 1 / prf;
      const fc = agil ? rng.range(cfg.eccm.agLowGHz, cfg.eccm.agHighGHz) : cfg.fGHz, lam = wavelength(fc);
      if (cacheF !== fc) {
        cacheF = fc; wT = buildWeights(arr, this._wOpts(fc, beam.az, beam.el, false));
        wR = buildWeights(arr, this._wOpts(fc, beam.az, beam.el, true, nullsBase));
        gT = states.map(s => gainEval(arr, wT, fc, s.az, s.el)); gR = states.map(s => gainEval(arr, wR, fc, s.az, s.el));
        jInfo = this.jammers.map(j => ({ gR: gainEval(arr, wR, fc, j.azDeg, j.elDeg), gTx: 0 }));
      }
      // 噪声类干扰：J/N0 = ERP·λ²·G_r/((4π)²R²·B_j)/(kT0F)
      let JN = 0; const jDetail = [];
      this.jammers.forEach((j, ji) => {
        if (j.type !== 'spot' && j.type !== 'barrage') return;
        const bw = j.type === 'barrage' ? 4000e6 : j.bwMHz * 1e6, fj = j.type === 'barrage' ? 10 : j.fGHz;
        const overlap = Math.abs(fc - fj) <= (bw / 1e9) / 2 + this.wf.B / 2e9 || j.type === 'barrage';
        if (!overlap) return;
        const R = j.rangeKm * 1000, atm = unDb(-0.5 * gasLossTwoWayDb(j.rangeKm, fc, cfg.own.alt, cfg.own.alt));
        const dens = unDb(j.erpDbW) * lam * lam * jInfo[ji].gR / (Math.pow(4 * Math.PI, 2) * R * R * bw) * atm;
        const jn = dens / (KB * T0 * unDb(cfg.rx.nfDb)); JN += jn; jDetail.push({ id: j.id, jnDb: dB(jn + 1e-30) });
      });
      this.targets.forEach((tg, i) => {
        const s = states[i];
        if (!s.visible || s.R > rLim || s.R < 100) return;
        const eclF = eclipseLossFraction(s.R, d.tau, d.trec, pri);
        const eq = this._link(tg, s, fc, gT[i], gR[i], eclF, 1);
        res.illum[tg.id] = Math.max(res.illum[tg.id] || -999, dB(gT[i] * gR[i]) - dB(gainEval0(this, fc)) * 2);
        let S = unDb(eq.snrDb) * (tg.swerling === 2 ? rng.exp() : (tg.swerling === 1 ? fluc[i] : 1));
        let CN = 0, clutterNote = null;
        const ci = this.clutterInfo(beam.az, beam.el, fc, prf);
        if (ci) {
          const fdApp = foldDoppler(s.fd, prf), dF = Math.abs(foldDoppler(fdApp - foldDoppler(ci.fcl, prf), prf));
          const ru = C / (2 * prf), rApp = s.R % ru, rg = ci.Rg % ru;
          let dR = Math.abs(rApp - rg); dR = Math.min(dR, ru - dR);
          if (dF < ci.width / 2 && dR < ci.footR / 2 + d.rangeRes) {
            const c = this._link(tg, s, fc, gT[i], gR[i], 0, ci.area * ci.sigma0 / tg.rcs);
            CN = unDb(c.snrDb);
            if (cfg.eccm.dpca) { const eps = Math.max(1e-4, dpcaResponse(s.vGround, lam, arr.diameter / 2, cfg.own.speed)); CN /= unDb(38); S *= eps; clutterNote = 'DPCA'; }
            else clutterNote = 'clutter';
          }
        }
        const ns = 1 + JN + CN;
        const hit = S / ns > 0.003 && mcDetect(rng, S, ns, d.alpha, cfg.rx.nRef);
        const ru = C / (2 * prf), sig = Math.max(d.rangeRes / Math.sqrt(2 * Math.max(S / ns, 1e-3)), d.rangeRes / 60);
        res.cells.push({ id: tg.id, burst: b, R: s.R, rApp: s.R % ru, fdApp: foldDoppler(s.fd, prf), snrDb: dB(S + 1e-30), jnDb: dB(JN + 1e-30), cnDb: dB(CN + 1e-30), hit, isFalse: false, eclipse: eclF, note: clutterNote });
        if (hit) { const h = hits[i]; h.n++; h.apps.push({ prf, r: Math.max(0, (s.R % ru) + sig * rng.gauss()), sig }); h.S += S / ns; }
      });
      // DRFM 假目标 (RGPO + 多假目标)
      this.jammers.forEach((j, ji) => {
        if (j.type !== 'drfm') return;
        const ti = this.targets.findIndex(tg => tg.id === j.targetId); if (ti < 0) return;
        const s = states[ti], tg = this.targets[ti]; if (!s.visible || s.R > rLim) return;
        const rho = (tx.waveform === 'costas' ? d.drfmRho.hop : 1) * (agil ? Math.min(1, (this.wf.B / 1e9) / (cfg.eccm.agHighGHz - cfg.eccm.agLowGHz)) : 1);
        const atm1 = unDb(-0.5 * gasLossTwoWayDb(s.R / 1000, fc, cfg.own.alt, s.alt));
        const Sf0 = unDb(j.erpDbW) * gR[ti] * lam * lam / (Math.pow(4 * Math.PI, 2) * s.R * s.R) * d.tau * tx.nCpi * rho * atm1 / (KB * T0 * unDb(cfg.rx.nfDb) * unDb(cfg.rx.lossDb)) / unDb(d.mfLossDb) * (1 - eclipseLossFraction(s.R, d.tau, d.trec, pri));
        const per = j.rgpoMaxKm * 1000 / j.rgpoRate + 4, ph = t % per;
        const off1 = Math.min(j.rgpoMaxKm * 1000, j.rgpoRate * ph), offs = [off1, 4500, 9000];
        offs.forEach((off, gi) => {
          const S = Sf0 * (gi === 0 ? 1 : 0.5), Rg = s.R + off, ru = C / (2 * prf);
          const hit = mcDetect(rng, S, 1 + JN, d.alpha, cfg.rx.nRef);
          res.cells.push({ id: tg.id + '~G' + gi, burst: b, R: Rg, rApp: Rg % ru, fdApp: foldDoppler(s.fd, prf), snrDb: dB(S + 1e-30), hit, isFalse: true, truth: tg.id, eclipse: 0 });
          if (hit) { const key = tg.id + '~G' + gi; const h = jamHits[key] || (jamHits[key] = { n: 0, apps: [], S: 0, R: Rg, s, tg }); h.n++; h.apps.push({ prf, r: Rg % ru, sig: d.rangeRes / 4 }); h.S += S; }
        });
      });
      // 旁瓣脉冲干扰：SLB 用保护通道(全向 5dBi)比较
      this.jammers.forEach((j, ji) => {
        if (j.type !== 'pulse' || b !== 0) return;
        const R = j.rangeKm * 1000, gj = jInfo[ji].gR, gGuard = unDb(5);
        if (cfg.eccm.slb && gj < gGuard) return;
        const Sj = unDb(j.erpDbW) * lam * lam * gj / (Math.pow(4 * Math.PI, 2) * R * R) * d.tau * tx.nCpi / (KB * T0 * unDb(cfg.rx.nfDb) * unDb(cfg.rx.lossDb));
        const k = rng.poisson(j.pulseRate);
        for (let q = 0; q < k; q++) if (mcDetect(rng, Sj, 1, d.alpha, cfg.rx.nRef)) {
          const rr = rng.range(5000, rLim); res.plots.push({ R: rr, az: beam.az + rng.gauss() * d.hpbwDeg * 0.3, el: beam.el, snrDb: dB(Sj), isFalse: true, truth: j.id, kind: 'pulse-jam' });
        }
      });
    }
    // M-of-N 判决 → 点迹
    this.targets.forEach((tg, i) => {
      const h = hits[i], s = states[i];
      const ts = this.targetStats[tg.id] || (this.targetStats[tg.id] = { dwells: 0, illum: 0, det: 0, lastDet: -1, lastSnr: -99 });
      if (res.illum[tg.id] != null && res.illum[tg.id] > -6) ts.illum++;
      if (h.n >= d.M) { ts.det++; ts.lastDet = t; this._emitPlot(res, tg.id, false, h, s, beam, tg.id, tg.isGround); ts.lastSnr = dB(h.S / h.n); }
    });
    for (const key in jamHits) { const h = jamHits[key]; if (h.n >= d.M) this._emitPlot(res, key, true, h, h.s, beam, h.tg.id, false, h.R); }
    // 虚警：单元数 × Pfa
    const nFa = rng.poisson(d.nCells * cfg.rx.pfa);
    for (let q = 0; q < nFa; q++) res.plots.push({ R: rng.range(5000, rLim), az: beam.az + rng.gauss() * d.hpbwDeg * 0.3, el: beam.el, snrDb: dB(d.alpha * 1.2), isFalse: true, truth: 'FA', kind: 'false-alarm' });
    for (const p of res.plots) { p.t = t; p.cart = cart(p.R, p.az, p.el); this.plots.push(p); }
    if (this.plots.length > 600) this.plots.splice(0, this.plots.length - 600);
    if (cfg.mode === 'GM') this._paintGm(beam);
    this._processPlots(res.plots, t, kind, beam);
    return res;
  }
  _emitPlot(res, id, isFalse, h, s, beam, truth, ground, Roverride) {
    const d = this.derived, rng = this.rng, apps = h.apps;
    const tol = 6 * Math.max(...apps.map(a => a.sig)) + d.rangeRes;
    let rr = resolveRange(apps, tol, this.cfg.rangeMaxKm * 1000 * 1.5), R, amb = false;
    if (rr.ok) { R = rr.R; amb = rr.single ? (C / (2 * apps[0].prf)) < this.cfg.rangeMaxKm * 1000 : !!rr.amb; } else { R = apps[0].r; amb = true; }
    const snr = h.S / h.n, sa = Math.hypot(s.az - beam.az, s.el - beam.el), inBeam = sa < 0.6 * d.hpbwRxDeg;
    const sigTh = d.hpbwRxDeg / (1.6 * Math.sqrt(2 * Math.max(snr, 1)));
    const az = (inBeam ? s.az : beam.az) + sigTh * rng.gauss(), el = (inBeam ? s.el : beam.el) + sigTh * rng.gauss();
    res.plots.push({ R, az, el, snrDb: dB(snr), isFalse, truth, kind: isFalse ? 'drfm' : 'target', amb, resolved: rr.ok && !rr.single, sidelobe: !inBeam, ground, id });
  }
  _link(tg, s, fc, gT, gR, eclF, rcsOverride) {
    const cfg = this.cfg, d = this.derived, rx = cfg.rx;
    const Rkm = s.R / 1000;
    return radarEquation({
      fGHz: fc, PtW: d.PtW, tauS: d.tau, nCoh: cfg.tx.nCpi, GtLin: gT, GrLin: gR, rcs: tg.rcs * rcsOverride, Rm: s.R, nfDb: rx.nfDb, lossDb: rx.lossDb + d.mfLossDb,
      atmDb: gasLossTwoWayDb(Rkm, fc, cfg.own.alt, s.alt),
      rainDb: rainLossTwoWayDb(Rkm, fc, cfg.env.rainMmH, cfg.env.rainExtentKm, cfg.own.alt, s.alt, cfg.env.rainTopKm),
      stcDb: rx.stc ? stcLossDb(Rkm, rx.stcRangeKm, 2.5) : 0, eclipseDb: eclF > 0 ? -dB(Math.max(1e-6, 1 - eclF)) : 0
    });
  }

  /* ---- tracker (alpha-beta, 笛卡尔) ---- */
  _processPlots(plots, t, kind, beam) {
    const d = this.derived, used = new Set(), le = this.cfg.eccm.leadingEdge;
    if (!d.modeInfo.tracks) return;
    for (const tr of this.tracks) {
      const dt = t - tr.t, pred = [tr.x + tr.vx * dt, tr.y + tr.vy * dt, tr.z + tr.vz * dt];
      const sp = Math.hypot(tr.vx, tr.vy, tr.vz), gate = this._gate(tr, dt, sp);
      let best = null, bs = -1e18;
      for (const p of plots) {
        if (used.has(p)) continue;
        const dd = Math.hypot(p.cart[0] - pred[0], p.cart[1] - pred[1], p.cart[2] - pred[2]);
        if (dd < gate) { const sc = le ? -p.R : p.snrDb; if (sc > bs) { bs = sc; best = p; } }
      }
      if (best) {
        used.add(best);
        const z = best.cart, r = [z[0] - pred[0], z[1] - pred[1], z[2] - pred[2]];
        const a = tr.hits < 2 ? 0.7 : 0.45, b = 0.08, ddt = Math.max(dt, 0.05);
        tr.x = pred[0] + a * r[0]; tr.y = pred[1] + a * r[1]; tr.z = pred[2] + a * r[2];
        if (tr.hits >= 1) { tr.vx += b * r[0] / ddt; tr.vy += b * r[1] / ddt; tr.vz += b * r[2] / ddt; }
        tr.t = t; tr.lastHit = t; tr.hits++; tr.missDw = 0; tr.truth = best.truth; tr.isFalse = best.isFalse; tr.snrDb = best.snrDb;
        if (tr.hits >= 3) tr.confirmed = true;
      } else if (kind === 'track' && tr.id === this.lastTrackedId) tr.missDw = (tr.missDw || 0) + 1;
    }
    for (const p of plots) {
      if (used.has(p) || p.kind === 'false-alarm' || p.kind === 'pulse-jam' || p.sidelobe) continue;
      let near = false;
      for (const tr of this.tracks) { const dt = t - tr.t; if (Math.hypot(p.cart[0] - (tr.x + tr.vx * dt), p.cart[1] - (tr.y + tr.vy * dt), p.cart[2] - (tr.z + tr.vz * dt)) < 6000) { near = true; break; } }
      if (!near) this.tracks.push({ id: 'TK' + (++this.trid), x: p.cart[0], y: p.cart[1], z: p.cart[2], vx: 0, vy: 0, vz: 0, t, lastHit: t, hits: 1, confirmed: false, truth: p.truth, isFalse: p.isFalse, snrDb: p.snrDb, missDw: 0, born: t });
    }
    this.tracks = this.tracks.filter(tr => {
      const lost = t - tr.lastHit > (tr.confirmed ? d.trackLossS : 1.5 * d.frameS + 2) || (tr.id === this.lockId && (tr.missDw || 0) > 40);
      if (lost && tr.id === this.lockId) { this.events.push({ t, msg: `STT 失锁 ${tr.id}` }); this.lockId = null; }
      return !lost;
    });
    if (this.cfg.mode === 'ACM' && !this.lockId) {
      const first = this.tracks.find(tr => tr.confirmed) || null;
      if (first || plots.some(p => !p.isFalse && p.kind === 'target')) {
        const tr = first || this.tracks[this.tracks.length - 1];
        if (tr) { tr.confirmed = true; this.lockId = tr.id; this.cfg.mode = 'STT'; this.events.push({ t, msg: `ACM 自动截获 → STT 锁定 ${tr.id}` }); this.rebuild(); }
      }
    }
  }
  _gate(tr, dt, sp) {
    const d = this.derived, R = Math.hypot(tr.x, tr.y, tr.z), sR = d.rangeRes, sA = R * d.hpbwRxDeg * DEG * 0.5;
    return 4 * Math.hypot(sR, sA) + 0.1 * sp * Math.max(dt, 0) + 500;
  }
  autoLockStrongest(n) {
    const c = this.tracks.filter(t => t.confirmed).sort((a, b) => b.snrDb - a.snrDb);
    this.lockIds = c.slice(0, n).map(t => t.id); this.lockId = this.lockIds[0] || null;
  }
  lockTrack(id) { this.lockId = id; if (!this.lockIds.includes(id)) this.lockIds.unshift(id); this.lockIds = this.lockIds.slice(0, 2); }
  breakLock() { this.lockId = null; this.lockIds = []; }

  /* ---- scheduler ---- */
  _trackBeam(tr) {
    const dt = this.time - tr.t, x = tr.x + tr.vx * dt, y = tr.y + tr.vy * dt, z = tr.z + tr.vz * dt;
    // 跟踪点为"相对载机的笛卡尔"，转成波束指向
    return { az: Math.atan2(x, y) * RAD, el: Math.atan2(z, Math.hypot(x, y)) * RAD };
  }
  nextBeam() {
    const m = this.cfg.mode, conf = this.tracks.filter(t => t.confirmed);
    const lock = this.tracks.find(t => t.id === this.lockId);
    if (m === 'STT' && lock) { this.lastTrackedId = lock.id; return Object.assign(this._trackBeam(lock), { kind: 'track' }); }
    if (m === 'DTT') {
      const ts = this.lockIds.map(id => this.tracks.find(t => t.id === id)).filter(Boolean);
      if (ts.length) { this.rr = (this.rr + 1) % ts.length; this.lastTrackedId = ts[this.rr].id; return Object.assign(this._trackBeam(ts[this.rr]), { kind: 'track' }); }
    }
    if (m === 'MTT' && conf.length) { const ts = conf.slice(0, 8); this.rr = (this.rr + 1) % ts.length; this.lastTrackedId = ts[this.rr].id; return Object.assign(this._trackBeam(ts[this.rr]), { kind: 'track' }); }
    if (m === 'TAS' && conf.length) {
      let pick = null, worst = 0;
      for (const tr of conf) { const age = this.time - (this.lastTrackDwell[tr.id] || -1e9); if (age > this.cfg.track.revisitS && age > worst) { worst = age; pick = tr; } }
      if (pick) { this.lastTrackDwell[pick.id] = this.time; this.lastTrackedId = pick.id; return Object.assign(this._trackBeam(pick), { kind: 'track' }); }
    }
    const p = this.scanList[this.scanIdx];
    this.scanIdx++;
    if (this.scanIdx >= this.scanList.length) { this.scanIdx = 0; this.frames++; this.frameTimeS = this.time - this.frameStartT; this.frameStartT = this.time; }
    return { az: p.az, el: p.el, kind: 'search', bar: p.bar };
  }
  step(dtReal, budgetMs) {
    const t0 = Date.now(); this.timeBudget += dtReal * this.cfg.timeScale; this.slow = false;
    const d = this.derived;
    let guard = 0;
    while (this.timeBudget >= d.dwellS && guard++ < 400) {
      const beam = this.nextBeam(); this.currentBeam = beam;
      this.dwell(beam, beam.kind); this.time += d.dwellS; this.timeBudget -= d.dwellS;
      if (budgetMs && Date.now() - t0 > budgetMs) { this.slow = true; this.timeBudget = 0; break; }
    }
    if (this.timeBudget > 2) this.timeBudget = 0;
    const horizon = Math.max(8, 2 * d.frameS + 4);
    this.plots = this.plots.filter(p => this.time - p.t < horizon);
  }

  /* ---- GM 实波束地图 ---- */
  _paintGm(beam) {
    const cfg = this.cfg, d = this.derived, nR = 120, nA = 161;
    if (!this.gm) this.gm = { nR, nA, v: new Float32Array(nR * nA), rKm: 60 };
    const g = this.gm, h = cfg.own.alt, fc = cfg.fGHz, rMax = g.rKm * 1000;
    for (let i = 0; i < g.v.length; i++) g.v[i] *= 0.9995;
    const hp = d.hpbwDeg, aMin = Math.floor(beam.az - hp), aMax = Math.ceil(beam.az + hp);
    const cutEl = d.twoWayEl, cutAzv = d.twoWayAz;
    const interp = (c, x) => { const a = c.ang, n = a.length; if (x <= a[0] || x >= a[n - 1]) return -60; const f = (x - a[0]) / (a[n - 1] - a[0]) * (n - 1), i = Math.floor(f); return Math.max(-60, c.db[i] + (c.db[i + 1] - c.db[i]) * (f - i)); };
    for (let az = aMin; az <= aMax; az++) {
      const ai = az + 80; if (ai < 0 || ai >= nA) continue;
      const azR = interp(cutAzv, az - beam.az);
      for (let ri = 0; ri < nR; ri++) {
        const R = (ri + 0.5) / nR * rMax; if (R <= h * 1.05) continue;
        const depr = Math.asin(h / R) * RAD, elR = interp(cutEl, -depr - beam.el);
        const x = R * Math.cos(depr * DEG) * Math.sin(az * DEG), y = R * Math.cos(depr * DEG) * Math.cos(az * DEG);
        const s0 = unDb(terrainSigma0Db(x, y)) * Math.sin(depr * DEG) / Math.sin(10 * DEG);
        const A = R * hp * DEG * d.rangeRes / Math.cos(depr * DEG);
        const eq = radarEquation({ fGHz: fc, PtW: d.PtW, tauS: d.tau, nCoh: cfg.tx.nCpi, GtLin: unDb(d.g0Db + azR / 2 + elR / 2), GrLin: unDb(d.grDb + azR / 2 + elR / 2), rcs: s0 * A, Rm: R, nfDb: cfg.rx.nfDb, lossDb: cfg.rx.lossDb + d.mfLossDb, atmDb: gasLossTwoWayDb(R / 1000, fc, h, 0) });
        const v = clamp((eq.snrDb + 5) / 45, 0, 1), k = ri * nA + ai;
        if (v > g.v[k]) g.v[k] = v;
      }
    }
  }

  /* ---- 解析：单目标可探测性诊断(雷达方程+扫描匹配) ---- */
  analyzeTarget(tg) {
    const cfg = this.cfg, d = this.derived, s = this.targetState(tg, this.time), fc = cfg.fGHz, arr = this.array;
    const out = { id: tg.id, name: tg.name, R: s.R, az: s.az, el: s.el, closing: s.closing, fd: s.fd, alt: s.alt, verdict: 'OK', reasons: [] };
    if (d.txInhibit) { out.verdict = 'TX_INHIBIT'; out.reasons.push(d.txInhibit === 'duty' ? `发射占空比 ${(d.duty * 100).toFixed(0)}% 超过 TR 热限 ${(cfg.tx.dutyMax * 100).toFixed(0)}%，发射被禁止` : '脉宽 ≥ PRI'); return out; }
    if (!s.visible) { out.verdict = 'NO_LOS'; out.reasons.push(`超出无线电视距 (${horizonKm(cfg.own.alt, s.alt).toFixed(0)} km)`); }
    if (s.R > this.rangeLimitM()) { out.verdict = 'TOO_FAR'; out.reasons.push(`超出处理量程 ${(this.rangeLimitM() / 1000).toFixed(0)} km`); }
    // 扫描匹配：目标与最近扫描波束的真实角度差
    let bestLoss = 1e9, nearest = null;
    const uT = Math.sin(s.az * DEG) * Math.cos(s.el * DEG), vT = Math.sin(s.el * DEG);
    const wT0 = Math.sqrt(Math.max(0, 1 - uT * uT - vT * vT));
    for (const p of this.scanList) {
      const uP = Math.sin(p.az * DEG) * Math.cos(p.el * DEG), vP = Math.sin(p.el * DEG);
      const wP = Math.sqrt(Math.max(0, 1 - uP * uP - vP * vP));
      const off = Math.acos(clamp(uT * uP + vT * vP + wT0 * wP, -1, 1)) * RAD;
      if (off < bestLoss) { bestLoss = off; nearest = p; }
    }
    if (!nearest) nearest = { az: s.az, el: s.el };
    const tracking = (cfg.mode === 'STT' || cfg.mode === 'DTT') && this.lockId;
    const steer = tracking ? { az: s.az, el: s.el } : nearest;
    // 诊断与 dwell 同源：用真实扫描波束权值在目标方向求增益
    const wT = buildWeights(arr, this._wOpts(fc, steer.az, steer.el, false));
    const wR = buildWeights(arr, this._wOpts(fc, steer.az, steer.el, true));
    const gT = gainEval(arr, wT, fc, s.az, s.el), gR = gainEval(arr, wR, fc, s.az, s.el);
    const gPk = gainEval(arr, wT, fc, steer.az, steer.el) * gainEval(arr, wR, fc, steer.az, steer.el);
    const scanLossDb = tracking ? 0 : Math.max(0, dB(gPk + 1e-30) - dB(gT * gR + 1e-30));
    out.scanOffsetDeg = tracking ? 0 : bestLoss;
    out.scanLossDb = scanLossDb;
    out.inScan = tracking ? true : scanLossDb < 4.5;
    if (!out.inScan) { out.verdict = out.verdict === 'OK' ? 'OUT_OF_SCAN' : out.verdict; out.reasons.push(`目标(方位 ${s.az.toFixed(1)}°, 俯仰 ${s.el.toFixed(1)}°)落在扫描图形之外，距最近波束 ${bestLoss.toFixed(1)}°，波束失配损耗 ${scanLossDb.toFixed(1)} dB`); }
    const ps = [];
    let jn = 0;
    for (const j of this.jammers) if (j.type === 'spot' || j.type === 'barrage') {
      const bw = j.type === 'barrage' ? 4000e6 : j.bwMHz * 1e6, fj = j.type === 'barrage' ? 10 : j.fGHz;
      const wRj = buildWeights(arr, this._wOpts(fc, s.az, s.el, true, cfg.eccm.adbf ? this.jammers.filter(q => q.type !== 'drfm').map(q => ({ az: q.azDeg, el: q.elDeg })) : []));
      if (!(j.type === 'barrage' || (!cfg.eccm.agility && Math.abs(fc - fj) <= bw / 2e9 + this.wf.B / 2e9))) continue;
      const lam = wavelength(fc), R = j.rangeKm * 1000, gj = gainEval(arr, wRj, fc, j.azDeg, j.elDeg);
      jn += unDb(j.erpDbW) * lam * lam * gj / (Math.pow(4 * Math.PI, 2) * R * R * bw) / (KB * T0 * unDb(cfg.rx.nfDb));
      if (cfg.eccm.agility && j.type === 'spot') jn *= 1;
    }
    out.jnDb = dB(jn + 1e-30);
    for (let b = 0; b < d.nB; b++) {
      const eclF = eclipseLossFraction(s.R, d.tau, d.trec, 1 / d.prfs[b]);
      ps.push(this._link(tg, s, fc, gT, gR, eclF, 1).snrDb);
    }
    out.eq = this._link(tg, s, fc, gT, gR, 0, 1);
    out.eclipse = d.prfs.map(f => eclipseLossFraction(s.R, d.tau, d.trec, 1 / f));
    out.gTdb = dB(gT); out.gRdb = dB(gR);
    // 杂波/多普勒判定(用第一个 PRF)
    const ci = this.clutterInfo(steer.az, steer.el, fc, d.prfs[0]);
    let cn = 0;
    if (ci) {
      const fdApp = foldDoppler(s.fd, d.prfs[0]), dF = Math.abs(foldDoppler(fdApp - foldDoppler(ci.fcl, d.prfs[0]), d.prfs[0]));
      out.clutterBand = { fcl: ci.fcl, width: ci.width, dF };
      const ru = C / (2 * d.prfs[0]); let dR = Math.abs(s.R % ru - ci.Rg % ru); dR = Math.min(dR, ru - dR);
      if (dF < ci.width / 2 && dR < ci.footR / 2 + d.rangeRes) {
        const c = this._link(tg, s, fc, gT, gR, 0, ci.area * ci.sigma0 / tg.rcs); cn = unDb(c.snrDb); out.cnDb = c.snrDb;
        if (cfg.eccm.dpca) { const eps = Math.max(1e-4, dpcaResponse(s.vGround, wavelength(fc), arr.diameter / 2, cfg.own.speed)); cn /= unDb(38); out.dpcaEps = eps; ps.forEach((v, i) => ps[i] = v + dB(eps)); }
        out.inClutter = true;
      }
    }
    const ns = 1 + jn + cn, nsDb = dB(ns); out.noiseRaiseDb = nsDb;
    const sinr = ps.map(v => unDb(v - nsDb)), pf = d.pfaBurst;
    out.snrBurstDb = ps.reduce((a, b) => a + b, 0) / ps.length - nsDb;
    out.pdBurst = sinr.map(S => pdBurst(S, pf, cfg.rx.nRef, tg.swerling));
    out.pdSys = pbTail(out.pdBurst, d.M);
    out.reqSnrDb = requiredSnrDb(pf, 0.9, cfg.rx.nRef, tg.swerling);
    out.marginDb = out.snrBurstDb - out.reqSnrDb;
    if (out.verdict === 'OK') {
      if (out.inClutter && out.pdSys < 0.5) { out.verdict = 'CLUTTER'; out.reasons.push('目标多普勒/距离落入主瓣杂波，SCR 不足'); }
      else if (jn > 1 && out.pdSys < 0.5) { out.verdict = 'JAMMED'; out.reasons.push(`压制干扰使噪声基底抬高 ${nsDb.toFixed(1)} dB，SINR 低于检测门限`); }
      else if (out.pdSys < 0.5) {
        const ecl = Math.max(...out.eclipse);
        out.verdict = ecl > 0.5 && out.marginDb > -3 ? 'ECLIPSED' : 'LOW_SNR';
        out.reasons.push(`单次 SINR ${out.snrBurstDb.toFixed(1)} dB < 所需 ${out.reqSnrDb.toFixed(1)} dB (Pd=0.9)，系统 Pd=${(out.pdSys * 100).toFixed(1)}%`);
      }
    }
    if (out.verdict === 'OK') out.reasons.push(`余量 +${out.marginDb.toFixed(1)} dB，系统 Pd=${(out.pdSys * 100).toFixed(1)}%`);
    // 雷达方程最大探测距离(波束对准、Pd=0.9)
    out.rmaxKm = this.maxRangeFor(tg, gT, gR, s, nsDb);
    return out;
  }
  maxRangeFor(tg, gT, gR, s, noiseRaiseDb) {
    const cfg = this.cfg, d = this.derived, fc = cfg.fGHz;
    const need = requiredSnrDb(d.pfaBurst, 0.9, cfg.rx.nRef, tg.swerling) + noiseRaiseDb;
    let lo = 1, hi = 1500;
    for (let i = 0; i < 40; i++) {
      const m = (lo + hi) / 2, ss = Object.assign({}, s, { R: m * 1000, alt: s.alt });
      const snr = this._link(tg, ss, fc, gT, gR, 0, 1).snrDb;
      if (snr > need) lo = m; else hi = m;
    }
    return lo;
  }

  /** 合成指定波束指向下的实时 A-Scope 脉压距离像与滑动窗 CA-CFAR 门限 */
  synthesizeAscope(beamAz, beamEl, nBins, rMaxKm) {
    nBins = nBins || 256;
    const cfg = this.cfg, d = this.derived, arr = this.array, tx = cfg.tx;
    const maxKm = rMaxKm || (cfg.mode === 'ACM' ? 20 : Math.min(cfg.rangeMaxKm, 200));
    const fc = cfg.fGHz, lam = wavelength(fc), prf = d.prfs[0], pri = 1 / prf;
    const nulls = cfg.eccm.adbf ? this.jammers.filter(j => j.type !== 'drfm').map(j => ({ az: j.azDeg, el: j.elDeg })) : [];
    const wT = buildWeights(arr, this._wOpts(fc, beamAz, beamEl, false));
    const wR = buildWeights(arr, this._wOpts(fc, beamAz, beamEl, true, nulls));

    // 1. 计算热噪声 + 压制干扰基底 (线性功率，以热噪声 N0=1 归一化)
    let JN = 0;
    for (const j of this.jammers) {
      if (j.type !== 'spot' && j.type !== 'barrage') continue;
      const bw = j.type === 'barrage' ? 4000e6 : j.bwMHz * 1e6, fj = j.type === 'barrage' ? 10 : j.fGHz;
      const overlap = j.type === 'barrage' || (!cfg.eccm.agility && Math.abs(fc - fj) <= (bw / 1e9) / 2 + this.wf.B / 2e9);
      if (!overlap) continue;
      const R = j.rangeKm * 1000, gJ = gainEval(arr, wR, fc, j.azDeg, j.elDeg);
      const atm = unDb(-0.5 * gasLossTwoWayDb(j.rangeKm, fc, cfg.own.alt, cfg.own.alt));
      JN += (unDb(j.erpDbW) * lam * lam * gJ / (Math.pow(4 * Math.PI, 2) * R * R * bw) * atm) / (KB * T0 * unDb(cfg.rx.nfDb));
    }
    const baseNoise = 1 + JN;
    const pwr = new Float64Array(nBins);
    const rKm = new Float64Array(nBins);
    const rng = makeRng((Math.floor(this.time * 20) + 101) >>> 0);
    for (let i = 0; i < nBins; i++) {
      rKm[i] = (i + 0.5) / nBins * maxKm;
      pwr[i] = baseNoise * Math.max(0.15, rng.exp());
    }

    // 2. 地杂波距离包络
    const ci = this.clutterInfo(beamAz, beamEl, fc, prf);
    if (ci && ci.Rg / 1000 < maxKm) {
      const dummyTg = { rcs: ci.area * ci.sigma0 };
      const cEq = this._link(dummyTg, { R: ci.Rg, alt: 0 }, fc, unDb(d.g0Db), unDb(d.grDb), 0, 1);
      let cLin = unDb(cEq.snrDb);
      if (cfg.eccm.dpca) cLin /= unDb(38);
      const cCenter = ci.Rg / 1000, cHalfKm = Math.max(1.5, ci.footR / 2000);
      for (let i = 0; i < nBins; i++) {
        const u = (rKm[i] - cCenter) / cHalfKm;
        if (Math.abs(u) < 2.5) pwr[i] += cLin * Math.exp(-1.5 * u * u) * (0.6 + 0.8 * rng.next());
      }
    }

    // 3. 注入目标脉压响应 (使用真实的匹配滤波包络 mf.env)
    const env = this.mf.env, cIdx = this.mf.centerIdx, halfSpan = Math.max(1, Math.floor(env.length / 2));
    const states = this.targets.map(tg => this.targetState(tg, this.time));
    const targetsInProfile = [];
    this.targets.forEach((tg, ti) => {
      const s = states[ti];
      if (!s.visible || s.R / 1000 > maxKm || d.txInhibit) return;
      const gT = gainEval(arr, wT, fc, s.az, s.el), gR = gainEval(arr, wR, fc, s.az, s.el);
      const eclF = eclipseLossFraction(s.R, d.tau, d.trec, pri);
      const eq = this._link(tg, s, fc, gT, gR, eclF, 1);
      let S = unDb(eq.snrDb);
      if (ci && cfg.eccm.dpca) {
        const fdApp = foldDoppler(s.fd, prf), dF = Math.abs(foldDoppler(fdApp - foldDoppler(ci.fcl, prf), prf));
        if (dF < ci.width / 2) S *= Math.max(1e-4, dpcaResponse(s.vGround, lam, arr.diameter / 2, cfg.own.speed));
      }
      const binF = (s.R / 1000) / maxKm * nBins, b0 = Math.round(binF);
      for (let k = -8; k <= 8; k++) {
        const bi = b0 + k;
        if (bi < 0 || bi >= nBins) continue;
        const envIdx = clamp(Math.round(cIdx + (k / 8) * halfSpan * 0.45), 0, env.length - 1);
        const shape = k === 0 ? 1.0 : Math.pow(env[envIdx], 2);
        pwr[bi] += S * shape;
      }
      targetsInProfile.push({ id: tg.id, name: tg.name, rKm: s.R / 1000, bin: clamp(b0, 0, nBins - 1), snrDb: dB(S + 1e-30), sinrDb: dB(S / baseNoise + 1e-30) });
    });

    // 4. 注入 DRFM 假目标 (受 Costas 跨脉冲匹配滤波失配因子 rho 调制)
    this.jammers.forEach(j => {
      if (j.type !== 'drfm' || d.txInhibit) return;
      const ti = this.targets.findIndex(tg => tg.id === j.targetId);
      if (ti < 0) return;
      const s = states[ti];
      if (!s.visible || s.R / 1000 > maxKm) return;
      const gR = gainEval(arr, wR, fc, s.az, s.el);
      const rho = (tx.waveform === 'costas' ? d.drfmRho.hop : 1) * (cfg.eccm.agility ? Math.min(1, (this.wf.B / 1e9) / (cfg.eccm.agHighGHz - cfg.eccm.agLowGHz)) : 1);
      const atm1 = unDb(-0.5 * gasLossTwoWayDb(s.R / 1000, fc, cfg.own.alt, s.alt));
      const Sf0 = unDb(j.erpDbW) * gR * lam * lam / (Math.pow(4 * Math.PI, 2) * s.R * s.R) * d.tau * tx.nCpi * rho * atm1 / (KB * T0 * unDb(cfg.rx.nfDb) * unDb(cfg.rx.lossDb)) / unDb(d.mfLossDb);
      const per = j.rgpoMaxKm * 1000 / j.rgpoRate + 4, ph = this.time % per;
      const offs = [Math.min(j.rgpoMaxKm * 1000, j.rgpoRate * ph), 4500, 9000];
      offs.forEach((off, gi) => {
        const RgKm = (s.R + off) / 1000;
        if (RgKm >= maxKm) return;
        const b0 = Math.round(RgKm / maxKm * nBins);
        if (b0 >= 0 && b0 < nBins) pwr[b0] += Sf0 * (gi === 0 ? 1 : 0.5);
      });
    });

    // 5. 滑动窗 CA-CFAR 门限计算
    const cfarDb = new Float64Array(nBins), powerDb = new Float64Array(nBins);
    const halfRef = Math.max(4, Math.min(16, Math.floor(cfg.rx.nRef / 2))), guard = 2;
    const detections = [];
    for (let i = 0; i < nBins; i++) powerDb[i] = dB(pwr[i] + 1e-30);
    for (let i = 0; i < nBins; i++) {
      let sum = 0, cnt = 0;
      for (let k = -halfRef - guard; k <= halfRef + guard; k++) {
        if (Math.abs(k) <= guard) continue;
        const idx = i + k;
        if (idx >= 0 && idx < nBins) { sum += pwr[idx]; cnt++; }
      }
      const refMean = cnt > 0 ? sum / cnt : baseNoise;
      const thLin = refMean * d.alpha;
      cfarDb[i] = dB(thLin + 1e-30);
      if (pwr[i] > thLin) detections.push({ bin: i, rKm: rKm[i], snrDb: dB(pwr[i] / refMean) });
    }
    return { rKm, powerDb, cfarDb, noiseFloorDb: dB(baseNoise), maxKm, detections, targetsInProfile };
  }

  /** 合成二维距离-多普勒 (Range-Doppler) 功率谱矩阵 */
  synthesizeRdMap(beamAz, beamEl, nR, nD) {
    nR = nR || 64; nD = nD || 40;
    const cfg = this.cfg, d = this.derived, arr = this.array;
    const fc = cfg.fGHz, lam = wavelength(fc), prf = d.prfs[0], ruKm = C / (2 * prf) / 1000;
    const nulls = cfg.eccm.adbf ? this.jammers.filter(j => j.type !== 'drfm').map(j => ({ az: j.azDeg, el: j.elDeg })) : [];
    const wT = buildWeights(arr, this._wOpts(fc, beamAz, beamEl, false));
    const wR = buildWeights(arr, this._wOpts(fc, beamAz, beamEl, true, nulls));

    let JN = 0;
    for (const j of this.jammers) {
      if (j.type !== 'spot' && j.type !== 'barrage') continue;
      const bw = j.type === 'barrage' ? 4000e6 : j.bwMHz * 1e6, fj = j.type === 'barrage' ? 10 : j.fGHz;
      const overlap = j.type === 'barrage' || (!cfg.eccm.agility && Math.abs(fc - fj) <= (bw / 1e9) / 2 + this.wf.B / 2e9);
      if (!overlap) continue;
      const R = j.rangeKm * 1000, gJ = gainEval(arr, wR, fc, j.azDeg, j.elDeg);
      JN += (unDb(j.erpDbW) * lam * lam * gJ / (Math.pow(4 * Math.PI, 2) * R * R * bw)) / (KB * T0 * unDb(cfg.rx.nfDb));
    }
    const baseNoise = 1 + JN;
    const gridDb = new Float32Array(nR * nD);
    const rng = makeRng((Math.floor(this.time * 10) + 303) >>> 0);
    for (let i = 0; i < gridDb.length; i++) gridDb[i] = dB(baseNoise * Math.max(0.2, rng.exp()));

    // 主瓣地杂波脊线
    const ci = this.clutterInfo(beamAz, beamEl, fc, prf);
    if (ci) {
      const fclFold = foldDoppler(ci.fcl, prf);
      const dummyTg = { rcs: ci.area * ci.sigma0 };
      let cLin = unDb(this._link(dummyTg, { R: ci.Rg, alt: 0 }, fc, unDb(d.g0Db), unDb(d.grDb), 0, 1).snrDb);
      if (cfg.eccm.dpca) cLin /= unDb(38);
      const dCenter = (fclFold / prf + 0.5) * (nD - 1);
      const dSigma = Math.max(1.2, (ci.width / prf) * nD * 0.5);
      for (let ri = 0; ri < nR; ri++) {
        for (let di = 0; di < nD; di++) {
          const df = (di - dCenter) / dSigma;
          if (Math.abs(df) < 2.8) {
            const cur = unDb(gridDb[ri * nD + di]);
            gridDb[ri * nD + di] = dB(cur + cLin * Math.exp(-0.8 * df * df) * 0.35);
          }
        }
      }
    }

    // 目标 R-D 峰值
    const markers = [];
    this.targets.forEach(tg => {
      if (d.txInhibit) return;
      const s = this.targetState(tg, this.time);
      if (!s.visible) return;
      const gT = gainEval(arr, wT, fc, s.az, s.el), gR = gainEval(arr, wR, fc, s.az, s.el);
      const eq = this._link(tg, s, fc, gT, gR, 0, 1);
      let S = unDb(eq.snrDb);
      if (ci && cfg.eccm.dpca) {
        const fdApp = foldDoppler(s.fd, prf), dF = Math.abs(foldDoppler(fdApp - foldDoppler(ci.fcl, prf), prf));
        if (dF < ci.width / 2) S *= Math.max(1e-4, dpcaResponse(s.vGround, lam, arr.diameter / 2, cfg.own.speed));
      }
      const rFoldKm = (s.R / 1000) % ruKm;
      const fdFold = foldDoppler(s.fd, prf);
      const rIdx = clamp(Math.floor(rFoldKm / ruKm * nR), 0, nR - 1);
      const dIdx = clamp(Math.floor((fdFold / prf + 0.5) * nD), 0, nD - 1);
      for (let dr = -1; dr <= 1; dr++) for (let dd = -1; dd <= 1; dd++) {
        const rr = rIdx + dr, dd2 = dIdx + dd;
        if (rr >= 0 && rr < nR && dd2 >= 0 && dd2 < nD) {
          const w = (dr === 0 && dd === 0) ? 1.0 : 0.25;
          gridDb[rr * nD + dd2] = dB(unDb(gridDb[rr * nD + dd2]) + S * w);
        }
      }
      markers.push({ id: tg.id, name: tg.name, rIdx, dIdx, rFoldKm, fdFold, snrDb: dB(S + 1e-30), sinrDb: dB(S / baseNoise + 1e-30) });
    });

    return { gridDb, nR, nD, ruKm, prfHz: prf, noiseFloorDb: dB(baseNoise), markers };
  }
}
function gainEval0(sim, fc) { return sim.derived.g0Db >= -100 ? Math.pow(10, sim.derived.g0Db / 10) : 1; }
function cart(R, az, el) { const ce = Math.cos(el * DEG); return [R * ce * Math.sin(az * DEG), R * ce * Math.cos(az * DEG), R * Math.sin(el * DEG)]; }

/** SAR：点散射体场景 × 由 (B, λ, V, T) 决定的二维点扩散函数 */
function sarRender(o) {
  const res = sarResolution({ bwHz: o.bwHz, lambda: o.lambda, R0: o.R0, Va: o.Va, Tsar: o.Tsar });
  const rr = res.rangeRes * (o.window === 'hamming' ? 1.3 : 1), ra = res.azRes, n = o.n || 128, ext = o.extentM;
  const img = new Float64Array(n * n), rng = makeRng(o.seed || 3), nCoh = Math.max(1, Math.round(o.prf * o.Tsar));
  const amps = o.scene.map(sc => Math.sqrt(unDb(radarEquation({ fGHz: C / o.lambda / 1e9, PtW: o.PtW, tauS: o.tau, nCoh, GtLin: o.G, GrLin: o.G, rcs: sc.rcs, Rm: o.R0, nfDb: o.nfDb, lossDb: o.lossDb, atmDb: o.atmDb || 0 }).snrDb)));
  const re = new Float64Array(n * n), im = new Float64Array(n * n);
  o.scene.forEach((sc, k) => {
    const ph = rng.next() * 2 * Math.PI;
    for (let iy = 0; iy < n; iy++) for (let ix = 0; ix < n; ix++) {
      const x = (ix / (n - 1) - 0.5) * ext, y = (iy / (n - 1) - 0.5) * ext;
      const sx = (x - sc.x) / (0.886 * ra), sy = (y - sc.y) / (0.886 * rr);
      const f = (Math.abs(sx) < 1e-9 ? 1 : Math.sin(Math.PI * sx) / (Math.PI * sx)) * (Math.abs(sy) < 1e-9 ? 1 : Math.sin(Math.PI * sy) / (Math.PI * sy));
      if (Math.abs(f) < 1e-3) continue;
      re[iy * n + ix] += amps[k] * f * Math.cos(ph); im[iy * n + ix] += amps[k] * f * Math.sin(ph);
    }
  });
  for (let i = 0; i < n * n; i++) { const a = re[i] + rng.gauss() / Math.SQRT2, b = im[i] + rng.gauss() / Math.SQRT2; img[i] = dB(a * a + b * b + 1e-9); }
  return { img, n, rangeRes: rr, azRes: ra, nCoh };
}

return {
  C, KB, T0, DEG, RAD, dB, unDb, clamp, wavelength, makeRng, fft,
  buildArray, makeHealth, buildWeights, gainEval, monopulseEval, powerCutLine, cutAz, cutEl, patternMap, analyzeCut, elementGain0,
  makeWaveform, matchedFilter, crossMatch, welchCostas, isCostas, BARKER13,
  gasLossTwoWayDb, rainLossTwoWayDb, rainCoeff, stcLossDb, eclipseLossFraction, foldDoppler, horizonKm, radarEquation,
  pdFixedThreshold, pdBurst, requiredSnrDb, cfarAlpha, cfarLossDb, albersheimDb, binomTail, pbTail, solvePerBurstPfa, mcDetect, resolveRange,
  dpcaResponse, sarResolution, sarRender, terrainSigma0Db, defaultConfig, MODE_PRESETS, PRF_SETS, ACM_SUBMODES, SCENARIOS, createScenario, RadarSim, cart
};
});
