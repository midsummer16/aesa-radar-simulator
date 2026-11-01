/* simulator_app.js — AESA 火控雷达显控台：所有显示与参数均来自 radar_engine.js */
(function () {
  'use strict';
  const RE = window.RadarEngine;
  const $ = id => document.getElementById(id);
  const D2R = Math.PI / 180, R2D = 180 / Math.PI;
  const clamp = RE.clamp;
  const MODES = ['RWS', 'TWS', 'TAS', 'MTT', 'DTT', 'STT', 'ACM', 'GMTI', 'GM', 'SAR'];
  const C = { bg: '#02070b', grid: '#12304a', txt: '#d6e8f8', dim: '#7b9ab8', green: '#00f59b', cyan: '#00d8ff', amber: '#ffb800', red: '#ff3b5c', purple: '#b975ff' };

  const state = {
    sim: null, scenario: 'default', targetId: null, jammerId: null,
    paused: false, speed: 1, truth: false, slew: false,
    mfdView: 'bscope', pattern: 'az', steer: 'target', arrayPhase: false,
    rangeDisplay: 160, diag: null, diagId: null, sarCache: null, scanBox: null, mfdBox: null
  };

  /* ---------------------------------------------------------- helpers */
  const fmt = (v, n) => (Number.isFinite(v) ? Number(v).toFixed(n) : '--');
  const unDb = x => Math.pow(10, x / 10);
  const pct = v => clamp(v * 100, 0, 100).toFixed(0) + '%';
  const target = () => state.sim && state.sim.targets.find(t => t.id === state.targetId);
  const jammer = () => state.sim && state.sim.jammers.find(j => j.id === state.jammerId);
  const beamNow = () => state.sim.currentBeam || state.sim.scanList[state.sim.scanIdx % state.sim.scanList.length] || { az: 0, el: 0 };
  const viewKm = () => Math.max(5, Math.min(state.rangeDisplay, state.sim.rangeLimitM() / 1000 || state.rangeDisplay));
  const setText = (id, v) => { const el = $(id); if (el) el.textContent = v; };
  const setVal = (id, v) => { const el = $(id); if (el) el.value = v; };
  const setChip = (id, on) => { const el = $(id); if (el) el.classList.toggle('active', !!on); };
  const debounce = (fn, ms) => { let t; return function () { clearTimeout(t); const a = arguments; t = setTimeout(() => fn.apply(null, a), ms); }; };
  const nearestBeam = s => {
    let best = 1e9, p = state.sim.scanList[0] || { az: 0, el: 0 };
    const uT = Math.sin(s.az * D2R) * Math.cos(s.el * D2R), vT = Math.sin(s.el * D2R);
    const wT = Math.sqrt(Math.max(0, 1 - uT * uT - vT * vT));
    for (const q of state.sim.scanList) {
      const uP = Math.sin(q.az * D2R) * Math.cos(q.el * D2R), vP = Math.sin(q.el * D2R);
      const wP = Math.sqrt(Math.max(0, 1 - uP * uP - vP * vP));
      const off = Math.acos(clamp(uT * uP + vT * vP + wT * wP, -1, 1)) * R2D;
      if (off < best) { best = off; p = q; }
    }
    return { p, off: best };
  };

  const VERDICT_SHORT = {
    OK: '已检出',
    LOW_SNR: '信噪比不足',
    OUT_OF_SCAN: '扫描扇区外',
    JAMMED: '强干扰压制',
    CLUTTER: '杂波/箔条淹没',
    ECLIPSED: '脉冲遮蔽盲区',
    NO_LOS: '视距遮挡',
    TOO_FAR: '超出量程',
    TX_INHIBIT: '发射闭锁'
  };

  function prep(canvas) {
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    const w = Math.max(40, Math.round(canvas.clientWidth)), h = Math.max(40, Math.round(canvas.clientHeight));
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = C.bg; ctx.fillRect(0, 0, w, h);
    return { ctx, w, h };
  }
  function text(ctx, s, x, y, color, size, align) {
    ctx.fillStyle = color || C.txt;
    ctx.font = `${size || 10}px 'JetBrains Mono', Consolas, monospace`;
    ctx.textAlign = align || 'left'; ctx.textBaseline = 'middle';
    ctx.fillText(s, x, y);
  }
  function axes(ctx, w, h, xmin, xmax, ymin, ymax, xlab, ylab, pad) {
    const m = Object.assign({ l: 40, r: 12, t: 20, b: 24 }, pad || {});
    const box = { x: m.l, y: m.t, w: w - m.l - m.r, h: h - m.t - m.b };
    const X = v => box.x + (v - xmin) / (xmax - xmin) * box.w;
    const Y = v => box.y + box.h - (v - ymin) / (ymax - ymin) * box.h;
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
    for (let i = 0; i <= 5; i++) {
      const x = box.x + box.w * i / 5, y = box.y + box.h * i / 5;
      ctx.beginPath(); ctx.moveTo(x, box.y); ctx.lineTo(x, box.y + box.h); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(box.x, y); ctx.lineTo(box.x + box.w, y); ctx.stroke();
    }
    text(ctx, xlab, box.x + box.w / 2, h - 8, C.dim, 9, 'center');
    ctx.save(); ctx.translate(10, box.y + box.h / 2); ctx.rotate(-Math.PI / 2);
    text(ctx, ylab, 0, 0, C.dim, 9, 'center'); ctx.restore();
    for (let i = 0; i <= 5; i++) {
      const xv = xmin + (xmax - xmin) * i / 5, yv = ymax - (ymax - ymin) * i / 5;
      text(ctx, fmt(xv, 0), box.x + box.w * i / 5, box.y + box.h + 10, C.dim, 8, 'center');
      text(ctx, fmt(yv, 0), box.x - 3, box.y + box.h * i / 5, C.dim, 8, 'right');
    }
    return { box, X, Y, xmin, xmax, ymin, ymax };
  }
  function polyline(ctx, pts, color, width, dash) {
    if (!pts.length) return;
    ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = width || 1;
    if (dash) ctx.setLineDash(dash);
    ctx.beginPath(); ctx.moveTo(pts[0][0], pts[0][1]);
    for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
    ctx.stroke(); ctx.restore();
  }
  function dot(ctx, x, y, r, color, fill) {
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI);
    if (fill) { ctx.fillStyle = color; ctx.fill(); } else { ctx.strokeStyle = color; ctx.lineWidth = 1.2; ctx.stroke(); }
  }
  function targetSymbol(ctx, x, y, color, label, locked, dashed) {
    ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = 1.2;
    if (dashed) ctx.setLineDash([2, 2]);
    ctx.beginPath(); ctx.moveTo(x, y - 6); ctx.lineTo(x + 6, y); ctx.lineTo(x, y + 6); ctx.lineTo(x - 6, y); ctx.closePath(); ctx.stroke();
    if (locked) { ctx.setLineDash([]); ctx.strokeRect(x - 11, y - 11, 22, 22); }
    if (label) text(ctx, label, x + 9, y - 9, color, 8);
    ctx.restore();
  }
  function drawTrackSymbol(ctx, x, y, tr, sim, vxScreen, vyScreen) {
    const isLocked = tr.id === sim.lockId || sim.lockIds.includes(tr.id);
    const age = sim.time - tr.lastHit;
    const isCoasting = (tr.missDw >= 2) || (age > Math.max(2.0, sim.derived.frameS * 1.25));
    ctx.save();
    let col = C.cyan;
    if (tr.isFalse) col = C.amber;
    else if (isLocked) col = C.red;
    else if (isCoasting) col = C.amber;
    else if (!tr.confirmed) col = '#7b9ab8';

    ctx.strokeStyle = col;
    ctx.lineWidth = isLocked ? 1.8 : 1.3;
    if (!tr.confirmed || isCoasting) ctx.setLineDash([3, 2]);

    // 战术航迹框 (HAFU 风格)
    ctx.strokeRect(x - 6, y - 6, 12, 12);
    if (isLocked) {
      ctx.setLineDash([]);
      ctx.strokeStyle = C.red;
      ctx.beginPath();
      ctx.moveTo(x, y - 12); ctx.lineTo(x + 12, y); ctx.lineTo(x, y + 12); ctx.lineTo(x - 12, y);
      ctx.closePath(); ctx.stroke();
    }
    // 速度矢量线
    if (Number.isFinite(vxScreen) && Number.isFinite(vyScreen) && (Math.abs(vxScreen) + Math.abs(vyScreen) > 0.5)) {
      ctx.setLineDash([]);
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x + vxScreen, y + vyScreen);
      ctx.stroke();
    }
    const statusTag = tr.isFalse ? ' [GHOST]' : (isLocked ? ' [LOCK]' : (isCoasting ? ' [MEM]' : (!tr.confirmed ? ` [${tr.hits}/3]` : '')));
    text(ctx, `${tr.id}${statusTag}`, x + 9, y + 7, col, 8);
    ctx.restore();
  }
  function beamEllipse(ctx, m, az, el, wDeg, hDeg, stroke, fill) {
    const px = Math.abs(m.X(az + wDeg / 2) - m.X(az - wDeg / 2)) / 2;
    const py = Math.abs(m.Y(el + hDeg / 2) - m.Y(el - hDeg / 2)) / 2;
    ctx.save(); ctx.beginPath(); ctx.ellipse(m.X(az), m.Y(el), Math.max(1, px), Math.max(1, py), 0, 0, 2 * Math.PI);
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.stroke(); }
    ctx.restore();
  }

  /* ---------------------------------------------------------- MFD */
  function drawMFD() {
    const sim = state.sim; if (!sim) return;
    const { ctx, w, h } = prep($('mfdCanvas'));
    const autoStr = sim.cfg.autoMode ? 'AUTO' : 'MAN';
    text(ctx, `FCR MFD · ${sim.cfg.mode} [${autoStr}] · ${sim.tacticalPhase} · T+${sim.time.toFixed(1)}s`, 10, 11, C.green, 10);
    if (state.mfdView === 'bscope') drawBscope(ctx, w, h);
    else if (state.mfdView === 'ppi') drawPPI(ctx, w, h);
    else if (state.mfdView === 'hud') drawHUD(ctx, w, h);
    else drawImaging(ctx, w, h);
  }

  function drawBscope(ctx, w, h) {
    const sim = state.sim, cfg = sim.cfg, d = sim.derived, beam = beamNow();
    const xmin = cfg.scan.azCenter - cfg.scan.azHalf - 5, xmax = cfg.scan.azCenter + cfg.scan.azHalf + 5;
    const rmax = viewKm();
    const m = axes(ctx, w, h, xmin, xmax, 0, rmax, '方位 Az (°)', '斜距 R (km)');
    state.mfdBox = { kind: 'bscope', m };
    ctx.save(); ctx.fillStyle = 'rgba(0,245,155,0.05)';
    const x0 = m.X(cfg.scan.azCenter - cfg.scan.azHalf), x1 = m.X(cfg.scan.azCenter + cfg.scan.azHalf);
    ctx.fillRect(Math.min(x0, x1), m.box.y, Math.abs(x1 - x0), m.box.h); ctx.restore();
    const bw = Math.max(0.4, d.hpbwDeg);
    ctx.save(); ctx.fillStyle = beam.kind === 'track' ? 'rgba(255,59,92,0.18)' : 'rgba(0,245,155,0.16)';
    ctx.fillRect(m.X(beam.az - bw / 2), m.box.y, Math.max(2, m.X(beam.az + bw / 2) - m.X(beam.az - bw / 2)), m.box.h);
    ctx.restore();

    // 电子战干扰方位告警扇区 (ESM strobe)
    for (const j of sim.jammers) {
      const x = m.X(j.azDeg);
      if (x < m.box.x || x > m.box.x + m.box.w) continue;
      const isChaff = j.type === 'chaff';
      polyline(ctx, [[x, m.box.y], [x, m.box.y + m.box.h]], isChaff ? 'rgba(255,184,0,0.45)' : 'rgba(255,59,92,0.55)', 1, [4, 4]);
      text(ctx, `${j.id}:${j.type.toUpperCase()}`, x + 3, m.box.y + 10, isChaff ? C.amber : C.red, 8);
    }

    // 1. 仅绘制实际通过雷达检测的点迹 (plots)
    for (const p of sim.plots) {
      if (p.R / 1000 > rmax) continue;
      const x = m.X(p.az), y = m.Y(p.R / 1000);
      if (x < m.box.x || x > m.box.x + m.box.w) continue;
      if (p.isFalse) {
        const isDecep = p.kind === 'drfm' || p.kind === 'vgpo' || p.kind === 'rvgpo';
        ctx.save(); ctx.strokeStyle = isDecep ? C.amber : C.red; ctx.lineWidth = 1.1;
        ctx.beginPath(); ctx.moveTo(x - 3, y - 3); ctx.lineTo(x + 3, y + 3); ctx.moveTo(x + 3, y - 3); ctx.lineTo(x - 3, y + 3); ctx.stroke(); ctx.restore();
      } else {
        dot(ctx, x, y, p.sidelobe ? 3 : 2.5, p.sidelobe ? C.amber : C.green, true);
      }
    }

    // 2. 绘制建立的航迹 (tracks)
    for (const tr of sim.tracks) {
      const dt = sim.time - tr.t;
      const tx = tr.x + tr.vx * dt, ty = tr.y + tr.vy * dt, tz = tr.z + tr.vz * dt;
      const R = Math.hypot(tx, ty, tz) / 1000, az = Math.atan2(tx, ty) * R2D;
      if (R > rmax) continue;
      const x = m.X(az), y = m.Y(R);
      if (x < m.box.x || x > m.box.x + m.box.w) continue;
      // 预测 8s 后的屏幕位置作为速度矢量线
      const tx2 = tx + tr.vx * 8, ty2 = ty + tr.vy * 8, tz2 = tz + tr.vz * 8;
      const R2 = Math.hypot(tx2, ty2, tz2) / 1000, az2 = Math.atan2(tx2, ty2) * R2D;
      const vxScr = clamp(m.X(az2) - x, -22, 22), vyScr = clamp(m.Y(R2) - y, -22, 22);
      drawTrackSymbol(ctx, x, y, tr, sim, vxScr, vyScr);
    }

    // 3. 若锁定目标，在右侧绘制武器动态发射包线 (DLZ: Rmax / Rne / Rmin)
    const lockTr = sim.tracks.find(t => t.id === sim.lockId);
    if (lockTr) {
      const dlzX = m.box.x + m.box.w - 18;
      const rAero = Math.min(rmax * 0.85, 85), rNe = Math.min(rmax * 0.45, 42), rMin = 3;
      const yMax = m.Y(rAero), yNe = m.Y(rNe), yMin = m.Y(rMin);
      polyline(ctx, [[dlzX, yMax], [dlzX, yMin]], C.red, 2, null);
      polyline(ctx, [[dlzX - 5, yMax], [dlzX + 5, yMax]], C.red, 1.5, null);
      polyline(ctx, [[dlzX - 5, yNe], [dlzX + 5, yNe]], C.amber, 1.5, null);
      polyline(ctx, [[dlzX - 5, yMin], [dlzX + 5, yMin]], C.red, 1.5, null);
      text(ctx, 'Rmax', dlzX - 7, yMax, C.red, 7, 'right');
      text(ctx, 'Rne', dlzX - 7, yNe, C.amber, 7, 'right');
    }

    // 4. 默认严格不显示未检出目标！仅在用户主动打开 TRU 调试开关时绘制虚线真值对比框
    if (state.truth) {
      sim.targets.forEach(tg => {
        const s = sim.targetState(tg, sim.time); if (s.R / 1000 > rmax) return;
        const st = sim.getTargetTacticalStatus(tg);
        targetSymbol(ctx, m.X(s.az), m.Y(s.R / 1000), st.isDetected ? C.purple : 'rgba(185,117,255,0.45)', `${tg.id}[真值]`, false, !st.isDetected);
      });
    }
    text(ctx, `扫描 ${cfg.scan.azCenter.toFixed(0)}°±${cfg.scan.azHalf.toFixed(0)}° / ${cfg.scan.bars} bars · 波束: ${beam.kind === 'track' ? '敏捷跟踪' : '搜索'}`, m.box.x + 4, m.box.y + 10, C.dim, 9);
  }

  function drawHUD(ctx, w, h) {
    const sim = state.sim, cfg = sim.cfg, d = sim.derived;
    const m = axes(ctx, w, h, -60, 60, -25, 25, '方位 Az (°)', '俯仰 El (°)');
    state.mfdBox = { kind: 'hud', m };
    ctx.save(); ctx.strokeStyle = 'rgba(0,245,155,0.25)';
    ctx.rect(m.X(cfg.scan.azCenter - cfg.scan.azHalf), m.Y(cfg.scan.elCenter + 12), Math.abs(m.X(cfg.scan.azCenter + cfg.scan.azHalf) - m.X(cfg.scan.azCenter - cfg.scan.azHalf)), Math.abs(m.Y(cfg.scan.elCenter - 12) - m.Y(cfg.scan.elCenter + 12)));
    ctx.stroke(); ctx.restore();
    for (const p of sim.scanList) beamEllipse(ctx, m, p.az, p.el, d.hpbwDeg, d.hpbwElDeg, 'rgba(0,216,255,0.28)', null);
    const beam = beamNow(); beamEllipse(ctx, m, beam.az, beam.el, d.hpbwDeg, d.hpbwElDeg, beam.kind === 'track' ? C.red : C.green, 'rgba(0,245,155,0.18)');
    ctx.save(); ctx.strokeStyle = C.dim; ctx.beginPath(); ctx.moveTo(m.X(0), m.Y(-25)); ctx.lineTo(m.X(0), m.Y(25)); ctx.moveTo(m.X(-60), m.Y(0)); ctx.lineTo(m.X(60), m.Y(0)); ctx.stroke(); ctx.restore();
    for (const p of sim.plots) {
      if (p.isFalse) continue;
      dot(ctx, m.X(p.az), m.Y(p.el), 2.4, p.sidelobe ? C.amber : C.green, true);
    }
    for (const tr of sim.tracks) {
      const dt = sim.time - tr.t;
      const tx = tr.x + tr.vx * dt, ty = tr.y + tr.vy * dt, tz = tr.z + tr.vz * dt;
      const az = Math.atan2(tx, ty) * R2D, el = Math.atan2(tz, Math.hypot(tx, ty)) * R2D;
      drawTrackSymbol(ctx, m.X(az), m.Y(el), tr, sim, 0, 0);
    }
    if (state.truth) {
      sim.targets.forEach(tg => {
        const s = sim.targetState(tg, sim.time);
        const st = sim.getTargetTacticalStatus(tg);
        targetSymbol(ctx, m.X(s.az), m.Y(s.el), st.isDetected ? C.purple : 'rgba(185,117,255,0.45)', `${tg.id}[真值]`, false, !st.isDetected);
      });
    }
    for (const j of sim.jammers) text(ctx, `${j.id}:${j.type}`, m.X(j.azDeg), m.Y(clamp(j.elDeg, -24, 24)) - 9, C.red, 9, 'center');
    text(ctx, `HUD · 扫描中心 ${cfg.scan.azCenter.toFixed(0)}° / ${cfg.scan.elCenter.toFixed(1)}°`, m.box.x + 4, m.box.y + 10, C.dim, 9);
  }

  function drawPPI(ctx, w, h) {
    const sim = state.sim, cfg = sim.cfg, beam = beamNow();
    const rmax = viewKm();
    const cx = w / 2, cy = h * 0.9, rad = Math.min(w * 0.47, h * 0.82);
    state.mfdBox = { kind: 'ppi', cx, cy, rad, rmax };
    const a0 = (cfg.scan.azCenter - cfg.scan.azHalf - 90) * D2R, a1 = (cfg.scan.azCenter + cfg.scan.azHalf - 90) * D2R;
    ctx.save(); ctx.fillStyle = 'rgba(0,245,155,0.05)';
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.arc(cx, cy, rad, a0, a1); ctx.closePath(); ctx.fill(); ctx.restore();
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
    for (let k = 1; k <= 4; k++) { ctx.beginPath(); ctx.arc(cx, cy, rad * k / 4, 0, 2 * Math.PI); ctx.stroke(); }
    for (let a = -180; a < 180; a += 30) { const r = a * D2R; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + rad * Math.sin(r), cy - rad * Math.cos(r)); ctx.stroke(); }
    const P = (R, az) => { const rr = clamp(R / rmax, 0, 1) * rad, a = az * D2R; return [cx + rr * Math.sin(a), cy - rr * Math.cos(a)]; };
    const sector = [P(rmax, cfg.scan.azCenter - cfg.scan.azHalf), P(rmax, cfg.scan.azCenter + cfg.scan.azHalf)];
    polyline(ctx, sector, 'rgba(0,245,155,0.55)', 1.5);
    const b0 = P(rmax, beam.az); polyline(ctx, [[cx, cy], b0], beam.kind === 'track' ? 'rgba(255,59,92,0.55)' : 'rgba(0,245,155,0.35)', 1, [3, 4]);
    for (const p of sim.plots) {
      const R = p.R / 1000; if (R > rmax) continue; const q = P(R, p.az);
      if (p.isFalse) { dot(ctx, q[0], q[1], 2, p.kind === 'drfm' ? C.amber : C.red, true); }
      else dot(ctx, q[0], q[1], 2.5, p.sidelobe ? C.amber : C.green, true);
    }
    for (const tr of sim.tracks) {
      const dt = sim.time - tr.t;
      const tx = tr.x + tr.vx * dt, ty = tr.y + tr.vy * dt, tz = tr.z + tr.vz * dt;
      const R = Math.hypot(tx, ty, tz) / 1000, az = Math.atan2(tx, ty) * R2D;
      if (R > rmax) continue;
      const q = P(R, az);
      drawTrackSymbol(ctx, q[0], q[1], tr, sim, 0, 0);
    }
    if (state.truth) {
      sim.targets.forEach(tg => {
        const s = sim.targetState(tg, sim.time); if (s.R / 1000 > rmax) return;
        const st = sim.getTargetTacticalStatus(tg);
        const q = P(s.R / 1000, s.az);
        targetSymbol(ctx, q[0], q[1], st.isDetected ? C.purple : 'rgba(185,117,255,0.45)', `${tg.id}[真值]`, false, !st.isDetected);
      });
    }
    for (const j of sim.jammers) {
      const q = P(j.rangeKm, j.azDeg), q2 = P(j.rangeKm * 0.5, j.azDeg);
      polyline(ctx, [q2, q], 'rgba(255,59,92,0.55)', 1, [4, 4]); text(ctx, `${j.id}:${j.type}`, q[0] + 4, q[1], C.red, 8);
    }
    text(ctx, `PPI ${rmax} km`, 8, 26, C.dim, 9);
    text(ctx, `${rmax >> 1} km`, cx, cy - rad / 2 + 10, C.dim, 8, 'center');
  }

  function sarImage() {
    const sim = state.sim, cfg = sim.cfg, d = sim.derived;
    const R0 = clamp(sim.targets.length ? sim.targetState(sim.targets[0], sim.time).R : 40000, 5000, 200000);
    const key = [cfg.mode, cfg.fGHz, cfg.tx.bwMHz, cfg.own.speed, R0.toFixed(0), sim.targets.map(t => t.id + ':' + t.rcs).join(',')].join('|');
    if (state.sarCache && state.sarCache.key === key) return state.sarCache.img;
    const scene = [];
    sim.targets.forEach(tg => {
      const diag = sim.analyzeTarget(tg);
      if (diag.verdict !== 'OK') return; // 未检出目标不在成像场景中显示
      const s = sim.targetState(tg, sim.time);
      if (Math.abs(s.R - R0) < 900) scene.push({ x: Math.sin(s.az * D2R) * R0, y: s.R - R0, rcs: tg.rcs });
    });
    const rng = RE.makeRng(77);
    for (let i = 0; i < 28; i++) scene.push({ x: rng.range(-900, 900), y: rng.range(-900, 900), rcs: rng.range(0.2, 8) });
    const img = RE.sarRender({
      bwHz: cfg.tx.bwMHz * 1e6, lambda: d.lambda, R0, Va: cfg.own.speed, Tsar: 2,
      prf: d.prfs[0], PtW: d.PtW, tau: d.tau, G: unDb(d.g0Db), nfDb: cfg.rx.nfDb,
      lossDb: cfg.rx.lossDb, atmDb: 1, scene, n: 128, extentM: 2200, seed: 3
    });
    state.sarCache = { key, img };
    return img;
  }

  function drawImaging(ctx, w, h) {
    const sim = state.sim, cfg = sim.cfg;
    state.mfdBox = null;
    if (cfg.mode === 'GM') {
      if (!sim.gm) { text(ctx, 'GM 实波束成像：等待扫描波束照射地面…', w / 2, h / 2, C.dim, 11, 'center'); return; }
      const g = sim.gm, box = { x: 10, y: 20, w: w - 20, h: h - 34 };
      const cw = box.w / g.nA, ch = box.h / g.nR;
      for (let r = 0; r < g.nR; r++) for (let a = 0; a < g.nA; a++) {
        const v = g.v[r * g.nA + a]; if (v < 0.02) continue;
        const q = Math.round(255 * clamp(v, 0, 1));
        ctx.fillStyle = `rgb(${q * 0.35 | 0},${q},${q * 0.7 | 0})`;
        ctx.fillRect(box.x + a * cw, box.y + box.h - (r + 1) * ch, cw + 0.6, ch + 0.6);
      }
      text(ctx, `GM 实波束地图 · ${g.rKm} km · ${g.nR}×${g.nA}`, box.x, 11, C.green, 10);
      text(ctx, '近', box.x, box.y + box.h + 8, C.dim, 8); text(ctx, '远', box.x + box.w, box.y + box.h + 8, C.dim, 8, 'right');
      return;
    }
    if (cfg.mode === 'SAR') {
      const img = sarImage(), box = { x: 10, y: 20, w: w - 20, h: h - 34 };
      let mn = 1e9, mx = -1e9;
      for (let i = 0; i < img.img.length; i++) { if (img.img[i] < mn) mn = img.img[i]; if (img.img[i] > mx) mx = img.img[i]; }
      const span = Math.max(1, mx - mn);
      for (let iy = 0; iy < img.n; iy++) for (let ix = 0; ix < img.n; ix++) {
        const v = (img.img[iy * img.n + ix] - mn) / span;
        const q = Math.round(255 * Math.pow(clamp(v, 0, 1), 0.7));
        ctx.fillStyle = `rgb(${q * 0.4 | 0},${q},${q * 0.75 | 0})`;
        ctx.fillRect(box.x + ix * box.w / img.n, box.y + iy * box.h / img.n, box.w / img.n + 0.6, box.h / img.n + 0.6);
      }
      text(ctx, `SAR 点扩散成像 · ΔR ${img.rangeRes.toFixed(1)} m / ΔAz ${img.azRes.toFixed(1)} m`, box.x, 11, C.green, 10);
      return;
    }
    text(ctx, '请选择 GM 或 SAR 模式后再使用成像视图', w / 2, h / 2, C.dim, 11, 'center');
  }

  /* ---------------------------------------------------------- scan pattern */
  function drawScan() {
    const sim = state.sim; if (!sim) return;
    const { ctx, w, h } = prep($('scanCanvas'));
    const d = sim.derived, cfg = sim.cfg;
    const m = axes(ctx, w, h, -75, 75, -30, 30, '方位 Az (°)', '俯仰 El (°)');
    state.scanBox = m;
    let e0 = cfg.scan.elCenter, e1 = cfg.scan.elCenter;
    for (const p of sim.scanList) { e0 = Math.min(e0, p.el); e1 = Math.max(e1, p.el); }
    e0 -= Math.max(2, d.hpbwElDeg / 2); e1 += Math.max(2, d.hpbwElDeg / 2);
    ctx.save(); ctx.fillStyle = 'rgba(0,216,255,0.05)';
    const x0 = m.X(cfg.scan.azCenter - cfg.scan.azHalf), x1 = m.X(cfg.scan.azCenter + cfg.scan.azHalf);
    const y0 = m.Y(e0), y1 = m.Y(e1);
    ctx.fillRect(Math.min(x0, x1), Math.min(y0, y1), Math.abs(x1 - x0), Math.abs(y1 - y0)); ctx.restore();
    ctx.setLineDash([4, 3]); ctx.strokeStyle = 'rgba(0,216,255,0.5)';
    ctx.strokeRect(Math.min(x0, x1), Math.min(y0, y1), Math.abs(x1 - x0), Math.abs(y1 - y0)); ctx.setLineDash([]);
    const cur = beamNow();
    for (const p of sim.scanList) {
      const broad = 1 / Math.max(0.35, Math.cos(p.az * D2R));
      const active = p === cur || (Math.abs(p.az - cur.az) < 1e-6 && Math.abs(p.el - cur.el) < 1e-6);
      beamEllipse(ctx, m, p.az, p.el, d.hpbwDeg * broad, d.hpbwElDeg, active ? C.green : 'rgba(0,216,255,0.35)', active ? 'rgba(0,245,155,0.28)' : null);
    }

    // 仅绘制实际检出的点迹与航迹，未检出目标不在扫描图上显示（除非开启 TRU 调试）
    for (const p of sim.plots) if (!p.isFalse) dot(ctx, m.X(p.az), m.Y(p.el), 2.2, C.green, true);
    for (const tr of sim.tracks) {
      const dt = sim.time - tr.t;
      const tx = tr.x + tr.vx * dt, ty = tr.y + tr.vy * dt, tz = tr.z + tr.vz * dt;
      const az = Math.atan2(tx, ty) * R2D, el = Math.atan2(tz, Math.hypot(tx, ty)) * R2D;
      drawTrackSymbol(ctx, m.X(az), m.Y(el), tr, sim, 0, 0);
    }
    if (state.truth) {
      sim.targets.forEach(tg => {
        const s = sim.targetState(tg, sim.time);
        const nb = nearestBeam(s), sel = tg.id === state.targetId;
        const st = sim.getTargetTacticalStatus(tg);
        targetSymbol(ctx, m.X(s.az), m.Y(s.el), sel ? C.purple : 'rgba(185,117,255,0.55)', `${tg.id}[真值]`, false, !st.isDetected);
        if (sel) polyline(ctx, [[m.X(s.az), m.Y(s.el)], [m.X(nb.p.az), m.Y(nb.p.el)]], C.amber, 1, [3, 3]);
      });
    }

    const tg = target(), diag = state.diag && state.diag.id === state.targetId ? state.diag : null;
    const badge = $('scanMatchBadge');
    if (badge) {
      if (!tg) { badge.textContent = '无诊断目标'; badge.style.color = C.dim; }
      else if (!diag) { badge.textContent = '正在解算扫描匹配…'; badge.style.color = C.cyan; }
      else {
        const off = (diag.scanOffsetDeg || 0).toFixed(1), loss = (diag.scanLossDb || 0).toFixed(1);
        if (diag.verdict === 'OUT_OF_SCAN') { badge.textContent = `未检出(画面已隐藏): 目标在扫描外 · 偏角 ${off}° / 失配 ${loss} dB`; badge.style.color = C.red; }
        else if (diag.verdict === 'OK') { badge.textContent = `波束正常覆盖且已检出 · 偏角 ${off}° / 失配 ${loss} dB / Pd ${pct(diag.pdSys)}`; badge.style.color = C.green; }
        else { badge.textContent = `未检出(画面已隐藏): 判决 ${diag.verdict} (${VERDICT_SHORT[diag.verdict] || ''}) · 偏角 ${off}°`; badge.style.color = C.amber; }
      }
    }
  }

  /* ---------------------------------------------------------- aperture */
  function drawArray() {
    const sim = state.sim; if (!sim) return;
    const { ctx, w, h } = prep($('arrayCanvas'));
    const beam = beamNow();
    const a = sim.arrayMap(beam.az, beam.el);
    const cx = w / 2, cy = h / 2, rad = Math.min(w, h) * 0.46;
    ctx.strokeStyle = C.grid; ctx.beginPath(); ctx.arc(cx, cy, rad, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx - rad, cy); ctx.lineTo(cx + rad, cy); ctx.moveTo(cx, cy - rad); ctx.lineTo(cx, cy + rad); ctx.stroke();
    let mx = 0; for (let i = 0; i < a.N; i++) if (a.amp[i] > mx) mx = a.amp[i];
    mx = Math.max(mx, 1e-9);
    for (let i = 0; i < a.N; i++) {
      const x = cx + a.x[i] / a.R * rad, y = cy - a.y[i] / a.R * rad;
      if (!a.health[i]) { ctx.fillStyle = C.red; ctx.fillRect(x - 1.3, y - 1.3, 2.6, 2.6); continue; }
      if (state.arrayPhase) {
        const hue = ((a.phase[i] * R2D) % 360 + 360) % 360;
        ctx.fillStyle = `hsl(${hue},85%,55%)`;
      } else {
        const q = Math.pow(clamp(a.amp[i] / mx, 0, 1), 0.55);
        ctx.fillStyle = `rgb(${Math.round(40 * q)},${Math.round(120 + 135 * q)},${Math.round(120 + 135 * q)})`;
      }
      ctx.fillRect(x - 1.05, y - 1.05, 2.1, 2.1);
    }
    text(ctx, `${state.arrayPhase ? '相位 φn' : '幅度 In'} · N=${a.N} · 健康 ${sim.derived.healthy}`, 6, 10, C.green, 9);
    text(ctx, '点击切换 幅/相', 6, h - 8, C.dim, 8);
  }

  /* ---------------------------------------------------------- pattern */
  function drawPattern() {
    const sim = state.sim; if (!sim) return;
    const { ctx, w, h } = prep($('patternCanvas'));
    const tg = target();
    const st = tg ? sim.getTargetTacticalStatus(tg) : null;
    const beam = state.steer === 'target' && tg && (st.isDetected || state.truth) ? sim.targetState(tg, sim.time) : beamNow();
    const az = beam.az, el = beam.el;
    text(ctx, `${state.steer === 'target' && tg && (st.isDetected || state.truth) ? '指向已检出目标' : '指向当前波束'} Az ${az.toFixed(1)}° / El ${el.toFixed(1)}°`, 8, 11, C.green, 10);
    if (state.pattern === '2d') { drawPattern2D(ctx, w, h, az, el, tg, st); return; }
    const p = sim.beamPattern(az, el, { points: 401, spanDeg: 50 });
    const cut = state.pattern === 'el' ? p.cutEl : p.cutAz;
    const angles = Array.from(cut.ang), txD = Array.from(cut.txDb), rxD = Array.from(cut.rxDb), twD = Array.from(cut.twoDb);
    let mx = -1e9; for (const v of twD) if (v > mx) mx = v;
    const ymin = mx - 55, ymax = mx + 3;
    const a0 = angles[0], a1 = angles[angles.length - 1];
    const m = axes(ctx, w, h, a0, a1, ymin, ymax, state.pattern === 'el' ? '俯仰 El (°)' : '方位 Az (°)', '增益 (dBi)', { l: 42, t: 26 });
    const line = (arr, color, width, dash) => polyline(ctx, arr.map((v, i) => [m.X(angles[i]), m.Y(clamp(v, ymin, ymax))]), color, width, dash);
    line(txD, 'rgba(0,216,255,0.5)', 1, [4, 3]);
    line(rxD, 'rgba(0,245,155,0.55)', 1, [2, 3]);
    line(twD, C.amber, 1.8, null);

    // 标示干扰零陷方向
    for (const j of sim.jammers) {
      const jAng = state.pattern === 'el' ? j.elDeg : j.azDeg;
      if (jAng > a0 && jAng < a1) {
        polyline(ctx, [[m.X(jAng), m.box.y], [m.X(jAng), m.box.y + m.box.h]], 'rgba(255,59,92,0.55)', 1, [3, 3]);
        text(ctx, `${j.id}${sim.cfg.eccm.adbf ? '[零陷]' : ''}`, m.X(jAng) + 3, m.box.y + 22, C.red, 8);
      }
    }

    // 仅当目标被检出或打开真值调试时才在方向图上绘制目标位置线
    if (tg && ((st && st.isDetected) || state.truth)) {
      const tgtAng = state.pattern === 'el' ? sim.targetState(tg, sim.time).el : sim.targetState(tg, sim.time).az;
      if (tgtAng > a0 && tgtAng < a1) {
        polyline(ctx, [[m.X(tgtAng), m.box.y], [m.X(tgtAng), m.box.y + m.box.h]], C.purple, 1, [2, 4]);
        text(ctx, `${tg.id}`, m.X(tgtAng) + 3, m.box.y + 12, C.purple, 8);
      }
    }
    polyline(ctx, [[m.box.x, m.Y(mx - 3)], [m.box.x + m.box.w, m.Y(mx - 3)]], 'rgba(255,255,255,0.18)', 1, [3, 4]);
    text(ctx, `峰值 ${mx.toFixed(1)} dBi · HPBW ${(state.pattern === 'el' ? p.hpbwEl : p.hpbwAz).toFixed(2)}° · SLL ${(state.pattern === 'el' ? p.sllEl : p.sllAz).toFixed(1)} dB`, m.box.x + 4, m.box.y + 10, C.dim, 9);
    text(ctx, '和波束两程', m.box.x + m.box.w - 70, m.box.y + 10, C.amber, 8);
  }

  function drawPattern2D(ctx, w, h, az, el, tg, st) {
    const sim = state.sim;
    const p = sim.beamPattern(az, el, { cut: '2d', nu: 49, nv: 49, spanDeg: 28 });
    const g = p.map, box = { x: 34, y: 24, w: w - 46, h: h - 40 };
    let mx = -1e9; for (const v of g.db) if (v > mx) mx = v;
    const cellW = box.w / g.nu, cellH = box.h / g.nv;
    for (let j = 0; j < g.nv; j++) for (let i = 0; i < g.nu; i++) {
      const v = clamp((g.db[j * g.nu + i] - (mx - 40)) / 40, 0, 1);
      const r = Math.round(255 * clamp(1.8 * v - 0.7, 0, 1)), gg = Math.round(255 * clamp(1.6 * v - 0.25, 0, 1)), b = Math.round(255 * clamp(1.3 * v, 0, 1));
      ctx.fillStyle = `rgb(${r},${gg},${b})`;
      ctx.fillRect(box.x + i * cellW, box.y + box.h - (j + 1) * cellH, cellW + 0.6, cellH + 0.6);
    }
    const uBeam = Math.sin(az * D2R) * Math.cos(el * D2R), vBeam = Math.sin(el * D2R);
    const X = u => box.x + (u - g.uMin) / (g.du * (g.nu - 1)) * box.w;
    const Y = v => box.y + box.h - (v - g.vMin) / (g.dv * (g.nv - 1)) * box.h;
    ctx.strokeStyle = C.green; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(X(uBeam) - 6, Y(vBeam)); ctx.lineTo(X(uBeam) + 6, Y(vBeam)); ctx.moveTo(X(uBeam), Y(vBeam) - 6); ctx.lineTo(X(uBeam), Y(vBeam) + 6); ctx.stroke();
    if (tg && ((st && st.isDetected) || state.truth)) {
      const s = sim.targetState(tg, sim.time);
      const u = Math.sin(s.az * D2R) * Math.cos(s.el * D2R), v = Math.sin(s.el * D2R);
      if (u > g.uMin && u < g.uMin + g.du * (g.nu - 1) && v > g.vMin && v < g.vMin + g.dv * (g.nv - 1)) {
        dot(ctx, X(u), Y(v), 5, C.purple, false); text(ctx, tg.id, X(u) + 7, Y(v) - 8, C.purple, 8);
      }
    }
    text(ctx, `2D (u,v) 两程方向图 · 峰值 ${mx.toFixed(1)} dBi`, box.x, 11, C.green, 10);
    text(ctx, 'u', box.x + box.w / 2, h - 8, C.dim, 9, 'center');
    text(ctx, 'v', 10, box.y + box.h / 2, C.dim, 9, 'center');
  }

  /* ---------------------------------------------------------- waveform */
  function drawWaveform() {
    const sim = state.sim; if (!sim) return;
    const { ctx, w, h } = prep($('waveCanvas'));
    const wf = sim.wf, mf = sim.mf;
    const rowH = (h - 16) / 3;
    const drawRow = (y, title, drawFn) => {
      ctx.strokeStyle = C.grid; ctx.strokeRect(8, y, w - 16, rowH - 4);
      text(ctx, title, 12, y + 9, C.dim, 8); drawFn(y + 14, rowH - 22);
    };
    drawRow(4, `I/Q 基带 · ${wf.type.toUpperCase()} τ=${(wf.tau * 1e6).toFixed(0)}μs B=${(wf.B / 1e6).toFixed(0)}MHz`, (y, hh) => {
      const n = wf.n, step = Math.max(1, Math.floor(n / (w - 24)));
      const re = [], im = [];
      for (let i = 0; i < n; i += step) { const x = 12 + i / (n - 1) * (w - 24); re.push([x, y + hh / 2 - wf.re[i] * hh * 0.44]); im.push([x, y + hh / 2 - wf.im[i] * hh * 0.44]); }
      polyline(ctx, re, C.cyan, 1, null); polyline(ctx, im, C.amber, 1, null);
    });
    drawRow(4 + rowH, '瞬时频率 f(t)', (y, hh) => {
      let mx = 1e-9; const f = [];
      for (let i = 0; i < wf.n - 1; i++) {
        let d = Math.atan2(wf.im[i + 1], wf.re[i + 1]) - Math.atan2(wf.im[i], wf.re[i]);
        if (d > Math.PI) d -= 2 * Math.PI; if (d < -Math.PI) d += 2 * Math.PI;
        const v = d / (2 * Math.PI) * wf.fs; f.push(v); mx = Math.max(mx, Math.abs(v));
      }
      const step = Math.max(1, Math.floor(f.length / (w - 24)));
      const pts = [];
      for (let i = 0; i < f.length; i += step) pts.push([12 + i / (f.length - 1) * (w - 24), y + hh / 2 - f[i] / mx * hh * 0.44]);
      polyline(ctx, pts, C.green, 1, null);
      text(ctx, `${(mx / 1e6).toFixed(1)} MHz`, w - 14, y + 2, C.dim, 8, 'right');
    });
    drawRow(4 + 2 * rowH, `匹配滤波脉压输出 · PSLR ${mf.pslrDb.toFixed(1)} dB · ΔR ${mf.rangeRes.toFixed(1)} m`, (y, hh) => {
      const env = mf.env, pts = [];
      for (let i = 0; i < env.length; i++) pts.push([12 + i / (env.length - 1) * (w - 24), y + hh - clamp(20 * Math.log10(env[i] + 1e-9) / -60, 0, 1) * hh]);
      polyline(ctx, pts, C.amber, 1, null);
    });
    setText('wfInfoBadge', `${wf.type.toUpperCase()} | τ=${(wf.tau * 1e6).toFixed(0)}μs | B=${(wf.B / 1e6).toFixed(0)}MHz | TBP=${(mf.compression).toFixed(0)} | 失配 ${mf.mismatchLossDb.toFixed(2)}dB`);
  }

  /* ---------------------------------------------------------- A-scope */
  function drawAscope() {
    const sim = state.sim; if (!sim) return;
    const { ctx, w, h } = prep($('ascopeCanvas'));
    const beam = beamNow(), rmax = viewKm();
    const asc = sim.synthesizeAscope(beam.az, beam.el, 180, rmax);
    let mx = asc.noiseFloorDb + 6, mn = asc.noiseFloorDb - 8;
    for (const v of asc.powerDb) if (v > mx) mx = v;
    for (const v of asc.cfarDb) if (v > mx) mx = v;
    mn = Math.min(mn, mx - 40);
    const m = axes(ctx, w, h, 0, rmax, mn, mx + 3, '距离 R (km)', '功率 (dB)', { l: 38, t: 18, b: 22 });
    const pts = []; for (let i = 0; i < asc.rKm.length; i++) pts.push([m.X(asc.rKm[i]), m.Y(clamp(asc.powerDb[i], mn, mx + 3))]);
    ctx.save(); ctx.beginPath(); ctx.moveTo(m.X(0), m.Y(mn));
    for (const q of pts) ctx.lineTo(q[0], q[1]);
    ctx.lineTo(m.X(rmax), m.Y(mn)); ctx.closePath(); ctx.fillStyle = 'rgba(0,216,255,0.12)'; ctx.fill(); ctx.restore();
    polyline(ctx, pts, C.cyan, 1, null);
    const cf = []; for (let i = 0; i < asc.rKm.length; i++) cf.push([m.X(asc.rKm[i]), m.Y(clamp(asc.cfarDb[i], mn, mx + 3))]);
    polyline(ctx, cf, C.red, 1, [4, 3]);
    polyline(ctx, [[m.box.x, m.Y(asc.noiseFloorDb)], [m.box.x + m.box.w, m.Y(asc.noiseFloorDb)]], 'rgba(255,255,255,0.18)', 1, [2, 4]);
    for (const det of asc.detections) dot(ctx, m.X(det.rKm), m.Y(clamp(det.snrDb + asc.noiseFloorDb, mn, mx)), 2.6, C.amber, true);

    // 严格仅对实际过 CFAR 门限检出的目标进行标注（未检出目标绝不画绿线或标注代号）
    for (const t of asc.targetsInProfile) {
      if (!t.detected && !state.truth) continue;
      const col = t.detected ? C.green : 'rgba(185,117,255,0.45)';
      polyline(ctx, [[m.X(t.rKm), m.box.y], [m.X(t.rKm), m.box.y + m.box.h]], col, 1, [2, 4]);
      text(ctx, `${t.id} ${t.snrDb.toFixed(0)}dB${t.detected ? '' : '[未检出]'}`, m.X(t.rKm) + 3, m.box.y + 10, col, 8);
    }
    text(ctx, `波束 Az ${beam.az.toFixed(1)}° / El ${beam.el.toFixed(1)}°`, m.box.x + 4, m.box.y + 10, C.dim, 9);
    const cfarType = sim.cfg.eccm.osCfar ? 'OS-CFAR' : 'CA-CFAR';
    setText('cfarInfoBadge', `${cfarType} Nref=${sim.cfg.rx.nRef} | Pfa=${sim.cfg.rx.pfa.toExponential(0)} | M-of-N ${sim.derived.M}/${sim.derived.nB}`);
  }

  /* ---------------------------------------------------------- range-Doppler */
  function drawRD() {
    const sim = state.sim; if (!sim) return;
    const { ctx, w, h } = prep($('rdCanvas'));
    const beam = beamNow();
    const rd = sim.synthesizeRdMap(beam.az, beam.el, 64, 40);
    const box = { x: 40, y: 20, w: w - 52, h: h - 38 };
    let mn = 1e9, mx = -1e9;
    for (const v of rd.gridDb) { if (v < mn) mn = v; if (v > mx) mx = v; }
    const span = Math.max(1, mx - mn), cw = box.w / rd.nR, ch = box.h / rd.nD;
    for (let i = 0; i < rd.nR; i++) for (let j = 0; j < rd.nD; j++) {
      const v = (rd.gridDb[i * rd.nD + j] - mn) / span;
      const r = Math.round(255 * clamp(1.5 * v - 0.6, 0, 1)), g = Math.round(255 * clamp(1.4 * v - 0.2, 0, 1)), b = Math.round(255 * clamp(1.2 * v, 0, 1));
      ctx.fillStyle = `rgb(${r},${g},${b})`;
      ctx.fillRect(box.x + i * cw, box.y + box.h - (j + 1) * ch, cw + 0.6, ch + 0.6);
    }
    // 严格仅对在 R-D 平面跨过检测门限的目标显示绿圈与编号
    for (const mk of rd.markers) {
      if (!mk.detected && !state.truth) continue;
      const col = mk.detected ? C.green : 'rgba(185,117,255,0.45)';
      dot(ctx, box.x + (mk.rIdx + 0.5) * cw, box.y + box.h - (mk.dIdx + 0.5) * ch, 4, col, false);
      text(ctx, mk.id + (mk.detected ? '' : '[隐]'), box.x + (mk.rIdx + 0.5) * cw + 6, box.y + box.h - (mk.dIdx + 0.5) * ch - 7, col, 8);
    }
    text(ctx, `R-D 图 · PRF ${(rd.prfHz / 1e3).toFixed(1)}kHz · Ru ${rd.ruKm.toFixed(1)}km · 噪底 ${rd.noiseFloorDb.toFixed(1)}dB`, box.x, 11, C.green, 9);
    text(ctx, '距离 (折叠)', box.x + box.w / 2, h - 7, C.dim, 8, 'center');
    text(ctx, '多普勒', 10, box.y + box.h / 2, C.dim, 8, 'center');
    text(ctx, '+', box.x - 8, box.y + 5, C.dim, 8, 'right');
    text(ctx, '-', box.x - 8, box.y + box.h, C.dim, 8, 'right');
  }

  /* ---------------------------------------------------------- telemetry / FSM / diagnostics */
  function updateFSMStrip() {
    const sim = state.sim; if (!sim) return;
    const ph = sim.tacticalPhase;
    const setNode = (id, cls) => {
      const el = $(id); if (!el) return;
      el.className = 'fsm-node' + (cls ? ' ' + cls : '');
    };
    setNode('fsmSearch', (ph === 'SEARCH' || ph === 'GMTI_SEARCH' || ph === 'IMAGING') ? 'active' : '');
    setNode('fsmAcquire', (ph === 'ACQUIRE' || ph === 'WVR_ACQUIRE') ? 'active' : '');
    setNode('fsmTrack', (ph === 'TRACK_SCAN' || ph === 'GMTT_TRACK') ? 'active' : '');
    setNode('fsmMulti', (ph === 'MULTI_TRACK' || ph === 'DUAL_LOCK') ? 'active' : '');
    setNode('fsmEngage', ph === 'LOCK_ENGAGE' ? 'lock' : '');
    setNode('fsmCoast', ph === 'COAST_MEMORY' ? 'warn' : '');

    setText('fsmModeBadge', sim.cfg.autoMode ? `[AUTO FSM · ${sim.cfg.mode}]` : `[MANUAL · ${sim.cfg.mode}]`);
    const mb = $('fsmModeBadge');
    if (mb) mb.style.color = sim.cfg.autoMode ? C.green : C.amber;

    const latest = sim.events[0];
    if (latest) {
      setText('fsmLastEvent', `[T+${latest.t.toFixed(1)}s] ${latest.msg}`);
    }
    const logBox = $('eventLogList');
    if (logBox) {
      logBox.innerHTML = sim.events.slice(0, 12).map(ev =>
        `<div style="color:${ev.to === 'LOST' ? C.red : (ev.from === 'ECCM' ? C.green : C.cyan)};">[T+${ev.t.toFixed(1)}s] ${ev.msg}</div>`
      ).join('');
    }
  }

  function updateTelemetry() {
    const sim = state.sim; if (!sim) return;
    const cfg = sim.cfg, d = sim.derived;
    setText('teleFreq', `${cfg.fGHz.toFixed(2)} GHz / ${(d.lambda * 100).toFixed(2)} cm`);
    setText('telePower', `${d.healthy}/${sim.array.N} TR / ${(d.PtW / 1000).toFixed(1)} kW`);
    const duty = d.txInhibit ? (d.txInhibit === 'duty' ? '占空比超限' : '脉宽≥PRI') : 'NORMAL';
    setText('teleDuty', `${(d.duty * 100).toFixed(1)}% (${duty})`);
    const dutyEl = $('teleDuty'); if (dutyEl) dutyEl.className = 'tele-val ' + (d.txInhibit ? 'err' : 'ok');
    setText('teleGain', `${d.g0Db.toFixed(1)} / ${d.grDb.toFixed(1)} dBi`);
    setText('teleBeam', `${d.hpbwDeg.toFixed(2)}° / ${d.sllRxDb.toFixed(1)} dB`);
    setText('teleRes', `${d.rangeRes.toFixed(1)} m / ${d.mfLossDb.toFixed(2)} dB`);
    setText('telePrf', `${d.prfs.map(f => (f / 1e3).toFixed(1)).join('/')} kHz`);
    setText('teleScan', `${d.nPos} 位 / ${d.frameS.toFixed(2)} s`);

    const detCount = sim.targets.filter(tg => sim.getTargetTacticalStatus(tg).isDetected).length;
    const conf = sim.tracks.filter(t => t.confirmed && !t.isFalse).length;
    setText('teleTracks', `${detCount}/${sim.targets.length} 检出 / ${conf} 批 / ${sim.lockId || '无'}`);
    const trEl = $('teleTracks');
    if (trEl) trEl.className = 'tele-val ' + (sim.lockId ? 'warn' : (detCount === sim.targets.length ? 'ok' : 'err'));

    const e = cfg.eccm;
    const activeEccm = ['adbf', 'agility', 'dpca', 'mtdNotch', 'osCfar', 'leadingEdge', 'dopplerGate', 'slb', 'burnThrough'].filter(k => e[k]);
    const jamStr = sim.jammers.length ? `告警(${sim.jammers.map(j => j.type.toUpperCase()).join('+')})` : 'CLEAN';
    setText('teleEccm', `${jamStr} / ${activeEccm.length}项抗干扰`);
    const ecEl = $('teleEccm');
    if (ecEl) ecEl.className = 'tele-val ' + (sim.jammers.length ? (activeEccm.length ? 'warn' : 'err') : 'ok');

    syncModeBar();
    syncOSB();
    updateFSMStrip();
  }

  function renderTargetStatusMatrix() {
    const sim = state.sim; if (!sim) return;
    const mat = $('targetStatusMatrix'); if (!mat) return;
    mat.innerHTML = '';
    let detCnt = 0;
    sim.targets.forEach(tg => {
      const st = sim.getTargetTacticalStatus(tg);
      const diag = st.diag || sim.analyzeTarget(tg);
      if (st.isDetected) detCnt++;
      const row = document.createElement('div');
      row.className = 'tg-row-card' + (tg.id === state.targetId ? ' selected' : '');

      let badgeCls = 'undet', badgeTxt = `未检出·已隐藏 (${VERDICT_SHORT[diag.verdict] || diag.verdict})`;
      if (st.status === 'LOCKED') { badgeCls = 'lock'; badgeTxt = `已锁定 (${st.track.id})`; }
      else if (st.status === 'TRACKED') { badgeCls = 'det'; badgeTxt = `稳定跟踪 (${st.track.id})`; }
      else if (st.status === 'ACQUIRING') { badgeCls = 'det'; badgeTxt = `点迹捕获中`; }
      else if (st.status === 'COASTING') { badgeCls = 'coast'; badgeTxt = `记忆外推 (暂失回波)`; }
      else if (st.status === 'DECEIVED') { badgeCls = 'lock'; badgeTxt = `波门被假目标拖引!`; }

      row.innerHTML = `
        <div>
          <strong style="color:${st.isDetected ? C.green : '#ff8096'};">${tg.id} ${tg.name}</strong>
          <span style="color:var(--txt-dim);margin-left:6px;">R=${(diag.R / 1000).toFixed(1)}km Az=${diag.az.toFixed(1)}° RCS=${tg.rcs}m²</span>
        </div>
        <div style="display:flex;align-items:center;gap:5px;">
          <span class="tg-badge ${badgeCls}">${badgeTxt}</span>
        </div>
      `;
      row.onclick = () => {
        state.targetId = tg.id;
        syncTarget();
        renderLists();
        updateDiagnostics();
      };
      mat.appendChild(row);
    });
    setText('detSummaryBadge', `${detCnt}/${sim.targets.length} 已检出显示 · ${sim.targets.length - detCnt} 未检出隐藏`);
    const sb = $('detSummaryBadge');
    if (sb) sb.style.color = detCnt === sim.targets.length ? C.green : C.amber;
  }

  function applySuggestionFix(sugg) {
    const sim = state.sim; if (!sim) return;
    if (sugg.modeSwitch) {
      setMode(sugg.modeSwitch);
    }
    if (sugg.patch) {
      patchConfig(sugg.patch);
    }
    sim._pushEvent('FIX', sim.cfg.mode, `执行诊断建议对策：${sugg.title}`);
    syncAll();
    updateDiagnostics();
    updateTelemetry();
  }

  function updateDiagnostics() {
    const sim = state.sim; if (!sim) return;
    renderTargetStatusMatrix();
    const tg = target();
    const causesSec = $('diagCausesSection'), causesList = $('diagCausesList');
    const suggestSec = $('diagSuggestSection'), suggestList = $('diagSuggestList');

    if (!tg) {
      setText('diagTargetBadge', '诊断目标: 无');
      setText('verdictTitle', '无目标 — 请新增或选择目标');
      setText('verdictReason', '所有点迹与航迹均由雷达方程/CFAR 实时产生。');
      if (causesSec) causesSec.style.display = 'none';
      if (suggestSec) suggestSec.style.display = 'none';
      for (const id of ['bgPos', 'bgDop', 'bgEnergy', 'bgGain', 'bgScanLoss', 'bgRcsTerm', 'bgSpread', 'bgLosses', 'bgNoiseRaise', 'bgSinr', 'bgPd']) setText(id, '--');
      return;
    }
    const o = sim.analyzeTarget(tg), d = sim.derived, cfg = sim.cfg;
    const st = sim.getTargetTacticalStatus(tg);
    state.diag = o; state.diagId = tg.id;
    setText('diagTargetBadge', `诊断目标: ${tg.id} ${tg.name} [${st.isDetected ? 'MFD显示中' : '未检出已隐藏'}]`);

    const cls = (o.verdict === 'OK' && !o.deceptionWarn) ? 'ok' : (o.verdict !== 'OK' ? 'fail' : 'warn');
    const vc = $('verdictContainer'); if (vc) vc.className = 'verdict-box ' + cls;
    const statusDesc = o.verdict === 'OK'
      ? (o.deceptionWarn ? '已检出，但遭受 DRFM/VGPO 欺骗干扰威胁' : `已成功检出 (${st.status}) — 满足雷达方程与波束照明条件`)
      : `未检出 (${VERDICT_SHORT[o.verdict] || o.verdict}) — 已在雷达显示器上隐藏该目标`;
    setText('verdictTitle', `状态: ${o.verdict} — ${statusDesc}`);
    setText('verdictReason', o.reasons.join('；') || '—');

    // 渲染未检出原因卡片
    if (causesSec && causesList) {
      if (o.causes && o.causes.length > 0) {
        causesSec.style.display = 'flex';
        causesList.innerHTML = '';
        o.causes.forEach(c => {
          const div = document.createElement('div');
          div.className = 'cause-card' + (c.severity === 'warn' ? ' warn' : '');
          div.innerHTML = `
            <div class="cause-title">
              <span>[${c.code}] ${c.title}</span>
              <span>${c.severity === 'critical' ? '导致目标无法检出' : '失锁风险'}</span>
            </div>
            <div class="cause-detail">${c.detail}</div>
          `;
          causesList.appendChild(div);
        });
      } else {
        causesSec.style.display = 'none';
      }
    }

    // 渲染针对性措施建议与一键修复按钮
    if (suggestSec && suggestList) {
      if (o.suggestions && o.suggestions.length > 0) {
        suggestSec.style.display = 'flex';
        suggestList.innerHTML = '';
        o.suggestions.forEach(sugg => {
          const card = document.createElement('div');
          card.className = 'suggest-card';
          card.innerHTML = `
            <div class="suggest-hdr">
              <span class="suggest-title">✦ ${sugg.title}</span>
              <span class="suggest-gain">${sugg.expectedGain}</span>
            </div>
            <div class="suggest-desc">${sugg.desc}</div>
          `;
          const btn = document.createElement('button');
          btn.className = 'act-fix-btn';
          btn.textContent = `⚡ ${sugg.actionLabel}`;
          btn.onclick = () => applySuggestionFix(sugg);
          card.appendChild(btn);
          suggestList.appendChild(card);
        });
      } else {
        suggestSec.style.display = 'none';
      }
    }

    setText('bgPos', `${(o.R / 1000).toFixed(1)} km / ${o.az.toFixed(1)}° / ${o.el.toFixed(1)}°`);
    setText('bgDop', `${o.closing.toFixed(1)} m/s / ${(o.fd / 1e3).toFixed(2)} kHz`);
    if (o.verdict === 'TX_INHIBIT') {
      for (const id of ['bgEnergy', 'bgGain', 'bgScanLoss', 'bgRcsTerm', 'bgSpread', 'bgLosses', 'bgNoiseRaise', 'bgSinr', 'bgPd']) setText(id, '--');
      return;
    }
    const nCoh = d.nCpiEff || cfg.tx.nCpi;
    setText('bgEnergy', `Pt ${(d.PtW / 1000).toFixed(1)} kW · τ ${(d.tau * 1e6).toFixed(0)} μs · N ${nCoh} = ${(10 * Math.log10(d.PtW * d.tau * nCoh)).toFixed(1)} dBJ`);
    setText('bgGain', `Gt ${o.gTdb.toFixed(1)} + Gr ${o.gRdb.toFixed(1)} = ${(o.gTdb + o.gRdb).toFixed(1)} dBi`);
    setText('bgScanLoss', `${(o.scanOffsetDeg || 0).toFixed(2)}° / ${(o.scanLossDb || 0).toFixed(2)} dB`);
    setText('bgRcsTerm', `${(10 * Math.log10(d.lambda * d.lambda * tg.rcs)).toFixed(1)} dB`);
    setText('bgSpread', `${(10 * Math.log10(Math.pow(4 * Math.PI, 3) * Math.pow(o.R, 4))).toFixed(1)} dB`);
    setText('bgLosses', `总 ${o.eq.lossTotalDb.toFixed(1)} dB (大气 ${o.eq.terms.atmDb.toFixed(1)} / 雨 ${o.eq.terms.rainDb.toFixed(1)} / 系统 ${o.eq.terms.lossDb.toFixed(1)})`);
    setText('bgNoiseRaise', `Jn ${o.jnDb.toFixed(1)} / Cn ${(o.cnDb || -300).toFixed(1)} / ΔN ${o.noiseRaiseDb.toFixed(1)} dB`);
    setText('bgSinr', `${o.snrBurstDb.toFixed(1)} dB / 需 ${o.reqSnrDb.toFixed(1)} dB (余量 ${o.marginDb.toFixed(1)} dB)`);
    setText('bgPd', `${pct(o.pdSys)} / Rmax ${o.rmaxKm.toFixed(0)} km`);
  }

  /* ---------------------------------------------------------- lists / sync */
  function renderLists() {
    const sim = state.sim; if (!sim) return;
    const tl = $('targetListBar'); if (tl) {
      tl.innerHTML = '';
      sim.targets.forEach(tg => {
        const b = document.createElement('button');
        const st = sim.getTargetTacticalStatus(tg);
        const dotCol = st.status === 'LOCKED' ? C.red : (st.status === 'TRACKED' ? C.green : (st.status === 'ACQUIRING' || st.status === 'COASTING' ? C.amber : C.dim));
        b.className = 'entity-pill' + (tg.id === state.targetId ? ' active' : '');
        b.innerHTML = `<span class="status-dot" style="background:${dotCol}"></span>${tg.id} ${tg.name} <span style="font-size:9px;opacity:0.8;">[${st.isDetected ? '检出' : '隐藏'}]</span>`;
        b.onclick = () => { state.targetId = tg.id; syncTarget(); renderLists(); updateDiagnostics(); };
        tl.appendChild(b);
      });
    }
    const jl = $('jammerListBar'); if (jl) {
      jl.innerHTML = '';
      sim.jammers.forEach(j => {
        const b = document.createElement('button');
        b.className = 'entity-pill' + (j.id === state.jammerId ? ' active' : '');
        b.innerHTML = `<span class="status-dot" style="background:${C.red}"></span>${j.id} ${j.type.toUpperCase()}`;
        b.onclick = () => { state.jammerId = j.id; syncJammer(); renderLists(); };
        jl.appendChild(b);
      });
    }
  }

  function syncAll() { syncRadar(); syncScan(); syncTarget(); syncJammer(); syncOSB(); syncChips(); syncModeBar(); }

  function syncModeBar() {
    const mode = state.sim.cfg.mode;
    document.querySelectorAll('#modeBar .mode-btn').forEach(b => b.classList.toggle('active', b.dataset.mode === mode));
    const btnAuto = $('btnAutoMode');
    if (btnAuto) {
      btnAuto.textContent = `状态自动切换: ${state.sim.cfg.autoMode ? '开启' : '手动'}`;
      btnAuto.className = 'btn ' + (state.sim.cfg.autoMode ? 'success' : 'warn');
    }
    const btnAutoEccm = $('btnAutoEccm');
    if (btnAutoEccm) {
      btnAutoEccm.textContent = `智能抗干扰: ${state.sim.cfg.eccm.autoEccm ? '开启' : '关闭'}`;
      btnAutoEccm.className = 'btn ' + (state.sim.cfg.eccm.autoEccm ? 'success' : 'primary');
    }
  }
  function syncChips() {
    const e = state.sim.cfg.eccm;
    [['viewBscope', 'bscope'], ['viewPpi', 'ppi'], ['viewHud', 'hud'], ['viewSar', 'sar']].forEach(([id, v]) => setChip(id, state.mfdView === v));
    setChip('btnCutAz', state.pattern === 'az'); setChip('btnCutEl', state.pattern === 'el'); setChip('btnPat2D', state.pattern === '2d');
    setChip('eccmAuto', e.autoEccm);
    setChip('eccmAdbf', e.adbf); setChip('eccmAgil', e.agility);
    setChip('eccmDpca', e.dpca); setChip('eccmMtd', e.mtdNotch);
    setChip('eccmOscfar', e.osCfar); setChip('eccmLe', e.leadingEdge);
    setChip('eccmDopGate', e.dopplerGate); setChip('eccmSlb', e.slb);
    setChip('eccmBurn', e.burnThrough);
    setChip('btnClutterToggle', state.sim.cfg.env.clutterOn); setChip('btnStcToggle', state.sim.cfg.rx.stc);
    const st = $('btnPatSteer'); if (st) st.textContent = state.steer === 'target' ? '指向: 选中目标' : '指向: 当前波束';
  }
  function syncRadar() {
    const c = state.sim.cfg, tx = c.tx, a = c.array;
    setVal('inpFGHz', c.fGHz.toFixed(2)); setText('valFGHz', `${c.fGHz.toFixed(2)} GHz`); setText('lblFreqBand', `λ = ${(RE.wavelength(c.fGHz) * 100).toFixed(2)} cm`);
    setVal('inpWaveform', tx.waveform); setVal('inpWindow', tx.window);
    setVal('inpTau', tx.tauUs); setText('valTau', `${tx.tauUs.toFixed(0)} μs`);
    setVal('inpBw', tx.bwMHz); setText('valBw', `${tx.bwMHz.toFixed(0)} MHz`);
    setVal('inpPrfRegime', tx.prfRegime);
    setVal('inpMofN', tx.mOfN); setText('valMofN', `${tx.mOfN} / ${state.sim.derived.nB}`);
    setVal('inpNcpi', tx.nCpi); setText('valNcpi', `${tx.nCpi} 脉冲`);
    setVal('inpTrW', a.trPeakW); setText('valTrW', `${a.trPeakW} W`);
    setVal('inpGrid', a.grid); setVal('inpTaper', a.rxTaper);
    setVal('inpDiam', a.diameter); setText('valDiam', `${a.diameter.toFixed(2)} m`);
    setVal('inpSpacing', a.spacingLambda); setText('valSpacing', `${a.spacingLambda.toFixed(2)} λ`);
    setVal('inpFail', a.failRate); setText('valFail', `${(a.failRate * 100).toFixed(0)}%`);
    setVal('inpBits', a.phaseBits); setText('valBits', `${a.phaseBits} bits`);
    document.querySelectorAll('.freq-preset').forEach(b => b.classList.toggle('active', Math.abs(+b.dataset.f - c.fGHz) < 0.001));
  }
  function syncScan() {
    const c = state.sim.cfg, s = c.scan;
    setVal('inpScanAz0', s.azCenter); setText('valScanAz0', `${s.azCenter.toFixed(0)}°`);
    setVal('inpScanAzHalf', s.azHalf); setText('valScanAzHalf', `±${s.azHalf.toFixed(0)}°`);
    setVal('inpScanEl0', s.elCenter); setText('valScanEl0', `${s.elCenter.toFixed(1)}°`);
    setVal('inpScanBars', s.bars); setText('valScanBars', `${s.bars} 行`);
    setVal('inpRngMax', c.rangeMaxKm); setText('valRngMax', `${c.rangeMaxKm.toFixed(0)} km`); state.rangeDisplay = c.rangeMaxKm;
    setVal('inpOwnAlt', c.own.alt); setText('valOwnInfo', `${c.own.alt.toFixed(0)} m`);
    setVal('inpOwnSpd', c.own.speed); setText('valOwnSpd', `${c.own.speed.toFixed(0)} m/s`);
    setVal('inpRain', c.env.rainMmH); setText('valRain', `${c.env.rainMmH.toFixed(0)} mm/h`);
    setVal('inpNf', c.rx.nfDb); setText('valNf', `${c.rx.nfDb.toFixed(1)} dB`);
    setVal('inpPfa', String(c.rx.pfa)); setText('valPfa', c.rx.pfa.toExponential(0));
    setChip('btnClutterToggle', c.env.clutterOn); setChip('btnStcToggle', c.rx.stc);
  }
  function syncTarget() {
    const tg = target();
    if (!tg) { setVal('tgName', '--'); setText('diagTargetBadge', '诊断目标: 无'); return; }
    const sim = state.sim, s = sim.targetState(tg, sim.time), sp = tg.spec;
    setVal('tgName', tg.name); setVal('tgDomain', tg.isGround ? 'ground' : 'air');
    setVal('inpTgR', (s.R / 1000).toFixed(1)); setText('valTgR', `${(s.R / 1000).toFixed(1)} km`);
    setVal('inpTgAz', s.az.toFixed(1)); setText('valTgAz', `${s.az.toFixed(1)}°`);
    setVal('inpTgAlt', s.alt.toFixed(0)); setText('valTgAlt', `${s.alt.toFixed(0)} m`);
    setVal('inpTgSpd', sp.speed); setText('valTgSpd', `${sp.speed.toFixed(0)} m/s`);
    setVal('inpTgHdg', sp.headingDeg); setText('valTgHdg', `${sp.headingDeg.toFixed(0)}°`);
    setVal('inpTgClimb', sp.climb); setText('valTgClimb', `${sp.climb.toFixed(0)} m/s`);
    setVal('inpTgRcs', tg.rcs); setText('valTgRcs', `${tg.rcs.toFixed(3)} m²`);
    setVal('inpTgSw', String(tg.swerling));
  }
  function syncJammer() {
    const j = jammer();
    if (!j) { setText('valJamErp', '--'); return; }
    setVal('jamType', j.type); setVal('inpJamErp', j.erpDbW); setText('valJamErp', `${j.erpDbW.toFixed(0)} dBW`);
    setVal('inpJamAz', j.azDeg); setText('valJamAz', `${j.azDeg.toFixed(0)}°`);
    setVal('inpJamFreq', j.fGHz); setText('valJamFreq', `${j.fGHz.toFixed(1)} GHz`);
    setVal('inpJamRng', j.rangeKm); setText('valJamRng', `${j.rangeKm.toFixed(0)} km`);
    setVal('inpJamEl', j.elDeg); setText('valJamEl', `${j.elDeg.toFixed(0)}°`);
    setVal('inpJamBw', j.bwMHz); setText('valJamBw', `${j.bwMHz.toFixed(0)} MHz`);
  }
  function syncOSB() {
    const c = state.sim.cfg, d = state.sim.derived;
    setText('osbMode', c.mode);
    setChip('osbAuto', c.autoMode);
    setText('osbAz', `±${c.scan.azHalf.toFixed(0)}°`);
    setText('osbBars', `${c.scan.bars}B`);
    setText('osbPrf', c.tx.prfRegime.toUpperCase());
    setText('osbFreq', `${c.fGHz.toFixed(1)}G`);
    setText('osbWf', c.tx.waveform.toUpperCase().slice(0, 4));
    setText('osbTaper', c.array.rxTaper === 'lowsll' ? 'LOWS' : c.array.rxTaper.toUpperCase().slice(0, 4));
    setChip('osbAdbf', c.eccm.adbf); setChip('osbAgil', c.eccm.agility); setChip('osbDpca', c.eccm.dpca);
    setChip('osbLe', c.eccm.leadingEdge); setChip('osbSlb', c.eccm.slb); setChip('osbTruth', state.truth); setChip('osbSlew', state.slew);
    setText('teleScan', `${d.nPos} 位 / ${d.frameS.toFixed(2)} s`);
    const note = $('mfdDisplayPolicyNote');
    if (note) {
      note.textContent = state.truth
        ? '● 调试真值对比已开启：紫色虚线框为未检出目标的参考真值位置'
        : '● 实测模式：仅显示雷达检出的点迹与航迹，检测不出的目标严格不显示';
      note.style.color = state.truth ? C.amber : C.green;
    }
  }

  /* ---------------------------------------------------------- config updates */
  function patchConfig(p) {
    state.sim.setConfig(p); state.sim.cfg.timeScale = state.speed;
    state.sarCache = null;
    syncRadar(); syncScan(); syncOSB(); syncChips(); updateDiagnostics();
  }
  function patchTarget(p) {
    const tg = target(); if (!tg) return;
    state.sim.updateTarget(tg.id, p);
    state.sarCache = null; renderLists(); updateDiagnostics();
  }
  function patchJammer(p) {
    const j = jammer(); if (!j) return;
    state.sim.updateJammer(j.id, p); renderLists(); updateDiagnostics();
  }
  function setMode(mode) {
    const sim = state.sim;
    if (mode !== 'STT' && mode !== 'DTT') sim.breakLock();
    sim.applyMode(mode); sim.cfg.timeScale = state.speed;
    if (mode === 'GM' || mode === 'SAR') state.mfdView = 'sar';
    else if (mode === 'GMTI') state.mfdView = 'ppi';
    else if (mode === 'ACM') state.mfdView = 'hud';
    else if (state.mfdView === 'sar') state.mfdView = 'bscope';
    syncAll(); renderLists(); updateDiagnostics();
  }

  function loadScenario(key) {
    const sim = RE.createScenario(key);
    sim.cfg.timeScale = state.speed;
    state.sim = sim; state.scenario = key;
    state.targetId = sim.targets[0] ? sim.targets[0].id : null;
    state.jammerId = sim.jammers[0] ? sim.jammers[0].id : null;
    state.truth = false; // 默认严格关闭真值透视，未检出目标不显示
    state.paused = false; state.diag = null; state.sarCache = null;
    if (sim.cfg.mode === 'GMTI') state.mfdView = 'ppi';
    else if (sim.cfg.mode === 'ACM') state.mfdView = 'hud';
    else state.mfdView = 'bscope';
    setVal('scenarioSelect', key); setText('btnPause', '暂停');
    syncAll(); renderLists(); updateDiagnostics(); updateTelemetry();
  }

  /* ---------------------------------------------------------- bindings */
  function on(id, ev, fn) { const el = $(id); if (el) el.addEventListener(ev, fn); }
  function bindRange(id, fn, labelId, fmtFn) {
    const el = $(id); if (!el) return;
    el.addEventListener('input', () => { fn(+el.value); if (labelId) setText(labelId, fmtFn(+el.value)); });
  }
  function bindTargetRange(id, labelId, key, fmtFn) {
    const el = $(id); if (!el) return;
    el.addEventListener('input', () => {
      const v = +el.value; setText(labelId, fmtFn(v)); patchTarget({ [key]: v });
    });
  }

  function bindUI() {
    document.querySelectorAll('#modeBar .mode-btn').forEach(b => b.onclick = () => setMode(b.dataset.mode));
    on('scenarioSelect', 'change', e => loadScenario(e.target.value));
    on('btnPause', 'click', () => { state.paused = !state.paused; setText('btnPause', state.paused ? '继续' : '暂停'); });
    on('btnStep', 'click', () => { state.sim.step(state.sim.derived.dwellS, 0); updateDiagnostics(); updateTelemetry(); });
    on('btnReset', 'click', () => loadScenario(state.scenario));
    on('speedSelect', 'change', e => { state.speed = +e.target.value; state.sim.cfg.timeScale = state.speed; });

    on('btnAutoMode', 'click', () => {
      state.sim.cfg.autoMode = !state.sim.cfg.autoMode;
      state.sim._pushEvent('FSM', state.sim.cfg.autoMode ? 'AUTO' : 'MANUAL', `状态机切换为${state.sim.cfg.autoMode ? '自动流转 (AUTO FSM)' : '手动锁定 (MANUAL)'}`);
      syncModeBar(); syncOSB(); updateFSMStrip();
    });
    on('btnAutoEccm', 'click', () => {
      state.sim.cfg.eccm.autoEccm = !state.sim.cfg.eccm.autoEccm;
      if (state.sim.cfg.eccm.autoEccm) state.sim._runAutoEccm();
      syncAll(); updateDiagnostics(); updateTelemetry();
    });

    [['viewBscope', 'bscope'], ['viewPpi', 'ppi'], ['viewHud', 'hud'], ['viewSar', 'sar']].forEach(([id, v]) => on(id, 'click', () => { state.mfdView = v; syncChips(); }));
    on('btnCutAz', 'click', () => { state.pattern = 'az'; syncChips(); });
    on('btnCutEl', 'click', () => { state.pattern = 'el'; syncChips(); });
    on('btnPat2D', 'click', () => { state.pattern = '2d'; syncChips(); });
    on('btnPatSteer', 'click', () => { state.steer = state.steer === 'target' ? 'beam' : 'target'; syncChips(); });

    // Radar / waveform / array
    bindRange('inpFGHz', debounce(v => patchConfig({ fGHz: v }), 80), 'valFGHz', v => `${v.toFixed(2)} GHz`);
    on('inpWaveform', 'change', e => patchConfig({ tx: { waveform: e.target.value } }));
    on('inpWindow', 'change', e => patchConfig({ tx: { window: e.target.value } }));
    bindRange('inpTau', debounce(v => patchConfig({ tx: { tauUs: v } }), 100), 'valTau', v => `${v.toFixed(0)} μs`);
    bindRange('inpBw', debounce(v => patchConfig({ tx: { bwMHz: v } }), 100), 'valBw', v => `${v.toFixed(0)} MHz`);
    on('inpPrfRegime', 'change', e => patchConfig({ tx: { prfRegime: e.target.value } }));
    bindRange('inpMofN', v => patchConfig({ tx: { mOfN: v } }), 'valMofN', v => `${v} / ${state.sim.derived.nB}`);
    bindRange('inpNcpi', debounce(v => patchConfig({ tx: { nCpi: v } }), 100), 'valNcpi', v => `${v} 脉冲`);
    bindRange('inpTrW', v => patchConfig({ array: { trPeakW: v } }), 'valTrW', v => `${v} W`);
    on('inpGrid', 'change', e => patchConfig({ array: { grid: e.target.value } }));
    on('inpTaper', 'change', e => patchConfig({ array: { rxTaper: e.target.value } }));
    bindRange('inpDiam', debounce(v => patchConfig({ array: { diameter: v } }), 120), 'valDiam', v => `${v.toFixed(2)} m`);
    bindRange('inpSpacing', debounce(v => patchConfig({ array: { spacingLambda: v } }), 120), 'valSpacing', v => `${v.toFixed(2)} λ`);
    bindRange('inpFail', debounce(v => patchConfig({ array: { failRate: v } }), 120), 'valFail', v => `${(v * 100).toFixed(0)}%`);
    bindRange('inpBits', debounce(v => patchConfig({ array: { phaseBits: v } }), 120), 'valBits', v => `${v} bits`);
    document.querySelectorAll('.freq-preset').forEach(b => b.onclick = () => { patchConfig({ fGHz: +b.dataset.f }); syncRadar(); });

    // Target editor
    on('btnAddTarget', 'click', () => {
      const n = state.sim.targets.length + 1;
      const tg = state.sim.addTarget({ name: 'Target-' + n, rangeKm: 60, azDeg: (n % 2 ? 12 : -12), altM: 8500, speed: 260, headingDeg: 180, rcs: 3, swerling: 1 });
      state.targetId = tg.id; syncTarget(); renderLists(); updateDiagnostics();
    });
    on('btnDelTarget', 'click', () => {
      if (!state.targetId) return;
      state.sim.removeTarget(state.targetId);
      state.targetId = state.sim.targets[0] ? state.sim.targets[0].id : null;
      syncTarget(); renderLists(); updateDiagnostics();
    });
    on('tgName', 'change', e => patchTarget({ name: e.target.value }));
    on('tgDomain', 'change', e => { const ground = e.target.value === 'ground'; patchTarget({ isGround: ground }); if (ground && target() && target().spec.altM > 1500) patchTarget({ altM: 100 }); syncTarget(); });
    bindTargetRange('inpTgR', 'valTgR', 'rangeKm', v => `${v.toFixed(1)} km`);
    bindTargetRange('inpTgAz', 'valTgAz', 'azDeg', v => `${v.toFixed(1)}°`);
    bindTargetRange('inpTgAlt', 'valTgAlt', 'altM', v => `${v.toFixed(0)} m`);
    bindTargetRange('inpTgSpd', 'valTgSpd', 'speed', v => `${v.toFixed(0)} m/s`);
    bindTargetRange('inpTgHdg', 'valTgHdg', 'headingDeg', v => `${v.toFixed(0)}°`);
    bindTargetRange('inpTgClimb', 'valTgClimb', 'climb', v => `${v.toFixed(0)} m/s`);
    on('inpTgRcs', 'change', e => { if (target()) { patchTarget({ rcs: +e.target.value }); setText('valTgRcs', `${(+e.target.value).toFixed(3)} m²`); } });
    on('inpTgSw', 'change', e => patchTarget({ swerling: +e.target.value }));
    document.querySelectorAll('.rcs-preset').forEach(b => b.onclick = () => { patchTarget({ rcs: +b.dataset.rcs }); syncTarget(); });

    // Jammer editor
    on('btnAddJam', 'click', () => {
      const j = state.sim.addJammer({ type: 'spot', azDeg: -22, elDeg: 0, rangeKm: 90, erpDbW: 68, fGHz: state.sim.cfg.fGHz, bwMHz: 25, targetId: state.targetId });
      state.jammerId = j.id; syncJammer(); renderLists(); updateDiagnostics();
    });
    on('btnDelJam', 'click', () => {
      if (!state.jammerId) return; state.sim.removeJammer(state.jammerId);
      state.jammerId = state.sim.jammers[0] ? state.sim.jammers[0].id : null; syncJammer(); renderLists(); updateDiagnostics();
    });
    on('jamType', 'change', e => patchJammer({ type: e.target.value, targetId: state.targetId }));
    bindRange('inpJamErp', v => patchJammer({ erpDbW: v }), 'valJamErp', v => `${v.toFixed(0)} dBW`);
    bindRange('inpJamAz', v => patchJammer({ azDeg: v }), 'valJamAz', v => `${v.toFixed(0)}°`);
    bindRange('inpJamFreq', v => patchJammer({ fGHz: v }), 'valJamFreq', v => `${v.toFixed(1)} GHz`);
    bindRange('inpJamRng', v => patchJammer({ rangeKm: v }), 'valJamRng', v => `${v.toFixed(0)} km`);
    bindRange('inpJamEl', v => patchJammer({ elDeg: v }), 'valJamEl', v => `${v.toFixed(0)}°`);
    bindRange('inpJamBw', v => patchJammer({ bwMHz: v }), 'valJamBw', v => `${v.toFixed(0)} MHz`);

    // Scan / environment / ECCM
    bindRange('inpScanAz0', v => patchConfig({ scan: { azCenter: v } }), 'valScanAz0', v => `${v.toFixed(0)}°`);
    bindRange('inpScanAzHalf', v => patchConfig({ scan: { azHalf: v } }), 'valScanAzHalf', v => `±${v.toFixed(0)}°`);
    bindRange('inpScanEl0', v => patchConfig({ scan: { elCenter: v } }), 'valScanEl0', v => `${v.toFixed(1)}°`);
    bindRange('inpScanBars', v => patchConfig({ scan: { bars: v } }), 'valScanBars', v => `${v} 行`);
    bindRange('inpRngMax', v => patchConfig({ rangeMaxKm: v }), 'valRngMax', v => `${v.toFixed(0)} km`);
    bindRange('inpOwnAlt', v => patchConfig({ own: { alt: v } }), 'valOwnInfo', v => `${v.toFixed(0)} m`);
    bindRange('inpOwnSpd', v => patchConfig({ own: { speed: v } }), 'valOwnSpd', v => `${v.toFixed(0)} m/s`);
    bindRange('inpRain', v => patchConfig({ env: { rainMmH: v } }), 'valRain', v => `${v.toFixed(0)} mm/h`);
    bindRange('inpNf', v => patchConfig({ rx: { nfDb: v } }), 'valNf', v => `${v.toFixed(1)} dB`);
    on('inpPfa', 'change', e => patchConfig({ rx: { pfa: +e.target.value } }));
    on('btnAlignScanToTarget', 'click', () => {
      const tg = target(); if (!tg) return;
      const s = state.sim.targetState(tg, state.sim.time);
      patchConfig({ scan: { azCenter: clamp(s.az, -60, 60), elCenter: clamp(s.el, -25, 25) } });
      setVal('inpScanAz0', s.az.toFixed(0)); setVal('inpScanEl0', s.el.toFixed(1));
    });
    on('btnClutterToggle', 'click', () => { patchConfig({ env: { clutterOn: !state.sim.cfg.env.clutterOn } }); });
    on('btnStcToggle', 'click', () => { patchConfig({ rx: { stc: !state.sim.cfg.rx.stc } }); });

    const eccmMap = [
      ['eccmAuto', 'autoEccm'],
      ['eccmAdbf', 'adbf'],
      ['eccmAgil', 'agility'],
      ['eccmDpca', 'dpca'],
      ['eccmMtd', 'mtdNotch'],
      ['eccmOscfar', 'osCfar'],
      ['eccmLe', 'leadingEdge'],
      ['eccmDopGate', 'dopplerGate'],
      ['eccmSlb', 'slb'],
      ['eccmBurn', 'burnThrough']
    ];
    eccmMap.forEach(([id, key]) => on(id, 'click', () => {
      const cfg = state.sim.cfg;
      cfg.eccm[key] = !cfg.eccm[key];
      if (key === 'autoEccm' && cfg.eccm.autoEccm) state.sim._runAutoEccm();
      state.sim.rebuild();
      state.sim._pruneUndetectablePlots();
      syncAll(); updateDiagnostics(); updateTelemetry();
    }));
    on('btnEnableAllEccm', 'click', () => {
      patchConfig({
        eccm: { adbf: true, agility: true, dpca: true, mtdNotch: true, osCfar: true, leadingEdge: true, dopplerGate: true, slb: true, burnThrough: true, autoEccm: true }
      });
      state.sim._pushEvent('ECCM', 'ALL', '一键激活全维电子反对抗 (ECCM) 矩阵');
      updateTelemetry();
    });

    // OSB softkeys
    on('osbMode', 'click', () => { const m = MODES[(MODES.indexOf(state.sim.cfg.mode) + 1) % MODES.length]; setMode(m); });
    on('osbAuto', 'click', () => { state.sim.cfg.autoMode = !state.sim.cfg.autoMode; syncModeBar(); syncOSB(); updateFSMStrip(); });
    on('osbAz', 'click', () => { const opts = [15, 30, 45, 60]; const v = opts[(opts.indexOf(state.sim.cfg.scan.azHalf) + 1) % opts.length] || 15; patchConfig({ scan: { azHalf: v } }); });
    on('osbBars', 'click', () => { const opts = [1, 2, 4, 8]; const v = opts[(opts.indexOf(state.sim.cfg.scan.bars) + 1) % opts.length] || 1; patchConfig({ scan: { bars: v } }); });
    on('osbPrf', 'click', () => { const opts = Object.keys(RE.PRF_SETS); const v = opts[(opts.indexOf(state.sim.cfg.tx.prfRegime) + 1) % opts.length]; patchConfig({ tx: { prfRegime: v } }); });
    on('osbFreq', 'click', () => { const opts = [8, 9, 10, 11, 12]; const v = opts[(opts.indexOf(Math.round(state.sim.cfg.fGHz)) + 1) % opts.length]; patchConfig({ fGHz: v }); });
    on('osbRngUp', 'click', () => patchConfig({ rangeMaxKm: Math.min(300, state.sim.cfg.rangeMaxKm + 20) }));
    on('osbRngDn', 'click', () => patchConfig({ rangeMaxKm: Math.max(20, state.sim.cfg.rangeMaxKm - 20) }));
    on('osbElUp', 'click', () => patchConfig({ scan: { elCenter: Math.min(25, state.sim.cfg.scan.elCenter + 2) } }));
    on('osbElDn', 'click', () => patchConfig({ scan: { elCenter: Math.max(-25, state.sim.cfg.scan.elCenter - 2) } }));
    on('osbTruth', 'click', () => { state.truth = !state.truth; syncOSB(); });
    on('osbLock', 'click', () => {
      state.sim.autoLockStrongest(state.sim.cfg.mode === 'DTT' ? 2 : 1);
      if (state.sim.lockId && state.sim.cfg.mode !== 'DTT') state.sim.applyMode('STT', `手动按下 LCK 锁定航迹 ${state.sim.lockId}`);
      syncAll(); updateDiagnostics(); updateTelemetry();
    });
    on('osbBrk', 'click', () => { state.sim.breakLock(); syncAll(); updateDiagnostics(); updateTelemetry(); });
    on('osbAdbf', 'click', () => { const c = state.sim.cfg; c.eccm.adbf = !c.eccm.adbf; state.sim.rebuild(); syncChips(); syncOSB(); updateDiagnostics(); });
    on('osbAgil', 'click', () => { const c = state.sim.cfg; c.eccm.agility = !c.eccm.agility; state.sim.rebuild(); syncChips(); syncOSB(); updateDiagnostics(); });
    on('osbDpca', 'click', () => { const c = state.sim.cfg; c.eccm.dpca = !c.eccm.dpca; state.sim.rebuild(); syncChips(); syncOSB(); updateDiagnostics(); });
    on('osbLe', 'click', () => { const c = state.sim.cfg; c.eccm.leadingEdge = !c.eccm.leadingEdge; state.sim.rebuild(); syncChips(); syncOSB(); });
    on('osbSlb', 'click', () => { const c = state.sim.cfg; c.eccm.slb = !c.eccm.slb; state.sim.rebuild(); syncChips(); syncOSB(); });
    on('osbWf', 'click', () => { const opts = ['lfm', 'barker', 'costas']; patchConfig({ tx: { waveform: opts[(opts.indexOf(state.sim.cfg.tx.waveform) + 1) % opts.length] } }); syncRadar(); });
    on('osbTaper', 'click', () => { const opts = ['lowsll', 'uniform', 'spoil', 'diffAz', 'diffEl']; patchConfig({ array: { rxTaper: opts[(opts.indexOf(state.sim.cfg.array.rxTaper) + 1) % opts.length] } }); syncRadar(); });
    on('osbSlew', 'click', () => { state.slew = !state.slew; syncOSB(); });

    // Tabs
    document.querySelectorAll('.tab-btn').forEach(b => b.onclick = () => {
      document.querySelectorAll('.tab-btn').forEach(q => q.classList.remove('active'));
      document.querySelectorAll('.tab-pane').forEach(q => q.classList.remove('active'));
      b.classList.add('active'); const p = $(b.dataset.tab); if (p) p.classList.add('active');
    });

    // Canvas interactions
    on('arrayCanvas', 'click', () => { state.arrayPhase = !state.arrayPhase; });
    on('scanCanvas', 'click', e => {
      const m = state.scanBox; if (!m) return;
      const r = e.target.getBoundingClientRect();
      const x = e.clientX - r.left, y = e.clientY - r.top;
      const az = m.xmin + (x - m.box.x) / m.box.w * (m.xmax - m.xmin);
      const el = m.ymax - (y - m.box.y) / m.box.h * (m.ymax - m.ymin);
      patchConfig({ scan: { azCenter: clamp(az, -60, 60), elCenter: clamp(el, -25, 25) } });
    });
    on('mfdCanvas', 'mousemove', e => {
      const m = state.mfdBox; if (!m) return;
      const r = e.target.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
      let az = 0, R = 0, el = null;
      if (m.kind === 'bscope' || m.kind === 'hud') {
        az = m.m.xmin + (x - m.m.box.x) / m.m.box.w * (m.m.xmax - m.m.xmin);
        if (m.kind === 'bscope') R = m.m.ymax - (y - m.m.box.y) / m.m.box.h * (m.m.ymax - m.m.ymin);
        else el = m.m.ymax - (y - m.m.box.y) / m.m.box.h * (m.m.ymax - m.m.ymin);
      } else if (m.kind === 'ppi') {
        const dx = x - m.cx, dy = m.cy - y; R = Math.hypot(dx, dy) / m.rad * m.rmax; az = Math.atan2(dx, dy) * R2D;
      }
      setText('mfdCursorInfo', el == null ? `R: ${R.toFixed(1)} km | Az: ${az.toFixed(1)}°` : `Az: ${az.toFixed(1)}° | El: ${el.toFixed(1)}°`);
    });
    on('mfdCanvas', 'click', e => {
      const sim = state.sim, m = state.mfdBox; if (!m) return;
      const r = e.target.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
      let az = 0, R = 0, el = null;
      if (m.kind === 'bscope' || m.kind === 'hud') {
        az = m.m.xmin + (x - m.m.box.x) / m.m.box.w * (m.m.xmax - m.m.xmin);
        if (m.kind === 'bscope') R = m.m.ymax - (y - m.m.box.y) / m.m.box.h * (m.m.ymax - m.m.ymin);
        else el = m.m.ymax - (y - m.m.box.y) / m.m.box.h * (m.m.ymax - m.m.ymin);
      } else if (m.kind === 'ppi') {
        const dx = x - m.cx, dy = m.cy - y; R = Math.hypot(dx, dy) / m.rad * m.rmax; az = Math.atan2(dx, dy) * R2D;
      } else return;
      let pick = null, bd = 1e9;
      sim.targets.forEach(tg => {
        const st = sim.getTargetTacticalStatus(tg);
        if (!st.isDetected && !state.truth) return; // 未检出目标不可通过点击雷达空域盲选
        const s = sim.targetState(tg, sim.time);
        const dd = el == null ? Math.abs(s.az - az) + Math.abs(s.R / 1000 - R) * 0.15 : Math.abs(s.az - az) + Math.abs(s.el - el);
        if (dd < bd) { bd = dd; pick = tg; }
      });
      if (pick && bd < 6) {
        state.targetId = pick.id;
        const st = sim.getTargetTacticalStatus(pick);
        if (st.track) sim.lockTrack(st.track.id);
        syncTarget(); renderLists(); updateDiagnostics();
        return;
      }
      if (state.slew) patchConfig({ scan: { azCenter: clamp(az, -60, 60), elCenter: el == null ? sim.cfg.scan.elCenter : clamp(el, -25, 25) } });
    });
  }

  /* ---------------------------------------------------------- main loop */
  let lastTs = 0;
  const lastR = { tele: 0, mfd: 0, scan: 0, array: 0, pattern: 0, wf: 0, ascope: 0, rd: 0, diag: 0, lists: 0 };
  function frame(ts) {
    if (!lastTs) lastTs = ts;
    const dt = Math.min(0.1, (ts - lastTs) / 1000); lastTs = ts;
    const sim = state.sim;
    if (sim && !state.paused) sim.step(dt, 12);
    if (ts - lastR.tele > 150) { updateTelemetry(); lastR.tele = ts; }
    if (ts - lastR.mfd > 60) { drawMFD(); lastR.mfd = ts; }
    if (ts - lastR.scan > 120) { drawScan(); lastR.scan = ts; }
    if (ts - lastR.array > 250) { drawArray(); lastR.array = ts; }
    if (ts - lastR.pattern > 350) { drawPattern(); lastR.pattern = ts; }
    if (ts - lastR.wf > 250) { drawWaveform(); lastR.wf = ts; }
    if (ts - lastR.ascope > 250) { drawAscope(); lastR.ascope = ts; }
    if (ts - lastR.rd > 450) { drawRD(); lastR.rd = ts; }
    if (ts - lastR.diag > 320) { updateDiagnostics(); lastR.diag = ts; }
    if (ts - lastR.lists > 800) { renderLists(); lastR.lists = ts; }
    requestAnimationFrame(frame);
  }

  function boot() {
    try {
      bindUI();
      loadScenario('default');
      requestAnimationFrame(frame);
    } catch (err) {
      const el = document.createElement('div');
      el.style.cssText = 'position:fixed;left:8px;bottom:8px;z-index:99;background:#3a0b12;color:#ff8096;border:1px solid #ff3b5c;padding:8px;font:12px monospace;max-width:60vw;white-space:pre-wrap;';
      el.textContent = 'simulator_app.js 启动失败: ' + (err && err.stack || err);
      document.body.appendChild(el);
      throw err;
    }
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();
})();
