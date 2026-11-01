# AN/APG-81X 机载有源相控阵（AESA）火控雷达全物理闭环模拟器

[![AESA Simulation](https://img.shields.io/badge/Radar-AESA%20X--Band-cyan.svg)](https://github.com/midsummer16)
[![Web-Based](https://img.shields.io/badge/Platform-Pure%20HTML5%20%2F%20Canvas%20%2F%20ES6-emerald.svg)](https://github.com/midsummer16)
[![Zero-Dependency](https://img.shields.io/badge/Dependencies-Zero%20(Pure%20Vanilla%20JS)-blue.svg)](https://github.com/midsummer16)
[![Physics Closed-Loop](https://img.shields.io/badge/Physics-Full%20Link%20Budget%20%2B%20Auto%20FSM-amber.svg)](https://github.com/midsummer16)

机载有源相控阵（Active Electronically Scanned Array, AESA）火控雷达工程级全物理闭环模拟器。以典型第四/五代战斗机机头 X 波段火控雷达（基准参考 AN/APG-77、AN/APG-81 同级体制）为蓝本构建。

本系统实现了从**千级阵元幅相加权合成 $\rightarrow$ 远场波束方向图 $\rightarrow$ 双程全链路雷达方程（含大气/降雨衰减、STC、杂波与压制/欺骗/箔条干扰）$\rightarrow$ 脉冲压缩与数字信号处理（CA/OS-CFAR、MTD、CRT、单脉冲）$\rightarrow$ 航迹关联滤波与全闭环工作状态机自动切换**的完整物理闭环。

---

## 🌟 核心特性与本次审查修复/优化亮点

### 1. 状态之间的全闭环自动切换（Auto FSM）
- **雷达工作模式自动流转**：支持 `RWS（广域搜索）` $\leftrightarrow$ `TWS（边扫边跟）` $\leftrightarrow$ `TAS（敏捷跟踪加搜索）` $\leftrightarrow$ `MTT/DTT（多目标/双目标精确跟踪）` $\leftrightarrow$ `ACM/STT（近距捕获/单目标火控锁定）` 以及 `GMTI` $\leftrightarrow$ `GMTT` 的全自动状态切换，并支持一键切换自动/手动模式。
- **失锁降级与重搜闭环**：当锁定或跟踪目标因强干扰压制、欺骗拖引或脱离扇区而失锁超时，系统自动降级退回 `TAS` / `RWS` / `ACM` 扩大空域重新搜索捕获。
- **实时状态流转拓扑链与事件日志**：顶栏实时高亮当前所处战术阶段（搜索 $\rightarrow$ 捕获 $\rightarrow$ 跟踪 $\rightarrow$ 多目标精跟 $\rightarrow$ 火控锁定 $\leftrightarrow$ 记忆外推），并记录每次状态切换的触发原因与时间戳。

### 2. 各类状态显示与目标显示优化（检测不出的目标严格不显示）
- **严格实测显示策略**：默认关闭真值透视（`TRU = OFF`）。在主战术显示器（B-Scope / PPI / HUD）、扫描波束视图（Scan View）、脉压距离像（A-Scope）、二维距离-多普勒谱（R-D Map）及 SAR 成像视图中，**未检出的目标严格不显示**，彻底杜绝未检出目标泄露真实坐标或代号标签。
- **分级航迹与战术符号系统**：清晰区分原始点迹（Bricks）、副瓣点迹、暂态捕获航迹（虚线框 + 命中计数）、稳定确认航迹（实线战术框 + 速度矢量线）、记忆外推航迹（`[MEM]` 琥珀色虚线框）、假目标告警航迹（`[GHOST]`）以及火控锁定航迹（红色双层菱形框 + 动态发射包线 DLZ）。
- **全域目标检出/隐藏状态监控表**：在右侧诊断面板实时汇总所有目标的探测状态（`已锁定` / `稳定跟踪` / `点迹捕获中` / `记忆外推` / `未检出·已隐藏`）。

### 3. 目标检测不出的原因提示与一键措施建议
- **14 类物理根因深度诊断**：实时诊断发射占空比闭锁（`TX_INHIBIT`）、地球曲率视距遮挡（`NO_LOS`）、超出量程（`TOO_FAR`）、扫描扇区外失配（`OUT_OF_SCAN`）、窄带瞄准/宽带阻塞/扫频灵巧噪声压制（`JAM_SPOT` / `JAM_BARRAGE` / `JAM_SWEPT`）、距离/速度欺骗拖引（`DECEPTION_THREAT`）、下视主瓣地杂波淹没（`CLUTTER_MAINLOBE`）、无源箔条走廊遮蔽（`CLUTTER_CHAFF`）、脉冲遮蔽盲区（`ECLIPSED`）、高频强降雨衰减（`RAIN_ATTEN`）、隐身目标低散射截面（`STEALTH_LOW_RCS`）及远距 $R^4$ 衰减（`RANGE_ATTEN`）。
- **量化措施建议与一键修复**：针对每一项未检出原因，给出具体物理对策、预期信干噪比改善量（dB），并提供 **[⚡ 一键执行该措施]** 按钮，点击即可自动完成参数调优并恢复目标检出。

### 4. 完备电子对抗（8类 ECM 干扰）与全维抗干扰（10项 ECCM 措施）
- **8 类干扰样式（ECM）**：
  1. `Spot`：窄带瞄准式噪声压制干扰
  2. `Barrage`：4 GHz 全频段宽带阻塞式噪声干扰
  3. `Swept / Smart Noise`：扫频与灵巧伴随式噪声干扰（污染 CFAR 参考窗造成门限抬升）
  4. `DRFM (RGPO)`：数字射频存储距离波门拖引与多假目标欺骗
  5. `VGPO`：速度波门拖引多普勒欺骗干扰（拉偏卡尔曼滤波器速度估计）
  6. `RVGPO`：距离-速度联合复合拖引欺骗干扰
  7. `Pulse`：旁瓣异步高功率脉冲干扰
  8. `Chaff`：无源箔条走廊 / 偶极子云团强杂波干扰
- **10 项抗干扰措施（ECCM）**：
  1. `Auto-ECCM`：认知自适应抗干扰总开关（自动识别电磁威胁并闭合最优对策组合）
  2. `ADBF`：MVDR 空域自适应方向图零陷（在有源干扰方向生成 $<-50\text{ dB}$ 深零陷）
  3. `Freq Agility`：8~12 GHz 全频段脉间伪随机频率捷变
  4. `DPCA`：双子孔径偏置相位中心主瓣地杂波相消（抑制比 $>38\text{ dB}$）
  5. `MTD-Notch`：自适应动目标零速/箔条杂波凹口滤波（抑制无源箔条云 $>50\text{ dB}$）
  6. `OS-CFAR`：有序统计量恒虚警检测（消除灵巧伴随噪声与箔条边缘引起的门限自遮蔽）
  7. `Leading-Edge`：距离波门前沿跟踪（抗 RGPO 距离后拖）
  8. `Doppler-Gate`：多普勒-距离变化率运动学一致性鉴别（抗 VGPO/RVGPO 速度拖引）
  9. `SLB`：双通道保护天线旁瓣匿影（抗异步脉冲干扰）
  10. `Burn-Through`：高能量凝视烧穿模式（集中驻留脉冲相参积累与峰值功率烧穿宽带阻塞干扰）

---

## 🚀 快速验证与测试

```bash
# 运行物理引擎与新增特性回归测试 (14 项测试)
node test_radar_physics.js

# 运行 UI 控件契约一致性测试
node test_ui_contract.js
```
