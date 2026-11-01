# AN/APG-81X 机载有源相控阵（AESA）火控雷达全物理闭环模拟器

[![AESA Simulation](https://img.shields.io/badge/Radar-AESA%20X--Band-cyan.svg)](https://github.com/midsummer16)
[![Web-Based](https://img.shields.io/badge/Platform-Pure%20HTML5%20%2F%20Canvas%20%2F%20ES6-emerald.svg)](https://github.com/midsummer16)
[![Zero-Dependency](https://img.shields.io/badge/Dependencies-Zero%20(Pure%20Vanilla%20JS)-blue.svg)](https://github.com/midsummer16)
[![Physics Closed-Loop](https://img.shields.io/badge/Physics-Full%20Link%20Budget-amber.svg)](https://github.com/midsummer16)

机载有源相控阵（Active Electronically Scanned Array, AESA）火控雷达工程级全物理闭环模拟器。以典型第四/五代战斗机机头 X 波段火控雷达（基准参考 AN/APG-77、AN/APG-81 同级体制）为蓝本构建。

不同于传统游戏或演示软件中简单的“几何圆锥碰撞检测”，本系统实现了从**千级阵元幅相加权合成 $\rightarrow$ 远场波束方向图 $\rightarrow$ 双程全链路雷达方程（含大气/降雨衰减、STC、杂波与压制干扰）$\rightarrow$ 脉冲压缩与数字信号处理（CFAR/MTD/CRT/单脉冲）$\rightarrow$ 航迹关联滤波与火控包线解算**的完整物理闭环。

---

## 🌟 核心特性与物理机制

### 1. 千级阵元正三角形圆形孔径几何
- **正三角形均匀栅格**：阵元间距满足 $d_x \le \frac{2}{\sqrt{3}} \frac{\lambda}{1 + \sin\theta_{\max}} \approx 0.6188\lambda$。在 $\pm 60^\circ$ 大扫描角下彻底消除栅瓣，相比矩形栅格节省约 $13.4\%$ 阵元。
- **真实硬件量化**：内置 6-bit 数字移相器（$5.625^\circ$ 步进）与 6-bit 数控衰减器（$0.5\text{ dB}$ 步进）量化模型，支持单组件战损/失效掩模动态注入。

### 2. 动态多模式波束赋形
- **均匀加权和波束（$\Sigma$）**：Airy 斑，第一副瓣理论值 $-17.6\text{ dB}$，用于远距搜索（RWS）与合成孔径（SAR）。
- **圆孔径 Taylor 锥削**：副瓣电平抑制至 $-35\text{ dB} \sim -40\text{ dB}$，大幅降低副瓣杂波与干扰灌入。
- **圆孔径 Bayliss 单脉冲差波束（$\Delta_{\text{Az}}, \Delta_{\text{El}}$）**：在波束轴线形成 $<-45\text{ dB}$ 陡峭零深，四象限单脉冲比幅鉴角曲线在主瓣内具备优异的线性测角斜率。
- **唯相二次赋形**：二次相位扰动展宽波束（ACM 近距大空域瞬间捕获）与俯仰余割平方（$\csc^2\theta$）地面扫描增益补偿。
- **空域 MVDR 自适应抗干扰零陷**：在保持主瓣指向不变的前提下，在全部有源压制干扰方向动态生成 $<-50\text{ dB}$ 的深零陷。

### 3. 全链路电磁传播损耗与雷达方程
- **双程自由空间扩散损耗**：严格遵循 $(4\pi)^3 R^4 / \lambda^2$（距离翻倍回波功率下降 $12.04\text{ dB}$）。
- **ITU-R P.676 大气气体吸收**：沿射线路径积分的海拔高度指数递减氧气（$\text{O}_2$）与水汽（$\text{H}_2\text{O}$）吸收衰减模型。
- **ITU-R P.838 降雨与云雾衰减**：基于幂律 $\gamma = k R_{\text{rain}}^\alpha$ 的频变降雨模型（$8 \sim 12\text{ GHz}$ 频变衰减敏感度分析）。
- **接收机控制与遮蔽**：近程灵敏度时间控制（STC）非线性衰减与多 PRF 发射遮蔽（Pulse Eclipsing）损耗。

### 4. 信号处理与多目标火控闭环
- **波形库与匹配滤波**：LFM 线性调频加窗脉压（PSLR $<-40\text{ dB}$）、13-bit 巴克码（Barker-13）、Welch-Costas 跳频码（图钉型模糊函数抗 DRFM 欺骗）。
- **动目标检测与解模糊**：多普勒 MTD 滤波器组、中国剩余定理（CRT）参差 PRF 解距离模糊、双子孔径偏置相位中心（DPCA）主瓣地杂波对消（抑制比 $>38\text{ dB}$）。
- **统计检测与航迹滤波**：Marcum-Q 数值积分与 Swerling 0/1/2 目标起伏模型、滑动窗 CA-CFAR 动态门限、$\alpha$-$\beta$ / 卡尔曼状态预测与动态发射包线（DLZ）解算。

### 5. 十大对空 / 对地工作模式全覆盖
- **对空模式**：`RWS`（测距边搜索）、`TWS`（边扫描边跟踪）、`TAS`（跟踪加搜索）、`MTT`（多目标跟踪）、`DTT`（双目标精确跟踪）、`STT`（单目标连续火控锁定）、`ACM`（近距格斗：轴线/垂直/HUD/随动 4 个子模式）。
- **对地模式**：`GM`（实波束对地测绘）、`SAR`（合成孔径高分辨率灰度成像）、`GMTI / GMTT`（对地动目标指示与敏捷波束跟踪）。

### 6. 电子对抗（ECM）与全维抗干扰（ECCM）闭环
- **敌方干扰样式**：瞄准式窄带噪声（Spot）、阻塞式宽带噪声（Barrage）、自卫式 DRFM 距离拖引假目标（RGPO）、高占空比旁瓣脉冲干扰。
- **雷达 ECCM 对策**：空域 MVDR 自适应零陷、8~12 GHz 宽带伪随机频率捷变、Costas 跨脉冲失配滤波、前沿波门跟踪保护（Leading-Edge Tracking）、双通道旁瓣消隐（SLB）。

---

## 🖥️ 座舱显控台功能模块

打开 `radar_simulator.html` 即可进入航空电子风格的暗色座舱战术显控台（60 FPS 实时运行）：

1. **左侧主战术显示器（MFD）**：
   - 四周配备 **20 个多功能软键（OSBs）**，支持一键切换 10 大工作模式、扫描角度、仰角 Bars、量程、波形与抗干扰体制。
   - 支持 **B-Scope**（方位-距离方显）、**PPI**（对地极坐标扇形）、**HUD**（平显视场投影）与 **Imaging**（实波束地形 / SAR 高分辨灰度）四视图无缝切换。
2. **千级阵元圆形相控阵阵面监视器**：
   - 实时呈现圆阵面上 1500+ 个 TR 组件几何分布、四象限单脉冲子阵划分、归一化幅度锥削分布、移相器相位条纹与战损阵元。
3. **合成波束方向图实时分析仪**：
   - 实时绘制方位向与俯仰向双程截面切面，动态标注 3dB 主瓣宽度（HPBW）、副瓣电平（SLL）与干扰零陷深度。
4. **脉冲压缩 A-Scope 与二维距离-多普勒（R-D）频谱仪**：
   - 展示接收回波经匹配滤波后的距离像、滑动窗 CA-CFAR 动态门限、主瓣地杂波脊线及多普勒频移分布。
5. **分项链路预算详表（Link Budget）**：
   - 实时列出单脉冲能量、同源阵面合成增益、几何失配损耗、双程自由空间损耗、气象衰减、杂波/干扰噪声抬升、净信干噪比（SINR）及单次 / 系统探测概率（$P_d$）。
6. **7 个预设实战想定**：
   - 多批次空地综合态势、远距 VLO 隐身目标突防、强噪声压制支援干扰、自卫 DRFM RGPO 假目标拖引、下视强地杂波地面装甲车队、近距大离轴角格斗、强降雨衰减与频点优选。

---

## 📂 项目结构

```
.
├── code_artifact.md       # 总体工程设计方案与数学规范说明书
├── radar_engine.js        # 核心物理电磁、信号处理与时间线调度引擎（Node.js / 浏览器双用）
├── simulator_app.js       # 前端显控交互逻辑与 Canvas 绘图控制器
├── radar_simulator.html   # 主战术显控台与综合工程分析仪表页面
├── aesa.html              # 交互式总体架构蓝图与数学模型在线验证沙盒
├── test_radar_physics.js  # 自动化物理与数学模型回归测试套件
├── test_ui_contract.js    # UI 契约自动化一致性检验脚本
└── README.md              # 项目说明文档
```

---

## 🚀 快速开始

本项目为**纯原生纯前端零外部依赖实现**，开箱即用：

### 1. 运行火控雷达模拟器
在任何现代浏览器（Chrome / Edge / Firefox / Safari）中直接双击打开：
```bash
# 方式 1：直接浏览器打开本地文件
radar_simulator.html

# 方式 2：使用任何本地静态服务器（可选）
npx serve .
# 或
python3 -m http.server 8080
```
访问 `http://localhost:8080/radar_simulator.html`。

### 2. 查阅交互式架构蓝图与理论沙盒
在浏览器中打开 `aesa.html`，可交互调节参数、查看天线阵列方向图演进与物理公式推导。

### 3. 运行物理引擎测试
```bash
node test_radar_physics.js
```
测试套件覆盖三种栅格拓扑与无栅瓣判据、频变增益与波束宽度、Taylor/Bayliss 方向图、雷达方程同源性、LFM/Barker/Costas 脉压、CA-CFAR 与 CRT 解模糊、以及全部 7 套想定闭环验证。

---

## 📄 许可协议

本项目采用 [MIT License](LICENSE) 授权许可。
