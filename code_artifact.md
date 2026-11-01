# 机载有源相控阵（AESA）火控雷达模拟器总体工程设计方案

---

## 1. 引言与理论依据

### 1.1 设计背景与仿真定位
现代超视距（BVR）与视距内（WVR）空战以及全天候精确对地打击高度依赖机载有源相控阵（Active Electronically Scanned Array, AESA）火控雷达。传统模拟器往往只停留在简单的“几何圆锥碰撞检测”层面，忽略了**千级阵元幅相合成方向图、不同工作模式下的波束赋形差异、距离四次方与大气降雨双程衰减、脉冲多普勒（PD）信号处理链以及复杂电磁对抗（ECM/ECCM）**等核心物理过程。

本方案旨在构建一套**物理机理严格正确、前后端高内聚低耦合、支持十大空地工作模式与全链路抗干扰能力**的工程级火控雷达模拟器。

### 1.2 核心参考资料与国际标准
为保证模拟器物理与数学模型的严谨性，本方案严格遵循以下雷达工程权威文献与国际标准：
1. **机载脉冲多普勒与火控体制**：G. W. Stimson, *Introduction to Airborne Radar* (3rd Edition, SciTech Publishing).
2. **相控阵天线与圆形口径赋形**：R. J. Mailloux, *Phased Array Antenna Handbook* (Artech House); M. I. Skolnik, *Radar Handbook* (3rd Edition, McGraw-Hill).
3. **雷达信号处理与波形模糊函数**：B. R. Mahafza, *Radar Systems Analysis and Design Using MATLAB*; M. A. Richards, *Fundamentals of Radar Signal Processing*.
4. **圆口径 Taylor 与 Bayliss 单脉冲综合**：T. T. Taylor, *"Design of circular apertures for narrow beamwidth and low sidelobes"*, IRE Trans. Antennas Propag., 1960; E. T. Bayliss, *"Design of monopulse antenna difference patterns with low sidelobes"*, Bell System Technical Journal, 1968.
5. **电磁波大气与气象衰减标准**：
   - **ITU-R P.676-13**：大气气体（氧气 $\text{O}_2$ 与水汽 $\text{H}_2\text{O}$）双程吸收衰减模型。
   - **ITU-R P.838-3**：不同降雨率下的微波特定衰减系数模型。

### 1.3 模拟雷达总体技术基准参数
本模拟器以典型第四/五代战斗机机头 X 波段 AESA 火控雷达（参考 AN/APG-77、AN/APG-81 同级物理体制）为基准设定默认工程参数：

| 参数类别 | 符号 | 典型设定值 | 物理与工程说明 |
| :--- | :--- | :--- | :--- |
| 工作中心频率 / 波长 | $f_0, \lambda$ | $f_0 = 10.0\text{ GHz}, \lambda = 0.03\text{ m}$ | 标准 X 波段机载火控频段（支持 $8.5 \sim 10.5\text{ GHz}$ 捷变） |
| 圆形阵面有效物理直径 | $D$ | $D = 0.70\text{ m}$ (半径 $R = 0.35\text{ m}$) | 适配典型战斗机机头圆形雷达罩截面 |
| 阵元数量与排布拓扑 | $N$ | $N = 1364 \sim 1680$ 个 TR 组件 | 正三角形均匀栅格（Equilateral Triangular Grid）圆形截断 |
| 单个 TR 组件峰值功率 | $P_{\text{TR}}$ | $10\text{ W} \sim 12\text{ W}$ (GaN/GaAs) | 全阵面合成峰值发射功率 $P_t \approx 13.6\text{ kW} \sim 20\text{ kW}$ |
| 移相器与数控衰减器位数 | $B_\varphi, B_A$ | $6\text{-bit}$ 移相器, $6\text{-bit}$ 衰减器 | 相位步进 $\Delta\varphi = 5.625^\circ$，幅度动态范围 $31.5\text{ dB}$ |
| 最大无栅瓣电子扫描角 | $\theta_{\max}$ | 方位 $\pm 60^\circ$，俯仰 $\pm 60^\circ$ | 满足大离轴角探测与中距弹大偏角中继制导 |
| 接收机噪声系数与系统损耗 | $F_n, L_{\text{sys}}$ | $F_n = 3.2\text{ dB}, L_{\text{sys}} = 3.8\text{ dB}$ | 包含雷达罩透波损耗、馈线损耗与量化损耗 |

---

## 2. 系统前后端分离总体架构设计

### 2.1 分层解耦设计思想
由于**千级阵元的二维复数方向图合成、宽带脉冲压缩（Matched Filtering）、动目标显示/检测（MTD/FFT）、合成孔径成像（SAR）及空时自适应处理（STAP/MVDR）**属于高密度矩阵运算，而**雷达多视图显控界面**需要保持 $60\text{ FPS}$ 的高帧率渲染，系统采用**“后端高实时物理与信号流计算引擎 + 双通道通信总线 + 前端 GPU 加速综合战术显控台”**的分离架构。

```
+-----------------------------------------------------------------------------------+
|                        前端：座舱综合战术显控与工程可视化终端                        |
|  +-----------------+ +-----------------+ +-----------------+ +-----------------+  |
|  | 主战术显示器     | | 千级阵元圆阵面   | | 2D/3D合成波束   | | 基带波形/脉压   |  |
|  | B-Scope/PPI/SAR | | 幅相与TR状态视图 | | 方向图实时分析仪 | | R-D多普勒频谱仪 |  |
|  +-----------------+ +-----------------+ +-----------------+ +-----------------+  |
|  +-----------------------------------------------------------------------------+  |
|  | 多功能显示器软键(MFD OSBs) · 十大工作模式切换 · 扫描空域控制 · ECCM抗干扰面板 |  |
|  +-----------------------------------------------------------------------------+  |
+------------------------------------^-------------------------|--------------------+
                                     |                         |
                     下行高速遥测流   | WebSocket Binary        | 上行控制流
                     (Float32Array)  | (30 ~ 60 Hz)            | JSON REST / WS
                                     |                         v
+------------------------------------|-------------------------v--------------------+
|                        后端：物理电磁环境与雷达信号处理引擎                          |
|  +-----------------------------------------------------------------------------+  |
|  | 1. 战场态势与电磁环境推演层：载机6-DOF、目标机动与Swerling RCS、ECM干扰源     |  |
|  +-----------------------------------------------------------------------------+  |
|  | 2. AESA阵面与微秒级时间线调度器：千元圆阵面坐标、幅相权值w_n、TAS/MTT驻留队列 |  |
|  +-----------------------------------------------------------------------------+  |
|  | 3. 电磁传播与回波合成层：双程R^4损耗、ITU-R大气/降雨衰减、Morchin地海杂波     |  |
|  +-----------------------------------------------------------------------------+  |
|  | 4. 基带信号处理流水线：波形产生 -> MVDR抗干扰 -> 脉冲压缩 -> MTD -> 2D-CFAR   |  |
|  +-----------------------------------------------------------------------------+  |
|  | 5. 数据处理与火控解算层：单脉冲测角 -> JPDA关联 -> IMM-UKF滤波 -> 攻击包线解算 |  |
|  +-----------------------------------------------------------------------------+  |
+-----------------------------------------------------------------------------------+
```

### 2.2 后端五大核心引擎模块职责
1. **战场态势与电磁环境推演层 (Scenario & Kinematics Engine)**：
   - 维护载机六自由度（6-DOF）位置、速度、姿态角（横滚、俯仰、偏航）与天线稳定坐标系转换。
   - 计算空中/地面多目标运动轨迹，并根据视线角（LOS）与 Swerling I $\sim$ IV 起伏模型动态更新目标雷达散射截面积（RCS, $\sigma$）。
   - 维护电子战干扰源（远距支援干扰机 SOJ、自卫干扰机 SSJ）的空间坐标、干扰样式与等效辐射功率（ERP）。
2. **AESA 阵面与时间线波束调度器 (Aperture & Timeline Scheduler)**：
   - 存储圆阵面 $>1000$ 个阵元的物理坐标 $(x_n, y_n)$、子阵归属与故障掩码 $g_n \in \{0, 1\}$。
   - 根据当前工作模式实时解算每个阵元的 $6\text{-bit}$ 幅度权值 $I_n$ 与相位权值 $\varphi_n$。
   - 维护相控阵驻留时间线（Dwell Timeline），实现搜索波束、跟踪确认波束与火控凝视波束的微秒级切换。
3. **传播损耗与杂波/回波合成层 (Propagation & Echo Generator)**：
   - 计算自由空间 $R^4$ 扩散损耗、ITU-R P.676 氧气/水汽吸收损耗、ITU-R P.838 降雨损耗以及大扫描角孔径投影余弦损耗。
   - 合成目标基带复包络回波、接收机热噪声、主瓣/旁瓣地面杂波与有源压制/欺骗干扰信号。
4. **基带信号处理流水线 (Signal Processing Pipeline)**：
   - 执行空域自适应波束形成（MVDR/LCMV 零陷）、数字正交下变频（DDC）、匹配滤波脉冲压缩、多普勒 FFT 滤波器组（MTD）、余数定理（CRT）解距离/速度模糊、二维恒虚警检测（CA-CFAR / OS-CFAR）以及 SAR 距离多普勒（R-D）成像算法。
5. **航迹管理与火控解算层 (Data Processing & Fire Control)**：
   - 执行四象限和差单脉冲比幅测角（$\Delta_{\text{Az}}/\Sigma, \Delta_{\text{El}}/\Sigma$），完成点迹凝聚。
   - 采用联合概率数据关联（JPDA）与交互多模型无迹卡尔曼滤波（IMM-UKF）维持多目标航迹，解算空空/空地武器动态发射包线（$R_{\max}, R_{\text{ne}}, R_{\min}$）。

### 2.3 前端五大可视化显控模块职责
1. **主战术雷达显示器 (Primary Tactical Display)**：
   - **对空模式（RWS/TWS/TAS/MTT/DTT/STT）**：标准方位-距离 **B-Scope** 视图，右侧集成俯仰波束层（Elevation Bars）与目标高度标尺，叠加原始检测点迹（Bricks）、关联航迹符号、速度矢量线、攻击包线与干扰源方位告警扇区。
   - **近距格斗模式（ACM）**：**HUD 平显视场投影视图**，动态绘制轴线圆圈（Boresight）、垂直扫描带（Vertical Scan）与平显框（HUD Scan）。
   - **对地模式（GM/SAR/GMTI/GMTT）**：**扇形 PPI 极坐标实波束地形视图**与高分辨率 **SAR 灰度成像瀑布视图**，并在 GMTI/GMTT 模式下高亮叠加地面慢速移动车队矢量。
2. **千级阵元圆形相控阵阵面监控视图 (Circular Aperture Telemetry View)**：
   - 实时渲染圆阵面上 $>1000$ 个 TR 组件的几何位置、四象限单脉冲分区（$Q_1 \sim Q_4$）、归一化幅度加权热力图、移相器相位条纹以及失效/战损阵元。
3. **合成波束方向图分析仪 (2D/3D Beam Pattern Analyzer)**：
   - 实时绘制方位维/俯仰维远场方向图切面（单位 $\text{dBi}$ 或归一化 $\text{dB}$）及二维 $u\text{-}v$ 空间方向图热力分布，直观展示不同模式下的主瓣展宽、副瓣抑制、单脉冲差波束零深以及对干扰源方向的自适应零陷。
4. **基带波形、脉冲压缩与距离-多普勒频谱仪 (Waveform & R-D Analyzer)**：
   - 显示发射波形时域实部/虚部、时频调制脊线（LFM / Costas / 捷变）、脉冲压缩后的距离像（A-Scope）与动态 CFAR 门限曲线、以及二维距离-多普勒（Range-Doppler）矩阵图。
5. **多功能软键与电子对抗控制面板 (MFD OSBs & ECCM Control Panel)**：
   - 仿真实战机载多功能显示器周边按键（OSB），支持实时切换十大工作模式、调整扫描方位宽度（$\pm 10^\circ / \pm 30^\circ / \pm 60^\circ$）、俯仰行数（$1 / 2 / 4 / 8\text{ Bars}$）、发射波形以及抗干扰开关。

---

## 3. 千级阵元圆形相控阵阵面与多模式波束赋形设计

### 3.1 圆形阵面均匀栅格排布与无栅瓣定理推导
机载相控阵天线安装在战斗机机头圆锥形雷达罩内，因此阵面口径必然设计为半径 $R = D/2$ 的圆形。为了在二维大角度扫描（最大扫描角 $\theta_{\max}$）时不在可见空间（Visible Space, $u^2 + v^2 \le 1$）内出现栅瓣（Grating Lobes），阵元间距必须满足空间奈奎斯特准则。

#### （1）矩形栅格（Rectangular Grid）与正三角形栅格（Equilateral Triangular Grid）对比
- **矩形均匀栅格**：阵元沿正交的 $x, y$ 轴等间距排布，间距 $d_x = d_y = d$。无栅瓣条件为：
  $$d \le \frac{\lambda}{1 + \sin\theta_{\max}}$$
  当 $\theta_{\max} = 60^\circ$ 时，$d \le \frac{\lambda}{1 + \sin 60^\circ} \approx 0.5359\lambda$。单个阵元占据的晶胞面积为：
  $$A_{\text{rect}} = d_x \cdot d_y \le 0.2872\lambda^2$$

- **正三角形均匀栅格（工程首选方案）**：奇数行与偶数行在水平方向交错偏移 $\frac{1}{2}d_x$，行间距 $d_y = \frac{\sqrt{3}}{2}d_x$。由于倒易晶格（Reciprocal Lattice）同样呈六边形对称，栅瓣首先出现在对角线方向，其无栅瓣条件放宽为：
  $$d_x \le \frac{2}{\sqrt{3}} \frac{\lambda}{1 + \sin\theta_{\max}} \approx 0.6188\lambda, \quad d_y = \frac{\sqrt{3}}{2}d_x \le 0.5359\lambda$$
  单个阵元占据的六边形晶胞面积为：
  $$A_{\text{tri}} = d_x \cdot d_y \le \frac{2}{\sqrt{3}} (0.5359\lambda)^2 \approx 0.3316\lambda^2$$

> **核心结论**：在完全相同的圆形口径面积 $\pi R^2$ 和相同的最大无栅瓣扫描角 $\theta_{\max} = 60^\circ$ 下，采用**正三角形均匀栅格**相比矩形栅格可减少 $\left(1 - \frac{\sqrt{3}}{2}\right) \approx 13.4\%$ 的阵元数量，显著降低机头重量、散热负荷与造价，同时在圆形边界内具备更优的圆对称副瓣特性。

#### （2）千级阵元数量验证与坐标生成公式
取圆形口径直径 $D = 0.70\text{ m}$ ($R = 0.35\text{ m}$)，波长 $\lambda = 0.03\text{ m}$，留出工程安全裕度取正三角形栅格参数 $d_x = 0.56\lambda = 0.0168\text{ m}$，$d_y = \frac{\sqrt{3}}{2}d_x = 0.01455\text{ m}$。
第 $(m, n)$ 网格点的二维笛卡尔坐标 $(x_{m,n}, y_{m,n})$ 由下式生成：
$$y_m = m \cdot d_y, \quad x_{m,n} = \left(n + \frac{|m| \bmod 2}{2}\right) d_x$$
仅保留满足圆形口径截断判据的阵元：
$$r_{m,n} = \sqrt{x_{m,n}^2 + y_{m,n}^2} \le R - \frac{d_x}{2}$$
总阵元数理论值为：
$$N \approx \frac{\pi R^2}{d_x d_y} = \frac{\pi \times 0.35^2}{0.0168 \times 0.01455} \approx 1574 \text{ 个 TR 组件}$$
当口径 $D$ 在 $0.60\text{ m} \sim 0.76\text{ m}$ 之间调整时，阵元总数稳定处于 $1150 \sim 1850$ 个之间，严格满足“千个以上均匀分布在圆形阵面”的设计指标。

### 3.2 四象限单脉冲子阵划分与远场合成数学模型
将圆形阵面按笛卡尔象限划分为四个对称的单脉冲子阵（Quadrants）：
- 第一象限 $Q_1$：$x_n \ge 0, y_n \ge 0$
- 第二象限 $Q_2$：$x_n < 0, y_n \ge 0$
- 第三象限 $Q_3$：$x_n < 0, y_n < 0$
- 第四象限 $Q_4$：$x_n \ge 0, y_n < 0$

引入方向余弦坐标（Sine Space Coordinates）：
$$u = \sin\theta\cos\phi, \quad v = \sin\theta\sin\phi$$
当波束指向目标方向 $(u_0, v_0)$ 时，记波数 $k = \frac{2\pi}{\lambda}$，考虑 $B_\varphi = 6\text{-bit}$ 数字移相器的量化算子 $\mathcal{Q}_\varphi(\cdot)$ 与 $B_A = 6\text{-bit}$ 数控衰减器的量化算子 $\mathcal{Q}_A(\cdot)$，第 $n$ 个阵元的复数激励权值为：
$$w_n(u_0, v_0) = g_n \cdot \mathcal{Q}_A(I_n) \cdot \exp\left( j \mathcal{Q}_\varphi\left( -k(x_n u_0 + y_n v_0) + \Delta\varphi_n \right) \right)$$
其中：
- $g_n \in \{0, 1\}$ 为阵元健康标志（$1$ 为正常，$0$ 为故障失效）；
- $I_n \in [0, 1]$ 为口径幅度锥削权值；
- $\Delta\varphi_n$ 为附加非线性赋形相位（用于宽波束展宽、余割平方赋形或空域自适应零陷）。

单个微带贴片/Vivaldi 阵元的单元方向图包络建模为余弦滚降形式：
$$f_e(\theta) = (\cos\theta)^{\kappa}, \quad \kappa \approx 0.6 \sim 0.75$$
则全阵面合成的远场复数辐射方向图 $F(u, v)$ 为：
$$F(u, v) = f_e(\theta) \sum_{n=1}^{N} w_n(u_0, v_0) \exp\left( j k (x_n u + y_n v) \right)$$

在接收端，四个象限子阵分别独立求和后通过单脉冲和差比较网络（Monopulse Comparator），同时形成三个正交波束通道：
$$\begin{aligned}
\Sigma(u, v) &= F_{Q_1}(u, v) + F_{Q_2}(u, v) + F_{Q_3}(u, v) + F_{Q_4}(u, v) && \text{（和波束：用于测距、检波与SAR成像）} \\
\Delta_{\text{Az}}(u, v) &= \left(F_{Q_1}(u, v) + F_{Q_4}(u, v)\right) - \left(F_{Q_2}(u, v) + F_{Q_3}(u, v)\right) && \text{（方位差波束：用于方位精密测角）} \\
\Delta_{\text{El}}(u, v) &= \left(F_{Q_1}(u, v) + F_{Q_2}(u, v)\right) - \left(F_{Q_3}(u, v) + F_{Q_4}(u, v)\right) && \text{（俯仰差波束：用于俯仰精密测角）}
\end{aligned}$$

### 3.3 不同工作模式下的波束合成法则
模拟器根据当前激活的工作模式，动态重构阵面权值向量 $\mathbf{w} = [w_1, w_2, \dots, w_N]^T$：

#### 1. 均匀加权高增益和波束（适用模式：RWS 远距搜索、SAR 聚束/条带成像）
发射端所有 TR 组件工作在饱和功放状态，接收端采用均匀加权 $I_n = 1, \Delta\varphi_n = 0$。连续圆形孔径的极限远场方向图为经典的艾里斑（Airy Pattern）：
$$F_{\text{uniform}}(\theta) \propto \frac{2 J_1(k R \sin\theta)}{k R \sin\theta}$$
- **波束特征**：获得物理孔径允许的最高峰值增益 $G_0 = \eta \left(\frac{\pi D}{\lambda}\right)^2 \approx 37.3\text{ dBi}$ 与最窄半功率波束宽度 $\theta_{3\text{dB}} \approx 1.02 \frac{\lambda}{D} \approx 2.5^\circ$，第一副瓣电平为 $-17.6\text{ dB}$。

#### 2. 圆 Taylor 低副瓣和波束（适用模式：TWS、TAS、MTT、GMTI）
在下视搜索与多目标跟踪时，为防止强地面杂波与敌方干扰从副瓣灌入，接收端采用**圆孔径 Taylor 加权**（设定设计副瓣电平 $\text{SLL} = -35\text{ dB} \sim -40\text{ dB}$，过渡零点数 $\bar{n} = 6$）。归一化径向坐标 $\rho_n = r_n / R \in [0, 1]$ 的幅度分布函数为：
$$I_{\text{Taylor}}(\rho_n) = \frac{1}{\pi} \left[ 1 + 2 \sum_{m=1}^{\bar{n}-1} F_m J_0(\mu_m \pi \rho_n) \right]$$
其中 $\mu_m$ 为一阶贝塞尔函数 $J_1(\pi \mu_m) = 0$ 的正根，$F_m$ 为圆 Taylor 展开系数。工程实时计算亦可采用高拟合度径向台座多项式近似：
$$I_{\text{Taylor}}(\rho_n) \approx \alpha_0 + (1 - \alpha_0)(1 - \rho_n^2)^p, \quad (\alpha_0 = 0.20, p = 1.45)$$
- **波束特征**：第一及远区副瓣全面压低至 $-35\text{ dB}$ 以下，主瓣宽度略展宽至 $\theta_{3\text{dB}} \approx 1.22 \frac{\lambda}{D} \approx 3.0^\circ$，口径效率损耗约 $0.9\text{ dB}$。

#### 3. 圆 Bayliss 低副瓣单脉冲差波束（适用模式：STT、DTT、MTT 精密测角）
在单目标/双目标精确火控锁定时，差波束通过**圆孔径 Bayliss 综合**同时实现高测角斜率与低差副瓣（$-35\text{ dB}$）。方位差与俯仰差通道的幅度加权为：
$$I_{\Delta\text{Az}}(x_n, y_n) = \frac{|x_n|}{R} \sum_{m=1}^{\bar{n}-1} B_m J_1\left(\nu_m \frac{r_n}{R}\right), \quad \Delta\varphi_n = \begin{cases} 0, & x_n \ge 0 \\ \pi, & x_n < 0 \end{cases}$$
- **波束特征**：在波束指向轴线上形成深度达 $-45\text{ dB} \sim -55\text{ dB}$ 的陡峭零深，归一化单脉冲鉴角曲线：
  $$\varepsilon_{\text{Az}} = \frac{1}{k_m} \text{Re}\left\{ \frac{\Delta_{\text{Az}}}{\Sigma} \right\}$$
  在主瓣内呈严格线性，测角精度达到 $\sigma_\theta \le 0.1\text{ mrad}$。

#### 4. 唯相二次赋形展宽与余割平方波束（适用模式：ACM 近距格斗、GM 对地实波束扫描）
发射端 TR 组件必须工作在非线性饱和区以维持最大功率，不能通过改变幅度 $I_n$ 赋形。因此采用**唯相赋形（Phase-Only Beam Spoiling）**：
- **ACM 宽波束快速捕获**：附加径向二次相差 $\Delta\varphi_n = \beta (x_n^2 + y_n^2) / R^2$，将铅笔波束展宽 $2.5 \sim 4$ 倍，实现近距大立体角瞬间截获。
- **GM 对地扫描余割平方赋形（$\csc^2\theta_{\text{dep}}$）**：附加俯仰维非线性相位分布 $\Delta\varphi_n(y_n)$，使俯仰方向图功率增益满足：
  $$G(\theta_{\text{dep}}) \propto \csc^2(\theta_{\text{dep}}) \cos^{1/2}(\theta_{\text{dep}}), \quad \theta_{\text{dep}} \in [\theta_1, \theta_2]$$
  从而补偿不同俯角下斜距 $R = H \csc\theta_{\text{dep}}$ 带来的回波衰减，保证近距与远距地面散射单元在 PPI 显示器上的亮度均匀一致。

#### 5. 空域 MVDR 自适应抗干扰零陷波束（适用模式：全模式 ECCM 激活态）
当检测到 $K$ 个方向为 $(u_{J,k}, v_{J,k})$ 的强有源压制干扰时，采用最小方差无失真响应（MVDR / Capon）准则求解最优复权向量：
$$\min_{\mathbf{w}} \mathbf{w}^H \mathbf{R}_{i+n} \mathbf{w} \quad \text{s.t.} \quad \mathbf{w}^H \mathbf{a}(u_0, v_0) = 1$$
其中 $\mathbf{a}(u, v) = \left[ e^{jk(x_1 u + y_1 v)}, \dots, e^{jk(x_N u + y_N v)} \right]^T$ 为空间导向矢量，$\mathbf{R}_{i+n}$ 为干扰加噪声协方差矩阵。其闭式解为：
$$\mathbf{w}_{\text{MVDR}} = \frac{\mathbf{R}_{i+n}^{-1} \mathbf{a}(u_0, v_0)}{\mathbf{a}^H(u_0, v_0) \mathbf{R}_{i+n}^{-1} \mathbf{a}(u_0, v_0)}$$
为降低千级阵元直接矩阵求逆的实时开销，后端采用**子阵级自适应波束形成（Subarray ADBF）**或**正交投影空间零陷算法**：
$$\mathbf{w} = \mathbf{w}_{\text{Taylor}} - \sum_{k=1}^{K} \frac{\mathbf{a}^H(u_{J,k}, v_{J,k}) \mathbf{w}_{\text{Taylor}}}{\|\mathbf{a}(u_{J,k}, v_{J,k})\|^2} \mathbf{a}(u_{J,k}, v_{J,k})$$
可在保持主瓣指向 $(u_0, v_0)$ 不变的前提下，在全部干扰源方向自动生成 $<-50\text{ dB}$ 的深零陷。

---

## 4. 十大工作模式状态机与时间线调度设计

### 4.1 十大工作模式技术指标全景表
模拟器内置十大核心工作模式（对空 7 种 + 对地 3 种），其状态转换与参数映射如下表所示：

| 模式代号 | 模式全称 | 类别 | 阵面波束合成类型 | 扫描空域 / 驻留调度体制 | PRF 与波形体制 | 核心信号与数据处理链 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **RWS** | Range While Search<br>测距边搜索 | 对空 | 均匀/轻度 Taylor 高增益和波束 $\Sigma$ | 方位 $\pm 10^\circ/\pm 30^\circ/\pm 60^\circ$<br>俯仰 $1/2/4/8\text{ Bars}$ 光栅扫描 | HPRF 与 MPRF 帧间/行间交替；宽脉冲 LFM | 脉冲压缩 $\rightarrow$ MTD/FFT $\rightarrow$ 2D-CFAR $\rightarrow$ CRT 解模糊 $\rightarrow$ 原始点迹（Bricks） |
| **TWS** | Track While Scan<br>边扫描边跟踪 | 对空 | 圆 Taylor $-35\text{ dB}$ 和差波束 $\Sigma, \Delta_{\text{Az}}, \Delta_{\text{El}}$ | 限制帧周期 $\le 2.0\text{ s}$（如 $\pm 30^\circ \times 4\text{ Bars}$ 或 $\pm 60^\circ \times 2\text{ Bars}$） | MPRF $3\text{-of-}8$ 参差解模糊；中脉宽 LFM/Barker | 单脉冲测角 $\rightarrow$ JPDA 关联 $\rightarrow$ IMM-UKF 滤波（同时维持 $10 \sim 20$ 条航迹） |
| **TAS** | Track And Search<br>跟踪加搜索 | 对空<br>(AESA核心) | 背景搜索 Taylor 波束 + 敏捷单脉冲跟踪波束 | $\pm 60^\circ$ 全空域搜索中微秒级穿插独立目标跟踪驻留 | 搜索 H/MPRF + 跟踪波束按目标距离定制无盲区 PRF | 时间线优先级抢占调度 + 独立波门卡尔曼状态更新（搜索与跟踪互不妥协） |
| **MTT** | Multiple Target Track<br>多目标精确跟踪 | 对空<br>火控 | 全时间线敏捷圆 Taylor/Bayliss 单脉冲 $\Sigma, \Delta$ | 暂停广域搜索，$100\%$ 时间线轮流凝视 $4 \sim 8$ 个高威胁目标（$10\text{ Hz}$/目标） | 各目标独立最优 MPRF/HPRF；宽带 LFM/Costas | 高精度单脉冲测角 + 距离/速度波门锁定 + 多枚中距弹数据链中继制导指令解算 |
| **DTT** | Dual Target Track<br>双目标精确跟踪 | 对空<br>火控 | 双子孔径同时双波束或高频分时圆 Bayliss $\Sigma, \Delta$ | 主目标（Primary）与次目标（Secondary）以 $25 \sim 50\text{ Hz}$ 极高率同步更新 | 高占空比 MPRF / HPRF 交替 | 双通道独立闭环距离/速度/角度鉴别器 + 双目标攻击包线同步解算 |
| **STT** | Single Target Track<br>单目标连续跟踪 | 对空<br>火控 | 全孔径圆 Bayliss 极低零深单脉冲 $\Sigma, \Delta_{\text{Az}}, \Delta_{\text{El}}$ | $100\%$ 能量连续凝视单一锁定目标（更新率 $100 \sim 200\text{ Hz}$） | HPRF 速度波门 + MPRF 距离波门；窄脉宽宽带波形 | 早-迟门距离跟踪环 + 锁频/锁相速度跟踪环 + 单脉冲角跟踪环 + 航炮/导弹连续火控解算 |
| **ACM** | Air Combat Mode<br>近距空战格斗 | 对空<br>格斗 | 唯相二次赋形展宽捕获波束 $\rightarrow$ 锁定后自动切 Bayliss | 轴线 $3^\circ \times 3^\circ$ / 垂扫 $10^\circ \times 60^\circ$ / HUD $20^\circ \times 20^\circ$ / 可调扇区 | 固定 $18.5\text{ km}$ ($10\text{ NM}$) 量程 MPRF；Barker-13 短脉冲 | 近距 STC 控制 $\rightarrow$ 快速 CFAR 判决 $\rightarrow$ 首目标过门限零延迟自动跳转 **STT** |
| **GM** | Ground Mapping<br>实波束对地扫描 | 对地 | 俯仰余割平方 $\csc^2\theta$ 赋形 + 方位窄波束 | 方位 $\pm 15^\circ/\pm 30^\circ/\pm 60^\circ$ 扇形地面扫描 | LPRF（无距离模糊）；中宽带 LFM | 脉冲压缩包络检波 $\rightarrow$ 地物后向散射 $\sigma^0(\theta)$ 映射 $\rightarrow$ 极坐标转直角坐标地形渲染 |
| **SAR** | Synthetic Aperture Radar<br>合成孔径雷达成像 | 对地<br>成像 | 聚束式（Spotlight）持续凝视波束或条带式（Stripmap） | 波束斜视/正侧视持续锁定地面成像测绘带（含运动补偿） | 超宽带 LFM ($B = 150 \sim 500\text{ MHz}$)，相参 PRF | 运动补偿（MoComp）$\rightarrow$ 距离向脉压 $\rightarrow$ 距离徙动校正（RCMC）$\rightarrow$ 方位向 FFT（R-D 成像） |
| **GMTI / GMTT** | Ground Moving Target Ind./Track<br>对地动目标检测与跟踪 | 对地<br>动目标 | 圆阵面左右子阵分割（DPCA / STAP 多通道）+ 敏捷单脉冲 | 扇形扫描检测（GMTI）+ 敏捷波束高重访率单脉冲跟踪（GMTT） | 相参 MPRF 脉冲串 | 子阵空时自适应处理（STAP）/ 相位中心偏置（DPCA）主瓣地杂波对消 $\rightarrow$ 慢速目标 CFAR 与卡尔曼跟踪 |

### 4.2 AESA 微秒级时间线（Timeline）驻留调度模型
在 **TAS、MTT、DTT、GMTT** 等相控阵特有模式下，后端通过一个基于最小堆（Min-Heap）的时间线调度器分配雷达每一毫秒的波束指向。设单个相参处理间隔（CPI / Dwell）时长为 $T_{\text{dwell}} \in [1.5\text{ ms}, 8.0\text{ ms}]$，电子移相切换耗时 $T_{\text{switch}} \approx 10\text{ }\mu\text{s}$。

在 **TAS（跟踪加搜索）** 模式下，时间帧周期 $T_{\text{frame}}$ 内用于目标跟踪的时间占空比 $\eta_{\text{track}}$ 满足：
$$\eta_{\text{track}} = \sum_{m=1}^{M_{\text{track}}} f_{u, m} \cdot N_{\text{PRF}, m} \cdot T_{\text{CPI}, m} \le \eta_{\max} \quad (\text{通常设定 } \eta_{\max} = 40\%)$$
其中 $M_{\text{track}}$ 为当前维持的独立跟踪目标数，$f_{u, m}$ 为第 $m$ 个目标的自适应重访率（高机动或导弹制导目标 $f_{u, m} = 10\text{ Hz}$，平稳飞行目标 $f_{u, m} = 1 \sim 2\text{ Hz}$）。剩余 $1 - \eta_{\text{track}} \ge 60\%$ 的时间片全部用于执行大空域背景搜索，从而在物理上真实还原 AESA 雷达“边全空域搜索、边高精度独立波束跟踪”的核心能力。

### 4.3 对地模式（GM / SAR / GMTI-GMTT）核心算法模型
1. **GM 实波束地形散射建模**：
   地面网格单元 $(x_g, y_g, z_g)$ 的后向散射截面积 $\sigma_g = \sigma^0(\theta_{\text{graz}}, \text{TerrainType}) \cdot \Delta A_g$，其中擦地角 $\theta_{\text{graz}} = \arcsin(H / R)$，分辨率单元面积 $\Delta A_g = \frac{c}{2B\cos\theta_{\text{graz}}} \cdot R \theta_{3\text{dB}}$。区分水面镜面反射、平原植被漫反射、山脉地形阴影遮挡（Shadowing）与城市建筑/机场强角反射器散射点。
2. **SAR 合成孔径高分辨成像模型**：
   - 距离向理论分辨率：
     $$\rho_r = \frac{c}{2 B}$$
     当发射超宽带线性调频信号带宽 $B = 300\text{ MHz}$ 时，$\rho_r = 0.5\text{ m}$。
   - 方位向理论分辨率（聚束式 SAR 在合成孔径积分时间 $T_{\text{sar}}$ 内）：
     $$\rho_a = \frac{\lambda R_0}{2 V_a T_{\text{sar}} \sin\theta_{\text{sq}}}$$
     其中 $V_a$ 为载机平台速度，$\theta_{\text{sq}}$ 为斜视角。后端执行**距离-多普勒（Range-Doppler, RD）成像流水线**：距离向 FFT 匹配滤波 $\rightarrow$ 距离弯曲插值校正（RCMC）$\rightarrow$ 方位向去斜（Dechirp）与加窗 IFFT，向前端实时推送高分辨率二维灰度雷达图像。
3. **GMTI / GMTT 主瓣地杂波抑制与对地跟踪**：
   由于载机高速运动（$V_a \approx 250\text{ m/s}$），地面静止杂波的多普勒频率随锥角 $\psi$ 展宽为 $f_d(\psi) = \frac{2 V_a}{\lambda} \cos\psi$，地面慢速移动目标（如 $v_t = 15\text{ m/s}$ 的装甲车队）完全落入主瓣杂波谱内。
   后端将圆形阵面沿横轴切分为左子阵 $L$ 与右子阵 $R$（相位中心间距 $d_{\text{sub}}$），当时间延迟满足 DPCA 条件 $\Delta t = \frac{d_{\text{sub}}}{2 V_a}$ 时，双通道杂波相消输出为：
   $$y_{\text{DPCA}}(t) = s_L(t) - s_R(t - \Delta t)$$
   静止地面杂波被大幅抵消（抑制比 $> 38\text{ dB}$），而具有径向速度 $v_r$ 的地面移动目标保留明显的剩余相位差 $\Delta\phi = \frac{4\pi v_r \Delta t}{\lambda}$，随后转入 **GMTT** 敏捷单脉冲波束连续跟踪。

---

## 5. 不同距离的电磁传播衰减与杂波/目标回波模型

### 5.1 全链路相参脉冲多普勒雷达方程
为精确模拟不同距离、不同扫描角、不同工作模式及不同气象条件下的回波衰减，后端严格计算单脉冲与相参脉冲串积累后的接收信噪比 $\text{SNR}(R)$：
$$\text{SNR}(R) = \frac{P_t \cdot G_t(\theta_0, \phi_0) \cdot G_r(\theta_0, \phi_0) \cdot \lambda^2 \cdot \sigma \cdot D_{\text{pc}} \cdot N_{\text{coh}} \cdot \alpha_{\text{STC}}(R)}{(4\pi)^3 \cdot R^4 \cdot k_B T_0 B_n F_n \cdot L_{\text{sys}} \cdot L_{\text{atm}}(R) \cdot L_{\text{rain}}(R)}$$
各项物理意义如下：
- $P_t = N_{\text{active}} P_{\text{TR}}$：当前激活 TR 组件合成峰值发射功率；
- $G_t(\theta_0, \phi_0), G_r(\theta_0, \phi_0)$：包含孔径锥削损耗与扫描投影损耗的发射/接收天线增益：
  $$G(\theta_0, \phi_0) = G_{\max} \cdot \eta_{\text{taper}} \cdot (\cos\theta_{\text{scan}})^{1.2}$$
- $D_{\text{pc}} = B \cdot \tau_p$：脉冲压缩时间带宽积增益（Matched Filter Processing Gain）；
- $N_{\text{coh}} = f_{\text{PRF}} \cdot T_{\text{dwell}}$：单个驻留 CPI 内 FFT 相参积累脉冲数；
- $k_B = 1.38 \times 10^{-23}\text{ J/K}$：玻尔兹曼常数，$T_0 = 290\text{ K}$ 为标准热力学温度，$B_n \approx 1/\tau_{\text{comp}}$ 为匹配滤波器等效噪声带宽。

### 5.2 四种距离相关衰减机制的精确数学模型

#### （1）自由空间双程球面扩散衰减（$R^4$ 定律）
电磁波从雷达辐射至距离 $R$ 处的目标并散射回雷达接收孔径，经历双程球面波扩散，自由空间传播损耗因子 $L_{\text{fs}}(R)$ 为：
$$L_{\text{fs}}(R) = \frac{(4\pi)^3 R^4}{\lambda^2}$$
转化为分贝（$\text{dB}$）表示：距离 $R$ 每增加一倍（例如从 $40\text{ km}$ 增加到 $80\text{ km}$），目标回波功率严格衰减 $40\log_{10}(2) \approx 12.04\text{ dB}$。

#### （2）ITU-R P.676 大气气体双程吸收衰减 $L_{\text{atm}}(R)$
在标准大气压与温湿度剖面下，X 波段（$10\text{ GHz}$）电磁波受大气中氧气分子（$\text{O}_2$）磁偶极矩谐振与水蒸气分子（$\text{H}_2\text{O}$）电偶极矩谐振吸收。考虑到机载火控雷达载机高度 $h_a$ 与目标高度 $h_t$ 随斜距变化，比衰减率 $\gamma_{\text{gas}}(h)$ 随海拔高度 $h$（单位：$\text{km}$）呈指数递减：
$$\gamma_{\text{gas}}(h) = \gamma_{\text{O}_2, 0} \exp\left(-\frac{h}{h_{\text{O}_2}}\right) + \gamma_{\text{H}_2\text{O}, 0} \exp\left(-\frac{h}{h_{\text{H}_2\text{O}}}\right) \quad (\text{dB/km})$$
在 $f_0 = 10\text{ GHz}$ 海平面基准下，$\gamma_{\text{O}_2, 0} \approx 0.0072\text{ dB/km}$，$\gamma_{\text{H}_2\text{O}, 0} \approx 0.0085\text{ dB/km}$，特征标高 $h_{\text{O}_2} \approx 6.0\text{ km}, h_{\text{H}_2\text{O}} \approx 2.1\text{ km}$。沿射线路径积分得到双程大气气体总衰减（$\text{dB}$）：
$$L_{\text{atm, dB}}(R) = 2 \int_{0}^{R} \gamma_{\text{gas}}\left(h(r)\right) \text{d}r$$

#### （3）ITU-R P.838 降雨与云雾双程气象衰减 $L_{\text{rain}}(R)$
X 波段波长（$\lambda = 3\text{ cm}$）与雨滴尺寸处于米氏散射（Mie Scattering）过渡区，对中到大雨极为敏感。根据 **ITU-R P.838-3** 标准，降雨比衰减率 $\gamma_{\text{rain}}$（$\text{dB/km}$）与降雨率 $R_{\text{rain}}$（$\text{mm/h}$）满足幂律关系：
$$\gamma_{\text{rain}} = k_r \cdot R_{\text{rain}}^{\alpha_r} \quad (\text{dB/km})$$
对于 $10\text{ GHz}$ 水平/垂直极化，$k_r = 0.0101, \alpha_r = 1.276$。同时计入浓雾/云层液态水含量 $M_w$（$\text{g/m}^3$）引起的比衰减 $\gamma_{\text{fog}} = K_l M_w \approx 0.098 M_w\text{ (dB/km)}$，则穿越有效雨区路径长度 $R_{\text{eff}}$ 的双程气象衰减为：
$$L_{\text{rain, dB}}(R) = 2 \left( k_r R_{\text{rain}}^{\alpha_r} + \gamma_{\text{fog}} \right) R_{\text{eff}}$$
- **仿真对比验证**：
  - 晴朗天气（$R_{\text{rain}} = 0\text{ mm/h}$）：$100\text{ km}$ 处双程大气衰减仅约 $1.8\text{ dB}$；
  - 中雨天气（$R_{\text{rain}} = 10\text{ mm/h}$）：比衰减率 $\gamma_{\text{rain}} \approx 0.19\text{ dB/km}$，穿越 $40\text{ km}$ 雨区双程额外衰减达 $15.2\text{ dB}$；
  - 强雷暴天气（$R_{\text{rain}} = 40\text{ mm/h}$）：比衰减率 $\gamma_{\text{rain}} \approx 1.12\text{ dB/km}$，穿越 $30\text{ km}$ 雨区双程额外衰减高达 $67.2\text{ dB}$，直接导致远距探测距离大幅缩水。

#### （4）近程灵敏度时间控制（STC）衰减曲线 $\alpha_{\text{STC}}(R)$
在近距离（$R < R_{\text{STC}} \approx 15\text{ km}$）处，由于 $R^{-4}$ 效应，近程地物杂波、鸟群及近距目标的回波功率极强，极易造成接收机低噪放（LNA）与 ADC 饱和失真。因此后端在 LPRF/MPRF 及 ACM 模式下引入数控 STC 衰减律：
$$\alpha_{\text{STC}}(R) = \begin{cases} \left( \dfrac{\max(R, R_{\min})}{R_{\text{STC}}} \right)^{\nu}, & R < R_{\text{STC}} \\ 1.0, & R \ge R_{\text{STC}} \end{cases} \quad (\nu \in [2.5, 3.5])$$

---

## 6. 波形库设计（正常波形与抗干扰波形）与 ECCM 抗干扰体系

### 6.1 正常探测与跟踪波形库数学模型

#### 1. 线性调频脉冲压缩波形（LFM Chirp）
用于 RWS、TWS、GM、SAR 等大多数模式。设脉冲宽度为 $\tau_p$，调频带宽为 $B$，调频斜率 $\mu = B / \tau_p$，基带发射复包络为：
$$s_{\text{LFM}}(t) = \text{rect}\left(\frac{t}{\tau_p}\right) \exp\left(j \pi \mu t^2\right), \quad |t| \le \frac{\tau_p}{2}$$
匹配滤波器冲激响应为 $h(t) = w_{\text{win}}(t) s_{\text{LFM}}^*(-t)$。脉冲压缩后的输出包络为加权 $\text{sinc}$ 函数：
$$y_{\text{pc}}(t) \approx \sqrt{B \tau_p} \cdot \frac{\sin(\pi B t)}{\pi B t}$$
加海明窗（Hamming）或 Taylor 窗 $w_{\text{win}}(t)$ 后，距离旁瓣从 $-13.2\text{ dB}$ 压低至 $-40\text{ dB}$。

#### 2. 巴克码与多相编码波形（13-bit Barker / Frank / P4 Code）
用于 ACM 近距格斗、中近距目标识别及距离波门核验。以 $N_c = 13$ 位巴克码为例，将脉宽 $\tau_p$ 均分为 $13$ 个子码片 $\tau_c = \tau_p / 13$：
$$s_{\text{Barker}}(t) = \sum_{k=0}^{12} c_k \cdot \text{rect}\left( \frac{t - k\tau_c}{\tau_c} \right), \quad \mathbf{c}_{13} = [+1, +1, +1, +1, +1, -1, -1, +1, +1, -1, +1, -1, +1]$$
其自相关函数主瓣峰值与最高距离旁瓣之比恒定为 $20\log_{10}(13) = 22.28\text{ dB}$，且实现简单、零多普勒敏感度低。

#### 3. HPRF / MPRF / LPRF 多重脉冲重复频率与余数定理（CRT）解模糊
- **HPRF（高脉冲重复频率，$f_r = 120 \sim 200\text{ kHz}$）**：最大无模糊速度 $v_{\max} = \frac{\lambda f_r}{4} > 900\text{ m/s}$，在多普勒域完全无速度模糊，迎头目标落入无杂波自由区（Clutter-Free Region），但距离高度模糊（最大无模糊距离 $R_u = \frac{c}{2 f_r} \approx 0.75 \sim 1.25\text{ km}$），需采用线性调频连续测距（FMICW）或三重 HPRF 解距离模糊。
- **MPRF（中脉冲重复频率，$f_r = 10 \sim 30\text{ kHz}$）**：距离与速度均存在模糊。模拟器采用工业标准的 **$3\text{-of-}8$ 参差 PRF 体制**（在一个波束驻留内连续发射 $3$ 个互质的脉冲重复频率 $f_{r1}, f_{r2}, f_{r3}$，对应无模糊距离单元数互质 $M_1, M_2, M_3$）。利用**中国剩余定理（Chinese Remainder Theorem, CRT）**，由三个视在距离单元余数 $(r_1, r_2, r_3)$ 唯一还原真实目标距离 $R_{\text{true}}$ 与真实径向速度 $V_{\text{true}}$。

### 6.2 抗干扰波形库（ECCM Waveforms）数学模型

#### 1. 宽带脉间/脉组全频段随机频率捷变波形（Frequency Agility）
针对敌方窄带瞄准式干扰（Spot Jamming），雷达在 $M$ 个离散跳频点 $f_m = f_0 + c_m \Delta F$（覆盖 $8.5 \sim 10.5\text{ GHz}$ 宽带）之间按加密伪随机序列 $c_m$ 在脉间或子CPI间高速跳变：
$$s_{\text{Agile}}(t) = \sum_{m=0}^{M-1} u(t - m T_r) \exp\left( j 2\pi (f_0 + c_m \Delta F) t \right)$$
当跳频速率超过敌方电子支援侦察（ESM）引导干扰机的测频与锁频响应时间时，瞄准式干扰失效，迫使干扰机只能将功率平铺在 $2\text{ GHz}$ 宽带上，干扰功率谱密度下降 $10\log_{10}(B_{\text{agile}} / B_n) > 25\text{ dB}$。

#### 2. Costas 阵列跳频 + 脉内相位复合调制波形（Costas + LFM Hybrid）
针对最危险的 **DRFM（数字射频存储器）全相参复制假目标干扰**与 **RGPO（距离波门拖引）**，模拟器设计了 **Costas 序列频率跳变复合波形**。将一个发射脉冲 $\tau_p$ 划分为 $M$ 个等长子脉冲 $\tau_{\text{sub}} = \tau_p / M$，每个子脉冲分配频率 $f_m = \theta_m \Delta f$，其中排列 $\boldsymbol{\theta} = [\theta_1, \theta_2, \dots, \theta_M]$ 为满足差三角形所有元素互异的 Costas 置换序列（例如 $M=7$ 时 $\boldsymbol{\theta} = [4, 7, 3, 6, 5, 1, 2]$；$M=10$ 时 $\boldsymbol{\theta} = [2, 4, 8, 5, 10, 9, 7, 3, 6, 1]$）：
$$s_{\text{Costas}}(t) = \frac{1}{\sqrt{M \tau_{\text{sub}}}} \sum_{m=0}^{M-1} \text{rect}\left( \frac{t - m\tau_{\text{sub}}}{\tau_{\text{sub}}} \right) \exp\left( j 2\pi f_m (t - m\tau_{\text{sub}}) + j \pi \mu_{\text{sub}} (t - m\tau_{\text{sub}})^2 \right)$$
- **抗干扰物理机理**：其距离-多普勒模糊函数 $|\chi(\tau, f_d)|$ 呈理想的**“图钉型（Thumbtack）”**，距离与速度耦合副瓣极低（$\approx 1/M$）。若敌方 DRFM 干扰机对前一脉冲采样并延迟转发以制造假目标，而雷达下一脉冲实时变换 Costas 跳频排列 $\boldsymbol{\theta}^{(k)}$，则敌方转发的假目标信号在经过当前脉冲匹配滤波器时完全失配，假目标峰值被直接压低 $10\log_{10}(M) \sim 20\log_{10}(M)\text{ dB}$！

#### 3. 脉冲重复间隔随机抖动与双正交相参编码（PRI Jitter & Orthogonal Phase Tagging）
对脉冲发射时刻引入受控随机抖动 $T_{r, k} = T_{r, 0} + \delta t_k$（$\delta t_k \sim \mathcal{U}(-\Delta T, +\Delta T)$），并在脉间附加正交相位标签 $\phi_{\text{tag}, k}$。接收机仅对匹配的当前帧相位标签进行解调，使跨周期转发的欺骗脉冲在多普勒 FFT 时散焦为白噪声基底。

### 6.3 电子对抗（ECM）威胁与四维抗干扰（ECCM）对应矩阵
模拟器支持在前端动态开启以下电子干扰，并实时对比启用 ECCM 前后的信干噪比（$\text{SINR}$）与目标检测结果：

| 敌方电子干扰类型 (ECM Threat) | 对常规雷达的破坏表现 | 模拟器后端实现的 ECCM 抗干扰对策 | 物理抑制效果指标 |
| :--- | :--- | :--- | :--- |
| **宽带阻塞式噪声压制**<br>(Barrage Noise Jamming) | 全频段抬高接收机热噪声基底，CFAR 门限大幅升高，远距弱目标被淹没 | **空域 MVDR 自适应零陷 (ADBF)** + **超低副瓣圆 Taylor 加权** + **相参长 CPI 积累** | 旁瓣干扰抑制 $> 45\text{ dB}$，恢复正常噪声基底 |
| **窄带瞄准式强噪声干扰**<br>(Spot Noise Jamming) | 在当前工作频点注入高功率密度噪声，导致当前频点完全致盲 | **宽带脉间伪随机频率捷变**（跳出干扰带宽）+ **频域陷波滤波** | 瞄准干扰失锁，等效信干比提升 $> 22\text{ dB}$ |
| **高占空比旁瓣脉冲干扰**<br>(Sidelobe Pulsed Jamming) | 假点迹充满整个 B-Scope 距离维，造成航迹关联器过载崩溃 | **双通道旁瓣消隐 (SLB, Guard Channel)** + **旁瓣相干对消 (SLC)** | 比较和通道 $\Sigma$ 与全向保护通道 $G$ 幅度，剔除 $99\%$ 旁瓣脉冲 |
| **DRFM 密集多假目标欺骗**<br>(DRFM Multiple False Targets) | 在真实目标前后产生多个高逼真度相参假目标，诱导武器脱靶 | **脉间动态 Costas 跳频编码** + **脉内正交调相匹配鉴别** + **OS-CFAR 有序统计恒虚警** | 跨脉冲转发假目标在匹配滤波器失配散焦，假目标抑制 $> 20\text{ dB}$ |
| **距离波门拖引 / 速度波门拖引**<br>(RGPO / VGPO Deception) | 先叠加在真目标回波上夺取 AGC 与波门控制权，随后逐渐拉大延迟将跟踪波门拖走并断开 | **前沿跟踪（Leading-Edge Tracking）** + **双波门/记忆外推波门（Memory Gate）** + **运动学机动合理性残差检验** | 识别真假波门分裂点，维持原距离/多普勒波门锁定在真实目标机身回波上 |

---

## 7. 前后端数据接口规范与前端全面显控设计

### 7.1 前后端实时通信协议定义
系统采用 **JSON 控制信道 + 二进制流（ArrayBuffer）遥测信道** 的混合协议，兼顾可读性与高吞吐率。

#### （1）上行控制指令 JSON 结构（Front-End $\rightarrow$ Back-End）
```json
{
  "cmdType": "UPDATE_RADAR_CONFIG",
  "mode": "TAS",
  "acmSubMode": "HUD_SCAN",
  "scanVolume": {
    "azimuthCenterDeg": 0.0,
    "azimuthLimitDeg": 60.0,
    "elevationCenterDeg": 0.0,
    "elevationBars": 4
  },
  "apertureConfig": {
    "diameterMeters": 0.70,
    "gridTopology": "TRIANGULAR",
    "weightingType": "TAYLOR_35DB",
    "randomFailureRate": 0.01
  },
  "waveformConfig": {
    "waveformType": "COSTAS_LFM_HYBRID",
    "prfRegime": "MPRF_3OF8",
    "bandwidthMHz": 50.0,
    "pulseWidthUs": 20.0
  },
  "eccmConfig": {
    "mvdrNullingEnabled": true,
    "slbBlankingEnabled": true,
    "frequencyAgilityEnabled": true,
    "rgpoLeadingEdgeGuard": true
  },
  "environmentConfig": {
    "rainRateMmPerHr": 4.0,
    "fogDensityGm3": 0.05,
    "activeJammers": [
      { "id": "JAM-01", "azDeg": -22.0, "elDeg": 1.5, "rangeKm": 95.0, "type": "DRFM_RGPO", "erpDbw": 75.0 }
    ]
  },
  "designatedTargetIds": ["TGT-101", "TGT-102"]
}
```

#### （2）下行高速遥测帧结构（Back-End $\rightarrow$ Front-End, $30 \sim 60\text{ Hz}$）
```json
{
  "frameId": 10482,
  "simTimeSec": 342.15,
  "activeMode": "TAS",
  "beamState": {
    "dwellType": "TAS_TRACK_UPDATE",
    "steerAzDeg": 18.4,
    "steerElDeg": 3.2,
    "currentBar": 2,
    "peakGainDbi": 36.8,
    "hpbwAzDeg": 2.85,
    "sidelobeLevelDb": -35.4,
    "nullAnglesDeg": [-22.0]
  },
  "apertureSummary": {
    "totalElements": 1574,
    "healthyElements": 1558,
    "failedElements": 16
  },
  "rawBricks": [
    { "rangeKm": 78.4, "azDeg": 18.3, "elDeg": 3.1, "dopplerVelMps": -412.0, "snrDb": 22.6, "isFalseAlarm": false }
  ],
  "tracks": [
    {
      "trackId": "TGT-101",
      "classification": "HOSTILE_FIGHTER",
      "rangeKm": 78.38,
      "azDeg": 18.32,
      "elDeg": 3.12,
      "altitudeMeters": 8500,
      "closingVelocityMps": 412.5,
      "headingDeg": 195.0,
      "rcsSqm": 3.2,
      "trackQuality": 0.98,
      "priorityRank": 1,
      "weaponEnvelope": { "rMaxKm": 92.0, "rNeKm": 54.0, "rMinKm": 3.5 }
    }
  ],
  "binaryBuffersMeta": {
    "elementWeightsOffset": 0,
    "beamPatternCutOffset": 6296,
    "rangeProfileOffset": 7740,
    "rdMatrixOffset": 9788
  }
}
```

### 7.2 前端全面显控布局设计规范
前端工作台采用航空电子暗色高对比度座舱主题（Avionics Dark Theme），分为五大同步刷新的功能区：
1. **顶部态势与模式总览栏**：实时显示工作频段、当前激活工作模式、发射峰值功率、脉冲重复频率（PRF）、大气/降雨率参数、当前帧耗时与抗干扰状态灯。
2. **左侧主战术雷达显示器（MFD Primary Radar Display）**：
   - 四周环绕 **20 个多功能软键（OSB）**，可直接点击切换 `RWS`、`TWS`、`TAS`、`MTT`、`DTT`、`STT`、`ACM`、`GM`、`SAR`、`GMTI/GMTT` 模式，以及切换量程（$10 / 20 / 40 / 80 / 160\text{ NM}$）、方位角（$\pm 10^\circ / \pm 30^\circ / \pm 60^\circ$）与俯仰行数（$1 / 2 / 4 / 8\text{ Bars}$）。
   - 主画布支持 **B-Scope 视图**（对空）、**HUD 格斗扫描视图**（ACM）、**扇形 PPI 地形/动目标视图**（GM / GMTI / GMTT）与 **SAR 高分辨灰度成像视图** 的无缝切换。
   - 实时渲染瞬时电子波束脚印（含 TAS/MTT 模式下的微秒级敏捷波束跳变残影）、目标航迹符号（符合北约/美军标准 NTDS/MIL-STD-2525 或标准战机 MFD 符号：带速度矢量线的矩形/三角形锁定框、威胁排序编号）、动态武器发射包线（Dynamic Launch Zone, DLZ）以及干扰源方位高亮频闪扇区。
3. **中上区：千级阵元圆形相控阵阵面实时监控器**：
   - 精确绘制 $>1000$ 个 TR 组件在圆形口径上的正三角形网格分布、$Q_1 \sim Q_4$ 单脉冲子阵分割线，支持一键切换查看**幅度加权热力分布 $I_n$**、**6-bit 移相器相位梯度条纹 $\varphi_n$** 与 **TR 通道战损/失效分布**。
4. **中下区：多模式合成波束方向图与单脉冲鉴角分析仪**：
   - 实时叠加显示和波束 $\Sigma(\theta)$、方位差波束 $\Delta_{\text{Az}}(\theta)$ 以及 MVDR 自适应抗干扰零陷波束切面，动态标出 $3\text{ dB}$ 主瓣宽度、第一副瓣电平（$\text{SLL}$）与干扰方向零深。
5. **右侧区：波形发生器、距离衰减曲线与脉冲压缩/R-D 频谱仪**：
   - 上半部展示当前发射波形（LFM / Barker / Costas / 捷变）的时域包络与时频调制脊线；
   - 中间部展示经过双程 $R^4$ 损耗、ITU-R 大气/降雨衰减、脉冲压缩及 ECCM 抗干扰处理后的 **A-Scope 距离像与动态 CFAR 门限**；
   - 下半部展示 **二维距离-多普勒（Range-Doppler）热力图**（清晰呈现主瓣地杂波带、高度线杂波、真实目标峰值与干扰噪声分布）。

---

## 8. 工程实施与验证清单

为确保模拟器从数学模型到代码实现 $100\%$ 还原设计要求，实施阶段按以下五项核心验收标准执行：
1. **千级阵元圆阵面验证**：圆阵面 TR 组件数量 $\ge 1000$ 个，正三角形均匀栅格排布，支持单独或批量注入故障阵元并实时反映在远场方向图副瓣抬升上。
2. **十种工作模式全覆盖验证**：`RWS`、`TWS`、`TAS`、`MTT`、`DTT`、`STT`、`ACM`、`GM`、`SAR`、`GMTI/GMTT` 十种模式均可独立切换运行，每种模式拥有专属的天线扫描规律、阵面加权方式与前端视图。
3. **多模式波束赋形差异验证**：切换模式时，可清晰观察到均匀铅笔波束、圆 Taylor 低副瓣波束、圆 Bayliss 差波束、唯相展宽/余割平方波束及 MVDR 自适应零陷波束的实时切换。
4. **距离与气象衰减物理验证**：目标由远及近或改变降雨率（$0 \sim 50\text{ mm/h}$）时，回波功率严格遵循 $R^{-4}$ 与 ITU-R P.676/P.838 指数衰减规律，并与 CFAR 检测门限联动决定目标截获或脱锁。
5. **抗干扰波形与 ECCM 闭环验证**：注入窄带瞄准、宽带阻塞、DRFM 多假目标或 RGPO 波门拖引干扰后，未开启抗干扰时出现目标淹没或波门被假目标拖走；切换至 Costas/捷变抗干扰波形并开启 MVDR+SLB+前沿波门保护后，假目标与干扰噪声被抑制，恢复对真实目标的稳定跟踪。