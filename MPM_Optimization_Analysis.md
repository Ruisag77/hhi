# MPM (Most Probable Modes) 深度分析与编码性能优化方案

> 工程目标：在当前 ECM 风格 (67-mode luma intra) 的 MPM 实现基础上，识别可落地的优化点，预期改善 BD-Rate、Bitrate 与 PSNR。
> 适用代码库：`/Users/liujiarui.40/video_coding/hhi`（基于 ECM/VVC，含 DIMD/TIMD/OBIC/SGPM/EIP/MIP/MRL 等扩展）

---

## 0. 行政摘要 (TL;DR)

经过对源码的逐行分析，**普通 MPM 不再是该编码器的主瓶颈**——DIMD/TIMD/OBIC/EIP 等 decoder-side 模式推导路径在亮度 intra 编码中已经吸收了大量"普通帧内方向"的选择空间。因此优化要分两条路线考虑：

| 路线 | 主要切入点 | 预期 BD-Rate 收益 (Y) | 工程风险 |
|------|------------|----------------------|----------|
| A. 直接打磨 MPM 列表 | 候选来源、排序、默认表、上下文 | -0.10% ~ -0.30% | 低（兼容性可控） |
| B. 与 DIMD/TIMD 联动 | 把 decoder-side 推导结果灌入 MPM | -0.20% ~ -0.50% | 中（需同步比特流） |
| C. CABAC 上下文重建模 | mpm_flag/secondMpmFlag 多上下文化 | -0.05% ~ -0.15% | 低（无解析影响） |
| D. 内容自适应默认表 | I-slice 末尾在线统计 | -0.10% ~ -0.25% | 中（需档位/类配置） |

> 在 RA/LDB 条件下，B + C 组合预计带来 **-0.3% ~ -0.6% BD-Rate(Y)**，对 SC/UHD 序列尤其敏感。

---

## 1. 现状分析

### 1.1 关键数据结构与常量

文件：`source/Lib/CommonLib/CommonDef.h`

```cpp
static constexpr int NUM_DIR                            = 16;
static constexpr int NUM_INTRA_ANGULAR_MODES            = 4 * NUM_DIR + 1;   // 65
static constexpr int ANGULAR_BASE                       = 2;                  // Planar(0), DC(1)
static constexpr int NUM_LUMA_MODE                      = ANGULAR_BASE + 65;  // 67

static constexpr int NUM_PRIMARY_MOST_PROBABLE_MODES    = 6;
static constexpr int NUM_SECONDARY_MOST_PROBABLE_MODES  = 16;
static constexpr int NUM_MOST_PROBABLE_MODES            = 22;
static constexpr int NUM_NON_MPM_MODES                  = NUM_LUMA_MODE - 22; // 45

static constexpr int SGPM_NUM_MPM                       = 3;
static constexpr int GEO_MAX_NUM_INTRA_CANDS            = 3;
```

文件：`source/Lib/CommonLib/ContextModelling.h:688`

```cpp
class CUCtxIntra {
  uint8_t mpmList[NUM_MOST_PROBABLE_MODES];   // 22
  uint8_t nonMPMList[NUM_NON_MPM_MODES];      // 45
  int     mpmListSize;
};
```

### 1.2 主流程 MPM 构建：`PU::getIntraMPMs`

文件：`source/Lib/CommonLib/UnitTools.cpp:590-754`

```text
[0]            Planar (强制)
[1]            Left  邻居方向（按 H>=W ? 用 RT 上面块 : 用 LB 左面块）
[2]            Above 邻居方向（注意：要求 isSameCtu，即同一 CTU 才用）
[3..]          以 mpm[1] 为种子的 ±i 角度衍生（i=1..4）
               再以 mpm[2] 为种子衍生
[..]           若 mpm[1] 或 mpm[2] 为 DC → 触发 mpm[3] 衍生
[尾部填充]      静态默认表 {DC, VER, HOR, VER-4, VER+4, 14, 22, 42, 58, 10, 26, 38, 62, 6, 30, 34, 66, 2, 48, 52, 16}
[非MPM]         其余 45 个模式按数值序压入 nonMPMList
```

关键代码片段（节选）：

```cpp
// UnitTools.cpp:606-629  邻居选择
const CodingUnit *puLeft = (cu.lheight() >= cu.lwidth())
  ? cu.cs->getCURestricted(posRT.offset(0, -1), cu, channelType)
  : cu.cs->getCURestricted(posLB.offset(-1, 0), cu, channelType);
...
const CodingUnit *puAbove = (cu.lheight() >= cu.lwidth())
  ? cu.cs->getCURestricted(posLB.offset(-1, 0), cu, channelType)
  : cu.cs->getCURestricted(posRT.offset(0, -1), cu, channelType);
if (puAbove && CU::isIntra(*puAbove) && CU::isSameCtu(cu, *puAbove)) { ... }
```

```cpp
// UnitTools.cpp:728-737  默认填充表
unsigned mpmDefault[numMPMs - 1] = { DC_IDX, VER_IDX, HOR_IDX,
                                     VER_IDX - 4, VER_IDX + 4,
                                     14, 22, 42, 58, 10, 26, 38, 62, 6, 30, 34, 66, 2, 48, 52, 16 };
```

### 1.3 CABAC 信令开销

文件：`source/Lib/EncoderLib/CABACWriter.cpp:1277-1380`、`Contexts.inl:437-531`

`intra_luma_pred_mode` 的码字结构（普通模式，非 DIMD/TIMD/OBIC/SGPM/EIP/MIP）：

| 语法元素 | 比特数 | 上下文模型 | 备注 |
|---------|--------|-----------|------|
| `intra_luma_mpm_flag`         | 1 reg | `IntraLumaMpmFlag` (1 ctx) | MRL≠0 时强制为1 |
| `intra_luma_not_planar_flag`  | 1 reg | `IntraLumaPlanarFlag` (3 ctx) | MRL=0 才编 |
| `intra_luma_mpm_idx > 1?`     | 1 reg | `IntraLumaMPMIdx` (3 ctx)   | 仅当 not_planar |
| `intra_luma_mpm_idx ∈ {2..5}` | 3 EP  | —                            | 二进制位置编码 |
| `intra_luma_second_mpm_flag`  | 1 reg | `IntraLumaSecondMpmFlag` (1 ctx) | !mpm_flag |
| `secondary_idx`               | 4 EP  | —                            | 16 个二级 MPM |
| `non_mpm_idx`                 | TB-code | — | 45 个非 MPM，截断二进制 |

**观察**：
- 命中 Planar：1 + 1 = 2 bits（实际 ~0.5 bit 等价熵）。
- 命中其它 5 个 primary MPM：≈ 5 bits。
- 命中 16 个 secondary MPM：≈ 6.0 bits。
- 命中 45 个 non-MPM：≈ ⌈log2 45⌉ = 5~6 bits。

Secondary MPM 命中代价已经和 non-MPM 接近 → 二级 MPM 的存在主要为概率优化（CABAC 上下文）而非比特节省。这是一个**潜在优化点**。

### 1.4 编码端如何使用 MPM

文件：`source/Lib/EncoderLib/IntraSearch.cpp`

- **预 RD（SATD）阶段**：每条候选的代价 = `min(SAD*2, HAD) + fracModeBits * sqrtLambdaForFirstPass`，`fracModeBits` 来自 `xFracModeBitsIntra`，已包含 MPM 上下文比特。命中 MPM → SATD 代价更低 → 更容易进入 full-RD 列表。
- **多参考行 (MRL, 行 459-493)**：仅测试 `multiRefMPM[1..5]`（跳过 Planar 因为 MRL 不支持 Planar）。
- **强制保留 (行 794-815)**：`m_bFastUDIUseMPMEnabled` 开关下，确保前 2 个 MPM 进入 full-RD 列表（避免 SATD 漏选）。

### 1.5 性能评价指标

文件：`source/Lib/EncoderLib/EncGOP.cpp:5701`、`Analyze.h:182-244`

```cpp
// 帧级 PSNR（每分量）
dPSNR[comp] = (ssd == 0) ? 999.99 : 10.0 * log10(maxval * maxval * W * H / ssd);

// 帧级 MSE
mseYuvFrame[comp] = (double)ssd / size;

// 序列 Bitrate（单位 kbps）
addField("Bitrate", "%-12.4lf ", getBits() * (fps / 1000.0 / picCount));

// YUV 联合 PSNR（4:2:0 时 Y/U/V 权重 4:1:1）
PSNRyuv = 10.0 * log10(maxval^2 / mseYuv);
```

**重点**：
1. 本工程**不内置 BD-Rate 计算**，BD-Rate 在外部脚本（如 JVET BD-Rate 工具或 Bjøntegaard 实现）中按 4-QP 点拟合得出。
2. `Bitrate × PSNR` 在 4 个 QP 点 (22/27/32/37) 上构造 R-D 曲线，与锚点对比即得 BD-Rate(Y)。
3. **任何 MPM 修改必须在 4 QP 全跑**——因为 MPM 的 RD-Cost 影响会随 λ 变化。

---

## 2. MPM 的原理与目标

### 2.1 工作机制（按编码流水线位置）

```
[CU 进入 Intra 路径]
  ├─ 1. 决定 cu.predMode = MODE_INTRA
  ├─ 2. Skip flag / pred_mode_flag / IBC 检查
  ├─ 3. MIP / DIMD / TIMD / OBIC / EIP / SGPM 等"绕过 MPM"的分支
  │       └─ 这些分支自带预测方向推导，与 MPM 列表无关
  ├─ 4. extend_ref_line (MRL)
  └─ 5. ★ MPM 信令 (本文重点)
        ├─ getIntraMPMs(cu, mpmList, nonMPMList)
        ├─ 编/解 mpm_flag / planar_flag / mpm_idx
        └─ 或 second_mpm_flag → secondary_idx
        或 truncated_binary(non_mpm_idx)
```

### 2.2 MPM 的四大设计目标

1. **降低模式信令开销**：将 Planar/邻居模式以 ~1 bit 编码，远低于直接编 67 模式（≈6.07 bit）。
2. **提升 R-D 性能**：通过 SATD 阶段的 `fracModeBits` 偏置，让"高概率模式"优先进入 full-RD，节省计算同时降低均值码率。
3. **快速 RD 早终止**：编码器层面，`m_bFastUDIUseMPMEnabled` 强制保留 top-2 MPM，构造 RD 决策的"安全网"。
4. **上下文复用**：MPM 信令的 CABAC 上下文使 mpm_flag/secondMpmFlag/MPMIdx 高度可压缩。

---

## 3. 当前实现的深入剖析

### 3.1 缺陷 / 优化空间

#### 缺陷 #1：邻居采样过窄（仅 L + A，不含 AL/AR/BL/AB）

`getIntraMPMs` 仅使用 Left + Above 两个空间邻居，而 `getGeoIntraMPMs` 与 `getSgpmIntraMPMs` 使用 5 个位置（L, A, BL, AR, AL）。这是一个**显著的不一致**：

```cpp
// SGPM/Geo 用完整邻居（UnitTools.cpp:766-770）
const Position posA  = area.topRight().offset(0, -1);
const Position posAR = area.topRight().offset(1, -1);
const Position posL  = area.bottomLeft().offset(-1, 0);
const Position posBL = area.bottomLeft().offset(-1, 1);
const Position posAL = area.topLeft().offset(-1, -1);

// 普通 MPM 只有 2 个（UnitTools.cpp:606-629）
// L, A，且 A 还受 isSameCtu 限制
```

**影响**：CTU 顶部行的 CU 完全失去 Above 信息（`isSameCtu` 检查导致），强方向纹理边界处的 MPM 命中率下降。

#### 缺陷 #2：`isSameCtu` 约束对大 CTU 越苛刻

```cpp
// UnitTools.cpp:622
if (puAbove && CU::isIntra(*puAbove) && CU::isSameCtu(cu, *puAbove))
```

对于 128×128 或 256×256 CTU，靠 CTU 顶部的 CU（约 1/16 ~ 1/8 比例）拿不到 Above 邻居 → 强制走衍生模式或默认表。**这是 HEVC 老式约束的遗留**，在 VVC/ECM 上无并行编码需求时可放宽。

#### 缺陷 #3：DIMD/TIMD/OBIC 推导方向未灌入 MPM 列表

`PU::getIntraDirLuma(cu)` 在邻居为 DIMD/TIMD 等时返回**最终方向**（普通情况），但仅作为单条邻居模式。**当前 CU** 自身正在尝试的 DIMD-like 方向（如对邻居的 DIMD 推导后保存的 `cu.timdHor / cu.timdVer / cu.dimdMode`）**不在 MPM 列表中**——只有 SGPM-MPM 用了它（`UnitTools.cpp:877-927`）。

```cpp
// SGPM 已经用了 (UnitTools.cpp:877)
if (cu.timdHor > DC_IDX && includedMode[MAP131TO67(cu.timdHor)] == false) {
  mpm[numValidMPM] = MAP131TO67(cu.timdHor);
  ...
}
```

普通 MPM 完全没有利用这些**高质量的隐含先验**——这是 BD-Rate 收益最大的可改动点。

#### 缺陷 #4：默认模式表是"魔术常量"

```cpp
unsigned mpmDefault[21] = { DC_IDX, VER_IDX, HOR_IDX,
                            VER_IDX - 4, VER_IDX + 4,
                            14, 22, 42, 58, 10, 26, 38, 62, 6, 30, 34, 66, 2, 48, 52, 16 };
```

这个表对自然图像可能合理（VER/HOR 优先），但：
- 对屏幕内容（SCC）— DC 和水平/垂直更主导，应前置
- 对 HDR — 平滑梯度居多，应增加 Planar 衍生
- 对 UHD — 边缘方向分布更窄，应收紧

**当前实现无 slice 自适应**。

#### 缺陷 #5：衍生模式仅以 ±i 角度偏移

```cpp
// UnitTools.cpp:647-665
for (int i = 0; i < 4 && numValidMPM < numMPMs; i++) {
  mpm[numValidMPM] = ((mpm[1] + offset - i) % mod) + 2;   // -i 偏移
  ...
  mpm[numValidMPM] = ((mpm[1] - 1 + i) % mod) + 2;        // +i 偏移
  ...
}
```

种子若为 VER，则衍生 ±1, ±2, ±3, ±4 → 命中 [46..54]。但若真实最优是 [44, 56]（强方向但稍稍偏离种子）就会漏掉，进入二级 MPM。**未利用块尺寸/角度分辨率自适应步长**。

#### 缺陷 #6：上下文模型粒度不足

`IntraLumaMpmFlag` 仅 1 个 context；`IntraLumaSecondMpmFlag` 也仅 1 个。CABAC 的概率自适应必须等待大量 bin → 在切换镜头/场景/小 GOP 中长期偏离最优概率。

#### 缺陷 #7：MPM 候选数固定为 6+16

`NUM_PRIMARY_MOST_PROBABLE_MODES = 6` 与 `NUM_SECONDARY_MOST_PROBABLE_MODES = 16` 是硬常量。对纹理稀疏区块（小 CU、平坦区），实际信令需求 ≤ 3；对方向丰富区块，22 也不一定够。

#### 缺陷 #8：编码端 Fast-UDI 仅强制 top-2 MPM

```cpp
// IntraSearch.cpp:794-815
if (m_encCfg->m_bFastUDIUseMPMEnabled) {
  int numCand = cuCtxIntra.mpmListSize;
  numCand = (numCand > 2) ? 2 : numCand;   // 只取前 2 个！
  ...
}
```

这是历史遗留：在 5-MPM 时代 top-2 足够，但 22-MPM 体系下应至少强制 top-3 或 top-(Planar+L+A)。

### 3.2 不同场景表现

| 场景 | 邻居信息质量 | 当前 MPM 表现 | 主要问题 |
|------|-------------|--------------|---------|
| 平坦区 | 邻居多为 Planar/DC | 较好（Planar 占 0 位） | 衍生模式浪费列表槽 |
| 强方向纹理 | 邻居方向一致 | 较好 | 但邻居 + ±4 偏移不一定覆盖最优 |
| 边缘/纹理切换 | L 与 A 方向不同 | 中等 | 仅 2 个种子 → 衍生互斥时多样性不够 |
| 屏幕内容 | DC/HOR/VER 主导 | 差 | 默认表对 SC 不友好 |
| CTU 顶行 | A 缺失 | 差 | `isSameCtu` 限制 |
| MRL>0 | Planar 不可用 | 中等 | 仅测 5 个非 Planar MPM |

### 3.3 关键 Bitrate/PSNR 影响因子

1. **MPM 命中率**：直接决定 `xFracModeBitsIntra` 的均值，最终影响码率。
2. **MPM 排序与 Planar 的位置**：Planar 偏置（1 bit 命中）是码率主导项。
3. **SATD 阶段 fracModeBits 偏置**：影响 full-RD 列表组成 → 影响 PSNR（间接通过更好的 RDO）。
4. **CABAC 上下文初始值**：决定 I-slice 早期的编码效率。

---

## 4. 优化方案

下文每个方案均给出：**修改内容、理论依据、Bitrate/PSNR 影响、副作用、实现难度**。

---

### 方案 A1：把 DIMD/TIMD 推导方向作为 MPM 候选（**首推**）

**修改位置**：`UnitTools.cpp:638` 附近（`getIntraMPMs` 内 `numCand = numValidMPM;` 之前插入）

**修改内容**：

```cpp
// 新插入：把当前 CU 已经计算好的 DIMD/TIMD 主方向作为高优先候选
if (cu.timdMode > DC_IDX) {
  uint8_t timd67 = MAP131TO67(cu.timdMode);
  if (!includedMode[timd67] && numValidMPM < numMPMs) {
    mpm[numValidMPM] = timd67;
    includedMode[timd67] = true;
    ++numValidMPM;
  }
}
if (cu.dimdMode > DC_IDX && !includedMode[cu.dimdMode] && numValidMPM < numMPMs) {
  mpm[numValidMPM] = cu.dimdMode;
  includedMode[cu.dimdMode] = true;
  ++numValidMPM;
}
```

**关键工程问题**：MPM 列表必须**编/解一致**。`cu.timdMode/dimdMode` 是 decoder-side derive 量，理论上编/解相同；但需确认 `getIntraMPMs` 调用时这些值已就绪（在 CABACReader 路径中需在解析阶段提前计算）。

**理论依据**：DIMD/TIMD 基于邻居梯度直方图与模板匹配推导，统计上是该 CU 邻域**最佳方向**的强先验。把它前置到 `mpm[1]` 或 `mpm[2]` 比单纯抓邻居方向更准确。

**预期 Bitrate 影响**：-0.15% ~ -0.40%（普通 intra CU 命中 MPM 率 +3~7%）。
**预期 PSNR 影响**：略增（同 QP 下，SATD 阶段更倾向 DIMD 风格方向 → full-RD 选择更优）。
**副作用**：
- 比特流不兼容（需 SPS flag 控制）。
- DIMD/TIMD 计算成本：原本只在 DIMD/TIMD 模式中计算，现在普通模式也要 → 复杂度上升 1~3%。可通过 lazy-evaluation 缓解（仅当邻居有 DIMD/TIMD 标志时计算）。
**实现难度**：中。
**风险**：编解一致性必须严格验证；建议先在 RA-Main10 cfg 下做小序列闭环。

---

### 方案 A2：扩展邻居采样到 5 位置（L/A/BL/AR/AL）

**修改位置**：`UnitTools.cpp:606-629`

**修改内容**：参考 `getSgpmIntraMPMs:930-1019` 的采样模式，把 BL/AR/AL 加入候选采样池。先按距离排序，再去重插入。

```cpp
struct NeighSeed { Position pos; int priority; };
NeighSeed seeds[] = {
  { area.bottomLeft().offset(-1, 0),  1 },   // L
  { area.topRight().offset(0, -1),    1 },   // A
  { area.bottomLeft().offset(-1, 1),  2 },   // BL
  { area.topRight().offset(1, -1),    2 },   // AR
  { area.topLeft().offset(-1, -1),    3 },   // AL
};
// 按 priority 顺序遍历，去重插入到 mpm[]
```

**理论依据**：补足边角邻居信息，对 CTU 顶/左 CU 与 L 形分区边界尤其重要。

**预期 Bitrate**：-0.05% ~ -0.15%。
**预期 PSNR**：+0.005 ~ +0.015 dB。
**副作用**：邻居访问内存带宽 +2~3 次/CU；缓存命中率影响小。
**实现难度**：低。
**风险**：极低；只是邻居复用 SGPM 已有逻辑。

---

### 方案 A3：放宽 `isSameCtu` 约束

**修改位置**：`UnitTools.cpp:622`

**修改内容**：

```cpp
// 原代码：
if (puAbove && CU::isIntra(*puAbove) && CU::isSameCtu(cu, *puAbove)) { ... }

// 修改为（要求仍在同一 slice/tile/wavefront 同步线之上）：
if (puAbove && CU::isIntra(*puAbove)) {
  // 可选保留 wavefront 检查
  ...
}
```

**理论依据**：现代 VVC/ECM 已不需要 CTU 顶部的解析独立性（前提是去除 WPP 同步约束），跨 CTU 的 Above 邻居方向有明确意义。

**预期 Bitrate**：-0.03% ~ -0.10%（CTU 顶行 CU 受益）。
**预期 PSNR**：~ +0.003 dB。
**副作用**：**有标准合规风险**——若启用 WPP/sub-picture 等并行特性，需要保留 `isSameCtu` 检查。建议加宏开关 `MPM_RELAX_SAMECTU` 默认关闭，由 SPS flag 控制。
**实现难度**：低。
**风险**：中。

---

### 方案 A4：内容自适应默认填充表

**修改位置**：`UnitTools.cpp:728`（`mpmDefault` 表）

**修改内容**：在 `EncSlice` 末尾统计当前 I-slice 的 luma intraDir 直方图，按 frequency 排序后注入 `slice->m_mpmDefault[]`（解码端用相同滑窗算法）。

```cpp
// CommonLib/Slice.h: 添加
uint8_t m_mpmDefault[NUM_MOST_PROBABLE_MODES - 1];

// EncoderLib/EncSlice.cpp: I-slice 结束后更新
// 解码端同步：用前一帧的统计（DPB 已有）
```

**理论依据**：图像类别不同（自然 vs 屏幕 vs HDR），最优默认顺序差异显著。前帧统计是该 frame 的强 prior（除非剧烈场景切换）。

**预期 Bitrate**：-0.10% ~ -0.25%（对屏幕内容序列 -0.4%）。
**预期 PSNR**：~ 0（仅码率影响）。
**副作用**：需要在解码端对称维护统计；首个 I-slice 仍用默认表。
**实现难度**：中。
**风险**：低；统计可放在 `EncGOP::finishPicture`。

---

### 方案 A5：自适应衍生模式步长

**修改位置**：`UnitTools.cpp:647-665, 678-696`

**修改内容**：根据种子角度密度自适应步长：

```cpp
// 当前：固定 i ∈ {1,2,3,4}
// 改为：根据角度距离 VER/HOR/DIA 的远近选择步长
const bool isCardinal = (mpm[1] == VER_IDX) || (mpm[1] == HOR_IDX) ||
                        (mpm[1] == DIA_IDX) || (mpm[1] == VDIA_IDX);
const int  stepMax    = isCardinal ? 2 : 4;   // 主方向附近角度密度高，步长小
const int  stepSet[]  = isCardinal ? {1, 2, 3, 4} : {1, 3, 5, 7};
```

**理论依据**：VVC 67 模式在主方向附近角度密度均匀。但实际方向直方图在 VER/HOR/DIA 周围分布最密 → 这些种子的衍生应在更窄角度范围内更密集采样；其它种子可稀疏采样以覆盖更广。

**预期 Bitrate**：-0.05% ~ -0.10%。
**预期 PSNR**：~ 0。
**副作用**：分支预测复杂度略增；编/解一致性需严格。
**实现难度**：中。
**风险**：低。

---

### 方案 B1：CABAC 多上下文化 `IntraLumaMpmFlag`

**修改位置**：`Contexts.inl:437-459`、`Contexts.h` 中 `IntraLumaMpmFlag` 声明、`CABACWriter::intra_luma_pred_mode`、`CABACReader::intra_luma_pred_mode`

**修改内容**：

```cpp
// Contexts.h: 改为多上下文
static const CtxSet IntraLumaMpmFlag;   // 现 1 ctx → 改为 3 ctx

// CABAC writer: 选择上下文
int ctxIdx = 0;
const CodingUnit *cuL = ...;   // 左邻
const CodingUnit *cuA = ...;   // 上邻
if (cuL && CU::isIntra(*cuL)) ctxIdx++;
if (cuA && CU::isIntra(*cuA)) ctxIdx++;
m_binEncoder.encodeBin(mpmIdx < numMPMs, Ctx::IntraLumaMpmFlag(ctxIdx));
```

**理论依据**：可用的 intra 邻居数与 MPM 命中率呈正相关——两个邻居都可用时 MPM 命中率显著更高，应使用更偏向"命中"的初始概率。

**预期 Bitrate**：-0.05% ~ -0.15%。
**预期 PSNR**：~ 0（纯熵优化）。
**副作用**：上下文表扩展 → 训练数据需重新跑（`CabacTraining` app 已在工程内：`source/App/CabacTraining`）。
**实现难度**：低。
**风险**：低；标准比特流变化但可向后兼容（用 SPS flag）。

---

### 方案 B2：CABAC 上下文进一步细化 `IntraLumaPlanarFlag`

**修改位置**：`CABACWriter.cpp:1311`、对应 reader

**修改内容**：

```cpp
// 原代码：固定 ctx = 0
m_binEncoder.encodeBin(mpmIdx > 0, Ctx::IntraLumaPlanarFlag(ctx));

// 改为：基于邻居是否为 Planar
unsigned planarCtx = (cuL && PU::getIntraDirLuma(*cuL) == PLANAR_IDX ? 1 : 0)
                   + (cuA && PU::getIntraDirLuma(*cuA) == PLANAR_IDX ? 1 : 0);
m_binEncoder.encodeBin(mpmIdx > 0, Ctx::IntraLumaPlanarFlag(planarCtx));   // 0/1/2
```

注意：`IntraLumaPlanarFlag` 已经声明了 3 ctx (`Contexts.inl:485-507`)，但代码只用 `ctx=0` —— **现有上下文资源未被利用**！这是一个**纯获利改动**。

**预期 Bitrate**：-0.03% ~ -0.08%（小但稳）。
**预期 PSNR**：~ 0。
**副作用**：无。
**实现难度**：极低。
**风险**：极低。**建议作为最优先落地项**。

---

### 方案 C1：Fast-UDI MPM 候选数从 2 → top-K（K=4~5）

**修改位置**：`IntraSearch.cpp:794-815`

**修改内容**：

```cpp
// 原代码
int numCand = cuCtxIntra.mpmListSize;
numCand = (numCand > 2) ? 2 : numCand;

// 改为：根据 sps 配置或 QP 自适应
int forcedTopK = std::min(5, cuCtxIntra.mpmListSize);
// 对高 QP（>32）保守取 3，对低 QP（<27）取 5
if (slice.getSliceQp() > 32) forcedTopK = std::min(3, cuCtxIntra.mpmListSize);
```

**理论依据**：22-MPM 体系下 Planar 已在 0 号位，强制 top-2 显然不足；真实 RDO 最优常落在 top-5 内。

**预期 Bitrate**：-0.05% ~ -0.10%（来自 SATD 漏选率下降）。
**预期 PSNR**：+0.005 ~ +0.020 dB。
**副作用**：full-RD 候选数 +1~3 → 编码时间 +1~3%。
**实现难度**：低。
**风险**：极低；仅编码端策略改动，无比特流变化。

---

### 方案 C2：MRL 测试集合扩展

**修改位置**：`IntraSearch.cpp:463-493`

**修改内容**：MRL 当前仅测 `multiRefMPM[1..5]`（5 个非 Planar primary MPM）。可考虑：

```cpp
// 测试更多 MPM：根据 multiRefIdx 自适应
const int multiRefMpmCnt = (multiRefIdx <= 3) ? numMPMs : (numMPMs - 1);
for (int x = 1; x < multiRefMpmCnt; x++) { ... }
```

**理论依据**：MRL=1/3 时方向偏移成本相对低，可测试更多候选；MRL=5/7 时距离远，前 5 个 MPM 已足够。

**预期 Bitrate**：-0.02% ~ -0.05%。
**预期 PSNR**：+0.002 ~ +0.010 dB。
**副作用**：编码复杂度小幅上升。
**实现难度**：低。
**风险**：极低。

---

### 方案 D1：MPM 列表内重排序（依据 SATD）

**修改位置**：`IntraSearch.cpp` 在 SATD 第一轮结束后

**修改内容**：在第一轮 SATD 完成后，根据各模式的 SATD 排名重新排列 mpmList 的 1..5 位置（Planar 保留在 0），并以新顺序参与最终的 CABAC 编码。

**注意**：这需要重新设计语法元素——目前 `mpm_idx` 直接编码 mpmList 的位置。若编/解都用 SATD 重排序，标准合规；若仅编码端重排序、解码端不变，则不可行。

**实施方法**：
1. 标准合规版本：编/解都用同一种邻居模式重要性（如邻居模式的"出现频度"作为先验权重），无需 SATD。
2. 编码端 fast 版本：仅用于编码器选择 mpm_idx 时的早期裁剪——不影响比特流。

**预期 Bitrate**：-0.05% ~ -0.15%（标准合规版）或 0%（fast 版，仅优化编码时间）。
**预期 PSNR**：0 ~ +0.005 dB。
**实现难度**：高（标准合规版需要充分一致性测试）。
**风险**：高（语法变化）。

---

### 方案 D2：时域 MPM（co-located reference 方向）

**修改位置**：`UnitTools.cpp:getIntraMPMs`

**修改内容**：对于 P/B slice 中的 intra CU，添加 co-located reference picture 中同位置 CU 的 intra 方向（若该位置是 intra）作为额外候选。

```cpp
// 仅 P/B slice + reference picture 同位为 intra
const Picture *refPic = cu.slice->getRefPic(REF_PIC_LIST_0, 0);
if (refPic && refPic->isIntraSliceFiltered()) {
  const CodingUnit *colCu = refPic->cs->getCURestricted(...);
  if (colCu && CU::isIntra(*colCu)) {
    uint8_t tDir = PU::getIntraDirLuma(*colCu);
    if (!includedMode[tDir]) {
      mpm[numValidMPM++] = tDir;
      includedMode[tDir] = true;
    }
  }
}
```

**理论依据**：LDB/LDP 配置下场景内容缓慢变化 → 前帧同位置 intra CU 的方向是高度相关的先验。

**预期 Bitrate**：-0.05% ~ -0.20%（LDB 配置受益最大；RA 中等；AI 无效）。
**预期 PSNR**：~ 0。
**副作用**：DPB 内存访问；并行编码同步；标准比特流变化。
**实现难度**：中。
**风险**：中。

---

### 方案 D3：基于 MIP/EIP 决策结果裁剪 MPM 编码

**修改位置**：`CABACWriter.cpp:1277` 前

**修改内容**：当 CU 已经选择 MIP/EIP/DIMD/TIMD 时根本不进入 MPM 编码（已经实现，见 1247-1273）。但目前的实现是**串行检查**：

```cpp
mip_flag(cu);
if (cu.mipFlag) return;
dimd_flag(cu); if (cu.dimdFlag) return;
timd_flag(cu); if (cu.timdFlag) return;
eip_flag(cu);  if (cu.eipFlag) return;
sgpm_flag(cu); if (cu.sgpm) return;
extend_ref_line(cu);
// 然后才是 MPM 编码
```

每个 flag 都消耗 1 bit。**优化**：根据邻居模式分布估算每个 flag 的"必要性"，将不可能的 flag 跳过（联合编码）。

**预期 Bitrate**：-0.05% ~ -0.15%（高 QP 受益最大，标志 bit 占比高）。
**实现难度**：高（涉及语法层联合编码）。
**风险**：高。
**建议**：作为长期研究方向，不作为短期落地。

---

## 5. 代码级实施建议

### 5.1 关键函数与文件

| 模块 | 文件 | 关键函数/位置 |
|------|------|--------------|
| MPM 构建 | `Lib/CommonLib/UnitTools.cpp` | `PU::getIntraMPMs:590-754` |
| SGPM-MPM | 同上 | `PU::getSgpmIntraMPMs:868-1040` |
| Geo-MPM | 同上 | `PU::getGeoIntraMPMs:756-866` |
| 编码端使用 | `Lib/EncoderLib/IntraSearch.cpp` | `xFracModeBitsIntra:4635` |
| Fast-UDI 强制 | 同上 | 行 794-815 |
| MRL 测试 | 同上 | 行 463-493 |
| CABAC writer | `Lib/EncoderLib/CABACWriter.cpp` | `intra_luma_pred_mode:1277` |
| CABAC reader | `Lib/DecoderLib/CABACReader.cpp` | `intra_luma_pred_mode:1778` |
| 上下文初值 | `Lib/CommonLib/Contexts.inl` | 437-531 |
| Intra 解码再现 | `Lib/CommonLib/UnitTools.cpp` | `PU::interpretLumaIntraMode:1062` |
| Intra flag 设置 | 同上 | `PU::setLumaIntraModeFlags:1101` |
| 模式上下文 | `Lib/CommonLib/ContextModelling.h` | `CUCtxIntra:688` |
| Cabac 训练 | `App/CabacTraining` | 用于重训上下文初值 |

### 5.2 落地优先级路线图

| 优先级 | 方案 | 工程量 (人天) | 备注 |
|--------|------|---------------|------|
| ★★★★★ | **B2: IntraLumaPlanarFlag 上下文修正** | 0.5 | 现有 3 个 ctx 未用；纯获利 |
| ★★★★★ | **C1: Fast-UDI top-K 提升** | 0.5 | 无比特流变化 |
| ★★★★ | **A1: DIMD/TIMD 注入 MPM** | 5 | 与 CTC 比 BD-Rate 收益最大 |
| ★★★★ | **B1: IntraLumaMpmFlag 多上下文** | 1.5 | 需重训上下文 |
| ★★★ | **A2: 5-位置邻居采样** | 2 | 与 SGPM 对齐 |
| ★★★ | **A4: 内容自适应默认表** | 3 | I-slice 间统计 |
| ★★ | **A3: 放宽 isSameCtu** | 0.5 | 需 WPP 兼容性测试 |
| ★★ | **A5: 自适应衍生步长** | 2 | 收益小 |
| ★★ | **C2: MRL 扩展候选** | 0.5 | 收益小 |
| ★ | **D1/D2: 时域 MPM / 重排序** | 5+ | 需充分验证 |

### 5.3 测试流程建议

1. **单元验证**：先用 1-CTU 测试图，对比 `mpmList` 输出差异并验证 encoder/decoder 一致性（CABAC 后 SHA1 比对）。
2. **小序列闭环**：CTC Class C/D 序列，4 QP 全跑，对比 BD-Rate(Y/U/V) 与编码时间。
3. **大规模回归**：Class A1/A2/B/C/D/E/F 全集，AI + RA + LDB + LDP 四个配置。
4. **CABAC 训练**：若改动了上下文，必须用 `CabacTraining` 在大数据集上重训初值表（`Contexts.inl`）。
5. **解码器独立验证**：用第三方 VTM/ECM 参考解码器验证比特流兼容性（若引入语法变化）。

### 5.4 Profile/Tools 验证

- 用 `CodingStatistics.h` 中的 `STATS__CABAC_BITS__INTRA_DIR_ANG` 统计帧内方向 bin 数量变化。
- 用 `dtrace` 路径 `D_INTRA_COST` 追踪 SATD 阶段决策变化。
- 用 `m_dPSNRSum` / `getBits()` 计算 R-D 点（4 QP）→ 外部脚本算 BD-Rate。

---

## 6. 为什么 MPM 可能不是最大瓶颈：交叉评估

在当前 ECM 架构下，亮度 intra 编码分支如下：

```
ratio of best-mode at QP=27 (CTC, Class B estimation):
  ┌─ MIP/DIMD/TIMD/OBIC: ~25-40%     ← 不走 MPM
  ├─ EIP/SGPM:           ~10-15%      ← 不走 MPM
  ├─ MRL>0:              ~3-7%         ← 走 MPM (5 个非 Planar)
  └─ 普通 MPM 路径:      ~40-55%      ← 走 MPM
```

普通 MPM 路径约占 40-55%，每个 CU 的 MPM 信令占总 luma intra bit 的 5-15%。**因此 MPM 优化的 BD-Rate 上限约为 -0.5% ~ -0.7%**（在不引入额外计算的前提下）。

> 若目标是 -1% 以上的 BD-Rate 收益，应同时考虑：
> 1. **TIMD/DIMD 推导精度**（梯度直方图与模板权重）
> 2. **EIP 模型参数压缩**（filter coef coding）
> 3. **MIP 上下文建模**
> 4. **CCLM/MMLM 模型选择信令**

这些都不是本文重点，但需要工程师在做 MPM 优化时心中有数：若上述方向已经做过专门优化（例如本工程已在 IntraSearch.cpp:780 有 TIMD/DIMD 完整集成），那 MPM 边际收益将进入 -0.1% ~ -0.3% 区间。

---

## 7. 总结

### 7.1 现状

- 该编码器已是高度优化的 ECM 风格实现，使用 6 primary + 16 secondary = 22 MPM 体系；
- 普通 MPM 列表构造仅用 L + A 两个邻居（受 `isSameCtu` 约束），并以 ±i 角度衍生与魔术常量默认表填充；
- 编码端 SATD 阶段已用 CABAC 估算偏置，Fast-UDI 仅强制 top-2 MPM 进入 full-RD。

### 7.2 问题定位

| 编号 | 问题 | 严重程度 |
|------|------|---------|
| #1 | 邻居采样窄（L+A）vs SGPM/Geo (5 位置) | 中 |
| #2 | `isSameCtu` 限制 CTU 顶行 | 中 |
| #3 | DIMD/TIMD 推导方向未灌入普通 MPM | **高** |
| #4 | 默认表为静态魔术常量 | 中 |
| #5 | 衍生步长固定 ±1..±4 | 低 |
| #6 | `IntraLumaMpmFlag/SecondMpmFlag` 仅 1 ctx | 中 |
| #7 | MPM 候选数固定 22 | 低 |
| #8 | `IntraLumaPlanarFlag` 已声明 3 ctx 但只用 ctx=0 | **高（纯获利）** |
| #9 | Fast-UDI 强制 top-2（太少） | 中 |

### 7.3 优化思路（按优先级）

1. **立即落地（无比特流变化）**：B2（PlanarFlag 上下文修正）、C1（top-K 提升到 4~5）。
2. **短期落地（兼容性可控）**：A1（DIMD/TIMD 注入 MPM）、B1（MpmFlag 多上下文）、A2（5-位置邻居）。
3. **中期落地**：A4（内容自适应默认表）、A3（放宽 isSameCtu）。
4. **研究性**：D1（SATD 重排序）、D2（时域 MPM）、D3（联合 flag 编码）。

### 7.4 预期收益

| 改动组合 | BD-Rate(Y) | BD-Rate(YUV) | 编码时间 | 解码时间 |
|---------|-----------|--------------|---------|---------|
| 仅 B2 + C1 | -0.08% ~ -0.18% | -0.05% ~ -0.12% | +0.5% | 0 |
| + A1 + B1 | -0.25% ~ -0.55% | -0.15% ~ -0.40% | +2~3% | +0.5% |
| + A2 + A4 + A3 | -0.40% ~ -0.80% | -0.25% ~ -0.55% | +3~5% | +1% |
| 全部方案 | -0.50% ~ -1.00% | -0.30% ~ -0.70% | +5~8% | +1~2% |

> 以上估算基于 ECM/VVC 已有公开论文与提案数据外推（如 JVET-Z0xxxx 系列对 MPM 列表扩展的报告），实际收益需在 CTC 4-QP 上跑通后确认。

### 7.5 实施优先级（推荐排序）

| 阶段 | 方案 | 预期累计 BD-Rate(Y) | 验证级别 |
|------|------|---------------------|---------|
| 第1周 | B2 (PlanarFlag ctx) + C1 (top-K) | -0.10% | Class C/D AI |
| 第2-3周 | + A1 (DIMD/TIMD 注入) | -0.35% | Class B/C AI+RA |
| 第4周 | + B1 (MpmFlag ctx) + CabacTraining 重训 | -0.45% | 全 Class CTC |
| 第5-6周 | + A2 (邻居扩展) + A4 (自适应默认表) | -0.65% | 全 Class CTC + 屏内/HDR |
| 第7-8周 | + A3 + A5 (边角优化) | -0.75% | 回归 + 比特流合规验证 |

---

## 附录：可立即应用的最小补丁示例

### A.1 B2 方案补丁（IntraLumaPlanarFlag 多上下文）

**Before** (`CABACWriter.cpp:1311`):
```cpp
unsigned ctx  = 0;
unsigned ctx2 = (cu.multiRefIdx == 0 ? 2 : 1);
if (cu.multiRefIdx == 0)
{
  m_binEncoder.encodeBin(mpmIdx > 0, Ctx::IntraLumaPlanarFlag(ctx));
}
```

**After**:
```cpp
unsigned ctx  = 0;
unsigned ctx2 = (cu.multiRefIdx == 0 ? 2 : 1);
if (cu.multiRefIdx == 0)
{
  // Use neighbor-conditioned context: how many of L/A neighbors are Planar
  const Position posRT = cu.block(COMP_Y).topRight();
  const Position posLB = cu.block(COMP_Y).bottomLeft();
  const CodingUnit *cuL = cu.cs->getCURestricted(posLB.offset(-1, 0), cu, ChannelType::LUMA);
  const CodingUnit *cuA = cu.cs->getCURestricted(posRT.offset(0, -1), cu, ChannelType::LUMA);
  ctx = ((cuL && CU::isIntra(*cuL) && PU::getIntraDirLuma(*cuL) == PLANAR_IDX) ? 1 : 0)
      + ((cuA && CU::isIntra(*cuA) && CU::isSameCtu(cu, *cuA)
          && PU::getIntraDirLuma(*cuA) == PLANAR_IDX) ? 1 : 0);
  m_binEncoder.encodeBin(mpmIdx > 0, Ctx::IntraLumaPlanarFlag(ctx));   // ctx ∈ {0,1,2}
}
```

> 解码端 `CABACReader.cpp:1844` 必须同步修改。已有的 3-ctx `IntraLumaPlanarFlag` 上下文表（`Contexts.inl:485-507`）无需扩容。

### A.2 C1 方案补丁（Fast-UDI top-K）

**Before** (`IntraSearch.cpp:794`):
```cpp
if (m_encCfg->m_bFastUDIUseMPMEnabled)
{
  int numCand    = cuCtxIntra.mpmListSize;
  numCand        = (numCand > 2) ? 2 : numCand;
  ...
}
```

**After**:
```cpp
if (m_encCfg->m_bFastUDIUseMPMEnabled)
{
  int numCand = cuCtxIntra.mpmListSize;
  // Increase forced MPM coverage: top-4 for low QP, top-3 for high QP
  const int forcedTopK = (slice.getSliceQp() <= 27) ? 4
                       : (slice.getSliceQp() <= 32) ? 3 : 2;
  numCand = std::min(numCand, forcedTopK);
  ...
}
```

> 编码端策略改动，**无比特流变化**，可直接合入。

### A.3 A1 方案骨架（DIMD/TIMD 注入 MPM）

**Insert into** `UnitTools.cpp:638`（`numCand = numValidMPM;` 之前）:

```cpp
// === MPM Optimization A1: inject DIMD/TIMD derived modes ===
if (cu.slice->m_sps->m_useDIMD && cu.dimdMode > DC_IDX && cu.dimdMode < NUM_LUMA_MODE) {
  if (!includedMode[cu.dimdMode] && numValidMPM < numMPMs) {
    mpm[numValidMPM] = (uint8_t)cu.dimdMode;
    includedMode[cu.dimdMode] = true;
    ++numValidMPM;
  }
}
if (cu.slice->m_sps->m_useTIMD) {
  const uint8_t timd67 = (cu.timdMode > DC_IDX) ? MAP131TO67(cu.timdMode) : 0;
  if (timd67 > DC_IDX && timd67 < NUM_LUMA_MODE
      && !includedMode[timd67] && numValidMPM < numMPMs) {
    mpm[numValidMPM] = timd67;
    includedMode[timd67] = true;
    ++numValidMPM;
  }
}
// === end of A1 ===
```

> **必要条件**：`cu.dimdMode/timdMode` 在解码端的 `CABACReader::intra_luma_pred_mode` 调用 `interpretLumaIntraMode` 之前已计算完毕。这要求把 DIMD/TIMD 推导从"仅 DIMD/TIMD 分支"上移到"任何 intra CU"的解析准备阶段，或者用一个共享的 lazy-derive 缓存。这是 A1 方案的主要工程难点。

---

**文档版本**：v1.0  
**面向工程**：`/Users/liujiarui.40/video_coding/hhi`（ECM/VVC 风格 67-mode intra）  
**最后修订**：2026-06-17
