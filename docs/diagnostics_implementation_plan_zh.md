# Voxel-Pillar：约束贡献与强度关联诊断实施方案

本文件为实施规格，不代表功能已经实现。适用范围：当前 `intensity_fusion`、`plane_refine` 和 PillarMap 的静态结构筛选；本轮不修改估计算法、不引入动态检测贡献、不计算没有参考真值支撑的“对应正确率”。

**1. 目标与交付边界**

使用单一 YAML 参数 `ablation_information` 统一启停以下两项诊断，所有结果写入 `ablation_out_path` 指定的同一个 TXT 文件：

- P：在同一位姿、地图和对应关系下，计算筛选前后及不同 Pillar 标签的局部几何信息贡献，并与相同保留点数的随机采样比较。
- I：在同一位姿、地图和候选集下，比较几何与强度评分的平面选择；另外记录实际运行的强度门限、搜索回退和最终选择。

共同要求：

1. `ablation_information` 默认 false；false 时关闭两项诊断的计算和全部新增输出，不拷贝诊断点云、不额外搜索、不创建目录、不打开/创建/追加/清空 TXT，也不输出本功能的控制台摘要。true 时同时开启 P、I 诊断及本方案规定的汇总与变化明细。既有算法日志不受该参数影响。
2. 开启时所有额外计算只读地图。不得更新位姿、状态协方差、平面强度 EMA、退化状态机、LRU 顺序或原始删点掩码。
3. 正常求解只消费原算法的对应关系。对照选择仅用于统计。
4. 为可比性，诊断采用显式传入的状态和配置，不临时修改 `config_setting_`、`state_` 或 `skip_list_` 来模拟开关。
5. 输出诊断耗时；正式实时性实验关闭诊断。确定性回放中对比诊断开关前后结果，不能以会丢包的高速在线回放验证行为一致性。
6. 本轮主实验要求 `dyn_detect_en=false`。若运行配置启用 dyn，允许记录，但元数据与每帧必须标记 `static_pillar_analysis_valid=false`，不得把混合删除归因为静态 PillarMap。

**2. 建议文件改动**

| 文件 | 改动 |
|---|---|
| `include/diagnostics.h`（新增） | 配置、快照、候选记录、统计结果、写文件接口 |
| `src/diagnostics.cpp`（新增） | 信息矩阵分析、随机对照、评分对照、单个 TXT 的结构化记录输出 |
| `include/voxel_map.h` | 只读候选查询/观测线性化接口；诊断对象与必要身份字段 |
| `src/voxel_map.cpp` | 提取公共计算函数；在关联与迭代边界接入诊断 |
| `include/LIVMapper.h`、`src/LIVMapper.cpp` | 帧编号/时间戳、删点前快照、逐帧生命周期 |
| 各数据集实际加载的 `config/*.yaml` | 在 YAML 顶层增加 `ablation_information` 和 `ablation_out_path`，不新增独立 P/I 开关 |
| `scripts/plot_diagnostics.py`（新增） | 读取单个 TXT，按运行标识与记录类型解析，按需离线生成图；节点运行时不自动生成其他文件 |
| `tests/` | 纯计算、候选选择、快照生命周期测试及回放验证说明 |
| `CMakeLists.txt` | 添加源文件和相应测试目标；不无故升级构建要求 |

公共函数提取须保持原有公式、遍历顺序、浮点运算顺序、距离门限、坐标量化、候选首选/平分规则。不要在该提交顺便修正已有模型或改变候选搜索范围。

**3. 公共数据结构与状态定义**

以下为接口草案，工程师可按项目风格调整命名，但必须保留这些语义：

```cpp
struct AblationConfig {
  bool ablation_information = false;
  std::string ablation_out_path = "Log/ablation_information.txt";
};
// writer 初始化成功后，内部 effective_enabled 才为 true。
// 两项诊断、缓存捕获和所有新增输出统一检查 effective_enabled。

struct PlaneKey {
  int64_t root_x, root_y, root_z;
  int layer;
  std::vector<uint8_t> octant_path; // 从根开始的八叉树路径
  int plane_id;                   // 辅助字段，不能仅用地址标识
};

struct FramePointTag {
  uint32_t original_index;        // 本帧降采样后、删点前索引
  int8_t pillar_label;            // NORMAL / REDUNDANT / ISOLATED
  bool removed_actual;           // 来自 DefineSkipPoints 的实际结果
  bool dyn_flag;                 // 仅用于识别混杂，不分析 dyn 贡献
};

struct DiagnosticFrameContext {
  uint64_t frame_id;
  double lidar_timestamp;         // LidarMeasures.last_lio_update_time
  StatesGroup propagated_state;
  bool map_existed_before_frame;
  bool fusion_effective_this_frame;
  double degeneracy_factor_previous;
  // 全量点/标签仅在 P 诊断需要时保存。
  // 原始点与删除后点的索引映射在需要 I 空间输出时保存。
};

struct CandidateRecord {
  PlaneKey key;
  PointToPlane observation;       // 值副本，便于计算 J、R 和后续独立评价
  double geometric_variance;      // 关联门限/评分中的 sigma_l
  double signed_geometric_residual;
  double intensity_difference;
  double intensity_variance;
  double incidence_gain;
  bool intensity_gate_pass;
  bool intensity_gate_applicable; // 观测数达到成熟门限
  bool finite_valid;
  // 原始生产分数及诊断对照分数，分数下溢/平分单独记录。
};
```

- 不持久保存可能因地图淘汰而失效的裸指针。候选记录在本帧完成归约和写出后释放。
- `pillar_label` 与 `removed_actual` 是两个独立维度。`delete_redundant=false` 时，冗余标签点属于保留集合。
- 诊断帧号独立定义，按进入 LIO 的帧递增；不要复用含义不明的地图维护计数器。
- OpenMP 使用每点独占结果槽或线程局部缓存；主线程按原始索引顺序归约和写文件。
- 数据有效性必须有独立字段；无匹配、非有限数和不适用均不能静默编码为“有效的零信息”。

**4. P 诊断：接入时序**

在 `LIVMapper::handleLIO()` 中：

```text
降采样、变换到世界系、初始化强度噪声估计
  -> 开始诊断帧：记录当前传播状态、时间戳、本帧实际融合状态
  -> 原有 BuildPillarMap / DetectNewPoints / UpdateHistory / pillarDetection
  -> 原有 DefineSkipPoints
  -> [公共捕获] 在 ClearPillarMapVoxels 和 removeFlaggedPoints 之前：
       主开关有效开启时读取分类标签、实际删除掩码
       生成 surviving_original_indices，恢复原始索引
       PillarMap 可用时，对完整点云运行既有地图/传播位姿下的只读信息分析
  -> 原有发布、清理、实际删点
  -> 原有 BuildVoxelMap（如需初始化）
  -> 原有 StateEstimation（含 I 诊断）
  -> 原有 UpdateVoxelMap
  -> 完成诊断帧输出
```

P 诊断可在标签仍有效时同步完成，避免深拷贝整幅地图。若延迟执行，必须深拷贝点云与标签；保存一个指向会被 `swap`/清空容器的智能指针不算快照。

主开关开启时维护 surviving_original_indices；即使 P 因 PillarMap 关闭而不适用，I 仍使用恒等索引映射和 NORMAL 标签正常诊断。索引映射按原始点顺序收集实际保留点，须与现有压缩后的点云逐项对应，不必修改 `removeFlaggedPoints()` 的生产行为。上述所有诊断节点在主开关关闭时直接绕过。

首帧没有历史地图时，输出 `status=map_not_ready`，跳过信息比值。不要先用当前帧建图再将“自匹配”统计当成普通历史地图分析。PillarMap 未开启时，P 诊断报告 `status=pillar_disabled`，不暗中开启分类或更新其历史。

**5. P 诊断：统一关联与观测线性化**

对删点前的全部点执行一次纯几何关联：

- 固定地图为当前帧更新前的地图。
- 固定位姿为本帧传播位姿 `x0`。
- 几何门限、根体素查找、邻居回退规则复用当前代码。
- 采用原有纯几何评分 `Glegacy`；关闭诊断查询中的强度融合及强度门限。
- 对全部点一次性确定对应关系，然后按实际掩码划分 keep/drop。禁止对子集重新选择对应关系。

从现有 `StateEstimation()` 提取两个无副作用计算函数：

```cpp
pointWithVar MakeQueryPoint(
    const PointType& point,
    const StatesGroup& iteration_state,
    const Calibration& calibration,
    const VoxelMapConfig& config);

ObservationRow LinearizeObservation(
    const PointToPlane& match,
    const StatesGroup& iteration_state,
    const StatesGroup& propagated_state,
    const Calibration& calibration);
// ObservationRow: J(1x6), variance(标量), inverse_variance, residual, valid
```

必须复用生产代码的传感器协方差、外参、零坐标处理和扰动约定。生产线性化同时使用迭代状态和传播状态，接口不能错误地合并成一个状态。P 诊断中两者都取 x0。

特别区分两个方差：

1. 候选关联中的 `sigma_l` 用于几何门限和候选评分。
2. 求解器观测方差为 `1 / R_inv(i)`，当前含 `0.001` 下限项、平面不确定性和点测量不确定性。

信息贡献必须使用第 2 个；不能直接拿候选评分方差替代。只分析 LiDAR 几何观测项，不加状态先验逆协方差，避免 IMU 先验掩盖几何弱约束。

**6. P 诊断：信息矩阵、方向和保留率**

采用状态顺序 `[dtheta_x,dtheta_y,dtheta_z,dt_x,dt_y,dt_z]`。

为了统一旋转和平移的比较尺度，定义固定特征长度 L（单位 m）：

```text
D = diag(1, 1, 1, L, L, L)
delta_xi = D * delta_z
J_scaled_i = J_i * D
A_i = inverse_variance_i * J_scaled_i.transpose() * J_scaled_i
```

L 在整个实验中固定，写入 TXT 元数据。本版内部默认 L=1 m；它是尺度约定，不是经过验证的最佳参数。不得逐帧改变 L 来改善谱指标。

计算并记录：

```text
A_all, A_keep, A_drop
A_label_normal, A_label_redundant, A_label_isolated
```

要求 `A_all ≈ A_keep + A_drop`，以及 `A_all ≈ 各 label 矩阵之和`。仅有效匹配贡献矩阵；无匹配点单独计数，在该条件分析的矩阵求和中贡献为零。

对对称化后的 `A_all` 进行特征分解：

```text
lambda_0 <= ... <= lambda_5
V = [v_0, ..., v_5]
c_i,k = inverse_variance_i * (J_scaled_i * v_k)^2
rho_keep,k = (v_k^T A_keep v_k) / lambda_k
share_label,k = (v_k^T A_label v_k) / lambda_k
```

使用 A_all 的同一组 v_k 分析 keep/drop/random，不分别重新分解各组后按特征值序号相除。

数值规则：

- `tau_num = max(eigen_abs_tol, eigen_rel_tol * lambda_5)`；`lambda_k <= tau_num` 时，比例输出缺失值与 `direction_valid=false`，而非 0 或 100%。保留原始方向信息量。
- `lambda_5 <= eigen_abs_tol` 时整帧标记 `no_geometric_information`。
- 明显负特征值标记数值异常，不能通过取绝对值隐藏。
- 按 `lambda_k/lambda_5 < weak_ratio_threshold` 标记弱方向；此阈值只是分析口径，不能回写原退化状态机。
- 重根或近重根的单个特征向量不稳定。记录谱间隙，跨帧图优先显示弱子空间投影信息总量；若子空间边界本身跨越近重根，标记该边界不稳定。
- 不把谱比改善写成“增加可观测性”。在该固定线性化分析中，keep 是 all 的子集。

可选增强：另外输出平移 3x3 信息块沿世界 X/Y/Z 轴的投影量，便于解释走廊方向；必须标注这是固定旋转扰动为零时的平移敏感度，不是旋转边缘化后的定位精度。

**7. P 诊断：同点数随机对照**

默认在原始 N 个点中无放回随机保留 K 个，K 等于 PillarMap 的实际保留点数。使用已经计算好的每点贡献，重复 B 次，不重新关联、不重建地图。

- 本版内部默认 B=50；随机引擎独立于生产算法，不新增 YAML 参数。
- 固定种子，并由种子和 frame_id 确定本帧随机序列；写出引擎与种子，保证相同输入可复现。
- 包含无匹配点进行抽样，其信息贡献记零。因此基准是“相同原始保留点数”，不是“相同有效匹配数”。
- 每次使用 A_all 的固定方向计算 rho_random,k，输出均值、P05/P95 和 Pillar 保留率超过随机结果的比例。
- 随机分布描述的是采样变异，不称为算法统计显著性或轨迹误差置信区间。
- 如另加同有效匹配数对照，必须另命名，不与默认结果混合。
- K=N、K=0、N=0 均需定义明确行为；无有效方向时保留点数统计，信息比例缺失。

第一阶段无需把 per-point 6x6 矩阵全部常驻内存：保存 J_scaled 与权重，或分解后保存每点六方向贡献即可。按需累计体素信息；不要用额外地图结构影响生产地图。

**8. I 诊断：三个评分对照与候选集口径**

定义三个对照，所有参数取本帧实际值：

```text
m_g = geometric_residual^2 / geometric_variance
m_I = intensity_difference^2 / intensity_variance

Glegacy = exp(-0.5*m_g) / sqrt(geometric_variance)
Gnorm   = exp(-m_g)
GI      = exp(-((1-w_I)*m_g + w_I*m_I))
```

- `Glegacy vs GI`：整个评分方案的改变。
- `Glegacy vs Gnorm`：几何评分形式改变的影响。
- `Gnorm vs GI`：在统一归一化几何评分基础上加入强度的影响。
- `w_I=0` 时 Gnorm 和 GI 应一致；`w_I=1` 时几何完全不参与融合排序，须记录。

以“通过几何门限、尚未做强度门限”的候选为原始候选，不因开启强度门限提前丢弃诊断记录。

将只读查询组织成：

```cpp
struct CandidatePools {
  std::vector<CandidateRecord> root_candidates;
  std::vector<CandidateRecord> fallback_candidates;
  bool root_exists;
};
CandidatePools CollectCandidatesReadOnly(...);
```

保持现有几何搜索语义：根不存在时当前算法不自动搜索邻居；递归到一个有效平面后不再深入其子树；邻居位置仍按现有代码计算。不得把“额外诊断查询邻居”变成生产估计新增候选。

I 诊断分别输出两个口径：

**A. 固定候选集的评分比较（主机制指标）**

- 根中存在几何有效候选，固定集合 C 就取根候选；否则取当前规则对应的邻居候选。
- Glegacy/Gnorm/GI 在同一 C 上分别选最大者，默认都不应用强度门限。
- 若分析强度门限后的排序，则三者必须使用同一门限后集合，另输出 `pool_policy=shared_intensity_gated`。
- 不将根与邻居无条件合并后声称复现当前算法。

**B. 按实际策略回放的门限/回退比较（运行解释指标）**

- 对每个策略独立执行“根中有可接受候选则使用根，否则按现有规则回退邻居”。
- 记录门限是否适用、拒绝了多少候选、是否触发邻居回退、是否获得或失去匹配。
- 额外遍历只服务诊断；不修改生产查询结果。
- 策略至少包括：几何且无强度门限、融合且无强度门限、融合且配置的强度门限，以及本帧实际生产策略。
- 回放的实际策略结果必须与生产返回的平面 key 一致，否则输出 mismatch 并将该条标记无效，不将其纳入有效变化统计。

保留生产代码的候选顺序、严格 `>` 比较和“第一个有效候选必填”语义，包括浮点下溢行为。可额外输出 log_score 用于分析，但不能用 log_score 静默替换生产算法评分。记录分数下溢、平分和非有限候选数，以免将数值行为误当作强度收益。

**9. I 诊断：接入迭代与计数定义**

在 `BuildResidualListOMP()` / `build_single_residual()` 的候选计算边界建立只读追踪接口或共用的候选求值函数。候选求值需要显式传入迭代位姿，特别是入射角补偿不能用过期旋转。

默认记录每帧第一次和最后一次实际 IESKF 迭代：

- 在每次候选查询时捕获该次迭代位姿、平面参数和值、生产选择。
- 第一轮摘要保存一次，最后一轮缓存不断覆盖；收敛退出后写最终缓存。
- 不在退出后使用已经更新的位姿重新查询并冒充“最后一轮实际关联”。
- 若只有一轮，输出一条记录，`is_first=true,is_last=true`。
- 每点候选临时记录归约后释放，仅保存第一轮、最后一轮摘要及对应变化明细；主开关同时控制这些记录，不另设明细开关。

帧开始时记录：

```text
fusion_effective_this_frame = intensity_fusion_en &&
    (!degeneracy_adaptive_en || intensity_fusion_active_at_frame_start)
```

帧结束另记录 `degeneracy_factor_last` 与 `fusion_effective_next_frame`。不能拿帧末更新的状态解释本帧是否使用了强度。

固定候选集比较中，每个 pair 分别记录：

```text
N_examined      实际检查的点数
N_both_valid    两个策略均有有效选择的点数
N_multi        N_both_valid 中，固定集合至少含两个不同 plane key 的点数
N_changed      N_both_valid 中，两个 plane key 不同的点数
N_changed_multi  N_multi 中改变选择的点数
change_rate    = N_changed / N_both_valid
ambiguity_change_rate = N_changed_multi / N_multi
multi_ratio    = N_multi / N_both_valid
```

零分母输出缺失值及有效标志。主指标使用点数而非候选数，不能重复累计迭代次数。按策略回放单独统计：`both_valid / both_invalid / gained / lost / switched / unchanged`；只有双方有效且 key 不同才计入 switched。

融合未启用时，GI 的选择属于 counterfactual（假设启用）的结果，输出 `applied=false`。不能把它报告为实际发生的强度修改。启用时若变化来自门限/邻居回退，应与固定集评分变化区分。

**10. 单个 TXT 的输出协议与最低字段**

`ablation_out_path` 表示完整 TXT 文件路径，不是目录。全部元数据、逐帧汇总、方向信息、矩阵、变化点与平面参数写入此文件；不再生成独立 CSV、JSON 或第二个诊断文件。

文件采用 UTF-8 文本，每条记录一行：

```text
RECORD_TYPE<TAB>key=value<TAB>key=value...
```

其中 `<TAB>` 是实际制表符，不是六个可见字符。解析时先按制表符切分，再按第一个 `=` 分离键和值。字段字符串按 JSON 字符串规则加引号并转义制表符、换行和引号，数值数组使用单行紧凑数组；有限浮点数以足够恢复 double 的精度（例如 `max_digits10`）输出。无效或不适用数统一写 `NA`，并给出 valid/status 字段；不使用数值 0 冒充缺失值。使用固定小数点区域设置，避免区域设置破坏解析。

所有记录均包含 `run_id`；帧相关记录还包含 `frame_id` 和传感器 `timestamp`。帧内关联使用 `stage`、`iteration_id` 和 `original_index` 明确标识，不能依靠相邻行顺序推断。记录类型如下：

| 记录类型 | 组织方式 | 最低字段 |
|---|---|---|
| `RUN_BEGIN` | 每次节点启动一条 | run_id、schema_version、会话开始时间、实际输出路径、诊断请求状态/有效状态 |
| `META` | 本次运行头部，可多条 | 代码版本/工作区修改标志、数据集/序列标识、全部算法有效参数、内部诊断常量、外参、线程数、随机引擎与种子、评分公式版本、坐标/状态约定 |
| `FRAME_BEGIN` | 每帧一条 | frame_id、timestamp、本帧融合状态、前帧退化因子 |
| `PILLAR_FRAME` | 每帧一条 | status、静态分析有效性、标签/实际保留删除的原始点数和有效匹配数、无匹配/异常数、信息矩阵分解状态、诊断耗时 |
| `PILLAR_DIRECTION` | 每帧每方向一条 | direction_id、lambda_all、direction_valid、is_weak、v 六分量、keep/drop/三标签信息量、rho_keep、随机均值/P05/P95/胜出比例、谱间隙标志 |
| `PILLAR_MATRIX` | 每帧每集合一条 | group、对称 6x6 矩阵上三角 21 个元素，固定顺序 |
| `ASSOCIATION_FRAME` | 每帧每选定迭代每比较对一条 | iteration_id、is_first/is_last、pool_policy、score_pair、全部计数和比例、门限/回退计数、fusion_effective_this_frame、applied、诊断耗时、回放一致性 |
| `ASSOCIATION_POINT` | 每个有变化的点一条或按比较对区分 | original_index、迭代、点坐标与强度、候选数、各选择 key、评分/归一化误差、门限/回退状态、发生何种改变 |
| `ASSOCIATION_PLANE` | 按帧/迭代/PlaneKey 去重 | PlaneKey、法向、中心、半径、平面不确定性；供变化点记录引用 |
| `FRAME_STATE` | 每个记录阶段一条 | stage、iteration_id、实际使用的位姿、propagated pose、前帧/末轮退化因子、本帧/下帧融合状态 |
| `FRAME_END` | 每帧一条 | status、本帧记录条数、诊断计算/序列化/写入耗时、统计完整性 |
| `RUN_END` | 正常结束一条 | 处理帧数、异常计数、会话完整性、结束原因 |

TXT 内容示意（尖括号为占位符，不是实验结果；实际使用制表符分列）：

```text
RUN_BEGIN  run_id="<session>"  schema_version=1
META  run_id="<session>"  characteristic_length_m=1.0  random_trials=50  random_seed=42
PILLAR_FRAME  run_id="<session>"  frame_id=<id>  timestamp=<t>  points=<N>  kept=<K>  status="ok"
PILLAR_DIRECTION  run_id="<session>"  frame_id=<id>  timestamp=<t>  direction_id=0  direction_valid=false  rho_keep=NA
ASSOCIATION_FRAME  run_id="<session>"  frame_id=<id>  timestamp=<t>  iteration_id=0  score_pair="Gnorm_vs_GI"  changed=<N>  applied=true
FRAME_END  run_id="<session>"  frame_id=<id>  timestamp=<t>  complete=true
RUN_END  run_id="<session>"  complete=true
```

主开关开启后，每帧汇总均输出；首帧、PillarMap 关闭、无候选等情况输出明确 status，不直接消失。发生以下任一情况的点输出明细：三种评分选择不同、门限/回退比较中获得或失去匹配、实际策略回放不一致。输出全部此类变化点及其选择平面参数，不设置另一明细开关、不默认采样或截断；无变化点只参与汇总。按帧缓冲和释放，不积累整条序列明细。

空间变化点的统计对象为实际参与估计的点，原始索引通过 surviving_original_indices 恢复。P 分析覆盖删点前全量点；两者总体不同，记录中用 `point_population` 明确区分。

I 诊断本轮不输出 correctness/improvement 成功率。它输出选择变化及待验证数据，后续基于独立参考位姿/表面标注另行评价。

文件生命周期要求：

1. false 时，不校验或访问输出文件，不创建父目录，不打开已有文件；即使路径无效也不产生该功能的路径错误日志。
2. true 时，在第一帧诊断前解析路径、创建缺失父目录并打开文件。绝对路径按原值使用；相对路径统一相对项目 `ROOT_DIR` 解析，避免受 roslaunch 工作目录影响。空路径视为配置错误。
3. 每次启动以追加模式打开同一指定文件；自动生成本次 run_id，以 `RUN_BEGIN/RUN_END` 区分多次运行，不截断旧结果，也不擅自改为带时间戳的另一个文件名。解析器必须能选择单个 run_id。
4. 禁止多个进程并发写入同一路径。工程实现使用覆盖整个运行期的非阻塞独占文件锁；获取失败仅禁用该进程诊断并报告原因，不向已有文件插入记录。追加打开前后的文件创建和锁定须避免先截断文件。
5. 主线程在点级并行计算归约后写入；每帧结束 flush，正常退出写 RUN_END 后 flush/close。析构在未启用或未打开时不能创建文件。
6. 路径不可写、获取锁失败、打开失败或写入/flush 失败时，ROS 单次报告错误并关闭内部 effective_enabled，停止后续额外计算和输出；原估计继续。请求参数值仍记录为 true（若文件还能写），不能伪装成用户主动关闭。无法写入的错误不能承诺已保存到 TXT。
7. 中途崩溃可能缺少 FRAME_END/RUN_END；解析器识别不完整帧/会话并标记，不当成正常完整数据。性能计时明确是否包含 flush，FRAME_END 本身和最后 flush 的开销不承诺记录在其写出前的计时字段中。
8. 成功运行不打印新增逐帧诊断摘要；所有诊断数据进入 TXT。启用失败或运行中 I/O 故障只发必要的单次错误日志。

**11. YAML 参数与加载方式**

仅新增两个对外 YAML 参数，放在各数据集 YAML 的顶层，不放入 `lio:`、`pillar_map:` 或 `diagnostics:`：

```yaml
ablation_information: false
ablation_out_path: "Log/ablation_information.txt"
```

开启示例：

```yaml
ablation_information: true
ablation_out_path: "/tmp/voxel_pillar_ablation.txt"
```

使用与数据集 YAML 加载位置一致的 ROS NodeHandle，在初始化时读取，不使用带前导 `/` 的全局参数名：

```cpp
AblationConfig cfg;
nh.param<bool>("ablation_information", cfg.ablation_information, false);
nh.param<std::string>("ablation_out_path", cfg.ablation_out_path,
                      std::string("Log/ablation_information.txt"));

// 先判断总开关，再做任何输出路径访问或诊断资源初始化。
if (cfg.ablation_information) {
  diagnostics.initialize(cfg); // 路径/文件锁/流成功后 effective_enabled=true
}
```

节点启动时读取一次；更改 YAML 后重启生效。本轮不实现运行中热切换。工程师需检查各 launch 的 rosparam 加载命名空间，确保 NodeHandle 能读取顶层键，并将解析后的参数名和值写入 META。

取消旧方案的独立 P/I 开关、输出目录参数、明细开关和控制台频率参数；不要让旧参数覆盖这个总开关。高级计算口径采用内部常量并写入同一 TXT 的 META：

| 内部常量 | 本版默认值 |
|---|---|
| frame_stride | 1，每帧 |
| characteristic_length_m | 1.0 |
| eigen_abs_tol / eigen_rel_tol | 1e-9 / 1e-6 |
| weak_ratio_threshold | 0.1，仅诊断分类 |
| random_trials / random_seed | 50 / 42 |
| association_iterations | first_last |
| association_point_details | 所有变化点，无默认截断 |

数值常量同样须通过内部有效性校验；未来若需调整，应更新元数据及实现版本，不静默改变分析口径。`ablation_information=true` 只开启分析，不自动开启 PillarMap、强度融合或其他被测算法模块。

**12. 验收测试**

总开关与单文件输出测试：

1. 两个 YAML 键缺省时总开关为 false，不创建默认 TXT。
2. false 配合不存在、不可写或空路径，均不访问路径、不创建目录、不输出本功能错误/摘要；原有 TXT 的内容、大小和修改时间保持不变。
3. true 配合可写路径，同时得到 P、I 记录或各自明确的不适用状态；节点不产生独立 CSV、JSON 或其他诊断文件。
4. 绝对路径按指定位置写入，相对路径始终相对 ROOT_DIR；更换启动工作目录不改变解析结果。
5. 连续两次运行写入同一 TXT，旧记录保留，两组 run_id 可独立解析，互不混入统计。
6. 两个进程使用相同输出路径时，后启动进程获取锁失败并只禁用其诊断，文件中不出现交错写入。
7. true 配合空路径/不可写路径，或模拟运行中写失败，单次报告错误并停用额外诊断计算，原估计继续；不得静默转存到另一路径。
8. TXT 中包含字符串转义、NA、数组、多个会话和不完整帧的样例，解析器能正确恢复并标记有效性。
9. 主开关开启时，变化点明细数量与对应汇总中的去重变化点数量一致；关闭时不执行候选对照、随机采样和诊断序列化。

P 的纯计算测试：

1. 水平平面：纯 X/Y 平移的点面敏感度为零，Z 非零；非原点点可对部分旋转有贡献。
2. 平面法向翻转及 d 同时翻转，信息贡献不变。
3. `A_all=A_keep+A_drop`；三类标签矩阵之和等于 A_all。
4. `sum_i c_i,k≈lambda_k`；有效 rho 位于 [0,1] 的数值容差内。
5. 无删除时 keep 比例与随机对照为 1；全部删除且原信息有效时为 0。
6. 零匹配、单平面秩亏、近重根、非有限输入均输出明确有效性，不产生误导比例。
7. 删除掩码与标签解耦：冗余标签且删除开关关闭时，仍计入 keep。
8. 随机种子固定时结果可复现；随机样本保持准确 K 个且无重复。

I 的候选与策略测试：

1. 单候选、门限关闭时三种评分选择相同，变化数为零。
2. 两个候选几何相似、强度不同的构造数据，GI 可以选出不同候选；该测试只验证机制，不证明真实数据正确率。
3. w_I=0 时 Gnorm 与 GI 一致；存在不同几何方差时 Glegacy 与 Gnorm 允许不同。
4. 根有几何候选但全部被强度门限拒绝时，策略回放准确进入邻居；固定候选集对照不偷偷扩大搜索范围。
5. 根体素不存在时，诊断回放保持当前生产行为。
6. 平分、exp 下溢、无候选、不成熟强度统计、非有限分数均有确定语义，实际策略回放与生产选择一致。
7. 退化状态机在帧末翻转时，本帧 applied 字段仍对应帧初状态。
8. 一帧多轮迭代不重复计入“每帧点数”；单轮只输出一次 first/last。

行为一致性回放：

- 先验证重构后的诊断关闭版本与改动前版本；再验证同一版本诊断关/开。
- 固定输入顺序、配置、线程数，确保回放不丢消息；比较每帧选中平面、有效残差数、位姿/协方差、地图统计和强度状态。
- 主路径运算顺序不变时应争取逐项一致；若存在既有并行非确定性，先测量基线重复运行差异，再给出数值容差及原因。
- 分别测量原算法阶段耗时、诊断搜索/计算耗时、诊断 I/O 耗时。不要从受额外诊断扰动的总耗时推断正式实时性能。
- 清理、点云压缩、零点、首帧、地图淘汰后均不出现索引错位或悬空引用。

**13. 实施顺序与论文用途**

按下列顺序提交，便于定位回归：

1. 两个 YAML 参数、单 TXT writer/解析协议、公共身份与元数据、无副作用候选/线性化接口；先证明诊断关闭行为不变。
2. I 固定候选集的三个评分对照，再实现门限/回退回放与 first/last 输出。
3. P 删点前固定快照分析、标签/实际掩码分组及信息矩阵输出。
4. P 同点数随机对照、诊断绘图、完整回放验收。

绘图最少包含：P 的点数保留率与各方向信息保留率、Pillar 与随机保留分布、三类点信息贡献占比；I 的多候选比例、实际/假设选择改变比例、评分形式与强度项影响的拆分、门限/回退计数与退化状态时间线。

最终交付应包含源码、两个 YAML 参数的配置说明、单 TXT 记录协议、短样例输出、读取 TXT 的绘图脚本和验收记录。诊断得到的是条件于当前地图和位姿的机制证据；不同算法运行的地图会分化，因此完整端到端消融仍需独立开展。只有诊断结果支持假设后，才能在论文中表述“保留弱方向信息”或“强度有效改变歧义关联”。
