# Talos 上手文档

> 面向新加入项目的开发者。读完本文后，你将能够：解释 Talos 的整体架构与五级流水线、定位每个模块的代码位置、跑通构建与测试、看懂一条数据从相机到云台的完整旅程，并知道去哪里继续深入。
>
> 配套资料：根目录 [README.md](../README.md)（项目概览）、[CLAUDE.md](../CLAUDE.md)（工程规范与调度器编程模型）、本目录下的 [fast-tf.md](fast-tf.md)、[toml-helper.md](toml-helper.md)、[rclcompat.md](rclcompat.md)。

---

## 1. Talos 是什么

Talos 是一个 **RoboMaster 自瞄上位机**（火控系统，FCS）。它把传感输入、目标感知、状态估计、瞄准规划、开火决策和控制输出组织为一条火控流水线：

```text
L1 Sensor ──▶ L2 Perception ──▶ L3 Estimation ──▶ L4 Planning ──▶ L5 Weapon ──▶ 云台 MCU
  (相机/IMU)    (装甲板/神符/LDM)   (EKF 跟踪)        (瞄准解算/目标选择)  (MPC/开火决策)   (USB/串口)
```

核心设计哲学（详见 CLAUDE.md）：

- **用 ML 思维写 C++**：`struct` 表达 product type，`std::variant` 表达 sum type，`std::expected<T, std::string>` 表达可恢复失败，RAII 表达生命周期；
- **先建模数据，再写控制流**：非法状态不可表示；
- **类型安全**：坐标系（fast_tf）、业务概念全部强类型，禁止字符串/裸指针/魔法数污染业务层；
- **数据驱动执行**：模块间通过显式声明的 Channel 依赖连接，由调度器自动分析 DAG 决定执行顺序，从结构上杜绝隐式共享状态。

## 2. 代码地图

```
talos26/
├── crates/                      # 核心库（"crate" 命名沿袭 Rust 风格）
│   ├── primitive/               # 通道原语：SPSC/SPMC 三重缓冲、spin、性能探针、线程亲和性
│   ├── scheduler/               # 调度器：World、System、DAG 依赖分析、执行策略
│   ├── fast_tf/                 # 类型安全坐标变换（强类型 frame，防止坐标系误用）
│   ├── math/                    # SO2、Euler 等数学工具
│   ├── toml/                    # TOML 配置解析辅助（READ 宏、merge）
│   ├── log/                     # spdlog 封装
│   ├── quanta/                  # 图传：HEVC 编码（FFmpeg/AX 后端）+ 分片传输
│   ├── chiral/                  # 数据采集/记录端点（共享内存）
│   ├── hardware/at_gimbal/      # 云台 MCU 协议（talos_gimbal：USB/串口帧收发）
│   ├── hardware/hik_camera_driver/  # HIK 工业相机驱动（仅 Linux x86_64）
│   └── hardware_daedalus/       # Daedalus IPC 共享内存客户端（真值/评测链路）
├── src/
│   ├── fcs/                     # 火控系统主体（五级流水线 + 标定 + 运行时）
│   │   ├── core/                # 领域类型、channel topics、时间、PNP 类型、弹道模型
│   │   ├── L1_sensor/           # 相机输入、输出接口（IPC/MCU/Chiral 三选一）、parcel
│   │   ├── L2_perception/       # armor（装甲板）、rune（能量机关）、ldm、common
│   │   ├── L3_estimation/       # tracker（EKF）、energy_meter、ldm_naive（InEKF）
│   │   ├── L4_planning/         # aimer、目标选择、gimbal planner、弹道构建
│   │   ├── L5_weapon/           # MPC 轨迹优化、开火决策
│   │   ├── calibration/         # 相机内参、ChArUco、棋盘格、手眼标定
│   │   ├── chiral/              # 采集系统（TrackerOutput → 共享内存）
│   │   ├── runtime/             # boot、配置加载、capturer、replay、图传系统
│   │   └── tests/               # 单元测试与算法回归测试
│   ├── fcs_visualization/       # Foxglove 可视化（WebSocket server、场景构建）
│   ├── main.cpp                 # 主程序入口
│   ├── playground.cpp           # 实验入口（talos-playground）
│   └── talos_replay.cpp         # 录制回放工具（talos-replay）
├── config/                      # 机器人/视觉配置（robot/、vision/）
├── at_vision.toml               # 运行时总配置入口
├── calibration.toml             # 标定配置（rm_calibration 使用）
├── scripts/                     # clang-format、pre-commit hooks、systemd、环境配置
├── 3dparty/                     # hik_sdk、lieplusplus、magic_enum、tinympc
├── models/                      # ONNX 模型文件
├── benchmark/                   # 基准测试（bench_mpc）
├── docs/                        # 文档（你在这里）
└── build.sh                     # 一键构建脚本
```

## 3. 核心架构概念：调度器编程模型

所有数据处理 System 都挂在自定义调度器上。一个 System 通过成员声明自己读/写哪些通道与资源：

```cpp
struct MySystem {
  spsc<Input, TagInput> reader;        // SPSC 只读（点对点）
  spmc_mut<Output, TagOutput> writer;  // SPMC 只写（一对多）
  res<Config> config;                  // 只读资源（版本追踪，变化时触发重跑）
  res_mut<State> state;                // 可写资源（写后版本号自增）
  local<Temp> temp;                    // 本地变量（不参与依赖分析）

  void bind(World& world);             // 初始化期绑定通道
  void run(World& world);              // 每次触发执行的逻辑
};
```

要点：

- **Channel**：SPSC/SPMC 无锁三重缓冲（三槽：写槽/读槽/中间槽），move 语义零拷贝，通过 `Tag` 类型区分 topic（定义见 [channel_topics.hpp](../src/fcs/core/channel_topics.hpp)）；
- **Resource**：`res`/`res_mut` 带版本号，调度器根据版本追踪自动触发依赖系统；
- **执行策略**：
  - `fixed_rate<Hz>`：独占线程定频触发（数据源、跟踪、控制环），如 `fixed_rate<250>`；
  - `fixed_rate_silent<Hz>`：定频但不通知下游（高频静默更新，如 IMU）；
  - `pool_compute`：TBB 线程池，依赖触发（默认）；
- **boot 流程**：`fcs::boot()`（[boot.cpp](../src/fcs/runtime/boot.cpp)）插入所有资源 → 注册 L1-L5 全部系统 → `scheduler.build()` 冻结结构 → `scheduler.run()`。

## 4. 构建与运行

### 4.1 工具链

| 项 | 要求 |
| --- | --- |
| 语言标准 | C++23 |
| 编译器 | Linux/嵌入：LLVM clang 21+ + libstdc++13（GCC13 工具链）；macOS：Apple Clang + libc++ |
| 构建系统 | CMake ≥ 3.14 + [build.sh](../build.sh) |
| 关键依赖 | oneTBB 2021.9+、Eigen3 3.4+、OpenCV 4.x、Ceres 2.x、FFmpeg 8.x、spdlog 1.17、ONNXRuntime ≥1.26、TensorRT（可选）、magic_enum、lieplusplus、tinympc、hik_sdk、foxglove_cpp |

支持平台：PC（x86_64）、NVIDIA Jetson Orin NX（aarch64）、Axera AX650N（aarch64）、macOS；Android/Termux 为实验性。交叉编译工具链位于 `cmake/toolchains/`，平台适配文档见 [ax650.md](ax650.md)。

### 4.2 常用命令

```bash
./build.sh                  # 默认构建（release）+ 运行 talos
./build.sh --debug          # Debug 构建
./build.sh --asan           # AddressSanitizer 构建
./build.sh --tsan           # ThreadSanitizer 构建
./build.sh --test           # 构建并跑全部单元测试（ctest）
./build.sh --calib          # 构建标定工具 rm_calibration（配 calibration.toml）
./build.sh --playground     # 构建实验入口 talos-playground
./build.sh --cross          # 交叉编译（目标平台按工具链配置）
./build.sh --no-run         # 只构建不运行
```

主要产物：`talos`（主程序）、`talos-playground`、`talos-replay`、`rm_calibration`、`bench_mpc`、各单元测试可执行文件。

### 4.3 配置

运行时只读一个文件 **`at_vision.toml`**，它按 **主机名自动匹配预设**：

```toml
backend = "Direct"            # Direct（直连相机+MCU）/ Daedalus（共享内存仿真）/ CameraOnly
robot = "infantry_4"          # 对应 config/robot/infantry_4.toml
vision = "daedalus_vision"    # 对应 config/vision/daedalus_vision.toml

[preset]                      # 主机名 → 预设映射（部署机自动生效）
"ax650" = { robot = "drone", vision = "drone" }
"byq"   = { backend = "Daedalus", robot = "infantry_4", vision = "daedalus_vision" }
```

- `config/robot/*.toml`：机器人结构（云台/相机/枪管外参）、MCU 参数、弹速；
- `config/vision/*.toml`：各流水线层的算法配置（检测阈值、EKF 噪声、aimer 延迟、L5 MPC、quanta 图传等）；
- `calibration.toml`：标定模式（Intrinsic/Handeye/Full）、标定板、采集约束（`rm_calibration` 使用）。

### 4.4 日志

spdlog 封装（`crates/log`），启动日志打印构建版本、检测颜色、后端名称、弹速等。运行时可观察 `color:`、`armor backend:`、`bullet_speed=` 等关键行确认配置生效。

## 5. 数据流主线：一帧图像的旅程

```text
[相机] (HIK / 共享内存)
   │ ImageFrame ── ImageChannelTopic (SPMC)
   ▼
[L2 ArmorDetectorSystem]  fixed_rate<200>        src/fcs/L2_perception/armor/systems.cpp
   │ ArmorDetectionBatch ── DetectionChannelTopic
   ▼
[L2 PnPSolverSystem]      fixed_rate<200>        src/fcs/L2_perception/armor/solver.hpp
   │ ArmorMeasurementBatch ── MeasurementChannelTopic（含位姿+协方差）
   ▼
[L3 armor_tracker]        fixed_rate<250>        src/fcs/L3_estimation/tracker_systems.cpp
   │ TrackerOutputs ── TrackerOutputChannelTopic（全部跟踪器，含 Idle）
   ▼
[L4 Aimer + TargetDecider]  fixed_rate<250>      src/fcs/L4_planning/aimer/aimer_systems.cpp
   │ ControlIntent ── ControlIntentChannelTopic（Track/Shot/Hold 三态 variant）
   ▼
[L5 Enhanced Weapon]      fixed_rate<250>        src/fcs/L5_weapon/enhanced/weapon_systems.cpp
   │ WeaponCommand ── WeaponCommandChannelTopic（yaw/pitch/角速度/开火）
   ▼
[L1 OutputInterface]                             src/fcs/L1_sensor/output_interface.hpp
   ├─ McuOutput   → USB/串口 → 云台 MCU（talos_gimbal 帧协议）
   ├─ IpcOutput   → 共享内存 → Rust 模拟器（Daedalus IPC）
   └─ ChiralOutput→ 共享内存 → 采集记录（chiral）
```

旁路：

- **图传**：图像 → `stream_encode`（HEVC）→ QuantaPacket 分片 → MCU 转发（见 §7.2）；
- **可视化**：各层系统把 debug 数据写入 Foxglove sink → WebSocket / MCAP（见 §7.3）；
- **采集**：`capturer` 按需落盘，`chiral` 把 TrackerOutput 发布到共享内存，`talos-replay` 回放录制。

## 6. 模块详解（按流水线）

### 6.1 L1 采集层（`src/fcs/L1_sensor/`）

| 文件 | 职责 |
| --- | --- |
| [camera_interface.hpp/.cpp](src/fcs/L1_sensor/camera_interface.hpp) | 相机统一接口（HIK 相机 / Daedalus 共享内存 / CameraOnly 三种来源） |
| [output_interface.hpp/.cpp](src/fcs/L1_sensor/output_interface.hpp) | 输出三选一：`IpcOutput` / `McuOutput` / `ChiralOutput`，用 `std::variant` 表达；`create_usb()` / `create_serial()` 工厂 |
| [parcel.hpp](src/fcs/L1_sensor/parcel.hpp) | `Frame`：seq + 纳秒时间戳 + cv::Mat |

### 6.2 L2 感知层（`src/fcs/L2_perception/`）

#### armor/ — 装甲板检测 + PnP 位姿解算

- **推理后端抽象**（[backend.hpp](src/fcs/L2_perception/armor/backend.hpp)）：`DetectorBackend` 用 `std::variant` 包四种后端 —— `TraditionalBackend`（传统 CV）、`OrtBackend`（ONNXRuntime）、`TrtBackend`（TensorRT）、`AxeraBackend`（AX650 NPU），由配置在运行时选择，`detect()` 是 noexcept 边界（内部错误用 `std::expected` 返回）；
- **ROI 回读**（[readback_roi.hpp](src/fcs/L2_perception/armor/readback_roi.hpp)）：从跟踪器回读上一帧目标位置，裁剪 ROI 加速检测；
- **PnP 位姿解算**（[solver.hpp](src/fcs/L2_perception/armor/solver.hpp)）——项目最核心的几何代码，流程：
  1. 4 角点 → 去畸变归一化平面（`cv::undistortPoints`），并做 **0.65px 相关尺度去偏**（检测角点普遍偏内）；
  2. `cv::solvePnP(IPPE)` 求 6DoF 初值（或先验种子 + ITERATIVE）；
  3. **Ceres 约束重投影 BA（物理 4-DoF 流形）**：只优化 `armor_yaw + 相机系 yaw/pitch/distance`（ypd 参数化），装甲板 pitch 固定（普通板 +15°，Outpost -15°）；`R = R_cam_imu · R_yaw · R_pitch`；
  4. 从重投影雅可比计算 **位姿协方差**（`cov_ypdr`）与条件数，随测量传给 L3 增强 EKF 的 R 矩阵；
- **装甲板 3D 模型**：小板 135×55mm、大板 230×55mm（`build_model_points()`）；
- 系统注册：检测 @200Hz → PnP @200Hz（[systems.cpp](src/fcs/L2_perception/armor/systems.cpp)）。

#### rune/ — 能量机关（神符）

- 传统 CV 流水线（[rune_detector.hpp](src/fcs/L2_perception/rune/rune_detector.hpp)）：中心 R 检测 → 箭矢检测 → 靶子（叶片）检测，帧间连续性追踪；
- 位姿：**5-DoF 约束重投影**（roll + yaw + translation，pitch 固定 0，神符平面无倾角），同样用 Ceres；
- 输出 `RuneObservation`，`pool_compute` 执行（[rune_systems.cpp](src/fcs/L2_perception/rune/rune_systems.cpp)）。

#### ldm/ — LDM 目标

- [ldm_detector.hpp](src/fcs/L2_perception/ldm/ldm_detector.hpp)：配置驱动检测器，@200Hz，输出 `LdmDetection` → `LdmMeasurement`（配合 `ldm_geometry.hpp`、`ldm_solver.hpp`）。

### 6.3 L3 估计层（`src/fcs/L3_estimation/`）

#### tracker/ — 主跟踪器（EKF）

- **EKF 实现**（[extended_kalman_filter.hpp](src/fcs/L3_estimation/tracker/extended_kalman_filter.hpp)）：模板化 EKF，用 `ceres::Jet` 自动微分计算雅可比（不手推导数）；自带**收敛监测**（NIS 归一化新息平方、协方差对角，Converging/Converged/Diverging 状态）；
- **运动模型**（[new_motion_model.hpp](src/fcs/L3_estimation/tracker/new_motion_model.hpp)）：
  - `RobotEkfMotionModel`（机器人，11 维）：`[xc, vx, yc, vy, z0, vz, yaw, v_yaw, log(r0), log(r1), h]`，4 块装甲（r0/r1 分低/高装甲半径，h 为高装甲高度差），CV 模型；
  - `OutpostEkfMotionModel`（前哨站，7 维）：`[xc, yc, yaw, v_yaw, z0, z1, z2]`，3 块装甲，固定半径 0.2765m、额定转速 2.5133 rad/s；
- **观测模型**：odom 球坐标 `z = [装甲方位角, 俯仰角, log(距离), 装甲朝向角]`；R 与距离/视角相关，并叠加 L2 传来的 **PnP 几何协方差**（`pnp_augmented_measurement_R_`）；
- **数据关联**（[new_tracker.hpp](src/fcs/L3_estimation/tracker/new_tracker.hpp)）：Mahalanobis gate + **相位先验**（由上一帧 armor_id 与 v_yaw 预测当前相位，Outpost 用 {-a, 0, +a} 三模式混合）+ **observation-only commit gate**（先验只影响候选排序，不污染 last_armor_id 锚点）；
- **跟踪状态机**：`Idle → Detecting → Tracking → TempLost → reset`，丢失期间用预测外推；
- **TrackerManager**（[manager.hpp](src/fcs/L3_estimation/manager.hpp)）：按 `(ArmorName, ArmorColor)` 路由多目标，输出**所有**跟踪器状态，目标选择交给 L4；
- 系统：`armor_tracker` @250Hz（[tracker_systems.cpp](src/fcs/L3_estimation/tracker_systems.cpp)）。

#### energy_meter/ 与 energy_meter_solver/ — 能量机关状态估计

- 视觉（L2 rune）给出叶片/箭矢观测 → `energy_meter_solver`（[motion_model.hpp](src/fcs/L3_estimation/energy_meter_solver/motion_model.hpp)、[voter.hpp](src/fcs/L3_estimation/energy_meter_solver/voter.hpp)）用**投票（RANSAC 风格）+ 匀速旋转模型**估计神符转速与相位 → `EnergyMeterState`。

#### ldm_naive/ — LDM 的 InEKF

- 状态直接存为 **SE2(3) 群元素**（`group::SEn3<double, 2>`，来自 3dparty/lieplusplus），用李代数**右扰动**做预测：`X̂₊ = X̂ * exp([0, 0, v·dt])`；平铺字段只出现在可视化边界（[types.hpp](src/fcs/L3_estimation/ldm_naive/types.hpp)）。

### 6.4 L4 规划层（`src/fcs/L4_planning/`）

- **Aimer**（[aimer.hpp](src/fcs/L4_planning/aimer/aimer.hpp)）：对每种目标类型（Robot/Outpost/Rune/Ldm）重载 `aim()` —— 目标状态外推预测 → 弹道解算 → fast_tf 组合云台角；支持额外预测延迟（补偿系统延迟）；
- **预测滤波链路**：tracker 状态（位置/速度/yaw/v_yaw）→ 按 `dt = 观测延迟 + 系统延迟 + 弹道飞行时间` 外推（CV + 装甲相位旋转，`predict_aim_point`）→ `TrajectorySolver` 求解射击角；
- **弹道模型**（`src/fcs/core/trajectory/`）：`IdealModel`（真空解析解）/ `LinearDragModel`（线性阻力解析解），求解器 `DirectSolver` / `IterativeSolver` 迭代 pitch 直到命中目标高度（[trajectory_solver.cpp](src/fcs/core/trajectory/solver/trajectory_solver.cpp)）；弹速来自资源 `bullet_speed_data`（裁判系统/MCU 上报）；
- **目标选择**（[armor_target_decider.hpp](src/fcs/L4_planning/aimer/armor_target_decider.hpp)）：`ArmorTargetDecider = variant<Unmanned, Manned>`，打分项含图像中心距离、跟踪状态、弹道飞行时间、云台机动代价、装甲类型，带切换迟滞（`switch_margin`）；
- **输出**：`ControlIntent = variant<TrackCommand, ShotCommand, HoldCommand>`（[control_intent.hpp](src/fcs/L4_planning/control_intent.hpp)）；附带 `SelectedTargetSnapshot`、`TargetSelectionTrace` 供可视化/采集/回放审计。

### 6.5 L5 武器层（`src/fcs/L5_weapon/`）

- **MPC 轨迹优化**（[enhanced/trajectory_optimizer.hpp](src/fcs/L5_weapon/enhanced/trajectory_optimizer.hpp)）：基于 tinympc，双求解器 `DualMpcOsqpSolver` / `DualSmallMpcSolver`，优化云台 yaw/pitch 轨迹并给出角速度/角加速度前馈；
- **WeaponCommand**（[fire_control.hpp](src/fcs/L5_weapon/fire_control.hpp)）：目标角 + 前馈（vel/accel）+ `fire` 指令 + 弹道信息（tof/distance）；MPC 失败时有降级输出并带 `degradation_reason`；
- 系统 @250Hz（[weapon_systems.cpp](src/fcs/L5_weapon/enhanced/weapon_systems.cpp)）。

## 7. 通信协议

### 7.1 云台 MCU 协议（`crates/hardware/at_gimbal/src/talos_gimbal/`）

自定义定长帧，帧头 `0x5A`、帧尾 `0xA5`（[packet.hpp](../crates/hardware/at_gimbal/src/talos_gimbal/packet.hpp)）：

| 帧 | 方向 | 内容 |
| --- | --- | --- |
| `ReceiveImuData` (id 0x01) | MCU→Talos | 己方颜色、弹速、云台 yaw/pitch/roll 及角速度 |
| `ReceiveCapabilitiesData` (id 0x03) | MCU→Talos | 跟随/开符/Quanta 图传开关 |
| `SendVisionData` | Talos→MCU | 开火建议 + 目标 yaw/pitch + 角速度/角加速度前馈 + 距离 |
| `SendQuantaData` (id 0x04) | Talos→MCU | 图传自定义字节块（298 B） |

收发通过 USB（`Stm32Impl`，[stm32.hpp](../crates/hardware/at_gimbal/src/talos_gimbal/stm32.hpp)）或串口（`/dev/ttyS4` @115200，[serial.hpp](../crates/hardware/at_gimbal/src/talos_gimbal/serial.hpp)），演示程序见 `src/quanta_ipc_demo.cpp`。

### 7.2 Quanta 图传（`crates/quanta/`）

HEVC 编码（[hevc_annexb](../crates/quanta/src/quanta/paramset/hevc_annexb.hpp)，FFmpeg/AX650 硬件编码双后端）→ `QuantaPacket` 分片：**5 字节头（seq、frag_idx、frag_total、keyframe）+ MTU 298 B**（[stream_transport.hpp](../crates/quanta/src/quanta/stream_transport.hpp)）→ 通过 MCU `SendQuantaData` 的自定义块透传。带丢帧保护（关键帧保护、队列统计）。

### 7.3 Foxglove 可视化（`src/fcs_visualization/`）

- WebSocket server（默认 `ws://0.0.0.0:8765`）或 MCAP 落盘（`record/foxglove.mcap`），基于 foxglove_cpp（[foxglove_server.hpp](src/fcs_visualization/foxglove_server.hpp)）；
- 各层系统把 debug 数据（检测框、跟踪轨迹、协方差椭圆、弹道、MPC 计划）写入 typed channels；浏览器打开 Foxglove Studio 连接即可在线调试。

### 7.4 Daedalus IPC（`crates/hardware_daedalus/`）

与外部（Rust 模拟器/评测系统）的共享内存协议，`/tmp/talos_ipc_meta` + `/tmp/talos_ipc_image_pool`，magic `0x54414C05`（"TAL05"）：

- 图像池：1440×1080 RGB8 **三重缓冲**（零拷贝，[shm_triple_buffer.hpp](../crates/hardware_daedalus/src/shm_triple_buffer.hpp)）；
- 消息：`ImageMeta` / `PoseMeta` / `GimbalCmd` / `CameraInfo` / 底盘观测 / **`GroundTruthBatch`（真值，用于评估）**，全部 C ABI 兼容 + 缓存行对齐（[shm_layout.hpp](../crates/hardware_daedalus/src/shm_layout.hpp)）。

### 7.5 Chiral 采集端点（`crates/chiral/`）

另一套共享内存端点（`TalosData` / `IncomingData`），`chiral_collector_system` 把 `SelectedTargetSnapshot`/跟踪结果发布出去，供外部采集、记录与回放注入。

## 8. 测试、录制与回放

### 8.1 单元测试

测试位于 `src/fcs/tests/`，随 `build.sh --test` 构建并注册到 ctest：

```bash
./build.sh --test            # 构建 + 运行全部测试
ctest --test-dir build --output-on-failure   # 或直接跑 ctest
```

覆盖：PNP 解算回归（`at_legacy_pnp_solver_test`）、ROI 回读（`at_legacy_readback_roi_test`）、跟踪器管理（`tracker_manager_test`）、LDM 检测/InEKF（`ldm_detector_solver_test`、`ldm_inekf_test`）、aimer（`aimer_phase_test`）、MPC（`trajectory_optimizer_test`）、配置解析等。

### 8.2 录制与回放

- **capturer**：`[capturer] enabled = true` 时按需落盘（含预留空间保护）；
- **chiral**：实时把跟踪结果发布到共享内存供外部采集；
- **talos-replay**：回放录制的 run（速度、循环、起始偏移、图像数限制可选，见 `src/fcs/runtime/replay.hpp`）：

```bash
./build/talos-replay --help
./build/talos-replay <record_path> --speed 1.0 --loop
```

## 9. 标定

`rm_calibration`（`./build.sh --calib` 构建，读取 `calibration.toml`）：

- **内参标定**：棋盘格 / ChArUco / 圆点阵列（`calibration.toml` 的 `[calibration.board]`）；
- **手眼标定**：AX=XB（[handeye_calibrator.hpp](src/fcs/calibration/handeye_calibrator.hpp)）；
- 实现见 `src/fcs/calibration/`（charuco_detector、chessboard_detector、intrinsic_calibrator），标定结果用于 PnP 解算的相机内参。

## 10. 上手建议路线

1. **读**：[README.md](../README.md) → 本文 → [CLAUDE.md](../CLAUDE.md) 的调度器编程模型与规范；
2. **跑**：`./build.sh --test` 确认工具链可用；改 `at_vision.toml` 的 `backend = "Daedalus"` 连模拟器跑通全链路；
3. **看数据流**：沿 §5 主线从 `L2_perception/armor/systems.cpp` 一路读到 `L5_weapon/enhanced/weapon_systems.cpp`；
4. **跟一条消息**：在 `channel_topics.hpp` 挑一个 topic，grep 它的生产者和消费者（IDE 里对 `spmc_mut<T, Topic>` / `spmc<T, Topic>` 做引用查找）；
5. **改一个参数**：在 `config/vision/*.toml` 找对应算法参数（如 tracker 噪声、aimer 延迟），用 Foxglove 观察效果；
6. **写一个 System**：照 CLAUDE.md 的模板在 L4/L5 之间插入一个调试 System，体会 bind/run 与版本触发。

### 关键文件速查

| 想了解 | 看这里 |
| --- | --- |
| 全部通道定义 | [core/channel_topics.hpp](../src/fcs/core/channel_topics.hpp) |
| 领域类型（检测/测量/跟踪输出） | [core/types.hpp](../src/fcs/core/types.hpp) |
| PnP 解算与协方差 | [L2_perception/armor/solver.hpp](../src/fcs/L2_perception/armor/solver.hpp) |
| EKF 与运动模型 | [L3_estimation/tracker/extended_kalman_filter.hpp](../src/fcs/L3_estimation/tracker/extended_kalman_filter.hpp)、[new_motion_model.hpp](../src/fcs/L3_estimation/tracker/new_motion_model.hpp) |
| 数据关联 | [L3_estimation/tracker/new_tracker.hpp](../src/fcs/L3_estimation/tracker/new_tracker.hpp) |
| 瞄准与预测 | [L4_planning/aimer/aimer.cpp](../src/fcs/L4_planning/aimer/aimer.cpp) |
| 弹道模型与求解 | [core/trajectory/solver/trajectory_solver.cpp](../src/fcs/core/trajectory/solver/trajectory_solver.cpp) |
| 目标选择 | [L4_planning/aimer/armor_target_decider.hpp](../src/fcs/L4_planning/aimer/armor_target_decider.hpp) |
| MCU 帧协议 | [at_gimbal packet.hpp](../crates/hardware/at_gimbal/src/talos_gimbal/packet.hpp) |
| 共享内存布局 | [hardware_daedalus shm_layout.hpp](../crates/hardware_daedalus/src/shm_layout.hpp) |
| 启动装配 | [runtime/boot.cpp](../src/fcs/runtime/boot.cpp) |
