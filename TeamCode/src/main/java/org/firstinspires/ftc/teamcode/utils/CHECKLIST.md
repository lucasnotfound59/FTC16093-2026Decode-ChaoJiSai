# Pan 视觉系统上场前 Checklist

新车装好后、第一次跑 TeleOp 之前依次过一遍。打勾的项目都做完才能信视觉数据。

## 〇、视觉总开关

- **文件**：[../subsystems/VisionBearingTracker.java](../subsystems/VisionBearingTracker.java) 的 `USE_LIMELIGHT`
- 设为 `false` 时整套视觉链路被绕过，AutoPan 永远走纯 odo（跟视觉接入前的行为一致）
- 用途：装车初期 / 视觉故障排查 / AB 对比测试
- 上场前确认是 `true`，否则下面所有标定都是白做

## 一、必做（不做视觉根本不工作）

### 1. AprilTag ID 写实际值
- **文件**：[FieldConstants.java](FieldConstants.java)
- **当前占位**：`TAG_ID_BLUE_GOAL = 20`、`TAG_ID_RED_GOAL = 24`
- **改成**：Decode 2026 官方手册里的实际 ID
- 怎么验证：装好 LL 后看 Web UI 的 fiducial 列表，对照手册

### 2. 机器人配置加 Limelight 设备
- **位置**：FTC RC app → Configure Robot
- **设备名**：`"limelight"`（小写，跟 [Hardwares.java](../Hardwares.java) 里的 `hardwareMap.get(..., "limelight")` 一致）
- **设备类**：Limelight3A
- 怎么验证：OpMode init 不报 `hardware not found`，telemetry 里 `vision fresh` 能变 true

### 3. LL 烧录的 pipeline 7 配置正确
- **位置**：LL Web UI → Pipeline 7
- **类型**：AprilTag
- **Family**：跟 Decode 2026 官方一致（36h11）
- **启用的 tag ID**：包含两个 goal tag 的 ID
- 怎么验证：把 tag 拿到 LL 前面，Web UI 能看到 fiducial 检测框

### 4. LL_PAN_OFFSET_DEG 标定
- **文件**：[../subsystems/VisionBearingTracker.java](../subsystems/VisionBearingTracker.java) 的 `LL_PAN_OFFSET_DEG`
- **默认值**：0.0（假设光轴完美对齐，几乎不可能）
- **标定流程**：
  1. 机械上把 pan 对准机器人正前方（直角尺或激光）
  2. 跑一次任意 OpMode，让 `STOP_AND_RESET_ENCODER` 把 pan_encoder = 0
  3. 在机器人正前方 ~3m 处放一个 AprilTag
  4. 看 LL 报告的 tx（Dashboard 或 LL Web UI）
  5. 把 `LL_PAN_OFFSET_DEG` 改成那个 tx 值
- 怎么验证：标定后 pan 在 0° 时 LL 报 tx ≈ 0
- 任何"系统性瞄准偏差"（每次都偏同一个方向）先怀疑这一步没做

## 二、强烈建议（不做会很难调）

### 5. Pan PIDF 调参
- **文件**：[../subsystems/AutoPan.java](../subsystems/AutoPan.java) 的 `PAN_P_POS / PAN_P_VEL / PAN_I / PAN_F / PAN_D`
- **当前**：`P_POS=15, P_VEL=30, I=0.01, F=0, D=0`（F 和 D 都是 0，欠调）
- **调法**：pan 单独 setMode(HOLD) + setHoldAngle(45) 做 step 输入，看响应曲线
- **目标**：无超调、< 200ms 到位、不 hunting

## 三、可选（看实战表现再决定）

### 6. 视觉调参（都在 [VisionBearingTracker.java](../subsystems/VisionBearingTracker.java)，`public static` 可 Dashboard 改）
| 常量 | 默认 | 调高 | 调低 |
|---|---|---|---|
| `VISION_HOLD_MS` | 80 | 遮挡频繁时多撑几帧 | odo 兜底立即生效 |
| `LOWPASS_ALPHA` | 0.4 | 响应快但抖 | 平滑但滞后 |
| `STALENESS_MAX_MS` | 100 | 接受更陈旧帧 | 严格丢帧 |

### 7. AutoPan 调参
| 常量 | 默认 | 备注 |
|---|---|---|
| `DEADBAND_DEG` | 0.3 | pan 抖就调大；几乎不用动 |
| `MAX_ANGLE_DEG` | 90 | 新车线缆走线如果限位不是 90°，改成实际限位 |

## 四、跑通验证流程

按这个顺序测，每步通过再下一步：

1. **静态验证 tx**：机器人不动，pan 不动 (HOLD)，把 tag 放在已知方位角，看 `pan_encoder + tx + heading` 等不等于真实方位角（误差 < 2°）
2. **静态闭环**：TRACK 模式，机器人不动，手转 pan 偏离 tag 30°，看 pan 能不能稳定收敛到 |tx| < 0.5° 且不震荡
3. **慢转跟踪**：操作手缓慢转 yaw，pan 反向补偿持续锁定 tag
4. **遮挡测试**：手挡 LL 1 秒，pan 应在前 80ms 保持指向，之后 source 切换到 ODO_FALLBACK
5. **撞击注入**：Dashboard 手动 `odo.setPosition(...)` 给 30cm 偏移，pan 角度**不应变化**（视觉源对 odo 漂移免疫）—— 这是这套系统最直接的价值证明
6. **完整射门**：操作手在 shooting 点用 Y/A/B/X 4 档位射，记命中率
