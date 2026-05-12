# Pan 视觉闭环瞄准方案（lucas-dev-pan）

## 目标

让云台 (pan) 直接用 Limelight 3A 的 AprilTag 横向偏移 `tx` 来实时锁定 goal 方向，绕开 odo 位置漂移对瞄准的影响。距离继续由 odo + 操作手按键档位决定。

不实现：odo 位置修正、botpose、两 tag 三角测量。

## 现状参考

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/AutoPan.java` —— 当前 pan 控制，每帧从 odo 算 `atan2(dy, dx) - heading` 作为目标角，RUN_TO_POSITION + PIDF 跟踪。
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Hardwares.java` —— 硬件注册，需要加 Limelight3A。
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/PinpointDriverData.java` —— 提供 `getHeadingDegrees()` 和 `getYawRate()`（度/秒），yaw rate 用于补偿 LL 延迟。
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleops/TeleOpBase.java` —— 每帧调用 `autoPan.run(pinpointDriverData, panOffset)`。
- 样例：`FtcRobotController/external/samples/SensorLimelight3A.java`。

## 总体设计

`AutoPan.run()` 内部新增一个"视觉融合层"：

1. 每帧尝试从 LL 拿一帧新数据。
2. 检出到正确 ID 的 fiducial 时，把 `tx` 翻译成 **世界系绝对方位角**（关键，避免自激震荡）并做一阶 lowpass 滤波。
3. 控制层：
   - 视觉数据"新鲜"（最近 N 毫秒内有有效帧）→ 用视觉测得的世界系方位角换算 pan 目标。
   - 视觉数据陈旧 → 退回现有 odo atan2 逻辑。
4. 加死区、tick 去抖动、yaw-rate 延迟补偿。
5. `HOLD` 模式保持原样不变（视觉只作用在 `TRACK` 模式）。

## 核心公式

定义"goal tag 在世界系的方位角"为一个外部不变量：

```
bearing_world = pan_encoder_deg + tx + heading_at_frame
```

其中：
- `pan_encoder_deg = panMotor.getCurrentPosition() / PAN_TICKS_PER_DEGREE`
- `tx` 来自 LL（度），右侧为正
- `heading_at_frame = heading_now - yaw_rate_deg_per_s * (capture_latency_s)` —— 用陀螺仪外推回 LL 帧捕获时刻

控制目标：

```
pan_target_deg = normalizeAngle(bearing_world_smoothed - heading_now)
```

这是 pan 在机器人系下的目标角度，可以直接送进现有的 ticks 转换 + RUN_TO_POSITION 流程。

## 详细实现步骤

### 1. Hardwares.java 加 Limelight

```java
// Sensors 内类
public Limelight3A ll;

// 构造器内
ll = hardwareMap.get(Limelight3A.class, "limelight");
ll.setPollRateHz(50);
ll.pipelineSwitch(0);  // 0 号 pipeline 配为 AprilTag 检测
ll.start();
```

import：`com.qualcomm.hardware.limelightvision.Limelight3A`、`LLResult`、`LLResultTypes`。

Limelight 配置端（Web UI）：
- Pipeline 0 → AprilTag (Family: 36h11，FTC 2026 默认)
- 启用所有相关 tag ID
- LL 装在 pan 上，光轴尽量平行于 pan 零度方向，光心尽量靠近 pan 旋转轴中心

### 2. FieldConstants.java（新建）

```java
package org.firstinspires.ftc.teamcode.utils;

public final class FieldConstants {
    private FieldConstants() {}

    // TODO: 实际 ID 待手册确认
    public static final int TAG_ID_BLUE_GOAL = 20;
    public static final int TAG_ID_RED_GOAL  = 24;
}
```

### 3. AutoPan.java 改造

**新增字段**：

```java
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

// 调参常量
private static final double VISION_HOLD_MS = 200.0;   // 视觉数据保鲜期
private static final double LOWPASS_ALPHA = 0.4;      // bearing 滤波系数
private static final double DEADBAND_DEG = 0.3;       // pan 目标变化死区
private static final double STALENESS_MAX_MS = 100.0; // 单帧最大可接受延迟

private final Limelight3A ll;
private final int targetTagId;

// 视觉状态
private double smoothedBearingWorld = 0.0;
private boolean hasSmoothed = false;
private long lastValidNanos = 0L;
private double lastCommandedAngle = 0.0;
```

**构造器改造**：
```java
public AutoPan(@NonNull Hardwares hardwares, double targetX, double targetY, int targetTagId) {
    this.panMotor = hardwares.motors.pan;
    this.odo = hardwares.sensors.odo;
    this.ll = hardwares.sensors.ll;
    this.targetTagId = targetTagId;
    this.targetX = targetX;
    this.targetY = targetY;
    init();
}
```

留一个回退构造器 `(hardwares, x, y)`，传一个 `-1` 当作"不用视觉"的哨兵，方便单元测试或者降级。

**核心 run 方法**（替换 TRACK 模式那段）：

```java
public void run(PinpointDriverData ppd, double offsetDegree) {
    if (ppd == null) return;

    double headingNow = ppd.getHeadingDegrees();
    double yawRate = ppd.getYawRate();  // 度/秒

    // === 1. 视觉测量更新 ===
    boolean visionFresh = updateVisionBearing(headingNow, yawRate);

    double targetAngleRaw;

    if (currentMode == Mode.HOLD) {
        targetAngleRaw = HOLD_ANGLE;
        isLimitReached = false;
    } else {
        // TRACK 模式
        if (visionFresh) {
            // 视觉来源
            targetAngleRaw = normalizeAngle(smoothedBearingWorld - headingNow);
        } else {
            // odo 回退（现有逻辑）
            targetAngleRaw = computeBearingFromOdo(ppd);
        }

        // 物理限位（同现有）
        if (Math.abs(targetAngleRaw) > MAX_ANGLE_DEG) {
            // 复用现有锁定状态机
            handleOutOfRange(targetAngleRaw);
            return;
        }
        trackState = TrackState.TRACKING;
        isLimitReached = false;
    }

    // === 2. 死区 ===
    if (Math.abs(targetAngleRaw - lastCommandedAngle) < DEADBAND_DEG) {
        targetAngleRaw = lastCommandedAngle;
    } else {
        lastCommandedAngle = targetAngleRaw;
    }

    currentRawTarget = targetAngleRaw;

    // === 3. 下发 ticks（同现有）===
    int targetTicks = (int) Math.round((targetAngleRaw + offsetDegree) * PAN_TICKS_PER_DEGREE);
    if (targetTicks != lastTargetTicks) {
        panMotor.setTargetPosition(targetTicks);
        if (panMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            panMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            panMotor.setPower(PAN_POWER);
        }
        lastTargetTicks = targetTicks;
    }
}
```

**视觉更新**：

```java
private boolean updateVisionBearing(double headingNow, double yawRate) {
    if (ll == null || targetTagId < 0) return isVisionFresh();

    LLResult result = ll.getLatestResult();
    if (result == null || !result.isValid()) return isVisionFresh();

    // 延迟（毫秒）
    double captureLatencyMs = result.getCaptureLatency() + result.getTargetingLatency();
    if (captureLatencyMs > STALENESS_MAX_MS) return isVisionFresh();

    for (FiducialResult fr : result.getFiducialResults()) {
        if (fr.getFiducialId() != targetTagId) continue;

        double tx = fr.getTargetXDegrees();
        double panNow = panMotor.getCurrentPosition() / PAN_TICKS_PER_DEGREE;
        double headingAtFrame = headingNow - yawRate * (captureLatencyMs / 1000.0);

        double bearingWorld = panNow + tx + headingAtFrame;

        if (!hasSmoothed) {
            smoothedBearingWorld = bearingWorld;
            hasSmoothed = true;
        } else {
            // 注意：方位角差也要 normalize，避免 ±180 跨越
            double delta = normalizeAngle(bearingWorld - smoothedBearingWorld);
            smoothedBearingWorld = normalizeAngle(smoothedBearingWorld + LOWPASS_ALPHA * delta);
        }
        lastValidNanos = System.nanoTime();
        return true;
    }
    return isVisionFresh();
}

private boolean isVisionFresh() {
    if (lastValidNanos == 0) return false;
    return (System.nanoTime() - lastValidNanos) / 1e6 < VISION_HOLD_MS;
}
```

**说明**：
- 滤波在"角度差"上做，避免 359→1 这种跨越被滤波器拉到 180。
- 没有新有效帧的情况下，`isVisionFresh()` 依然返回 true（在保鲜期内），pan 继续指向最后一次平滑过的 bearing。
- 超过保鲜期 → `false` → 退回 odo 逻辑。

### 4. TeleOpBase / Auto 调用方

`AutoPan` 构造器签名加一个 `targetTagId`。各 alliance-specific 子类传入对应 ID：

- `TeleOpBlueBottom`、`BottomAutoBlue`、`TopAutoBlue` → `FieldConstants.TAG_ID_BLUE_GOAL`
- 对应 Red 子类 → `TAG_ID_RED_GOAL`

外部循环不需要改动。

### 5. 遥测

`AutoPan.TelemetryState` 加几个字段方便现场调试：
- `boolean visionFresh`
- `double rawTxIfSeen`（最近一次的 tx，看不到 tag 时设 NaN）
- `double smoothedBearingWorld`
- `String source`（"VISION" / "ODO_FALLBACK" / "HOLD"）

在 `TeleOpBase.run()` 的 telemetry 里加 3 行显示这些。

## 调参顺序（按这个顺序在 dashboard 里 tune）

1. **零参数验证 tx**
   pan 在 0 度，机器人静止，把 tag 放在已知方位角（用卷尺和量角器）。读 `pan_encoder + tx + heading`，应等于 tag 的世界方位角。误差 > 2° → 安装/标定问题，先解决。

2. **静态闭环收敛**
   机器人不动，启用 TRACK。手转 pan 偏离 tag 30°，看 pan 是否平稳收敛到 |tx| < 0.5° 且不震荡。
   - 震荡 → 降低 `LOWPASS_ALPHA`（如 0.4 → 0.2）或增大 `DEADBAND_DEG`
   - 收敛慢 → 提高 `LOWPASS_ALPHA`（如 0.4 → 0.6）

3. **慢速旋转跟踪**
   操作手用最右摇杆缓慢转 yaw，pan 应反向补偿持续锁定 tag。如果有滞后，确认 yaw rate 延迟补偿在工作（看 `headingAtFrame` 和 `headingNow` 差值，应跟实际 yaw rate 一致）。

4. **遮挡回退**
   用手挡住 LL 1 秒：pan 应在前 200ms 保持指向，之后切换 source 为 ODO_FALLBACK，pan 角度按 odo 计算（如果 odo 此刻是对的，pan 应几乎不动）。松手立刻切回 VISION。

5. **撞击注入测试**
   FTCDashboard 里手动调用 `odo.setPosition(...)` 给一个 30cm 偏移。pan 角度不应变化（视觉源对 odo 漂移免疫）。这是这个方案最直接的价值证明。

6. **完整射门**
   在 shooting 点用 Y/A/B/X 4 个档位射，记命中率。对照只用 odo 的命中率。

## 待你确认 / 我不知道的

- **AprilTag ID**：Decode 2026 两个 goal 的实际 tag ID 我不知道，placeholder 是 20 / 24，按官方手册确认。
- **AprilTag family**：FTC 2026 用 36h11 还是 16h5（决定 Pipeline 配置）。
- **LL 配置名**：`hardwareMap.get(Limelight3A.class, "limelight")` 这里的 "limelight" 是 robot config 里的名字，确认一下。
- **shooter 结构件是否会遮挡 LL 视线**：装上之后实测看 LL 实时画面，pan 在 ±60° 范围里 LL 视野应该一直清晰。
- **LL 安装相对 pan 的姿态**：理想是光轴跟 pan 零度方向**严格平行**，光心在旋转轴正上方。装的时候用方块靠齐。如果有不可消除的小偏置，写成常量加到 `bearingWorld` 计算里。

## 不在本方案里的（避免 scope creep）

- 任何修正 odo 位置的逻辑
- LL botpose / MegaTag 调用
- 两 tag 测量
- 距离估计（继续走档位按键 + odo）
- Auto 路径里插入额外动作

如果上述 6 步调通后还有命中率 gap，再考虑下一阶段。
