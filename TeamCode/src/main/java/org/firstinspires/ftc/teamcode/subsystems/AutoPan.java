package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;

/**
 * 自动云台控制系统。
 *
 * <h3>两层状态</h3>
 * <ul>
 *   <li><b>{@link Mode}</b>（HOLD / TRACK）—— 外部控制，由 TeleOp DPAD_UP / Auto setMode() 切换。
 *       本类内部从不自动改 {@code currentMode}，操作手是唯一决定瞄不瞄的入口。</li>
 *   <li><b>{@link TrackState}</b>（TRACKING / LOCKED）—— TRACK 模式的内部子状态，处理物理软限位。
 *       目标超出 ±MAX_ANGLE_DEG 时进入 LOCKED 把 pan 顶到限位侧（给 LL FOV 一个机会捕获到越界 tag），
 *       目标回到范围内自动解锁。LOCKED 不会反向影响 Mode，DPAD_UP 仍然能切走。</li>
 * </ul>
 *
 * <h3>TRACK 模式的数据来源（每帧 run() 里）</h3>
 * <ol>
 *   <li>视觉新鲜（{@link VisionBearingTracker#isFresh()} == true）→ 用 LL 给出的世界系方位角
 *       减当前 heading；这是首选，因为 LL 的角度测量精度（亚度）远高于 odo 漂移后的精度。</li>
 *   <li>视觉陈旧 → 退回 odo {@code atan2(targetY-robotY, targetX-robotX) - heading}；
 *       odo 位置可能因为撞击离地漂过，但短暂遮挡（&lt; VISION_HOLD_MS）情况下 odo 仍是合理近似。</li>
 * </ol>
 *
 * <h3>输出去抖动</h3>
 * 死区 ({@link #DEADBAND_DEG}) + ticks 去重，避免电机 PIDF 内环 hunting。
 * 两者作用环节不同：死区滤掉控制目标的微小变化，ticks 去重消除量化误差导致的重复下发。
 */
public class AutoPan {

    // ==========================================
    // 常量配置
    // ==========================================
    public static final double MOTOR_TICKS_PER_REV = 145.6; // 28 tick × 5.2
    public static final double PAN_REV_PER_MOTOR_REVS = 100.0 / 20.0;
    public static final double PAN_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * PAN_REV_PER_MOTOR_REVS) / 360.0;

    public static final double PAN_POWER = 1;
    public static final double MAX_ANGLE_DEG = 90.0; // 物理线缆限位

    /**
     * pan 目标角变化小于此值时不重新下发，避免电机 PIDF 内环 hunting。
     *
     * 注意：这看起来跟 {@code lastTargetTicks != targetTicks} 的 ticks 去重重叠
     * （1 tick ≈ 0.495°，比 0.3° 还大），但实际上两者作用不同：
     * - ticks 去重消除量化抖动（角度小变化但 tick 相同）
     * - 死区在角度本身就稳定但浮点尾数不停跳时止血（防止 PIDF 反复重启目标）
     * GoBilda 电机带齿轮减速后惯量大、响应慢，多一层保护值得。
     */
    public static double DEADBAND_DEG = 0.3;

    private static final double PAN_P_POS = 15, PAN_P_VEL = 30, PAN_I = 0.01, PAN_F = 0, PAN_D = 0;

    private double HOLD_ANGLE = 0.0;

    // ==========================================
    // 状态定义
    // ==========================================
    /** 外部控制的主模式。本类从不自动改 currentMode。 */
    public enum Mode {
        HOLD,   // 锁定朝前 (HOLD_ANGLE)，视觉数据忽略
        TRACK   // 自动跟踪目标坐标，视觉优先、odo 兜底
    }

    /** TRACK 模式的子状态，仅用于物理软限位。 */
    private enum TrackState {
        TRACKING,   // 正常跟踪
        LOCKED      // 目标超出 ±MAX_ANGLE_DEG，pan 顶到限位侧等待目标回到范围
    }

    /** 当前一帧 pan 目标角的来源，仅用于遥测/调试，操作手可据此判断 LL 是否工作。 */
    public enum Source {
        HOLD,           // currentMode == HOLD
        VISION,         // currentMode == TRACK 且 tracker.isFresh()
        ODO_FALLBACK    // currentMode == TRACK 但视觉陈旧
    }

    // ==========================================
    // 成员变量
    // ==========================================
    private final DcMotorEx panMotor;
    private final GoBildaPinpointDriver odo;
    private final VisionBearingTracker tracker;

    private Mode currentMode = Mode.HOLD;
    private TrackState trackState = TrackState.TRACKING;
    private Source currentSource = Source.HOLD;

    private final double targetX;
    private final double targetY;

    private boolean isLimitReached = false;
    private double currentRawTarget = 0.0;
    /**
     * 目标进入 LOCKED 状态时记录的方位符号（+1 = 目标在右侧 / -1 = 在左侧）。
     * LOCKED 期间 pan 物理上被顶到 {@code lockedSignum * MAX_ANGLE_DEG}，
     * 让 LL 在 FOV 内最大化捕获越界 tag 的机会。
     */
    private double lockedSignum = 0.0;

    private int lastTargetTicks = Integer.MIN_VALUE;
    private double lastCommandedAngle = 0.0;

    /**
     * @param hardwares    硬件映射
     * @param targetX      目标 X (cm)，场地坐标系
     * @param targetY      目标 Y (cm)，场地坐标系
     * @param targetTagId  对应 goal 的 AprilTag ID；&lt; 0 禁用视觉，纯 odo 跟踪
     */
    public AutoPan(@NonNull Hardwares hardwares, double targetX, double targetY, int targetTagId) {
        this.panMotor = hardwares.motors.pan;
        this.odo = hardwares.sensors.odo;
        this.targetX = targetX;
        this.targetY = targetY;
        this.tracker = new VisionBearingTracker(hardwares.sensors.ll, targetTagId);
        this.init();
    }

    /** 向后兼容：不启用视觉。 */
    public AutoPan(@NonNull Hardwares hardwares, double targetX, double targetY) {
        this(hardwares, targetX, targetY, FieldConstants.TAG_ID_NONE);
    }

    /**
     * 初始化硬件，重置编码器零点。确保上电时云台处于正前方。
     */
    public void init() {
        panMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        panMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        panMotor.setPositionPIDFCoefficients(PAN_P_POS);
        panMotor.setVelocityPIDFCoefficients(PAN_P_VEL, PAN_I, PAN_D, PAN_F);
        panMotor.setPower(0);

        lastTargetTicks = 0;
        lastCommandedAngle = 0.0;
        trackState = TrackState.TRACKING;
        tracker.reset();

        odo.recalibrateIMU();
    }

    /**
     * 比赛开始时调用：启动 RUN_TO_POSITION 并锁定 0 度，同时启动视觉。
     */
    public void setup() {
        panMotor.setTargetPosition(0);
        panMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        panMotor.setPower(PAN_POWER);
        this.currentMode = Mode.HOLD;
        tracker.start();
    }

    /**
     * 切换主模式。进入 TRACK 时清空视觉历史，避免 HOLD 期间 pan 转向其他方向
     * 偶然扫到的 tag 留下污染的 smoothedBearing，导致 TRACK 一开始就指向错误位置。
     */
    public void setMode(Mode mode) {
        if (this.currentMode != mode) {
            this.currentMode = mode;
            if (mode == Mode.TRACK) {
                trackState = TrackState.TRACKING;
                tracker.reset();
            }
        }
    }

    public void switchMode() {
        setMode(currentMode == Mode.HOLD ? Mode.TRACK : Mode.HOLD);
    }

    public Mode getMode() {
        return currentMode;
    }

    public VisionBearingTracker getTracker() {
        return tracker;
    }

    // ==========================================
    // 遥测
    // ==========================================
    public static class TelemetryState {
        public final Mode mode;
        public final TrackState trackState;
        public final Source source;
        public final double rawTargetAngle;
        public final double currentAngle;
        public final boolean isLimitReached;
        public final double motorPower;
        public final boolean visionFresh;
        public final double lastTxDeg;
        public final double smoothedBearingWorldDeg;

        public TelemetryState(Mode mode, TrackState trackState, Source source,
                              double rawTargetAngle, double currentAngle,
                              boolean isLimitReached, double motorPower,
                              boolean visionFresh, double lastTxDeg, double smoothedBearingWorldDeg) {
            this.mode = mode;
            this.trackState = trackState;
            this.source = source;
            this.rawTargetAngle = rawTargetAngle;
            this.currentAngle = currentAngle;
            this.isLimitReached = isLimitReached;
            this.motorPower = motorPower;
            this.visionFresh = visionFresh;
            this.lastTxDeg = lastTxDeg;
            this.smoothedBearingWorldDeg = smoothedBearingWorldDeg;
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(java.util.Locale.US,
                    "Mode: %s (%s, src=%s)\nAngle(Target/Curr): %.1f / %.1f\nLimit: %b\nPower: %.2f\nVision: fresh=%b tx=%.2f bearing=%.1f",
                    mode, trackState, source, rawTargetAngle, currentAngle, isLimitReached, motorPower,
                    visionFresh, lastTxDeg, smoothedBearingWorldDeg);
        }
    }

    public TelemetryState getTelemetryStatus() {
        double currentPosDeg = panMotor.getCurrentPosition() / PAN_TICKS_PER_DEGREE;
        return new TelemetryState(
                currentMode, trackState, currentSource,
                currentRawTarget, currentPosDeg,
                isLimitReached, panMotor.getPower(),
                tracker.isFresh(), tracker.getLastTxDeg(), tracker.getBearingWorldDeg()
        );
    }

    // ==========================================
    // 主循环
    // ==========================================
    public void run(PinpointDriverData pinpointDriverData) {
        run(pinpointDriverData, 0);
    }

    /**
     * 每帧调用，更新云台目标位置。
     *
     * @param ppd          最新里程计数据
     * @param offsetDegree 手动偏置角度（度）
     */
    public void run(PinpointDriverData ppd, double offsetDegree) {
        if (ppd == null) return;

        double headingNow = ppd.getHeadingDegrees();
        double yawRate = ppd.getYawRate();
        double panEncoderDeg = panMotor.getCurrentPosition() / PAN_TICKS_PER_DEGREE;
        // getVelocity() 是带符号的真实编码器速度（ticks/s），跟 getCurrentPosition()
        // 共用同一套方向约定。tracker 用它把 panEncoderDeg 外推回 LL 帧捕获时刻，
        // 避免 pan 自己快速旋转时（如刚切 TRACK 大幅扫向目标）bearing 计算出现 6°+ 偏差。
        double panRateDegPerSec = panMotor.getVelocity() / PAN_TICKS_PER_DEGREE;

        // 1. 视觉测量
        //    每帧都跑，即使 HOLD 模式也跑——目的是让 TRACK 模式刚切换过来时
        //    smoothedBearing 已经有最新数据可用（虽然 setMode 会 reset，但下一帧 LL
        //    一进来就能立刻 fresh，比等 20ms 一帧再 fresh 更顺）。
        tracker.update(panEncoderDeg, panRateDegPerSec, headingNow, yawRate);

        // 2. 计算原始目标角
        double targetAngleRaw;
        if (currentMode == Mode.HOLD) {
            // HOLD 完全无视视觉，pan 锁在外部设置的 HOLD_ANGLE
            targetAngleRaw = HOLD_ANGLE;
            currentSource = Source.HOLD;
            isLimitReached = false;
        } else {
            // TRACK 模式：视觉优先 → odo 兜底
            double relativeAngle;
            if (tracker.isFresh()) {
                // 世界系 bearing 减当前 heading = 机器人系下的 pan 目标角
                relativeAngle = normalizeAngle(tracker.getBearingWorldDeg() - headingNow);
                currentSource = Source.VISION;
            } else {
                // 视觉超过保鲜期（VISION_HOLD_MS），LL 可能被挡或硬件未配
                relativeAngle = computeBearingFromOdo(ppd);
                currentSource = Source.ODO_FALLBACK;
            }
            // LOCKED 状态下也会返回有效角度（被夹在 ±MAX_ANGLE_DEG），
            // 走正常的死区 + tick 下发流程，pan 物理上停在限位
            targetAngleRaw = applyLockingStateMachine(relativeAngle);
        }

        // 3. 角度死区
        if (Math.abs(normalizeAngle(targetAngleRaw - lastCommandedAngle)) < DEADBAND_DEG) {
            targetAngleRaw = lastCommandedAngle;
        } else {
            lastCommandedAngle = targetAngleRaw;
        }
        currentRawTarget = targetAngleRaw;

        // 4. 下发 ticks（变化才发，节省 CAN）
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

    /**
     * 视觉不可用时的兜底方案：从 odo 位置算出 pan 目标角。
     * odo 漂移会直接转化为这里的角度误差——这是本系统当初引入视觉的根本动因。
     * 兜底逻辑只在 VISION_HOLD_MS 这段保鲜期之外才用，正常工作时几乎不走这条路径。
     */
    private double computeBearingFromOdo(PinpointDriverData ppd) {
        double dx = targetX - ppd.getRobotX();
        double dy = targetY - ppd.getRobotY();
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        return normalizeAngle(angleToGoal - ppd.getHeadingDegrees());
    }

    /**
     * 物理软限位状态机：pan 因线缆走线限制只能在 ±MAX_ANGLE_DEG 内转。
     * 超出范围时把 pan 顶到限位侧不动，等目标自然回到范围。
     * 关键：把 pan 推到限位（而不是冻在原地）能让 LL 在 FOV 内捕获到稍微越界的 tag，
     * 视觉一旦更新 tracker，下一帧相对角度就可能回到 ±90° 内，自动解锁。
     *
     * 注意：这是 TRACK 模式的内部状态，跟外部的 HOLD/TRACK Mode 互相独立——
     * LOCKED 状态下 currentMode 仍然是 TRACK，操作手按 DPAD_UP 仍能切走。
     */
    private double applyLockingStateMachine(double relativeAngle) {
        boolean inRange = Math.abs(relativeAngle) <= MAX_ANGLE_DEG;
        if (trackState == TrackState.TRACKING) {
            if (inRange) {
                isLimitReached = false;
                return relativeAngle;
            }
            // 进入锁定：把 pan 推到限位侧，让 LL 在 FOV 内最大化捕获 tag 的机会。
            // 真实目标在 ±90°~±121° 内（限位 + LL HFOV ±31°）时视觉仍可能锁定，
            // 一旦 tracker 看到 tag，下一帧 relativeAngle 用视觉算就可能回到 ±90° 内自动解锁。
            trackState = TrackState.LOCKED;
            lockedSignum = Math.signum(relativeAngle);
            isLimitReached = true;
            return lockedSignum * MAX_ANGLE_DEG;
        } else {
            // LOCKED：继续把 pan 顶在限位处
            if (!inRange) {
                isLimitReached = true;
                return lockedSignum * MAX_ANGLE_DEG;
            }
            // 回到范围内 → 解锁
            trackState = TrackState.TRACKING;
            isLimitReached = false;
            return relativeAngle;
        }
    }

    private static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public void setPanMotorPIDF(double pPos, double pVel, double i, double d, double f) {
        panMotor.setPositionPIDFCoefficients(pPos);
        panMotor.setVelocityPIDFCoefficients(pVel, i, d, f);
    }

    public void setHoldAngle(double holdAngle) {
        HOLD_ANGLE = holdAngle;
    }

    public boolean isPanBusy() {
        return panMotor.isBusy();
    }
}
