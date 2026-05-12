package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;

/**
 * 自动云台控制系统
 * 功能：
 * 1. HOLD 模式：锁定朝前 (HOLD_ANGLE)
 * 2. TRACK 模式：自动跟踪 goal
 *    - 视觉新鲜（{@link VisionBearingTracker#isFresh()}）→ 用 LL 给出的世界系方位角
 *    - 视觉陈旧 → 回退到 odo + atan2
 *    - 目标超出 ±MAX_ANGLE_DEG → 电子锁定（保持当前位置不动）
 * 3. 输出去抖动：角度死区 + ticks 去重
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

    /** pan 目标角变化小于此值时不重新下发，避免高频抖动。 */
    public static double DEADBAND_DEG = 0.3;

    private static final double PAN_P_POS = 15, PAN_P_VEL = 30, PAN_I = 0.01, PAN_F = 0, PAN_D = 0;

    private double HOLD_ANGLE = 0.0;

    // ==========================================
    // 状态定义
    // ==========================================
    public enum Mode {
        HOLD,   // 锁定朝前
        TRACK   // 自动跟踪目标坐标
    }

    private enum TrackState {
        TRACKING,
        LOCKED
    }

    /** 当前一帧 pan 目标角的来源，仅用于遥测。 */
    public enum Source {
        HOLD,
        VISION,
        ODO_FALLBACK
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

    public void setMode(Mode mode) {
        if (this.currentMode != mode) {
            this.currentMode = mode;
            if (mode == Mode.TRACK) {
                trackState = TrackState.TRACKING;
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

        // 1. 视觉测量（无副作用读 LL）
        tracker.update(panEncoderDeg, headingNow, yawRate);

        // 2. 计算原始目标角
        double targetAngleRaw;
        if (currentMode == Mode.HOLD) {
            targetAngleRaw = HOLD_ANGLE;
            currentSource = Source.HOLD;
            isLimitReached = false;
        } else {
            double relativeAngle;
            if (tracker.isFresh()) {
                relativeAngle = normalizeAngle(tracker.getBearingWorldDeg() - headingNow);
                currentSource = Source.VISION;
            } else {
                relativeAngle = computeBearingFromOdo(ppd);
                currentSource = Source.ODO_FALLBACK;
            }
            targetAngleRaw = applyLockingStateMachine(relativeAngle);
            if (Double.isNaN(targetAngleRaw)) return; // 锁定中，不下发
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

    private double computeBearingFromOdo(PinpointDriverData ppd) {
        double dx = targetX - ppd.getRobotX();
        double dy = targetY - ppd.getRobotY();
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        return normalizeAngle(angleToGoal - ppd.getHeadingDegrees());
    }

    /** 哨兵值：表示当前帧处于锁定状态，调用方应直接 return 不下发。 */
    private static final double HOLD_BY_LOCK = Double.NaN;

    private double applyLockingStateMachine(double relativeAngle) {
        boolean inRange = Math.abs(relativeAngle) <= MAX_ANGLE_DEG;
        if (trackState == TrackState.TRACKING) {
            if (inRange) {
                isLimitReached = false;
                return relativeAngle;
            }
            // 进入锁定
            trackState = TrackState.LOCKED;
            lockedSignum = Math.signum(relativeAngle);
            isLimitReached = true;
            return HOLD_BY_LOCK;
        } else {
            // LOCKED
            if (!inRange) {
                isLimitReached = true;
                return HOLD_BY_LOCK;
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
