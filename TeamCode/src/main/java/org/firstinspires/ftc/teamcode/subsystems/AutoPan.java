package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;

/**
 * 自动云台控制系统（仅 odo 版本，用于 leslie-pan-test 分支测试底盘闭环效果）。
 *
 * <h3>两层状态</h3>
 * <ul>
 *   <li><b>{@link Mode}</b>（HOLD / TRACK）—— 外部控制，由 TeleOp DPAD_UP / Auto setMode() 切换。
 *       本类内部从不自动改 {@code currentMode}，操作手是唯一决定瞄不瞄的入口。</li>
 *   <li><b>{@link TrackState}</b>（TRACKING / LOCKED）—— TRACK 模式的内部子状态，处理物理软限位。
 *       目标超出 ±MAX_ANGLE_DEG 时进入 LOCKED 把 pan 顶到限位侧，目标回到范围内自动解锁。
 *       LOCKED 不会反向影响 Mode，DPAD_UP 仍然能切走。</li>
 * </ul>
 *
 * <h3>TRACK 模式的数据来源</h3>
 * 仅使用 odo：{@code atan2(targetY-robotY, targetX-robotX) - heading}。
 * 视觉闭环已剥离，pan 目标角完全依赖里程计位姿的精度。
 *
 * <h3>输出去抖动</h3>
 * 死区 ({@link #DEADBAND_DEG}) + ticks 去重，避免电机 PIDF 内环 hunting。
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
     */
    public static double DEADBAND_DEG = 0.3;

    private static final double PAN_P_POS = 15, PAN_P_VEL = 30, PAN_I = 0.01, PAN_F = 0, PAN_D = 0;

    private double HOLD_ANGLE = 0.0;

    // ==========================================
    // 状态定义
    // ==========================================
    /** 外部控制的主模式。本类从不自动改 currentMode。 */
    public enum Mode {
        HOLD,   // 锁定朝前 (HOLD_ANGLE)
        TRACK   // 自动跟踪目标坐标，纯 odo 计算
    }

    /** TRACK 模式的子状态，仅用于物理软限位。 */
    private enum TrackState {
        TRACKING,   // 正常跟踪
        LOCKED      // 目标超出 ±MAX_ANGLE_DEG，pan 顶到限位侧等待目标回到范围
    }

    /** 当前一帧 pan 目标角的来源，仅用于遥测/调试。 */
    public enum Source {
        HOLD,   // currentMode == HOLD
        ODO     // currentMode == TRACK，使用 odo 计算 bearing
    }

    // ==========================================
    // 成员变量
    // ==========================================
    private final DcMotorEx panMotor;
    private final GoBildaPinpointDriver odo;

    private Mode currentMode = Mode.HOLD;
    private TrackState trackState = TrackState.TRACKING;
    private Source currentSource = Source.HOLD;

    private final double targetX;
    private final double targetY;

    private boolean isLimitReached = false;
    private double currentRawTarget = 0.0;
    /**
     * 目标进入 LOCKED 状态时记录的方位符号（+1 = 目标在右侧 / -1 = 在左侧）。
     */
    private double lockedSignum = 0.0;

    private int lastTargetTicks = Integer.MIN_VALUE;
    private double lastCommandedAngle = 0.0;

    /**
     * @param hardwares    硬件映射
     * @param targetX      目标 X (cm)，场地坐标系
     * @param targetY      目标 Y (cm)，场地坐标系
     * @param targetTagId  保留参数以兼容现有调用方；本分支视觉已剥离，参数被忽略
     */
    public AutoPan(@NonNull Hardwares hardwares, double targetX, double targetY, int targetTagId) {
        this.panMotor = hardwares.motors.pan;
        this.odo = hardwares.sensors.odo;
        this.targetX = targetX;
        this.targetY = targetY;
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

        odo.recalibrateIMU();
    }

    /**
     * 比赛开始时调用：启动 RUN_TO_POSITION 并锁定 0 度。
     */
    public void setup() {
        panMotor.setTargetPosition(0);
        panMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        panMotor.setPower(PAN_POWER);
        this.currentMode = Mode.HOLD;
    }

    /**
     * 切换主模式。
     */
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
                false, 0.0, 0.0
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

        // 1. 计算原始目标角
        double targetAngleRaw;
        if (currentMode == Mode.HOLD) {
            targetAngleRaw = HOLD_ANGLE;
            currentSource = Source.HOLD;
            isLimitReached = false;
        } else {
            // TRACK 模式：纯 odo 计算 bearing
            double relativeAngle = computeBearingFromOdo(ppd);
            currentSource = Source.ODO;
            targetAngleRaw = applyLockingStateMachine(relativeAngle);
        }

        // 2. 角度死区
        if (Math.abs(normalizeAngle(targetAngleRaw - lastCommandedAngle)) < DEADBAND_DEG) {
            targetAngleRaw = lastCommandedAngle;
        } else {
            lastCommandedAngle = targetAngleRaw;
        }
        currentRawTarget = targetAngleRaw;

        // 3. 下发 ticks（变化才发，节省 CAN）
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
     * 从 odo 位置算出 pan 目标角。odo 漂移会直接转化为这里的角度误差。
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
            trackState = TrackState.LOCKED;
            lockedSignum = Math.signum(relativeAngle);
            isLimitReached = true;
            return lockedSignum * MAX_ANGLE_DEG;
        } else {
            if (!inRange) {
                isLimitReached = true;
                return lockedSignum * MAX_ANGLE_DEG;
            }
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