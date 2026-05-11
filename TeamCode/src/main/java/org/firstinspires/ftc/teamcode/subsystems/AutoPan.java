package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardwares;

/**
 * 自动云台控制系统
 * 功能：
 * 1. HOLD 模式：锁定朝前 (0度)
 * 2. TRACK 模式：自动跟踪场地坐标，含电子锁定逻辑
 *    - 目标在 ±90° 范围内：正常跟踪
 *    - 目标超出范围：电子锁定（保持当前位置不动）
 *    - 目标从同侧回来：直接解锁继续跟踪
 *    - 目标从异侧回来：解锁后云台自然摆向另一侧跟踪
 * 3. 优化的 CAN 总线通信（仅在目标 ticks 变化时发送指令）
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

    private static final double PAN_P_POS = 15, PAN_P_VEL = 30, PAN_I = 0.01, PAN_F = 0, PAN_D = 0;

    private double HOLD_ANGLE = 0.0;

    // ==========================================
    // 状态定义
    // ==========================================
    public enum Mode {
        HOLD,   // 锁定朝前
        TRACK   // 自动跟踪目标坐标
    }

    // TRACK 模式内部状态
    private enum TrackState {
        TRACKING,   // 正常跟踪
        LOCKED      // 目标超出范围，电子锁定
    }

    // ==========================================
    // 成员变量
    // ==========================================
    private final DcMotorEx panMotor;
    private final GoBildaPinpointDriver odo;
    private Mode currentMode = Mode.HOLD;
    private TrackState trackState = TrackState.TRACKING;

    private final double targetX;
    private final double targetY;

    private boolean isLimitReached = false;
    private double currentRawTarget = 0.0;
    // 锁定时记录目标离开的方向：+1 右侧，-1 左侧
    private double lockedSignum = 0.0;

    private int lastTargetTicks = Integer.MIN_VALUE;

    /**
     * @param hardwares 硬件映射
     * @param targetX   目标 X (cm)，场地坐标系
     * @param targetY   目标 Y (cm)，场地坐标系
     */
    public AutoPan(@NonNull Hardwares hardwares, double targetX, double targetY) {
        this.panMotor = hardwares.motors.pan;
        this.odo = hardwares.sensors.odo;
        this.targetX = targetX;
        this.targetY = targetY;
        this.init();
    }

    /**
     * 初始化硬件，重置编码器零点。
     * 确保上电时云台处于正前方。
     */
    public void init() {
        panMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        panMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        panMotor.setPositionPIDFCoefficients(PAN_P_POS);
        panMotor.setVelocityPIDFCoefficients(PAN_P_VEL, PAN_I, PAN_D, PAN_F);
        panMotor.setPower(0);

        lastTargetTicks = 0;
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

    public void setMode(Mode mode) {
        if (this.currentMode != mode) {
            this.currentMode = mode;
            // 切换到 TRACK 时重置锁定状态
            if (mode == Mode.TRACK) {
                trackState = TrackState.TRACKING;
            }
        }
    }

    public void switchMode() {
        if (this.currentMode == Mode.HOLD) {
            setMode(Mode.TRACK);
        } else {
            setMode(Mode.HOLD);
        }
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
        public final double rawTargetAngle;
        public final double currentAngle;
        public final boolean isLimitReached;
        public final double motorPower;

        public TelemetryState(Mode mode, TrackState trackState, double rawTargetAngle,
                              double currentAngle, boolean isLimitReached, double motorPower) {
            this.mode = mode;
            this.trackState = trackState;
            this.rawTargetAngle = rawTargetAngle;
            this.currentAngle = currentAngle;
            this.isLimitReached = isLimitReached;
            this.motorPower = motorPower;
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(java.util.Locale.US,
                    "Mode: %s (%s)\nAngle(Target/Curr): %.1f / %.1f\nLimit Reached: %b\nPower: %.2f",
                    mode, trackState, rawTargetAngle, currentAngle, isLimitReached, motorPower);
        }
    }

    public TelemetryState getTelemetryStatus() {
        double currentPosDeg = panMotor.getCurrentPosition() / PAN_TICKS_PER_DEGREE;
        return new TelemetryState(
                currentMode,
                trackState,
                currentRawTarget,
                currentPosDeg,
                isLimitReached,
                panMotor.getPower()
        );
    }

    // ==========================================
    // 主循环
    // ==========================================
    public void run(PinpointDriverData pinpointDriverData) {
        run(pinpointDriverData, 0);
    }

    /**
     * 在主循环中每帧调用，更新云台目标位置。
     * @param pinpointDriverData 最新里程计数据
     * @param offsetDegree       手动偏置角度（度）
     */
    public void run(PinpointDriverData pinpointDriverData, double offsetDegree) {
        if (pinpointDriverData == null) return;

        double targetAngleRaw;

        if (currentMode == Mode.HOLD) {
            targetAngleRaw = HOLD_ANGLE;
            isLimitReached = false;
        } else {
            // ---- TRACK 模式 ----
            double dx = targetX - pinpointDriverData.getRobotX();
            double dy = targetY - pinpointDriverData.getRobotY();
            double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
            // 目标相对机器人朝向的角度，归一化到 [-180, 180]
            double relativeAngle = normalizeAngle(angleToGoal - pinpointDriverData.getHeadingDegrees());

            boolean inRange = Math.abs(relativeAngle) <= MAX_ANGLE_DEG;

            if (trackState == TrackState.TRACKING) {
                if (inRange) {
                    // 正常跟踪
                    targetAngleRaw = relativeAngle;
                    isLimitReached = false;
                } else {
                    // 目标离开范围 → 电子锁定，保持当前位置
                    trackState = TrackState.LOCKED;
                    lockedSignum = Math.signum(relativeAngle);
                    isLimitReached = true;
                    return; // 不更新电机目标，保持锁定
                }
            } else {
                // ---- LOCKED 状态 ----
                if (!inRange) {
                    // 目标仍在范围外，继续锁定
                    isLimitReached = true;
                    return;
                }

                // 目标回到范围内 → 解锁
                trackState = TrackState.TRACKING;
                isLimitReached = false;
                targetAngleRaw = relativeAngle;
                // 同侧（signum 相同）：云台小幅调整回到目标
                // 异侧（signum 不同）：云台自然从一侧摆向另一侧（约 180° 摆动）
                // 两种情况下均直接命令到目标角度，行为由电机 PIDF 自然完成
            }
        }

        this.currentRawTarget = targetAngleRaw;

        int targetTicks = (int) Math.round((targetAngleRaw + offsetDegree) * PAN_TICKS_PER_DEGREE);

        // 仅在目标变化时发送指令，节省 CAN 带宽
        if (targetTicks != lastTargetTicks) {
            panMotor.setTargetPosition(targetTicks);
            if (panMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                panMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                panMotor.setPower(PAN_POWER);
            }
            lastTargetTicks = targetTicks;
        }
    }

    private double normalizeAngle(double angle) {
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
