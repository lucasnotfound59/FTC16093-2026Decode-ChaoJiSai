package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

/**
 * 视觉方位角跟踪器：封装 Limelight 3A 的读取、ID 过滤、延迟补偿、滤波和保鲜期判断。
 *
 * 输出"目标在世界系下的方位角"（度），是一个不变量——只跟 tag 真实位置有关，
 * 跟 pan 当前角度无关。AutoPan 拿到这个值后减去当前 heading 就是 pan 目标角。
 *
 * 在世界系闭环（而不是"pan_target += tx"）是避免自激震荡的关键：
 * LL 有 20–30ms 延迟，pan 在帧捕获到帧使用之间会动，机器人 heading 也会变。
 * 把测量翻译成世界系常量后，下游想用的时候按当前 heading 转回机器人系就行。
 */
public class VisionBearingTracker {

    // ==========================================
    // 调参（public 方便从 Dashboard 改）
    // ==========================================
    /** 视觉数据保鲜期（毫秒）：超过这个时间没有有效帧，isFresh() 返回 false。 */
    public static double VISION_HOLD_MS = 200.0;
    /** 一阶 lowpass 系数，0–1。越大响应越快、越抖；越小越平滑、越滞后。 */
    public static double LOWPASS_ALPHA = 0.4;
    /** 单帧最大可接受延迟（毫秒），超出则丢弃。 */
    public static double STALENESS_MAX_MS = 100.0;

    // ==========================================
    // 成员
    // ==========================================
    private final Limelight3A ll;
    private final int targetTagId;

    private double smoothedBearingWorldDeg = 0.0;
    private boolean hasSmoothed = false;
    private long lastValidNanos = 0L;
    private double lastTxDeg = Double.NaN;

    /**
     * @param ll          Limelight3A 实例，null 表示无视觉（始终返回 isFresh=false）
     * @param targetTagId 要跟踪的 AprilTag ID，&lt; 0 表示禁用
     */
    public VisionBearingTracker(Limelight3A ll, int targetTagId) {
        this.ll = ll;
        this.targetTagId = targetTagId;
    }

    public void start() {
        if (ll != null) {
            ll.setPollRateHz(50);
            ll.start();
        }
    }

    public void stop() {
        if (ll != null) ll.stop();
    }

    public void reset() {
        hasSmoothed = false;
        lastValidNanos = 0L;
        lastTxDeg = Double.NaN;
    }

    /**
     * 每帧调用。读取 LL 最新一帧，找到目标 tag 后更新世界系方位角。
     *
     * @param panEncoderDeg     当前 pan 编码器角度（度）
     * @param headingDegNow     当前机器人 heading（度），来自 odo 陀螺仪
     * @param yawRateDegPerSec  当前 yaw 角速度（度/秒），用于补偿 LL 延迟期间的旋转
     */
    public void update(double panEncoderDeg, double headingDegNow, double yawRateDegPerSec) {
        if (ll == null || targetTagId < 0) return;

        LLResult result = ll.getLatestResult();
        if (result == null || !result.isValid()) return;

        double captureLatencyMs = result.getCaptureLatency() + result.getTargetingLatency();
        if (captureLatencyMs > STALENESS_MAX_MS) return;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return;

        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() != targetTagId) continue;

            double tx = fr.getTargetXDegrees();
            double headingAtFrame = headingDegNow - yawRateDegPerSec * (captureLatencyMs / 1000.0);
            double bearingWorld = panEncoderDeg + tx + headingAtFrame;

            if (!hasSmoothed) {
                smoothedBearingWorldDeg = bearingWorld;
                hasSmoothed = true;
            } else {
                // 在角度差上滤波，避免 359 → 1 跨越被拉到错误中点
                double delta = normalizeAngle(bearingWorld - smoothedBearingWorldDeg);
                smoothedBearingWorldDeg = normalizeAngle(
                        smoothedBearingWorldDeg + LOWPASS_ALPHA * delta);
            }
            lastTxDeg = tx;
            lastValidNanos = System.nanoTime();
            return;
        }
    }

    /** 视觉数据是否在保鲜期内（最近 VISION_HOLD_MS 毫秒内有有效更新）。 */
    public boolean isFresh() {
        if (lastValidNanos == 0L) return false;
        double elapsedMs = (System.nanoTime() - lastValidNanos) / 1e6;
        return elapsedMs < VISION_HOLD_MS;
    }

    /** 平滑后的世界系方位角（度），仅在 isFresh() 为 true 时有效。 */
    public double getBearingWorldDeg() {
        return smoothedBearingWorldDeg;
    }

    /** 最近一次看到 tag 时的原始 tx（度）；NaN 表示从未看到过。仅供遥测/调试。 */
    public double getLastTxDeg() {
        return lastTxDeg;
    }

    public int getTargetTagId() {
        return targetTagId;
    }

    public boolean isEnabled() {
        return ll != null && targetTagId >= 0;
    }

    private static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}
