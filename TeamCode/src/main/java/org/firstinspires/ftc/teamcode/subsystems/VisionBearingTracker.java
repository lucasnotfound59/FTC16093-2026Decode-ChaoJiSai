package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

/**
 * 视觉方位角跟踪器：封装 Limelight 3A 的读取、ID 过滤、延迟补偿、滤波和保鲜期判断。
 *
 * <h3>核心输出</h3>
 * "目标在世界系下的方位角"（度），是一个不变量——只跟 tag 真实位置有关，
 * 跟 pan 当前角度无关。AutoPan 拿到这个值后减去当前 heading 就是 pan 目标角。
 *
 * <h3>为什么在世界系闭环</h3>
 * 直觉上的做法是 {@code pan_target = pan_current + tx}，但这会震荡：
 * LL 有 20–30ms 延迟，pan 在帧捕获到帧使用之间会动，反馈量被自己改变 → 自激。
 * 把每次测量翻译成世界系常量 ({@code pan_encoder + tx + heading_at_frame}) 后，
 * 下游用最新 heading 反解回机器人系，pan 自己的运动就不会污染反馈了。
 *
 * <h3>不做帧去重</h3>
 * FTC 控制循环 ~30–50Hz，跟 LL 50Hz 同量级，重复消费同一帧的概率不高。
 * SDK 没有可靠的"唯一帧 ID"字段；用 staleness 判同帧不稳。
 * 偶发的重复消费在 lowpass 上等效于 α 略偏大，影响微乎其微，故不做。
 *
 * <h3>典型调用流程（每个控制循环）</h3>
 * <pre>
 *   tracker.update(panEncoderDeg, headingDegNow, yawRateDegPerSec);
 *   if (tracker.isFresh()) {
 *       panTarget = tracker.getBearingWorldDeg() - headingDegNow;
 *   } else {
 *       panTarget = ... // 退回 odo 或其他来源
 *   }
 * </pre>
 */
public class VisionBearingTracker {

    // ==========================================
    // 调参（public 方便从 Dashboard 改）
    // ==========================================
    /**
     * 视觉数据保鲜期（毫秒）：超过这个时间没有有效帧，isFresh() 返回 false。
     * 80ms ≈ LL 50Hz 下 4 帧的容错；超出后立即退回 odo，避免在保鲜期里机器人继续
     * 移动导致存储的世界系方位角越用越偏。
     */
    public static double VISION_HOLD_MS = 80.0;
    /**
     * 一阶 lowpass 系数，0–1。越大响应越快、越抖；越小越平滑、越滞后。
     * 0.4 经验值：LL tx 噪声大约 ±0.2°，滤波后衰减 ~60% 到 ±0.08°，
     * 配合 AutoPan 的 0.3° 死区基本看不到抖动。
     */
    public static double LOWPASS_ALPHA = 0.4;
    /**
     * 单帧最大可接受延迟（毫秒），超出则丢弃。
     * 防御性阈值：正常 LL 延迟 20–30ms，超过 100ms 说明 USB 抖动或处理积压，
     * 该帧的世界系方位角已经不可信。
     */
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

    /**
     * 清空所有视觉历史状态。
     * AutoPan 在切换到 TRACK 模式时调用，防止 HOLD 模式下扫到的"杂帧"污染
     * 接下来的平滑值（pan 在 HOLD 时可能转过其他 tag 的视野，留下错误测量）。
     */
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
            // ID 过滤：必须是当前 alliance 的 goal tag，不接受任何其他 fiducial
            if (fr.getFiducialId() != targetTagId) continue;

            double tx = fr.getTargetXDegrees();
            // 用 yaw rate 把 heading 外推回 LL 帧捕获时刻：
            // captureLatency 是从图像曝光到 SDK 拿到结果之间的时间，
            // 这段时间里机器人可能已经转过 (yawRate × latency) 度，必须减回去。
            double headingAtFrame = headingDegNow - yawRateDegPerSec * (captureLatencyMs / 1000.0);
            // 世界系方位角 = pan 在机器人系角度 + LL 在 pan 上看到的水平偏移 + 当时的机器人朝向
            // 这三项之和与 tag 的真实位置一一对应，pan 现在转到哪里、heading 现在是多少都不影响
            double bearingWorld = panEncoderDeg + tx + headingAtFrame;

            if (!hasSmoothed) {
                // 第一帧直接接受，不做加权——否则 smoothed 从 0 开始要花很多帧才能拉到真值
                smoothedBearingWorldDeg = bearingWorld;
                hasSmoothed = true;
            } else {
                // 在角度差上滤波，避免 359 → 1 跨越被拉到错误中点（直接加权会得到 ~180）
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
