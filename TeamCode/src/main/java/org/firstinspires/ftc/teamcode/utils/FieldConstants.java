package org.firstinspires.ftc.teamcode.utils;

/**
 * 场地常量：AprilTag ID、Goal 坐标查找等。
 *
 * 坐标系约定：场地中心原点，Y > 0 为蓝方半场，Y < 0 为红方半场。
 *
 * <h3>注意：{@link #tagIdForGoalY(double)} 仅适用于 TeleOp</h3>
 * Auto 的 BottomAutoBase 用"起始位置"作原点，TopAutoBase 用"goal"作原点
 * （两位程序员各写一边，坐标系约定从未统一）。这意味着 TopAuto 里的 {@code targetY}
 * 跟 alliance 没有对应关系（TopAutoBlue 和 TopAutoRed 都传 (30, 75)）。
 * 给 Auto 加视觉跟踪前必须显式从 alliance 选 tag ID，不能依赖 Y 符号推断。
 *
 * <h3>Tag ID 占位</h3>
 * 实际 ID 待 Decode 2026 官方手册确认；占位值仅为编译通过用，
 * 上场前必须更新。
 */
public final class FieldConstants {
    private FieldConstants() {}

    // TODO: 用 Decode 2026 官方手册的实际 tag ID 替换
    public static final int TAG_ID_BLUE_GOAL = 20;
    public static final int TAG_ID_RED_GOAL  = 24;

    /** 无视觉目标的哨兵值，传给 AutoPan 表示禁用视觉闭环。 */
    public static final int TAG_ID_NONE = -1;

    /**
     * 根据 goal 的场地 Y 坐标推断要瞄准的 AprilTag ID。
     * Y > 0 → 蓝方 goal，Y < 0 → 红方 goal。
     */
    public static int tagIdForGoalY(double goalY) {
        if (goalY > 0) return TAG_ID_BLUE_GOAL;
        if (goalY < 0) return TAG_ID_RED_GOAL;
        return TAG_ID_NONE;
    }
}
