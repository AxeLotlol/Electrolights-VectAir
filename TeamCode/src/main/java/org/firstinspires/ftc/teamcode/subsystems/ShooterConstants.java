package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class ShooterConstants {
    public static Pose GOAL_POS_RED = new Pose(144, 144);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = 31; //inches
    public static double SCORE_HEIGHT_CLOSER = 34; //inches

    public static double SCORE_ANGLE = Math.toRadians(-24);
    public static double SCORE_ANGLE_CLOSER = Math.toRadians(-50);

    public static double PASS_THROUGH_POINT_RADIUS = 4; //inches

    public static double RAISE_TIME = 0.15;

}
