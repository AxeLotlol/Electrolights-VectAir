package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

public class ShooterConstants {
    public static Pose GOAL_POS_RED = new Pose(144, 144);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = 0.85125*39.37; //inches

    public static double SCORE_ANGLE = Math.toRadians(-5);

    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches

    public static double RAISE_TIME = 0.15;

}
