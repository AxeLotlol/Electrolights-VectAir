package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Limelight AprilTag localization converted for Pedro Pathing
 *
 * Limelight frame:
 *   +X = right
 *   +Y = forward
 *   units = meters
 *
 * Pedro frame:
 *   +X = forward
 *   +Y = left
 *   units = inches
 */
public class LimelightLocalization implements Subsystem {

    public static final LimelightLocalization INSTANCE =
            new LimelightLocalization();

    private LimelightLocalization() {}

    private Limelight3A limelight;
    private IMU imu;

    // Pedro-compatible pose
    private double xInches = 0.0;
    private double yInches = 0.0;
    private double headingRad = 0.0;

    private boolean hasPose = false;

    private static final double METERS_TO_INCHES = 39.3701;

    @Override
    public void initialize() {

        limelight = ActiveOpMode.hardwareMap()
                .get(Limelight3A.class, "limelight");

        imu = ActiveOpMode.hardwareMap()
                .get(IMU.class, "imu");

        // AprilTag pipeline
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {

        // ---------------- IMU HEADING ----------------
        YawPitchRollAngles orientation =
                imu.getRobotYawPitchRollAngles();

        double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
        headingRad = Math.toRadians(yawDeg);

        // Provide heading to Limelight
        limelight.updateRobotOrientation(yawDeg);

        // ---------------- LIMELIGHT POSE ----------------
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            hasPose = false;
            return;
        }

        Pose3D botPose = result.getBotpose_MT2();
        if (botPose == null) {
            hasPose = false;
            return;
        }

        // Limelight pose (meters)
        double llX = botPose.getPosition().x; // right
        double llY = botPose.getPosition().y; // forward

        // Convert to Pedro coordinate frame (inches)
        xInches =  llY * METERS_TO_INCHES;
        yInches = -llX * METERS_TO_INCHES;

        hasPose = true;
    }

    // ---------------- PUBLIC GETTERS ----------------

    /** Pedro X (forward, inches) */
    public double getX() {
        return xInches;
    }

    /** Pedro Y (left, inches) */
    public double getY() {
        return yInches;
    }

    /** Pedro heading (radians, CCW+) */
    public double getHeading() {
        return headingRad;
    }

    /** Whether Limelight currently has a valid field pose */
    public boolean hasValidPose() {
        return hasPose;
    }
}
