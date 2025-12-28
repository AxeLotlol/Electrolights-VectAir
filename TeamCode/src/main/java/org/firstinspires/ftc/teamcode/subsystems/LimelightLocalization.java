package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.nextftc.core.subsystems.Subsystem;

public class LimelightLocalization implements Subsystem {

    public static final LimelightLocalization INSTANCE = new LimelightLocalization();
    private LimelightLocalization() { }

    Limelight3A limelight;
    IMU imu;

    public void initialize() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void periodic() {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double robotYaw = orientation.getYaw(AngleUnit.DEGREES);

        limelight.updateRobotOrientation(robotYaw);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            Pose3D botPose = result.getBotpose_MT2();

            double x = botPose.getPosition().x;
            double y = botPose.getPosition().y;

            telemetry.addData("MT2 X", x);
            telemetry.addData("MT2 Y", y);
        }
    }
}