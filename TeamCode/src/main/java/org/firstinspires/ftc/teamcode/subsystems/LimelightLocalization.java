package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class LimelightLocalization implements Subsystem {

    public static final LimelightLocalization INSTANCE = new LimelightLocalization();
    private LimelightLocalization() { }

    Limelight3A limelight;
    IMU imu;
    private double x;
    private double y;

    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        imu = ActiveOpMode.hardwareMap().get(IMU.class, "imu");

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double robotYaw = orientation.getYaw(AngleUnit.DEGREES);

        limelight.updateRobotOrientation(robotYaw);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            Pose3D botPose = result.getBotpose_MT2();

            x = botPose.getPosition().x;
            y = botPose.getPosition().y;
        }
    }

    public double returnX() {return x;}
    public double returnY() {return y;}

}
