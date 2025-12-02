package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class DistanceRed implements Subsystem {

    private Limelight3A limelight3A;
    double ta;

    private IMU imu;

    public DistanceRed() {

    }

    public static final DistanceRed INSTANCE = new DistanceRed();

    public void initialize() {
        limelight3A = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(7); //april tag 8 pipeline
        limelight3A.start();
        imu = ActiveOpMode.hardwareMap().get(IMU.class, "imu");
    }

    public void periodic() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            ta = llResult.getTa();
        }
    }

    public double getDistanceFromTag() {
        double distance = 1.892*Math.pow(ta, -0.513) + 0.08;
        return distance;
    }

}
