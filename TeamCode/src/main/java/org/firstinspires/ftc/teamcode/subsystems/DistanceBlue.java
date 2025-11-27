package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.nextftc.core.subsystems.Subsystem;

public class DistanceBlue implements Subsystem {

    private Limelight3A limelight3A;
    TestBench bench = new TestBench();
    public double distance;
    double ta;

    public void initialize() {
        bench.init(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8); //april tag 8 pipeline
        limelight3A.start();
    }

    public void periodic() {
        YawPitchRollAngles orientation = bench.getOrientation();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            ta = llResult.getTa();
        }
    }

    public double getDistanceFromTag() {
        double scale = 5249; // y - value in graph equation (Ayush FTC yt playlist)
        double distance = scale / ta;
        return distance;
    }

}
