package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;

public class MotifScanningSingle extends NextFTCOpMode {

    public static final MotifScanningSingle INSTANCE = new MotifScanningSingle();
    private MotifScanningSingle() { }

    private static int tagID;
    private static Limelight3A limelight;
    private static LLResult llResult;

    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.start();
    }

    public void periodic() {
        llResult = limelight.getLatestResult();
    }

    public static int findMotif() {
        tagID = -1;
        limelight.pipelineSwitch(1); //Motif pipeline
        if (llResult != null && llResult.isValid()) {
        tagID = llResult.getFiducialResults().get(0).getFiducialId();
        }
        return tagID;
    }

}
