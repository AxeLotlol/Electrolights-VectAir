package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed.isRed;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
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

    private LimelightLocalization() {
    }

    private Limelight3A limelight;
    private IMU imu;

    // Pedro-compatible pose
    private double xInches = 0.0;
    private double yInches = 0.0;
    private double headingRad = 0.0;

    private boolean hasPose = false;

    private static final double METERS_TO_INCHES = 39.3701;

    public int alliance;

    @Override
    public void initialize() {

        if(isBlue()==true) {
            alliance=1;
        }
        if(isRed()==true){
            alliance=-1;
        }

        limelight = ActiveOpMode.hardwareMap()
                .get(Limelight3A.class, "limelight");

        imu = ActiveOpMode.hardwareMap()
                .get(IMU.class, "imu");

        // AprilTag pipeline
        limelight.pipelineSwitch(0);
        limelight.start();

        //Pose startingpose = new Pose (72, 72, Math.toRadians(90));
        //follower = PedroComponent.follower();
        //follower.setStartingPose(startingpose);
        //follower.update();
    }



    @Override
    public void periodic() {

        // ---------------- IMU HEADING ----------------

        //follower.update();
        double headingnow = follower.getHeading();
        // Provide heading to Limelight
        //limelight.updateRobotOrientation((90 - headingnow + 360) % 360);

        // ---------------- LIMELIGHT POSE ----------------
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            hasPose = false;
            return;
        }

        Pose3D botPose = result.getBotpose();
        if (botPose == null) {
            hasPose = false;
            return;
        }


        // Convert to Pedro coordinate frame (inches)
        xInches =  botPose.getPosition().y * METERS_TO_INCHES + 72;
        yInches = -1*botPose.getPosition().x * METERS_TO_INCHES + 72;

        ActiveOpMode.telemetry().addData("LL X inches:", xInches);
        ActiveOpMode.telemetry().addData("LL Y inches:", yInches);

        hasPose = true;
    }

}
