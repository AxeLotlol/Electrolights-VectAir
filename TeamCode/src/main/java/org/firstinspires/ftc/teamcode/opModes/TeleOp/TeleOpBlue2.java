package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain2;
import org.firstinspires.ftc.teamcode.vision.KalmanFilter;
import org.firstinspires.ftc.teamcode.vision.Limelight;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue2")
public class TeleOpBlue2 extends NextFTCOpMode {
    public MotorEx intakeMotor;
    public MotorEx transfer;

    private Limelight limelight;
    private KalmanFilter kalmanFilter;
    private boolean limelightEnabled = false;

    public TeleOpBlue2() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(DriveTrain2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static boolean blue;

    public static boolean isBlue(){
        return blue;
    }

    @Override
    public void onInit() {
        blue=true;
        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> intakeMotor.setPower(-1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> transfer.setPower(1))
                .whenBecomesFalse(() -> transfer.setPower(0));
        Gamepads.gamepad1().x().whenBecomesTrue(()->follower.setPose(new Pose(79.967,9.271,Math.toRadians(90))));

        // Limelight + Kalman Filter — pipeline setup only, no polling until toggled
        limelight = new Limelight(hardwareMap);
        limelight.initialize();
        kalmanFilter = new KalmanFilter(follower.getPose(), 0.05, 0.3);

        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> {
            limelightEnabled = !limelightEnabled;
            if (limelightEnabled) {
                limelight.start();
                telemetry.addLine("Limelight: ON");
                gamepad1.rumble(100);
            } else {
                limelight.stop();
                telemetry.addLine("Limelight: OFF");
                gamepad1.rumble(0.5, 0.5, 100);
            }
        });
    }

    @Override
    public void onUpdate() {
        if (limelightEnabled) {
            limelight.periodic();
            Pose odomPose = follower.getPose();
            kalmanFilter.predict(odomPose);

            if (limelight.canSeeTarget()) {
                double dist = limelight.getDistanceInches();
                Pose visionPose = limelight.getPose(odomPose.getHeading());
                if (visionPose != null) {
                    kalmanFilter.correct(visionPose, dist);
                    follower.setPose(kalmanFilter.getFusedPose(odomPose.getHeading()));
                    telemetry.addData("Limelight Dist (in)", dist);
                }
            }
        }
    }

    @Override
    public void onStartButtonPressed() {
    }

    public void onStop(){
        if (limelight != null && limelightEnabled) {
            limelight.stop();
        }
        blue=false;
    }
}
