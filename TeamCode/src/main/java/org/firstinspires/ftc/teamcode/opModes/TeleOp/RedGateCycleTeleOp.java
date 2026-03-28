//package org.firstinspires.ftc.teamcode.opModes.TeleOp;
//
//import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
//import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.Vector;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
//import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
//import org.firstinspires.ftc.teamcode.subsystems.Storage;
//
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.delays.Delay;
//import dev.nextftc.core.commands.groups.ParallelGroup;
//import dev.nextftc.core.commands.groups.SequentialGroup;
//import dev.nextftc.core.commands.utility.LambdaCommand;
//import dev.nextftc.core.components.BindingsComponent;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.extensions.pedro.FollowPath;
//import dev.nextftc.extensions.pedro.PedroComponent;
//import dev.nextftc.ftc.ActiveOpMode;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.ftc.components.BulkReadComponent;
//import dev.nextftc.hardware.impl.Direction;
//import dev.nextftc.hardware.impl.IMUEx;
//import dev.nextftc.hardware.impl.MotorEx;
//import dev.nextftc.hardware.impl.ServoEx;
//import dev.nextftc.hardware.positionable.SetPosition;
//
//@TeleOp
//@Configurable
//public class RedGateCycleTeleOp extends NextFTCOpMode {
//
//    public RedGateCycleTeleOp() {
//        addComponents(
//                new SubsystemComponent(Flywheel.INSTANCE, DistanceRed.INSTANCE),
//                BulkReadComponent.INSTANCE,
//                BindingsComponent.INSTANCE,
//                new PedroComponent(hwMap -> Constants.createFollower(hwMap))
//        );
//    }
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//    public Pose start = new Pose(91, 115, Math.toRadians(90));
//
//    private Paths paths;
//    public MotorEx intakeMotor;
//    private MotorEx transfer1;
//    private ServoEx transfer2;
//
//    public static MotorEx flywheel  = new MotorEx("launchingmotor");
//    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");
//
//    private static Servo   hoodServo1n;
//    private static Servo   hoodServo2n;
//    private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
//    private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);
//
//    public static double hoodToPos(double runtime) {
//        if (!Double.isNaN(runtime)) {
//            ActiveOpMode.telemetry().addData("runtime", runtime);
//            ParallelGroup HoodRunUp = new ParallelGroup(
//                    new SetPosition(hoodServo1,      runtime),
//                    new SetPosition(hoodServo2, -1 * runtime)
//            );
//            HoodRunUp.schedule();
//            return runtime;
//        } else {
//            ActiveOpMode.telemetry().addLine("NaN");
//            return 0;
//        }
//    }
//
//    private Command intakeMotorOn = new LambdaCommand()
//            .setStart(() -> intakeMotor.setPower(-1));
//
//    Command intakeMotorOff = new LambdaCommand()
//            .setStart(() -> intakeMotor.setPower(0));
//
//    Command transferOn = new LambdaCommand()
//            .setStart(() -> transfer1.setPower(-1));
//
//    Command transferOff = new LambdaCommand()
//            .setStart(() -> transfer1.setPower(0));
//
//    public Command opentransfer = new LambdaCommand()
//            .setStart(() -> transfer2.setPosition(0.3));
//
//    public Command closeTransfer = new LambdaCommand()
//            .setStart(() -> transfer2.setPosition(0.635));
//
//    public SequentialGroup shoot = new SequentialGroup(
//            opentransfer,
//            new Delay(0.05),
//            transferOn,
//            new Delay(0.3),
//            transferOff,
//            closeTransfer
//    );
//
//    public Command reverseIntakeForMe = new LambdaCommand()
//            .setStart(() -> intakeMotor.setPower(0.5));
//
//    private SequentialGroup oneCycle() {
//        return new SequentialGroup(
//                intakeMotorOn,
//                transferOn,
//                closeTransfer,
//                new FollowPath(paths.resetAndIntake2, true, 1.0),
//                new Delay(1.6),
//                new FollowPath(paths.launchSpam2, true, 1.0),
//                shoot,
//                intakeMotorOn,
//                transferOn
//        );
//    }
//
//    private Command buildFullRoutine() {
//        return new SequentialGroup(
//                new FollowPath(paths.parkToLaunch, true, 1.0),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle()
//
//        );
//    }
//
//    @Override
//    public void onInit() {
//        telemetry.addLine("Initializing RedGateCycleTeleOp...");
//        telemetry.update();
//
//        follower = PedroComponent.follower();
//
//        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();
//
//        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(7);
//        limelight.start();
//
//        intakeMotor = new MotorEx("intake");
//        transfer1   = new MotorEx("transfer");
//        transfer2   = new ServoEx("transferServo1");
//
//        hoodServo1n = ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
//        hoodServo2n = ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");
//
//        pathTimer   = new Timer();
//        actionTimer = new Timer();
//        opmodeTimer = new Timer();
//
//        follower.setStartingPose(start);
//        paths = new Paths(follower);
//
//        follower.update();
//
//        telemetry.addLine("RedGateCycleTeleOp ready!");
//        telemetry.update();
//    }
//
//    @Override
//    public void onStartButtonPressed() {
//        opmodeTimer.resetTimer();
//        pathTimer.resetTimer();
//        shooter(1085);
//        buildFullRoutine().schedule();
//    }
//
//    @Override
//    public void onUpdate() {
//        follower.update();
//
//        if (opmodeTimer.getElapsedTimeSeconds() >= 150) {
//            follower.breakFollowing();
//            intakeMotor.setPower(0);
//            transfer1.setPower(0);
//            return;
//        }
//
//        Pose currPose = follower.getPose();
//        double robotHeading = follower.getPose().getHeading();
//        Vector robotToGoalVector = new Vector(
//                follower.getPose().distanceFrom(new Pose(138, 141)),
//                Math.atan2(141 - currPose.getY(), 138 - currPose.getX())
//        );
//        Double[] results = calculateShotVectorandUpdateHeading(
//                robotHeading, robotToGoalVector, follower.getVelocity()
//        );
//        double flywheelSpeed = results[0];
//        shooter((float) (flywheelSpeed + 30));
//        double hoodAngle = results[1];
//        hoodToPos(hoodAngle);
//
//        Storage.currentPose = follower.getPose();
//
//        ActiveOpMode.telemetry().addData("Cycle time (s)", opmodeTimer.getElapsedTimeSeconds());
//        ActiveOpMode.telemetry().addData("Pose", follower.getPose());
//        ActiveOpMode.telemetry().update();
//    }
//
//    @Override
//    public void onStop() {
//        Storage.currentPose = follower.getPose();
//        follower.breakFollowing();
//        intakeMotor.setPower(0);
//        transfer1.setPower(0);
//        telemetry.addLine("RedGateCycleTeleOp Stopped.");
//        telemetry.update();
//    }
//
//    public class Paths {
//
//        public PathChain parkToLaunch;
//        public PathChain resetAndIntake2;
//        public PathChain launchSpam2;
//
//        public Paths(Follower follower) {
//
//            parkToLaunch = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(91, 115),
//                                    new Pose(92, 94)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(46))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.8)
//                    .build();
//
//            resetAndIntake2 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(92.000, 94.000),
//                                    new Pose(104.000, 67.000),
//                                    new Pose(130.5, 62)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(23))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.8)
//                    .addTemporalCallback(0.1, intakeMotorOn)
//                    .addTemporalCallback(0.1, transferOn)
//                    .build();
//
//            launchSpam2 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(130.5, 63),
//                                    new Pose(104.000, 67.000),
//                                    new Pose(92, 94)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(17), Math.toRadians(47))
//                    .setVelocityConstraint(0.3)
//                    .setTValueConstraint(0.95)
//                    .addPoseCallback(new Pose(118, 64), reverseIntakeForMe, 0.4)
//                    .build();
//        }
//    }
//}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------
package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@TeleOp(name = "Red Cycle Spam")
@Configurable
public class RedGateCycleTeleOp extends NextFTCOpMode {

    public RedGateCycleTeleOp() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, DistanceRed.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))
        );
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean stopped = false;
    private boolean parked = false;

    public Pose start = new Pose(91, 115, Math.toRadians(90));

    private Paths paths;
    public MotorEx intakeMotor;
    private MotorEx transfer1;
    private ServoEx transfer2;

    public static MotorEx flywheel  = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    private static Servo   hoodServo1n;
    private static Servo   hoodServo2n;
    private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
    private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);

    public static double hoodToPos(double runtime) {
        if (!Double.isNaN(runtime)) {
            ActiveOpMode.telemetry().addData("runtime", runtime);
            ParallelGroup HoodRunUp = new ParallelGroup(
                    new SetPosition(hoodServo1,      runtime),
                    new SetPosition(hoodServo2, -1 * runtime)
            );
            HoodRunUp.schedule();
            return runtime;
        } else {
            ActiveOpMode.telemetry().addLine("NaN");
            return 0;
        }
    }

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(-1));

    Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0));

    Command transferOn = new LambdaCommand()
            .setStart(() -> transfer1.setPower(-1));

    Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));

    public Command opentransfer = new LambdaCommand()
            .setStart(() -> transfer2.setPosition(0.3));

    public Command closeTransfer = new LambdaCommand()
            .setStart(() -> transfer2.setPosition(0.635));

    public SequentialGroup shoot = new SequentialGroup(
            opentransfer,
            new Delay(0.05),
            transferOn,
            new Delay(0.3),
            transferOff,
            closeTransfer
    );

    public Command reverseIntakeForMe = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0.5));

    private SequentialGroup oneCycle() {
        return new SequentialGroup(
                intakeMotorOn,
                transferOn,
                closeTransfer,
                new FollowPath(paths.resetAndIntake2, true, 1.0),
                new Delay(1.6),
                new FollowPath(paths.launchSpam2, true, 1.0),
                shoot,
                intakeMotorOn,
                transferOn
        );
    }

    private Command buildFullRoutine() {
        return new SequentialGroup(
                intakeMotorOn,
                transferOn,
                closeTransfer,
                new FollowPath(paths.parkToIntake, true, 1.0),
                new Delay(1.6),
                new FollowPath(paths.launchSpam2, true, 1.0),
                shoot,
                intakeMotorOn,
                transferOn,
                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),
                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),
                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),
                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),
                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle()
        );
    }

    @Override
    public void onInit() {
        telemetry.addLine("Initializing RedGateCycleTeleOp...");
        telemetry.update();

        follower = PedroComponent.follower();
        stopped = false;
        parked = false;

        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();

        intakeMotor = new MotorEx("intake");
        transfer1   = new MotorEx("transfer");
        transfer2   = new ServoEx("transferServo1");

        hoodServo1n = ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
        hoodServo2n = ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");

        pathTimer   = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower.setStartingPose(start);
        paths = new Paths(follower);

        follower.update();

        telemetry.addLine("RedGateCycleTeleOp ready!");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        shooter(1085);
        buildFullRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        follower.update();

        // At 117s, cancel current path and start parking
        if (opmodeTimer.getElapsedTimeSeconds() >= 117 && !parked && !stopped) {
            parked = true;
            follower.breakFollowing();
            intakeMotor.setPower(0);
            transfer1.setPower(0);
            new FollowPath(paths.parkPath, true, 1.0).schedule();
        }

        // Shut down when EITHER:
        // - Park path finished naturally (!follower.isBusy())
        // - 119.5s safety cutoff in case park takes too long
        if (parked && !stopped && (!follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() >= 119.5)) {
            stopped = true;
            follower.breakFollowing();
            intakeMotor.setPower(0);
            transfer1.setPower(0);
            flywheel.setPower(0);
            flywheel2.setPower(0);
            transfer2.setPosition(0.635);
            shooter(0);
            return;
        }

        if (stopped) return;

        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();
        Vector robotToGoalVector = new Vector(
                follower.getPose().distanceFrom(new Pose(138, 141)),
                Math.atan2(141 - currPose.getY(), 138 - currPose.getX())
        );
        Double[] results = calculateShotVectorandUpdateHeading(
                robotHeading, robotToGoalVector, follower.getVelocity()
        );
        double flywheelSpeed = results[0];
        shooter((float) (flywheelSpeed + 30));
        double hoodAngle = results[1];
        hoodToPos(hoodAngle);

        Storage.currentPose = follower.getPose();

        ActiveOpMode.telemetry().addData("Cycle time (s)", opmodeTimer.getElapsedTimeSeconds());
        ActiveOpMode.telemetry().addData("Pose", follower.getPose());
        ActiveOpMode.telemetry().update();
    }

    @Override
    public void onStop() {
        Storage.currentPose = follower.getPose();
        follower.breakFollowing();
        intakeMotor.setPower(0);
        transfer1.setPower(0);
        flywheel.setPower(0);
        flywheel2.setPower(0);
        shooter(0);
        telemetry.addLine("RedGateCycleTeleOp Stopped.");
        telemetry.update();
    }

    public class Paths {

        public PathChain parkToIntake;
        public PathChain resetAndIntake2;
        public PathChain launchSpam2;
        public PathChain parkPath;

        public Paths(Follower follower) {

            parkToIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(91, 115),
                                    new Pose(104.000, 67.000),
                                    new Pose(131.5, 59.25)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(25))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeMotorOn)
                    .addTemporalCallback(0.1, transferOn)
                    .build();

            resetAndIntake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 94.000),
                                    new Pose(104.000, 67.000),
                                    new Pose(130.5, 60.75)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(25))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeMotorOn)
                    .addTemporalCallback(0.1, transferOn)
                    .build();

            launchSpam2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(131.5, 61.5),
                                    new Pose(104.000, 67.000),
                                    new Pose(92, 94)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(17), Math.toRadians(46))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(118, 64), reverseIntakeForMe, 0.4)
                    .build();

            parkPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(92, 94),
                                    new Pose(38.630, 33.572)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(90))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .build();
        }
    }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------