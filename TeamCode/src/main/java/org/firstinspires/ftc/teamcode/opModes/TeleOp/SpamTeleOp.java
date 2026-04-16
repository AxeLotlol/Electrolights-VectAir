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
//@TeleOp(name = "Solo TeleOp")
//@Configurable
//public class SpamTeleOp extends NextFTCOpMode {
//
//    public SpamTeleOp() {
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
//
//
//    private Command fullRoutine;
//
//    public Pose start = new Pose(112.158,135.289, Math.toRadians(90));
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
//                    new SetPosition(hoodServo1, runtime),
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
//
//    private SequentialGroup park() {
//        return new SequentialGroup(
//                new FollowPath(paths.parkPath, true, 1.0),
//                intakeMotorOff,
//                transferOff
//
//        );
//    }
//
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
//    private SequentialGroup intakeSet1Cycle() {
//        return new SequentialGroup(
//                intakeMotorOn,
//                transferOn,
//                closeTransfer,
//                new FollowPath(paths.toIntakeSet1Pre, true, 1.0),
//                intakeMotorOn,
//                transferOn,
//                new FollowPath(paths.launchFromSet1, true, 1.0),
//                shoot,
//                intakeMotorOn,
//                transferOn
//        );
//    }
//
//    private SequentialGroup intakeSet2Cycle() {
//        return new SequentialGroup(
//                intakeMotorOn,
//                transferOn,
//                closeTransfer,
//                new FollowPath(paths.toIntakeSet2Pre, true, 1.0),
//                intakeMotorOn,
//                transferOn,
//                new FollowPath(paths.launchFromSet2, true, 1.0),
//                shoot,
//                intakeMotorOn,
//                transferOn
//        );
//    }
//
//    private SequentialGroup intakeSet3Cycle() {
//        return new SequentialGroup(
//                intakeMotorOn,
//                transferOn,
//                closeTransfer,
//                new FollowPath(paths.toIntakeSet3Pre, true, 1.0),
//                intakeMotorOn,
//                transferOn,
//                new FollowPath(paths.launchFromSet3),
//
//                shoot,
//                intakeMotorOn,
//                transferOn
//        );
//    }
//
//    private Command buildFullRoutine() {
//        fullRoutine = new SequentialGroup(
//                intakeMotorOn,
//                transferOn,
//                closeTransfer,
//                new FollowPath(paths.parkToIntake, true, 1.0),
//                new Delay(1.1),
//                new FollowPath(paths.launchSpam2, true, 1.0),
//                shoot,
//                intakeMotorOn,
//                transferOn,
//                oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(), oneCycle(),
//                oneCycle(), oneCycle(), oneCycle(),oneCycle(), oneCycle(), lastCall, park()
//        );
//        return fullRoutine;
//    }
//
//    @Override
//    public void onInit() {
//        telemetry.addLine("Initializing Solo TeleOP...");
//        telemetry.update();
//
//        follower = PedroComponent.follower();
//        endgame = false;
//        fullRoutine = null;
//
//        start = new Pose(112.158,135.289, Math.toRadians(90));
//        follower.setStartingPose(start);
//        paths = new Paths(follower);
//
//        follower.update();
//
//        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();
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
//
//
//        telemetry.addLine("Solo TeleOP Initialized");
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
//    public boolean endgame = false;
//
//    Command lastCall = new LambdaCommand()
//            .setStart(()-> endgame= true);
//    @Override
//    public void onUpdate() {
//        follower.update();
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
//
//        if(!endgame) {
//
//            shooter((float) (flywheelSpeed + 30));
//            double hoodAngle = results[1];
//            hoodToPos(hoodAngle);
//        }
//        else{
//            shooter(0);
//        }
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
//        if (fullRoutine != null) fullRoutine.cancel();
//        follower.breakFollowing();
//        intakeMotor.setPower(0);
//        transfer1.setPower(0);
//        flywheel.setPower(0);
//        flywheel2.setPower(0);
//        shooter(0);
//        telemetry.addLine("Solo TeleOP Stopped.");
//        telemetry.update();
//    }
//
//    public class Paths {
//
//        public PathChain parkToIntake;
//        public PathChain resetAndIntake2;
//        public PathChain launchSpam2;
//        public PathChain parkPath;
//
//        public PathChain toIntakeSet1Pre;
//        public PathChain toIntakeSet1Final;
//        public PathChain launchFromSet1;
//
//        public PathChain toIntakeSet2Pre;
//        public PathChain toIntakeSet2Final;
//        public PathChain launchFromSet2;
//
//        public PathChain toIntakeSet3Pre;
//        public PathChain toIntakeSet3Final;
//        public PathChain launchFromSet3;
//
//        public Paths(Follower follower) {
//
//            parkToIntake = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    start,
//                                    new Pose(104.000, 67.000),
//                                    new Pose(132.5, 62.25)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(30))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.8)
//                    .addTemporalCallback(0.1, intakeMotorOn)
//                    .addTemporalCallback(0.1, transferOn)
//                    .build();
//
//            resetAndIntake2 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(92, 94),
//                                    new Pose(104.000, 67.000),
//                                    new Pose(132.5, 62.25)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(30))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.8)
//                    .addTemporalCallback(0.1, intakeMotorOn)
//                    .addTemporalCallback(0.1, transferOn)
//                    .build();
//
//            launchSpam2 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(132.5, 62.52),
//                                    new Pose(104.000, 67.000),
//                                    new Pose(92, 94)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(17), Math.toRadians(46))
//                    .setVelocityConstraint(0.3)
//                    .setTValueConstraint(0.95)
//                    .addPoseCallback(new Pose(118, 64), reverseIntakeForMe, 0.4)
//                    .build();
//
//            toIntakeSet1Pre = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(92.000, 94.000),
//                                    new Pose(50.000, 87.000),
//                                    new Pose(20, 93)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.8)
//                    .addTemporalCallback(0.1, intakeMotorOn)
//                    .addTemporalCallback(0.1, transferOn)
//                    .build();
//
//            /*toIntakeSet1Final = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(40, 93),
//                                    new Pose(20, 93)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(179))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.9)
//                    .addTemporalCallback(0.1, intakeMotorOn)
//                    .addTemporalCallback(0.1, transferOn)
//                    .build();*/
//
//            launchFromSet1 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(20, 88.898),
//                                    new Pose(92.000, 94.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(179), Math.toRadians(46))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.95)
//                    .addPoseCallback(new Pose(40, 87), reverseIntakeForMe, 0.4)
//                    .build();
//
//            toIntakeSet2Pre = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(92.000, 94.000),
//                                    new Pose(57.000, 56.798),
//                                    new Pose(17.660, 64.590)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.8)
//                    .addTemporalCallback(0.1, intakeMotorOn)
//                    .addTemporalCallback(0.1, transferOn)
//                    .build();
//
//            /*toIntakeSet2Final = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(20.660, 64.590),
//                                    new Pose(15.000, 64.590)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(195))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.9)
//                    .addTemporalCallback(0.1, intakeMotorOn)
//                    .addTemporalCallback(0.1, transferOn)
//                    .build();*/
//
//            launchFromSet2 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(17.660, 64.590),
//                                    new Pose(38.000, 67.000),
//                                    new Pose(92.000, 94.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(46))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.95)
//                    .addPoseCallback(new Pose(24, 64), reverseIntakeForMe, 0.4)
//                    .build();
//
//            toIntakeSet3Pre = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(92.000, 94.000),
//                                    new Pose(54.000, 89.500),
//                                    new Pose(67.000, 28.615),
//                                    new Pose(20.660, 42.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.8)
//                    .addTemporalCallback(0.1, intakeMotorOn)
//                    .addTemporalCallback(0.1, transferOn)
//                    .build();
//
//            /*toIntakeSet3Final = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(16.000, 42.000),
//                                    new Pose(14, 42.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.9)
//                    .addTemporalCallback(0.1, intakeMotorOn)
//                    .addTemporalCallback(0.1, transferOn)
//                    .build();*/
//
//            launchFromSet3 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(20.660, 42.000),
//                                    new Pose(38.000, 67.000),
//                                    new Pose(92.000, 94.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(179), Math.toRadians(46))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.95)
//                    .addPoseCallback(new Pose(35, 60), reverseIntakeForMe, 0.5)
//                    .build();
//
//            parkPath = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(92, 94),
//                                    new Pose(42.630, 40.572)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(90))
//                    .setVelocityConstraint(1.0)
//                    .setTValueConstraint(0.8)
//                    .build();
//        }
//    }
//}
//

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

@TeleOp(name = "Solo Spam + Spiking")
@Configurable
public class SpamTeleOp extends NextFTCOpMode {

    public SpamTeleOp() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, DistanceRed.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))
        );
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;



    private Command fullRoutine;

    public Pose start;

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
                    new SetPosition(hoodServo1, runtime),
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


    private SequentialGroup park() {
        return new SequentialGroup(
                new FollowPath(paths.parkPath, true, 1.0),
                intakeMotorOff,
                transferOff

        );
    }


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
    private SequentialGroup oneCycleAfterSpike1() {
        return new SequentialGroup(
                intakeMotorOn,
                transferOn,
                closeTransfer,
                new FollowPath(paths.resetAndIntake1, true, 1.0),
                new Delay(0.05),
                new FollowPath(paths.moverBacker, true, 1.0),
                new Delay(1.3),
                new FollowPath(paths.launchSpam2, true, 1.0),
                shoot,
                intakeMotorOn,
                transferOn
        );
    }

    private SequentialGroup oneCycleAfterSpike2() {
        return new SequentialGroup(
                intakeMotorOn,
                transferOn,
                closeTransfer,
                new FollowPath(paths.resetAndIntake1, true, 1.0),
                new Delay(0.05),
                new FollowPath(paths.moverBacker2, true, 1.0),
                new Delay(1.3),
                new FollowPath(paths.launchSpam2, true, 1.0),
                shoot,
                intakeMotorOn,
                transferOn
        );
    }

    private SequentialGroup intakeSet1Cycle() {
        return new SequentialGroup(
                intakeMotorOn,
                transferOn,
                closeTransfer,
                new FollowPath(paths.toIntakeSet1Pre, true, 1.0),
                intakeMotorOn,
                transferOn,
                new FollowPath(paths.launchFromSet1, true, 1.0),
                shoot,
                intakeMotorOn,
                transferOn
        );
    }

    private SequentialGroup intakeSet2Cycle() {
        return new SequentialGroup(
                intakeMotorOn,
                transferOn,
                closeTransfer,
                new FollowPath(paths.toIntakeSet2Pre, true, 1.0),
                intakeMotorOn,
                transferOn,
                new FollowPath(paths.launchFromSet2, true, 1.0),
                shoot,
                intakeMotorOn,
                transferOn
        );
    }

    private SequentialGroup intakeSet3Cycle() {
        return new SequentialGroup(
                intakeMotorOn,
                transferOn,
                closeTransfer,
                new FollowPath(paths.toIntakeSet3Pre, true, 1.0),
                intakeMotorOn,
                transferOn,
                new FollowPath(paths.launchFromSet3),

                shoot,
                intakeMotorOn,
                transferOn
        );
    }

    private Command buildFullRoutine() {
        fullRoutine = new SequentialGroup(
                intakeMotorOn,
                transferOn,
                closeTransfer,
                new FollowPath(paths.parkToIntake, true, 1.0),
                new Delay(1),
                new FollowPath(paths.launchSpam2, true, 1.0),
                shoot,
                intakeMotorOn,
                transferOn,

                oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(), oneCycle(),

                intakeSet1Cycle(),

                oneCycleAfterSpike1(), oneCycle(), oneCycle(), oneCycle(),
                oneCycle(), oneCycle(),
                intakeSet2Cycle(),

                oneCycleAfterSpike2(), oneCycle(), oneCycle(), oneCycle(), lastCall, park()
        );
        return fullRoutine;
    }

    @Override
    public void onInit() {
        telemetry.addLine("Initializing Solo TeleOP...");
        telemetry.update();

        follower = PedroComponent.follower();
        endgame = false;
        fullRoutine = null;

        //start = new Pose(112.158,135.289, Math.toRadians(90));
        start = Storage.currentPose;
        follower.setStartingPose(start);
        paths = new Paths(follower);

        follower.update();

        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        intakeMotor = new MotorEx("intake");
        transfer1   = new MotorEx("transfer");
        transfer2   = new ServoEx("transferServo1");

        hoodServo1n = ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
        hoodServo2n = ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");

        pathTimer   = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();



        telemetry.addLine("Solo TeleOP Initialized");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        start = Storage.currentPose;
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        shooter(1085);
        buildFullRoutine().schedule();
    }
    public boolean endgame = false;

    Command lastCall = new LambdaCommand()
            .setStart(()-> endgame= true);
    @Override
    public void onUpdate() {
        follower.update();

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

        if(!endgame) {

            shooter((float) (flywheelSpeed + 30));
            double hoodAngle = results[1];
            hoodToPos(hoodAngle);
        }
        else{
            shooter(0);
        }

        Storage.currentPose = follower.getPose();

        ActiveOpMode.telemetry().addData("Cycle time (s)", opmodeTimer.getElapsedTimeSeconds());
        ActiveOpMode.telemetry().addData("Pose", follower.getPose());
        ActiveOpMode.telemetry().update();
    }

    @Override
    public void onStop() {
        Storage.currentPose = follower.getPose();
        if (fullRoutine != null) fullRoutine.cancel();
        follower.breakFollowing();
        intakeMotor.setPower(0);
        transfer1.setPower(0);
        flywheel.setPower(0);
        flywheel2.setPower(0);
        shooter(0);
        telemetry.addLine("Solo TeleOP Stopped.");
        telemetry.update();
    }

    public class Paths {

        public PathChain parkToIntake;
        public PathChain resetAndIntake2;
        public PathChain launchSpam2;
        public PathChain parkPath;

        public PathChain toIntakeSet1Pre;
        public PathChain launchFromSet1;

        public PathChain toIntakeSet2Pre;
        public PathChain launchFromSet2;

        public PathChain toIntakeSet3Pre;
        public PathChain launchFromSet3;

        public PathChain resetAndIntake1;

        public PathChain moverBacker;
        public PathChain moverBacker2;

        public Paths(Follower follower) {

            parkToIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    start,
                                    new Pose(89.000, 67.000),
                                    new Pose(132.5, 62.25)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(30))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeMotorOn)
                    .addTemporalCallback(0.1, transferOn)
                    .build();

            resetAndIntake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92, 94),
                                    new Pose(104.000, 67.000),
                                    new Pose(132.5, 62.25)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(30))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeMotorOn)
                    .addTemporalCallback(0.1, transferOn)
                    .build();

            launchSpam2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(132.5, 62.52),
                                    new Pose(104.000, 67.000),
                                    new Pose(92, 94)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(17), Math.toRadians(43))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(118, 64), reverseIntakeForMe, 0.4)
                    .build();

            toIntakeSet1Pre = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 94.000),
                                    new Pose(50.000, 87.000),
                                    new Pose(20, 99)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(177))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeMotorOn)
                    .addTemporalCallback(0.1, transferOn)
                    .build();


            launchFromSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20, 99),
                                    new Pose(92.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(177), Math.toRadians(43))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addPoseCallback(new Pose(40, 87), reverseIntakeForMe, 0.4)
                    .build();

            toIntakeSet2Pre = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 94.000),
                                    new Pose(57.000, 56.798),
                                    new Pose(17.660, 64.590)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeMotorOn)
                    .addTemporalCallback(0.1, transferOn)
                    .build();


            launchFromSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(17.660, 64.590),
                                    new Pose(38.000, 67.000),
                                    new Pose(92.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(43))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(24, 64), reverseIntakeForMe, 0.4)
                    .build();

            toIntakeSet3Pre = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 94.000),
                                    new Pose(54.000, 89.500),
                                    new Pose(67.000, 28.615),
                                    new Pose(18.660, 42.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeMotorOn)
                    .addTemporalCallback(0.1, transferOn)
                    .build();

            launchFromSet3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(18.660, 42.000),
                                    new Pose(38.000, 67.000),
                                    new Pose(92.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(43))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(35, 60), reverseIntakeForMe, 0.5)
                    .build();

            resetAndIntake1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 94.000),
                                    new Pose(104.000, 67.000),
                                    new Pose(131, 64)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(20))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1,intakeMotorOn)
                    .addTemporalCallback(0.1,transferOn)

                    .build();

            moverBacker = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131, 64),
                                    new Pose(130.75, 59)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(45))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)

                    .build();

            moverBacker2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130, 64),
                                    new Pose(132.25, 59)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(45))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)

                    .build();

            parkPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(92, 94),
                                    new Pose(42.630, 40.572)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(90))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .build();
        }
    }
}
