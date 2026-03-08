package org.firstinspires.ftc.teamcode.opModes.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.AutoShooterCalc.calculateShotVectorandUpdateHeading;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;

import dev.nextftc.extensions.pedro.TurnTo;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Autonomous
@Configurable
public class blueFarZone extends NextFTCOpMode {
    public blueFarZone() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, DistanceBlue.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))
        );
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Paths paths;
    public MotorEx intakeMotor;

    int tagId = 0;

    public Pose start = new Pose(63.163, 8.692, Math.toRadians(90));

    private MotorEx transfer1;

    private ServoEx transfer2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    private Boolean preloadspin;

    private double preloadtps;

    private double shoottps;
    Command findTPSShoot = new LambdaCommand()
            .setStart(() -> shoottps = findTPS(DistanceBlue.INSTANCE.getDistanceFromTag()));
    Command findTPSPreload = new LambdaCommand()
            .setStart(() -> preloadtps = findTPS(DistanceBlue.INSTANCE.getDistanceFromTag()));
    private static Servo hoodServo1n;
    private static Servo hoodServo2n;

    private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
    private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);

    public void onInit() {
        telemetry.addLine("Initializing Follower...");

        telemetry.update();
        follower = PedroComponent.follower();

        hoodServo1n= ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
        hoodServo2n=  ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");

        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();
        paths = new Paths(follower);
        intakeMotor = new MotorEx("intake");
        transfer1 = new MotorEx("transfer");
        transfer2 = new ServoEx("transferServo1");

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);

        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();

        follower.update();
    }

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(-1));

    Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0));

    double distance;

    Command transferOn = new LambdaCommand()
            .setStart(() -> transfer1.setPower(-1));

    Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));

    Command transferOnForIntake = new LambdaCommand()
            .setStart(() -> transfer1.setPower(-1));

    public Command opentransfer = new LambdaCommand()
            .setStart(() -> {
                transfer2.setPosition(0.3);
            });

    public Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                transfer2.setPosition(0.635);
            });

    public SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.05), transferOn, new Delay(0.4), transferOff, closeTransfer);

    public boolean spinup = true;
    public Command spinupfalse = new LambdaCommand()
            .setStart(() -> {
                spinup = false;
            });

    Command spinupPLEASEEIsagiINEEDTHIS = new LambdaCommand()
            .setStart(() -> {
                flywheel.setPower(1);
                flywheel2.setPower(-1);
            });

    public Command reverseIntakeForMe = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0.5));

    public boolean lift;
    boolean lowerangle = false;
    private Boolean preloadspinreal = false;

    public Command turnTo(Pose targetPose, double timeoutSeconds) {
        return new LambdaCommand("Turn to " + targetPose.getHeading())
                .setStart(() -> follower.holdPoint(targetPose))
                .setIsDone(() -> false)
                .raceWith(new Delay(timeoutSeconds));
    }

    Command preloadSpun = new LambdaCommand().setStart(() -> preloadspinreal = true);
    Command preloadSpunReal = new LambdaCommand().setStart(() -> preloadspinreal = false);

    public static double hoodToPos(double runtime) {
        if(!Double.isNaN(runtime)) {
            ActiveOpMode.telemetry().addData("runtime", runtime);
            ParallelGroup HoodRunUp = new ParallelGroup(
                    new SetPosition(hoodServo1, runtime),
                    new SetPosition(hoodServo2, -1*runtime)
            );
            HoodRunUp.schedule();
            return runtime;
        }
        else {
            ActiveOpMode.telemetry().addLine("NaN");
            return 0;
        }
    }

    public Command Auto() {
        return new SequentialGroup(
                new FollowPath(paths.ShootPreload1),
                new Delay(2),
                shoot,
                intakeMotorOn,
                transferOnForIntake,
                transferOn,

                new FollowPath(paths.Intake3rdSpike),
                transferOn,

                intakeMotorOff,
                new FollowPath(paths.Launch3rd),

                shoot,

                intakeMotorOn,
                transferOn,
                new FollowPath(paths.IntakeHigher),
                new Delay(1.0),
                intakeMotorOff,
                new FollowPath(paths.LaunchHigher),
                shoot,

                intakeMotorOn,
                transferOn,
                new FollowPath(paths.IntakeLoading),
                new Delay(1.0),
                intakeMotorOff,

                new FollowPath(paths.LaunchLoading),
                shoot,
                transferOn,
                intakeMotorOn,
                new FollowPath(paths.IntakeHigher),
                new Delay(1.0),
                intakeMotorOff,
                new FollowPath(paths.LaunchHigher),
                shoot,
                new FollowPath(paths.Park,true)
        );
    }

    public static boolean farzoneBlue = false;

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();

        preloadspinreal = true;
        shooter(1500);

        Auto().schedule();
        farzoneBlue = true;
    }

    @Override
    public void onUpdate() {
        follower.update();

        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();

        // Transformed the goal from (6, 138) to (144-6, 138) -> (138, 138)
        Vector robotToGoalVector = new Vector(follower.getPose().distanceFrom(new Pose(138, 138)), Math.atan2(138 - currPose.getY(), 138 - currPose.getX()));

        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity());
        double flywheelSpeed = results[0];
        shooter((float) flywheelSpeed);
        double hoodAngle = results[1];
        hoodToPos(hoodAngle);
    }

    @Override
    public void onStop() {
        follower.breakFollowing();
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }

    public class Paths {
        public PathChain ShootPreload1;
        public PathChain Intake3rdSpike;
        public PathChain Launch3rd;
        public PathChain IntakeHigher;
        public PathChain LaunchHigher;
        public PathChain IntakeLoading;
        public PathChain LaunchLoading;
        public PathChain Park;

        public Paths(Follower follower) {
            ShootPreload1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.500, 8.100),
                                    new Pose(58.000, 13.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(107))
                    .build();

            Intake3rdSpike = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.000, 13.000),
                                    new Pose(50.656, 35.774),
                                    new Pose(14.454, 36.361)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Launch3rd = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(14.454, 36.361),
                                    new Pose(57.860, 12.851)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(178), Math.toRadians(107))
                    .build();

            IntakeHigher = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.860, 12.851),
                                    new Pose(40.134, 15.455),
                                    new Pose(12, 14.314)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            LaunchHigher = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(12, 14.314),
                                    new Pose(57.986, 13.109)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(107))
                    .build();

            IntakeLoading = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.986, 13.109),
                                    new Pose(36.009, 9.247),
                                    new Pose(12, 9.896)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(107), Math.toRadians(180))
                    .build();

            LaunchLoading = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(12, 9.896),
                                    new Pose(58.384, 13.325)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(107))
                    .build();

            Park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(58.384, 13.325),
                                    new Pose(33.607, 12.961)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(107), Math.toRadians(90))
                    .build();
        }
    }
}