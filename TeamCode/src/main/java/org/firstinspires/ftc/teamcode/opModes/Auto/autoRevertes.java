package org.firstinspires.ftc.teamcode.opModes.Auto;


import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.MotifScanning;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
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
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.powerable.SetPower;


@Autonomous(name = "Red Auto REVERTED BACK TO DECEMBER 23RD Default Position", group = "Auto+nomous")
@Configurable
public class autoRevertes extends NextFTCOpMode {
    public autoRevertes() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, DistanceRed.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))


        );
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    private Paths paths;

    public Pose start = new Pose(125.79827045231767, 125.69678743996302, Math.toRadians(39));

    public Pose PreloadCP = new Pose(118.82561804849406, 122.21046123805121);

    public Pose PreLoadLaunch1 = new Pose(110.78558507577333, 115.9135913662864);

    public Pose intake1CP1 = new Pose(108.89607353678281, 84.44081612930407);

    //public Pose intake1CP2 = new Pose(125,79.5);

    public Pose Intake1stSetClassifier = new Pose(129.86565102121477, 84.24612836558748);

    public Pose launchCP1 = new Pose(98.46728971962617, 103.40186915887851);

    public Pose Launch1 = new Pose(89.15468008859412, 93.26674156863335);

    public Pose intake2CP = new Pose(101.39398703893502, 66.13871482396966);
    public Pose intake2CP2 = new Pose(111.27191127768515, 59.96606242014606);

    public Pose intake2 = new Pose(128.99406947073683, 63.78080707157992);

    public Pose launch2Cp = new Pose(90.64448124970694, 92.57668852180086);

    public Pose launch2 = new Pose(77.60747663551402, 79.17757009345794);

    public Pose intake3CP = new Pose(86.57710068080985, 30.984925621358947);

    public Pose intake3 = new Pose(123.189868489156, 30.113344070881013);

    public Pose launch3CP = new Pose(92.96869871764815, 100.42092247610242);

    public Pose launch3 = new Pose(80.60747663551402, 80.17757009345794, Math.toRadians(50));

    public Pose loadingZoneCP1 = new Pose(116.11335716300232, 75.43580555091452);

    public Pose loadingZoneCP2 = new Pose(140.73202013289117, 42.32121008943673);

    public Pose loadingZone = new Pose(135.67065489374124, 9.055706819278855);

    public Pose loadingZoneLaunchcp1 = new Pose(94.24923323651377, 98.785840812213);

    public Pose loadingZoneLaunch = new Pose(74.69158878504672, 72.89719626168224);

    public Pose Teleop1 = new Pose(100.11214953271028, 80.009345794392516);


    private static final int APRILTAG_PIPELINE = 8;


    private MotorEx FlywheelReal;


    private MotorEx intakeMotor;

    // private MotorEx spindexerMotor;

    private boolean path2Following = false;
    int ball1Color = 0;
    int ball2Color = 0;
    int ball3Color = 0;
    int tagId = 0;


    public static double spindexvelocity;


    Command flywheelYes = new LambdaCommand()
            .setStart(() -> shooter(1500));


    /*Command pathCommand = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake1set))
            .setIsDone(() -> !follower.isBusy())
            .named("Path2 Command");*/
    Command pathCommand2 = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake2ndSet))
            .setIsDone(() -> !follower.isBusy())
            .named("Path4 Command");
    Command pathCommand3 = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake3rdSet))
            .setIsDone(() -> !follower.isBusy())
            .named("Path4 Command");


    public static void velocityControlWithFeedforwardExample(KineticState currentstate, float configtps) {
        ControlSystem controller = ControlSystem.builder()
                .velPid(0.1, 0.01, 0.05) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(0.0067, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        controller.setGoal(new KineticState(0.0, configtps, 0.0));

        double power = controller.calculate(currentstate);

    }

    private MotorEx transfer1;

    private int newtps;
    private ServoEx transfer2;
    private ServoEx transfer3;
    private ServoEx hoodservo1;
    private ServoEx hoodservo2;
    private Command shootClose;
    private Command intakeMotorOn;

    private Command shootFar;

    private CRServo hoodServo1n;
    private CRServo hoodServo2n;

    private CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);

    private boolean shootClose1;

    public void onInit() {
        telemetry.addLine("Initializing Follower...");


        telemetry.update();
        follower = PedroComponent.follower();
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();


        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();


        paths = new Paths(follower);
        intakeMotor = new MotorEx("intake");
        transfer1 = new MotorEx("transfer");
        transfer2 = new ServoEx("transferServo1");

        transfer3 = new ServoEx("transferServo2");
        hoodservo1 = new ServoEx("hoodServo1");
        hoodservo2 = new ServoEx("hoodServo2");

        FlywheelReal = new MotorEx("launchingmotor");


        shootClose = new LambdaCommand()
                .setStart(() -> {
                    shooter(1250);
                    shootClose1 = true;


                });
        shootFar = new LambdaCommand()
                .setStart(() -> {
                    shooter(1330);
                    shootClose1 = false;
                });
        intakeMotorOn = new LambdaCommand()
                .setStart(() -> intakeMotor.setPower(-1));
        hoodServo1n = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo1");
        hoodServo2n = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo2");
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);


        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
        shootClose1 = true;

        follower.update();


    }


    Command stopFlywheel = new LambdaCommand()
            .setStart(() -> shooter(0));

    Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0));
    Command hoodUp = new LambdaCommand()
            .setStart(() -> hoodservo1.setPosition(0.2));
    Command hoodup2 = new LambdaCommand()
            .setStart(() -> hoodservo2.setPosition(0.2));
    Command getMotif = new LambdaCommand()
            .setStart(() -> tagId = MotifScanning.INSTANCE.findMotif());

    Command distance = new LambdaCommand()
            .setStart(() -> findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));

    Command opentransfer = new LambdaCommand()
            .setStart(() -> {
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.3);
                //transfer3.setPosition(0.75);
            });
    Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                //transfer2.setPosition(1);
                transfer2.setPosition(0.7);
                //transfer3.setPosition(0);
            });

    Command transferOn = new LambdaCommand()
            .setStart(() -> transfer1.setPower(-1));
    Command transferOnIntake = new LambdaCommand()
            .setStart(() -> transfer1.setPower(-0.6));
    Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));
    Command shootCloseyipeee = new LambdaCommand()
            .setStart(() -> shooter(1250));

    /*Command shootByTag1 = new LambdaCommand()
                .setStart(() -> {
                if (tagId == 21) {
                    opentransfer.schedule();
                    transfer1.setPower(0.5);
                    new Delay(0.4);
                    new ParallelGroup(
                            hoodUp,
                            hoodup2+

                    );


                }


            });*/
    ParallelGroup HoodRunUp = new ParallelGroup(
            new SetPower(hoodServo1, 1),
            new SetPower(hoodServo2, -1)
    );
    public ParallelGroup HoodPowerZero = new ParallelGroup(
            new SetPower(hoodServo1, 0),
            new SetPower(hoodServo2, 0)
    );

    public SequentialGroup HoodUp = new SequentialGroup(
            HoodRunUp,
            new Delay(0.1),
            HoodPowerZero
    );

    ParallelGroup HoodRunDown = new ParallelGroup(
            new SetPower(hoodServo1, -1),
            new SetPower(hoodServo2, 1)
    );

    public SequentialGroup HoodDown = new SequentialGroup(
            HoodRunDown,
            new Delay(0.17),
            HoodPowerZero
    );
    Command spinUp = new LambdaCommand()
            .setStart(() -> FlywheelReal.setPower(-1));


    public Command Auto() {
        return new SequentialGroup(


                spinUp,
                new Delay(0.3),
                //shootCloseyipeee,
                shootClose,
                //intakeMotorOn,
                opentransfer,
                //new Delay(1.5),
                new FollowPath(paths.PreLoadLaunch, true, 0.99),


                transferOn,
                new Delay(1.5),

                transferOff,
                //new TurnTo(Angle.fromDeg(90)),
                //getMotif,
                //shootFar,
                closeTransfer,


                intakeMotorOn,


                new Delay(0.2),
                transferOnIntake,
                intakeMotorOn,
                new FollowPath(paths.IntakeandClassifier, false, 0.9),
                transferOff,
                //new Delay(1.0),

                /*new FollowPath(paths.ClassifierRamp1,true,0.8),
                new Delay(0.5),
                //transferOff,
                //new Delay(0.1),
                // Sorting logic all here with the order, etc
                */
                //intakeMotorOn,
                new FollowPath(paths.Launch1Real, true, 0.9),

                opentransfer,
                new Delay(0.2),
                transferOn,


                // Transfer logic with transfer
                new Delay(1.5),
                transferOff,
                closeTransfer,
                new Delay(0.3),

                intakeMotorOn,
                transferOnIntake,
                shootFar,
                new FollowPath(paths.Intake2ndSet, false, 0.8),

                transferOff,
                new Delay(0.3),
                //opentransfer,

                //intakeMotorOff,

                // Sorting logic and order here


                new FollowPath(paths.Launch2, true, 0.9),
                opentransfer,
                intakeMotorOn,


                transferOn,

                // Transfer logic with transfer
                new Delay(1.75),
                closeTransfer,
                transferOff,


                intakeMotorOn,
                transferOnIntake,
                new FollowPath(paths.Intake3rdSet, false, 0.9),

                transferOff,


                // Sorting logic here
                new FollowPath(paths.Launch3, false, 0.9),
                opentransfer,
                new Delay(0.3),

                // Transfer and shoot logic
                opentransfer,
                transferOn,
                intakeMotorOff,

                new Delay(1.5),
                //closeTransfer,
                transferOff,
                closeTransfer,

                intakeMotorOn,
                transferOn,

                new FollowPath(paths.LoadingZone, false, 0.9),
                transferOff,
                new Delay(0.4),


                new FollowPath(paths.LoadingZoneLaunch, true, 0.9),
                opentransfer,
                transferOn,
                new Delay(1.5),
                opentransfer,
                transferOff,
                stopFlywheel,
                new FollowPath(paths.teleOp, true, 0.9)


        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        //int tag=MotifScanning.INSTANCE.findMotif();
        Auto().schedule();


        telemetry.addLine("The shooter has started btw");
        telemetry.addLine("Started Path 1");
        telemetry.update();


    }

    @Override
    public void onUpdate() {

        if (shootClose1) {
            shooter(1250);

        } else if (!shootClose1) {
            shooter(1300);
        }


        follower.update();


    }


    @Override
    public void onStop() {
        follower.breakFollowing();
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }

    public class Paths {
        public PathChain PreLoadLaunch;
        public PathChain IntakeandClassifier;

        //public PathChain ClassifierRamp1;

        public PathChain Launch1Real;

        public PathChain Intake2ndSet;

        public PathChain Launch2;

        public PathChain Intake3rdSet;

        public PathChain Launch3;

        public PathChain LoadingZone;

        public PathChain LoadingZoneLaunch;

        public PathChain teleOp;


        public Paths(Follower follower) {
            //Path1 = follower.pathBuilder()
            //.addPath(new BezierLine(
            // new Pose(56.000, 8.000),
            // new Pose(57.078, 32.451)
            //))
            //.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            //.build();

            PreLoadLaunch = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            start,
                            PreloadCP,
                            PreLoadLaunch1
                    ))
                    .setTangentHeadingInterpolation().setReversed()
                    //.setVelocityConstraint(50)
                    .build();
            IntakeandClassifier = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            PreLoadLaunch1,
                            intake1CP1,
                            Intake1stSetClassifier

                    ))
                    .setTangentHeadingInterpolation()

                    .build();
           /* ClassifierRamp1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Intake1,
                            ClassifierRampPoint,
                            ClassifierRamp


                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();*/
            Launch1Real = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Intake1stSetClassifier,
                            launchCP1,
                            Launch1

                    ))
                    .setTangentHeadingInterpolation().setReversed()

                    .build();
            Intake2ndSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Launch1,
                            intake2CP,
                            intake2CP2,
                            intake2

                    ))
                    .setTangentHeadingInterpolation()

                    .build();
            Launch2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            intake2,
                            launch2Cp,
                            launch2
                    ))
                    .setTangentHeadingInterpolation().setReversed()

                    .build();
            Intake3rdSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            launch2,
                            intake3CP,
                            intake3

                    ))
                    .setTangentHeadingInterpolation()

                    .build();
            Launch3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            intake3,
                            launch3CP,
                            launch3
                    ))
                    .setTangentHeadingInterpolation().setReversed()

                    .build();

            LoadingZone = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            launch3,
                            loadingZoneCP1,
                            loadingZoneCP2,
                            loadingZone
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            LoadingZoneLaunch = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            loadingZone,
                            loadingZoneLaunchcp1,
                            loadingZoneLaunch
                    ))
                    .setTangentHeadingInterpolation().setReversed()
                    .build();
            teleOp = follower.pathBuilder()
                    .addPath(new BezierLine(
                            launch3,
                            Teleop1

                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

        }
    }
}