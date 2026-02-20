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
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
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


@Autonomous(name = "Red Far Zone", group = "Autonomous")
@Configurable
public class redFarZone extends NextFTCOpMode {
    public redFarZone() {
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
    public MotorEx intakeMotor;

    int tagId = 0;

    public Pose start = new Pose(80.837, 8.692, Math.toRadians(90));


    private MotorEx transfer1;

    private ServoEx transfer2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

   /* private CRServo hoodServo1n;
    private CRServo hoodServo2n;

    private CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);






    ParallelGroup HoodRunUp=new ParallelGroup(
            new SetPower(hoodServo1,-1),
            new SetPower(hoodServo2,1)
    );

    public ParallelGroup HoodPowerZero=new ParallelGroup(
            new SetPower(hoodServo1,0),
            new SetPower(hoodServo2,0)
    );

    public SequentialGroup HoodUp=new SequentialGroup(
            HoodRunUp,
            new Delay(0.18),
            HoodPowerZero
    );

    ParallelGroup HoodRunDown=new ParallelGroup(
            new SetPower(hoodServo1,1),
            new SetPower(hoodServo2,-1)
    );

    public SequentialGroup HoodDown=new SequentialGroup(
            HoodRunDown,
            new Delay(0.17),
            HoodPowerZero
    );

    public SequentialGroup HoodUpAuto=new SequentialGroup(
            HoodRunUp,
            new Delay(0.12),
            HoodPowerZero
    );*/

    private Boolean preloadspin;

    private double preloadtps;

    private double shoottps;
    Command findTPSShoot = new LambdaCommand()
            .setStart(() -> shoottps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
    Command findTPSPreload = new LambdaCommand()
            .setStart(() -> preloadtps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
    private static Servo hoodServo1n;
    private static Servo hoodServo2n;

    private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
    private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);
    public void onInit() {
        telemetry.addLine("Initializing Follower...");

        telemetry.update();
        follower = PedroComponent.follower();
        /*Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();*/

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
        //hoodServo1n= ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo1");
        // hoodServo2n=  ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo2");
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
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.3);
            });

    public Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                transfer2.setPosition(0.7);
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
        if(Double.isNaN(runtime)!=true) {
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
                new FollowPath(paths.Path1),

                new Delay(3.0),
                shoot,
                new Delay(0.5),
                intakeMotorOn,
                transferOnForIntake,
                //turnTo(new Pose(140,7),0.7),

                new FollowPath(paths.Path2),
                new Delay(2.0),
                intakeMotorOff,
                new FollowPath(paths.Path3),
                shoot,
                new Delay(0.5),
                intakeMotorOn,
                new FollowPath(paths.Path2,false,1.0),
                new Delay(2.0),

                intakeMotorOff,
                new FollowPath(paths.Path3),
                shoot,
                new FollowPath(paths.Path4)


                //new FollowPath(paths.Leave)


        );
    }

    public static boolean farzoneRed = false;

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        /*flywheel.setPower(1);
        flywheel2.setPower(-1);*/

        preloadspinreal = true;
        shooter(1500);

        //int tag=MotifScanning.INSTANCE.findMotif();
        Auto().schedule();

        farzoneRed = true;


    }

    @Override
    public void onUpdate() {
        follower.update();

        /*if (preloadspinreal) {
            shooter(1285);
        } else {
            if (DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
                shooter(findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
                ActiveOpMode.telemetry().addData("Limelight!", findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
            } else if (DistanceRed.INSTANCE.getDistanceFromTag() == 0) {
                shooter(1285);
            }
        }*/
        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();
        Vector robotToGoalVector = new Vector(follower.getPose().distanceFrom(new Pose(6, 138)), Math.atan2(138 - currPose.getY(), 6 - currPose.getX()));
        //Vector v = new Vector(new Pose(138, 138));
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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.501, 8.113),

                                    new Pose(86, 11.372)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.030, 11.372),

                                    new Pose(131, 9.167)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(-3))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.728, 9.167),

                                    new Pose(86, 11.372)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-3), Math.toRadians(67))

                    .build();
            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(86, 11.372),
                            new Pose(96,8.5)

                    )

            )
                    .setLinearHeadingInterpolation(Math.toRadians(67),Math.toRadians(90))
                    .build();
        }
    }
}