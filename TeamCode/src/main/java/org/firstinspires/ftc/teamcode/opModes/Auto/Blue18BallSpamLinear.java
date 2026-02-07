package org.firstinspires.ftc.teamcode.opModes.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
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
public class Blue18BallSpamLinear extends NextFTCOpMode {
    public Blue18BallSpamLinear(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE,DistanceRed.INSTANCE),
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

    public Pose start = new Pose(24.6,126.4, Math.toRadians(131));


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
            .setStart(()->shoottps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
    Command findTPSPreload = new LambdaCommand()
            .setStart(()->preloadtps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));


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
        hoodServo1n= ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
        hoodServo2n=  ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");

        follower.update();


    }

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

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(-1));

    Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0));

    double distance;

    Command transferOn = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-1));
    Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));
    Command transferOnForIntake = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-1));

    public Command opentransfer = new LambdaCommand()
            .setStart(()-> {
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
            .setStart(()-> {
                spinup=false;
            });

    Command spinupPLEASEEIsagiINEEDTHIS = new LambdaCommand()
            .setStart(()->{
                flywheel.setPower(1);
                flywheel2.setPower(-1);

            });
    public Command reverseIntakeForMe = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0.5));

    public boolean lift;
    boolean lowerangle = false;
    private Boolean preloadspinreal = false;

    Command preloadSpun = new LambdaCommand().setStart(() -> preloadspinreal = true);
    Command preloadSpunReal = new LambdaCommand().setStart(() -> preloadspinreal = false);
    public boolean intake3 = false;
    Command shootIntake3 = new LambdaCommand()
            .setStart(()->intake3=true);

    public boolean intakeSpam = false;

    Command shootIntakeSpam = new LambdaCommand()
            .setStart(()->intakeSpam=true);




    public Command Auto(){
        return new SequentialGroup(

                /*spinupPLEASEEIsagiINEEDTHIS,
                preloadSpun*/
                new Delay(0.5),
                new FollowPath(paths.preloadLaunch,true,1.0),
                shoot,
                intakeMotorOn,
                transferOn,

                new FollowPath(paths.intakeSet2,true,1.0),



                new FollowPath(paths.launchSet2,true,1.0),
                shoot,
                intakeMotorOn,
                transferOn,

                new FollowPath(paths.resetAndIntake1,true,1.0),
                new FollowPath(paths.moverBacker,true,1.0),
                new Delay(1.0),
                //reverseIntakeForMe,
                new FollowPath(paths.launchSpam1, true, 1.0),
                shoot,

                intakeMotorOn,
                transferOn,


                new FollowPath(paths.resetAndIntake2, true, 1.0),
                new Delay(1.75),
                //reverseIntakeForMe,

                new FollowPath(paths.launchSpam2, true, 1.0),
                shoot,

                intakeMotorOn,
                transferOn,
                new FollowPath(paths.intakeSet1,true,1.0),
                new FollowPath(paths.launchSet1,true,1.0),
                shoot,

                intakeMotorOn,
                transferOn,
                new FollowPath(paths.intakeSet3,true,1.0),
                new FollowPath(paths.launchSet3,true,1.0),
                shoot,

                new FollowPath(paths.teleOpPark,true,1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        /*flywheel.setPower(1);
        flywheel2.setPower(-1);*/

        preloadspinreal = true;
        shooter(1085);

        //int tag=MotifScanning.INSTANCE.findMotif();
        Auto().schedule();



    }

    @Override
    public void onUpdate(){
        follower.update();

        /*if(preloadspinreal) {
            shooter(1080);
        }
        else{
            if (DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
                shooter(findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
                ActiveOpMode.telemetry().addData("Limelight!", findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
            } else if (DistanceRed.INSTANCE.getDistanceFromTag() == 0 && !intake3&&!intakeSpam) {
                shooter(1065);
            }
            else if(intake3){
                shooter(1070);
            }
            else if(intakeSpam){
                shooter(1065);
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
        public PathChain preloadLaunch;
        public PathChain intakeSet2;
        public PathChain launchSet2;
        public PathChain resetAndIntake1;
        public PathChain moverBacker;
        public PathChain launchSpam1;
        public PathChain resetAndIntake2;
        public PathChain launchSpam2;
        public PathChain intakeSet1;
        public PathChain launchSet1;
        public PathChain intakeSet3;
        public PathChain launchSet3;
        public PathChain teleOpPark;

        public Paths(Follower follower) {
            // Preload Launch: X (119.4 -> 24.6), X (110 -> 34), X (100 -> 44)
            preloadLaunch = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(24.600, 126.400),
                                    new Pose(34.000, 110.500),
                                    new Pose(44.000, 100.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .setReversed()
                    .build();

            // intakeSet2: X (100 -> 44, 85 -> 59, 130 -> 14)
            // Heading: 46 -> 134, -15 -> 195
            intakeSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(44.000, 100.000),
                                    new Pose(59.000, 56.798),
                                    new Pose(10, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(195))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .build();

            // launchSet2: X (130 -> 14, 104 -> 40, 92 -> 52)
            // Heading: 0 -> 180, 47 -> 133
            launchSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10, 60.000),
                                    new Pose(40.000, 67.000),
                                    new Pose(52.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    .build();

            // resetAndIntake1: X (92 -> 52, 104 -> 40, 129 -> 15)
            // Heading: 47 -> 133, 23 -> 157
            resetAndIntake1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(52.000, 94.000),
                                    new Pose(40.000, 67.000),
                                    new Pose(11, 59.25)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(157))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeMotorOn)
                    .addTemporalCallback(0.1, transferOn)
                    .build();

            // moverBacker: X (129 -> 15, 125 -> 19)
            // Heading: 23 -> 157, 30 -> 150
            moverBacker = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11, 59.25),
                                    new Pose(13.233, 55.091)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(145))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .build();

            // launchSpam1: X (125 -> 19, 104 -> 40, 92 -> 52)
            // Heading: 30 -> 150, 44 -> 136
            launchSpam1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.233, 55.091),
                                    new Pose(40.000, 67.000),
                                    new Pose(52.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(136))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(20.0, 61.0), reverseIntakeForMe, 0.3) // 144 - 124 = 20
                    .build();

            // resetAndIntake2: X (92 -> 52, 104 -> 40, 128.75 -> 15.25)
            // Heading: 44 -> 136, 23 -> 157
            resetAndIntake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(52.000, 94.000),
                                    new Pose(40.000, 67.000),
                                    new Pose(11, 59.289)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(165))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeMotorOn)
                    .addTemporalCallback(0.1, transferOn)
                    .build();

            // launchSpam2: X (128.75 -> 15.25, 104 -> 40, 92 -> 52)
            // Heading: 23 -> 157, 47 -> 133
            launchSpam2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.448, 59.289),
                                    new Pose(40.000, 67.000),
                                    new Pose(52.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(165), Math.toRadians(133))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(26.0, 64.0), reverseIntakeForMe, 0.4) // 144 - 118 = 26
                    .build();

            // intakeSet1: X (92 -> 52, 104 -> 40, 124.665 -> 19.335)
            intakeSet1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(52.000, 94.000),
                                    new Pose(40.000, 87.000),
                                    new Pose(17, 86.898)
                            )
                    ).setTangentHeadingInterpolation()
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeMotorOn)
                    .addTemporalCallback(0.1, transferOn)
                    .build();

            // launchSet1: X (124.665 -> 19.335, 92 -> 52)
            // Heading: 1 -> 179, 47 -> 133
            launchSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17, 86.898),
                                    new Pose(52.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(179), Math.toRadians(133))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(42.0, 86.0), reverseIntakeForMe, 0.4) // 144 - 102 = 42
                    .build();

            // intakeSet3: X (92 -> 52, 88 -> 56, 75 -> 69, 129.151 -> 14.849)
            // Heading: 44 -> 136, 0 -> 180
            intakeSet3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(52.000, 94.000),
                                    new Pose(56.000, 89.500),
                                    new Pose(69.000, 28.615),
                                    new Pose(8, 34.938)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeMotorOn)
                    .addTemporalCallback(0.1, transferOn)
                    .build();

            // launchSet3: X (129.151 -> 14.849, 92 -> 52)
            // Heading: 1 -> 179, 45 -> 135
            launchSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8, 34.938),
                                    new Pose(52.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(179), Math.toRadians(135))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(35.0, 59.0), reverseIntakeForMe, 0.55) // 144 - 109 = 35
                    .build();

            // teleOpPark: X (92 -> 52, 89.168 -> 54.832)
            // Heading: 180 - 90 = 90
            teleOpPark = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(52.000, 94.000),
                                    new Pose(54.832, 63.953)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }
}