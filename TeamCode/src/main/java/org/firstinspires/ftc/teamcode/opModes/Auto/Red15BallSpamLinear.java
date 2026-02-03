package org.firstinspires.ftc.teamcode.opModes.Auto;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;

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
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;


@Autonomous
@Configurable
public class Red15BallSpamLinear extends NextFTCOpMode {
    public Red15BallSpamLinear(){
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

    public Pose start = new Pose(119.4,126.4, Math.toRadians(49));


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

    public SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.15), transferOn, new Delay(0.97), transferOff, closeTransfer);

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

                new FollowPath(paths.PreloadLaunch,true,1.0),
                spinupfalse,
                intakeMotorOn,


                opentransfer,
                new Delay(0.5),


                shoot,
                transferOnForIntake,
                preloadSpunReal,
                new FollowPath(paths.intakeSet2,true,1.0),

                new FollowPath(paths.launchSet2,true,1.0),
                intakeMotorOn,
                shoot,
                intakeMotorOff,
                transferOnForIntake,
                new FollowPath(paths.resetHelper,true,1.0),
                shootIntakeSpam,
                new Delay(0.2),
                intakeMotorOn,
                new FollowPath(paths.resetIntakeSpam, true, 1.0),
                new Delay(1.6),
                transferOff,


                new FollowPath(paths.launchSpam,true,0.9),
                intakeMotorOn,
                shoot,
                transferOnForIntake,
                new FollowPath(paths.intakeSet1,true,1.0),


                new FollowPath(paths.launchSet1,true,1.0),
                intakeMotorOn,
                shoot,
                transferOnForIntake,
                shootIntake3,
                new FollowPath(paths.intakeSet3,true,1.0),


                new FollowPath(paths.launchSet3,true,1.0),
                shoot,
                new FollowPath(paths.teleOpPar,true,1.0)
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
        Vector robotToGoalVector = new Vector(follower.getPose().distanceFrom(new Pose(138, 138)), Math.atan2(138 - currPose.getY(), 138 - currPose.getX()));
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

        public PathChain PreloadLaunch;
        public PathChain intakeSet2;
        public PathChain launchSet2;
        public PathChain resetHelper;
        public PathChain resetIntakeSpam;
        public PathChain launchSpam;
        public PathChain intakeSet1;
        public PathChain launchSet1;
        public PathChain intakeSet3;
        public PathChain launchSet3;
        public PathChain teleOpPar;


        public Paths(Follower follower) {
            PreloadLaunch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.400, 126.400),

                                    new Pose(93.774, 95.871)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(41))


                    .build();

            intakeSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(93.774, 95.871),
                                    new Pose(85.000, 86.907),
                                    new Pose(85.183, 65.967),
                                    new Pose(125.000, 65.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            launchSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.000, 65.000),
                                    new Pose(87.700, 55.500),
                                    new Pose(93.774, 95.871)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-3), Math.toRadians(41))

                    .addTemporalCallback(1.3,transferOff)

                    .build();

            resetHelper = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(93.774, 95.871),
                                    new Pose(85.779, 74.456),
                                    new Pose(98.938, 60.551),
                                    new Pose(103.305, 69.294),
                                    new Pose(126.000, 69.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    //.setConstantHeadingInterpolation(45)

                    .build();

            resetIntakeSpam = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.000, 68.000),
                                    new Pose(126.374, 62.813),
                                    new Pose(127.000, 59)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(49))


                    .build();

            launchSpam = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(126.000, 56.500),
                                    new Pose(126.000, 58.000),
                                    new Pose(107.400, 56.000),
                                    new Pose(86.512, 90.001)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(47), Math.toRadians(39))
                    .addTemporalCallback(2.3,reverseIntakeForMe)
                    //.addPoseCallback(107.4,reverseIntakeForMe,0.6,)
                    .addTemporalCallback(1.67,transferOff)

                    .build();

            intakeSet1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.512, 90.001),
                                    new Pose(88.903, 87.056),
                                    new Pose(101.500, 86.000),
                                    new Pose(118.665, 86.898)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(41), Math.toRadians(0))

                    .build();

            launchSet1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(118.665, 86.898),
                                    new Pose(101.500, 84.000),
                                    new Pose(93.774, 95.871)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))

                    //.addTemporalCallback(2,transferOff)

                    .build();

            intakeSet3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(93.774, 95.871),
                                    new Pose(95.280, 84.031),
                                    new Pose(84.682, 44.648),
                                    new Pose(124.500, 43.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            launchSet3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(124.500, 43.000),
                                    new Pose(100.003, 43.010),
                                    new Pose(93.774, 95.871)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(37))
                    //.addTemporalCallback(1.7,reverseIntakeForMe)
                    .addTemporalCallback(1.7,transferOff)

                    .build();

            teleOpPar = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(93.774, 95.871),

                                    new Pose(87.790, 109.420)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();
        }
    }

}