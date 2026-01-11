package org.firstinspires.ftc.teamcode.opModes.Auto;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
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
import dev.nextftc.hardware.powerable.SetPower;


@Autonomous(name = "Red Auto 15 Ball SPam Linear", group = "Autonomous")
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
    private MotorEx intakeMotor;

    int tagId = 0;

    public Pose start = new Pose(119.4,126.4, Math.toRadians(49));


    private MotorEx transfer1;

    private ServoEx transfer2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    private CRServo hoodServo1n;
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
    );

    private Boolean preloadspin;

    private double preloadtps;

    private double shoottps;
    Command findTPSShoot = new LambdaCommand()
            .setStart(()->shoottps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
    Command findTPSPreload = new LambdaCommand()
            .setStart(()->preloadtps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));

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
        hoodServo1n= ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo1");
        hoodServo2n=  ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo2");
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

    public SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.35), transferOn, new Delay(0.67), transferOff, closeTransfer);

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
    private Boolean preloadspinreal;

    Command preloadSpun = new LambdaCommand().setStart(() -> preloadspinreal = true);
    Command preloadSpunReal = new LambdaCommand().setStart(() -> preloadspinreal = false);

    public Command Auto(){
        return new SequentialGroup(

                spinupPLEASEEIsagiINEEDTHIS,
                preloadSpun,

                new FollowPath(paths.PreloadLaunch,true,1.0),
                spinupfalse,
                intakeMotorOn,


                opentransfer,
                new Delay(1.15),

                new Delay(0.5),
                shoot,
                transferOnForIntake,
                preloadSpunReal,
                new FollowPath(paths.intakeSet2,true,0.9),

                transferOff,
                new FollowPath(paths.launchSet2,true,0.9),
                shoot,
                transferOnForIntake,
                new FollowPath(paths.resetHelper,true,1.0),
                new Delay(0.3),
                new FollowPath(paths.resetIntakeSpam, true, 1.0),
                new Delay(1.3),
                transferOff,
                reverseIntakeForMe,

                new FollowPath(paths.launchSpam,true,0.9),
                intakeMotorOn,
                shoot,
                transferOnForIntake,
                new FollowPath(paths.intakeSet1,true,1.0),
                transferOff,
                new FollowPath(paths.launchSet1,true,1.0),
                shoot,
                transferOnForIntake,
                new FollowPath(paths.intakeSet3,true,1.0),

                transferOff,
                new FollowPath(paths.launchSet3,true,1.0),
                shoot,
                new FollowPath(paths.teleOpPar,true,1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        flywheel.setPower(1);
        flywheel2.setPower(-1);

        //int tag=MotifScanning.INSTANCE.findMotif();
        Auto().schedule();



    }

    @Override
    public void onUpdate(){
        follower.update();

        if(preloadspinreal) {
            shooter(1155);
        }
        else{
            if (DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
                shooter(findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
                ActiveOpMode.telemetry().addData("Limelight!", findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
            } else if (DistanceRed.INSTANCE.getDistanceFromTag() == 0) {
                shooter(1122);
            }
        }

    }




    @Override
    public void onStop() {
        follower.breakFollowing();
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }

    public static class Paths {
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

                                    new Pose(82.600, 87.200)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(41))

                    .build();

            intakeSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(85.000, 87.000),
                                    new Pose(84.000, 62.000),
                                    new Pose(124.000, 62.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            launchSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(124.000, 59.000),
                                    new Pose(87.700, 55.500),
                                    new Pose(82.600, 87.200)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-3), Math.toRadians(39))
                    .build();

            resetHelper = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(80.400, 86.400),
                                    new Pose(97.200, 56.200),
                                    new Pose(106.300, 62.000),
                                    new Pose(125.000, 68.500)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            resetIntakeSpam = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 68.500),

                                    new Pose(130.000, 62.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(45))

                    .build();

            launchSpam = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130.000, 62.000),
                                    new Pose(107.400, 50.000),
                                    new Pose(82.600, 87.200)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(29), Math.toRadians(39))


                    .build();

            intakeSet1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(91.500, 97.300),
                                    new Pose(101.500, 87.000),
                                    new Pose(124.665, 86.898)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            launchSet1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.665, 86.898),
                                    new Pose(101.500, 84.000),
                                    new Pose(82.600, 87.200)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(39))
                    .build();

            intakeSet3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(88.000, 89.500),
                                    new Pose(84.000, 42.500),
                                    new Pose(124.000, 43.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            launchSet3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(124.000, 40.000),
                                    new Pose(100.000, 43.000),
                                    new Pose(82.600, 87.200)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(39))
                    .build();

            teleOpPar = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.600, 87.200),

                                    new Pose(108.000, 72.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();
        }
    }
}