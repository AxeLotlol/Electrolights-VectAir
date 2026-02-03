package TRASH;


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


@Autonomous(name = "Red Auto 18 Ball SPam Linear", group = "Autonomous")
@Configurable
public class Red18BallSpamLinear extends NextFTCOpMode {
    public Red18BallSpamLinear(){
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
    private Boolean preloadspinreal = false;

    Command preloadSpun = new LambdaCommand().setStart(() -> preloadspinreal = true);
    Command preloadSpunReal = new LambdaCommand().setStart(() -> preloadspinreal = false);




    public Command Auto(){
        return new SequentialGroup(

                /*spinupPLEASEEIsagiINEEDTHIS,
                preloadSpun*/

                new FollowPath(paths.Path1,true,1.0),
                spinupfalse,
                intakeMotorOn,
                opentransfer,
                shoot,
                transferOnForIntake,
                intakeMotorOn,
                new FollowPath(paths.Path2,true,1.0),
                transferOff,
                new FollowPath(paths.Path3,true,1.0),
                new Delay(0.2),
                shoot,
                transferOnForIntake,
                new FollowPath(paths.Path4,true,0.9),
                new Delay(0.15),
                new FollowPath(paths.Path5,true,0.9),
                new Delay(1.3),
                reverseIntakeForMe,
                new FollowPath(paths.Path6,true,1.0),
                new Delay(0.2),
                shoot,

                transferOnForIntake,
                new FollowPath(paths.Path7,true,0.9),
                new Delay(0.15),

                new FollowPath(paths.Path8,true,0.9),
                new Delay(1.3),
                reverseIntakeForMe,
                new FollowPath(paths.Path9,true,1.0),
                new Delay(0.2),
                shoot,
                transferOnForIntake,
                new FollowPath(paths.Path10,false,1.0),
                transferOff,
                new FollowPath(paths.Path11,true,1.0),
                new Delay(0.2),
                shoot,
                transferOnForIntake,
                new FollowPath(paths.Path12,true,1.0),
                transferOff,
                new FollowPath(paths.Path13,true,1.0),
                new Delay(0.2),
                shoot,
                transferOff,
                intakeMotorOff,

                new FollowPath(paths.Path14,true,1.0)










        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        /*flywheel.setPower(1);
        flywheel2.setPower(-1);*/

        preloadspinreal = true;
        shooter(1155);

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

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;

        public PathChain Path14;


        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(119.722, 126.594), new Pose(85.758, 88.384))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(41))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.758, 88.384),
                                    new Pose(76.418, 58.029),
                                    new Pose(126.000, 58.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(126.000, 58.000),
                                    new Pose(93.612, 97.088),
                                    new Pose(85.758, 88.597)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(39))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.758, 88.597),
                                    new Pose(99.981, 64.822),
                                    new Pose(114.627, 60.152),
                                    new Pose(125.000, 68.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.000, 68.000), new Pose(130.000, 62.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.000, 62.000),
                                    new Pose(93.612, 97.087),
                                    new Pose(85.758, 88.597)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(39))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.758, 88.597),
                                    new Pose(99.981, 64.822),
                                    new Pose(114.627, 60.152),
                                    new Pose(125.000, 68.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.000, 68.000), new Pose(130.000, 62.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.000, 62.000),
                                    new Pose(93.612, 97.087),
                                    new Pose(85.758, 88.597)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(39))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.758, 88.597),
                                    new Pose(90.428, 86.474),
                                    new Pose(124.000, 85.200)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(124.000, 85.200),
                                    new Pose(93.400, 97.088),
                                    new Pose(85.121, 88.597)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(39))
                    .build();

            Path12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.121, 88.597),
                                    new Pose(74.083, 33.618),
                                    new Pose(124.359, 35.104)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path13 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(124.359, 35.104),
                                    new Pose(93.400, 96.239),
                                    new Pose(85.121, 88.597)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(39))
                    .build();
            Path14 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(85.121, 88.597), new Pose(108.896, 71.614))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

        }
    }
}