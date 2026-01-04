

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;


@Autonomous(name = "Blue Auto 15 Ball Spam Moksh", group = "Autonomous")
@Configurable
public class Blue15BallSpam extends NextFTCOpMode {
    public Blue15BallSpam(){
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


    public Pose start = new Pose(119.4, 126.4, Math.toRadians(49));


    public Pose preloadA = new Pose(119.400, 126.400);
    public Pose preloadB = new Pose(82.600, 87.200);


    public Pose intake2A = new Pose(82.600, 87.200);
    public Pose intake2B = new Pose(85.000, 87.000);
    public Pose intake2C = new Pose(84.000, 61.000);
    public Pose intake2D = new Pose(127.000, 59.000);


    public Pose launch2A = new Pose(127.000, 59.000);
    public Pose launch2B = new Pose(87.700, 55.500);
    public Pose launch2C = new Pose(97.500, 103.500);
    public Pose launch2D = new Pose(82.600, 87.200);


    public Pose resetA = new Pose(82.600, 87.200);
    public Pose resetB = new Pose(80.400, 86.400);
    public Pose resetC = new Pose(97.200, 56.200);
    public Pose resetD = new Pose(106.300, 61.000);
    public Pose resetE = new Pose(123.000, 65.000);

    // ===== RESET INTAKE SPAM =====
    public Pose resetSpamA = new Pose(123.000, 65.000);
    public Pose resetSpamB = new Pose(126.000, 60.000);


    public Pose launchSpamA = new Pose(126.000, 60.000);
    public Pose launchSpamB = new Pose(113.400, 50.000);
    public Pose launchSpamC = new Pose(96.000, 102.500);
    public Pose launchSpamD = new Pose(82.600, 87.200);


    public Pose intake1A = new Pose(82.600, 87.200);
    public Pose intake1B = new Pose(91.500, 97.300);
    public Pose intake1C = new Pose(101.500, 84.000);
    public Pose intake1D = new Pose(125.000, 84.000);

    public Pose launch1A = new Pose(125.000, 84.000);
    public Pose launch1B = new Pose(101.500, 84.000);
    public Pose launch1C = new Pose(92.100, 97.700);
    public Pose launch1D = new Pose(82.600, 87.200);



    public Pose intake3A = new Pose(82.600, 87.200);
    public Pose intake3B = new Pose(88.000, 89.500);
    public Pose intake3C = new Pose(84.000, 39.500);
    public Pose intake3D = new Pose(120.000, 40.000);

    public Pose launch3A = new Pose(120.000, 40.000);
    public Pose launch3B = new Pose(100.000, 43.000);
    public Pose launch3C = new Pose(95.000, 101.000);
    public Pose launch3D = new Pose(82.600, 87.200);


    public Pose teleOpA = new Pose(82.600, 87.200);
    public Pose teleOpB = new Pose(108.000, 72.000);



    private MotorEx transfer1;

    private ServoEx transfer2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");


    public void onInit() {
        telemetry.addLine("Initializing Follower...");

        telemetry.update();
        follower = PedroComponent.follower();
        /*Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();*/


        start.mirror();

        preloadA.mirror();
        preloadB.mirror();

        intake2A.mirror();
        intake2B.mirror();
        intake2C.mirror();
        intake2D.mirror();

        launch2A.mirror();
        launch2B.mirror();
        launch2C.mirror();
        launch2D.mirror();

        resetA.mirror();
        resetB.mirror();
        resetC.mirror();
        resetD.mirror();
        resetE.mirror();

        resetSpamA.mirror();
        resetSpamB.mirror();

        launchSpamA.mirror();
        launchSpamB.mirror();
        launchSpamC.mirror();
        launchSpamD.mirror();

        intake1A.mirror();
        intake1B.mirror();
        intake1C.mirror();
        intake1D.mirror();

        launch1A.mirror();
        launch1B.mirror();
        launch1C.mirror();
        launch1D.mirror();

        intake3A.mirror();
        intake3B.mirror();
        intake3C.mirror();
        intake3D.mirror();

        launch3A.mirror();
        launch3B.mirror();
        launch3C.mirror();
        launch3D.mirror();

        teleOpA.mirror();
        teleOpB.mirror();



        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();


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

    Command distance = new LambdaCommand()
            .setStart(()-> findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));

    Command transferOn = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-0.5));
    Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));

    public Command opentransfer = new LambdaCommand()
            .setStart(()-> {
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.3);
            });

    public Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                transfer2.setPosition(0.7);
            });

    public SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.35), transferOn, new Delay(0.65), transferOff, closeTransfer);

    public boolean spinup = true;
    public Command spinupfalse = new LambdaCommand()
            .setStart(()-> {
                spinup=false;
            });
    public Command Auto(){
        return new SequentialGroup(
            new FollowPath(paths.PreloadLaunch,true,1.0),
            spinupfalse,
            intakeMotorOn,
            new Delay(1),
            shoot,
            transferOn,
            new FollowPath(paths.intakeSet2,true,1.0),
            transferOff,
            new FollowPath(paths.launchSet2,true,1.0),
            shoot,
            transferOn,
            new FollowPath(paths.resetHelper,true,1.0),
            new FollowPath(paths.resetIntakeSpam, true, 1.0),
            new Delay(0.8),
            transferOff,
            new FollowPath(paths.launchSpam,true,1.0),
            shoot,
            transferOn,
            new FollowPath(paths.intakeSet1,true,1.0),
            transferOff,
            new FollowPath(paths.launchSet1,true,1.0),
            shoot,
            transferOn,
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
        //int tag=MotifScanning.INSTANCE.findMotif();
        Auto().schedule();


    }

    @Override
    public void onUpdate(){
        if(spinup==true){
            flywheel.setPower(1);
        }
        else if(spinup==false){
            shooter(1140);
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


            PreloadLaunch = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            preloadA,
                            preloadB
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


            intakeSet2 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            intake2A,
                            intake2B,
                            intake2C,
                            intake2D
                    ))
                    .setTangentHeadingInterpolation()
                    .build();


            launchSet2 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            launch2A,
                            launch2B,
                            launch2C,
                            launch2D
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


            resetHelper = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            resetA,
                            resetB,
                            resetC,
                            resetD,
                            resetE
                    ))
                    .setTangentHeadingInterpolation()
                    .build();


            resetIntakeSpam = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            resetSpamA,
                            resetSpamB
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(40))
                    .build();


            launchSpam = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            launchSpamA,
                            launchSpamB,
                            launchSpamC,
                            launchSpamD
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


            intakeSet1 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            intake1A,
                            intake1B,
                            intake1C,
                            intake1D
                    ))
                    .setTangentHeadingInterpolation()
                    .build();


            launchSet1 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            launch1A,
                            launch1B,
                            launch1C,
                            launch1D
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();



            intakeSet3 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            intake3A,
                            intake3B,
                            intake3C,
                            intake3D
                    ))
                    .setTangentHeadingInterpolation()
                    .build();


            launchSet3 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            launch3A,
                            launch3B,
                            launch3C,
                            launch3D
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


            teleOpPar = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            teleOpA,
                            teleOpB
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
    }

}