

package org.firstinspires.ftc.teamcode.opModes.Auto;


import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.powerable.SetPower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;


import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.MotifScanning;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;


@Autonomous(name = "Red15SpamTest", group = "Autonomous")
@Configurable
public class red15Spam extends NextFTCOpMode {
    public red15Spam(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))


        );
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;




    private Paths paths;
    public Pose start = new Pose(119.4,126.4, Math.toRadians(49));

    private static final int APRILTAG_PIPELINE = 8;








    private MotorEx intakeMotor;

    int tagId = 0;




    private MotorEx transfer1;
    private static ServoEx transfer2;
    private ServoEx transfer3;
    private ServoEx hoodservo1;
    private ServoEx hoodservo2;
    private Command spinFlyWheel1500;
    private Command intakeMotorOn;

    private CRServo hoodServo1n;
    private CRServo hoodServo2n;

    private CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);




    public void onInit() {
        telemetry.addLine("Initializing Follower...");





        telemetry.update();
        follower = PedroComponent.follower();


        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();


        paths = new Paths(follower);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();


        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);

        intakeMotor = new MotorEx("intake");
        transfer1 = new MotorEx("transfer");
        transfer2 = new ServoEx("transferServo1");

        transfer3 = new ServoEx("transferServo2");
        hoodservo1 = new ServoEx("hoodServo1");
        hoodservo2 = new ServoEx("hoodServo2");


        spinFlyWheel1500 = new LambdaCommand()
                .setStart(() -> shooter(1130));
        intakeMotorOn = new LambdaCommand()
                .setStart(() -> intakeMotor.setPower(-1));
        hoodServo1n= ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo1");
        hoodServo2n=  ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo2");


        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
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
            .setStart(()-> findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));

    public static Command openTransfer = new LambdaCommand()
            .setStart(()-> {
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.25);
            });
    public static Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                //transfer2.setPosition(1);
                transfer2.setPosition(1);
            });

    Command transferOn = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-0.67));
    Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));
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
    ParallelGroup HoodRunUp=new ParallelGroup(
            new SetPower(hoodServo1,1),
            new SetPower(hoodServo2,-1)
    );
    public ParallelGroup HoodPowerZero=new ParallelGroup(
            new SetPower(hoodServo1,0),
            new SetPower(hoodServo2,0)
    );

    public SequentialGroup HoodUp=new SequentialGroup(
            HoodRunUp,
            new Delay(0.1),
            HoodPowerZero
    );

    ParallelGroup HoodRunDown=new ParallelGroup(
            new SetPower(hoodServo1,-1),
            new SetPower(hoodServo2,1)
    );

    public SequentialGroup HoodDown=new SequentialGroup(
            HoodRunDown,
            new Delay(0.17),
            HoodPowerZero
    );



    public Command Auto(){
        return new SequentialGroup(
                intakeMotorOn,
                new FollowPath(paths.PreloadLaunch,true,0.8),
                new Delay(0.4),
                new FollowPath(paths.intakeSet2,true,0.8),
                new Delay(0.4),
                new FollowPath(paths.launchSet2,true,0.8),
                new Delay(0.4),
                transferOn,
                new FollowPath(paths.resetHelper,true,0.8),
                new FollowPath(paths.resetIntakeSpam, true, 1.0),
                new Delay(1),
                transferOff,
                new FollowPath(paths.launchSpam,true,0.8),
                new Delay(0.4),
                new FollowPath(paths.intakeSet1,true,0.8),
                new Delay(0.4),
                new FollowPath(paths.launchSet1,true,0.8),
                new Delay(0.4),
                new FollowPath(paths.intakeSet3,true,0.8),
                new Delay(0.4),
                new FollowPath(paths.launchSet3,true,0.8),
                new Delay(0.4),
                new FollowPath(paths.teleOpPar,true,0.8)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        Auto().schedule();
        telemetry.update();
    }

    @Override
    public void onUpdate(){

        shooter(1125);


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
            PreloadLaunch = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(119.4,126.4), new Pose(76.000, 76.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeSet2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 72.000),
                                    new Pose(85.000, 87.000),
                                    new Pose(85.000, 59.000),
                                    new Pose(127.000, 59.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            launchSet2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(127.000, 59.000),
                                    new Pose(85.000, 59.000),
                                    new Pose(85.000, 87.000),
                                    new Pose(72.000, 72.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            resetHelper = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 72.000),
                                    new Pose(80.400, 86.400),
                                    new Pose(97.200, 56.200),
                                    new Pose(106.300, 61.000),
                                    new Pose(123.000, 65.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            resetIntakeSpam = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.000, 65.000), new Pose(126.000, 60.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(40))
                    .build();

            launchSpam = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.500, 60.000),
                                    new Pose(112.000, 48.000),
                                    new Pose(81.000, 82.500),
                                    new Pose(72.000, 72.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeSet1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 72.000),
                                    new Pose(82.000, 87.000),
                                    new Pose(125.000, 84.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            launchSet1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(125.000, 84.000),
                                    new Pose(86.000, 87.000),
                                    new Pose(72.000, 72.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeSet3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 72.000),
                                    new Pose(88.000, 89.000),
                                    new Pose(84.000, 36.000),
                                    new Pose(125.000, 35.500)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            launchSet3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(125.000, 35.500),
                                    new Pose(84.000, 36.000),
                                    new Pose(88.000, 89.500),
                                    new Pose(72.000, 72.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            teleOpPar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 72.000), new Pose(84.000, 60.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }
}