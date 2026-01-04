

package org.firstinspires.ftc.teamcode.opModes.Auto;


import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.TurnTo;
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

import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;


import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.MotifScanning;
import org.jetbrains.annotations.Async;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;


@Autonomous(name = "Red Auto 15 Ball Spam Moksh", group = "Autonomous")
@Configurable
public class Red15BallSpam extends NextFTCOpMode {
    public Red15BallSpam(){
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


    public void onInit() {
        telemetry.addLine("Initializing Follower...");

        telemetry.update();
        follower = PedroComponent.follower();
        /*Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();*/


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
            .setStart(()-> transfer1.setPower(-0.8));
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

    public SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.35), transferOn, new Delay(1), transferOff, closeTransfer);

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
            opentransfer,
            new Delay(1),
            shoot,
            transferOn,
            new FollowPath(paths.intakeSet2,true,1.0),
            transferOff,
            new FollowPath(paths.launchSet2,true,1.0),
            shoot,
            transferOn,
            new FollowPath(paths.resetHelper,true,1.0),
            new Delay(0.15),
            new FollowPath(paths.resetIntakeSpam, true, 1.0),
            new Delay(1.3),
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
                            new BezierLine(new Pose(119.400, 126.400), new Pose(82.600, 87.200))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeSet2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(85.000, 87.000),
                                    new Pose(84.000, 61.000),
                                    new Pose(124.000, 59.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            launchSet2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(127.000, 59.000),
                                    new Pose(87.700, 55.500),
                                    new Pose(97.500, 103.500),
                                    new Pose(82.600, 87.200)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            resetHelper = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(80.400, 86.400),
                                    new Pose(97.200, 56.200),
                                    new Pose(106.300, 62.000),
                                    new Pose(125, 68.5)
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
                                    new Pose(126.000, 60.000),
                                    new Pose(107.400, 50.000),
                                    new Pose(96.000, 102.500),
                                    new Pose(82.600, 87.200)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeSet1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(91.500, 97.300),
                                    new Pose(101.500, 87.000),
                                    new Pose(125.665, 86.898)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            launchSet1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(125.000, 84.000),
                                    new Pose(101.500, 84.000),
                                    new Pose(92.100, 97.700),
                                    new Pose(82.600, 87.200)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeSet3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(88.000, 89.500),
                                    new Pose(84.000, 39.500),
                                    new Pose(124.000, 40.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            launchSet3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(120.000, 40.000),
                                    new Pose(100.000, 43.000),
                                    new Pose(96.584, 106.639),
                                    new Pose(82.600, 87.200)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            teleOpPar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(82.600, 87.200), new Pose(108.000, 72.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }
}