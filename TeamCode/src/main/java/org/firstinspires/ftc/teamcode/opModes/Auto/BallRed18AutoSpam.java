

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


@Autonomous(name = "Official 18 Spam Red Autp", group = "Autonomous")
@Configurable
public class BallRed18AutoSpam extends NextFTCOpMode {
    public BallRed18AutoSpam(){
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
            .setStart(()-> transfer1.setPower(-1.0));
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
    Command stopFlywheel = new LambdaCommand()
            .setStart(() -> flywheel.setPower(0));

    public SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.35), transferOn, new Delay(1), transferOff, closeTransfer);

    public boolean spinup = true;
    public Command spinupfalse = new LambdaCommand()
            .setStart(()-> {
                spinup=false;
            });
    public Command Auto(){
        return new SequentialGroup(
                new FollowPath(paths.Preload,true,1.0),
                spinupfalse,
                intakeMotorOn,
                opentransfer,
                new Delay(1),
                shoot,
                transferOn,
                new FollowPath(paths.Path2,true,0.9),
                transferOff,
                new FollowPath(paths.Path3,true,1.0),
                shoot,
                transferOn,
                new FollowPath(paths.Path4,true,1.0),
                new Delay(0.15),
                new FollowPath(paths.Path5, true, 1.0),
                new Delay(1.3),
                transferOff,
                new FollowPath(paths.Path6,true,1.0),
                shoot,
                transferOn,
                new FollowPath(paths.Path7,true,1.0),
                new Delay(0.15),
                new FollowPath(paths.Path8, true, 1.0),
                new Delay(1.3),
                transferOff,
                new FollowPath(paths.Path9,true,1.0),
                shoot,
                transferOn,

                new FollowPath(paths.Path10,true,1.0),
                transferOff,
                new FollowPath(paths.Path11,true,1.0),
                shoot,
                transferOn,
                new FollowPath(paths.Path12,true,1.0),

                transferOff,
                new FollowPath(paths.Path13,true,1.0),
                shoot,
                stopFlywheel,
                new FollowPath(paths.Path14,true,1.0)
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

        public PathChain Preload;
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
            Preload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.57109116743206, 126.59361007794115), new Pose(82.600, 87.200))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(46))
                    .setReversed()
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(90.703, 50.154),
                                    new Pose(131.397, 59.515)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.397, 59.515),
                                    new Pose(83.968, 74.401),
                                    new Pose(82.600, 87.200)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(47))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(101.679, 62.275),
                                    new Pose(127.364, 67.369)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(127.364, 67.369), new Pose(131.609, 59.303))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(12), Math.toRadians(40))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.609, 59.303),
                                    new Pose(90.428, 96.663),
                                    new Pose(82.600, 87.200)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(101.679, 62.275),
                                    new Pose(127.364, 67.369)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(127.364, 67.369), new Pose(131.609, 59.303))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(12), Math.toRadians(40))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.609, 59.303),
                                    new Pose(90.428, 96.875),
                                    new Pose(82.600, 87.200)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(102.316, 88.809),
                                    new Pose(124.604, 85.200)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(124.604, 85.200),
                                    new Pose(90.853, 96.875),
                                    new Pose(82.600, 87.200)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(351), Math.toRadians(46))
                    .setReversed()
                    .build();

            Path12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.600, 87.200),
                                    new Pose(68.776, 32.557),
                                    new Pose(123.208, 34.679)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path13 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(123.208, 34.679),
                                    new Pose(91.065, 96.875),
                                    new Pose(82.600, 87.200)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            Path14 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(82.600, 87.200),
                            new Pose(105.712,80.105)



                    ))
                    .setConstantHeadingInterpolation(90)
                    .build();
        }
        }
    }
