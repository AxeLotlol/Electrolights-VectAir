

package org.firstinspires.ftc.teamcode;

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
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



import org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;

import java.time.Duration;


@Autonomous(name = "Red Auto Default Position", group = "Autonomous")
@Configurable
public class redDefault extends NextFTCOpMode {
    public redDefault(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public static final ServoEx servoPos = new ServoEx("servoPos");
    private Paths paths;
    public Pose start = new Pose(118.45161290322581,126, Math.toRadians(51));

    public Pose PreLoadLaunch1 = new Pose(86.2258064516129,97.83870967741936);

    public Pose ControlPoint1 = new Pose(58.354838709677416,82.74193548387096);

    public Pose Intake1 = new Pose(128.61290322580646,83.32258064516128);

    public Pose ClassifierRampPoint = new Pose(63.58064516129032,74.32258064516128);

    public Pose ClassifierRamp = new Pose(131.80645161290323,75.19354838709678);

    public Pose Launch1 = new Pose(86.2258064516129, 85.64516129032258);

    public Pose ControlPoint2 = new Pose(39,57);

    public Pose Intake2 = new Pose(132.67741935483872,60.09677419354839);

    public Pose ControlPoint3 = new Pose(45.29032258064516,25.258064516129032);

    public Pose Intake3 = new Pose(131.2258064516129,35.12903225806451);

    public Pose Teleop1 = new Pose(72.29032258064517,46.16129032258064);













    private MotorEx intakeMotor;

    // private MotorEx spindexerMotor;

    private boolean path2Following= false;
    int ball1Color = 0;
    int ball2Color = 0;
    int ball3Color = 0;
    int tagId = 0;
    private ServoEx servo = new ServoEx("servoPos");

    public static double spindexvelocity;
    public static MotorEx spindex = new MotorEx("spindexer");


    Command pathCommand = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake1set))
            .setIsDone(() -> !follower.isBusy())
            .named("Path2 Command");
    Command pathCommand2 = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake2ndSet))
            .setIsDone(() -> !follower.isBusy())
            .named("Path4 Command");
    Command pathCommand3 = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake3rdSet))
            .setIsDone(() -> !follower.isBusy())
            .named("Path4 Command");

    public static MotorEx flywheel = new MotorEx("launchingmotor").reversed();


    public static void velocityControlWithFeedforwardExample(KineticState currentstate, float configtps) {
        ControlSystem controller = ControlSystem.builder()
                .velPid(0.1, 0.01, 0.05) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(0.0067, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        controller.setGoal(new KineticState(0.0, configtps, 0.0));

        double power = controller.calculate(currentstate);
        spindex.setPower(power);
    }
    public static void spin(float tps) {
        BindingManager.update();
        spindexvelocity = spindex.getVelocity();
        KineticState currentState = new KineticState(0, spindexvelocity, 0.0);
        velocityControlWithFeedforwardExample(currentState, tps);
    }



    public void onInit() {
        telemetry.addLine("Initializing Follower...");


        telemetry.update();
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        intakeMotor = new MotorEx("intake");

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);
        Flywheel.shooter(0);
        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
    }


    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        //int tag=MotifScanning.INSTANCE.findMotif();
        follower.followPath(paths.PreLoadLaunch);
        Flywheel.shooter(1500);
        telemetry.addLine("The shooter has started btw");
        telemetry.addLine("Started Path 1");
        telemetry.update();
    }


    public void onUpdate() {
        follower.update();
        switch (pathState) {

            case 0:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();


                    pathState++;

                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();

                    follower.followPath(paths.ClassifierRamp1);
                    pathState++;
                }







        }

        telemetry.addData("State", pathState);
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Path Timer", pathTimer.getElapsedTime());
        telemetry.update();
    }

    @Override
    public void onStop() {
        follower.breakFollowing();
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }

    public class Paths {
        public PathChain PreLoadLaunch;
        public PathChain Intake1set;

        public PathChain ClassifierRamp1;

        public PathChain Launch1Real;

        public PathChain Intake2ndSet;

        public PathChain Launch2;

        public PathChain Intake3rdSet;

        public PathChain Launch3;

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
                    .addPath(new BezierLine(
                            start,
                            PreLoadLaunch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(56))
                    //.setVelocityConstraint(50)
                    .build();
            Intake1set = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            PreLoadLaunch1,
                            ControlPoint1,
                            Intake1

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(56), Math.toRadians(0))

                    .build();
            ClassifierRamp1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Intake1,
                            ClassifierRampPoint,
                            ClassifierRamp


                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();
            Launch1Real = follower.pathBuilder()
                    .addPath(new BezierLine(
                            ClassifierRamp,
                            Launch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(59))

                    .build();
            Intake2ndSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Launch1,
                            ControlPoint2,
                            Intake2

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

                    .build();
            Launch2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            Intake2,
                            Launch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(59))

                    .build();
            Intake3rdSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Launch1,
                            ControlPoint3,
                            Intake3

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))

                    .build();
            Launch3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            Intake3,
                            Launch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(59))

                    .build();
            teleOp = follower.pathBuilder()
                    .addPath(new BezierLine(
                            Launch1,
                            Teleop1

                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

        }
    }
}