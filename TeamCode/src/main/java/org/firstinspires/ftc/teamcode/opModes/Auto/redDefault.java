

package org.firstinspires.ftc.teamcode.opModes.Auto;

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
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



import org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;


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

    public Pose Intake1 = new Pose(127.4018691588785,83.66355140186916);

    public Pose ClassifierRampPoint = new Pose(63.58064516129032,74.32258064516128);

    public Pose ClassifierRamp = new Pose(128.74766355140187,75.14018691588785);

    public Pose Launch1 = new Pose(86.2258064516129, 85.64516129032258);

    public Pose ControlPoint2 = new Pose(57.19626168224299,51.364485981308405);

    public Pose Intake2 = new Pose(129.19626168224298,59.43925233644861);

    public Pose ControlPoint3 = new Pose(61.23364485981308,28.71028037383178);

    public Pose Intake3 = new Pose(131.2258064516129,35.12903225806451);

    public Pose Teleop1 = new Pose(84.11214953271028,37.009345794392516);













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

    public static MotorEx transfer = new MotorEx("transfer");

    Command  flywheelYes= new LambdaCommand()
            .setStart(() -> Flywheel.shooter(1500));






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

        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
    }

    Command spinFlyWheel1500 = new LambdaCommand()
            .setStart(() -> Flywheel.shooter(1500));

    Command stopFlywheel = new LambdaCommand()
            .setStart(() -> Flywheel.shooter(0));
    Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(-1));
    Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0));

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        //int tag=MotifScanning.INSTANCE.findMotif();

        telemetry.addLine("The shooter has started btw");
        telemetry.addLine("Started Path 1");
        telemetry.update();

        new SequentialGroup(
                spinFlyWheel1500,
                new FollowPath(paths.PreLoadLaunch,true,0.5),
                // Shooting logic with transfer insert here
                new Delay(2),
                stopFlywheel,
                intakeMotorOn,
                new FollowPath(paths.Intake1set,false,0.5),

                new FollowPath(paths.ClassifierRamp1,false,0.5),
                intakeMotorOff,
                spinFlyWheel1500,
                // Sorting logic all here with the order, etc
                new FollowPath(paths.Launch1Real,true,0.5),
                // Transfer logic with transfer
                new Delay(2),
                intakeMotorOn,
                new FollowPath(paths.Intake2ndSet,false,0.5),
                intakeMotorOff,
                spinFlyWheel1500,
                // Sorting logic and order here

                new FollowPath(paths.Launch2,true,0.5),
                // Transfer logic with transfer
                new Delay(1),
                intakeMotorOn,
                stopFlywheel,
                new FollowPath(paths.Intake3rdSet,false,0.5),
                intakeMotorOff,
                spinFlyWheel1500,
                // Sorting logic here
                new FollowPath(paths.Launch3,false,0.5),
                // Transfer and shoot logic
                new Delay(2),
                new FollowPath(paths.teleOp)



        );


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