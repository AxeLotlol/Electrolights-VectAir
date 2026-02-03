

package TRASH;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.MotifScanning;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
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
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;


@Autonomous(name = "Blue Auto Default Position", group = "Autonomous")
@Configurable
public class BlueDefault extends NextFTCOpMode {
    public BlueDefault(){
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
    public Pose start = new Pose(23.485714285714288,126.68571428571428, Math.toRadians(125));


    public Pose PreLoadLaunch1 = new Pose(57.6,101.27472527472527);

    public Pose ControlPoint1 = new Pose(73.71428571428571,82.11428571428571 );

    public Pose Intake1 = new Pose(17.142857142857142,80.91428571428571);


    public Pose Launch1 = new Pose(72, 72);

    public Pose ControlPoint2 = new Pose(59,86);

    public Pose Intake2 = new Pose(17.485714285714284,57.94285714285714);

    public Pose ControlPoint3 = new Pose(54.68571428571428,52.28571428571429);

    //public Pose ClassifierRampPoint = new Pose(69.38709677419355,69.67741935483872);
    //NOT SURE WHAT THIS IS -DEB

    public Pose ClassifierRamp = new Pose(12,61.71428571428572);

    public Pose ControlPoint4 = new Pose(103,42);
    public Pose Launch2 = new Pose(72, 72);

    public Pose Intake3 = new Pose(23.82857142857143,32.74285714285714);
    public Pose ControlPoint5 = new Pose(78.5142857142857,31.028571428571432);

    public Pose Launch3 = new Pose(72, 72);
    public Pose ControlPoint6 = new Pose(53.48571428571429,91.37142857142857);

    public Pose Teleop1 = new Pose(60.68571428571428,36.17142857142857);
    public Pose ControlPoint7 = new Pose(60,68.57142857142857);

    /*
    second auto paths in order:
    START:(24.342857142857145,125.4857142857143)
    (ALL LAUNCHES ARE 72, 72 ON PEDROPATHING VISUALIZER)
    PRELOAD LAUNCH: (72,72)
    INTAKE 1:(19.02857142857143,81.77142857142857)
    CONTROL POINT 1: (60.17142857142857,83.82857142857142)
    INTAKE 2: (23.82857142857143,57.08571428571429)
    CONTROL POINT 2: (77.82857142857142,57.6)
    RAMP: (13.542857142857143,61.71428571428572)
    CONTROL POINT 3: ( 62.74285714285715,39.25714285714286)
    BACK: (20.74285714285714,55.54285714285714)
    INTAKE 3:(24.342857142857145,32.4)
    CONTROL POINT 4:(69.94285714285714,30.34285714285715)
    TELEOP:(36.68571428571428,59.14285714285714)
    CONTROL POINT 5:(36.51428571428571,80.74285714285715)
    LAUNCH CONTROL POINTS IN ORDER:
    (36.68571428571428,114.85714285714286)
    (24.514285714285716,110.39999999999999)
    (52.114285714285714,96.51428571428572)
    (93.77142857142857,52.45714285714286)
    INTAKE AND RESET RAMP IS AT (14,59.7)
     */










    private MotorEx intakeMotor;

    // private MotorEx spindexerMotor;

    private boolean path2Following= false;
    int ball1Color = 0;
    int ball2Color = 0;
    int ball3Color = 0;
    int tagId = 0;


    public static double spindexvelocity;





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




    public static void velocityControlWithFeedforwardExample(KineticState currentstate, float configtps) {
        ControlSystem controller = ControlSystem.builder()
                .velPid(0.1, 0.01, 0.05) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(0.0067, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        controller.setGoal(new KineticState(0.0, configtps, 0.0));

        double power = controller.calculate(currentstate);

    }

    private MotorEx transfer1;
    private ServoEx transfer2;
    private ServoEx transfer3;
    private ServoEx hoodservo1;
    private ServoEx hoodservo2;
    private Command spinFlyWheel1500;
    private Command intakeMotorOn;
    public void onInit() {
        telemetry.addLine("Initializing Follower...");





        telemetry.update();
        follower = PedroComponent.follower();


        paths = new Paths(follower);
        intakeMotor = new MotorEx("intake");
        transfer1 = new MotorEx("transfer");
        transfer2 = new ServoEx("transferServo1");
        transfer3 = new ServoEx("transferServo2");
        hoodservo1 = new ServoEx("hoodServo1");
        hoodservo2 = new ServoEx("hoodServo2");

        spinFlyWheel1500 = new LambdaCommand()
                .setStart(() -> Flywheel.shooter(1500));
        intakeMotorOn = new LambdaCommand()
                .setStart(() -> intakeMotor.setPower(-1));

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);


        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
    }


    public MotorEx flywheel = new MotorEx("launchingmotor").reversed();


    Command stopFlywheel = new LambdaCommand()
            .setStart(() -> Flywheel.shooter(0));

    Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0));
    Command hoodUp = new LambdaCommand()
            .setStart(() -> hoodservo1.setPosition(0.2));
    Command hoodup2 = new LambdaCommand()
            .setStart(() -> hoodservo2.setPosition(0.2));
    Command getMotif = new LambdaCommand()
            .setStart(() -> tagId = MotifScanning.INSTANCE.findMotif());

    Command opentransfer = new LambdaCommand()
            .setStart(()-> {
                transfer2.setPosition(0.7);
                transfer3.setPosition(0.7);
            });
    Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                transfer2.setPosition(0);
                transfer3.setPosition(0);
            });
    Command transferOn = new LambdaCommand()
            .setStart(()-> transfer1.setPower(0.5));
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
                            hoodup2

                    );


                }


            });*/

    public Command Auto(){
        return new SequentialGroup(

                spinFlyWheel1500,
                new FollowPath(paths.PreLoadLaunch,true,0.9),
                opentransfer,
                transferOn,
                new Delay(2.0),
                stopFlywheel,
                transferOff,
                //new TurnTo(Angle.fromDeg(90)),
                //getMotif,

                new Delay(1.0),
                closeTransfer,

                intakeMotorOn,
                transferOn,
                new FollowPath(paths.Intake1set,false,0.8),

                new FollowPath(paths.ClassifierRamp1,true,0.7),
                transferOff,
                intakeMotorOff,
                spinFlyWheel1500,
                new Delay(1.5),
                // Sorting logic all here with the order, etc
                new FollowPath(paths.Launch1Real,true,0.9),
                opentransfer,
                transferOn,


                // Transfer logic with transfer
                new Delay(2.0),
                closeTransfer,
                transferOff,
                stopFlywheel,

                intakeMotorOn,
                transferOn,
                new FollowPath(paths.Intake2ndSet,false,0.8),

                intakeMotorOff,
                transferOff,
                spinFlyWheel1500,
                // Sorting logic and order here


                new FollowPath(paths.Launch2,true,0.9),
                opentransfer,
                transferOn,

                // Transfer logic with transfer
                new Delay(1.0),
                closeTransfer,


                intakeMotorOn,
                stopFlywheel,
                new FollowPath(paths.Intake3rdSet,false,0.8),
                intakeMotorOff,
                transferOff,
                spinFlyWheel1500,

                // Sorting logic here
                new FollowPath(paths.Launch3,false,0.9),
                // Transfer and shoot logic
                opentransfer,
                transferOn,

                new Delay(2.0),
                closeTransfer,
                transferOff,
                stopFlywheel,
                new FollowPath(paths.teleOp,true,0.9)




        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        //int tag=MotifScanning.INSTANCE.findMotif();
        Auto().schedule();


        telemetry.addLine("The shooter has started btw");
        telemetry.addLine("Started Path 1");
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
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(130))
                    //.setVelocityConstraint(50)
                    .build();
            Intake1set = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            PreLoadLaunch1,
                            ControlPoint1,
                            Intake1

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))

                    .build();
            ClassifierRamp1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Intake1,
                            //ClassifierRampPoint,
                            ClassifierRamp


                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
            Launch1Real = follower.pathBuilder()
                    .addPath(new BezierLine(
                            ClassifierRamp,
                            Launch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();
            Intake2ndSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Launch1,
                            ControlPoint2,
                            Intake2

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))

                    .build();
            Launch2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            Intake2,
                            Launch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();
            Intake3rdSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Launch1,
                            ControlPoint3,
                            Intake3

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))

                    .build();
            Launch3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            Intake3,
                            Launch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

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