

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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.MotifScanning;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
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
import dev.nextftc.hardware.powerable.SetPower;


@Autonomous(name = "Blue Auto Default Position MIRRORED", group = "Autonomous")
@Configurable
public class blueDefaultNewMirrored extends NextFTCOpMode {
    public blueDefaultNewMirrored(){
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
    public Pose start = new Pose(24.677419354838708,125.70967741935485, Math.toRadians(125));

    public Pose PreLoadLaunch1 = new Pose(62.12903225806452,96.67741935483872);

    public Pose ControlPoint1 = new Pose(50.225806451612904,80.41935483870968);

    public Pose Intake1 = new Pose(15.25233644859813,83.66355140186916);

    public Pose ClassifierRampPoint = new Pose(63.58064516129032,74.32258064516128);

    public Pose ClassifierRamp = new Pose(124.74766355140187,75.14018691588785);

    public Pose Launch1 = new Pose(62.12903225806452, 96.67741935483872);

    public Pose ControlPoint2 = new Pose(63.87096774193548,54.70967741935483);

    public Pose Intake2 = new Pose(12.11214953271028,58.11214953271029);
    public Pose ShootCP = new Pose(50,56);

    public Pose ControlPoint3 = new Pose(72.87096774193549,25.258064516129032);

    //public Pose ControlPoint4 = new Pose(74.46728971962617,38.439252336448604);

    public Pose Intake3 = new Pose(11.214953271028037,36.78504672897196);

    public Pose Teleop1 = new Pose(60.11214953271028,80.009345794392516);




    private static final int APRILTAG_PIPELINE = 8;







    private MotorEx intakeMotor;

    // private MotorEx spindexerMotor;

    private boolean path2Following= false;
    int ball1Color = 0;
    int ball2Color = 0;
    int ball3Color = 0;
    int tagId = 0;


    public static double spindexvelocity;





    Command  flywheelYes= new LambdaCommand()
            .setStart(() -> shooter(1500));






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

    private int newtps;
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
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();


        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();





        paths = new Paths(follower);
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

        //start.mirror();
        //PreLoadLaunch1.mirror();
        //ControlPoint1.mirror();
        //Intake1.mirror();
        /*ClassifierRampPoint.mirror();
        ClassifierRamp.mirror();
        ControlPoint2.mirror();
        Intake2.mirror();
        Launch1.mirror();
        ControlPoint3.mirror();
        //ControlPoint4.mirror();
        Intake3.mirror();
        Teleop1.mirror();*/



        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);


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

    Command opentransfer = new LambdaCommand()
            .setStart(()-> {
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.3);
                //transfer3.setPosition(0.75);
            });
    Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                //transfer2.setPosition(1);
                transfer2.setPosition(1.25);
                //transfer3.setPosition(0);
            });

    Command transferOn = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-0.7));
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


    public Command Auto(){
        return new SequentialGroup(

                spinFlyWheel1500,
                intakeMotorOn,
                new FollowPath(paths.PreLoadLaunch,true,0.8),
                opentransfer,
                new Delay(0.3),
                transferOn,
                new Delay(3),
                transferOff,
                //new TurnTo(Angle.fromDeg(90)),
                //getMotif,



                intakeMotorOn,

                new Delay(0.2),
                closeTransfer,
                new Delay(0.2),
                transferOn,
                new FollowPath(paths.Intake1set,false,0.8),
                transferOff,

                /*new FollowPath(paths.ClassifierRamp1,true,0.8),
                new Delay(0.5),
                //transferOff,
                //new Delay(0.1),
                // Sorting logic all here with the order, etc
                */intakeMotorOn,
                new FollowPath(paths.Launch1Real,true,0.8),

                opentransfer,
                new Delay(0.2),
                transferOn,


                // Transfer logic with transfer
                new Delay(3),
                transferOff,
                closeTransfer,
                new Delay(0.3),

                intakeMotorOn,
                transferOn,
                new FollowPath(paths.Intake2ndSet,false,0.7),
                new Delay(0.2),
                transferOff,
                new Delay(0.3),
                opentransfer,

                //intakeMotorOff,

                // Sorting logic and order here


                new FollowPath(paths.Launch2,true,0.9),
                intakeMotorOn,


                transferOn,

                // Transfer logic with transfer
                new Delay(3),
                closeTransfer,
                transferOff,



                intakeMotorOn,
                transferOn,
                /*new FollowPath(paths.Intake3rdSet,false,0.8),

                transferOff,
                new Delay(0.3),
                opentransfer,

                // Sorting logic here
                new FollowPath(paths.Launch3,false,0.8),
                new Delay(0.3),

                // Transfer and shoot logic
                opentransfer,
                transferOn,
                intakeMotorOff,

                new Delay(1.5),
                //closeTransfer,
                transferOff,*/
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
    public void onUpdate(){

        shooter(1125);


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
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(143))
                    //.setVelocityConstraint(50)
                    .build();
            Intake1set = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            PreLoadLaunch1,
                            ControlPoint1,
                            Intake1

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

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
                            Intake1,
                            Launch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                    .build();
            Intake2ndSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Launch1,
                            ControlPoint2,
                            Intake2

                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                    .build();
            Launch2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Intake2,
                            ShootCP,
                            Launch1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

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
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))

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