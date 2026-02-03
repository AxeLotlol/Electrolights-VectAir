

package TRASH;


import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
//import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.flywheel;
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
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.powerable.SetPower;



@Autonomous(name = "Blue Auto SPAM REALLER Position", group = "Autonomous")
@Configurable
public class blue15spamreal extends NextFTCOpMode {
    public blue15spamreal(){
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

    public Pose start = new Pose(23.609756097560975,125.46341463414635, Math.toRadians(134));

    //public Pose PreloadCP = new Pose(76.000,76.000);

    public Pose PreLoadLaunch1 = new Pose(57.36585365853659,91.70731707317074);

    public Pose Intake2AndClassifierCP = new Pose(56.19512195121951,49.5609756097561);

    public Pose Intake2AndClassifier = new Pose(13.26829268292683,62.63414634146339);

    public Pose Launch2CP1 = new Pose(31.804878048780488,61.07317073170732);

    public Pose Launch2CP2 = new Pose(54.048780487804876,95.02439024390243);

    public Pose Launch2REALLL = new Pose(57.36585365853659,91.51219512195124);

    public Pose intake3reall3Cp = new Pose(79,41);

    public Pose intake3Real = new Pose(17.5609756097561,41.17073170731707);

    public Pose launch3CP = new Pose(46.04878048780488,102.82926829268291);

    public Pose Launch3REALLL = new Pose(57.36585365853659,91.70731707317074);

    public Pose Intake1ball = new Pose(16,86.04878048780488);

    public Pose Laaunch1CP = new Pose(53.853658536585364,95.21951219512195);

    public Pose Laaunch1 = new Pose(57.36585365853659,91.70731707317074);

    public Pose teleopreall = new Pose(39.27051384854741,85.2003657510939);




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






    /*Command pathCommand = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake1set))
            .setIsDone(() -> !follower.isBusy())
            .named("Path2 Command");*/
    /*Command pathCommand2 = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake2ndSet))
            .setIsDone(() -> !follower.isBusy())
            .named("Path4 Command");*/
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



    private Command shootClose;
    private Command intakeMotorOn;

    private Command shootFar;

    private CRServo hoodServo1n;
    private CRServo hoodServo2n;

    private CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);

    public SequentialGroup HoodUp;
    public ParallelGroup HoodRunUp;
    public ParallelGroup HoodPowerZero;
    public ParallelGroup HoodRunDown;


    private boolean shootClose1;

    public void onInit() {
        telemetry.addLine("Initializing Follower...");
        ParallelGroup HoodRunUp=new ParallelGroup(
                new SetPower(hoodServo1,1),
                new SetPower(hoodServo2,-1)
        );
        ParallelGroup HoodPowerZero=new ParallelGroup(
                new SetPower(hoodServo1,0),
                new SetPower(hoodServo2,0)
        );

        SequentialGroup HoodUp = new SequentialGroup(
                new ParallelGroup(
                        new SetPower(hoodServo1, 1.0),
                        new SetPower(hoodServo2, -1.0),
                        new Delay(0.15)   // power is held while delay runs
                ),
                new ParallelGroup(
                        new SetPower(hoodServo1, 0.0),
                        new SetPower(hoodServo2, 0.0)
                ));


        ParallelGroup HoodRunDown=new ParallelGroup(
                new SetPower(hoodServo1,-1),
                new SetPower(hoodServo2,1)
        );

        SequentialGroup HoodDown=new SequentialGroup(
                HoodRunDown,
                new Delay(0.17),
                HoodPowerZero
        );





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



        shootClose = new LambdaCommand()
                .setStart(() -> {
                    shooter(1400);
                     shootClose1 = true;


                });
        shootFar = new LambdaCommand()
                .setStart(()->{
                    shooter(1330);
                     shootClose1 = false;
                });
        intakeMotorOn = new LambdaCommand()
                .setStart(() -> intakeMotor.setPower(-1));
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


        shootClose1 = false;

        follower.update();


    }




    Command stopFlywheel = new LambdaCommand()
            .setStart(() -> shooter(0));

    Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0));
    /*Command hoodUp = new LambdaCommand()
            .setStart(() -> hoodservo1.setPosition(0.2));*/
   /* Command hoodup2 = new LambdaCommand()
            .setStart(() -> hoodservo2.setPosition(0.2));*/
    Command getMotif = new LambdaCommand()
            .setStart(() -> tagId = MotifScanning.INSTANCE.findMotif());

    Command distance = new LambdaCommand()
            .setStart(()-> findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));

    Command opentransfer = new LambdaCommand()
            .setStart(()-> {
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.4);
                //transfer3.setPosition(0.75);
            });
    Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                //transfer2.setPosition(1);
                transfer2.setPosition(0.7);
                //transfer3.setPosition(0);
            });

    Command transferOn = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-1));
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

    Command shootForMe = new LambdaCommand()
            .setStart(()->shooter(1358));
    /*Command spin = new LambdaCommand()
            .setStart(()-> flywheel.setPower(0.4));*/

    public Command Auto(){
        return new SequentialGroup(
                //spin,
                opentransfer,
                new Delay(0.5),

                transferOn,
                new FollowPath(paths.PreLoadLaunch,true,0.99),
                new Delay(1.0),
                transferOff,
                closeTransfer









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
    public void onUpdate() {
        super.onUpdate();
        shooter(1350);
        follower.update();
    }





    @Override
    public void onStop() {
        follower.breakFollowing();
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }

    public class Paths {
        public PathChain PreLoadLaunch;
        public PathChain Intake2Spam;

        //public PathChain ClassifierRamp1;

        public PathChain LaunchSpamReal;

        public PathChain Intake3rdSet;

        //public PathChain Intake2ndSetYES;

        public PathChain Launch3RealLLLLLLLLLL;

        //public PathChain Intake3rdSet;



        public PathChain Intake1stSetFRRRRR;

        public PathChain Launch1stsetFRRR;

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
                    .setConstantHeadingInterpolation(Math.toRadians(138))
                    //.setVelocityConstraint(50)
                    .build();
            Intake2Spam = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            PreLoadLaunch1,
                            Intake2AndClassifierCP,
                            Intake2AndClassifier


                    ))
                    .setTangentHeadingInterpolation()

                    .build();
           /* ClassifierRamp1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Intake1,
                            ClassifierRampPoint,
                            ClassifierRamp


                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();*/
            LaunchSpamReal = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Intake2AndClassifier,
                            Launch2CP1,
                            Launch2CP2,
                            Launch2REALLL

                    ))
                    .setTangentHeadingInterpolation().setReversed()

                    .build();
            Intake3rdSet = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Launch2REALLL,
                            intake3reall3Cp,
                            intake3Real


                    ))
                    .setTangentHeadingInterpolation()

                    .build();
            Launch3RealLLLLLLLLLL = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            intake3Real,
                            launch3CP,
                            Launch3REALLL

                    ))
                    .setTangentHeadingInterpolation().setReversed()
                    .build();
            Intake1stSetFRRRRR = follower.pathBuilder()
                    .addPath(new BezierLine(
                            Launch3REALLL,
                            Intake1ball


                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(187))

                    .build();
            Launch1stsetFRRR = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            Intake1ball,
                            Laaunch1CP,
                            Laaunch1

                    ))
                    .setTangentHeadingInterpolation()

                    .build();



            teleOp = follower.pathBuilder()
                    .addPath(new BezierLine(
                            Laaunch1,
                            teleopreall


                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

        }
    }
}