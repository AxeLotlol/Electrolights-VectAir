

package org.firstinspires.ftc.teamcode.opModes.Auto;


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


@Autonomous(name = "Red Auto SPAM REALLER Position", group = "Autonomous")
@Configurable
public class red15spamreal extends NextFTCOpMode {
    public red15spamreal(){
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

    public Pose start = new Pose(119.4,126.4, Math.toRadians(49));

    //public Pose PreloadCP = new Pose(76.000,76.000);

    public Pose PreLoadLaunch1 = new Pose(76,76);

    public Pose intake2CP1 = new Pose(85,87);

    public Pose intake2CP2 = new Pose(85,59);

    public Pose Intake2nd = new Pose(127,59);

    public Pose launch2CP1 = new Pose(85,89);
    public Pose launch2CP = new Pose(85,87);

    public Pose Launch21 = new Pose(76,76);

    public Pose resetCP1 = new Pose(80.4,86.4);

    public Pose resetCP2 = new Pose(97.2,56.2);

    public Pose resetcp3 = new Pose(106.3,61);

    public Pose resetHelper = new Pose(123,65);



    public Pose resetIntake2 = new Pose(126.000,60);

    public Pose intake3CP = new Pose(90,35);

    public Pose intake3 = new Pose(130,35);

    public Pose launchspam1 = new Pose(133.500,60);
    public Pose launchspam2 = new Pose(112,48);

    public Pose launchspam3 = new Pose(81,82.5);

    public Pose launchspam4 = new Pose(76,76);

    public Pose intake1 = new Pose(82,87);

    public Pose intake12 = new Pose(125,84);

    public Pose schoolLauncher = new Pose(86,87);

    public Pose schoolLauncherFinal = new Pose(76,76);



    public Pose whatAyushDoestoHAMDLOintake3 = new Pose(88,89);

    public Pose WhatAyushDoestoHAMDLOintake31 = new Pose(84,40);

    public Pose WhatAyushDoestoHAMDLOintake32 = new Pose(125,39.5);

    public Pose ayushHASALAUNCHER = new Pose(84,36);

    public Pose ayushHasAnotherLauncher = new Pose(88,89.5);

    public Pose AyushLaunchedMEE = new Pose(76,76);

    public Pose launch3 = new Pose(77,77);

    public Pose Teleop1 = new Pose(84,60);




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
            .named("Path4 Command");
    Command pathCommand3 = new LambdaCommand()
            .setStart(() -> follower.followPath(paths.Intake3rdSet))
            .setIsDone(() -> !follower.isBusy())
            .named("Path4 Command");*/




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
    private Command shootClose;
    private Command intakeMotorOn;

    private Command shootFar;

    private CRServo hoodServo1n;
    private CRServo hoodServo2n;

    private CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);

    private boolean shootClose1;

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


        shootClose = new LambdaCommand()
                .setStart(() -> {
                    shooter(1125);
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

    /*Command spin = new LambdaCommand()
            .setStart(()->flywheel.setPower(-1));*/

    public Command Auto(){
        return new SequentialGroup(
                //spin,

                shootClose,
                intakeMotorOn,
                new FollowPath(paths.Path1,true,0.99),
                opentransfer,
                new Delay(0.3),
                transferOn,
                new Delay(1),
                transferOff,
                //new TurnTo(Angle.fromDeg(90)),
                //getMotif,
                shootFar,
                closeTransfer,
                closeTransfer,



                intakeMotorOn,

                new Delay(0.2),

                new Delay(0.2),
                transferOn,
                new FollowPath(paths.Path2,false,1.0),
                transferOff,


                /*new FollowPath(paths.ClassifierRamp1,true,0.8),
                new Delay(0.5),
                //transferOff,
                //new Delay(0.1),
                // Sorting logic all here with the order, etc
                */intakeMotorOn,
                new FollowPath(paths.Path3,true,0.99),

                opentransfer,
                new Delay(0.2),
                transferOn,


                // Transfer logic with transfer
                new Delay(1),
                transferOff,
                closeTransfer,
                new Delay(0.3),

                intakeMotorOn,
                transferOn,
                new FollowPath(paths.Path4,false,1.0),

                new Delay(0.8),
                new Delay(0.2),
                transferOff,
                new Delay(0.3),


                //intakeMotorOf

                // Sorting logic and order here


                new FollowPath(paths.Path5,true,0.99),
                opentransfer,
                intakeMotorOn,


                transferOn,

                // Transfer logic with transfer
                new Delay(1),
                closeTransfer,
                transferOff,



                intakeMotorOn,
                transferOn,
                new FollowPath(paths.Path6,false,0.99),

                transferOff,
                new Delay(0.3),
                opentransfer,

                // Sorting logic here
                new FollowPath(paths.Path7,false,0.99),
                new Delay(0.3),

                // Transfer and shoot logic
                opentransfer,
                transferOn,
                intakeMotorOff,

                new Delay(1),
                transferOff,
                closeTransfer,
                transferOn,
                intakeMotorOn,

                new FollowPath(paths.Path8,false,0.99),
                transferOff,
                intakeMotorOff,
                new FollowPath(paths.Path9,true,0.99),
                opentransfer,
                transferOn,
                new Delay(1.0),
                transferOff,
                closeTransfer,
                stopFlywheel,
                new FollowPath(paths.Path10,true,1.0)




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

        //if(shootClose1){
        //shooter(1125);

        //}
        //else if(!shootClose1){
        shooter(1300);
        //}



        follower.update();


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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.996, 126.594), new Pose(82.621, 87.198))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(49),Math.toRadians(39))
                    .setReversed()
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.621, 87.198),
                                    new Pose(79.815, 61.426),
                                    new Pose(118.755, 62.275)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(123.755, 62.275),
                                    new Pose(110.594, 69.280),
                                    new Pose(90.800, 95.814),
                                    new Pose(82.574, 86.899)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.574, 86.899),
                                    new Pose(86.183, 53.784),
                                    new Pose(130.123, 70)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.185, 64.185),
                                    new Pose(116.326, 59.728),
                                    new Pose(90.853, 95.814),
                                    new Pose(82.574, 86.899)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.574, 86.899),
                                    new Pose(74.932, 38.713),
                                    new Pose(122.057, 40.411)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(122.057, 40.411),
                                    new Pose(91.065, 96.026),
                                    new Pose(82.574, 87.111)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(82.574, 87.111), new Pose(129.062, 83.927))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(129.062, 83.927),
                                    new Pose(90.853, 96.026),
                                    new Pose(82.574, 87.111)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(82.574, 87.111), new Pose(104.014, 81.592))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(350))
                    .build();
        }
    }
    }
