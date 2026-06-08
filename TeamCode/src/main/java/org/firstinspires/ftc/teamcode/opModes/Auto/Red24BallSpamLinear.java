package org.firstinspires.ftc.teamcode.opModes.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Airsort.transfer;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.hoodServo;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeadingWithAcceleration;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

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
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Autonomous
@Configurable
public class Red24BallSpamLinear extends NextFTCOpMode {
    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE = 224.75;

    public Red24BallSpamLinear(){
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
    public MotorEx intakeMotor;

    int tagId = 0;

    public Pose start = new Pose(109.81,133.272, Math.toRadians(270));

    private MotorEx transfer1;
    private ServoEx transfer2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    private Boolean preloadspin;
    private double preloadtps;
    private double shoottps;

    private static Servo hoodServo1n;
    private static Servo hoodServo2n;

    private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
    private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);

    private static ServoEx turret1;
    private static ServoEx turret2;

    public void onInit() {
        telemetry.addLine("Initializing Follower...");
        telemetry.update();
        follower = PedroComponent.follower();

        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();
        paths = new Paths(follower);
        intakeMotor = new MotorEx("intake");
        transfer1 = new MotorEx("transfer");
        transfer2 = new ServoEx("transferServo1");
        turret1 = new ServoEx("turretServo1");
        turret2 = new ServoEx("turretServo2");

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);

        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
        hoodServo1n= ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
        hoodServo2n=  ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");

        follower.update();
    }

    public static double hoodToPos(double runtime) {
        if(Double.isNaN(runtime)!=true) {
            ActiveOpMode.telemetry().addData("runtime", runtime);
            ParallelGroup HoodRunUp = new ParallelGroup(
                    new SetPosition(hoodServo1, runtime),
                    new SetPosition(hoodServo2, -1*runtime)
            );
            HoodRunUp.schedule();
            return runtime;
        }
        else {
            ActiveOpMode.telemetry().addLine("NaN");
            return 0;
        }
    }

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(-1));

    Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0));

    double distance;

    Command transferOn = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-1));
    Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));
    Command transferOnForIntake = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-1));

    public Command opentransfer = new LambdaCommand()
            .setStart(()-> {
                transfer2.setPosition(0.3);
            });

    public Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                transfer2.setPosition(0.635);
            });

    public SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.05), transferOn, new Delay(0.3), transferOff, closeTransfer);

    public boolean spinup = true;
    public Command spinupfalse = new LambdaCommand()
            .setStart(()-> {
                spinup=false;
            });

    Command spinupPLEASEEIsagiINEEDTHIS = new LambdaCommand()
            .setStart(()->{
                flywheel.setPower(1);
                flywheel2.setPower(-1);
            });
    public Command reverseIntakeForMe = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0.5));
    public Command aimAtGoal = new LambdaCommand()
            .setStart(() -> {
                Pose currPose = follower.getPose();

                Vector robotToGoalVector = new Vector(
                        follower.getPose().distanceFrom(new Pose(138, 141)),
                        Math.atan2(141 - currPose.getY(), 138 - currPose.getX())
                );

                Double[] results = calculateShotVectorandUpdateHeading(
                        currPose.getHeading(),
                        robotToGoalVector,
                        follower.getVelocity()
                );

                double targetHeading = results[2];
                double flywheelSpeed = results[0];
                double hoodAngle = results[1];

                shooter((float) (flywheelSpeed + 30));
                hoodToPos(hoodAngle);

                follower.turn(targetHeading);
            });
    public boolean lift;
    boolean lowerangle = false;
    private Boolean preloadspinreal = false;

    Command preloadSpun = new LambdaCommand().setStart(() -> preloadspinreal = true);
    Command preloadSpunReal = new LambdaCommand().setStart(() -> preloadspinreal = false);
    public boolean intake3 = false;
    Command shootIntake3 = new LambdaCommand()
            .setStart(()->intake3=true);
    private double currentTurretPos = 180.0;
    public boolean intakeSpam = false;

    Command shootIntakeSpam = new LambdaCommand()
            .setStart(()->intakeSpam=true);
    public double getClosestValidTurretAngle(double relativeGoalDegrees) {
        double option1 = relativeGoalDegrees;

        double option2 = (option1 > 180.0) ? (option1 - 360.0) : (option1 + 360.0);

        boolean opt1Valid = (option1 >= MIN_ANGLE && option1 <= MAX_ANGLE);
        boolean opt2Valid = (option2 >= MIN_ANGLE && option2 <= MAX_ANGLE);

        if (opt1Valid && opt2Valid) {
            return (Math.abs(option1 - currentTurretPos) < Math.abs(option2 - currentTurretPos)) ? option1 : option2;
        }

        if (opt1Valid) return option1;
        if (opt2Valid) return option2;

        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, option1));
    }
    boolean autoShoot = false;
    public Command autoShoote = new LambdaCommand()
            .setStart(()->autoShoot = true);

    public Command Auto(){
        return new SequentialGroup(
                new FollowPath(paths.path1)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();

        preloadspinreal = true;
        shooter(1085);
        autoShoot= false;

        Auto().schedule();
    }

    @Override
    public void onUpdate(){
        follower.update();

        Pose currPose = follower.getPose();
        double robotHeading = currPose.getHeading();
        Vector robotToGoalVector = new Vector(currPose.distanceFrom(new Pose(138, 141)), Math.atan2(141 - currPose.getY(), 138 - currPose.getX()));

        Double[] results = calculateShotVectorandUpdateHeadingWithAcceleration(robotHeading, robotToGoalVector, follower.getVelocity(), (Math.abs(follower.getVelocity().getMagnitude()) < 8) ? follower.getAcceleration() : new Vector(0,0));
        Double headingError = results[2];
        double flywheelSpeed = results[0];
        shooter((float) flywheelSpeed);
        double hoodAngle = results[1];
        hoodServo.setPosition(hoodAngle);

        double targetTurretAngle = getClosestValidTurretAngle(headingError);
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));
        if(servoPositionSignal>0.2&&servoPositionSignal<0.8) {
            turret1.setPosition(servoPositionSignal);
            turret2.setPosition(servoPositionSignal);
        }
        currentTurretPos=((turret1.getPosition() - 0.05) / 0.90) * 449.51 - 44.75;

        ActiveOpMode.telemetry().addData("launch?", isOverlappingLaunchZone(currPose));
        if(isOverlappingLaunchZone(currPose)&&autoShoot){
            intakeMotor.setPower(-1);
            transfer.setPower(1);
        }
        else{
            intakeMotor.setPower(0);
            transfer.setPower(0);
        }

        Storage.currentPose = currPose;
    }

    @Override
    public void onStop() {
        Storage.currentPose = follower.getPose();
        follower.breakFollowing();
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }

    public class Paths {
        public PathChain path1;
        public PathChain path2;
        public PathChain path3;
        public PathChain path4;
        public PathChain path5;
        public PathChain path6;
        public PathChain path7;
        public PathChain path8;
        public PathChain path9;
        public PathChain path10;
        public PathChain path11;
        public PathChain path12;
        public PathChain path13;

        public Paths(Follower follower) {
            path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(109.810, 133.272),
                                    new Pose(95.005, 94.650)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(240))
                    .addPoseCallback(new Pose(98.973,105.001),autoShoote,0.7)
                    .build();

            path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(95.005, 94.650),
                                    new Pose(84.764, 66.002),
                                    new Pose(108.822, 61.002),
                                    new Pose(125.790, 58.117)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(0))
                    .build();

            path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.790, 58.117),
                                    new Pose(80.854, 69.703)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80.854, 69.703),
                                    new Pose(131.042, 59.208)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();

            path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.042, 59.208),
                                    new Pose(82.058, 73.334)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(-15))
                    .build();

            path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.058, 73.334),
                                    new Pose(131.042, 59.208)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(40))
                    .build();

            path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.042, 59.208),
                                    new Pose(82.100, 73.334)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(-15))
                    .build();

            path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.100, 73.334),
                                    new Pose(131.042, 59.208)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(40))
                    .build();

            path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.042, 59.208),
                                    new Pose(82.100, 73.334)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(-15))
                    .build();

            path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.100, 73.334),
                                    new Pose(131.042, 59.208)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(40))
                    .build();

            path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.042, 59.208),
                                    new Pose(82.000, 73.334)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(-15))
                    .build();

            path12 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.000, 73.334),
                                    new Pose(87.809, 79.238),
                                    new Pose(119.796, 81.183)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(30))
                    .build();

            path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.796, 81.183),
                                    new Pose(94.967, 83.711)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(270))
                    .build();
        }
    }
}