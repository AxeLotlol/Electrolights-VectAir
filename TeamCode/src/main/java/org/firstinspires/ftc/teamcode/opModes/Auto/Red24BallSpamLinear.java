package org.firstinspires.ftc.teamcode.opModes.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Airsort.transfer;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.hoodServo;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
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


@Autonomous 
@Configurable
public class Red24BallSpamLinear extends NextFTCOpMode {

    public Red24BallSpamLinear() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))
        );
    }

    private Follower follower;
    public static double gateTime = 1.32;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Paths paths;
    public MotorEx intakeMotor;

    int tagId = 0;
    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE = 224.75;

    public Pose start = new Pose(110.58652658884565, 133.1659559014267, Math.toRadians(270));

    private MotorEx transfer1;

    public static MotorEx flywheel = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    private static ServoEx turret1;
    private static ServoEx turret2;

    public void onInit() {
        telemetry.addLine("Initializing Follower...");
        telemetry.update();
        follower = PedroComponent.follower();

        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        paths = new Paths(follower);
        intakeMotor = new MotorEx("intakeMotor");
        transfer1 = new MotorEx("transferMotor");
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

        follower.update();
    }

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() ->{
                intakeMotor.setPower(1);
                transfer.setPower(1);}
            );

    Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(0);
                transfer.setPower(0);}
            );




    //public SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.05), intakeMotorOn, new Delay(0.3), intakeMotorOff, closeTransfer);

    private Boolean preloadspinreal = false;

    Command preloadSpun = new LambdaCommand().setStart(() -> preloadspinreal = true);
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

    public Command Auto() {
        return new SequentialGroup(
                new FollowPath(paths.preLoadShoot, true, 1.0),
                new FollowPath(paths.spike2, true, 1.0),
                new FollowPath(paths.launchSpike2, true, 1.0),
                new FollowPath(paths.gate1, true, 1.0),
                new Delay(gateTime),
                new FollowPath(paths.openForPartner, true, 1.0),

                new FollowPath(paths.spike1, true, 1.0),
                new FollowPath(paths.gate2Intake, true, 1.0),
                new Delay(gateTime),
                new FollowPath(paths.openForPartner, true, 1.0),

                new FollowPath(paths.gateShoot, true, 1.0),
                new FollowPath(paths.gateIntake, true, 1.0),
                new Delay(gateTime),
                new FollowPath(paths.openForPartner, true, 1.0),

                new FollowPath(paths.gateShoot, true, 1.0),
                new FollowPath(paths.gateIntake, true, 1.0),
                new Delay(gateTime),
                new FollowPath(paths.openForPartner, true, 1.0),

                new FollowPath(paths.gateShoot, true, 1.0),
                new FollowPath(paths.gateIntake, true, 1.0),
                new Delay(gateTime),
                new FollowPath(paths.openForPartner, true, 1.0),

                new FollowPath(paths.gateShoot, true, 1.0),
                new FollowPath(paths.park, true, 1.0)
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

        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity());
        Double headingError = results[2];
        double flywheelSpeed = results[0];
        shooter((float) flywheelSpeed);
        double hoodAngle = results[1];
        hoodServo.setPosition(hoodAngle);
        //if this doesnt work its cuz hoodServo isnt defined here and nithin is gay asf

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
            intakeMotor.setPower(1);
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
            public PathChain preLoadShoot;
            public PathChain spike2;
            public PathChain launchSpike2;
            public PathChain gate1;
            public PathChain openForPartner;
            public PathChain spike1;
            public PathChain gate2Intake;
            public PathChain gateIntake;
            public PathChain gateShoot;
            public PathChain park;

            public Paths(Follower follower) {


                preLoadShoot = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(110.587, 133.166),
                                        new Pose(97.723, 87.429)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setConstantHeadingInterpolation(Math.toRadians(270))
                        .build();

                spike2 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(97.723, 87.429),
                                        new Pose(85.512, 58.943),
                                        new Pose(104.243, 59.193)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        new Pose(104.243, 59.193),
                                        new Pose(126.044, 59.193)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setTangentHeadingInterpolation()
                        .build();


                launchSpike2 = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(126.044, 59.193),
                                        new Pose(84.228, 69.272)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();


                gate1 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(84.228, 69.272),
                                        new Pose(101.121, 49.603),
                                        new Pose(134.754, 60.596)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                        .build();


                openForPartner = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(134.754, 60.596),
                                        new Pose(131.710, 59.133),
                                        new Pose(131.154, 59.202),
                                        new Pose(130.639, 59.300),
                                        new Pose(130.164, 59.427),
                                        new Pose(129.729, 59.584),
                                        new Pose(129.335, 59.770),
                                        new Pose(128.981, 59.986),
                                        new Pose(128.667, 60.230),
                                        new Pose(128.394, 60.505),
                                        new Pose(128.161, 60.808),
                                        new Pose(127.968, 61.141),
                                        new Pose(127.816, 61.503),
                                        new Pose(127.704, 61.895),
                                        new Pose(131.138, 65.082)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();


                spike1 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(131.138, 65.082),
                                        new Pose(95.715, 67.249),
                                        new Pose(96.535, 82.990)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        new Pose(96.535, 82.990),
                                        new Pose(123.903, 82.439)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setTangentHeadingInterpolation()
                        .addPath(
                                new BezierLine(
                                        new Pose(123.903, 82.439),
                                        new Pose(96.535, 82.439)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build();


                gate2Intake = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(96.535, 82.439),
                                        new Pose(101.121, 49.603),
                                        new Pose(134.754, 60.596)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                        .build();


                gateIntake = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(83.921, 69.318),
                                        new Pose(101.121, 49.603),
                                        new Pose(134.754, 60.596)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                        .build();


                gateShoot = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(131.138, 65.082),
                                        new Pose(95.715, 67.249),
                                        new Pose(83.921, 69.318)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(0))
                        .build();


                park = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(83.921, 69.318),
                                        new Pose(128.904, 83.589)
                                )
                        )
                        .setTValueConstraint(0.8)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();
            }
        }
}