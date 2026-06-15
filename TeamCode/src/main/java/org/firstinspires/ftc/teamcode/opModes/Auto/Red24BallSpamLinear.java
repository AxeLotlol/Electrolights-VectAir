package org.firstinspires.ftc.teamcode.opModes.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.closeStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.openStopperPos;
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
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
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
    private MotorEx transfer;
    private Timer opmodeTimer;
    private Paths paths;

    public Pose start = new Pose(109.810, 132.272, Math.toRadians(270));

    // --- Turret tracking ---
    private ServoEx turret1;
    private ServoEx servoStopper;
    private ServoEx turret2;
    private ServoEx hoodServo;
    private MotorEx intakeMotor;
    double goalY = 144;
    double goalX = 144;
    public static double gateX = 132;
    public static double gateY = 54.85;

    public static double gateHeading = 30;

    public static double gateX1 = 131.4;
    public static double gateY1 = 56;
    public static double turretHeading1=60;
    public static double turretHeading2=55;
    public static double turretHeading3=75;
    public static double gateHeading1 = 30;
    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE =  224.75;
    private static final double TURRET_RANGE =  449.51;
    private double currentTurretPos = 180.0;

    private boolean matchStarted = false;

    private boolean useAutoGoalTracking = true;

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(1);
                transfer.setPower(1);
            });

    private Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(0);
                transfer.setPower(0);
            });

    public Command servoOpen = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(0.96);
            });

    public Command servoClose = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(0.86);
            });

    //public SequentialGroup shoot = new SequentialGroup(
            //servoOpen,
            //new Delay(0.3)
            //servoClose
    //);

    double targetTurretAngle;


    public Command setTurretHeading(double targetAngleDegrees) {
        return new LambdaCommand()
                .setStart(() -> {
                    useAutoGoalTracking = false;
                    targetTurretAngle = getClosestValidTurretAngle(targetAngleDegrees);
                });
    }


    public Command enableGoalTracking() {
        return new LambdaCommand()
                .setStart(() -> useAutoGoalTracking = true);
    }

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


    // ----------------------

    public void onInit() {
        follower = PedroComponent.follower();
        follower.setStartingPose(start);
        paths = new Paths(follower);
        opmodeTimer = new Timer();
        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");
        turret1  = new ServoEx("turretServo1");
        turret2  = new ServoEx("turretServo2");
        hoodServo = new ServoEx("hoodServo");
        servoStopper = new ServoEx("stopperServo");
        telemetry.addLine("Initialized");
        telemetry.update();
    }
    public Command closeStopper = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(closeStopperPos); // close
            }).setIsDone(() -> true);
    public Command openStopper = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(openStopperPos); // open
            }).setIsDone(() -> true);

    public Command Auto() {
        return new SequentialGroup(
                setTurretHeading(160),
                new FollowPath(paths.Path1, false, 1.0),

                intakeMotorOn,
                new FollowPath(paths.Path2, false, 1.0),
                setTurretHeading(turretHeading1),
                new FollowPath(paths.Path3, false, 1.0),

                //shoot,
                setTurretHeading(turretHeading2),





                new FollowPath(paths.Path4, true, 1.0),
                new Delay(1.2),
                new FollowPath(paths.Path5, false, 1.0),
                //shoot,

                new FollowPath(paths.Path6, true, 1.0),
                new Delay(1.8),

                new FollowPath(paths.Path7, false, 1.0),
                //shoot,

                new FollowPath(paths.Path8, true, 1.0),
                new Delay(1.8),

                new FollowPath(paths.Path9, false, 1.0),
                //shoot,
                new FollowPath(paths.Path10, true, 1.0),
                new Delay(1.8),
                new FollowPath(paths.Path11, false, 1.0),
                //shoot,
                new FollowPath(paths.Path12, true, 1.0),
                new Delay(1.8),
                new FollowPath(paths.Path13, false, 1.0),
                //shoot,
                setTurretHeading(turretHeading3),

                new FollowPath(paths.Path14, false, 1.0),
                intakeMotorOff,
                new FollowPath(paths.Path15, false, 1.0),
                //shoot,
                new FollowPath(paths.Path16, false, 1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;
        Auto().schedule();
    }

    @Override
    public void onUpdate() {
        follower.update();
        Storage.currentPose = follower.getPose();

        if (!matchStarted) return;

        // --- Continuous Tracking / Calculation Logic ---
        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();
        Vector robotToGoalVector = new Vector(follower.getPose().distanceFrom(new Pose(goalX, goalY)), Math.atan2(goalY - currPose.getY(), goalX - currPose.getX()));
        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity(), follower.getAcceleration());
        double flywheelSpeed = results[0];
        shooter((float) flywheelSpeed);
        double hoodAngle = results[1];
        hoodServo.setPosition(hoodAngle);

        // Only update targetTurretAngle from vector calculations if auto goal tracking is allowed.
        // If a command forced a static heading, onUpdate leaves 'targetTurretAngle' completely alone.
        if (useAutoGoalTracking) {
            double headingError = results[2];
            targetTurretAngle = headingError;
        }

        // --- Hardware Execution Loop ---
        // This blocks runs every frame, applying whatever position is inside 'targetTurretAngle'
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));
        turret1.setPosition(servoPositionSignal);
        turret2.setPosition(servoPositionSignal);

        currentTurretPos = ((turret1.getPosition() - 0.05) / 0.90) * 449.51 - 44.75;
        if(isOverlappingLaunchZone(PedroComponent.follower().getPose()) && robotToGoalVector.getMagnitude()>60){
            intakeMotor.setPower(1);
            transfer.setPower(1);
            openStopper.schedule();
        }
        else{
            closeStopper.schedule();
        }
        Storage.currentPose = follower.getPose();
        Storage.setPose = true;
    }

    @Override
    public void onStop() {
        Storage.currentPose = follower.getPose();
        follower.breakFollowing();
    }

    public class Paths {
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
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;
        public PathChain Path16;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(109.810, 132.272),
                            new Pose(95.005, 94.650)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(240))
                    //.addPoseCallback(new Pose(96.892,99.572), shoot,0.9 )
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(95.005, 94.650),
                            new Pose(84.764, 64),
                            new Pose(108.822, 61),
                            new Pose(125.790, 58.117)))
                    .setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(30))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(125.790, 58.117),
                            new Pose(80.854, 69.703)))
                    .setLinearHeadingInterpolation(Math.toRadians(30),Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(80.854, 69.703),
                            new Pose(gateX1, gateY1)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gateHeading1))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX1, gateY1),
                            new Pose(82.058, 73.32)))
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading1), Math.toRadians(-15))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(82.058, 73.328),
                            new Pose(gateX + 0.05, gateY)))
                    .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(gateHeading))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX + 0.05, gateY),
                            new Pose(82.100, 73.328)))
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(-15))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(82.100, 73.328),
                            new Pose(gateX + 0.1, gateY)))
                    .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(gateHeading))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX + 0.1, gateY),
                            new Pose(82.100, 73.328)))
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(-15))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(82.100, 73.328),
                            new Pose(gateX + 0.18, gateY)))
                    .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(gateHeading))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX + 0.18, gateY),
                            new Pose(82.000, 73.328)))
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(-15))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(82.000, 73.328),
                            new Pose(gateX + 0.22,gateY)))
                    .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(gateHeading))
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX + 0.22,gateY),
                            new Pose(82, 73.328)))
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(-15))
                    .build();

            Path14 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(82.000, 73.328),
                            new Pose(87.80866850393573, 79.2375828716257),
                            new Pose(119.796411454575153, 81.1828200643779)))
                    .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(0))
                    .build();

            Path15 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(119.796411454575153, 81.1828200643779),
                            new Pose(94.96695962391377, 83.71111297536585)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path16 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(94.967, 83.711),
                            new Pose(104.777, 81.009)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
    }
}