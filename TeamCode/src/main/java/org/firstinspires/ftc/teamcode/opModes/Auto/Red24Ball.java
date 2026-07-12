package org.firstinspires.ftc.teamcode.opModes.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.servoOffset;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.closeStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.openStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Autonomous
@Configurable
public class Red24Ball extends NextFTCOpMode {

    public Red24Ball() {
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

    public static double startX = 107.17; // 142 - 33.83
    public static double startY = 134.4;

    public Pose start = new Pose(startX, startY, Math.toRadians(270));

    // --- Turret tracking ---
    private ServoEx servoStopper;
    private ServoEx hoodServo;

    double goalY = 140;
    double goalX = 141;

    public static double gateX = 130; // 142 - 11.8
    public static double gateY = 60;

    public static double gateX2 = 131.5; // 142 - 11.2
    public static double gateY2 = 61;

    public static double PivotPoseX = 110;

    public static double PivotPoseY = 70;

    public static double controlX = 7;

    public static double controlY = -4.5;

    public static double launchX = 80.7613;
    public static double launchY = 87.9072;

    public static double gateHeading = 38;

    public static double gateX1 = 131.5; // 142 - 10.5
    public static double gateY1 = 59;


    public static double gateHeading1 = 26;

    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE = 224.75;
    private static final double TURRET_RANGE = 449.51;

    private double currentTurretPos = 90;

    public static double turretHeading4 = -140;

    private boolean matchStarted = false;
    private boolean autoShoot = false;
    private boolean useAutoGoalTracking = true;

    // --- Goal Tracking Override Flags ---
    private boolean isOverridden = false;
    private double overriddenTurretAngle = 0.0;

    private MotorEx intakeMotor;
    private ServoImplEx turret1;
    private ServoImplEx turret2;

    public static double turretOffset = -20;
    public static double turretOffsetStep = -5;

    // Inches from the Pinpoint/Pedro robot pose origin to the turret pivot.
    public static double turretForwardOffset = -0.52588;
    public static double turretStrafeOffset = 0;

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

    private List<LynxModule> allHubs;
    public Pose currPose;

    public double getClosestValidTurretAngle(double relativeGoalDegrees) {
        double option1 = normalizeDegrees(relativeGoalDegrees);
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, option1));
    }

    private double normalizeDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees <= -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    private Pose getTurretPose(Pose robotPose) {
        double heading = robotPose.getHeading();

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double turretX = robotPose.getX()
                + turretForwardOffset * cos
                - turretStrafeOffset * sin;

        double turretY = robotPose.getY()
                + turretForwardOffset * sin
                + turretStrafeOffset * cos;

        return new Pose(turretX, turretY, heading);
    }

    private Vector getTurretToGoalVector(Pose turretPose) {
        return new Vector(
                turretPose.distanceFrom(new Pose(goalX, goalY)),
                Math.atan2(goalY - turretPose.getY(), goalX - turretPose.getX())
        );
    }

    //public SequentialGroup shoot = new SequentialGroup(
    //servoOpen,
    //new Delay(0.3)
    //servoClose
    //);

    double targetTurretAngle;




    public boolean manualTPS = true;






    public Command autoShootEnable(){
        return new LambdaCommand()
                .setStart(()->autoShoot = true);
    }
    // --- Custom Override Tracking Commands ---
    public Command setTurretHeading(double degrees) {
        return new LambdaCommand("Set Turret Heading: " + degrees)
                .setStart(() -> {
                    isOverridden = true;
                    overriddenTurretAngle = getClosestValidTurretAngle(degrees);
                })
                .setIsDone(() -> true);
    }

    /*public Command enableGoalTracking() {
        return new LambdaCommand("Enable Goal Tracking")
                .setStart(() -> {
                    isOverridden = false;
                })
                .setIsDone(() -> true);
    }*/

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    // ----------------------

    @Override
    public void onInit() {

        allHubs = ActiveOpMode.hardwareMap().getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = PedroComponent.follower();

        follower.setStartingPose(start);

        paths = new Paths(follower);

        opmodeTimer = new Timer();

        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");

        turret1 = ActiveOpMode.hardwareMap().get(
                ServoImplEx.class,
                "turretServo1");

        turret2 = ActiveOpMode.hardwareMap().get(
                ServoImplEx.class,
                "turretServo2");

        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        isOverridden = true;
        preload = true;



        overriddenTurretAngle = getClosestValidTurretAngle(150);

        double robotAngularVelocityRads = follower.getAngularVelocity();
        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
        double feedforwardOffset = 0;

        targetTurretAngle = getClosestValidTurretAngle(overriddenTurretAngle-27 - feedforwardOffset);
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

        turret1.setPosition(servoPositionSignal + servoOffset);
        turret2.setPosition(servoPositionSignal - servoOffset);
        double lastServoPos = servoPositionSignal;


        currentTurretPos = targetTurretAngle;
        //enableGoalTracking();


        hoodServo = new ServoEx("hoodServo");

        servoStopper = new ServoEx("stopperServo");

        telemetry.addLine("Initialized");
        telemetry.update();

        //setTurretHeading(turretHeading1).schedule();

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

    public Command Pivot = new LambdaCommand()
            .setStart(()->{
                follower.turnTo(Math.toRadians(40));
            }).setIsDone(()->true);
    public Command enableGoalTracking() {
        return new LambdaCommand("Enable Goal Tracking")
                .setStart(() -> {
                    isOverridden = false;
                })
                .setIsDone(() -> true);
    }


    public Command Auto() {
        return new SequentialGroup(
                (Command) new Delay(0.3),
                new FollowPath(paths.Preload, false, 1.0),
                enableGoalTracking(),
                disablePreload,
                intakeMotorOn,
                openStopper,
                new Delay(0.2),
                closeStopper,
                new FollowPath(paths.Spike2, false, 1.0),
                new FollowPath(paths.launchSpike2, false, 1.0),
                //6 balls scored, spike 2 + preloads
                new FollowPath(paths.gateIntake, false, 1.0),
                new Delay(1.05),
                new FollowPath(paths.launchFromGate, false, 1.0),
                //9 balls scored
                new FollowPath(paths.gateIntake, false, 1.0),
                new Delay(2.25),
                new FollowPath(paths.launchFromGate, false, 1.0),
                //12 balls scored
                new FollowPath(paths.gateIntake, false, 1.0),
                new Delay(2.25),
                new FollowPath(paths.launchFromGate, false, 1.0),
                //15 balls scored
                new FollowPath(paths.intakeSpike1, false, 1.0),
                intakeMotorOff,
                new FollowPath(paths.launchSpike1, false, 1.0),
                //18 balls scored
                new FollowPath(paths.gateIntake, false, 1.0),
                new Delay(1.05),
                new FollowPath(paths.launchFromGate, false, 1.0),
                //21 balls scored
                new FollowPath(paths.gateIntake, false, 1.0),
                new Delay(2.25),
                new FollowPath(paths.launchFromGate, false, 1.0)
                //24 balls scored
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;
        //shooter(2500);
        Auto().schedule();
    }
    private boolean preload = true;

    public Command disablePreload = new LambdaCommand()
            .setStart(()->preload=false);
    private double flywheelSpeed;


    @Override
    public void onUpdate() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.update();

        Storage.currentPose = follower.getPose();

        if (!matchStarted) return;

        Pose currPose = follower.getPose();

        Pose turretPose = getTurretPose(currPose);

        double robotHeading = follower.getPose().getHeading();

        Vector robotToGoalVector = getTurretToGoalVector(turretPose);

        Double[] results = calculateShotVectorandUpdateHeading(
                robotHeading,
                robotToGoalVector,
                follower.getVelocity().times(1.0),
                1.25);

        flywheelSpeed = results[0];

        if (preload == true) {

            shooter((float) flywheelSpeed + 30);

        }

        if (preload == false) {

            //turretOffset = 17;

            shooter((float) flywheelSpeed-10);
        }

        double hoodAngle = results[1];

        hoodServo.setPosition(hoodAngle);

        double headingError = results[2];

        double robotAngularVelocityRads = follower.getAngularVelocity();

        double robotAngularVelocityDegs =
                Math.toDegrees(robotAngularVelocityRads);

        double feedforwardOffset =
                robotAngularVelocityDegs * 0.225;

        double targetTurretAngle;

        if(!isOverridden) {

            targetTurretAngle = getClosestValidTurretAngle(headingError + turretOffset - feedforwardOffset);
        }
        else{
            targetTurretAngle = getClosestValidTurretAngle(overriddenTurretAngle+turretOffset - feedforwardOffset);
        }
        double servoPositionSignal =
                0.05
                        + ((targetTurretAngle - MIN_ANGLE)
                        / 449.51)
                        * 0.90;

        servoPositionSignal =
                Math.max(0.05,
                        Math.min(0.95, servoPositionSignal));

        turret1.setPosition(servoPositionSignal);
        turret2.setPosition(servoPositionSignal);

        currentTurretPos = targetTurretAngle;

        Pose futurepose = new Pose(
                follower.getPose().getX()
                        + follower.getVelocity().getXComponent() * 0.5,

                follower.getPose().getY()
                        + follower.getVelocity().getYComponent() * 0.3,

                follower.getHeading());

        if (isOverlappingLaunchZone(futurepose)
                && robotToGoalVector.getMagnitude() > 45) {

            intakeMotor.setPower(1);
            transfer.setPower(1);

            openStopper.schedule();

        } else {

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


        //dih
        public PathChain Preload;
        public PathChain Spike2;
        public PathChain launchSpike2;
        public PathChain gateIntake1;

        public PathChain gateIntake;

        public PathChain intakeSpike1;

        public PathChain launchSpike1;

        public PathChain launchFromGate;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public PathChain Pivot2;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;
        public PathChain PivotPath;

        public Paths(Follower follower) {

            Preload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(startX, startY),
                            new Pose(92.505, 94.650)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(270),
                            Math.toRadians(240))
                    .build();

            Spike2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(92.505, 94.650),
                            new Pose(90.98, 59.7),
                            new Pose(123, 59.5)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(240),
                            Math.toRadians(20))
                    .build();

            launchSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(123, 59.5),
                                    new Pose(119.95007385524372, 53.966543574593786),
                                    new Pose(launchX, launchY)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)


                    .build();

            /*gateIntake1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(launchX, launchY),
                            new Pose(118.5+gateXtranslation+controlX,52+gateYtranslation+controlY),
                            new Pose(129.7+gateXtranslation, 60.5+gateYtranslation)))
                    .setTangentHeadingInterpolation()
                    .build();*/

            gateIntake = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(launchX, launchY),
                            new Pose(PivotPoseX, PivotPoseY)))
                    .setTangentHeadingInterpolation()
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addPath(new BezierLine(
                            new Pose(PivotPoseX, PivotPoseY),
                            new Pose(gateX, gateY)))
                    .setConstantHeadingInterpolation(
                            Math.toRadians(gateHeading))
                    .build();

            launchFromGate = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX, gateY),
                            //new Pose(118.5+gateXtranslation+controlX,52+gateYtranslation+controlY),
                            new Pose(launchX, launchY)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeSpike1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(launchX, launchY),
                            new Pose(102.95903256,79.415),
                            new Pose(121.5, 83.8)))
                    .setTangentHeadingInterpolation()
                    .build();

            launchSpike1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(121.5, 83.8),
                            new Pose(103.0568,78.6283415),
                            new Pose(launchX, launchY)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(79.558, 73.32),
                            new Pose(gateX+0.5, gateY-1)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(345),
                            Math.toRadians(gateHeading))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX+0.5, gateY-1),
                            new Pose(79.558, 75)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(gateHeading),
                            Math.toRadians(345))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(79.558, 75),
                            new Pose(gateX+0.5, gateY-1)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(345),
                            Math.toRadians(gateHeading))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX+0.5, gateY-1),
                            new Pose(79.558, 75)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(gateHeading),
                            Math.toRadians(345))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(92.4669596239, 83.71111297536585),
                            new Pose(100.519, 63.28),
                            new Pose(gateX2+0.5, gateY2-1)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(345),
                            Math.toRadians(gateHeading))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX2+0.5, gateY2-1),
                            new Pose(93.31, 92)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(gateHeading),
                            Math.toRadians(35))
                    //toRed(215))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(93.31, 92),
                            new Pose(106.414, 66.749),
                            new Pose(gateX2+0.5, gateY2-1)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(35),
                            Math.toRadians(gateHeading))
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX2+0.5, gateY2-1),
                            new Pose(81.5, 104.0)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(gateHeading),
                            Math.toRadians(90))
                    .build();

            Path14 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(79.558, 75),
                            new Pose(85.3086685039, 79.2375828716257),
                            new Pose(121.5, 83.8)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(345),
                            Math.toRadians(0))
                    .build();

            Path15 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(118.5, 83.8),
                            new Pose(92.4669596239, 83.71111297536585)))
                    .setConstantHeadingInterpolation(
                            Math.toRadians(0))
                    .build();
        }
    }
}