package org.firstinspires.ftc.teamcode.opModes.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.AutoShooterCalc.calculateShotVectorandUpdateHeading;
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
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Autonomous
@Configurable
public class Blue24BallSpamLinearPivot extends NextFTCOpMode {

    public Blue24BallSpamLinearPivot() {
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

    public static double startX = 33.83; // 144 - 110.17
    public static double startY = 134.4;
    public Pose start = new Pose(startX, startY, Math.toRadians(-90)); // 180 - 270

    // --- Turret tracking ---
    private ServoEx servoStopper;
    private ServoEx hoodServo;

    double goalY = 144;
    double goalX = 0; // 144 - 144
    public static double gateX = 11; // 144 - 133.2
    public static double gateY = 59.25;

    public static double gateHeading = 138.75; // 180 - 41.25

    public static double gateX1 = 10.5; // 144 - 133.5
    public static double gateY1 = 58.75;
    public static double turretHeading1 = 120; // 180 - 60
    public static double turretHeading2 = 125; // 180 - 55
    public static double turretHeading3 = 105; // 180 - 75
    public static double gateHeading1 = 138; // 180 - 42
    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE =  224.75;
    private static final double TURRET_RANGE =  449.51;
    private double currentTurretPos = 180.0;

    private boolean matchStarted = false;
    private boolean autoShoot = false;
    private boolean useAutoGoalTracking = true;
    private MotorEx intakeMotor;
    private ServoImplEx turret1;
    private ServoImplEx turret2;

    public static double turretOffset = 6;
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

    public Command servoOpen = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(0.96);
            });

    public Command servoClose = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(0.86);
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
        double turretX = robotPose.getX()
                + turretForwardOffset * Math.cos(heading)
                - turretStrafeOffset * Math.sin(heading);
        double turretY = robotPose.getY()
                + turretForwardOffset * Math.sin(heading)
                + turretStrafeOffset * Math.cos(heading);

        return new Pose(turretX, turretY, heading);
    }

    private Vector getTurretToGoalVector(Pose turretPose) {
        return new Vector(
                turretPose.distanceFrom(new Pose(goalX, goalY)),
                Math.atan2(goalY - turretPose.getY(), goalX - turretPose.getX())
        );
    }

    double targetTurretAngle;

    public boolean manualTPS = true;

    public Command autoShootEnable(){
        return new LambdaCommand()
                .setStart(()->autoShoot = true);
    }
    public Command turnOffManualtps(){
        return new LambdaCommand()
                .setStart(()->manualTPS=false);
    }
    public Command turnOffPreload(){
        return new LambdaCommand()
                .setStart(()->preload=false);
    }

    public static MotorEx flywheel = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

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
        turret1 = ActiveOpMode.hardwareMap().get(ServoImplEx.class, "turretServo1");
        turret2 = ActiveOpMode.hardwareMap().get(ServoImplEx.class,"turretServo2");
        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hoodServo = new ServoEx("hoodServo");
        servoStopper = new ServoEx("stopperServo");
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    public Command turnTo(Pose targetPose, double timeoutSeconds) {
        return new LambdaCommand("Turn to " + targetPose.getHeading())
                .setStart(() -> follower.holdPoint(targetPose))
                .setIsDone(() -> false)
                .raceWith(new Delay(timeoutSeconds));
    }
    private boolean manualShooting = true;

    public Command closeStopper = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(closeStopperPos);
            }).setIsDone(() -> true);
    public Command openStopper = new LambdaCommand()
            .setStart(() -> {
                servoStopper.setPosition(openStopperPos);
            }).setIsDone(() -> true);

    public Command Auto() {
        return new SequentialGroup(
                new FollowPath(paths.Path1, false, 1.0),

                intakeMotorOn,
                new FollowPath(paths.Path2, false, 1.0),
                turnOffPreload(),

                new FollowPath(paths.Path3, false, 1.0),
                new FollowPath(paths.Path4, true, 1.0),
                new FollowPath(paths.Path16,true,1.0),
                new Delay(1.25),

                new FollowPath(paths.Path5, false, 1.0),

                new FollowPath(paths.Path6, true, 1.0),
                new Delay(2.25),

                new FollowPath(paths.Path7, false, 1.0),

                new FollowPath(paths.Path8, true, 1.0),
                new Delay(2.25),

                new FollowPath(paths.Path9, false, 1.0),
                new FollowPath(paths.Path14, false, 1.0),
                intakeMotorOff,
                new FollowPath(paths.Path15, false, 1.0),
                intakeMotorOn,
                new FollowPath(paths.Path10, true, 1.0),
                new Delay(1.25),
                new FollowPath(paths.Path11, false, 1.0),
                new FollowPath(paths.Path12, true, 1.0),
                new Delay(2.25),
                new FollowPath(paths.Path13, false, 1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;
        shooter(2200);
        Auto().schedule();
    }
    private boolean preload = true;
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
        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity().times(1), follower.getAcceleration());

        flywheelSpeed = results[0];

        if(preload==true){
            shooter(2200);
        }
        if(preload==false){
            shooter((float) flywheelSpeed - 38);
        }
        double hoodAngle = results[1];
        hoodServo.setPosition(hoodAngle);

        double headingError = results[2];
        double robotAngularVelocityRads = follower.getAngularVelocity();
        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
        double feedforwardOffset = robotAngularVelocityDegs * 0.225;
        double targetTurretAngle = getClosestValidTurretAngle(headingError + turretOffset - feedforwardOffset);
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));
        turret1.setPosition(servoPositionSignal);
        turret2.setPosition(servoPositionSignal);
        currentTurretPos = targetTurretAngle;
        Pose futurepose = new Pose(follower.getPose().getX()+follower.getVelocity().getXComponent()*0.5, follower.getPose().getY()+follower.getVelocity().getYComponent()*0.5, follower.getHeading());

        if(isOverlappingLaunchZone(futurepose) && robotToGoalVector.getMagnitude()>45&&autoShoot){
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
                            new Pose(startX, startY),
                            new Pose(48.995, 94.650))) // 144 - 95.005
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-60)) // 180-270, 180-240
                    .addPoseCallback(new Pose(46.255,101.833), autoShootEnable(),0.8 ) // 144 - 97.745
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(48.995, 94.650),
                            new Pose(50.52, 59.7), // 144 - 93.48
                            new Pose(18.5, 59.5))) // 144 - 125.5
                    .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(160)) // 180-240, 180-20
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.5, 59.5), // 144 - 125.790
                            new Pose(63.146, 69.703))) // 144 - 80.854
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180)) // 180-20, 180-0
                    .build();

            Path4 = follower.pathBuilder() //gateItake
                    .addPath(new BezierLine(
                            new Pose(63.146, 69.703),
                            new Pose(14.86, 62.15))) // 144 - 133
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(175)) // 180-0, 180-5
                    .build();

            Path16 = follower.pathBuilder() //Pivot path
                    .addPath(new BezierLine(
                            new Pose(14.86, 62.15),
                            new Pose(14.96, 62.5))) // 144 - 132.8
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(143)) // 180-5, 180-37
                    .build();

            Path5 = follower.pathBuilder() //shoot
                    .addPath(new BezierLine(
                            new Pose(14.96, 62.5), // 144 - 132.5
                            new Pose(61.942, 73.32))) // 144 - 82.058
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading1), Math.toRadians(195)) // 180 - (-15)
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.942, 73.32),
                            new Pose(gateX, gateY)))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(gateHeading))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX, gateY),
                            new Pose(61.942, 73.32))) // 144 - 82.100
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(195))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.942, 73.32),
                            new Pose(gateX, gateY)))
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(gateHeading))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX, gateY),
                            new Pose(61.942, 73.32)))
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(195))
                    .build();

            Path10 = follower.pathBuilder() // gateintake
                    .addPath(new BezierCurve(
                            new Pose(49.03304037608623, 83.71111297536585), // 144 - 94.9669...
                            new Pose(40.981, 63.28), // 144 - 103.019
                            new Pose(12, 59.25)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(gateHeading)) // 180-0
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12, 59.25),
                            new Pose(48.19, 91.6))) // 144 - 95.81
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(215)) // 180 - (-35)
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(48.19, 91.6),
                            new Pose(35.086, 66.749), // 144 - 108.914
                            new Pose(13, 59.25)))
                    .setLinearHeadingInterpolation(Math.toRadians(215), Math.toRadians(gateHeading))
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(gateX, gateY),
                            new Pose(60.0, 104.0))) // 144 - 84
                    .setLinearHeadingInterpolation(Math.toRadians(gateHeading), Math.toRadians(90)) // 180-90
                    .build();

            Path14 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(61.942, 73.32), // 144 - 82
                            new Pose(56.19133149606427, 79.2375828716257), // 144 - 87.8086...
                            new Pose(23.0, 83.8))) // 144 - 121
                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(180))
                    .build();

            Path15 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(23.0, 83.8),
                            new Pose(49.03304037608623, 83.71111297536585)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}