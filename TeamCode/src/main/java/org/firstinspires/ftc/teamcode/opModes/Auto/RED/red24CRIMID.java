package org.firstinspires.ftc.teamcode.opModes.Auto.RED;

import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.closeStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.openStopperPos;
import static org.firstinspires.ftc.teamcode.subsystems.DriveTrain2.servoOffset;
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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Autonomous(name = "Red CRI NEAR 24 V1")
@Configurable
public class red24CRIMID extends NextFTCOpMode {

    public red24CRIMID() {
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

    public static double startX = 107.17;
    public static double startY = 134.4;

    public Pose start = new Pose(startX, startY, Math.toRadians(270));

    // --- Turret tracking ---
    private ServoEx servoStopper;
    private ServoEx hoodServo;

    double goalY = 140;
    double goalX = 141;

    public static double gateX = 130.5; // 142 - 11.8
    public static double gateY = 60;

    public static double gateX2 = 130.5; // 142 - 11.2
    public static double gateY2 = 61;

    public static double PivotPoseX = 110;

    public static double PivotPoseY = 70;

    public static double controlX = 7;

    public static double controlY = -4.5;

    public static double launchX = 80.7613;
    public static double launchY = 87.9072;

    public static double gateHeading = 29;

    public static double gateX1 = 131.5; // 142 - 10.5
    public static double gateY1 = 59;


    public static double gateHeading1 = 29;

    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE = 224.75;
    private static final double TURRET_RANGE = 449.51;

    private double currentTurretPos = 90;

    public static double turretHeading4 = -140;

    private boolean matchStarted = false;
    private boolean autoShoot = false;
    private boolean useAutoGoalTracking = true;


    private boolean isOverridden = false;
    private double overriddenTurretAngle;

    private MotorEx intakeMotor;
    private ServoImplEx turret1;
    private ServoImplEx turret2;

    public static double turretOffset = 8;
    public static double turretOffset2 = 2;
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
        hoodServo = new ServoEx("hoodServo");

        servoStopper = new ServoEx("stopperServo");

        isOverridden = true;
        preload = true;



        overriddenTurretAngle = getClosestValidTurretAngle(160);
        double hoodAngle = 0.4;
        hoodServo.setPosition(hoodAngle);
        servoStopper.setPosition(closeStopperPos);
        double robotAngularVelocityRads = follower.getAngularVelocity();
        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
        double feedforwardOffset = 0;

        targetTurretAngle = getClosestValidTurretAngle(overriddenTurretAngle-turretOffset - feedforwardOffset);
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

        turret1.setPosition(servoPositionSignal + servoOffset);
        turret2.setPosition(servoPositionSignal - servoOffset);
        double lastServoPos = servoPositionSignal;


        currentTurretPos = targetTurretAngle;
        //enableGoalTracking();




        telemetry.addLine("Initialized");
        telemetry.update();

        //setTurretHeading(turretHeading1).schedule();



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

                setTurretHeading(overriddenTurretAngle),
                new Delay(0.3),

                new FollowPath(paths.Path1, true, 1.0),

                intakeMotorOn,
                openStopper,
                new Delay(0.2),
                closeStopper,
                disablePreload,

                //autoShootEnable(),

                new FollowPath(paths.Path2, true, 1.0),
                new FollowPath(paths.Path3, true, 1.0),
                //new FollowPath(paths.Intake, true, 1.0),
                //new FollowPath(paths.Shoot, true, 1.0),
                new FollowPath(paths.Path4, true, 1.0),
                new FollowPath(paths.Path5,true,1.0),
                new FollowPath(paths.Path6, true, 1.0),
                new FollowPath(paths.Path7, true, 1.0),
                new FollowPath(paths.Path8, true, 1.0),

                new FollowPath(paths.Path9, true, 1.0),
                new FollowPath(paths.Path10, true, 1.0),

                new FollowPath(paths.Path11, true, 1.0),
                new FollowPath(paths.Path12, true, 1.0),

                new FollowPath(paths.Path13, true, 1.0),
                new FollowPath(paths.Path14, true, 1.0),
                intakeMotorOff,
                new FollowPath(paths.Path15, true, 1.0),
                intakeMotorOn,
                new FollowPath(paths.Path16, true, 1.0),

                new FollowPath(paths.Path17, true, 1.0),
                new FollowPath(paths.Path18, true, 1.0)
                
        );
    }


    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;
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
                follower.getVelocity().times(1.0), 1.25);

        flywheelSpeed = results[0];

        if (preload == true) {
            double hoodAngle = results[1];
            hoodServo.setPosition(hoodAngle);
            shooter((float) flywheelSpeed + 30);
            double robotAngularVelocityRads = follower.getAngularVelocity();
            double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
            double feedforwardOffset = 0;

            targetTurretAngle = getClosestValidTurretAngle(overriddenTurretAngle-turretOffset - feedforwardOffset);
            double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
            servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

            turret1.setPosition(servoPositionSignal + servoOffset);
            turret2.setPosition(servoPositionSignal - servoOffset);
            double lastServoPos = servoPositionSignal;


            currentTurretPos = targetTurretAngle;


        }

        if (preload == false) {
            shooter((float) flywheelSpeed);
            double hoodAngle = results[1];
            hoodServo.setPosition(hoodAngle);
            double headingError = results[2];
            double robotAngularVelocityRads = follower.getAngularVelocity();
            double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
            double feedforwardOffset = robotAngularVelocityDegs * 0.225;
            targetTurretAngle = getClosestValidTurretAngle(headingError + turretOffset - feedforwardOffset);
            double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
            servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

            turret1.setPosition(servoPositionSignal + servoOffset);
            turret2.setPosition(servoPositionSignal- servoOffset);

            currentTurretPos = targetTurretAngle;
        }



        Pose futurepose = new Pose(follower.getPose().getX() + (follower.getVelocity().getXComponent() * 0.25), follower.getPose().getY() + (follower.getVelocity().getYComponent() * 0.25), follower.getHeading());

        if (isOverlappingLaunchZone(futurepose) && robotToGoalVector.getMagnitude() > 45) {
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
//ayush loves anjana and moksh is less chopped than deb am i right? yes im always right heehheheheheheheeheheheheh lwk mithun is angy at me hes kinda mean lol


        //dih
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
        public PathChain Path17;
        public PathChain Path18;
        public PathChain Intake;
        public PathChain Shoot;
        double ControlX = 180;
        double ControlY = 74;
        double GoalX = 143;
        double GoalY = 78;
        double ShootX = 111;
        double ShootY = 75;



        public Paths(Follower follower) {


            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(110.318, 159.809),
                                    new Pose(115.44244186046514, 104.0113811841796),
                                    new Pose(151.765, 83.457)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
            //shoot
            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(151.765, 83.457),
                                    new Pose(178.760, 82.983)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            //shoot
            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(178.760, 82.983),
                                    new Pose(141.976, 84.243),
                                    new Pose(118.269, 58.246)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            //shoot
            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(178.114, 71.351),
                                    new Pose(137.914, 66.277),
                                    new Pose(115.355, 57.131)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            //shoot
            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(177.381, 75.469),
                                    new Pose(147.138, 69.391),
                                    new Pose(114.857, 59.219)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            //shoot
            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(117.146, 57.693),
                                    new Pose(177.381, 75.469)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(178.525, 71.870),
                                    new Pose(148.901, 69.326),
                                    new Pose(117.146, 57.693)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            //shoot
            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(115.355, 57.131),
                                    new Pose(138.264, 64.668),
                                    new Pose(178.525, 71.870)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(116.302, 57.054),
                                    new Pose(141.850, 74.919),
                                    new Pose(178.114, 71.351)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            //shoot
            Path10 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(178.207, 71.338),
                                    new Pose(138.057, 75.495),
                                    new Pose(116.302, 57.054)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            Path11 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(119.539, 54.536),
                                    new Pose(149.076, 67.407),
                                    new Pose(178.207, 71.338)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            //shoot
            Path12 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(176.834, 81.138),
                                    new Pose(163.204, 78.852),
                                    new Pose(119.539, 54.536)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            Path13 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(115.132, 58.943),
                                    new Pose(145.977, 83.725),
                                    new Pose(176.834, 81.138)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            //shoot
            Path14 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(179.860, 75.194),
                                    new Pose(147.545, 80.987),
                                    new Pose(115.132, 58.943)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            Path15 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(118.162, 60.871),
                                    new Pose(155.346, 78.775),
                                    new Pose(179.860, 75.194)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            //shoot
            Path16 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(179.033, 82.906),
                                    new Pose(150.113, 88.690),
                                    new Pose(118.162, 60.871)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            Path17 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(118.269, 58.246),
                                    new Pose(139.095, 87.864),
                                    new Pose(179.033, 82.906)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();


            Intake = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(ShootX,ShootY),
                                    new Pose(ControlX,ControlY),
                                    new Pose(GoalX,GoalY)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            Intake = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(GoalX,GoalY),
                                    new Pose(ControlX,ControlY),
                                    new Pose(ShootX,ShootY)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
    }
}