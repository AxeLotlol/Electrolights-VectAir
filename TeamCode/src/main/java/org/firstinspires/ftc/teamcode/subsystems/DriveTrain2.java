package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue2.isBlue;

import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed2.isRed;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.requiredTPS;

import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
// import org.firstinspires.ftc.teamcode.vision.KalmanFilter;
// import org.firstinspires.ftc.teamcode.vision.Limelight;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

import java.util.List;
import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;


@Configurable
public class DriveTrain2 implements Subsystem {

    public static final DriveTrain2 INSTANCE = new DriveTrain2();

    public DriveTrain2() {
    }


    public static final MotorEx fL = new MotorEx("frontLeft").brakeMode();
    public static final MotorEx fR = new MotorEx("frontRight").brakeMode();
    public static final MotorEx bL = new MotorEx("backLeft").brakeMode();
    public static final MotorEx bR = new MotorEx("backRight").brakeMode();

    private IMUEx imu;
    public static double tolarance = 10;
    public double currentMotorSpeed = 0;

    public boolean firsttime = true;

    public static double servoOffset = 0.03;

    public int alliance;
    public boolean far;

    // Limelight + Kalman Filter (commented out until mount is ready)
    // private Limelight limelight;
    // private KalmanFilter kalmanFilter;
    // private boolean limelightEnabled = false;

    private ServoImplEx turret1;
    private ServoImplEx turret2;

    public static double turretOffset = 10;
    public static double turretOffset2 = 2;
    public static double turretOffsetStep = -5;
    // Inches from the Pinpoint/Pedro robot pose origin to the turret pivot.
    public static double turretForwardOffset = -0.52588;
    public static double turretStrafeOffset = 0;

    public static double openStopperPos = 0;
    public static double closeStopperPos = 0.05;
    public Command driveToGate = new LambdaCommand()
            .setStart(() -> dToGate = true);
    public static boolean dToGate = false;
    // Loop time tracking
    private long lastLoopTime = 0;
    private double loopTimeMs = 0;
    public Pose currPose;

    // OPTIMIZATION: Cache LynxModules to avoid allocation every loop
    private List<LynxModule> allHubs;
    // OPTIMIZATION: Reduce telemetry frequency
    private int telemetryCounter = 0;
    private static final int TELEMETRY_EVERY_N_LOOPS = 5;
    // OPTIMIZATION: Avoid redundant servo writes
    private double lastServoPos = -1;

    public Supplier<Double> yVCtx;

    /*public static double hoodToPos(double runtime) {
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
    }*/

    @Override
    public Command getDefaultCommand() {

        configureAllianceTarget();
        follower.update();
        currPose = follower.getPose();
        {
            return new MecanumDriverControlled(
                    fL,
                    fR,
                    bL,
                    bR,
                    Gamepads.gamepad1().leftStickX().map(it -> alliance * it),
                    Gamepads.gamepad1().leftStickY().map(it -> alliance * it),
                    Gamepads.gamepad1().rightStickX().map(it -> 0.8*it),
                    new FieldCentric(() -> Angle.fromRad(follower.getHeading() - Math.PI / 2))
            );
        }

        //return null;
    }

    public static MotorEx intakeMotor;
    public static MotorEx transfer;


    public Command localize;

    public static ServoEx hoodServo = new ServoEx("hoodServo");

    public Command turretzero = new LambdaCommand()
            .setStart(() -> {
                //`5transfer2.setPosition(-0.25);
                turret1.setPosition(0);
                turret2.setPosition(0);
            }).setIsDone(() -> true);
    public Command turrethalf = new LambdaCommand()
            .setStart(() -> {
                //`5transfer2.setPosition(-0.25);
                turret1.setPosition(0.5);
                turret2.setPosition(0.5);
            }).setIsDone(() -> true);


    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE = 224.75;

    // Tracks where the turret is across frames (Double, initialized to center)
    private double currentTurretPos = 0;
    public boolean wrapping = false;
    public static ServoEx stopperServo = new ServoEx("stopperServo");

    public double getClosestValidTurretAngle(double relativeGoalDegrees) {
        double option1 = normalizeDegrees(relativeGoalDegrees);
        wrapping = false;
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

    private void configureAllianceTarget() {
        if (isRed()) {
            alliance = -1;
            goalXDist = 140;
            goalX = 140;
            localizeX = 136;
        } else if (isBlue()) {
            alliance = 1;
            goalXDist = 3;
            goalX = 3;
            localizeX = 8;
        } else {
            ActiveOpMode.telemetry().addLine("No direction set");
        }
    }

    public static Command closeStopper = new LambdaCommand()
            .setStart(() -> {
                stopperServo.setPosition(closeStopperPos); // close
            }).setIsDone(() -> true);
    public static Command openStopper = new LambdaCommand()
            .setStart(() -> {
                stopperServo.setPosition(openStopperPos); // open
            }).setIsDone(() -> true);

    @Override
    public void initialize() {

        firsttime = true;

        shooting = false;
        follower = PedroComponent.follower();
        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");

        // OPTIMIZATION: Cache LynxModules once during initialization
        allHubs = ActiveOpMode.hardwareMap().getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        closeStopper.schedule();
        configureAllianceTarget();
        imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();
        Pose startingpose = new Pose(72, 72, Math.toRadians(90));
        if(Storage.setPose){
            startingpose = Storage.currentPose;
            Storage.setPose=false;
        }
        follower.setStartingPose(startingpose);

//        if (alliance == -1) {
//            localize = new LambdaCommand()
//                    .setStart(() -> follower.setPose(new Pose(129, 90, Math.toRadians(90))));
//
//        }
//        if (alliance == 1) {
//            localize = new LambdaCommand()
//                    .setStart(() -> follower.setPose(new Pose(15, 90, Math.toRadians(90))));
//
//        }
        turret1 = ActiveOpMode.hardwareMap().get(ServoImplEx.class, "turretServo1");
        turret2 = ActiveOpMode.hardwareMap().get(ServoImplEx.class,"turretServo2");
        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        follower.update();

        // OPTIMIZATION: Move hardware init from periodic() to initialize()


        // Limelight + Kalman Filter initialization (commented out until mount is ready)
        // limelight = new Limelight(ActiveOpMode.hardwareMap());
        // limelight.initialize();
        // kalmanFilter = new KalmanFilter(follower.getPose(), 0.05, 0.3);

        // Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> {
        //     limelightEnabled = !limelightEnabled;
        //     if (limelightEnabled) {
        //         limelight.start();
        //         ActiveOpMode.telemetry().addLine("Limelight: ON");
        //         Gamepads.gamepad1().rumble(100);
        //     } else {
        //         limelight.stop();
        //         ActiveOpMode.telemetry().addLine("Limelight: OFF");
        //         Gamepads.gamepad1().rumble(0.5, 0.5, 100);
        //     }
        // });
    }
    private boolean autoShoot = true;

    public Command toggleAutoShoot = new LambdaCommand()
            .setStart(()->{
                if(autoShoot){
                    autoShoot = false;
                }
                else{
                    autoShoot = true;
                }
            });

    private static MotorEx transfer1;
    private static ServoEx transfer2;

    double goalY = 140.5;
    double goalX = 140;

    static double localizeX;
    double goalXDist = 140;


    static boolean shooting = false;
    Command dToGateFalse = new LambdaCommand()
            .setStart(() -> dToGate = false);

    static Command shootFalse = new LambdaCommand()
            .setStart(() -> shooting = false);

    public boolean lift;

    public boolean decrease = false;

    static double transferpower = -1.0;

    /*public static Command opentransfer = new LambdaCommand()
            .setStart(()-> {
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.35);
            }).setIsDone(() -> true);
    public static Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                transfer2.setPosition(0.635);
            }).setIsDone(() -> true);
    static Command transferOn = new LambdaCommand()
            .setStart(()-> transfer1.setPower(transferpower))
            .setIsDone(() -> true);
    static Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0))
            .setIsDone(() -> true);


    public static void shoot(){
        if(shooting==false){
            shooting = true;
            SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.1), transferOn, new Delay(0.4), transferOff, closeTransfer, shootFalse);
            shoot.schedule();
        }
    }*/


    //private static Servo hoodServo1n;
    //private static Servo hoodServo2n;

    //private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
    //private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);
    public Command Localize() {
        return localize;
    }

    Command shooterer = new LambdaCommand()
            .setStart(() -> shoot());
    private static Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(1);
                transfer.setPower(1);
            });


    private static Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> {
                intakeMotor.setPower(0);
                transfer.setPower(0);
            });

    public boolean autoshoot = false;


    /*public boolean wraptofalseexecuted = false;
    public Command wrapfalse() {wrapping = false; wraptofalseexecuted=false; return null;}
    public void wrapperforwrap  (){
            SequentialGroup wraptofalse = new SequentialGroup(new Delay(0.3),wrapfalse());
            wraptofalse.schedule();
    }*/
    public static void shoot() {
        if (shooting == false) {
            shooting = true;
            SequentialGroup shoot = new SequentialGroup(openStopper, intakeMotorOn, new Delay(0.3), intakeMotorOff, closeStopper, shootFalse);
            shoot.schedule();
        }
    }

    public static void shootreal(){
        //shoot = true;
    }
    private double targetTurretAngle;
    private double avgLoopTime = 0;


    @Override
    public void periodic() {

        long currentTime = System.nanoTime();
        if (lastLoopTime != 0) {
            loopTimeMs = (currentTime - lastLoopTime) / 1_000_000.0;
            avgLoopTime +=loopTimeMs;
        }
        lastLoopTime = currentTime;


        if (firsttime == true) {
            // Schedule the command stored in the localize variable
            Gamepads.gamepad1().rightBumper().whenBecomesTrue(()->openStopper.schedule())
                    .whenBecomesFalse(()->closeStopper.schedule());
            // Correct binding: Runs the path once per press, and automatically clears out when done
            Gamepads.gamepad1().dpadUp().whenBecomesTrue(getDriveToGateCommand());
            //Gamepads.gamepad1().leftBumper().whenBecomesTrue(toggleAutoShoot);
            Gamepads.gamepad1().rightTrigger().greaterThan(0.3).whenBecomesTrue(shooterer);
            //Gamepads.gamepad1().square().whenBecomesTrue(() -> farAngle());
            firsttime = false;


            /*ParallelGroup HoodPowerZero=new ParallelGroup(
                    new SetPosition(hoodServo1,0),
                    new SetPosition(hoodServo2,0)
            );
            HoodPowerZero.schedule();*/
        }
        //follower.update();

        // Limelight + Kalman Filter periodic update (commented out until mount is ready)
        // if (limelightEnabled) {
        //     limelight.periodic();
        //     Pose odomPose = follower.getPose();
        //     kalmanFilter.predict(odomPose);
        //
        //     if (limelight.canSeeTarget()) {
        //         double dist = limelight.getDistanceInches();
        //         Pose visionPose = limelight.getPose(odomPose.getHeading());
        //         Pose visionPose = limelight.getPose(odomPose.getHeading());
        //         if (visionPose != null) {
        //             kalmanFilter.correct(visionPose, dist);
        //             follower.setPose(kalmanFilter.getFusedPose(odomPose.getHeading()));
        //             ActiveOpMode.telemetry().addData("Limelight Dist (in)", dist);
        //         }
        //     }
        // }

        Pose currPose = follower.getPose();
        Vector velocity = follower.getVelocity();


        Pose turretPose = getTurretPose(currPose);
        double robotHeading = currPose.getHeading();
        Vector robotToGoalVector = getTurretToGoalVector(turretPose);

        Double[] results = calculateShotVectorandUpdateHeading(
                robotHeading,
                robotToGoalVector,
                velocity,
                1.5

        );
        Double headingError = results[2];
        double flywheelSpeed = results[0];
        shooter((float) flywheelSpeed);
        currentMotorSpeed = flywheelSpeed;
        double hoodAngle = results[1];
        hoodServo.setPosition(hoodAngle);
        double robotAngularVelocityRads = follower.getAngularVelocity();
        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
        double feedforwardOffset = robotAngularVelocityDegs * 0.2;
        if(alliance == -1) {
            targetTurretAngle = getClosestValidTurretAngle(headingError + turretOffset - feedforwardOffset);
        }
        else{
            targetTurretAngle = getClosestValidTurretAngle(headingError + turretOffset2 - feedforwardOffset);
        }

        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

        turret1.setPosition(servoPositionSignal + servoOffset);
        turret2.setPosition(servoPositionSignal - servoOffset);
        lastServoPos = servoPositionSignal;


        currentTurretPos = targetTurretAngle;

            ActiveOpMode.telemetry().addData("hoodAngle", hoodAngle);
            ActiveOpMode.telemetry().addData("ballVelocity", flywheelSpeed);
            ActiveOpMode.telemetry().addData("flywheelSpeed", requiredTPS);
            ActiveOpMode.telemetry().addData("launch?", isOverlappingLaunchZone(currPose));
            ActiveOpMode.telemetry().addData("turret", servoPositionSignal);
            ActiveOpMode.telemetry().addData("Loop Time (ms)", loopTimeMs);
            ActiveOpMode.telemetry().addData("Avg Loop Time", avgLoopTime);
            ActiveOpMode.telemetry().addData("alliance", alliance);
            ActiveOpMode.telemetry().addData("goalX", goalX);
            ActiveOpMode.telemetry().addData("goalY", goalY);
            ActiveOpMode.telemetry().addData("RobotX", currPose.getX());
            ActiveOpMode.telemetry().addData("RobotY", currPose.getY());
            ActiveOpMode.telemetry().addData("Robot Heading: ", follower.getHeading());
            ActiveOpMode.telemetry().addData("Turret Offset", turretOffset);

            ActiveOpMode.telemetry().update();










    }


    // Fixed constructor name to match class name exactly
    public Command getDriveToGateCommand() {
        return new LambdaCommand()
                .setStart(() -> {
                    // 1. Build the path dynamically from your current position
                    PathChain dGatePath = follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            follower.getPose(),
                                            new Pose(115.158, 61.289),
                                            new Pose(133, 55.5)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(32))
                            .setVelocityConstraint(1.0)
                            .setTValueConstraint(0.8)

                            // 2. FIXED: Loosen the arrival tolerances directly on this specific path segment
                            // This allows Pedro to mark the path complete without infinitely trying to correct micro-inches
                            .setTranslationalConstraint(0.5) // Allow 1.5 inches of margin at destination
                            .setHeadingConstraint(Math.toRadians(2.0)) // Allow 3 degrees of heading margin
                            .build();

                    // 3. Command Pedro to run the path
                    follower.followPath(dGatePath, false);
                })
                // 4. End the command when Pedro stops driving
                .setIsDone(() -> !follower.isBusy())
                .setStop((Boolean interrupted) -> {
                    follower.breakFollowing(); // Safely hand control back to the joysticks
                })
                .requires(this); // Lock out your TeleOp joysticks while this is executing
    }
}