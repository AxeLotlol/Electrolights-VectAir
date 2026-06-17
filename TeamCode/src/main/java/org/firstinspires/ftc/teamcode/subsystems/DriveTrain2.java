package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue2.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed2.isRed;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooterWithoutBindingUpdate;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public boolean firsttime = true;

    public int alliance;
    public boolean far;

    private ServoImplEx turret1;
    private ServoImplEx turret2;

    public static double turretOffset = 6;
    public static double turretOffsetStep = -5;
    // Inches from the Pinpoint/Pedro robot pose origin to the turret pivot.
    public static double turretForwardOffset = -0.52588;
    public static double turretStrafeOffset = 0;

    public static double openStopperPos = 0.7;
    public static double closeStopperPos = 0.634;
    public Command driveToGate = new LambdaCommand()
            .setStart(() -> dToGate = true);
    public static boolean dToGate = false;
    // Loop time tracking
    private long lastLoopTime = 0;
    private double loopTimeMs = 0;
    public Pose currPose;
    private static final double POWER_EPSILON = 0.005;
    private static final double SERVO_EPSILON = 0.001;
    private static double lastStopperPosition = Double.NaN;
    private static final long TELEMETRY_INTERVAL_NANOS = 100_000_000L;
    private double lastIntakePower = Double.NaN;
    private double lastTransferPower = Double.NaN;
    private double lastHoodPosition = Double.NaN;
    private double lastTurretPosition = Double.NaN;
    private long lastTelemetryTime = 0;
    private double telemetryLoopTimeSumMs = 0;
    private double telemetryLoopTimeMaxMs = 0;
    private int telemetryLoopCount = 0;
    private double bulkClearMaxMs = 0;
    private double followerUpdateMaxMs = 0;
    private double shooterUpdateMaxMs = 0;
    private List<LynxModule> allHubs;
    private final ShooterCalc.ShotVectorResult shotVectorResult = new ShooterCalc.ShotVectorResult();


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

        //return null;
    }

    public static MotorEx intakeMotor;
    public static MotorEx transfer;


    public Command localize;

    public static ServoEx hoodServo = new ServoEx("hoodServo");

    public Command turretzero = new LambdaCommand()
            .setStart(() -> {
                //`5transfer2.setPosition(-0.25);
                setTurretPositionCached(0);
            }).setIsDone(() -> true);
    public Command turrethalf = new LambdaCommand()
            .setStart(() -> {
                //`5transfer2.setPosition(-0.25);
                setTurretPositionCached(0.5);
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

    private void configureAllianceTarget() {
        if (isRed()) {
            alliance = -1;
            goalXDist = 144;
            goalX = 144;
            localizeX = 8;
        } else if (isBlue()) {
            alliance = 1;
            goalXDist = 0;
            goalX = 0;
            localizeX = 136;
        } else {
            ActiveOpMode.telemetry().addLine("No direction set");
        }
    }

    public static Command closeStopper = new LambdaCommand()
            .setStart(() -> {
                setStopperPositionCached(closeStopperPos); // close
            }).setIsDone(() -> true);
    public static Command openStopper = new LambdaCommand()
            .setStart(() -> {
                setStopperPositionCached(openStopperPos); // open
            }).setIsDone(() -> true);

    private static void setStopperPositionCached(double position) {
        if (Double.isNaN(lastStopperPosition) || Math.abs(position - lastStopperPosition) > SERVO_EPSILON) {
            stopperServo.setPosition(position);
            lastStopperPosition = position;
        }
    }

    private void setIntakePowerCached(double power) {
        if (Double.isNaN(lastIntakePower) || Math.abs(power - lastIntakePower) > POWER_EPSILON) {
            intakeMotor.setPower(power);
            lastIntakePower = power;
        }
    }

    private void setTransferPowerCached(double power) {
        if (Double.isNaN(lastTransferPower) || Math.abs(power - lastTransferPower) > POWER_EPSILON) {
            transfer.setPower(power);
            lastTransferPower = power;
        }
    }

    public void invalidateIntakeTransferPowerCache() {
        lastIntakePower = Double.NaN;
        lastTransferPower = Double.NaN;
    }

    private void setHoodPositionCached(double position) {
        if (Double.isNaN(lastHoodPosition) || Math.abs(position - lastHoodPosition) > SERVO_EPSILON) {
            hoodServo.setPosition(position);
            lastHoodPosition = position;
        }
    }

    private void setTurretPositionCached(double position) {
        if (Double.isNaN(lastTurretPosition) || Math.abs(position - lastTurretPosition) > SERVO_EPSILON) {
            turret1.setPosition(position);
            turret2.setPosition(position);
            lastTurretPosition = position;
        }
    }

    @Override
    public void initialize() {

        firsttime = true;
        shooting = false;
        lastIntakePower = Double.NaN;
        lastTransferPower = Double.NaN;
        lastHoodPosition = Double.NaN;
        lastTurretPosition = Double.NaN;
        lastStopperPosition = Double.NaN;
        lastTelemetryTime = 0;
        telemetryLoopTimeSumMs = 0;
        telemetryLoopTimeMaxMs = 0;
        telemetryLoopCount = 0;
        bulkClearMaxMs = 0;
        followerUpdateMaxMs = 0;
        shooterUpdateMaxMs = 0;
        allHubs = ActiveOpMode.hardwareMap().getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        Flywheel.resetPowerCache();
        follower = PedroComponent.follower();
        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");
        closeStopper.schedule();
        configureAllianceTarget();
        imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();
        Pose startingpose = new Pose(72, 72, Math.toRadians(90));
        if(Storage.setPose){
            startingpose = Storage.currentPose;
            Storage.setPose=false;
        }
        follower.setStartingPose(startingpose);

        if (alliance == -1) {
            localize = new LambdaCommand()
                    .setStart(() -> follower.setPose(new Pose(129, 90, Math.toRadians(90))));

        }
        if (alliance == 1) {
            localize = new LambdaCommand()
                    .setStart(() -> follower.setPose(new Pose(15, 90, Math.toRadians(90))));

        }
        turret1 = ActiveOpMode.hardwareMap().get(ServoImplEx.class, "turretServo1");
        turret2 = ActiveOpMode.hardwareMap().get(ServoImplEx.class,"turretServo2");
        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(()->openStopper.schedule())
                .whenBecomesFalse(()->closeStopper.schedule());
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(toggleAutoShoot);
        Gamepads.gamepad1().rightTrigger().greaterThan(0.3).whenBecomesTrue(shooter);
        //Gamepads.gamepad1().square().whenBecomesTrue(() -> farAngle());
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(turretzero);
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(turrethalf);
        transfer1 = new MotorEx("transferMotor");
        //transfer2 = new ServoEx("transferServo1");
        firsttime = false;
        follower.update();
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

    double goalY = 144;
    double goalX = 144;

    static double localizeX;
    double goalXDist = 144;


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
    /*Command shooter = new LambdaCommand()
            .setStart(()-> shoot());*/
    public Command Localize() {
        return localize;
    }

    Command shooter = new LambdaCommand()
            .setStart(() -> shoot());


    /*public boolean wraptofalseexecuted = false;
    public Command wrapfalse() {wrapping = false; wraptofalseexecuted=false; return null;}
    public void wrapperforwrap(){
            SequentialGroup wraptofalse = new SequentialGroup(new Delay(0.3),wrapfalse());
            wraptofalse.schedule();
    }*/
    public static void shoot() {
        //if (shooting == false) {
        //    shooting = true;
        //SequentialGroup shoot = new SequentialGroup(new Delay(0.3), shootFalse);
        //    shoot.schedule();
        //}
        shooting = true;
        new Delay(0.3);
        shootFalse.schedule();
    }

    public static void shootreal(){
        //shoot = true;
    }

    @Override
    public void periodic() {
        long currentTime = System.nanoTime();
        long sectionStartTime = currentTime;
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        bulkClearMaxMs = Math.max(bulkClearMaxMs, (System.nanoTime() - sectionStartTime) / 1_000_000.0);

        if (lastLoopTime != 0) {
            loopTimeMs = (currentTime - lastLoopTime) / 1_000_000.0;
            telemetryLoopTimeSumMs += loopTimeMs;
            telemetryLoopTimeMaxMs = Math.max(telemetryLoopTimeMaxMs, loopTimeMs);
            telemetryLoopCount++;
        }
        lastLoopTime = currentTime;
        sectionStartTime = System.nanoTime();
        follower.update();
        followerUpdateMaxMs = Math.max(followerUpdateMaxMs, (System.nanoTime() - sectionStartTime) / 1_000_000.0);


        Pose currPose = follower.getPose();
        double robotHeading = currPose.getHeading();
        Vector robotVelocity = follower.getVelocity();
        double robotX = currPose.getX();
        double robotY = currPose.getY();
        double cosHeading = Math.cos(robotHeading);
        double sinHeading = Math.sin(robotHeading);
        double turretX = robotX
                + turretForwardOffset * cosHeading
                - turretStrafeOffset * sinHeading;
        double turretY = robotY
                + turretForwardOffset * sinHeading
                + turretStrafeOffset * cosHeading;
        double goalDeltaX = goalX - turretX;
        double goalDeltaY = goalY - turretY;
        double robotToGoalDistance = Math.hypot(goalDeltaX, goalDeltaY);
        double robotToGoalTheta = Math.atan2(goalDeltaY, goalDeltaX);
        boolean shouldUpdateTelemetry = currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL_NANOS;
        calculateShotVectorandUpdateHeading(robotHeading, robotToGoalDistance, robotToGoalTheta, robotVelocity, 1.7, shotVectorResult, false);
        double headingError = shotVectorResult.headingAngle;
        double flywheelSpeed = shotVectorResult.flywheelSpeed;
        sectionStartTime = System.nanoTime();
        shooterWithoutBindingUpdate((float) flywheelSpeed);
        shooterUpdateMaxMs = Math.max(shooterUpdateMaxMs, (System.nanoTime() - sectionStartTime) / 1_000_000.0);
        double hoodAngle = shotVectorResult.hoodTime;
        setHoodPositionCached(hoodAngle);
        double robotAngularVelocityRads = follower.getAngularVelocity();
        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);
        double feedforwardOffset = robotAngularVelocityDegs * 0.225;
        double targetTurretAngle = getClosestValidTurretAngle(headingError + turretOffset - feedforwardOffset);
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));
        setTurretPositionCached(servoPositionSignal);
        currentTurretPos = targetTurretAngle;

        boolean launchZone = false;
        boolean futureLaunchZone = false;
        boolean shouldFeed = shooting == true;
        if (!shouldFeed && robotToGoalDistance > 50) {
            launchZone = isOverlappingLaunchZone(robotX, robotY, robotHeading);
            if (!launchZone) {
                futureLaunchZone = isOverlappingLaunchZone(
                        robotX+robotVelocity.getXComponent()*0.3,
                        robotY+robotVelocity.getYComponent()*0.3,
                        robotHeading
                );
            }
            shouldFeed = launchZone || futureLaunchZone;
        } else if (shouldUpdateTelemetry) {
            launchZone = isOverlappingLaunchZone(robotX, robotY, robotHeading);
        }
        //if((isOverlappingLaunchZone(PedroComponent.follower().getPose())||isOverlappingLaunchZone(futurepose)) && robotToGoalVector.getMagnitude()>40){
        if(shouldFeed){
            setIntakePowerCached(1);
            setTransferPowerCached(1);
            setStopperPositionCached(openStopperPos);
        }
        else{
            setIntakePowerCached(0);
            setTransferPowerCached(0);
            setStopperPositionCached(closeStopperPos);
        }

        if (shouldUpdateTelemetry) {
            Telemetry telemetry = ActiveOpMode.telemetry();
            lastTelemetryTime = currentTime;
            telemetry.addData("hoodAngle", shotVectorResult.hoodDegrees);
            telemetry.addData("ballVelocity", shotVectorResult.ballVelocity);
            telemetry.addData("flywheelSpeed", shotVectorResult.flywheelSpeed);
            telemetry.addData("launch?", launchZone);
            telemetry.addData("turret", servoPositionSignal);
            telemetry.addData("Loop Time (ms)", loopTimeMs);
            telemetry.addData("Avg Loop Time (ms)", telemetryLoopCount == 0 ? 0 : telemetryLoopTimeSumMs / telemetryLoopCount);
            telemetry.addData("Max Loop Time (ms)", telemetryLoopTimeMaxMs);
            telemetry.addData("Max Bulk Clear (ms)", bulkClearMaxMs);
            telemetry.addData("Max Follower Update (ms)", followerUpdateMaxMs);
            telemetry.addData("Max Shooter Update (ms)", shooterUpdateMaxMs);
            //ActiveOpMode.telemetry().addData("Motor1Speed", s1speed);
            //ActiveOpMode.telemetry().addData("Motor2Speed", s2speed);
            telemetry.addData("far", far);
            telemetry.addData("alliance", alliance);
            telemetry.addData("hoodPos", hoodAngle);
            //ActiveOpMode.telemetry().addData("servo2pos", hoodServo2.getPosition());

            //double frontLeftRPM = 28 / 60 * fL.getVelocity();
            //double frontRightRPM = 28 / 60 * fR.getVelocity();
            //double backLeftRPM = 28 / 60 * bL.getVelocity();
            //double backRightRPM = 28 / 60 * bR.getVelocity();
            //ActiveOpMode.telemetry().addData("frontRightRPM", frontRightRPM);
            //ActiveOpMode.telemetry().addData("backRightRPM", backRightRPM);
            //ActiveOpMode.telemetry().addData("frontLeftRPM", frontLeftRPM);
            //ActiveOpMode.telemetry().addData("backLeftRPM", backLeftRPM);

            telemetry.addData("goalX", goalX);
            telemetry.addData("goalY", goalY);
            telemetry.addData("RobotX", robotX);
            telemetry.addData("RobotY", robotY);
            telemetry.addData("TurretX", turretX);
            telemetry.addData("TurretY", turretY);
            //ActiveOpMode.telemetry().addData("goalXDist", goalXDist);
            //ActiveOpMode.telemetry().addData("goalYDist", goalYDist);
            //ActiveOpMode.telemetry().addData("robotHeading", Math.toDegrees(robotHeading));
            telemetry.addData("velocity", robotVelocity);
            telemetry.addData("headingError", headingError);
            telemetry.addData("angularFF", feedforwardOffset);
            //ActiveOpMode.telemetry().addData("distance", distance);
            //ActiveOpMode.telemetry().addData("yVCtx", visionYawCommand(headingError));
            telemetry.addData("Robot Heading: ",robotHeading);
            telemetry.addLine("==== BACKUP CONSTANTS ====");
            telemetry.addData("Turret Offset", turretOffset);
            telemetry.addData("Turret Forward Offset", turretForwardOffset);
            telemetry.addData("Turret Strafe Offset", turretStrafeOffset);
            telemetry.addData("RPM Vertical Shift", ShooterCalc.verticalShift);
            telemetry.update();
            telemetryLoopTimeSumMs = 0;
            telemetryLoopTimeMaxMs = 0;
            telemetryLoopCount = 0;
            bulkClearMaxMs = 0;
            followerUpdateMaxMs = 0;
            shooterUpdateMaxMs = 0;
        }
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
