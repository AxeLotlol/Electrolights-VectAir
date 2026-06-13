package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue2.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed2.isRed;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchDetector.isOverlappingLaunchZone;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
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

    private static ServoEx turret1;
    private static ServoEx turret2;

    public static double turretOffset = -3;
    public static double turretOffsetStep = -5;

    public static double openStopperPos = 0.5;
    public static double closeStopperPos = 0.42;
    public Command driveToGate = new LambdaCommand()
            .setStart(() -> dToGate = true);
    public static boolean dToGate = false;
    public Pose currPose;


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


    //Pose startingpose = Storage.currentPose;
    Pose startingpose = new Pose(72, 72, Math.toRadians(90));

    //Pose startingpose = new Pose(72,72, Math.toRadians(90));
    @Override
    public Command getDefaultCommand() {

        if (isBlue() != true && isRed() != true) {
            ActiveOpMode.telemetry().addLine("No direction set");
        } else {
            if (isBlue() == true) {
                alliance = 1;
            }
            if (isRed() == true) {
                alliance = -1;
            }
        }
        follower.update();
        currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();
        Vector robotToGoalVector = new Vector(follower.getPose().distanceFrom(new Pose(goalX, goalY)), Math.atan2(goalY - currPose.getY(), goalX - currPose.getX()));
        //Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity());
        Angle e = Angle.fromRad(follower.getHeading() - Math.PI / 2);
        {
            return new MecanumDriverControlled(
                    fL,
                    fR,
                    bL,
                    bR,
                    Gamepads.gamepad1().leftStickX().map(it -> alliance * it),
                    Gamepads.gamepad1().leftStickY().map(it -> alliance * it),
                    Gamepads.gamepad1().rightStickX().map(it -> it),
                    new FieldCentric(() -> e)
            );
        }

        //return null;
    }

    public static MotorEx intakeMotor;
    public static MotorEx transfer;
    public Command localize;

    public static ServoEx hoodServo = new ServoEx("hoodServo");

    public static Command turretzero = new LambdaCommand()
            .setStart(() -> {
                //`5transfer2.setPosition(-0.25);
                turret1.setPosition(0);
                turret2.setPosition(0);
            }).setIsDone(() -> true);
    public static Command turrethalf = new LambdaCommand()
            .setStart(() -> {
                //`5transfer2.setPosition(-0.25);
                turret1.setPosition(0.5);
                turret2.setPosition(0.5);
            }).setIsDone(() -> true);


    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE = 224.75;

    // Tracks where the turret is across frames (Double, initialized to center)
    private double currentTurretPos = 180.0;
    public boolean wrapping = false;
    public static ServoEx stopperServo = new ServoEx("stopperServo");

    public double getClosestValidTurretAngle(double relativeGoalDegrees) {
        // Option 1: The raw 0-360 input from your vector calculation
        double option1 = relativeGoalDegrees;

        // Option 2: The 360-degree alternative wrap position
        double option2 = (option1 > 180.0) ? (option1 - 360.0) : (option1 + 360.0);

        boolean opt1Valid = (option1 >= MIN_ANGLE && option1 <= MAX_ANGLE);
        boolean opt2Valid = (option2 >= MIN_ANGLE && option2 <= MAX_ANGLE);

        // If both options are mechanically safe, pick the one closest to current position
        if (opt1Valid && opt2Valid) {
            return (Math.abs(option1 - currentTurretPos) < Math.abs(option2 - currentTurretPos)) ? option1 : option2;
        }

        if (opt1Valid) {
            return option1;
        }
        if (opt2Valid) {
            wrapping = true;
            return option2;
        }

        // Safety fallback clamp
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, option1));
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
        follower = follower();
        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");
        closeStopper.schedule();
        if (isBlue() != true && isRed() != true) {
            ActiveOpMode.telemetry().addLine("No direction set");
        } else {
            if (isBlue() == true) {
                alliance = 1;
            }
            if (isRed() == true) {
                alliance = -1;
            }
        }
        imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();
        //startingpose = Storage.currentPose;
        //if(Storage.currentPose!=new Pose(0, 0, 0)) {
        follower.setStartingPose(startingpose);
        //}

        if (alliance == -1) {
            localize = new LambdaCommand()
                    .setStart(() -> follower.setPose(new Pose(129, 90, Math.toRadians(90))));

        }
        if (alliance == 1) {
            localize = new LambdaCommand()
                    .setStart(() -> follower.setPose(new Pose(15, 90, Math.toRadians(90))));

        }
        //hoodServo1n= ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
        //hoodServo2n=  ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");
        follower.update();
    }


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
        if (shooting == false) {
            shooting = true;
            SequentialGroup shoot = new SequentialGroup(openStopper, new SetPower(intakeMotor, 1), new SetPower(transfer, 1), new Delay(0.3), closeStopper, new SetPower(intakeMotor, 0), new SetPower(transfer, 0), shootFalse);
            shoot.schedule();
        }
    }

    @Override
    public void periodic() {
        if (firsttime == true) {
            // Schedule the command stored in the localize variable
            Gamepads.gamepad1().x().whenBecomesTrue((() -> Localize().schedule()));
            Gamepads.gamepad1().rightTrigger().greaterThan(0.3).whenBecomesTrue(shooter);
            //Gamepads.gamepad1().square().whenBecomesTrue(() -> farAngle());
            Gamepads.gamepad1().rightBumper().whenBecomesTrue(turretzero);
            Gamepads.gamepad1().leftBumper().whenBecomesTrue(turrethalf);
            MotorEx intakeMotor = new MotorEx("intakeMotor");
            transfer1 = new MotorEx("transferMotor");
            //transfer2 = new ServoEx("transferServo1");
            firsttime = false;
            turret1 = new ServoEx("turretServo1");
            turret2 = new ServoEx("turretServo2");

            /*ParallelGroup HoodPowerZero=new ParallelGroup(
                    new SetPosition(hoodServo1,0),
                    new SetPosition(hoodServo2,0)
            );
            HoodPowerZero.schedule();*/
        }
        follower.update();


        if (isBlue() == true) {
            goalXDist = 0;
            goalX = 0;
            localizeX = 136;
        }
        if (isRed() == true) {
            goalXDist = 144;
            goalX = 144;
            localizeX = 8;
        }
        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();
        Vector robotToGoalVector = new Vector(follower.getPose().distanceFrom(new Pose(goalX, goalY)), Math.atan2(goalY - currPose.getY(), goalX - currPose.getX()));
        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity(), follower.getAcceleration());
        Double headingError = results[2];
        double flywheelSpeed = results[0];
        shooter((float) flywheelSpeed);
        double hoodAngle = results[1];
        hoodServo.setPosition(hoodAngle);
        double robotAngularVelocityRads = follower.getAngularVelocity();
        double robotAngularVelocityDegs = Math.toDegrees(robotAngularVelocityRads);

        // Calculate the feedforward offset (Velocity * Time)
        double feedforwardOffset = robotAngularVelocityDegs * 0.225;


        //if(follower.getAngularVelocity()>5){
            double targetTurretAngle = getClosestValidTurretAngle(headingError + turretOffset - feedforwardOffset);
            double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
            servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));
            turret1.setPosition(servoPositionSignal);
            turret2.setPosition(servoPositionSignal);
            //currentTurretPos = ((servoPositionSignal - 0.05) / 0.90) * 449.51 - 44.75;
        //}
        //else{
            double targetTurretAngle2 = getClosestValidTurretAngle(headingError + turretOffset);
            double servoPositionSignal2 = 0.05 + ((targetTurretAngle2 - MIN_ANGLE) / 449.51) * 0.90;
            servoPositionSignal2 = Math.max(0.05, Math.min(0.95, servoPositionSignal2));
            //turret1.setPosition(servoPositionSignal2);
            //turret2.setPosition(servoPositionSignal2);
            currentTurretPos = ((servoPositionSignal2 - 0.05) / 0.90) * 449.51 - 44.75;
        //}

        /*if(wrapping==true && wraptofalseexecuted==false){
            wrapperforwrap();
            wraptofalseexecuted = true;
        }*/
        //currentTurretPos=targetTurretAngle;

        ActiveOpMode.telemetry().addData("launch?", isOverlappingLaunchZone(follower().getPose()));
        //if(isOverlappingLaunchZone(follower().getPose()) && robotToGoalVector.getMagnitude()>30 && wrapping == false){
        /*if(isOverlappingLaunchZone(follower().getPose()) && robotToGoalVector.getMagnitude()>30){
            intakeMotor.setPower(1);
            transfer.setPower(1);
            openStopper.schedule();
        }
        else{
            intakeMotor.setPower(0);
            transfer.setPower(0);
            closeStopper.schedule();
        }*/


        //ActiveOpMode.telemetry().addData("Motor1Speed", s1speed);
        //ActiveOpMode.telemetry().addData("Motor2Speed", s2speed);
        ActiveOpMode.telemetry().addData("far", far);
        ActiveOpMode.telemetry().addData("alliance", alliance);
        ActiveOpMode.telemetry().addData("hoodPos", hoodServo.getPosition());
        //ActiveOpMode.telemetry().addData("servo2pos", hoodServo2.getPosition());

        //double frontLeftRPM = 28 / 60 * fL.getVelocity();
        //double frontRightRPM = 28 / 60 * fR.getVelocity();
        //double backLeftRPM = 28 / 60 * bL.getVelocity();
        //double backRightRPM = 28 / 60 * bR.getVelocity();
        //ActiveOpMode.telemetry().addData("frontRightRPM", frontRightRPM);
        //ActiveOpMode.telemetry().addData("backRightRPM", backRightRPM);
        //ActiveOpMode.telemetry().addData("frontLeftRPM", frontLeftRPM);
        //ActiveOpMode.telemetry().addData("backLeftRPM", backLeftRPM);

        //ActiveOpMode.telemetry().addData("goalX", goalX);
        //ActiveOpMode.telemetry().addData("goalY", goalY);
        ActiveOpMode.telemetry().addData("RobotX", currPose.getX());
        ActiveOpMode.telemetry().addData("RobotY", currPose.getY());
        //ActiveOpMode.telemetry().addData("goalXDist", goalXDist);
        //ActiveOpMode.telemetry().addData("goalYDist", goalYDist);
        //ActiveOpMode.telemetry().addData("robotHeading", Math.toDegrees(robotHeading));
        ActiveOpMode.telemetry().addData("velocity", follower.getVelocity());
        //ActiveOpMode.telemetry().addData("headingError", headingError);
        //ActiveOpMode.telemetry().addData("distance", distance);
        //ActiveOpMode.telemetry().addData("yVCtx", visionYawCommand(headingError));

        ActiveOpMode.telemetry().addLine("==== BACKUP CONSTANTS ====");
        ActiveOpMode.telemetry().addData("Turret Offset", turretOffset);
        ActiveOpMode.telemetry().addData("RPM Vertical Shift", ShooterCalc.verticalShift);
        ActiveOpMode.telemetry().update();
        Gamepads.gamepad1().rightTrigger().greaterThan(0.3).whenBecomesTrue(shooter);
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(getDriveToGateCommand())
                .whenBecomesFalse(dToGateFalse);






        if (firsttime == true) {
            // Correct binding: Runs the path once per press, and automatically clears out when done
            Gamepads.gamepad1().rightStickButton().whenBecomesTrue(getDriveToGateCommand());

            firsttime = false; // Prevents re-binding buttons every loop
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