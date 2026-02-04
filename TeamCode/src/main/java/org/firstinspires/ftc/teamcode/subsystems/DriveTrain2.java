package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed.isRed;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;


@Configurable
public class DriveTrain2 implements Subsystem {

    public static final DriveTrain2 INSTANCE = new DriveTrain2();
    public DriveTrain2() {
    }

    private Limelight3A limelight;

    private boolean hasTag;

    private double tx;

    private boolean autolock = false;

    public double aimMultiplier = 0.575;

    private boolean slow = false;
    // === AprilTag/Limelight align tuning ===
    private static final int APRILTAG_PIPELINE = 8;   // <-- set to your AprilTag pipeline index
    private static final double YAW_KP = 0.050;      // deg -> yaw power (flip sign if turning wrong way)
    private static final double YAW_MAX = 0.7;        // yaw cap
    private static final double YAW_DEADBAND_DEG = 1.0;

    public double currentHoodState = 0;

    public double hoodAngleDriver = 0;

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }



    private double visionYawCommand(double txDeg) {
        if (Math.abs(txDeg) < YAW_DEADBAND_DEG) return 0.0;
        return aimMultiplier*clip(YAW_KP * txDeg, -YAW_MAX, YAW_MAX);
    }

    private void autolocktrue(){
        autolock = true;
    }

    private void autolockfalse(){
        autolock = false;
    }


    private void slowtrue(){
        slow = true;
    }

    private void slowfalse(){
        slow = false;
    }
    public static final MotorEx fL = new MotorEx("frontLeft").brakeMode().reversed();
    public static final MotorEx fR = new MotorEx("frontRight").brakeMode();
    public static final MotorEx bL = new MotorEx("backLeft").brakeMode().reversed();
    public static final MotorEx bR = new MotorEx("backRight").brakeMode();
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");
    public static MotorEx flywheel= new MotorEx("launchingmotor");

    private IMUEx imu;

    public boolean firsttime = true;

    public int alliance;

    public Supplier<Double> yVCtx;

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

    @Override
    public Command getDefaultCommand() {

        if(isBlue()==true) {
            alliance=1;
        }
        if(isRed()==true){
            alliance=-1;
        }
        else if(isBlue()!=true && isRed()!=true) {
            alliance=-1;
        }

            {
                return new MecanumDriverControlled(
                        fL,
                        fR,
                        bL,
                        bR,
                        Gamepads.gamepad1().leftStickX().map(it -> alliance *it),
                        Gamepads.gamepad1().leftStickY().map(it -> alliance *it),
                        Gamepads.gamepad1().rightStickX().map(it -> it * 0.75),
                        new FieldCentric(imu)
                );
            }
        }

    @Override
    public void initialize() {

        firsttime = true;
        shooting = false;
        autolock = false;

        if(isBlue()==true) {
            alliance=1;
        }
        if(isRed()==true){
            alliance=-1;
        }
        else if(isBlue()!=true && isRed()!=true) {
            ActiveOpMode.telemetry().addLine("No direction set");
        }
        imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();
        Pose startingpose = new Pose (72, 72, Math.toRadians(90));
        if(alliance ==-1){
            startingpose=new Pose (72, 72, Math.toRadians(90));
        }
        if(alliance ==1){
            startingpose=new Pose (72, 72, Math.toRadians(90));
        }

        hoodServo1n= ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
        hoodServo2n=  ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");


    }

    public double anglechange;
    private MotorEx intakeMotor;
    private static MotorEx transfer1;
    private static ServoEx transfer2;

    double goalY = 138;
    double goalX = 138;

    double goalYDist = 138;
    double goalXDist = 138;


    static boolean shooting = false;

    static Command shootTrue = new LambdaCommand()
            .setStart(() -> shooting=true);

    static Command shootFalse = new LambdaCommand()
            .setStart(() -> shooting=false);


    static boolean lowerangle = false;

    static boolean loweranglemid = false;

    static boolean didFirst = false;

    //public static SequentialGroup shoot = new SequentialGroup(new SetPosition(transfer2, 0.3), new Delay(0.4), new SetPower(transfer1, -0.75), new Delay(0.75), new SetPower(transfer1, 0), new SetPosition(transfer2, 0.7));





    public boolean lift;

    public boolean liftmid;


    public void hood(){
        if(lift==false){
            lift=true;
            lowerangle=true;
            ActiveOpMode.telemetry().addLine("HoodUp");
        }
        else if (lift==true) {
            lift=false;
            lowerangle=false;
            ActiveOpMode.telemetry().addLine("HoodDown");
        }
        else if (lift!=true&&lift!=false) {
            lift=true;
            lowerangle=true;
            ActiveOpMode.telemetry().addLine("HoodUp");
        }

    }

    public boolean decrease = false;

    public void hoodControl(){
        if(decrease!=true&&hoodAngleDriver<1){
            hoodAngleDriver = hoodAngleDriver+0.1;
        }
        else if (decrease==true&&hoodAngleDriver<1&&hoodAngleDriver>1) {
            decrease=true;
            hoodAngleDriver = 1;
        }
        else{
            hoodAngleDriver=0;
        }

    }

    static double transferpower = -1.0;

    double distance;

    public static Command opentransfer = new LambdaCommand()
            .setStart(()-> {
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.3);
            }).setIsDone(() -> true);
    public static Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                transfer2.setPosition(0.7);
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
            SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.1), transferOn, new Delay(0.25), transferOff, closeTransfer, shootFalse);
            shoot.schedule();
        }
    }

    public boolean isInLaunchZone(double x, double y) {

        // Vertices: (-8, 144), (152, 144), (72, 64)
        // This triangle exists between y = 64 and y = 144.
        if (y >= 64 && y <= 144) {
            // As y increases from 64 to 144, the width of the triangle increases.
            // The slope of the edges is (144 - 64) / (152 - 72) = 80 / 80 = 1.
            double halfWidth = (y - 64);
            if (x >= (72 - halfWidth) && x <= (72 + halfWidth)) {
                return true;
            }
        }

        // Vertices: (72, 32), (104, 0), (40, 0)
        // This triangle exists between y = 0 and y = 32.
        if (y >= 0 && y <= 32) {
            // As y decreases from 32 to 0, the width increases.
            // The slope of the edges is (32 - 0) / (72 - 40) = 32 / 32 = 1.
            double halfWidth = (32 - y);
            if (x >= (72 - halfWidth) && x <= (72 + halfWidth)) {
                return true;
            }
        }

        return false;
    }

    private static Servo hoodServo1n;
    private static Servo hoodServo2n;

    private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
    private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);
    Command shooter = new LambdaCommand()
            .setStart(()-> shoot());
    @Override
    public void periodic() {
        if (firsttime == true) {

            Gamepads.gamepad1().triangle().whenBecomesTrue(() -> autolocktrue())
                    .whenBecomesFalse(() -> autolockfalse());
            Gamepads.gamepad1().cross().whenBecomesTrue(() -> hoodControl());
            Gamepads.gamepad1().rightTrigger().greaterThan(0.3).whenBecomesTrue(shooter);
            intakeMotor = new MotorEx("intake");
            transfer1 = new MotorEx("transfer");
            transfer2 = new ServoEx("transferServo1");
            firsttime = false;
            ParallelGroup HoodPowerZero=new ParallelGroup(
                    new SetPosition(hoodServo1,0),
                    new SetPosition(hoodServo2,0)
            );
            HoodPowerZero.schedule();


        }

        ActiveOpMode.telemetry().addData("Lowangle:", lowerangle);

        double frontLeftRPM = 28 / 60 * fL.getVelocity();
        double frontRightRPM = 28 / 60 * fR.getVelocity();
        double backLeftRPM = 28 / 60 * bL.getVelocity();
        double backRightRPM = 28 / 60 * bR.getVelocity();
        ActiveOpMode.telemetry().addData("frontRightRPM", frontRightRPM);
        ActiveOpMode.telemetry().addData("backRightRPM", backRightRPM);
        ActiveOpMode.telemetry().addData("frontLeftRPM", frontLeftRPM);
        ActiveOpMode.telemetry().addData("backLeftRPM", backLeftRPM);


        if (isBlue() == true) {
            goalXDist = 6;
            goalX = 6;
        }
        if (isRed() == true) {
            goalXDist = 138;
            goalX = 138;
        }

        ActiveOpMode.telemetry().update();
    }
}