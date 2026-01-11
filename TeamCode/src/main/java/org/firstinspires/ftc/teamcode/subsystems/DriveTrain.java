package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.DEGREES;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS44;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.lowangle;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed.isRed;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;
import dev.nextftc.hardware.powerable.SetPower;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@Configurable
public class DriveTrain implements Subsystem {

    public static final DriveTrain INSTANCE = new DriveTrain();
    public DriveTrain() {
    }

    private Limelight3A limelight;

    private boolean hasTag;

    private double tx;

    private boolean autolock = false;

    public double aimMultiplier = 0.475;

    private boolean slow = false;
    // === AprilTag/Limelight align tuning ===
    private static final int APRILTAG_PIPELINE = 8;   // <-- set to your AprilTag pipeline index
    private static final double YAW_KP = 0.050;      // deg -> yaw power (flip sign if turning wrong way)
    private static final double YAW_MAX = 0.7;        // yaw cap
    private static final double YAW_DEADBAND_DEG = 1.0;

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
    public static final MotorEx fL = new MotorEx("frontLeft").brakeMode();
    public static final MotorEx fR = new MotorEx("frontRight").brakeMode();
    public static final MotorEx bL = new MotorEx("backLeft").brakeMode();
    public static final MotorEx bR = new MotorEx("backRight").brakeMode();
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");
    public static MotorEx flywheel= new MotorEx("launchingmotor");

    private IMUEx imu;

    public boolean firsttime = true;

    public int alliance;

    public Supplier<Double> yVCtx;

    @Override
    public Command getDefaultCommand() {

        if(isBlue()==true) {
            alliance=1;
        }
        if(isRed()==true){
            alliance=-1;
        }
        else if(isBlue()!=true && isRed()!=true) {
            ActiveOpMode.telemetry().addLine("No direction set");
        }



        if (autolock == true) {
            Pose currPose = follower.getPose();
            Vector OrthogonalVector = new Vector(1, -1*follower.getVelocity().getTheta());
            Vector vectorProjected = OrthogonalVector.times((follower.getVelocity().dot(OrthogonalVector))/(OrthogonalVector.dot(OrthogonalVector)));
            Vector vP = vectorProjected.times(0.6);
            Pose virtualGoal = new Pose(goalX-vP.getXComponent(), goalY-vP.getYComponent());
            double targetHeading = Math.toDegrees(Math.atan2(virtualGoal.getY() - currPose.getY(), virtualGoal.getX() - currPose.getX()));
            double robotHeading = Math.toDegrees(follower.getPose().getHeading());
            double headingError = robotHeading - targetHeading;
            aimMultiplier = 0.7;
            if(alliance==1 && DistanceBlue.INSTANCE.getDistanceFromTag() != 0) {
                headingError = DistanceBlue.getTx();
                aimMultiplier = 0.475;
            }
            if(alliance==-1 && DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
                headingError = DistanceRed.getTx();
                aimMultiplier = 0.475;
            }
            double finalHeadingError = headingError;
            yVCtx = () -> visionYawCommand(finalHeadingError);

            return new MecanumDriverControlled(
                    fL,
                    fR,
                    bL,
                    bR,
                    Gamepads.gamepad1().leftStickX().map(it -> alliance * it),
                    Gamepads.gamepad1().leftStickY().map(it -> alliance * it),
                    yVCtx,
                    new FieldCentric(imu)
            );
        }
        else// IF AUTOLOCK IS NOT ON
        {
            if (slow == true) {
                return new MecanumDriverControlled(
                        fL,
                        fR,
                        bL,
                        bR,
                        Gamepads.gamepad1().leftStickX().map(it -> alliance * it * 0.4),
                        Gamepads.gamepad1().leftStickY().map(it -> alliance * it *0.4),
                        Gamepads.gamepad1().rightStickX().map(it -> it * 0.4 * 0.75),
                        new FieldCentric(imu)
                );
            }
            else //IF SLOW IS OFF
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
    }

    @Override
    public void initialize() {
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
        Pose startingpose = new Pose (115, 66, Math.toRadians(90));
        if(alliance ==-1){
            startingpose=new Pose (115, 66, Math.toRadians(90));
        }
        if(alliance ==1){
            startingpose=new Pose (29, 66, Math.toRadians(90));
        }


        follower = follower();
        follower.setStartingPose(startingpose);
        follower.update();


    }

    public double anglechange;
    private MotorEx intakeMotor;
    private static MotorEx transfer1;
    private static ServoEx transfer2;

    double goalY = 136;
    double goalX = 132;

    double goalYDist = 130.4;
    double goalXDist = 127.6;

    double shotTime = 0.4;

    static boolean shooting = false;

    static Command shootTrue = new LambdaCommand()
            .setStart(() -> shooting=true);

    static Command shootFalse = new LambdaCommand()
            .setStart(() -> shooting=false);


    static boolean lowerangle = false;

    static boolean didFirst = false;

    //public static SequentialGroup shoot = new SequentialGroup(new SetPosition(transfer2, 0.3), new Delay(0.4), new SetPower(transfer1, -0.75), new Delay(0.75), new SetPower(transfer1, 0), new SetPosition(transfer2, 0.7));





    public boolean lift;


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
            SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.4), transferOn, new Delay(0.6), transferOff, closeTransfer, shootFalse);
            shoot.schedule();
        }
    }
    Command shooter = new LambdaCommand()
            .setStart(()-> shoot());
    @Override
    public void periodic() {
        if (firsttime == true) {
            Gamepads.gamepad1().triangle().whenBecomesTrue(() -> autolocktrue())
                    .whenBecomesFalse(() -> autolockfalse());
            Gamepads.gamepad2().cross().whenBecomesTrue(() -> hood());
            Gamepads.gamepad1().rightTrigger().greaterThan(0.3).whenBecomesTrue(shooter);
            intakeMotor = new MotorEx("intake");
            transfer1 = new MotorEx("transfer");
            transfer2 = new ServoEx("transferServo1");
            firsttime = false;


        }
        follower.update();

        ActiveOpMode.telemetry().addData("Lowangle:", lowerangle);


        if (isBlue() == true) {
            goalXDist = 16.4;
            goalX = 12;
        }
        if (isRed() == true) {
            goalXDist = 127.6;
            goalX = 132;
        }
        double robotVelX = follower.getVelocity().getXComponent();
        double robotVelY = follower.getVelocity().getYComponent();
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double headingError = 0;
        Pose currPose = follower.getPose();
        Vector OrthogonalVector = new Vector(1, -1 * follower.getVelocity().getTheta());
        Vector vectorProjected = OrthogonalVector.times((follower.getVelocity().dot(OrthogonalVector)) / (OrthogonalVector.dot(OrthogonalVector)));
        Vector vP = vectorProjected.times(0.6);
        Pose virtualGoal = new Pose(goalX - vP.getXComponent(), goalY - vP.getYComponent());
        double targetHeading = Math.toDegrees(Math.atan2(virtualGoal.getY() - currPose.getY(), virtualGoal.getX() - currPose.getX()));
        double robotHeading = Math.toDegrees(follower.getPose().getHeading());
        //if (DistanceRed.INSTANCE.getDistanceFromTag() == 0 && DistanceBlue.INSTANCE.getDistanceFromTag() == 0){
        headingError = robotHeading - targetHeading;
        aimMultiplier = 0.7;
        if(alliance==1 && DistanceBlue.INSTANCE.getDistanceFromTag() != 0) {
            headingError = DistanceBlue.getTx();
            aimMultiplier = 0.475;
        }
        if(alliance==-1 && DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
            headingError = DistanceRed.getTx();
            aimMultiplier = 0.475;
        }

        Pose virtualGoalDist = new Pose(goalXDist - vP.getXComponent(), goalYDist - vP.getYComponent());
        double finalHeadingError = headingError;
        yVCtx = () -> visionYawCommand(finalHeadingError);
        if (DistanceRed.INSTANCE.getDistanceFromTag() == 0 && DistanceBlue.INSTANCE.getDistanceFromTag() == 0) {
            distance = follower.getPose().distanceFrom(virtualGoalDist);
            if (lowerangle == true) {
                shooter(findTPS44((distance / 39.37)));
            } else if (lowerangle == false) {
                shooter(findTPS((distance / 39.37)));
            }
        }
        if (lowerangle == true) {
            transferpower = -0.67;
        } else if (lowerangle == false) {
            transferpower = -1;
        }
        double s1speed = 60 * flywheel.getVelocity()/28;
        double s2speed = 60 * flywheel2.getVelocity()/28;

        ActiveOpMode.telemetry().addData("Motor1Speed", s1speed);
        ActiveOpMode.telemetry().addData("Motor2Speed", s2speed);




        ActiveOpMode.telemetry().addData("distancered", DistanceRed.INSTANCE.getDistanceFromTag());
        ActiveOpMode.telemetry().addData("distanceblue", DistanceBlue.INSTANCE.getDistanceFromTag());
        ActiveOpMode.telemetry().addData("RobotVelX", robotVelX);
        ActiveOpMode.telemetry().addData("RobotVelY", robotVelY);
        ActiveOpMode.telemetry().addData("shooting", shooting);
        ActiveOpMode.telemetry().addData("RobotX", robotX);
        ActiveOpMode.telemetry().addData("RobotY", robotY);
        ActiveOpMode.telemetry().addData("goalX", goalX);
        ActiveOpMode.telemetry().addData("goalY", goalY);
        ActiveOpMode.telemetry().addData("goalXDist", goalXDist);
        ActiveOpMode.telemetry().addData("goalYDist", goalYDist);
        ActiveOpMode.telemetry().addData("virtualGoalX", virtualGoal.getX());
        ActiveOpMode.telemetry().addData("virtualGoalY", virtualGoal.getY());
        ActiveOpMode.telemetry().addData("targetHeading", targetHeading);
        ActiveOpMode.telemetry().addData("robotHeading", robotHeading);
        ActiveOpMode.telemetry().addData("headingError", headingError);
        ActiveOpMode.telemetry().addData("distance", distance);
        ActiveOpMode.telemetry().addData("yVCtx", visionYawCommand(headingError));
        ActiveOpMode.telemetry().update();
    }
}