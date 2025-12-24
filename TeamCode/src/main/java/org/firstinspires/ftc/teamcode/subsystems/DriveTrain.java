package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.DEGREES;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
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
    public static boolean spinstop = false;
    private double tx;
    private boolean hasTag;

    private boolean autolock = false;

    private boolean SWM = false;

    private boolean slow = false;
    // === AprilTag/Limelight align tuning ===
    private static final int APRILTAG_PIPELINE = 8;   // <-- set to your AprilTag pipeline index
    private static final double YAW_KP = 0.050;      // deg -> yaw power (flip sign if turning wrong way)
    private static final double YAW_MAX = 0.7;        // yaw cap
    private static final double YAW_DEADBAND_DEG = 1.0;

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }


    double feedTime = 1.0; //adjust soon please

    GoBildaPinpointDriver pinpoint;

    private double visionYawCommand(double txDeg) {
        if (Math.abs(txDeg) < YAW_DEADBAND_DEG) return 0.0;
        return 0.55*clip(YAW_KP * txDeg, -YAW_MAX, YAW_MAX);
    }

    private void autolocktrue(){
        autolock = true;
    }

    private void autolockfalse(){
        autolock = false;
    }

    private void SWMtrue(){
        SWM = true;
    }

    private void SWMfalse(){
        SWM = false;
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

    public static double sensistivity = 1;

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
                Vector vP = vectorProjected.times(0.4);
                Pose virtualGoal = new Pose(goalX-vP.getXComponent(), goalY-vP.getYComponent());
                double targetHeading = Math.atan2(virtualGoal.getY() - currPose.getY(), virtualGoal.getX() - currPose.getX());
                double robotHeading = follower.getPose().getHeading();
                double headingError = Math.toDegrees(targetHeading) - robotHeading;
                yVCtx = () -> visionYawCommand(headingError);
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
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        Pose startingpose=new Pose (72, 72, 90);
        follower = PedroComponent.follower();
        follower.setStartingPose(startingpose);
        follower.update();

    }

    public double anglechange;
    private MotorEx intakeMotor;
    private static MotorEx transfer1;
    private static ServoEx transfer2;
    private ServoEx transfer3;
    public static Command opentransfer = new LambdaCommand()
            .setStart(()-> {
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.25);
            });
    public static Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                //transfer2.setPosition(1);
                transfer2.setPosition(1);
            });
    static Command transferOn = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-0.9));
    static Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));

    double goalY = 137.64;
    double goalX = 13.63;

    double shotTime = 0.4;

    public static SequentialGroup shoot = new SequentialGroup(opentransfer, transferOn, new Delay(1.5), transferOff, closeTransfer);

    @Override
    public void periodic() {
        if (firsttime==true){
            Gamepads.gamepad1().triangle().whenBecomesTrue(() -> autolocktrue())
                    .whenBecomesFalse(() -> autolockfalse());
            Gamepads.gamepad1().circle().whenBecomesTrue(() -> SWMtrue())
                    .whenBecomesFalse(() -> SWMfalse());
            Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> slowtrue())
                    .whenFalse(() -> slowfalse());
            intakeMotor = new MotorEx("intake");
            transfer1 = new MotorEx("transfer");
            transfer2 = new ServoEx("transferServo1");

            transfer3 = new ServoEx("transferServo2");
            firsttime=false;


        }
        follower.update();

        ActiveOpMode.telemetry().addData("Lowangle:", lowangle);


        if(isBlue()==true) {
            goalX = 130.37;
            //shotTime = 0.3; //EDIT THIS TO BE AIRTIME CALCULATIONS
        }
        if(isRed()==true){
            goalX =  13.63;
            //shotTime = 0.3; //EDIT THIS TO BE AIRTIME CALCULATIONS
        }
        double robotVelX = follower.getVelocity().getXComponent();
        double robotVelY = follower.getVelocity().getYComponent();
        double robotX = follower.getPose().getX(); //get position of robot
        double robotY = follower.getPose().getY();

        Pose currPose = follower.getPose();
        Vector OrthogonalVector = new Vector(1, -1*follower.getVelocity().getTheta());
        Vector vectorProjected = OrthogonalVector.times((follower.getVelocity().dot(OrthogonalVector))/(OrthogonalVector.dot(OrthogonalVector)));
        Vector vP = vectorProjected.times(0.4);
        Pose virtualGoal = new Pose(goalX-vP.getXComponent(), goalY-vP.getYComponent());
        double targetHeading = Math.atan2(virtualGoal.getY() - currPose.getY(), virtualGoal.getX() - currPose.getX());
        double robotHeading = follower.getPose().getHeading();
        double headingError = Math.toDegrees(targetHeading) - robotHeading;

        yVCtx = () -> visionYawCommand(headingError);
        double distance = follower.getPose().distanceFrom(virtualGoal);
        shooter(findTPS(distance /  39.37));
        //goal = actual goal - k * projected velocity vector


        ActiveOpMode.telemetry().addData("RobotVelX", robotVelX);
        ActiveOpMode.telemetry().addData("RobotVelY", robotVelY);
        ActiveOpMode.telemetry().addData("RobotX", robotX);
        ActiveOpMode.telemetry().addData("RobotY", robotY);
        ActiveOpMode.telemetry().addData("goalX", goalX);
        ActiveOpMode.telemetry().addData("goalY", goalY);
        ActiveOpMode.telemetry().addData("virtualGoalX", virtualGoal.getX());
        ActiveOpMode.telemetry().addData("virtualGoalY", virtualGoal.getY());
        ActiveOpMode.telemetry().addData("targetHeading", targetHeading);
        ActiveOpMode.telemetry().addData("robotHeading", robotHeading);
        ActiveOpMode.telemetry().addData("headingError", headingError);
        ActiveOpMode.telemetry().addData("distance", distance);
        ActiveOpMode.telemetry().update();
    }
}
