package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.lowangle;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed.isRed;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@Configurable
public class DriveTrain implements Subsystem {

    public static final DriveTrain INSTANCE = new DriveTrain();
    private DriveTrain() { }

    private Limelight3A limelight;
    public static boolean spinstop = false;
    private double tx;
    private boolean hasTag;

    private boolean autolock = false;

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
        return 0.55*clip(YAW_KP * txDeg, -YAW_MAX, YAW_MAX);
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
            if(alliance==1){
                limelight.pipelineSwitch(8);
            }
            if(alliance==-1){
                limelight.pipelineSwitch(7);
            }
            //limelight.pipelineSwitch(APRILTAG_PIPELINE);
            LLResult result = limelight.getLatestResult();
            hasTag = (result != null) && result.isValid() && !result.getFiducialResults().isEmpty();

            if (hasTag) {
                tx = result.getTx(); // deg
                ActiveOpMode.telemetry().addData("Tx", tx);
                ActiveOpMode.telemetry().update();
            } else {
                tx = 0.0;
            }
            yVCtx = () -> visionYawCommand(tx);
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
        else // IF AUTOLOCK IS NOT ON
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
                //if doesnt work, remove else here
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
        //limelight.pipelineSwitch(APRILTAG_PIPELINE);
        //limelight.start();



    }
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

    public static SequentialGroup shoot = new SequentialGroup(opentransfer, transferOn, new Delay(1.5), transferOff, closeTransfer);

    @Override
    public void periodic() {
        if (firsttime==true){
            Gamepads.gamepad1().triangle().whenBecomesTrue(() -> autolocktrue())
                    .whenBecomesFalse(() -> autolockfalse());
            Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> slowtrue())
                    .whenFalse(() -> slowfalse());
            intakeMotor = new MotorEx("intake");
            transfer1 = new MotorEx("transfer");
            transfer2 = new ServoEx("transferServo1");

            transfer3 = new ServoEx("transferServo2");
            firsttime=false;


        }

        ActiveOpMode.telemetry().addData("Lowangle:", lowangle);

        LLResult result = limelight.getLatestResult();
        hasTag = (result != null) && result.isValid() && !result.getFiducialResults().isEmpty();

        if (hasTag) {
            tx = result.getTx();
        } else {
            tx = 0.0;
        }
        yVCtx = () -> visionYawCommand(tx);
        //ActiveOpMode.telemetry().update();
        //IF DISTANCE DOESNT WORK ANYMORE, ADD THIS
        /*float newtps = 1200;
        if(isBlue()==true) {
            newtps=findTPS(DistanceBlue.INSTANCE.getDistanceFromTag());
        }
        else if(isRed()==true){
            newtps=findTPS(DistanceRed.INSTANCE.getDistanceFromTag());
        }
        else if(isBlue()!=true && isRed()!=true) {
            newtps=findTPS(DistanceBlue.INSTANCE.getDistanceFromTag());
        }
        ActiveOpMode.telemetry().addLine(String.valueOf(newtps));
        ActiveOpMode.telemetry().addLine(String.valueOf(hasTag));
        shooter(newtps);*/
        ActiveOpMode.telemetry().update();
    }
}
