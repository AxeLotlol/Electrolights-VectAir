package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain2;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Flywheel Test")
public class FlywheelTest extends NextFTCOpMode {
    public FlywheelTest() {
    }
    public static float flywheelspeed = 800;

    public static double servopos = 0;
    public static MotorEx flywheel = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");
    public static ServoEx hoodServo = new ServoEx("hoodServo");
    @Override
    public void onInit() {







    }

    @Override
    public void onUpdate() {
        //shooter(flywheelspeed);
        flywheel.setPower(1);
        flywheel2.setPower(-1);
        double ticksPerSecond = flywheel.getVelocity();
        double rpm = (ticksPerSecond / 28) * 60.0;
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Motor RPM", rpm);
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
        hoodServo.setPosition(servopos);

    }


    @Override
    public void onStartButtonPressed() {

    }
}
