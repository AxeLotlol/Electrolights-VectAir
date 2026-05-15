package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "FlywheelTuning")
public class FlywheelTuning extends NextFTCOpMode {

    public FlywheelTuning() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }
    public static MotorEx flywheel = new MotorEx("launchingmotor");


    @Override
    public void onInit() {



    }

    @Override
    public void onUpdate() {
        float configvelocity=findTPS(2);
        shooter(configvelocity);
        double ticksPerSecond = flywheel.getVelocity();
        double rpm = (ticksPerSecond / 28) * 60.0;
        double goal = (configvelocity / 28) * 60.0;
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Motor RPM", rpm);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Required RPM", goal);
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);

    }



    @Override
    public void onStartButtonPressed() {



    }
}