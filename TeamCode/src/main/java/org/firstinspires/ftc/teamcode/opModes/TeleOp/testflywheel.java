package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import android.os.Build;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.ftc.ActiveOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import java.time.*;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Flywheel")
public class testflywheel extends NextFTCOpMode {

    public testflywheel() {
        addComponents(
                new SubsystemComponent(TempHood.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }



    @Override
    public void onInit(){

        }
    @Override
    public void onStartButtonPressed() {
        shooter(1500);
    }
    @Override
    public void onUpdate(){

    }
}

