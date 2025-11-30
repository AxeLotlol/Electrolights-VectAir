package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
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
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

import java.time.*;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Flywheel")
public class testflywheel extends NextFTCOpMode {

    public testflywheel() {
        addComponents(
                new SubsystemComponent(TempHood.INSTANCE, Flywheel.INSTANCE/*Intake.INSTANCE, Spindexer.INSTANCE*/),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }

    public static MotorEx flywheel = new MotorEx("launchingmotor");



    public boolean lift;

    public boolean running=false;

    public void hood(){
        if(lift==false){
            lift=true;
            TempHood.INSTANCE.HoodUp.schedule();
        }
        else if (lift==true) {
            lift=false;
            TempHood.INSTANCE.HoodDown.schedule();
        }
        else if (lift!=true&&lift!=false) {
            lift=true;
            TempHood.INSTANCE.HoodUp.schedule();
        }

    }



    @Override
    public void onInit(){

        }
    @Override
    public void onStartButtonPressed() {
        flywheel.setPower(0.1);
        Gamepads.gamepad1().triangle().whenBecomesTrue(() -> hood());
        Gamepads.gamepad1().circle().whenBecomesTrue(() -> TempHood.INSTANCE.HoodPowerZero.schedule());
    }
    @Override
    public void onUpdate(){
        float newtps=findTPS(2);
        shooter(newtps);
        double ticksPerSecond = flywheel.getVelocity();

        double rpm = (ticksPerSecond / 28) * 60.0;

        ActiveOpMode.telemetry().addData("Motor RPM", rpm);
        ActiveOpMode.telemetry().update();

        /*if (lift == true) {
            if (running == false) {
                running = true;
                TempHood.INSTANCE.HoodUp.schedule();
                ActiveOpMode.telemetry().addLine("power set up");
                running = false;

            }
        }
        else if (lift == false) {
            if (running == false) {
                running = true;
                TempHood.INSTANCE.HoodDown.schedule();
                ActiveOpMode.telemetry().addLine("power set down");
                running = false;
            }
        }*/

    }
}

