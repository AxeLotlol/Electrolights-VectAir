package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;


import android.os.Build;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.ActiveOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
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
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.powerable.SetPower;

import java.time.*;


@Configurable
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

    static boolean shooting = false;

    public static float rpm = 2700;

    private static MotorEx transfer1;
    private static ServoEx transfer2;

    static Command shootFalse = new LambdaCommand()
            .setStart(() -> shooting=false);

    public static Command opentransfer = new LambdaCommand()
            .setStart(()-> {
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.4);
            });
    public static Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                transfer2.setPosition(0.7);
            });
    static Command transferOn = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-1));
    static Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));



    public static SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.5), transferOn, new Delay(0.5), transferOff, closeTransfer, shootFalse);

    public static void shoot(){
        if(shooting == false){
            shoot.schedule();
        }
    }





    @Override
    public void onInit(){
        transfer1 = new MotorEx("transfer");
        transfer2 = new ServoEx("transferServo1");
        }
    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().rightTrigger().greaterThan(0.4).whenBecomesTrue(() -> shoot());
        flywheel.setPower(0.1);
        Gamepads.gamepad1().triangle().whenBecomesTrue(() -> hood());
        Gamepads.gamepad1().circle().whenBecomesTrue(() -> TempHood.INSTANCE.HoodPowerZero.schedule());
    }
    @Override
    public void onUpdate(){
        shooter((rpm*28)/60);
        double ticksPerSecond = flywheel.getVelocity();

        double currRPM = (ticksPerSecond / 28) * 60.0;


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

