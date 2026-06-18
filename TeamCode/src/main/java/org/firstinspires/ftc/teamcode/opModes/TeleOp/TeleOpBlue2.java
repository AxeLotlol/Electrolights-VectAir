package org.firstinspires.ftc.teamcode.opModes.TeleOp;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

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


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue2")
public class TeleOpBlue2 extends NextFTCOpMode {
    public MotorEx intakeMotor;
    public MotorEx transfer;
    public TeleOpBlue2() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(DriveTrain2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE


        );
    }

    public static boolean blue;


    public static boolean isBlue(){
        return blue;
    }

    @Override
    public void onInit() {
        blue=true;
        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> transfer.setPower(1))
                .whenBecomesFalse(() -> transfer.setPower(0));
        Gamepads.gamepad1().x().whenBecomesTrue(()->follower.setPose(new Pose(79.967,9.271,Math.toRadians(90))));






    }

    @Override
    public void onUpdate() {
    }


    @Override
    public void onStartButtonPressed() {

    }


    public void onStop(){
        blue=false;
    }
}