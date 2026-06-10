package org.firstinspires.ftc.teamcode.opModes.TeleOp;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain2;
import org.firstinspires.ftc.teamcode.subsystems.ShooterCalc;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpRed2")
public class TeleOpRed2 extends NextFTCOpMode {
    public MotorEx intakeMotor;
    public MotorEx transfer;
    public TeleOpRed2() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(DriveTrain2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE


        );
    }

    public static boolean red;


    public static boolean isRed(){
        return red;
    }

    @Override
    public void onInit() {
        red=true;
        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> transfer.setPower(1))
                .whenBecomesFalse(() -> transfer.setPower(0));
        Gamepads.gamepad1().x().whenBecomesTrue(()->follower.setPose(new Pose(79.967,9.271,Math.toRadians(90))));

        // Backup Controls
        Gamepads.gamepad2().leftTrigger().greaterThan(0.5).whenBecomesTrue(() -> DriveTrain2.turretOffset -= DriveTrain2.turretOffsetStep);
        Gamepads.gamepad2().rightTrigger().greaterThan(0.5).whenBecomesTrue(() -> DriveTrain2.turretOffset += DriveTrain2.turretOffsetStep);
        Gamepads.gamepad2().a().whenBecomesTrue(() -> DriveTrain2.turretOffset = 0);
        Gamepads.gamepad2().dpadUp().whenBecomesTrue(() -> ShooterCalc.verticalShift += ShooterCalc.verticalShiftStep);
        Gamepads.gamepad2().dpadDown().whenBecomesTrue(() -> ShooterCalc.verticalShift -= ShooterCalc.verticalShiftStep);
    }

    @Override
    public void onUpdate() {
    }


    @Override
    public void onStartButtonPressed() {

    }


    public void onStop(){
        red=false;
    }
}