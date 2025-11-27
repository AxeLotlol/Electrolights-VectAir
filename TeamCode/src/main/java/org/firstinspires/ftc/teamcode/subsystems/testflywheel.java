package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Flywheelellla")
public class testflywheel extends NextFTCOpMode {

    public testflywheel() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }
    @Override
    public void onStartButtonPressed(){

        shooter(1500);



    }


}
