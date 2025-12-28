package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightLocalization;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class LimelightLocalizationTest extends NextFTCOpMode {

    public LimelightLocalizationTest() {
        addComponents(
                new SubsystemComponent(LimelightLocalization.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }

    @Override
    public void onInit() {

    }

    @Override
    public void onStartButtonPressed() {

    }

    @Override
    public void onUpdate() {

    }

}
