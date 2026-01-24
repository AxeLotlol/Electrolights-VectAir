package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightLocalization;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class LimelightLocalizationTest extends NextFTCOpMode {

    private double x;
    private double y;

    public LimelightLocalizationTest() {
        addComponents(
                new SubsystemComponent(LimelightLocalization.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }

    @Override
    public void onInit() {}

    @Override
    public void onStartButtonPressed() {telemetry.addLine("Running Code.");}

    @Override
    public void onUpdate() {

        x = LimelightLocalization.INSTANCE.returnX();
        y = LimelightLocalization.INSTANCE.returnY();
        telemetry.addLine("Running Loop");

        telemetry.addData("x:", x);
        telemetry.addData("y:", y);
        telemetry.update();
    }

}
