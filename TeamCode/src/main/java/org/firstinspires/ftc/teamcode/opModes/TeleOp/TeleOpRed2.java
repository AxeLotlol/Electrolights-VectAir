package org.firstinspires.ftc.teamcode.opModes.TeleOp;


import org.firstinspires.ftc.teamcode.subsystems.DriveTrain2;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpRed2")
public class TeleOpRed2 extends NextFTCOpMode {
    public TeleOpRed2() {
        addComponents(
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