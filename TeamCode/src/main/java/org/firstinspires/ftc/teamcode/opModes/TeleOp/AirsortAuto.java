

package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.powerable.SetPower;


import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Airsort")
@Configurable
public class AirsortAuto extends NextFTCOpMode {
    public AirsortAuto(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, DriveTrain.INSTANCE, TempHood.INSTANCE, DistanceRed.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }



    private ServoEx transferServo1;
    public MotorEx transfer;
    private ServoEx transferServo2;
    public static MotorEx flywheel = new MotorEx("launchingmotor");
    public Limelight3A limelight;








    public void onInit() {
        transferServo1 = new ServoEx("transferServo1");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();
        transfer = new MotorEx("transfer").reversed();
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();

    }

    public void onUpdate(){
        double ticksPerSecond = flywheel.getVelocity();

        float newtps;
        newtps=findTPS(DistanceRed.INSTANCE.getDistanceFromTag());
        ActiveOpMode.telemetry().addLine(String.valueOf(newtps));
        shooter(newtps);
        ActiveOpMode.telemetry().update();

        //shooter(configvelocity);
        ActiveOpMode.telemetry().addData("Required RPM", newtps);


        double rpm = (ticksPerSecond / 28) * 60.0;

        ActiveOpMode.telemetry().addData("Motor RPM", rpm);
        telemetry.update();
    }




    public void onStartButtonPressed() {
        SequentialGroup onStart= new SequentialGroup(
                new Delay(2),
                //TempHood.INSTANCE.HoodUp,
                new SetPower(transfer, 0.7),
                new Delay(0.0000),
                //TempHood.INSTANCE.HoodUp,
                //TempHood.INSTANCE.HoodUp,
                new Delay(1.0),
                //TempHood.INSTANCE.HoodDown,
                new Delay(1.0),
                new SetPower(transfer, 0)
        );
        //int tag=MotifScanning.INSTANCE.findMotif();
        onStart.schedule();
    }




    @Override
    public void onStop() {
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }

    }