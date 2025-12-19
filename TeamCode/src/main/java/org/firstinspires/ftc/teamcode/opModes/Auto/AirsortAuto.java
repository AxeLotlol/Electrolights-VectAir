

package org.firstinspires.ftc.teamcode.opModes.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPositions;
import dev.nextftc.hardware.powerable.SetPower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;


@Autonomous(name = "Airsort Auto", group = "Autonomous")
@Configurable
public class AirsortAuto extends NextFTCOpMode {
    public AirsortAuto(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, TempHood.INSTANCE, DistanceRed.INSTANCE),
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
                new Delay(1.5),
                new SetPositions(transferServo1.to(-0.25)),
                new SetPower(transfer, 1),
                new Delay(0.6),
                TempHood.INSTANCE.HoodDown,
                new Delay(1.0),
                TempHood.INSTANCE.HoodUp,
                new Delay(1.0),
                new SetPower(transfer, 0),
                new SetPositions(transferServo1.to(0.6))
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