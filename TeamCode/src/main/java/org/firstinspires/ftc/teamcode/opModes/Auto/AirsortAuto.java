

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;


@Autonomous(name = "Airsort Auto", group = "Autonomous")
@Configurable
public class AirsortAuto extends NextFTCOpMode {
    public AirsortAuto(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, TempHood.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }













    private MotorEx intakeMotor;

    private ServoEx transferServo1;
    private ServoEx transferServo2;

    // private MotorEx spindexerMotor;

    private boolean path2Following= false;
    int ball1Color = 0;
    int ball2Color = 0;
    int ball3Color = 0;
    int tagId = 0;
    public static MotorEx spindex = new MotorEx("spindexer");

    public static MotorEx transfer = new MotorEx("transfer");

    public static MotorEx flywheel = new MotorEx("launchingmotor");








    public void onInit() {
        telemetry.addLine("Initializing Follower...");
        transferServo1 = new ServoEx("transferServo1");
        transferServo2 = new ServoEx("transferServo2");

        telemetry.update();


        intakeMotor = new MotorEx("intake");




        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
    }

    public void onUpdate(){
        double ticksPerSecond = flywheel.getVelocity();

        float newtps=findTPS(2);
        shooter(newtps);

        //shooter(configvelocity);
        ActiveOpMode.telemetry().addData("Required RPM", newtps);


        double rpm = (ticksPerSecond / 28) * 60.0;

        ActiveOpMode.telemetry().addData("Motor RPM", rpm);
        telemetry.update();
    }


    public SequentialGroup onStart= new SequentialGroup(
            new Delay(1.0),
            new SetPositions(transferServo1.to(0.5), transferServo2.to(0.5)),
            new Delay(0.2),
            TempHood.INSTANCE.HoodUp,
            new Delay(0.1),
            TempHood.INSTANCE.HoodDown
    );

    public void onStartButtonPressed() {
        float newtps=findTPS(2);
        shooter(newtps);
        //int tag=MotifScanning.INSTANCE.findMotif();
        onStart.schedule();


    }




    @Override
    public void onStop() {
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }

    }