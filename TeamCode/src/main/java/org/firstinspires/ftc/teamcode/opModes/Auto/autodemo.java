

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
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



import org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
@Autonomous
public class autodemo extends NextFTCOpMode{
    public autodemo(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }
    public static MotorEx flywheel = new MotorEx("launchingmotor");

    private ServoEx transfer2;
    private ServoEx transfer3;

    Command opentransfer = new LambdaCommand()
            .setStart(()-> {
                //`5transfer2.setPosition(-0.25);
                //transfer2.setPosition(0.3);
                transfer3.setPosition(-0.7);
            });
    Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                //transfer2.setPosition(1);
                //transfer2.setPosition(1.25);
                transfer3.setPosition(0.2);
            });


    @Override
    public void onInit(){

        transfer2 = new ServoEx("transferServo1");

        transfer3 = new ServoEx("transferServo2");




    }

    public Command Auto(){
        return new SequentialGroup(
                opentransfer,
                new Delay(1)
                //closeTransfer

        );
    }

    @Override
    public void onStartButtonPressed(){
        Auto().schedule();
    }
    @Override
    public void onUpdate(){
        float newtps=findTPS(1.2);
        shooter(newtps);
    }
}
