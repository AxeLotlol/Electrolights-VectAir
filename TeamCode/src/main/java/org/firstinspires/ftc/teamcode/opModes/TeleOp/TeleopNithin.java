package org.firstinspires.ftc.teamcode.opModes.TeleOp;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS44;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LimelightLocalization;
import org.firstinspires.ftc.teamcode.subsystems.PositionalHood;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.Drivetrain;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@TeleOp
public class TeleopNithin extends NextFTCOpMode{

    public TeleopNithin(){
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(DistanceRed.INSTANCE, TempHood.INSTANCE, PositionalHood.INSTANCE, DriveTrain.INSTANCE, LimelightLocalization.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    @Override
    public void onInit(){



    }
}
