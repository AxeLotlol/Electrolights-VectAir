package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed.isRed;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;


@Configurable
public class DriveTrain2 implements Subsystem {

    public static final DriveTrain2 INSTANCE = new DriveTrain2();
    public DriveTrain2() {
    }

    public static final MotorEx fL = new MotorEx("frontLeft").brakeMode();
    public static final MotorEx fR = new MotorEx("frontRight").brakeMode().reversed();
    public static final MotorEx bL = new MotorEx("backLeft").brakeMode();
    public static final MotorEx bR = new MotorEx("backRight").brakeMode().reversed();


    public boolean firsttime = true;
    private IMUEx imu;
    public int alliance;


    @Override
    public Command getDefaultCommand() {
        return new MecanumDriverControlled(
                fL, fR, bL, bR,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()

        );
    }




    @Override
    public void initialize() {
        imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();
        if(isBlue()!=true && isRed()!=true) {
            ActiveOpMode.telemetry().addLine("No direction set");
        }
        else{
            if(isBlue()==true) {
                alliance=1;
            }
            if(isRed()==true){
                alliance=-1;
            }}

        firsttime = true;
    }



    @Override
    public void periodic() {
        if (firsttime == true) {
            firsttime = false;
        }

    }
}