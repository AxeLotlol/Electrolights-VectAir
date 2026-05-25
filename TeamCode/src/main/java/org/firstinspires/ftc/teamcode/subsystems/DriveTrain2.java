package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
public class DriveTrain2 implements Subsystem {

    public static final DriveTrain2 INSTANCE = new DriveTrain2();
    public DriveTrain2() {
    }

    public static final MotorEx fL = new MotorEx("frontLeft").brakeMode();
    public static final MotorEx fR = new MotorEx("frontRight").brakeMode().reversed();
    public static final MotorEx bL = new MotorEx("backLeft").brakeMode();
    public static final MotorEx bR = new MotorEx("backRight").brakeMode().reversed();

    private IMUEx imu;

    @Override
    public Command getDefaultCommand() {
        {
            return new MecanumDriverControlled(
                    fL,
                    fR,
                    bL,
                    bR,
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().leftStickY(),
                    Gamepads.gamepad1().rightStickX(),
                    new FieldCentric(imu)
            );
        }

        //return null;
    }




    @Override
    public void initialize() {
        imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();
    }



    @Override
    public void periodic() {
    }
}