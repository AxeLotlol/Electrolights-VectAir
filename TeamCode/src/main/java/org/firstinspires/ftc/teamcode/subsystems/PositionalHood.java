package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;


@Configurable
public class PositionalHood implements Subsystem {



    public static final PositionalHood INSTANCE = new PositionalHood();
    private PositionalHood() { }

    private static Servo hoodServo1n;
    private static Servo hoodServo2n;

    private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
    private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);






    public static ParallelGroup HoodPowerZero=new ParallelGroup(
            new SetPosition(hoodServo1,0),
            new SetPosition(hoodServo2,0)
    );

    /*public SequentialGroup HoodUpMidRange=new SequentialGroup(
            HoodRunUp,
            new Delay(0.05),
            HoodPowerZero
    );*/

    public static ParallelGroup HoodRunDown=new ParallelGroup(
            new SetPosition(hoodServo1,1),
            new SetPosition(hoodServo2,-1)
    );

    public static double hoodToPos(double runtime) {
        if(Double.isNaN(runtime)!=true) {
            ActiveOpMode.telemetry().addData("runtime", runtime);
            ParallelGroup HoodRunUp = new ParallelGroup(
                    new SetPosition(hoodServo1, -1 * runtime),
                    new SetPosition(hoodServo2, runtime)
            );
            HoodRunUp.schedule();
            return runtime;
        }
        else {
            ActiveOpMode.telemetry().addLine("NaN");
            return 0;
        }
    }




    @Override
    public void initialize() {

        hoodServo1n= ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
        hoodServo2n=  ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");


    }

    @Override
    public void periodic() {

    }
}
