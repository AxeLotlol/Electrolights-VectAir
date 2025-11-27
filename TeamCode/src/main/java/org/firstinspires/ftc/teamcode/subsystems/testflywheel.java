package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Flywheelellla")
public class testflywheel extends NextFTCOpMode {

    private Servo hoodServo1;
    private Servo hoodServo2;

    public boolean lift=false;

    public void hood(){
        if(lift==false){
        hoodServo1.setPosition(-0.5);
        hoodServo2.setPosition(-0.5);
        lift=true;
        }
        if(lift==true){
            hoodServo1.setPosition(0.0);
            hoodServo2.setPosition(0.0);
            lift=false;
        }

    }




    public testflywheel() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE

        );
    }

    @Override
    public void onInit(){
        hoodServo1=ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
            hoodServo2=ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");
            hoodServo1.setPosition(0.0);
            hoodServo2.setPosition(0.0);

        }

    @Override
    public void onStartButtonPressed(){

        shooter(1500);
        Gamepads.gamepad1().x().whenBecomesTrue(()-> hood()).whenBecomesFalse(()-> hood());;




    }


}

