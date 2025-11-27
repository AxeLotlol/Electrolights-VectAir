package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@Configurable
public class TempHood implements Subsystem {

    public static final TempHood INSTANCE = new TempHood();
    private TempHood() { }

    private CRServo hoodServo1;
    private CRServo hoodServo2;

    public boolean lift=false;

    public boolean running=false;

    public void hood(){
        if(lift==false){
            lift=true;
        }
        else if(lift==true){
            lift=false;
        }

    }




    @Override
    public Command getDefaultCommand() {

        if(lift==false){
            if(running==false){
            hoodServo1.setPower(1);
            hoodServo2.setPower(-1);
            //new Delay(0.3);
            hoodServo1.setPower(0.0);
            hoodServo2.setPower(0.0);
            running=true;
            }
        }
        if(lift==true){
            if(running==false) {
                hoodServo1.setPower(-1);
                hoodServo2.setPower(1);
                //new Delay(0.3);
                hoodServo1.setPower(0.0);
                hoodServo2.setPower(0.0);
                running = true;
            }
        }
        ActiveOpMode.telemetry().addLine(String.valueOf(lift));
        ActiveOpMode.telemetry().update();


        return null;
    }

    @Override
    public void initialize() {

        hoodServo1= ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo1");
        hoodServo2= ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo2");
        Gamepads.gamepad1().triangle().whenBecomesTrue(()-> hood());

    }

    @Override
    public void periodic() {

    }
}
