package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS44;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpRed")
public class TeleOpRed extends NextFTCOpMode {

    public MotorEx intakeMotor;
    public MotorEx transfer;
    public TeleOpRed() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(DistanceRed.INSTANCE, TempHood.INSTANCE, DriveTrain.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE


        );
    }

    public static boolean red;
    public static boolean isRed(){
        return red;
    }

    public static int tagID;
    public static boolean findMotif = false;
    public static int ball1Color = 0; //green = 1, purple = 2
    public static int ball2Color = 0;
    public static int ball3Color = 0;

    public static int getBall1Color() {
        return ball1Color;
    }

    public static int getBall2Color() {
        return ball2Color;
    }
    public static int getBall3Color() {
        return ball3Color;
    }
    public boolean lift;
    boolean lowerangle = false;

    public void hood(){
        if(lift==false){
            lift=true;
            lowerangle=true;
            TempHood.INSTANCE.HoodUp.schedule();
            ActiveOpMode.telemetry().addLine("HoodUp");
        }
        else if (lift==true) {
            lift=false;
            lowerangle=false;
            TempHood.INSTANCE.HoodDown.schedule();
            ActiveOpMode.telemetry().addLine("HoodDown");
        }
        else if (lift!=true&&lift!=false) {
            lift=true;
            lowerangle=true;
            TempHood.INSTANCE.HoodUp.schedule();
            ActiveOpMode.telemetry().addLine("HoodUp");
        }

    }



    private static final int APRILTAG_PIPELINE = 7;
    @Override
    public void onInit() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
        red=true;
        intakeMotor = new MotorEx("intake").reversed();
        transfer = new MotorEx("transfer").reversed();
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(()-> transfer.setPower(1))
                .whenBecomesFalse(() -> transfer.setPower(0));
        Gamepads.gamepad2().leftTrigger().greaterThan(0.3).whenBecomesTrue(()->intakeMotor.setPower(-1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad2().rightTrigger().greaterThan(0.3).whenBecomesTrue(()-> transfer.setPower(-1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));




    }

    @Override
    public void onUpdate() {
        float newtps=1000;
        if(lowerangle==true){
            newtps = findTPS44(DistanceRed.INSTANCE.getDistanceFromTag());
            //ActiveOpMode.telemetry().addData("Lowerangle:", lowerangle);
        }
        else if(lowerangle==false) {
            newtps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag());
            //ActiveOpMode.telemetry().addData("Lowerangle:", lowerangle);
        }
        if (DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
            shooter(newtps);
            //ActiveOpMode.telemetry().addData("newtps", newtps);
        }
    }

    public boolean shoot;

    @Override
    public void onStartButtonPressed() {

        //Gamepads.gamepad1().rightTrigger().greaterThan(0.2).whenBecomesTrue(() -> DriveTrain.shoot.schedule());
        Gamepads.gamepad2().cross().whenBecomesTrue(() -> hood());
        /*SequentialGroup onStart= new SequentialGroup(
                new Delay(2),
                //TempHood.INSTANCE.HoodUp,
                new SetPower(transfer, 0.25),
                new Delay(0.01),
                new SetPower(transfer, 0),
                TempHood.INSTANCE.HoodUp,
                new SetPower(transfer, 1),
                new Delay(0.5),
                TempHood.INSTANCE.HoodDown,
                new SetPower(transfer, 0)
        );
        //int tag=MotifScanning.INSTANCE.findMotif();
        onStart.schedule();*/
    }


    public void onStop(){
        red=false;
    }
}