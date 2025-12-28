package org.firstinspires.ftc.teamcode.opModes.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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

    public boolean running=false;

    private Follower follower;


    public void hood(){
        if(lift==false){
            lift=true;
            TempHood.INSTANCE.HoodUp.schedule();
            ActiveOpMode.telemetry().addLine("HoodUp");
        }
        else if (lift==true) {
            lift=false;
            TempHood.INSTANCE.HoodDown.schedule();
            ActiveOpMode.telemetry().addLine("HoodDown");
        }
        else if (lift!=true&&lift!=false) {
            lift=true;
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
        //follower = PedroComponent.follower();
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(()-> transfer.setPower(0.3))
                .whenBecomesFalse(() -> transfer.setPower(0));


    }

    @Override
    public void onUpdate() {
        /*float newtps;
        newtps=findTPS(DistanceRed.INSTANCE.getDistanceFromTag());
        ActiveOpMode.telemetry().addLine(String.valueOf(newtps));
        shooter(newtps);
        ActiveOpMode.telemetry().update();*/
    }

    public boolean shoot;

    public void shoot(){
        if(shoot==false){
            shoot=true;
            DriveTrain.opentransfer.schedule();
            shoot=false;
        }
    }

    @Override
    public void onStartButtonPressed() {

        //Gamepads.gamepad1().rightTrigger().greaterThan(0.2).whenBecomesTrue(() -> DriveTrain.shoot.schedule());
        Gamepads.gamepad2().cross().whenBecomesTrue(() -> hood());
        Gamepads.gamepad2().circle().whenBecomesTrue(() -> TempHood.INSTANCE.HoodPowerZero.schedule());
        Gamepads.gamepad1().rightTrigger().greaterThan(0.4).whenBecomesTrue(() -> shoot());
        Gamepads.gamepad2().rightTrigger().greaterThan(0.4).whenBecomesTrue(() -> DriveTrain.closeTransfer.schedule());
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