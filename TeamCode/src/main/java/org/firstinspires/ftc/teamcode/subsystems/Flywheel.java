package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.bindings.*;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
@Configurable
public class Flywheel implements Subsystem {
    public Flywheel() {

    }

    public static final Flywheel INSTANCE = new Flywheel();
    public static double flywheelvelocity;

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static PIDCoefficients myPidCoeff = new PIDCoefficients(0.125, 0.005, 0.00);
    public static BasicFeedforwardParameters myFF = new BasicFeedforwardParameters(0.0067, 0, 0.01);

    public static double configvelocity = 1400; //far zone - ~1500. near zone - ~1200-1300

    public static void velocityControlWithFeedforwardExample(KineticState currentstate, float configtps) {
        // Create a velocity controller with PID and feedforward
        ControlSystem controller = ControlSystem.builder()
                .velPid(myPidCoeff) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(myFF) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        controller.setGoal(new KineticState(0.0, configtps, 0.0));

        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity

        double power = controller.calculate(currentstate);
        flywheel.setPower(power);
    }
    public static void shooter(float tps) {
        BindingManager.update();
        flywheelvelocity = flywheel.getVelocity();
        KineticState currentState = new KineticState(0, flywheelvelocity, 0.0);
        //if(tps-(-1*flywheelvelocity)<7 && tps-(-1*flywheelvelocity)>-7){
        velocityControlWithFeedforwardExample(currentState, tps);
        double rpm = (flywheelvelocity / 28) * 60.0;
        ActiveOpMode.telemetry().addData("Flywheel RPM", rpm);

    }
    @Override public void initialize() {

    }

    @Override public void periodic() {

    }
}

