package org.firstinspires.ftc.teamcode.subsystems;


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

    public static double flywheelvelocity2;
    private static ControlSystem controller1;
    private static ControlSystem controller2;
    private static double controllerKP = Double.NaN;
    private static double controllerKS = Double.NaN;
    private static double lastFlywheelPower = Double.NaN;
    private static double lastFlywheel2Power = Double.NaN;
    private static final double POWER_EPSILON = 0.005;

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    public static double kS = 0.07;
    public static double kF = 0.000449;
    public static double kP = 0.009;

    public static double configvelocity = 1400; //far zone - ~1500. near zone - ~1200-1300

    private static ControlSystem buildController() {
        return ControlSystem.builder()
                .velPid(new PIDCoefficients(kP, 0, 0))
                .basicFF(new BasicFeedforwardParameters(0, 0, kS))
                .build();
    }

    private static void updateControllersIfNeeded() {
        if (controller1 == null || controller2 == null || controllerKP != kP || controllerKS != kS) {
            controller1 = buildController();
            controller2 = buildController();
            controllerKP = kP;
            controllerKS = kS;
        }
    }

    private static void setFlywheelPower(double power) {
        if (Double.isNaN(lastFlywheelPower) || Math.abs(power - lastFlywheelPower) > POWER_EPSILON) {
            flywheel.setPower(power);
            lastFlywheelPower = power;
        }
    }

    private static void setFlywheel2Power(double power) {
        if (Double.isNaN(lastFlywheel2Power) || Math.abs(power - lastFlywheel2Power) > POWER_EPSILON) {
            flywheel2.setPower(power);
            lastFlywheel2Power = power;
        }
    }

    public static void resetPowerCache() {
        lastFlywheelPower = Double.NaN;
        lastFlywheel2Power = Double.NaN;
    }

    public static void velocityControlWithFeedforwardExample(KineticState currentstate, float configtps) {
        updateControllersIfNeeded();
        controller1.setGoal(new KineticState(0.0, configtps, 0.0));
        setFlywheelPower(controller1.calculate(currentstate) + kF * configtps);
    }

    public static void velocityControlWithFeedforwardExample2(KineticState currentstate, float configtps) {
        updateControllersIfNeeded();
        controller2.setGoal(new KineticState(0.0, configtps, 0.0));
        setFlywheel2Power(-1 * (controller2.calculate(currentstate) + kF * configtps));
    }

    public static void shooter(float tps) {
        BindingManager.update();
        shooterWithoutBindingUpdate(tps);
    }

    public static void shooterWithoutBindingUpdate(float tps) {
        flywheelvelocity = flywheel.getVelocity();
        flywheelvelocity2 = flywheel2.getVelocity();
        KineticState currentState = new KineticState(0, flywheelvelocity, 0.0);
        KineticState currentState2 = new KineticState(0, -1*flywheelvelocity2, 0.0);
        //if(tps-(-1*flywheelvelocity)<7 && tps-(-1*flywheelvelocity)>-7){
        velocityControlWithFeedforwardExample(currentState, tps);
        velocityControlWithFeedforwardExample2(currentState2, tps);

    }
    @Override public void initialize() {

    }

    @Override public void periodic() {

    }
}
