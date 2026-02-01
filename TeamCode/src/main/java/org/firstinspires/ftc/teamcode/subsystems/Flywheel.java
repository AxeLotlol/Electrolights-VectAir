package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable // make the class configurable
public class Flywheel implements Subsystem {

    public static final Flywheel INSTANCE = new Flywheel();

    // Motors
    public static MotorEx flywheel1 = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    // PID + Feedforward
    public static PIDCoefficients myPidCoeff = new PIDCoefficients(0.15, 0.005, 0.0);
    public static BasicFeedforwardParameters myFF = new BasicFeedforwardParameters(0.0067, 0.0, 0.01);

    // ===== BYLazar Panels (must be public static double) =====
    public static double motor1RPM = 0;
    public static double motor2RPM = 0;
    public static double motor1Power = 0;
    public static double motor2Power = 0;
    public static double targetTPSGraph = 1400; // your TPS target

    public static double configTPS = 1400; // default target

    private Flywheel() {}

    // ===== Motor 1 Control =====
    public static void controlMotor1(KineticState currentState, double targetTPS) {
        ControlSystem controller = ControlSystem.builder()
                .velPid(myPidCoeff)
                .basicFF(myFF)
                .build();
        controller.setGoal(new KineticState(0.0, targetTPS, 0.0));
        double power = controller.calculate(currentState);
        power = Math.max(-1.0, Math.min(1.0, power));
        flywheel1.setPower(power);
        motor1Power = power;
    }

    // ===== Motor 2 Control =====
    public static void controlMotor2(KineticState currentState, double targetTPS) {
        ControlSystem controller = ControlSystem.builder()
                .velPid(myPidCoeff)
                .basicFF(myFF)
                .build();
        controller.setGoal(new KineticState(0.0, targetTPS, 0.0));
        double power = controller.calculate(currentState);
        power = Math.max(-1.0, Math.min(1.0, power));
        flywheel2.setPower(-power); // inverted
        motor2Power = power;
    }

    // ===== Shooter method =====
    public static void shooter(double targetTPS) {
        BindingManager.update();

        double velocity1 = flywheel1.getVelocity();
        double velocity2 = flywheel2.getVelocity();

        KineticState state1 = new KineticState(0.0, velocity1, 0.0);
        KineticState state2 = new KineticState(0.0, -velocity2, 0.0);

        controlMotor1(state1, targetTPS);
        controlMotor2(state2, targetTPS);

        motor1RPM = (velocity1 / 28.0) * 60.0;
        motor2RPM = (velocity2 / 28.0) * 60.0;
        targetTPSGraph = targetTPS;
    }

    @Override
    public void initialize() {
        flywheel1.setPower(0);
        flywheel2.setPower(0);

        motor1RPM = 0;
        motor2RPM = 0;
        motor1Power = 0;
        motor2Power = 0;
        targetTPSGraph = configTPS;
    }

    @Override
    public void periodic() {}
}
