package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class newFlywheel implements Subsystem{

    public static final newFlywheel INSTANCE = new newFlywheel();

    // Motors
    public static MotorEx flywheel1 = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");
    private ControlSystem controller;


    public void onInit(){

        controller = ControlSystem.builder()
                .velPid(new PIDCoefficients(0.15, 0.005, 0.0))
                .basicFF(new BasicFeedforwardParameters(0.0067, 0.0, 0.01))
                .build();
        controller.setGoal(new KineticState(0.0, 0.0));






    }





}
