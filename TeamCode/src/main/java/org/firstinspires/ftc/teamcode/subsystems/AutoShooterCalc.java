package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.SCORE_HEIGHT;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class AutoShooterCalc implements Subsystem {

    // OPTIMIZATION: Delegates to unified ShooterCalc with shooterMode=1
    // Original implementation merged into ShooterCalc.java for code reuse and performance

    public static double requiredRPM;

    public static double requiredTPS = (28*requiredRPM)/60;
    public static double verticalShift = 0;

    public static double accelScalar = 0;

    /**
     * Autonomous shooter calculation - delegates to unified ShooterCalc
     * @param robotHeading Current robot heading
     * @param robotToGoalVector Vector from robot to goal
     * @param robotVel Current robot velocity
     * @param robotAccel Current robot acceleration (for compensation)
     * @return [requiredTPS, hoodTime, headingAngle]
     */
    public static Double[] calculateShotVectorandUpdateHeading(double robotHeading, Vector robotToGoalVector, Vector robotVel, Vector robotAccel){
        // Set auto mode and delegate to unified implementation
        ShooterCalc.shooterMode = 1;
        Double[] result = ShooterCalc.calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, robotVel, robotAccel);
        
        // Sync static fields for backward compatibility
        requiredTPS = ShooterCalc.requiredTPS;
        verticalShift = ShooterCalc.verticalShift;
        
        return result;
    }
    
    @Override
    public void initialize() {
        // No initialization needed - uses ShooterCalc
    }
    
    @Override
    public void periodic() {
        // No periodic needed - uses ShooterCalc
    }
}