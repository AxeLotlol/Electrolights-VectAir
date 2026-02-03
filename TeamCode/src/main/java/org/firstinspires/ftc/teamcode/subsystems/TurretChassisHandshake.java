package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.vision.ColorBlobDetector;
// Import your Limelight helper classes (e.g., Limelight3A)

public class TurretChassisHandshake {
    private DcMotor turretMotor;
    // private Limelight3A limelight; // Uncomment if you have the hardware object

    private static final double MAX_LIMIT = 320.0;
    private static final double MIN_LIMIT = -320.0;

    public TurretChassisHandshake(DcMotor motor) {
        this.turretMotor = motor;
    }

    /**
     * Checks if either camera sees an incoming robot.
     * @param c270Data Data from the base camera
     * //@param limelightTID The "Texture ID" or classification from Limelight (if using neural net)
     * OR simply check if Limelight pipeline sees a large blob
     * @return A heading offset (in radians) to "nudge" the robot away.
     * Returns 0 if safe.
     */
    public double getSafetyNudge(ColorBlobDetector.BlobData c270Data, double limelightArea, double limelightTx) {
        double nudge = 0;

        // 1. CHECK BASE CAMERA (C270)
        // This is most critical because it faces where we are driving/intaking
        if (c270Data.collisionRisk) {
            // Convert the avoidance turn (degrees) to Radians for Pedro Pathing
            nudge += Math.toRadians(c270Data.avoidanceTurn);
        }

        // 2. CHECK TURRET CAMERA (Limelight)
        // If the turret is facing forward, this acts as a second pair of eyes.
        // If the turret is facing backward, this warns us of rear-end collisions.

        // Threshold: If Limelight sees a blob taking up > 10% of view (ta > 10.0)
        if (limelightArea > 10.0) {
            // Logic: If opponent is on right (tx > 0), dodge left.
            double dodgeDir = (limelightTx > 0) ? -1 : 1;
            nudge += Math.toRadians(dodgeDir * 15); // Nudge 15 degrees away
        }

        return nudge;
    }

    public void updateTurret(double targetYaw, double currentAngle) {
        double best = findBestAngleInRange(currentAngle + targetYaw, currentAngle);
        turretMotor.setTargetPosition((int)best);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.8);
    }

    private double findBestAngleInRange(double target, double current) {
        double[] options = {target, target + 360, target - 360};
        double best = target;
        double minMove = Double.MAX_VALUE;
        for (double opt : options) {
            if (opt >= MIN_LIMIT && opt <= MAX_LIMIT) {
                if (Math.abs(opt - current) < minMove) {
                    minMove = Math.abs(opt - current);
                    best = opt;
                }
            }
        }
        return best;
    }
}