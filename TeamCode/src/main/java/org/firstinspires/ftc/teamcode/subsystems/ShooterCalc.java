package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.SCORE_HEIGHT;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.TVector;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import java.lang.Math;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable

class ShotData {
    public double flywheel;
    public double hood;
    public double heading;
}
public class ShooterCalc implements Subsystem {

    //public static double farzoneangle = -0.34006585;

    //public static double farzoneheight = 10.25;


    public static double requiredRPM;

    public static double rpmoffset = 200;
    public static double requiredTPS = (28 * requiredRPM) / 60;
    public static double verticalShift = 0;
    public static double verticalShiftStep = 50;

    public static double accelScalar = 0.015; // Set to 0 to disable

    // OPTIMIZATION: Reuse object to reduce garbage collection
    private static final ShotData reusableShotData = new ShotData();

    /**
     * Shooter calculation mode: 0 = TeleOp (default), 1 = Autonomous
     */
    public static int shooterMode = 0;

    // OPTIMIZATION: Pre-computed constants to avoid repeated Math calls in hot loop
    private static final double G = 32.174 * 12;
    private static final double METERS_TO_INCHES = 39.37;
    private static final double SCORE_ANGLE_RAD = Math.toRadians(-33);
    private static final double AUTO_SCORE_ANGLE_RAD = Math.toRadians(-23);
    private static final double CLOSE_ANGLE_RAD = Math.toRadians(-45);
    private static final double MIN_HOOD_RAD = Math.toRadians(40);
    private static final double MAX_HOOD_RAD = Math.toRadians(75);
    private static final double DEFAULT_HOOD_RAD = Math.toRadians(75);

    /**
     * Unified shooter calculation for both TeleOp and Autonomous.
     * Mode 0 (TeleOp): uses sotmFactor, SCORE_ANGLE_RAD, positive parallelComponent
     * Mode 1 (Auto): uses robotAccel, AUTO_SCORE_ANGLE_RAD, negative parallelComponent, accel compensation
     */
    public static Double[] calculateShotVectorandUpdateHeading(double robotHeading, Vector robotToGoalVector, Vector robotVel, double sotmFactor) {
        return calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, robotVel, sotmFactor, null);
    }

    public static Double[] calculateShotVectorandUpdateHeading(double robotHeading, Vector robotToGoalVector, Vector robotVel, Vector robotAccel) {
        return calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, robotVel, 0, robotAccel);
    }

    private static Double[] calculateShotVectorandUpdateHeading(double robotHeading, Vector robotToGoalVector, Vector robotVel, double sotmFactor, Vector robotAccel) {
        boolean isAuto = (shooterMode == 1) || (robotAccel != null);
        double x = robotToGoalVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double temp = x / METERS_TO_INCHES;

        double y, a;
        if (isAuto) {
            y = SCORE_HEIGHT;
            a = AUTO_SCORE_ANGLE_RAD;
        } else {
            y = SCORE_HEIGHT + 2;
            a = SCORE_ANGLE_RAD;
        }
        if (x < 50) {
            a = CLOSE_ANGLE_RAD;
            y = isAuto ? SCORE_HEIGHT + 4 : SCORE_HEIGHT + 4;
        }
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), MIN_HOOD_RAD, MAX_HOOD_RAD);

        if (isNaN(hoodAngle)) {
            hoodAngle = DEFAULT_HOOD_RAD;
        }
        // OPTIMIZATION: Cache trig results to avoid repeated Math.cos/sin/pow calls
        double cosHood = Math.cos(hoodAngle);
        double cosHoodSq = cosHood * cosHood;
        double sinHood = Math.sin(hoodAngle);
        double tanHood = sinHood / cosHood;
        double flywheelSpeed = Math.sqrt(G * x * x / (2 * cosHoodSq * (x * tanHood - y)));
        if (x < 50) {
            a = CLOSE_ANGLE_RAD;
            y = isAuto ? SCORE_HEIGHT + 4 : SCORE_HEIGHT + 2;
        }
        Vector robotVelocity = new Vector(robotVel.getMagnitude(), robotVel.getTheta());

        // Auto mode: acceleration compensation
        if (isAuto && robotAccel != null && robotAccel.getMagnitude() > 10) {
            robotVelocity.setMagnitude(robotVelocity.getMagnitude() + (robotAccel.getMagnitude() * accelScalar));
        }

        if (robotVelocity.getMagnitude() < 5.0) {
            robotVelocity = new Vector(0, 0);
        }

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        // TeleOp: positive parallelComponent, Auto: negative parallelComponent
        double parallelComponent = isAuto
                ? -Math.cos(coordinateTheta) * robotVelocity.getMagnitude()
                : Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double vz = flywheelSpeed * sinHood;
        double time = (x / (flywheelSpeed * cosHood));

        // TeleOp: ivr = x/time - parallelComponent, Auto: ivr = x/time + parallelComponent
        double ivr = isAuto ? x / time + parallelComponent : x / time - parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        // OPTIMIZATION: Reuse cached trig for second hoodAngle calculation
        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), MIN_HOOD_RAD, MAX_HOOD_RAD);

        if (isNaN(hoodAngle)) {
            hoodAngle = DEFAULT_HOOD_RAD;
        }
        double newtemp = (ndr + 5) / METERS_TO_INCHES;

        //y = /*SCORE_HEIGHT*/ -4.5745*newtemp*newtemp*newtemp + 25.978*newtemp*newtemp - 48.395*newtemp + 58.675; // -0.7135x2 + 0.8315x + 33.532
        cosHood = Math.cos(hoodAngle);
        cosHoodSq = cosHood * cosHood;
        sinHood = Math.sin(hoodAngle);
        tanHood = sinHood / cosHood;
        flywheelSpeed = Math.sqrt(G * ndr * ndr / (2 * cosHoodSq * (ndr * tanHood - y)));
        flywheelSpeed = flywheelSpeed / METERS_TO_INCHES;

        // TeleOp: sotmFactor * atan(perp/safeIvr), Auto: 1.5 * atan(perp/ivr)
        double headingVelCompOffset;
        if (isAuto) {
            headingVelCompOffset = 1.5 * Math.atan(perpendicularComponent / ivr);
        } else {
            double safeIvr = Math.max(ivr, 50.0);
            headingVelCompOffset = sotmFactor * Math.atan(perpendicularComponent / safeIvr);
        }
        double headingAngle = Math.toDegrees(robotToGoalVector.getTheta() - robotHeading - headingVelCompOffset);

        //requiredRPM = (1.4286* Math.pow(flywheelSpeed, 3) - 39.264*Math.pow(flywheelSpeed, 2) + 863.57*flywheelSpeed-1373.9 + rpmoffset);
        //requiredTPS = (28 * requiredRPM) / 60;
        //requiredTPS = 12.79084*Math.pow(flywheelSpeed, 2)-69.40057*flywheelSpeed+849.93005;

        requiredTPS = (-16.19 * Math.pow(flywheelSpeed, 2) + 449.11 * flywheelSpeed - 964.9) + verticalShift;

        // Auto mode: add 40 TPS if velocity > 20
        if (isAuto && robotVel.getMagnitude() > 20) {
            requiredTPS = requiredTPS + 40;
        }

        double what = Math.toDegrees(hoodAngle);

        //double c1 = (double) -13 /376;

        //double why = what * c1;

        //double when = (double) 475 /188;

        double hoodTime = (0.01625 * what) - 0.6;
        ActiveOpMode.telemetry().addData("hoodAngle", what);
        ActiveOpMode.telemetry().addData("ballVelocity", flywheelSpeed);
        ActiveOpMode.telemetry().addData("flywheelSpeed", requiredTPS);

        Double[] returnvalue = {requiredTPS, hoodTime, headingAngle};
        return returnvalue;
    }
}
