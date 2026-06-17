package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.SCORE_HEIGHT;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import java.lang.Math;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class ShooterCalc implements Subsystem {

    //public static double farzoneangle = -0.34006585;

    //public static double farzoneheight = 10.25;


    public static double requiredRPM;

    public static double rpmoffset = 200;
    public static double requiredTPS = (28*requiredRPM)/60;
    public static double verticalShift = 0;
    public static double verticalShiftStep = 50;

    public static double accelScalar = 0.015; // Set to 0 to disable
    private static final double GRAVITY_INCHES_PER_SECOND_SQUARED = 32.174*12;
    private static final double MIN_HOOD_ANGLE = Math.toRadians(40);
    private static final double MAX_HOOD_ANGLE = Math.toRadians(75);

    public static class ShotVectorResult {
        public double flywheelSpeed;
        public double hoodTime;
        public double headingAngle;
    }

    public static void calculateShotVectorandUpdateHeading(double robotHeading, Vector robotToGoalVector, Vector robotVel, double sotmFactor, ShotVectorResult result){
        calculateShotVectorandUpdateHeading(
                robotHeading,
                robotToGoalVector.getMagnitude(),
                robotToGoalVector.getTheta(),
                robotVel,
                sotmFactor,
                result
        );
    }

    public static void calculateShotVectorandUpdateHeading(double robotHeading, double robotToGoalMagnitude, double robotToGoalTheta, Vector robotVel, double sotmFactor, ShotVectorResult result){
        double x = robotToGoalMagnitude-ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        //double y = -4.5745*temp*temp*temp + 25.978*temp*temp - 48.395*temp + 58.675;
        double y = SCORE_HEIGHT;
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(ShooterConstants.SCORE_ANGLE)), MIN_HOOD_ANGLE,
                MAX_HOOD_ANGLE);

        if(isNaN(hoodAngle)){
            hoodAngle=MAX_HOOD_ANGLE;
        }
        double hoodCos = Math.cos(hoodAngle);
        double hoodTan = Math.tan(hoodAngle);
        double flywheelSpeed = Math.sqrt(GRAVITY_INCHES_PER_SECOND_SQUARED * x * x / (2 * hoodCos * hoodCos * (x * hoodTan - y)));
        if(x<50){
            y=SCORE_HEIGHT + 2;
        }
        double robotVelocityMagnitude = robotVel.getMagnitude();
        double robotVelocityTheta = robotVel.getTheta();
        if (robotVelocityMagnitude < 5.0) {
            robotVelocityMagnitude = 0;
            robotVelocityTheta = 0;
        }

        double coordinateTheta = robotVelocityTheta - robotToGoalTheta;

        double parallelComponent = Math.cos(coordinateTheta) * robotVelocityMagnitude;
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocityMagnitude;

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = (x / (flywheelSpeed * hoodCos)); //maybe try 1.25 SIDENOTE revert back to 1.2 if accel change does NOT work, then multiply the speed itself by 1.2
        double ivr = x / time - parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), MIN_HOOD_ANGLE,
                MAX_HOOD_ANGLE);

        if(isNaN(hoodAngle)){
            hoodAngle=MAX_HOOD_ANGLE;
        }

        //y = /*SCORE_HEIGHT*/ -4.5745*newtemp*newtemp*newtemp + 25.978*newtemp*newtemp - 48.395*newtemp + 58.675; // -0.7135x2 + 0.8315x + 33.532
        hoodCos = Math.cos(hoodAngle);
        hoodTan = Math.tan(hoodAngle);
        flywheelSpeed = Math.sqrt(GRAVITY_INCHES_PER_SECOND_SQUARED * ndr * ndr / (2 * hoodCos * hoodCos * (ndr * hoodTan - y)));
        flywheelSpeed = flywheelSpeed/ 39.37;

        double safeIvr = Math.max(ivr, 50.0);
        double headingVelCompOffset = sotmFactor * Math.atan(perpendicularComponent / safeIvr);
        double headingAngle = Math.toDegrees(robotToGoalTheta - robotHeading - headingVelCompOffset);

        //requiredRPM = (1.4286* Math.pow(flywheelSpeed, 3) - 39.264*Math.pow(flywheelSpeed, 2) + 863.57*flywheelSpeed-1373.9 + rpmoffset);
        //requiredTPS = (28 * requiredRPM) / 60;
        //requiredTPS = 12.79084*Math.pow(flywheelSpeed, 2)-69.40057*flywheelSpeed+849.93005;

        double flywheelSpeedSquared = flywheelSpeed * flywheelSpeed;
        requiredTPS = (-16.19*flywheelSpeedSquared+ 449.11*flywheelSpeed- 964.9) + verticalShift;
        double what = Math.toDegrees(hoodAngle);

        //double c1 = (double) -13 /376;

        //double why = what * c1;

        //double when = (double) 475 /188;

        double hoodTime = (0.01625 * what) - 0.6;
        ActiveOpMode.telemetry().addData("hoodAngle", what);
        ActiveOpMode.telemetry().addData("ballVelocity", flywheelSpeed);
        ActiveOpMode.telemetry().addData("flywheelSpeed", requiredTPS);

        result.flywheelSpeed = requiredTPS;
        result.hoodTime = hoodTime;
        result.headingAngle = headingAngle;
    }

    public static Double[] calculateShotVectorandUpdateHeading(double robotHeading, Vector robotToGoalVector, Vector robotVel, double sotmFactor){
        ShotVectorResult result = new ShotVectorResult();
        calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, robotVel, sotmFactor, result);
        return new Double[] {result.flywheelSpeed, result.hoodTime, result.headingAngle};
    }
}
