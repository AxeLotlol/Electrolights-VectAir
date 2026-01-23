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

    public static boolean lowangle = false;

    double dist;


    public static double requiredRPM;
    public static double requiredTPS = (28*requiredRPM)/60;

    public static Double[] calculateShotVectorandUpdateHeading(double robotHeading, Vector robotToGoalVector, Vector robotVel){
        //Vector robotToGoalVector = new Vector(goalX-robotPoseX, goalY - robotPoseY);
        double g = 32.174*12;
        double x = robotToGoalVector.getMagnitude()-ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = ShooterConstants.SCORE_ANGLE;

        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), Math.toRadians(44.2),
                Math.toRadians(63));

        if(isNaN(hoodAngle)){
            hoodAngle=Math.toRadians(63);
        }

        //double numerator = 9.81 * Math.pow(x, 2);
        //double denominator = (2 * Math.pow(Math.cos(Math.toRadians(hoodAngle)), 2) * (x * Math.tan(Math.toRadians(hoodAngle)) - (y / 39.37)));
        //double v0 = Math.sqrt(numerator / denominator);
        //double flywheelSpeed = v0 * 39.37;
        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math. tan(hoodAngle) - y)));
        //double flywheelSpeed = Math.sqrt(g * x * x / (2* Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));
        //flywheelSpeed = flywheelSpeed/ 39.37;
        Vector robotVelocity = robotVel;

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), Math.toRadians(44.2),
                Math.toRadians(63));

        if(isNaN(hoodAngle)){
            hoodAngle=Math.toRadians(63);
        }

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math. tan(hoodAngle) - y)));
        flywheelSpeed = flywheelSpeed/ 39.37;
        double headingVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double headingAngle = Math.toDegrees(robotHeading - robotToGoalVector.getTheta() + headingVelCompOffset);

        requiredRPM = 1.4286* Math.pow(flywheelSpeed, 3) - 39.264*Math.pow(flywheelSpeed, 2) + 863.57*flywheelSpeed-1373.9;
        requiredTPS = (28 * requiredRPM) / 60;

        double what = Math.toDegrees(hoodAngle);

        double c1 = (double) -13 /376;

        double why = what * c1;

        double when = (double) 475 /188;

        double hoodTime = why + when;/*Math.toDegrees(hoodAngle) * ShooterConstants.RAISE_TIME / 18.8;*/

        ActiveOpMode.telemetry().addData("v0", flywheelSpeed);
        ActiveOpMode.telemetry().addData("rpm", requiredRPM);
        ActiveOpMode.telemetry().addData("angle", Math.toDegrees(hoodAngle));
        ActiveOpMode.telemetry().addData("time", hoodTime);
        ActiveOpMode.telemetry().addData("c1", c1);
        ActiveOpMode.telemetry().addData("why", why);
        ActiveOpMode.telemetry().addData("when", when);
        ActiveOpMode.telemetry().addData("distance", robotToGoalVector.getMagnitude()/39.37);

        Double[] returnvalue = {requiredTPS, hoodTime, headingAngle};
        return returnvalue;
    }
}