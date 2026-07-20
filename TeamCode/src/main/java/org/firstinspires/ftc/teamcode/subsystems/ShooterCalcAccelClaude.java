package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.SCORE_ANGLE;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Shoot-on-the-move solver.
 *
 * Restructured to follow "On-Bot Control & Dynamic Targeting" (Control Lab,
 * Module 7, Advanced Research):
 *
 *   Step 3 - SOTM is a fixed-point problem. The virtual target changes the
 *            distance, the distance changes flight time, and flight time
 *            changes the virtual target. Iterate to a tolerance with a hard
 *            iteration cap so the real-time loop always returns.
 *
 *   Step 4 - The robot is braking while the shot happens. Project the pose
 *            and velocity forward with the closed-form coast model and hand
 *            the PROJECTED state to the solver. For a servo turret this
 *            projection doubles as the velocity reference (Step 5).
 *
 * The lesson writes the lead as G*v*(t_d + t_f). This implementation splits
 * that product: t_d (feeder delay + turret settle) is applied by projecting the
 * robot's launch state forward through the coast, and t_f is applied as the
 * lead inside the fixed-point loop. That split is deliberate and matters,
 * because the robot's speed is not constant across the two intervals - it is
 * still decaying. Using one averaged velocity across (t_d + t_f) leads wrong.
 */
@Configurable
public class ShooterCalcAccelClaude implements Subsystem {

    // ---- Step 3: fixed-point solver parameters -----------------------------

    /** Feeder / launch delay before free flight, seconds (t_d in the lesson).
     *  Set to 0.2 based on the team's own estimate, matching the lookahead
     *  already used by the auto's predictive launch-zone check.
     *  Worth confirming on high-frame-rate video: stopper-open to ball-exit. */
    public static double feederDelay = 0.1;

    /** Servo turret transport lag, seconds. Added to the projection horizon so
     *  the turret is commanded toward where the shot will be taken rather than
     *  chasing. The lesson's Step 5 note: a servo turret has a position-only
     *  interface, so aiming at the projected pose is the velocity reference. */
    public static double turretSettleTime = 0.03;

    /** Empirical lead gain (G in the lesson). 1.0 = pure geometric lead. */
    public static double leadGain = 1;

    /** Convergence tolerance on flight time, seconds. */
    public static double solveTolerance = 0.001;

    /** Hard iteration cap. The lesson notes convergence normally takes 2-3
     *  rounds; the cap exists only to bound the loop if the model misbehaves. */
    public static int maxIterations = 5;

    // ---- Step 4: coast projection -----------------------------------------

    // ---- Step 4: launch-state projection -----------------------------------

    /** Project using measured along-track acceleration. Default: adapts to
     *  coasting, active braking, and powered acceleration alike. */
    public static final int MODE_MEASURED_ACCEL = 0;
    /** Project using the fitted coast curve in CoastProjection. Correct only
     *  if the robot always decelerates the same way. */
    public static final int MODE_FIXED_COAST = 1;
    /** No projection: assume constant velocity through the horizon. */
    public static final int MODE_NONE = 2;

    /** Which projection to use. Field-tunable for A/B testing. */
    public static int projectionMode = MODE_MEASURED_ACCEL;

    /** EMA coefficient on along-track acceleration while DECELERATING.
     *  Tuned for noise robustness, not tracking speed: at a 0.25s horizon the
     *  projection tolerates lag far better than it tolerates noise. Simulation
     *  worst-case hit rate (7in target, sigma=75 in/s^2 accel noise) was 96%
     *  at 0.05 versus 28% at 0.3. */
    public static double accelFilterAlphaDecel = 0.15;

    /** EMA coefficient while ACCELERATING. Powered acceleration decays as the
     *  robot approaches free speed, so a fast filter overshoots. */
    public static double accelFilterAlphaAccel = 0.05;

    /** Hard ceiling on projected speed, in/s. The drivetrain physically cannot
     *  exceed its free speed (~123 in/s at 15V, ~98 at 12V), so any projection
     *  above this is filter overshoot, not real motion. Without this clamp the
     *  hit rate while accelerating dropped from 100% to 24% in simulation. */
    public static double maxProjectedSpeed = 125.0;

    /** Compensate for the velocity vector rotating during the projection
     *  horizon. Without this a steady turn leaves the lead pointing where the
     *  robot USED to be heading: at 30 deg/s the hit rate on a 7in target fell
     *  to 50%, and with it restored to 99%. */
    public static boolean compensateTurning = true;

    /** Ignore |angular velocity| below this (rad/s) as noise. */
    public static double minAngularVelocity = 0.02;

    /** Clamp |angular velocity| to this (rad/s) so a bad reading cannot swing
     *  the lead wildly. 6 rad/s is about 344 deg/s. */
    public static double maxAngularVelocity = 6.0;

    /** Reject |acceleration| above this (in/s^2) as sensor noise. The
     *  drivetrain is traction-limited near 300-430 in/s^2, so anything much
     *  beyond that is not physically reachable. */
    public static double accelClamp = 400.0;

    /** Retained EMA state. Reset by resetProjectionFilter(). */
    private static double filteredAlongTrackAccel = 0.0;

    /** Clear the acceleration filter. Call from opmode init so a stale value
     *  from the previous run cannot leak into the first loops of the next. */
    public static void resetProjectionFilter() {
        filteredAlongTrackAccel = 0.0;
        lastAlongTrackAccel = 0.0;
        lastProjectedSpeed = 0.0;
    }

    // ---- Existing empirical terms -----------------------------------------

    /** Time-of-flight drag correction. Kept from the original calibration. */
    public static double dragTimeFactor = 1;

    /** @deprecated Unused. The coast model replaced the accel lookahead scalar.
     *  Kept only so existing references elsewhere still compile. */
    @Deprecated
    public static double accelScalar = 0.0;

    /** @deprecated Unused; the RPM path is commented out. Kept for compatibility. */
    @Deprecated
    public static double rpmoffset = 200.0;

    public static double sotmFactor = 1.0;
    public static double sotmOffset = 0.0;
    public static double verticalShift = 0.0;
    public static double verticalShiftStep = 50.0;

    /** Below this projected speed (in/s) the shot is treated as stationary. */
    public static double sotmMinSpeed = 2.0;

    /** Width of the blend between calibration zones, inches. Replaces the hard
     *  switches, which produced a 13 degree step in commanded entry angle at
     *  x = 136 and could flip frame to frame on velocity noise. */
    public static double zoneBlendWidth = 10.0;

    // ---- Telemetry outputs -------------------------------------------------
    public static double requiredTPS = 0.0;
    public static double requiredRPM = 0.0;
    public static int lastIterations = 0;
    public static boolean lastConverged = false;
    public static double lastFlightTime = 0.0;
    public static double lastProjectedSpeed = 0.0;
    public static double lastAlongTrackAccel = 0.0;

    private static final double G_IN = 32.174 * 12.0;   // in/s^2
    private static final double HOOD_MIN = Math.toRadians(40.0);
    private static final double HOOD_MAX = Math.toRadians(75.0);

    private static final double CLOSE_ZONE_END = 66.29;
    private static final double FAR_ZONE_START = 136.0;

    // ---- Calibration curves ------------------------------------------------

    /** Target height above the launch point, inches, blended across zones. */
    public static double targetHeight(double range) {
        double poly = 0.0032 * range * range - 0.6653 * range + 66.888 + 1.0;
        if (range <= FAR_ZONE_START) return poly;
        if (range >= FAR_ZONE_START + zoneBlendWidth) return 36.0;
        double f = (range - FAR_ZONE_START) / zoneBlendWidth;
        return poly + (36.0 - poly) * f;
    }

    /** Desired entry angle, radians, blended across zones. */
    public static double entryAngle(double range) {
        double closeDeg = 0.6106 * range - 57.478;
        double midDeg = SCORE_ANGLE;
        double farDeg = -30.0;
        double half = zoneBlendWidth / 2.0;

        double deg;
        if (range < CLOSE_ZONE_END - half) {
            deg = closeDeg;
        } else if (range < CLOSE_ZONE_END + half) {
            double f = (range - (CLOSE_ZONE_END - half)) / zoneBlendWidth;
            deg = closeDeg + (midDeg - closeDeg) * f;
        } else if (range < FAR_ZONE_START) {
            deg = midDeg;
        } else if (range < FAR_ZONE_START + zoneBlendWidth) {
            double f = (range - FAR_ZONE_START) / zoneBlendWidth;
            deg = midDeg + (farDeg - midDeg) * f;
        } else {
            deg = farDeg;
        }
        return Math.toRadians(deg);
    }

    /**
     * Ballistic solve for a static target.
     * @return {hoodAngleRadians, launchSpeedInchesPerSecond}
     */
    public static double[] solveStatic(double range, double height, double entry) {
        if (range <= 1e-6) return new double[]{HOOD_MAX, 0.0};

        double hood = Math.atan(2.0 * height / range - Math.tan(entry));
        if (isNaN(hood)) hood = HOOD_MAX;
        hood = MathFunctions.clamp(hood, HOOD_MIN, HOOD_MAX);

        double denom = 2.0 * Math.pow(Math.cos(hood), 2) * (range * Math.tan(hood) - height);
        if (denom <= 0.0) {
            // Geometry unreachable at this hood angle. Try the steepest hood
            // before giving up, since a higher arc can clear a near target.
            hood = HOOD_MAX;
            denom = 2.0 * Math.pow(Math.cos(hood), 2) * (range * Math.tan(hood) - height);
            if (denom <= 0.0) return new double[]{hood, 0.0};
        }
        double speed = Math.sqrt(G_IN * range * range / denom);
        if (isNaN(speed) || Double.isInfinite(speed)) speed = 0.0;
        return new double[]{hood, speed};
    }

    /** Wrap an angle in degrees into (-180, 180]. */
    public static double normalizeDegrees(double degrees) {
        if (isNaN(degrees) || Double.isInfinite(degrees)) return 0.0;
        degrees = degrees % 360.0;
        if (degrees > 180.0) degrees -= 360.0;
        if (degrees <= -180.0) degrees += 360.0;
        return degrees;
    }

    /** Flight time to a target at the given horizontal range, seconds. */
    public static double flightTime(double range) {
        double[] s = solveStatic(range, targetHeight(range), entryAngle(range));
        double horizontal = s[1] * Math.cos(s[0]);
        if (horizontal < 1e-6) return Double.NaN;
        return dragTimeFactor * (range / horizontal);
    }

    /**
     * @param robotHeading      field heading, radians
     * @param robotToGoalVector turret-to-goal, magnitude in inches
     * @param robotVel          chassis velocity, in/s
     * @param robotAccel        chassis acceleration, in/s^2. Used by
     *                          MODE_MEASURED_ACCEL to project the launch state.
     *                          May be null or zero; the projection degrades to
     *                          constant velocity in that case.
     * @return {requiredTPS, hoodServoPosition, turretHeadingErrorDegrees}
     */
    public static Double[] calculateShotVectorandUpdateHeading(
            double robotHeading, Vector robotToGoalVector, Vector robotVel, Vector robotAccel) {
        return calculateShotVectorandUpdateHeading(
                robotHeading, robotToGoalVector, robotVel, robotAccel, 0.0);
    }

    /**
     * @param robotAngularVelocity chassis yaw rate, rad/s. Used to rotate the
     *                             velocity vector forward through the horizon
     *                             so the lead points where the robot will
     *                             actually be heading at launch. Pass 0 if
     *                             unavailable; turning compensation is skipped.
     */
    public static Double[] calculateShotVectorandUpdateHeading(
            double robotHeading, Vector robotToGoalVector, Vector robotVel,
            Vector robotAccel, double robotAngularVelocity) {

        double x = robotToGoalVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        if (x < 1.0) x = 1.0;

        double goalTheta = robotToGoalVector.getTheta();

        // ---- Step 4: project the robot state forward to the LAUNCH INSTANT ---
        // The solver must be fed where the robot WILL be, not where it was when
        // the trigger was pulled. This also gives the servo turret its command
        // early, which is the poor-man's velocity reference of Step 5.
        //
        // The projection horizon is the time from now until the ball actually
        // LEAVES the robot: turret settle + feeder delay. Once the ball is in
        // free flight it no longer cares what the chassis does, so the lead
        // inside the fixed-point loop below is flight time ALONE - the feeder
        // delay is already accounted for by projecting to the launch instant.
        // Double-counting it there would over-lead by v * feederDelay.
        //
        // MODE_MEASURED_ACCEL is the default because this robot's deceleration
        // is not repeatable: a zero-power coast is about 47 in/s^2 while active
        // braking is traction-limited near 300 in/s^2. A single fitted coast
        // curve cannot cover both. Reading the actual acceleration adapts to
        // whichever is happening, at the cost of sensor noise - hence the EMA.
        double currentSpeed = robotVel.getMagnitude();
        double velTheta = robotVel.getTheta();

        double horizon = feederDelay + turretSettleTime;

        // Signed acceleration along the direction of travel. getMagnitude() is
        // never negative, so the component must be recovered by projection.
        // Negative = slowing down, positive = speeding up.
        double aAlongV = 0.0;
        if (robotAccel != null
                && currentSpeed > 1e-6
                && robotAccel.getMagnitude() > 1e-6) {
            aAlongV = robotAccel.getMagnitude()
                    * Math.cos(robotAccel.getTheta() - velTheta);
        }
        if (isNaN(aAlongV) || Double.isInfinite(aAlongV)) aAlongV = 0.0;
        // Reject physically impossible readings before they reach the filter.
        if (aAlongV > accelClamp) aAlongV = accelClamp;
        if (aAlongV < -accelClamp) aAlongV = -accelClamp;

        // Asymmetric EMA on the along-track acceleration. Pedro's acceleration
        // is a differentiated velocity, so it is noisy; unfiltered it is
        // roughly 4x worse than filtered at this horizon. Deceleration is
        // tracked faster than acceleration because braking is close to
        // constant while powered acceleration decays toward free speed.
        if (isNaN(filteredAlongTrackAccel)) filteredAlongTrackAccel = 0.0;
        double alpha = (aAlongV < 0.0) ? accelFilterAlphaDecel : accelFilterAlphaAccel;
        filteredAlongTrackAccel += alpha * (aAlongV - filteredAlongTrackAccel);
        lastAlongTrackAccel = filteredAlongTrackAccel;

        double projectedSpeed;
        double projectedDistance;

        switch (projectionMode) {
            case MODE_MEASURED_ACCEL: {
                double a = filteredAlongTrackAccel;
                double vEnd = currentSpeed + a * horizon;
                if (vEnd < 0.0) {
                    // The robot stops partway through the horizon. Integrate
                    // only up to the stop instead of letting speed go negative.
                    double tStop = (a < -1e-9) ? (currentSpeed / -a) : 0.0;
                    if (tStop < 0.0) tStop = 0.0;
                    if (tStop > horizon) tStop = horizon;
                    projectedSpeed = 0.0;
                    projectedDistance = currentSpeed * tStop + 0.5 * a * tStop * tStop;
                } else {
                    // Clamp to physical free speed. Filter lag during powered
                    // acceleration otherwise predicts speeds the drivetrain
                    // cannot reach, which over-leads badly.
                    if (vEnd > maxProjectedSpeed) vEnd = maxProjectedSpeed;
                    projectedSpeed = vEnd;
                    double vAvg = 0.5 * (currentSpeed + vEnd);
                    projectedDistance = vAvg * horizon;
                }
                break;
            }
            case MODE_FIXED_COAST: {
                projectedSpeed = CoastProjection.speedAfter(currentSpeed, horizon);
                projectedDistance = CoastProjection.distanceAfter(currentSpeed, horizon);
                break;
            }
            case MODE_NONE:
            default: {
                projectedSpeed = currentSpeed;
                projectedDistance = currentSpeed * horizon;
                break;
            }
        }

        if (isNaN(projectedSpeed) || projectedSpeed < 0.0) projectedSpeed = 0.0;
        if (isNaN(projectedDistance) || projectedDistance < 0.0) projectedDistance = 0.0;
        lastProjectedSpeed = projectedSpeed;

        // Rotate the velocity direction forward through the horizon. The ball
        // inherits the chassis velocity AT LAUNCH, not now, and on a curved
        // path those differ by omega*horizon. Using the stale direction points
        // the lead where the robot used to be going.
        double omega = robotAngularVelocity;
        if (isNaN(omega) || Double.isInfinite(omega)) omega = 0.0;
        if (!compensateTurning || Math.abs(omega) < minAngularVelocity) omega = 0.0;
        if (omega > maxAngularVelocity) omega = maxAngularVelocity;
        if (omega < -maxAngularVelocity) omega = -maxAngularVelocity;

        // Direction the robot will be travelling when the ball leaves.
        double velThetaLaunch = velTheta + omega * horizon;
        // Mean direction over the horizon, used for the displacement chord.
        double velThetaMid = velTheta + omega * horizon * 0.5;

        // Shift the goal vector to account for the ground covered during the
        // projection window. Working in a frame where +X points at the goal.
        double coastAlongGoal = projectedDistance * Math.cos(velThetaMid - goalTheta);
        double coastPerpGoal = projectedDistance * Math.sin(velThetaMid - goalTheta);

        double projX = x - coastAlongGoal;
        double projY = -coastPerpGoal;
        double projectedRange = Math.hypot(projX, projY);
        if (projectedRange < 1.0) projectedRange = 1.0;
        // Angle from the projected robot position to the goal, relative to the
        // present goal bearing.
        double projectedBearingShift = Math.atan2(projY, projX);

        boolean sotmActive = projectedSpeed >= sotmMinSpeed;

        // Velocity components in the projected goal frame, using the direction
        // the robot will actually be travelling at launch.
        double coordinateTheta = velThetaLaunch - (goalTheta + projectedBearingShift);
        double parallelComponent = sotmActive ? Math.cos(coordinateTheta) * projectedSpeed : 0.0;
        double perpendicularComponent = sotmActive ? Math.sin(coordinateTheta) * projectedSpeed : 0.0;

        // ---- Step 3: bounded fixed-point solve ------------------------------
        //   virtual   <- projected_target - lead_gain * projected_velocity * t_f
        //   next_time <- flight_time(distance(shooter, virtual))
        //   stop when |next_time - t_f| < tolerance, or at maxIterations
        // The lesson's t_d appears in the projection horizon above, not here.
        double tf = flightTime(projectedRange);
        if (isNaN(tf) || tf <= 0.0) tf = 0.5;   // seed if the model fails

        double virtualRange = projectedRange;
        double virtualX = projectedRange;
        double virtualY = 0.0;
        int iterations = 0;
        boolean converged = false;

        for (int i = 0; i < maxIterations; i++) {
            iterations = i + 1;

            // Lead is flight time only. The feeder delay was already consumed
            // by projecting the launch state forward through the coast above.
            // Virtual target: displaced opposite the robot's motion.
            virtualX = projectedRange - leadGain * parallelComponent * tf;
            virtualY = -leadGain * perpendicularComponent * tf;
            virtualRange = Math.hypot(virtualX, virtualY);
            if (virtualRange < 1.0) virtualRange = 1.0;

            double nextTf = flightTime(virtualRange);
            if (isNaN(nextTf) || nextTf <= 0.0) break;

            if (Math.abs(nextTf - tf) < solveTolerance) {
                tf = nextTf;
                converged = true;
                break;
            }
            tf = nextTf;
        }
        lastIterations = iterations;
        lastConverged = converged;
        lastFlightTime = tf;

        // ---- Final ballistic solution at the virtual target ------------------
        double[] sol = solveStatic(virtualRange, targetHeight(virtualRange), entryAngle(virtualRange));
        double hoodAngle = sol[0];
        double launchSpeed = sol[1];

        double ballSpeedMps = launchSpeed / 39.37;

        // ---- Turret heading --------------------------------------------------
        // Bearing to the virtual target, relative to the present goal bearing.
        double virtualBearing = goalTheta + projectedBearingShift + Math.atan2(virtualY, virtualX);
        double headingAngle = normalizeDegrees(Math.toDegrees(virtualBearing - robotHeading));

        // sotmFactor scales the lead for empirical trim without touching the
        // solver. At 1.0 the geometric solution is used unmodified.
        if (sotmActive && sotmFactor != 1.0) {
            double baseBearing = normalizeDegrees(Math.toDegrees(goalTheta - robotHeading));
            double leadDelta = normalizeDegrees(headingAngle - baseBearing);
            headingAngle = normalizeDegrees(baseBearing + sotmFactor * leadDelta);
        }

        // ---- Flywheel command -------------------------------------------------
        double tps = (-16.19 * ballSpeedMps * ballSpeedMps + 449.11 * ballSpeedMps - 964.9)
                + verticalShift;
        if (sotmActive) tps -= sotmOffset;

        // The TPS regression is a downward parabola: it turns over near
        // 13.9 m/s and goes negative below about 2.3 m/s. Guard both rails so a
        // bad solve cannot command a nonsense wheel speed.
        if (isNaN(tps) || Double.isInfinite(tps) || launchSpeed <= 0.0) tps = 0.0;
        if (tps < 0.0) tps = 0.0;

        requiredTPS = tps;
        requiredRPM = tps * 60.0 / 28.0;

        double hoodDegrees = Math.toDegrees(hoodAngle);
        double hoodTime = (0.01625 * hoodDegrees) - 0.6;
        if (isNaN(hoodTime)) hoodTime = 0.0;

        ActiveOpMode.telemetry().addData("range (now)", x);
        ActiveOpMode.telemetry().addData("range (projected)", projectedRange);
        ActiveOpMode.telemetry().addData("range (virtual)", virtualRange);
        ActiveOpMode.telemetry().addData("speed now / projected",
                currentSpeed + " / " + projectedSpeed);
        ActiveOpMode.telemetry().addData("flightTime", tf);
        ActiveOpMode.telemetry().addData("solver iters / converged",
                iterations + " / " + converged);
        ActiveOpMode.telemetry().addData("aAlongV (filt)", filteredAlongTrackAccel);
        ActiveOpMode.telemetry().addData("projMode", projectionMode);
        ActiveOpMode.telemetry().addData("sotmActive", sotmActive);
        ActiveOpMode.telemetry().addData("headingAngle", headingAngle);

        return new Double[]{tps, hoodTime, headingAngle};
    }
}
