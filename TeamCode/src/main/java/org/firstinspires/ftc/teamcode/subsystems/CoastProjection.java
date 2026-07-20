package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Closed-form robot coast projection.
 *
 * Implements Step 4 of "On-Bot Control & Dynamic Targeting" (Control Lab,
 * Module 7). Derivation credited there to Havish Sripada (Pedro Pathing),
 * braking data by Jacob Ophoven.
 *
 * The braking-distance fit
 *
 *     D(v) = a*v^2 + b*v
 *
 * implies a deceleration at every speed:
 *
 *     vdot = -v / (2*a*v + b)
 *
 * Separating variables gives
 *
 *     2a*v + b*ln(v) = 2a*v0 + b*ln(v0) - t
 *
 * which solves for v(t) through the Lambert W function:
 *
 *     v(t) = (b / 2a) * W( (2a*v0 / b) * exp( (2a*v0 - t) / b ) )
 *
 * and the ground covered follows without a second integral, because the
 * remaining stopping distance is always D of the current speed:
 *
 *     x(t) = D(v0) - D(v(t))
 *
 * UNITS: the lesson states a in s^2/m and b in s. This class works entirely
 * in INCHES and seconds to match Pedro's pose/velocity units, so a is in
 * s^2/in and b remains in s. Fit the coefficients in inches accordingly.
 *
 * IMPORTANT (from the lesson's own caveat): these coefficients describe ONE
 * braking behavior. A zero-power coast and an actively braking follower
 * (Pedro applies roughly -0.2 power) fit the same D(v) = av^2 + bv form with
 * different numbers. Fit whichever behavior you actually shoot under.
 */
@Configurable
public class CoastProjection {

    /**
     * Quadratic braking coefficient, s^2/in.
     *
     * DERIVED FROM YOUR PEDRO CONSTANTS, not measured independently.
     * Pedro's forwardZeroPowerAcceleration = -47 in/s^2 is a CONSTANT
     * deceleration model, and constant decel gives D(v) = v^2 / (2|A|).
     * Matching that to D(v) = a*v^2 + b*v gives a = 1/(2*47) and b = 0.
     *
     * Your lateralZeroPowerAcceleration is -83 in/s^2, so strafing stops
     * much harder: a = 1/(2*83) = 0.006024. This class uses a single scalar
     * along the velocity direction, so 47 is the conservative choice (it
     * under-predicts the coast when strafing, which under-corrects rather
     * than over-corrects). If you want direction-dependent behavior, that is
     * a worthwhile upgrade once the scalar version is validated.
     */
    public static double coastA = 1.0 / (2.0 * 47.0);   // = 0.010638

    /**
     * Linear braking coefficient, s. Absorbs back-EMF and viscous drag.
     *
     * ZERO is correct for Pedro's constant-deceleration model. The class
     * handles b = 0 exactly via a constant-decel branch (the Lambert W form
     * has a removable singularity there).
     *
     * If you later fit real roll-out data you will likely find b > 0, because
     * real drivetrains do not decelerate at a constant rate. That fit will be
     * more accurate than this derivation. Until then this is consistent with
     * what Pedro itself believes about your robot.
     */
    public static double coastB = 0.0;

    /** Default projection horizon, seconds. The lesson projects about half a
     *  second ahead. NOTE: ShooterCalcAccel does not use this - it builds its
     *  own horizon from (feederDelay + turretSettleTime), because the ball only
     *  cares about the chassis state up to the instant it leaves. Kept for
     *  callers that want a general-purpose lookahead. */
    public static double projectionTime = 0.5;

    /** Battery voltage the coefficients were regressed at. */
    public static double nominalVoltage = 12.0;

    /** Master enable, so the projection can be toggled on the robot for A/B testing. */
    public static boolean enabled = true;

    /** Speeds below this (in/s) are treated as stopped. */
    private static final double MIN_SPEED = 1e-3;

    /**
     * Lambert W principal branch (W0), Halley's method.
     * W(z) is the inverse of u*e^u. Valid for z >= -1/e.
     */
    public static double lambertW0(double z) {
        double minZ = -1.0 / Math.E;
        if (Double.isNaN(z)) return 0.0;
        // At and below the branch point W = -1 exactly. Returning early also
        // avoids the Halley denominator (2w + 2) going singular at w = -1.
        if (z <= minZ) return -1.0;
        if (z == 0.0) return 0.0;

        // Initial guess: series near the branch point, log-based for large z.
        double w;
        if (z < 1.0) {
            double p = Math.sqrt(2.0 * (Math.E * z + 1.0));
            w = -1.0 + p - (p * p) / 3.0 + (11.0 / 72.0) * p * p * p;
        } else {
            double lz = Math.log(z);
            w = lz - Math.log(Math.max(lz, 1e-12));
        }
        if (w < -1.0 + 1e-9) w = -1.0 + 1e-9;

        // Halley iteration. Converges in a handful of steps.
        for (int i = 0; i < 60; i++) {
            double ew = Math.exp(w);
            double f = w * ew - z;
            double d1 = 2.0 * w + 2.0;
            if (Math.abs(d1) < 1e-12) break;
            double denom = ew * (w + 1.0) - ((w + 2.0) * f) / d1;
            if (Math.abs(denom) < 1e-300) break;
            double step = f / denom;
            w -= step;
            if (w < -1.0) w = -1.0;
            if (Math.abs(step) < 1e-12 * (1.0 + Math.abs(w))) break;
        }
        if (Double.isNaN(w) || Double.isInfinite(w)) return 0.0;
        return w;
    }

    /** Stopping distance D(v) = a*v^2 + b*v, inches. */
    public static double stoppingDistance(double v) {
        if (v <= 0.0) return 0.0;
        return coastA * v * v + coastB * v;
    }

    /**
     * Speed t seconds into a coast that began at v0.
     *
     * v(t) = (b / 2a) * W( (2a*v0 / b) * exp( (2a*v0 - t) / b ) )
     *
     * @param v0 speed at the moment power is cut, in/s
     * @param t  time into the coast, s
     * @return projected speed, in/s (never negative)
     */
    public static double speedAfter(double v0, double t) {
        if (v0 <= MIN_SPEED) return 0.0;
        if (t <= 0.0) return v0;
        if (coastA <= 0.0) return v0;   // no model at all; degrade to no coast

        // b == 0 is the constant-deceleration case, which is what Pedro's
        // zeroPowerAcceleration model gives: D(v) = v^2 / (2|A|) = a*v^2.
        // The Lambert W form has a removable singularity at b = 0, so solve it
        // directly instead. |A| = 1 / (2a).
        if (coastB <= 0.0) {
            double decel = 1.0 / (2.0 * coastA);      // in/s^2, positive
            double v = v0 - decel * t;
            return v > 0.0 ? v : 0.0;
        }

        double twoAv0 = 2.0 * coastA * v0;

        // Argument of W. The exponential overflows for large (2a*v0 - t)/b, so
        // evaluate in log space and only exponentiate when it is safe.
        double logArg = Math.log(twoAv0 / coastB) + (twoAv0 - t) / coastB;

        double w;
        if (logArg > 700.0) {
            // exp() would overflow. For large z, W(z) ~ L1 - L2 + L2/L1 where
            // L1 = ln(z), L2 = ln(ln(z)). Use the asymptotic form directly.
            double l1 = logArg;
            double l2 = Math.log(l1);
            w = l1 - l2 + l2 / l1;
        } else {
            w = lambertW0(Math.exp(logArg));
        }

        double v = (coastB / (2.0 * coastA)) * w;
        if (Double.isNaN(v) || v < 0.0) return 0.0;
        return Math.min(v, v0);   // a coast can only slow down
    }

    /**
     * Ground covered t seconds into the coast: x(t) = D(v0) - D(v(t)).
     *
     * @return distance travelled along the velocity direction, inches
     */
    public static double distanceAfter(double v0, double t) {
        if (v0 <= MIN_SPEED) return 0.0;
        if (t <= 0.0) return 0.0;
        if (coastA <= 0.0) return v0 * t;   // no model; constant velocity
        double d = stoppingDistance(v0) - stoppingDistance(speedAfter(v0, t));
        return Math.max(0.0, d);
    }

    /**
     * Voltage-compensated speed projection. The lesson notes the coefficients
     * were regressed at some V0, and at a different voltage the speed profile
     * scales by Vnew / V0.
     */
    public static double speedAfter(double v0, double t, double batteryVolts) {
        double v = speedAfter(v0, t);
        if (batteryVolts > 1.0 && nominalVoltage > 1.0) {
            v *= (batteryVolts / nominalVoltage);
        }
        return Math.min(v, v0);
    }
}
