package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.geometry.Pose;

public class LaunchDetector {

        private static class Triangle {
            double x1, y1, x2, y2, x3, y3;
            double detT;

            Triangle(double x1, double y1, double x2, double y2, double x3, double y3) {
                this.x1 = x1; this.y1 = y1;
                this.x2 = x2; this.y2 = y2;
                this.x3 = x3; this.y3 = y3;
                this.detT = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);
            }

            /**
             * Checks if the center point (px, py) is inside the triangle using Barycentric coordinates.
             */
            boolean containsCenter(double px, double py) {
                double alpha = ((y2 - y3) * (px - x3) + (x3 - x2) * (py - y3)) / detT;
                double beta = ((y3 - y1) * (px - x3) + (x1 - x3) * (py - y3)) / detT;
                double gamma = 1.0 - alpha - beta;

                return alpha >= 0 && beta >= 0 && gamma >= 0;
            }
        }

        // --- DECODE FIELD COORDINDATES (INCHES) ---
        private static final Triangle LARGE_LAUNCH_ZONE = new Triangle(72.0, 72.0, 144.0, 144.0, 0.0, 144.0);
        private static final Triangle SMALL_LAUNCH_ZONE = new Triangle(48.0, 0.0, 72.0, 24.0, 96.0, 0.0);

        // --- ROBOT DIMENSIONS (412.7mm x 429.612mm -> Inches) ---
        private static final double ROBOT_WIDTH = 16.248;
        private static final double ROBOT_LENGTH = 16.914;
        private static final double HALF_ROBOT_WIDTH = ROBOT_WIDTH / 2.0;
        private static final double HALF_ROBOT_LENGTH = ROBOT_LENGTH / 2.0;

        public static boolean isOverlappingLaunchZone(Pose robotPose) {
            return isOverlappingLaunchZone(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        }

        public static boolean isOverlappingLaunchZone(double cx, double cy, double heading) {
            // TEST 1: Absolute Subsumption Check (Is the robot's center fully inside either zone?)
            if (LARGE_LAUNCH_ZONE.containsCenter(cx, cy) || SMALL_LAUNCH_ZONE.containsCenter(cx, cy)) {
                return true;
            }

            // TEST 2: Straddling Check (Do the robot's perimeter walls cross the triangle tape lines?)
            double hW = HALF_ROBOT_WIDTH;
            double hL = HALF_ROBOT_LENGTH;

            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            double x0 = cx + (hL * cos + hW * sin);
            double y0 = cy + (hL * sin - hW * cos);
            double x1 = cx + (hL * cos - hW * sin);
            double y1 = cy + (hL * sin + hW * cos);
            double x2 = cx + (-hL * cos - hW * sin);
            double y2 = cy + (-hL * sin + hW * cos);
            double x3 = cx + (-hL * cos + hW * sin);
            double y3 = cy + (-hL * sin - hW * cos);

            return robotEdgesIntersectTriangle(x0, y0, x1, y1, x2, y2, x3, y3, LARGE_LAUNCH_ZONE) ||
                    robotEdgesIntersectTriangle(x0, y0, x1, y1, x2, y2, x3, y3, SMALL_LAUNCH_ZONE);
        }

        private static boolean robotEdgesIntersectTriangle(double x0, double y0, double x1, double y1,
                                                           double x2, double y2, double x3, double y3,
                                                           Triangle t) {
            return robotEdgeIntersectsTriangle(x0, y0, x1, y1, t)
                    || robotEdgeIntersectsTriangle(x1, y1, x2, y2, t)
                    || robotEdgeIntersectsTriangle(x2, y2, x3, y3, t)
                    || robotEdgeIntersectsTriangle(x3, y3, x0, y0, t);
        }

        private static boolean robotEdgeIntersectsTriangle(double rx1, double ry1, double rx2, double ry2, Triangle t) {
            return lineIntersectsSegment(rx1, ry1, rx2, ry2, t.x1, t.y1, t.x2, t.y2)
                    || lineIntersectsSegment(rx1, ry1, rx2, ry2, t.x2, t.y2, t.x3, t.y3)
                    || lineIntersectsSegment(rx1, ry1, rx2, ry2, t.x3, t.y3, t.x1, t.y1);
        }

        private static boolean lineIntersectsSegment(double x1, double y1, double x2, double y2,
                                                     double x3, double y3, double x4, double y4) {
            double den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (den == 0) return false;

            double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
            double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;

            return t >= 0 && t <= 1 && u >= 0 && u <= 1;
        }
}
