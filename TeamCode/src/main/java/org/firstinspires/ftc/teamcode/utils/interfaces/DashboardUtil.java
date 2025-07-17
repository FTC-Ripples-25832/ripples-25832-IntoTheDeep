package org.firstinspires.ftc.teamcode.utils.interfaces;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.hardware.Localizer;

import java.util.List;
import java.util.Vector;

/**
 * Set of helper functions for drawing FTC Dashboard visualizations.
 */
public class DashboardUtil {
        private static final double ROBOT_RADIUS = 230; // mm (9 inches)
        private static final double POLE_RADIUS = 12.7; // mm (0.5 inches)

        /**
         * localizer data
         */
        public static void drawRobot(Canvas canvas, String color) {
                // Convert from mm to inches for display
                double xInches = Localizer.X / 25.4;
                double yInches = Localizer.Y / 25.4;
                double theta = Localizer.theta; // theta is already in radians

                canvas.setStroke(color);
                canvas.strokeCircle(xInches, yInches, ROBOT_RADIUS / 25.4);
                canvas.strokeCircle(xInches, yInches, 12.7 / 25.4); // 0.5 inch center point

                // Draw heading indicator line
                double headingX = xInches + Math.cos(theta) * (ROBOT_RADIUS / 25.4);
                double headingY = yInches + Math.sin(theta) * (ROBOT_RADIUS / 25.4);
                canvas.strokeLine(xInches, yInches, headingX, headingY);
        }

        /**
         * no heading
         */
        public static void drawRobotPosition(Canvas canvas) {
                double xInches = Localizer.X / 25.4;
                double yInches = Localizer.Y / 25.4;
                canvas.strokeCircle(xInches, yInches, ROBOT_RADIUS / 25.4);
        }

        /**
         * Draws a trail of robot poses from position history
         */
//        public static void drawPoseHistory(Canvas canvas, List<double[]> poseHistory) {
//                if (poseHistory.isEmpty())
//                        return;
//
//                double[] xPoints = new double[poseHistory.size()];
//                double[] yPoints = new double[poseHistory.size()];
//
//                // Convert from mm to inches for each point
//                for (int i = 0; i < poseHistory.size(); i++) {
//                        double[] pose = poseHistory.get(i);
//                        xPoints[i] = pose[0] / 25.4; // X in inches
//                        yPoints[i] = pose[1] / 25.4; // Y in inches
//                }
//                canvas.strokePolyline(xPoints, yPoints);
//        }

        /**
         * target pos
         */
        public static void drawPole(Canvas canvas, double x, double y) {
                // Convert from mm to inches for display
                canvas.strokeCircle(x / 25.4, y / 25.4, POLE_RADIUS / 25.4);
        }

        /**
         * anglewrap to match localizer
         */
        public static double angleWrap(double radians) {
                return Localizer.angleWrap(radians);
        }

        // clueless

        public static void drawField() {
                final int[] mirror = { 1, -1 };
                Canvas canvas = TelemetryUtil.packet.fieldOverlay();
                canvas.setFill("#e0e0e0e0");
                canvas.fillRect(-72, -72, 144, 144); // background
                canvas.setStroke("#808080");
                canvas.strokeRect(-15, -24, 30, 48); // submersible
                for (int i : mirror) {
                        for (int j : mirror)
                                canvas.strokeLine(i * 24, j * 24, i * 36, 0); // ascent zone
                        canvas.strokeLine(i * -24, i * 24, i * 24, i * 24);
                }
                canvas.setStroke("#0000ff");
                for (int i : mirror) {
                        canvas.strokeLine(i * 48, i * 72, i * 72, i * 48); // net zone
                        canvas.strokeLine(i * -72, i * 60, i * -48, i * 60); // observation zone
                        canvas.strokeLine(i * -48, i * 60, i * -36, i * 72); // observation zone
                        canvas.strokeLine(i * -15, i * 24, i * 15, i * 24); // specimen rung
                        for (int j : mirror) {
                                for (int k = 0; k < 3; ++k) {
                                        canvas.strokeLine(i * j * (-49.5 - 10 * k), i * 24, i * j * (-49.5 - 10 * k),
                                                        i * 27.5); // preset
                                }
                                canvas.setStroke("#ffff00");
                        }
                        canvas.setStroke("#ff0000");
                }
                canvas.drawGrid(0, 0, 144, 144, 7, 7);
        }

        public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
                double[] xPoints = new double[poseHistory.size()];
                double[] yPoints = new double[poseHistory.size()];
                for (int i = 0; i < poseHistory.size(); i++) {
                        Pose2d pose = poseHistory.get(i);
                        xPoints[i] = pose.position.x;
                        yPoints[i] = pose.position.y;
                }
                canvas.strokePolyline(xPoints, yPoints);
        }

        public static void drawRobot(Canvas canvas, Pose2d pose, String color) {
                canvas.setStroke(color);
                canvas.strokeCircle(pose.position.x, pose.position.y, ROBOT_RADIUS);
                canvas.strokeCircle(pose.position.x, pose.position.y, 0.5);
                Pose2d v = new Pose2d(new Vector2d(Math.cos(pose.heading.toDouble()) * ROBOT_RADIUS, Math.sin(pose.heading.toDouble()) * ROBOT_RADIUS), pose.heading.toDouble());
                double x1 = pose.position.x + v.position.x / 2, y1 = pose.position.y + v.position.y / 2;
                double x2 = pose.position.x + v.position.x, y2 = pose.position.y + v.position.y;
                canvas.strokeLine(x1, y1, x2, y2);
        }

        public static void drawRobot(Canvas canvas, Pose2d pose, double ext, String color) {
                canvas.setStroke(color);
                double x = pose.position.x, y = pose.position.y;
                double s = Math.sin(pose.heading.real), c = Math.cos(pose.heading.toDouble());
                double re1 = 6, re2 = 9 + ext, rh = 4;
                double x1 = x + c * re1, y1 = y + s * re1;
                double x2 = x + c * re2, y2 = y + s * re2;
                double xh = x + c * rh, yh = y + s * rh;
                canvas.strokeCircle(x, y, ROBOT_RADIUS);
                canvas.strokeCircle(x, y, 0.5);
                canvas.strokeLine(x, y, xh, yh);
                canvas.strokeLine(x1, y1, x2, y2);
                canvas.strokeCircle(x2, y2, 0.5);
        }

        public static void drawPole(Canvas canvas, Pose2d pose) {
                canvas.strokeCircle(pose.position.x, pose.position.y, POLE_RADIUS);
        }
}