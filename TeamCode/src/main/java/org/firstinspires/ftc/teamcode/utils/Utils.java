package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;

public class Utils {
        public Utils() {
        }

        public static double minMaxClip(double value, double min, double max) {
                return Math.min(Math.max(min, value), max);
        }

        public static int minMaxClip(int value, int min, int max) {
                return Math.min(Math.max(min, value), max);
        }

        public static double headingClip(double value) {
                while (value >= Math.PI) {
                        value -= 2 * Math.PI;
                }
                while (value <= -Math.PI) {
                        value += 2 * Math.PI;
                }
                return value;
        }

        public static boolean withinThreshold(double value, double minThreshold, double maxThreshold) {
                return value > minThreshold && value < maxThreshold;
        }

        public static double kalmanFilter(double value1, double value2, double value2Weight) {
                return (value1 * (1.0 - value2Weight)) + (value2 * value2Weight);
        }

        public static double calculateDistanceBetweenPoints(Pose2d point1, Pose2d point2) {
                double deltaX = point1.position.x - point2.position.x;
                double deltaY = point1.position.y - point2.position.y;
                return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        }
}
