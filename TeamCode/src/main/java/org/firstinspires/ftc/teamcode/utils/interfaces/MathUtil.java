package org.firstinspires.ftc.teamcode.utils.interfaces;

import com.acmerobotics.roadrunner.Pose2d;

public class MathUtil {
        public static int minMaxClip(int val, int min, int max) {
                return Math.max(min, Math.min(max, val));
        }

        public static double minMaxClip(double val, double min, double max) {
                return Math.max(min, Math.min(max, val));
        }

        public static double distance(Pose2d a, Pose2d b) {
                return Math.hypot(a.position.x - b.position.x, a.position.y - b.position.y);
        }

        public static double errorX(Pose2d a, Pose2d b) {
                return Math.abs(a.position.x - b.position.x);
        }

        public static double errorY(Pose2d a, Pose2d b) {
                return Math.abs(a.position.y - b.position.y);
        }
        // Add more helpers as needed
}