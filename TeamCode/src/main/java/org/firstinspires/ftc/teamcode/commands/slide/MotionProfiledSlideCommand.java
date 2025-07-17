package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

/**
 * Command to move slides to a target position using motion profiling for smooth
 * movement.
 * This command provides controlled acceleration and deceleration to prevent
 * slip and
 * improve precision during autonomous operations.
 */
public class MotionProfiledSlideCommand extends CommandBase {
        private final LowerSlide lowerSlide;
        private final UpperSlide upperSlide;
        private final Double lowerTargetCM;
        private final Double upperTargetCM;
        private final double tolerance;

        /**
         * Create a motion profiled slide command for both slides
         * 
         * @param lowerSlide    Lower slide subsystem
         * @param upperSlide    Upper slide subsystem
         * @param lowerTargetCM Target position for lower slide in centimeters (null to
         *                      not move)
         * @param upperTargetCM Target position for upper slide in centimeters (null to
         *                      not move)
         * @param tolerance     Position tolerance in centimeters for completion
         */
        public MotionProfiledSlideCommand(LowerSlide lowerSlide, UpperSlide upperSlide,
                        Double lowerTargetCM, Double upperTargetCM, double tolerance) {
                this.lowerSlide = lowerSlide;
                this.upperSlide = upperSlide;
                this.lowerTargetCM = lowerTargetCM;
                this.upperTargetCM = upperTargetCM;
                this.tolerance = tolerance;

                if (lowerSlide != null) {
                        addRequirement(lowerSlide);
                }
                if (upperSlide != null) {
                        addRequirement(upperSlide);
                }
        }

        /**
         * Create a motion profiled slide command for both slides with default tolerance
         */
        public MotionProfiledSlideCommand(LowerSlide lowerSlide, UpperSlide upperSlide,
                        Double lowerTargetCM, Double upperTargetCM) {
                this(lowerSlide, upperSlide, lowerTargetCM, upperTargetCM, 1.0); // 1cm tolerance
        }

        /**
         * Create a motion profiled command for lower slide only
         */
        public static MotionProfiledSlideCommand lowerSlideOnly(LowerSlide lowerSlide, double targetCM) {
                return new MotionProfiledSlideCommand(lowerSlide, null, targetCM, null, 1.0);
        }

        /**
         * Create a motion profiled command for upper slide only
         */
        public static MotionProfiledSlideCommand upperSlideOnly(UpperSlide upperSlide, double targetCM) {
                return new MotionProfiledSlideCommand(null, upperSlide, null, targetCM, 1.0);
        }

        @Override
        public void initialize() {
                // Start motion profiles for the specified slides
                if (lowerTargetCM != null && lowerSlide != null) {
                        lowerSlide.setMotionProfileEnabled(true);
                        lowerSlide.setPositionCMWithProfile(lowerTargetCM);
                }

                if (upperTargetCM != null && upperSlide != null) {
                        upperSlide.setMotionProfileEnabled(true);
                        upperSlide.setPositionCMWithProfile(upperTargetCM);
                }
        }

        @Override
        public void execute(TelemetryPacket packet) {
                // The motion profiling is handled by the slide updatePID() methods
                // which are called continuously by the slide update commands
                super.execute(packet);
        }

        @Override
        public boolean isFinished() {
                boolean lowerFinished = true;
                boolean upperFinished = true;

                // Check lower slide completion
                if (lowerTargetCM != null && lowerSlide != null) {
                        if (lowerSlide.isMotionProfileActive()) {
                                // Still running motion profile
                                lowerFinished = false;
                        } else {
                                // Motion profile complete, check final position
                                double error = Math.abs(lowerSlide.getCurrentPositionCM() - lowerTargetCM);
                                lowerFinished = error <= tolerance;
                        }
                }

                // Check upper slide completion
                if (upperTargetCM != null && upperSlide != null) {
                        if (upperSlide.isMotionProfileActive()) {
                                // Still running motion profile
                                upperFinished = false;
                        } else {
                                // Motion profile complete, check final position
                                double currentPos = upperSlide.getCurrentPosition();
                                double targetTicks = upperTargetCM * ((28.0 * 5.23) / (34 * 3.14)) * 10; // Convert cm
                                                                                                         // to ticks
                                double error = Math.abs(currentPos - targetTicks);
                                double errorCM = error / (((28.0 * 5.23) / (34 * 3.14)) * 10);
                                upperFinished = errorCM <= tolerance;
                        }
                }

                return lowerFinished && upperFinished;
        }

        @Override
        public void end(boolean interrupted) {
                // If interrupted, stop motion profiles
                if (interrupted) {
                        if (lowerSlide != null) {
                                lowerSlide.setMotionProfileEnabled(false);
                        }
                        if (upperSlide != null) {
                                upperSlide.setMotionProfileEnabled(false);
                        }
                }
        }

        /**
         * Get the estimated time remaining for the longest running motion profile
         */
        public double getTimeRemaining() {
                double maxTime = 0;

                if (lowerSlide != null && lowerSlide.isMotionProfileActive()) {
                        maxTime = Math.max(maxTime, lowerSlide.getMotionProfileTimeRemaining());
                }

                if (upperSlide != null && upperSlide.isMotionProfileActive()) {
                        maxTime = Math.max(maxTime, upperSlide.getMotionProfileTimeRemaining());
                }

                return maxTime;
        }

        /**
         * Check if any motion profiles are currently running
         */
        public boolean isProfileActive() {
                boolean lowerActive = lowerSlide != null && lowerSlide.isMotionProfileActive();
                boolean upperActive = upperSlide != null && upperSlide.isMotionProfileActive();
                return lowerActive || upperActive;
        }
}
