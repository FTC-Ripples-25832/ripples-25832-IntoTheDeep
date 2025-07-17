package org.firstinspires.ftc.teamcode.utils.motion;

import org.firstinspires.ftc.teamcode.utils.PIDFController;

/**
 * A motion profile controller that integrates trapezoidal motion profiles
 * with PIDF control for smooth and precise position control.
 * 
 * This controller uses position, velocity, and acceleration feedforward
 * combined with position feedback to achieve optimal tracking performance.
 */
public class MotionProfileController {
        private final PIDFController pidfController;

        // Motion profile constraints
        private double maxVelocity;
        private double maxAcceleration;

        // Current motion profile
        private TrapezoidalMotionProfile currentProfile;
        private long profileStartTime;
        private boolean profileActive;

        // Feedforward gains
        private double kV; // Velocity feedforward gain
        private double kA; // Acceleration feedforward gain

        /**
         * Create a motion profile controller
         * 
         * @param pidfController  The PIDF controller to use for position feedback
         * @param maxVelocity     Maximum velocity for motion profiles (units/second)
         * @param maxAcceleration Maximum acceleration for motion profiles
         *                        (units/second²)
         * @param kV              Velocity feedforward gain
         * @param kA              Acceleration feedforward gain
         */
        public MotionProfileController(PIDFController pidfController,
                        double maxVelocity, double maxAcceleration,
                        double kV, double kA) {
                this.pidfController = pidfController;
                this.maxVelocity = Math.abs(maxVelocity);
                this.maxAcceleration = Math.abs(maxAcceleration);
                this.kV = kV;
                this.kA = kA;
                this.profileActive = false;
        }

        /**
         * Set a target position with motion profiling
         * 
         * @param currentPosition Current position
         * @param targetPosition  Target position
         */
        public void setTarget(double currentPosition, double targetPosition) {
                // Create new motion profile
                currentProfile = new TrapezoidalMotionProfile(
                                currentPosition, targetPosition, maxVelocity, maxAcceleration);

                // Start timing the profile
                profileStartTime = System.currentTimeMillis();
                profileActive = true;

                // Set the PIDF target to the final position
                pidfController.setDestination(targetPosition);
        }

        /**
         * Calculate the motor power using motion profiling and PIDF control
         * 
         * @param currentPosition Current measured position
         * @return Motor power (-1.0 to 1.0)
         */
        public double calculate(double currentPosition) {
                if (!profileActive) {
                        // No active profile, use regular PIDF control
                        return pidfController.calculate(currentPosition);
                }

                // Calculate elapsed time in seconds
                double elapsedTime = (System.currentTimeMillis() - profileStartTime) / 1000.0;

                // Get motion profile setpoints
                double targetPosition = currentProfile.getPosition(elapsedTime);
                double targetVelocity = currentProfile.getVelocity(elapsedTime);
                double targetAcceleration = currentProfile.getAcceleration(elapsedTime);

                // Check if profile is complete
                if (currentProfile.isFinished(elapsedTime)) {
                        profileActive = false;
                        // Switch to pure PIDF control for final positioning
                        return pidfController.calculate(currentPosition);
                }

                // Calculate position error
                double positionError = targetPosition - currentPosition;

                // Calculate PIDF output with position feedback
                double pidOutput = pidfController.kp * positionError;

                // Add integral term if needed (be careful with motion profiling)
                // We typically don't use integral during motion profiling to avoid windup

                // Add derivative term based on position error rate
                // This is handled by the motion profile's velocity feedforward

                // Feedforward terms
                double velocityFeedforward = kV * targetVelocity;
                double accelerationFeedforward = kA * targetAcceleration;

                // Combine all terms
                double output = pidOutput + velocityFeedforward + accelerationFeedforward;

                // Clamp output to motor limits
                return Math.max(-1.0, Math.min(1.0, output));
        }

        /**
         * Check if a motion profile is currently active
         * 
         * @return True if motion profile is running
         */
        public boolean isProfileActive() {
                return profileActive;
        }

        /**
         * Get the current target position from the motion profile
         * 
         * @return Current target position, or Double.NaN if no profile is active
         */
        public double getCurrentTarget() {
                if (!profileActive || currentProfile == null) {
                        return Double.NaN;
                }

                double elapsedTime = (System.currentTimeMillis() - profileStartTime) / 1000.0;
                return currentProfile.getPosition(elapsedTime);
        }

        /**
         * Get the current target velocity from the motion profile
         * 
         * @return Current target velocity, or 0 if no profile is active
         */
        public double getCurrentTargetVelocity() {
                if (!profileActive || currentProfile == null) {
                        return 0;
                }

                double elapsedTime = (System.currentTimeMillis() - profileStartTime) / 1000.0;
                return currentProfile.getVelocity(elapsedTime);
        }

        /**
         * Stop the current motion profile
         */
        public void stopProfile() {
                profileActive = false;
        }

        /**
         * Update motion profile constraints
         * 
         * @param maxVelocity     Maximum velocity (units/second)
         * @param maxAcceleration Maximum acceleration (units/second²)
         */
        public void setConstraints(double maxVelocity, double maxAcceleration) {
                this.maxVelocity = Math.abs(maxVelocity);
                this.maxAcceleration = Math.abs(maxAcceleration);
        }

        /**
         * Update feedforward gains
         * 
         * @param kV Velocity feedforward gain
         * @param kA Acceleration feedforward gain
         */
        public void setFeedforwardGains(double kV, double kA) {
                this.kV = kV;
                this.kA = kA;
        }

        /**
         * Get the underlying PIDF controller for direct access
         * 
         * @return The PIDF controller
         */
        public PIDFController getPIDFController() {
                return pidfController;
        }

        /**
         * Get the estimated time remaining for the current profile
         * 
         * @return Time remaining in seconds, or 0 if no profile is active
         */
        public double getTimeRemaining() {
                if (!profileActive || currentProfile == null) {
                        return 0;
                }

                double elapsedTime = (System.currentTimeMillis() - profileStartTime) / 1000.0;
                return Math.max(0, currentProfile.getTotalTime() - elapsedTime);
        }
}
