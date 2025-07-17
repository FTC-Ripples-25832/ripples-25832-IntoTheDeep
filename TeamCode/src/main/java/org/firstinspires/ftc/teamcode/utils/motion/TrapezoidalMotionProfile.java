package org.firstinspires.ftc.teamcode.utils.motion;

/**
 * A trapezoidal motion profile implementation for smooth position control.
 * This creates a velocity profile that looks like a trapezoid, with controlled
 * acceleration, cruise, and deceleration phases to prevent slip and provide
 * smooth movement.
 * 
 * Based on the motion profiling concepts from CTRL ALT FTC documentation.
 */
public class TrapezoidalMotionProfile {
        private final double maxAcceleration;
        private final double maxVelocity;
        private final double distance;
        private final double startPosition;

        // Calculated profile parameters
        private final double accelerationTime;
        private final double accelerationDistance;
        private final double cruiseTime;
        private final double cruiseDistance;
        private final double decelerationTime;
        private final double totalTime;
        private final double actualMaxVelocity;

        /**
         * Create a trapezoidal motion profile
         * 
         * @param startPosition   Starting position
         * @param endPosition     Target position
         * @param maxVelocity     Maximum velocity (units/second)
         * @param maxAcceleration Maximum acceleration (units/second²)
         */
        public TrapezoidalMotionProfile(double startPosition, double endPosition,
                        double maxVelocity, double maxAcceleration) {
                this.maxAcceleration = Math.abs(maxAcceleration);
                this.maxVelocity = Math.abs(maxVelocity);
                this.startPosition = startPosition;
                this.distance = Math.abs(endPosition - startPosition);

                // Calculate the time it takes to accelerate to max velocity
                double timeToMaxVel = this.maxVelocity / this.maxAcceleration;

                // Calculate distance covered during acceleration to max velocity
                double distanceToMaxVel = 0.5 * this.maxAcceleration * timeToMaxVel * timeToMaxVel;

                // Check if we can reach max velocity in the given distance
                if (2 * distanceToMaxVel <= distance) {
                        // We can reach max velocity - normal trapezoidal profile
                        this.accelerationTime = timeToMaxVel;
                        this.accelerationDistance = distanceToMaxVel;
                        this.actualMaxVelocity = this.maxVelocity;

                        // Calculate cruise phase
                        this.cruiseDistance = distance - 2 * accelerationDistance;
                        this.cruiseTime = cruiseDistance / this.actualMaxVelocity;

                        this.decelerationTime = accelerationTime;
                } else {
                        // We can't reach max velocity - triangular profile
                        this.accelerationTime = Math.sqrt(distance / this.maxAcceleration);
                        this.accelerationDistance = distance / 2;
                        this.actualMaxVelocity = this.maxAcceleration * accelerationTime;

                        // No cruise phase
                        this.cruiseDistance = 0;
                        this.cruiseTime = 0;

                        this.decelerationTime = accelerationTime;
                }

                this.totalTime = accelerationTime + cruiseTime + decelerationTime;
        }

        /**
         * Get the target position at a given time
         * 
         * @param elapsedTime Time since profile started (seconds)
         * @return Target position
         */
        public double getPosition(double elapsedTime) {
                if (elapsedTime <= 0) {
                        return startPosition;
                }

                if (elapsedTime >= totalTime) {
                        return startPosition + distance;
                }

                double position;

                if (elapsedTime <= accelerationTime) {
                        // Acceleration phase: x = x₀ + ½at²
                        position = 0.5 * maxAcceleration * elapsedTime * elapsedTime;
                } else if (elapsedTime <= accelerationTime + cruiseTime) {
                        // Cruise phase: x = x₀ + v₀t (constant velocity)
                        double cruiseElapsedTime = elapsedTime - accelerationTime;
                        position = accelerationDistance + actualMaxVelocity * cruiseElapsedTime;
                } else {
                        // Deceleration phase: x = x₀ + v₀t - ½at²
                        double decelerationElapsedTime = elapsedTime - accelerationTime - cruiseTime;
                        position = accelerationDistance + cruiseDistance
                                        + actualMaxVelocity * decelerationElapsedTime
                                        - 0.5 * maxAcceleration * decelerationElapsedTime * decelerationElapsedTime;
                }

                return startPosition + position;
        }

        /**
         * Get the target velocity at a given time
         * 
         * @param elapsedTime Time since profile started (seconds)
         * @return Target velocity
         */
        public double getVelocity(double elapsedTime) {
                if (elapsedTime <= 0 || elapsedTime >= totalTime) {
                        return 0;
                }

                if (elapsedTime <= accelerationTime) {
                        // Acceleration phase: v = at
                        return maxAcceleration * elapsedTime;
                } else if (elapsedTime <= accelerationTime + cruiseTime) {
                        // Cruise phase: constant velocity
                        return actualMaxVelocity;
                } else {
                        // Deceleration phase: v = v₀ - at
                        double decelerationElapsedTime = elapsedTime - accelerationTime - cruiseTime;
                        return actualMaxVelocity - maxAcceleration * decelerationElapsedTime;
                }
        }

        /**
         * Get the target acceleration at a given time
         * 
         * @param elapsedTime Time since profile started (seconds)
         * @return Target acceleration
         */
        public double getAcceleration(double elapsedTime) {
                if (elapsedTime <= 0 || elapsedTime >= totalTime) {
                        return 0;
                }

                if (elapsedTime <= accelerationTime) {
                        // Acceleration phase
                        return maxAcceleration;
                } else if (elapsedTime <= accelerationTime + cruiseTime) {
                        // Cruise phase
                        return 0;
                } else {
                        // Deceleration phase
                        return -maxAcceleration;
                }
        }

        /**
         * Check if the profile is complete
         * 
         * @param elapsedTime Time since profile started (seconds)
         * @return True if profile is complete
         */
        public boolean isFinished(double elapsedTime) {
                return elapsedTime >= totalTime;
        }

        /**
         * Get the total time for the profile
         * 
         * @return Total time in seconds
         */
        public double getTotalTime() {
                return totalTime;
        }

        /**
         * Get the actual maximum velocity reached
         * 
         * @return Maximum velocity in units/second
         */
        public double getActualMaxVelocity() {
                return actualMaxVelocity;
        }

        /**
         * Get the total distance of the profile
         * 
         * @return Total distance
         */
        public double getDistance() {
                return distance;
        }
}
