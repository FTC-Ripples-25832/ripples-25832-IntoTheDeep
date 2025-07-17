package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;
import java.util.HashSet;
import java.util.Set;

/**
 * Optimized wait command using ElapsedTime instead of
 * System.currentTimeMillis()
 * This reduces the overhead of system calls in tight loops
 */
public class OptimizedWaitCommand implements Command {
        private final double seconds;
        private final ElapsedTime timer = new ElapsedTime();

        public OptimizedWaitCommand(double seconds) {
                this.seconds = seconds;
        }

        @Override
        public void initialize() {
                timer.reset();
        }

        @Override
        public void execute(TelemetryPacket packet) {
                // Nothing to do while waiting
        }

        @Override
        public boolean isFinished() {
                return timer.seconds() >= seconds;
        }

        @Override
        public void end(boolean interrupted) {
                // Nothing to clean up
        }

        @Override
        public Set<SubsystemBase> getRequirements() {
                return new HashSet<>();
        }

        @Override
        public long getTimeout() {
                return (long) (seconds * 1000) + 1000; // Add 1 second buffer
        }
}
