package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Optimizes hardware communication by managing bulk reads efficiently
 * This reduces the number of individual hardware calls which are expensive
 * operations
 */
public class BulkReadManager {
        private final List<LynxModule> allHubs;

        public BulkReadManager(HardwareMap hardwareMap) {
                allHubs = hardwareMap.getAll(LynxModule.class);

                // Set all hubs to MANUAL mode for maximum control
                for (LynxModule hub : allHubs) {
                        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
                }
        }

        /**
         * Call this once per loop iteration to update all sensor readings
         * This replaces multiple individual hardware calls with one bulk operation
         * PERFORMANCE OPTIMIZED: No timer - update every loop for maximum speed
         */
        public void updateBulkRead() {
                // Clear cache and trigger new bulk read every loop iteration
                for (LynxModule hub : allHubs) {
                        hub.clearBulkCache();
                }
        }

        /**
         * Force an immediate bulk read update
         * Use sparingly - only when you need the most recent data immediately
         */
        public void forceUpdate() {
                for (LynxModule hub : allHubs) {
                        hub.clearBulkCache();
                }
        }
}
