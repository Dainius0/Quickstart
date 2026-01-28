package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * DistanceSensorSystem - Manages three distance sensors to detect game balls
 *
 * This system monitors three distance sensors positioned to detect when balls
 * are loaded in the robot. When all three sensors detect balls simultaneously,
 * it provides haptic feedback to the driver via controller vibration.
 */
public class DistanceSensorSystem {
    // Hardware components - three distance sensors for ball detection
    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;
    private DistanceSensor distanceSensor3;

    // Distance threshold in centimeters - objects closer than this are considered "detected"
    // Adjust this value based on your sensor positioning and ball size
    private static final double DISTANCE_THRESHOLD = 5; // cm - ball detected when closer than this

    // State tracking for edge detection - prevents continuous vibration
    // This flag remembers if three balls were detected in the previous loop iteration
    private boolean wasThreeBallsDetected = false;

    /**
     * Initialize all three distance sensors from the hardware map
     *
     * @param hwMap The robot's hardware map containing sensor configurations
     * @throws IllegalArgumentException if sensors are not found in configuration
     */
    public void init(HardwareMap hwMap) {
        // Retrieve sensors by their configured names in the robot configuration
        distanceSensor1 = hwMap.get(DistanceSensor.class, "distance_sensor1");
        distanceSensor2 = hwMap.get(DistanceSensor.class, "distance_sensor2");
        distanceSensor3 = hwMap.get(DistanceSensor.class, "distance_sensor3");
    }

    /**
     * Get the current distance reading from sensor 1
     *
     * @return Distance in centimeters
     */
    public double getDistance1() {
        return distanceSensor1.getDistance(DistanceUnit.CM);
    }

    /**
     * Get the current distance reading from sensor 2
     *
     * @return Distance in centimeters
     */
    public double getDistance2() {
        return distanceSensor2.getDistance(DistanceUnit.CM);
    }

    /**
     * Get the current distance reading from sensor 3
     *
     * @return Distance in centimeters
     */
    public double getDistance3() {
        return distanceSensor3.getDistance(DistanceUnit.CM);
    }

    /**
     * Determine if a given distance reading indicates a ball is present
     *
     * @param distance The distance reading to check (in cm)
     * @return true if distance is below threshold (ball detected), false otherwise
     */
    public boolean detectsBall(double distance) {
        return distance < DISTANCE_THRESHOLD;
    }

    /**
     * Check if both sensor 1 and sensor 2 detect balls
     * Legacy method - consider using allThreeDetectBalls() instead
     *
     * @return true if both sensors 1 and 2 detect balls
     */
    public boolean bothDetectBall() {
        double distance1 = getDistance1();
        double distance2 = getDistance2();
        return detectsBall(distance1) && detectsBall(distance2);
    }

    /**
     * Monitor all three sensors and provide haptic feedback when fully loaded
     *
     * This method should be called repeatedly in your main teleop loop.
     * It checks if all three sensors detect balls and vibrates the controller
     * ONLY on the transition from "not all detected" to "all detected".
     * This edge detection prevents annoying continuous vibration.
     *
     * Usage in teleop:
     *   if (sensorsAvailable && loopCounter % DISTANCE_CHECK_INTERVAL == 0) {
     *       distanceSensors.checkThreeBallsAndVibrate(gamepad1);
     *   }
     *
     * @param gamepad The gamepad controller to vibrate (typically gamepad1)
     */
    public void checkThreeBallsAndVibrate(Gamepad gamepad) {
        // Read current distances from all three sensors
        double distance1 = getDistance1();
        double distance2 = getDistance2();
        double distance3 = getDistance3();

        // Check if ALL three sensors currently detect balls
        boolean allThreeDetected = detectsBall(distance1) &&
                detectsBall(distance2) &&
                detectsBall(distance3);

        // Edge detection: Only vibrate when transitioning from false→true
        // This prevents the controller from vibrating continuously while all
        // three balls remain loaded. Vibration only happens once when the
        // third ball is first detected.
        if (allThreeDetected && !wasThreeBallsDetected) {
            // Vibrate both motors at full intensity (1.0) for 500 milliseconds
            // This gives the driver clear tactile feedback that robot is fully loaded
            gamepad.rumble(1.0, 1.0, 500);
        }

        // Store current state for next loop iteration's edge detection
        wasThreeBallsDetected = allThreeDetected;
    }

    /**
     * Check if all three sensors detect balls without triggering vibration
     *
     * Use this method when you only need to read the state for telemetry
     * or decision-making logic, without providing haptic feedback.
     *
     * @return true if all three sensors detect balls
     */
    public boolean allThreeDetectBalls() {
        double distance1 = getDistance1();
        double distance2 = getDistance2();
        double distance3 = getDistance3();

        return detectsBall(distance1) &&
                detectsBall(distance2) &&
                detectsBall(distance3);
    }

    /**
     * Manually reset the vibration state tracker
     *
     * Call this method when you want the next "all three detected" event to
     * trigger vibration again. This is useful after outtaking balls - it
     * resets the system so that when you collect three balls again, you'll
     * get vibration feedback.
     *
     * Example usage: Call this when the driver presses dpad_down to outtake
     */
    public void resetVibrationState() {
        wasThreeBallsDetected = false;
    }
}