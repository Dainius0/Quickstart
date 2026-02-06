package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DistanceSensorSystem {
    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;
    private DistanceSensor distanceSensor3;
    private static final double DISTANCE_THRESHOLD = 5; // cm - ball detected when closer than this

    // Vibration state tracking
    private boolean wasThreeBallsDetected = false;

    public void init(HardwareMap hwMap) {
        distanceSensor1 = hwMap.get(DistanceSensor.class, "distance_sensor1");
        distanceSensor2 = hwMap.get(DistanceSensor.class, "distance_sensor2");
        distanceSensor3 = hwMap.get(DistanceSensor.class, "distance_sensor3");
    }

    public double getDistance1() {
        return distanceSensor1.getDistance(DistanceUnit.CM);
    }

    public double getDistance2() {
        return distanceSensor2.getDistance(DistanceUnit.CM);
    }

    public double getDistance3() {
        return distanceSensor3.getDistance(DistanceUnit.CM);
    }

    public boolean detectsBall(double distance) {
        return distance < DISTANCE_THRESHOLD;
    }

    public boolean bothDetectBall() {
        double distance1 = getDistance1();
        double distance2 = getDistance2();
        return detectsBall(distance1) && detectsBall(distance2);
    }

    /**
     * Check if all three sensors detect balls and vibrate controller when this happens
     * Call this method in your main loop to enable vibration feedback
     *
     * @param gamepad Controller to vibrate (gamepad1 or gamepad2)
     */
    public void checkThreeBallsAndVibrate(Gamepad gamepad) {
        double distance1 = getDistance1();
        double distance2 = getDistance2();
        double distance3 = getDistance3();

        boolean allThreeDetected = detectsBall(distance1) &&
                detectsBall(distance2) &&
                detectsBall(distance3);

        // Only vibrate on the transition from false to true (edge detection)
        // This prevents continuous vibration
        if (allThreeDetected && !wasThreeBallsDetected) {
            // Vibrate for 500ms with full rumble on both motors
            gamepad.rumble(1.0, 1.0, 500);
        }

        // Update state for next iteration
        wasThreeBallsDetected = allThreeDetected;
    }

    /**
     * Check if all three sensors detect balls (without vibration)
     * Use this if you just want to check the state without triggering vibration
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
     * Reset the vibration state - call this if you want to manually reset
     * so the next detection triggers vibration again
     */
    public void resetVibrationState() {
        wasThreeBallsDetected = false;
    }
}