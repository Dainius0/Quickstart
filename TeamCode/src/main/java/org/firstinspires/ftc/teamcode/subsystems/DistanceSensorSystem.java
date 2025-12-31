package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public double getDistance1(Telemetry telemetry) {
        double distance = distanceSensor1.getDistance(DistanceUnit.CM);
        telemetry.addData("Sensor 1 Distance (cm)", String.format("%.2f", distance));
        return distance;
    }

    public double getDistance2(Telemetry telemetry) {
        double distance = distanceSensor2.getDistance(DistanceUnit.CM);
        telemetry.addData("Sensor 2 Distance (cm)", String.format("%.2f", distance));
        return distance;
    }

    public double getDistance3(Telemetry telemetry) {
        double distance = distanceSensor3.getDistance(DistanceUnit.CM);
        telemetry.addData("Sensor 3 Distance (cm)", String.format("%.2f", distance));
        return distance;
    }

    public boolean detectsBall(double distance) {
        return distance < DISTANCE_THRESHOLD;
    }

    public boolean bothDetectBall(Telemetry telemetry) {
        double distance1 = getDistance1(telemetry);
        double distance2 = getDistance2(telemetry);
        return detectsBall(distance1) && detectsBall(distance2);
    }

    /**
     * Check if all three sensors detect balls and vibrate controller when this happens
     * Call this method in your main loop to enable vibration feedback
     *
     * @param telemetry Telemetry object for displaying status
     * @param gamepad Controller to vibrate (gamepad1 or gamepad2)
     */
    public void checkThreeBallsAndVibrate(Telemetry telemetry, Gamepad gamepad) {
        double distance1 = getDistance1(telemetry);
        double distance2 = getDistance2(telemetry);
        double distance3 = getDistance3(telemetry);

        boolean allThreeDetected = detectsBall(distance1) &&
                detectsBall(distance2) &&
                detectsBall(distance3);

        // Only vibrate on the transition from false to true (edge detection)
        // This prevents continuous vibration
        if (allThreeDetected && !wasThreeBallsDetected) {
            // Vibrate for 500ms with full rumble on both motors
            gamepad.rumble(1.0, 1.0, 500);
            telemetry.addLine(">>> THREE BALLS DETECTED! <<<");
        }

        // Update state for next iteration
        wasThreeBallsDetected = allThreeDetected;

        telemetry.addData("Three Balls Loaded", allThreeDetected ? "YES" : "NO");
    }

    /**
     * Check if all three sensors detect balls (without vibration)
     * Use this if you just want to check the state without triggering vibration
     *
     * @param telemetry Telemetry object for displaying status
     * @return true if all three sensors detect balls
     */
    public boolean allThreeDetectBalls(Telemetry telemetry) {
        double distance1 = getDistance1(telemetry);
        double distance2 = getDistance2(telemetry);
        double distance3 = getDistance3(telemetry);

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