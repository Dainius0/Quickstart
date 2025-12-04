package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DistanceSensorSystem {
    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;
    private static final double DISTANCE_THRESHOLD = 10; // cm - ball detected when closer than this

    public void init(HardwareMap hwMap) {
        distanceSensor1 = hwMap.get(DistanceSensor.class, "distance_sensor1");
        distanceSensor2 = hwMap.get(DistanceSensor.class, "distance_sensor2");
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

    public boolean detectsBall(double distance) {
        return distance < DISTANCE_THRESHOLD;
    }

    public boolean bothDetectBall(Telemetry telemetry) {
        double distance1 = getDistance1(telemetry);
        double distance2 = getDistance2(telemetry);
        return detectsBall(distance1) && detectsBall(distance2);
    }
}