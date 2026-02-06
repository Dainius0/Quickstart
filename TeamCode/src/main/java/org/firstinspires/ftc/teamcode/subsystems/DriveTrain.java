package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveTrain {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private GoBildaPinpointDriver pinpoint;

    // TUNING CONSTANT: Compensate for center of mass being toward back
    // When strafing, this adds counter-rotation to prevent the robot from spinning
    // Positive values = add clockwise rotation when strafing right
    // Negative values = add counter-clockwise rotation when strafing right
    // Start with 0.15-0.25 and adjust based on testing
    private static final double STRAFE_ROTATION_COMPENSATION = 0.05;

    // Cache the last heading to avoid redundant Pinpoint updates
    private double cachedHeading = 0;
    private long lastHeadingUpdateTime = 0;
    private static final long HEADING_CACHE_MS = 10; // Cache heading for 10ms

    // Original constructor - keeps existing behavior
    public DriveTrain(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    // New constructor with option to preserve heading
    public DriveTrain(HardwareMap hardwareMap, boolean resetYaw) {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Get Pinpoint - it's already configured by Pedro Pathing
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Reset IMU if requested
        if (resetYaw) {
            pinpoint.recalibrateIMU();
        }

        // Initialize cached heading
        pinpoint.update();
        cachedHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
        lastHeadingUpdateTime = System.currentTimeMillis();
    }

    public void drive(double x, double y, double rx, double slowModeTrigger) {
        // Update heading from cache or refresh if needed
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastHeadingUpdateTime > HEADING_CACHE_MS) {
            pinpoint.update();
            cachedHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
            lastHeadingUpdateTime = currentTime;
        }

        // Use cached heading for field-centric calculation
        double botHeading = cachedHeading;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        // Apply strafe compensation - add counter-rotation based on strafe direction
        // This counteracts unwanted rotation caused by center of mass being toward back
        double compensatedRx = rx + (rotX * STRAFE_ROTATION_COMPENSATION);

        // Apply slow mode if trigger is pressed
        double speedMultiplier = 1.0 - (slowModeTrigger * 0.3);
        rotX *= speedMultiplier;
        rotY *= speedMultiplier;
        compensatedRx *= speedMultiplier;

        // Calculate motor powers with compensation
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(compensatedRx), 1);
        double frontLeftPower = (rotY + rotX + compensatedRx) / denominator;
        double backLeftPower = (rotY - rotX + compensatedRx) / denominator;
        double frontRightPower = (rotY - rotX - compensatedRx) / denominator;
        double backRightPower = (rotY + rotX - compensatedRx) / denominator;

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void resetHeading() {
        // Force an update first to get current position
        pinpoint.update();

        // Get current X and Y coordinates
        Pose2D currentPos = pinpoint.getPosition();
        double currentX = currentPos.getX(DistanceUnit.MM);
        double currentY = currentPos.getY(DistanceUnit.MM);

        // Reset position with heading set to 0
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, currentX, currentY, AngleUnit.DEGREES, 0));

        // Force another update to apply the change
        pinpoint.update();

        // Update cached heading
        cachedHeading = 0;
        lastHeadingUpdateTime = System.currentTimeMillis();
    }

    public double getHeading() {
        // Update cache if needed
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastHeadingUpdateTime > HEADING_CACHE_MS) {
            pinpoint.update();
            cachedHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
            lastHeadingUpdateTime = currentTime;
        }

        // Convert cached heading from radians to degrees for return
        return Math.toDegrees(cachedHeading);
    }

    /**
     * Get the Pinpoint driver for advanced usage
     */
    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }
}