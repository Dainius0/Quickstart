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
    private static final double STRAFE_ROTATION_COMPENSATION = 0;

    // MOTOR CALIBRATION VALUES - Update these from calibration program
    // These compensate for differences in motor performance
    private static final double BR_COMPENSATION = 1;
    private static final double FR_COMPENSATION = 1;
    private static final double BL_COMPENSATION = 1;
    private static final double FL_COMPENSATION = 1;

    // Cache the last heading to avoid redundant Pinpoint updates
    private double cachedHeading = 0;
    private long lastHeadingUpdateTime = 0;
    private static final long HEADING_CACHE_MS = 10;

    public DriveTrain(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        if (resetYaw) {
            pinpoint.recalibrateIMU();
        }

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

        double botHeading = cachedHeading;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double compensatedRx = rx + (rotX * STRAFE_ROTATION_COMPENSATION);

        // Apply slow mode if trigger is pressed
        double speedMultiplier = 1.0 - (slowModeTrigger * 0.3);
        rotX *= speedMultiplier;
        rotY *= speedMultiplier;
        compensatedRx *= speedMultiplier;

        // Calculate base motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(compensatedRx), 1);
        double frontLeftPower = (rotY + rotX + compensatedRx) / denominator;
        double backLeftPower = (rotY - rotX + compensatedRx) / denominator;
        double frontRightPower = (rotY - rotX - compensatedRx) / denominator;
        double backRightPower = (rotY + rotX - compensatedRx) / denominator;

        // Apply motor compensation to correct for hardware differences
        frontLeftPower *= FL_COMPENSATION;
        backLeftPower *= BL_COMPENSATION;
        frontRightPower *= FR_COMPENSATION;
        backRightPower *= BR_COMPENSATION;

        // Renormalize after compensation to prevent power > 1.0
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void resetHeading() {
        pinpoint.update();
        Pose2D currentPos = pinpoint.getPosition();
        double currentX = currentPos.getX(DistanceUnit.MM);
        double currentY = currentPos.getY(DistanceUnit.MM);

        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, currentX, currentY, AngleUnit.DEGREES, 0));
        pinpoint.update();

        cachedHeading = 0;
        lastHeadingUpdateTime = System.currentTimeMillis();
    }

    public double getHeading() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastHeadingUpdateTime > HEADING_CACHE_MS) {
            pinpoint.update();
            cachedHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
            lastHeadingUpdateTime = currentTime;
        }

        return Math.toDegrees(cachedHeading);
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    /**
     * Get current motor compensation values for debugging
     */
    public String getCompensationValues() {
        return String.format("FL:%.4f BL:%.4f FR:%.4f BR:%.4f",
                FL_COMPENSATION, BL_COMPENSATION, FR_COMPENSATION, BR_COMPENSATION);
    }
}