package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DriveTrain {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private GoBildaPinpointDriver pinpoint;
    private IMU imu;

    private static final double STRAFE_ROTATION_COMPENSATION = 0.05;

    // Control Hub is mounted with USB port facing UP
    // Logo direction depends on which face is pointing forward - set to FORWARD as best guess
    // If field-centric driving feels rotated 90°, try changing LOGO_DIRECTION to LEFT or RIGHT
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    private double cachedHeadingDegrees = 0;

    public DriveTrain(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    public DriveTrain(HardwareMap hardwareMap, boolean resetYaw) {
        frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor   = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor  = hardwareMap.dcMotor.get("backRightMotor");

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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION)
        ));

        if (resetYaw) {
            imu.resetYaw();
        }

        cachedHeadingDegrees = 0;
    }

    /**
     * Call this ONCE at the top of every loop iteration.
     */
    public void update() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        cachedHeadingDegrees = angles.getYaw(AngleUnit.DEGREES);
    }

    public void drive(double x, double y, double rx, double slowModeTrigger) {
        double botHeadingRad = Math.toRadians(cachedHeadingDegrees);

        double rotX = x * Math.cos(-botHeadingRad) - y * Math.sin(-botHeadingRad);
        double rotY = x * Math.sin(-botHeadingRad) + y * Math.cos(-botHeadingRad);

        rotX = rotX * 1.1;

        double compensatedRx = rx + (rotX * STRAFE_ROTATION_COMPENSATION);

        double speedMultiplier = 1.0 - (slowModeTrigger * 0.3);
        rotX *= speedMultiplier;
        rotY *= speedMultiplier;
        compensatedRx *= speedMultiplier;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(compensatedRx), 1);
        frontLeftMotor.setPower((rotY + rotX + compensatedRx) / denominator);
        backLeftMotor.setPower((rotY - rotX + compensatedRx) / denominator);
        frontRightMotor.setPower((rotY - rotX - compensatedRx) / denominator);
        backRightMotor.setPower((rotY + rotX - compensatedRx) / denominator);
    }

    public void resetHeading() {
        imu.resetYaw();
        cachedHeadingDegrees = 0;

        pinpoint.update();
        Pose2D currentPos = pinpoint.getPosition();
        double currentX = currentPos.getX(DistanceUnit.MM);
        double currentY = currentPos.getY(DistanceUnit.MM);
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, currentX, currentY, AngleUnit.DEGREES, 0));
    }

    public double getHeading() {
        return cachedHeadingDegrees;
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }
}