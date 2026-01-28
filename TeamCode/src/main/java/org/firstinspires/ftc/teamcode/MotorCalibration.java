package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Motor Calibration", group = "Calibration")
public class MotorCalibration extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private GoBildaPinpointDriver pinpoint;

    private static final double TEST_POWER = 0.5;
    private static final long TEST_DURATION_MS = 1000;

    // Calibration results
    private double flCompensation = 1.0;
    private double blCompensation = 1.0;
    private double frCompensation = 1.0;
    private double brCompensation = 1.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
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
        pinpoint.recalibrateIMU();

        telemetry.addLine("Motor Calibration Ready");
        telemetry.addLine("");
        telemetry.addLine("This will test each movement direction:");
        telemetry.addLine("1. Forward");
        telemetry.addLine("2. Backward");
        telemetry.addLine("3. Strafe Right");
        telemetry.addLine("4. Strafe Left");
        telemetry.addLine("");
        telemetry.addLine("Keep robot on flat surface with space to move");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.clear();

            // Test 1: Forward movement
            telemetry.addLine("Test 1/4: Forward Movement");
            telemetry.update();
            sleep(1000);
            double forwardRotation = testMovement(0, 1, 0);

            // Test 2: Backward movement
            telemetry.addLine("Test 2/4: Backward Movement");
            telemetry.update();
            sleep(1000);
            double backwardRotation = testMovement(0, -1, 0);

            // Test 3: Strafe Right
            telemetry.addLine("Test 3/4: Strafe Right");
            telemetry.update();
            sleep(1000);
            double strafeRightRotation = testMovement(1, 0, 0);

            // Test 4: Strafe Left
            telemetry.addLine("Test 4/4: Strafe Left");
            telemetry.update();
            sleep(1000);
            double strafeLeftRotation = testMovement(-1, 0, 0);

            // Calculate compensation values
            calculateCompensation(forwardRotation, backwardRotation,
                    strafeRightRotation, strafeLeftRotation);

            // Display results
            displayResults();
        }
    }

    private double testMovement(double x, double y, double rx) {
        // Reset position
        pinpoint.recalibrateIMU();
        sleep(500);
        pinpoint.update();

        double startHeading = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);

        // Apply power for test duration
        double rotX = x;
        double rotY = y;

        if (x != 0) rotX = rotX * 1.1; // Apply existing strafe compensation

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator * TEST_POWER;
        double backLeftPower = (rotY - rotX + rx) / denominator * TEST_POWER;
        double frontRightPower = (rotY - rotX - rx) / denominator * TEST_POWER;
        double backRightPower = (rotY + rotX - rx) / denominator * TEST_POWER;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        sleep(TEST_DURATION_MS);

        // Stop motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        sleep(500);

        // Measure final heading
        pinpoint.update();
        double endHeading = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);

        double rotationError = normalizeAngle(endHeading - startHeading);

        telemetry.addData("Rotation Error", "%.2f degrees", rotationError);
        telemetry.update();
        sleep(1000);

        return rotationError;
    }

    private void calculateCompensation(double forward, double backward,
                                       double strafeRight, double strafeLeft) {
        // Forward: FL and FR should be equal, BL and BR should be equal
        // Positive rotation = right side weaker
        // Negative rotation = left side weaker

        double forwardBias = forward / 10.0; // Scale down the correction
        double backwardBias = backward / 10.0;
        double strafeRightBias = strafeRight / 10.0;
        double strafeLeftBias = strafeLeft / 10.0;

        // Forward movement compensation
        if (forward > 0) { // Robot turned right, left side stronger
            frCompensation += Math.abs(forwardBias);
            brCompensation += Math.abs(forwardBias);
        } else { // Robot turned left, right side stronger
            flCompensation += Math.abs(forwardBias);
            blCompensation += Math.abs(forwardBias);
        }

        // Backward movement compensation
        if (backward > 0) { // Robot turned right while going back
            frCompensation += Math.abs(backwardBias);
            brCompensation += Math.abs(backwardBias);
        } else {
            flCompensation += Math.abs(backwardBias);
            blCompensation += Math.abs(backwardBias);
        }

        // Strafe right compensation
        if (strafeRight > 0) {
            frCompensation += Math.abs(strafeRightBias);
            blCompensation += Math.abs(strafeRightBias);
        } else {
            flCompensation += Math.abs(strafeRightBias);
            brCompensation += Math.abs(strafeRightBias);
        }

        // Strafe left compensation
        if (strafeLeft > 0) {
            flCompensation += Math.abs(strafeLeftBias);
            brCompensation += Math.abs(strafeLeftBias);
        } else {
            frCompensation += Math.abs(strafeLeftBias);
            blCompensation += Math.abs(strafeLeftBias);
        }

        // Normalize to keep the strongest motor at 1.0
        double maxComp = Math.max(Math.max(flCompensation, blCompensation),
                Math.max(frCompensation, brCompensation));

        flCompensation /= maxComp;
        blCompensation /= maxComp;
        frCompensation /= maxComp;
        brCompensation /= maxComp;
    }

    private void displayResults() {
        telemetry.clear();
        telemetry.addLine("===== CALIBRATION COMPLETE =====");
        telemetry.addLine("");
        telemetry.addLine("Add these constants to DriveTrain.java:");
        telemetry.addLine("");
        telemetry.addData("FL_COMPENSATION", "%.4f", flCompensation);
        telemetry.addData("BL_COMPENSATION", "%.4f", blCompensation);
        telemetry.addData("FR_COMPENSATION", "%.4f", frCompensation);
        telemetry.addData("BR_COMPENSATION", "%.4f", brCompensation);
        telemetry.addLine("");
        telemetry.addLine("Then multiply motor powers by these values");
        telemetry.addLine("in the drive() method");
        telemetry.update();

        while (opModeIsActive()) {
            sleep(100);
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}