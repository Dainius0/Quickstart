package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "BLUETeleop")
public class BLUEMecanumTeleOp extends LinearOpMode {

    // Odometry target point
    private static final double TARGET_X = 0;
    private static final double TARGET_Y = 144;

    // Reset calibration point
    private static final double RESET_X = 32;
    private static final double RESET_Y = 136;
    private static final double RESET_ANGLE = 90; // degrees
    private static final double RESET_TOLERANCE = 3.0; // inches tolerance for reset location

    // Initial pose for odometry - matches end position from autonomous
    private static final Pose INITIAL_POSE = new Pose(48, 60, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Pedro Pathing follower for odometry
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setPose(INITIAL_POSE);

        // Pass 'false' to prevent resetting IMU yaw - preserves heading from autonomous
        DriveTrain drive = new DriveTrain(hardwareMap, false);
        ShooterSystem shooter = new ShooterSystem(hardwareMap);
        IntakeSystem intake = new IntakeSystem(hardwareMap);
        TurretSystem turret = new TurretSystem(hardwareMap);

        boolean dynamicMode = true;
        boolean lastDynamicMode = true;
        boolean xPressed = false;
        boolean yPressed = false;
        boolean bPressed = false;
        boolean trianglePressed = false;
        boolean squarePressed = false;
        boolean turretTrackingEnabled = true;
        boolean odometryMode = false; // NEW: Odometry targeting mode

        waitForStart();

        // START SHOOTER IMMEDIATELY - motors regulate RPM continuously
        shooter.setVelocity(dynamicMode);
        shooter.setAngle(dynamicMode);
        shooter.homeServo();

        while (opModeIsActive()) {
            // Update odometry
            follower.update();
            Pose currentPose = follower.getPose();

            // Manual reset option - press options if you need to reset heading
            if (gamepad1.options) drive.resetHeading();

            // NEW: Check if at reset location and reset odometry if Share button pressed
            if (gamepad1.share) {
                double distanceToReset = Math.hypot(
                        currentPose.getX() - RESET_X,
                        currentPose.getY() - RESET_Y
                );

                if (distanceToReset <= RESET_TOLERANCE) {
                    // Robot is at reset location, reset odometry
                    follower.setPose(new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_ANGLE)));
                    telemetry.addLine("ODOMETRY RESET!");
                    telemetry.update();
                    sleep(500); // Brief pause to show message
                }
            }

            // Mode switching - X toggles fixed mode on/off, Y toggles dynamic mode on/off
            if (gamepad1.x && !xPressed) {
                dynamicMode = !dynamicMode;
                shooter.setVelocity(dynamicMode);
                shooter.setAngle(dynamicMode);
                xPressed = true;
            }
            else if (!gamepad1.x) xPressed = false;

            // Y button also toggles (alternative to X)
            if (gamepad1.y && !yPressed) {
                dynamicMode = !dynamicMode;
                shooter.setVelocity(dynamicMode);
                shooter.setAngle(dynamicMode);
                yPressed = true;
            }
            else if (!gamepad1.y) yPressed = false;



            // Toggle turret tracking with B button (disables both IMU and odometry tracking)
            if (gamepad1.b && !bPressed) {
                turretTrackingEnabled = !turretTrackingEnabled;
                if (!turretTrackingEnabled) {
                    odometryMode = false; // Also disable odometry mode
                }
                bPressed = true;
            }
            else if (!gamepad1.b) bPressed = false;

            // Toggle turret home position with Triangle button
            if (gamepad1.triangle && !trianglePressed) {
                if (turretTrackingEnabled) {
                    // If currently tracking, go to home (0 position)
                    turretTrackingEnabled = false;
                    odometryMode = false;
                    turret.moveToAngle(0, 0.8);
                } else {
                    // If at home, resume tracking (IMU mode by default)
                    turretTrackingEnabled = true;
                    odometryMode = false;
                }
                trianglePressed = true;
            }
            else if (!gamepad1.triangle) trianglePressed = false;

            // NEW: Toggle odometry targeting mode with Square button
            if (gamepad1.square && !squarePressed) {
                if (turretTrackingEnabled) {
                    odometryMode = !odometryMode;
                }
                squarePressed = true;
            }
            else if (!gamepad1.square) squarePressed = false;

            // Drive
            drive.drive(
                    gamepad1.left_stick_x * 1.1,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    gamepad1.left_trigger
            );

            // SHOOTING: DPAD UP - Extend servo and run intake
            if (gamepad1.dpad_up) {
                shooter.shoot();
                intake.intakeIn();
            }
            // LEFT BUMPER - Force feed with reduced power
            else if (gamepad1.left_bumper) {
                shooter.shoot();
                intake.intakeInReduced();
            }
            // Not shooting
            else {
                shooter.homeServo();

                // Normal intake controls
                if (gamepad1.right_bumper) {
                    intake.intakeIn();
                } else if (gamepad1.dpad_down) {
                    intake.intakeOut();
                } else {
                    intake.stop();
                }
            }

            // Turret control - choose mode based on flags
            if (turretTrackingEnabled) {
                if (odometryMode) {
                    // NEW: Odometry-based targeting
                    double targetAngle = calculateTurretAngleToTarget(currentPose);
                    turret.moveToAngle(targetAngle, 0.8);
                } else {
                    // Original IMU-based tracking
                    double imuHeading = drive.getHeading();
                    double targetAngle = dynamicMode ? -44.5 - imuHeading : -72 - imuHeading;
                    turret.moveToAngle(targetAngle, 0.8);
                }
            }

            // Telemetry
            telemetry.addData("=== ODOMETRY ===", "");
            telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

            // Show distance to reset point
            double distToReset = Math.hypot(
                    currentPose.getX() - RESET_X,
                    currentPose.getY() - RESET_Y
            );
            telemetry.addData("Distance to Reset Point", "%.1f inches", distToReset);
            if (distToReset <= RESET_TOLERANCE) {
                telemetry.addLine(">>> PRESS SHARE TO RESET ODOMETRY <<<");
            }

            telemetry.addData("", "");
            telemetry.addData("=== DRIVE ===", "");
            telemetry.addData("IMU Heading", "%.1f°", drive.getHeading());
            telemetry.addData("", "");
            telemetry.addData("=== TURRET ===", "");

            if (!turretTrackingEnabled) {
                telemetry.addData("Tracking Mode", "HOME POSITION (0°)");
                telemetry.addData("Target Angle", "0° (Home)");
            } else if (odometryMode) {
                telemetry.addData("Tracking Mode", "ODOMETRY TARGETING");
                telemetry.addData("Target Point", "X: %.0f, Y: %.0f", TARGET_X, TARGET_Y);
                double targetAngle = calculateTurretAngleToTarget(currentPose);
                telemetry.addData("Target Angle", "%.1f°", targetAngle);
                double distToTarget = Math.hypot(
                        currentPose.getX() - TARGET_X,
                        currentPose.getY() - TARGET_Y
                );
                telemetry.addData("Distance to Target", "%.1f inches", distToTarget);
            } else {
                telemetry.addData("Tracking Mode", "IMU TRACKING");
                double imuHeading = drive.getHeading();
                double targetAngle = dynamicMode ? -44.5 - imuHeading : -72 - imuHeading;
                telemetry.addData("Target Angle", "%.1f°", targetAngle);
            }

            telemetry.addData("Press B to Toggle Tracking", "");
            telemetry.addData("Press Triangle for Home/Resume", "");
            telemetry.addData("Press Square for Odometry Mode", "");
            telemetry.addData("", "");
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Mode", dynamicMode ? "DYNAMIC" : "FIXED");
            telemetry.addData("Target Velocity", dynamicMode ? ShooterSystem.VELOCITY_DYNAMIC : ShooterSystem.VELOCITY_FIXED);
            telemetry.addData("Current Velocity", "%.0f", shooter.getVelocity());
            telemetry.addData("At Target RPM", shooter.isAtTargetVelocity(dynamicMode) ? "YES" : "NO");
            telemetry.addData("Servo Position", gamepad1.dpad_up || gamepad1.left_bumper ? "SHOOTING" : "HOME");
            telemetry.addData("", "");
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("Right Bumper", "Normal Intake");
            telemetry.addData("DPAD Down", "Outtake");
            telemetry.addData("DPAD Up", "Shoot (servo + intake)");
            telemetry.addData("Left Bumper", "Force Feed (reduced power)");
            telemetry.addData("X or Y", "Toggle Dynamic/Fixed Mode");
            telemetry.addData("B", "Toggle Tracking On/Off");
            telemetry.addData("Triangle", "Home Position / Resume Tracking");
            telemetry.addData("Square", "Toggle Odometry Targeting");
            telemetry.addData("Options", "Reset IMU Heading");
            telemetry.addData("Share", "Reset Odometry (at reset point)");
            telemetry.update();
        }

        // Stop shooter when match ends
        shooter.stop();
    }

    /**
     * Calculate the turret angle needed to point at the target
     * based on current robot position from odometry
     */
    private double calculateTurretAngleToTarget(Pose robotPose) {
        // Get robot position
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading(); // in radians

        // Calculate vector from robot to target
        double deltaX = TARGET_X - robotX;
        double deltaY = TARGET_Y - robotY;

        // Calculate absolute angle to target (field-centric)
        double angleToTarget = Math.atan2(deltaY, deltaX); // radians

        // Convert to degrees
        double angleToTargetDegrees = Math.toDegrees(angleToTarget);
        double robotHeadingDegrees = Math.toDegrees(robotHeading);

        // Calculate turret angle relative to robot
        // This is the difference between where we need to point and where robot is facing
        double turretAngle = angleToTargetDegrees - robotHeadingDegrees;

        // Normalize to -180 to 180 range
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;

        return turretAngle;
    }
}