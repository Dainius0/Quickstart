package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "REDTeleOp")
public class REDMecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Pass 'false' to prevent resetting IMU yaw - preserves heading from autonomous
        DriveTrain drive = new DriveTrain(hardwareMap, false);
        ShooterSystem shooter = new ShooterSystem(hardwareMap);
        IntakeSystem intake = new IntakeSystem(hardwareMap);
        TurretSystem turret = new TurretSystem(hardwareMap);

        boolean dynamicMode = true;
        boolean lastDynamicMode = true;
        boolean xPressed = false, aPressed = false;
        boolean turretTrackingEnabled = true;
        boolean bPressed = false;
        boolean slowModeEnabled = false;
        boolean yPressed = false;

        // Shooting state
        boolean isShooting = false;
        boolean isForceFeeding = false;

        waitForStart();

        // START SHOOTER IMMEDIATELY - set velocity and angle once
        shooter.setVelocity(dynamicMode);
        shooter.setAngle(dynamicMode);
        shooter.homeServo(); // Make sure servo is home

        while (opModeIsActive()) {

            // Manual reset option - press options if you need to reset heading
            if (gamepad1.options) drive.resetHeading();

            // Toggle slow mode with Y button
            if (gamepad1.y && !yPressed) {
                slowModeEnabled = !slowModeEnabled;
                yPressed = true;
            }
            else if (!gamepad1.y) yPressed = false;

            // Mode switching - only update when mode changes
            if (gamepad1.x && !xPressed) {
                dynamicMode = false;
                xPressed = true;
            }
            else if (!gamepad1.x) xPressed = false;

            if (gamepad1.a && !aPressed) {
                dynamicMode = true;
                aPressed = true;
            }
            else if (!gamepad1.a) aPressed = false;

            // Only update shooter when mode actually changes
            if (dynamicMode != lastDynamicMode) {
                shooter.setVelocity(dynamicMode);
                shooter.setAngle(dynamicMode);
                lastDynamicMode = dynamicMode;
            }

            // Toggle turret tracking with B button
            if (gamepad1.b && !bPressed) {
                turretTrackingEnabled = !turretTrackingEnabled;
                bPressed = true;
            }
            else if (!gamepad1.b) bPressed = false;

            // Drive with slow mode
            drive.drive(
                    gamepad1.left_stick_x * 1.1,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    slowModeEnabled ? 1.0 : 0.0
            );

            // LEFT BUMPER - Force feed (no RPM check, reduced power)
            if (gamepad1.left_bumper) {
                // Only extend servo once when starting to force feed
                if (!isForceFeeding) {
                    shooter.shoot();
                    isForceFeeding = true;
                }

                // Always run intakes at reduced power
                intake.intakeInReduced();

            }
            // DPAD UP - Normal shoot (waits for target velocity)
            else if (gamepad1.dpad_up) {
                // Only extend servo once when starting to shoot
                if (!isShooting) {
                    shooter.shoot();
                    isShooting = true;
                }

                // Run intakes only if at target velocity
                if (shooter.isAtTargetVelocity(dynamicMode)) {
                    intake.intakeIn();
                } else {
                    intake.stop();
                }
            }
            // Neither bumper nor dpad up pressed
            else {
                // Retract servo if we were shooting or force feeding
                if (isShooting || isForceFeeding) {
                    shooter.homeServo();
                    isShooting = false;
                    isForceFeeding = false;
                }

                // Normal intake controls
                if (gamepad1.right_bumper) {
                    intake.intakeIn();
                } else if (gamepad1.dpad_down) {
                    intake.intakeOut();
                } else {
                    intake.stop();
                }
            }

            // Turret follow IMU heading
            double imuHeading = drive.getHeading();
            double targetAngle = dynamicMode ? 44.5  - imuHeading : 65 - imuHeading;  //pamazinau 72 -> 70 -> 68 kai mazeja, tada sukasi desiniau (tolimas sovimas)
            turret.moveToAngle(targetAngle, 0.8); // 45 -> 50

            // Telemetry
            telemetry.addData("=== DRIVE ===", "");
            telemetry.addData("IMU Heading", "%.1f°", imuHeading);
            telemetry.addData("Slow Mode (Y)", slowModeEnabled ? "ON (80%)" : "OFF (100%)");
            telemetry.addData("", "");
            telemetry.addData("=== TURRET ===", "");
            telemetry.addData("Tracking Mode", turretTrackingEnabled ? "IMU TRACKING" : "HOME POSITION");
            telemetry.addData("Target Angle", "%.1f°", targetAngle);
            telemetry.addData("Press B to Toggle", "");
            telemetry.addData("", "");
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Mode", dynamicMode ? "DYNAMIC" : "FIXED");
            telemetry.addData("Target Velocity", dynamicMode ? ShooterSystem.VELOCITY_DYNAMIC : ShooterSystem.VELOCITY_FIXED);
            telemetry.addData("Current Velocity", "%.0f", shooter.getVelocity());
            telemetry.addData("Shooting (DPAD UP)", isShooting ? "YES - Waiting for RPM" : "NO");
            telemetry.addData("Force Feeding (L-BUMPER)", isForceFeeding ? "YES - Reduced power" : "NO");
            telemetry.addData("At Target RPM", shooter.isAtTargetVelocity(dynamicMode) ? "YES" : "NO");
            telemetry.addData("", "");
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("Y", "Toggle Slow Mode (80%)");
            telemetry.addData("Right Bumper", "Normal Intake");
            telemetry.addData("DPAD Down", "Outtake");
            telemetry.addData("DPAD Up", "Shoot (waits for RPM)");
            telemetry.addData("Left Bumper", "Force Feed (no wait)");
            telemetry.addData("A", "Dynamic Mode");
            telemetry.addData("X", "Fixed Mode");
            telemetry.addData("Options", "Reset IMU Heading");
            telemetry.update();
        }

        // Stop shooter when match ends
        shooter.stop();
    }
}