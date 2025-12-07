package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "BLUETeleop")
public class BLUEMecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Pass 'false' to prevent resetting IMU yaw - preserves heading from autonomous
        DriveTrain drive = new DriveTrain(hardwareMap, false);
        ShooterSystem shooter = new ShooterSystem(hardwareMap);
        IntakeSystem intake = new IntakeSystem(hardwareMap);
        TurretSystem turret = new TurretSystem(hardwareMap);

        boolean dynamicMode = true;
        boolean lastDynamicMode = true;
        boolean xPressed = false;
        boolean aPressed = false;
        boolean bPressed = false;
        boolean trianglePressed = false;
        boolean turretTrackingEnabled = true;

        // Shooting state
        boolean isShooting = false;
        boolean isForceFeeding = false;
        int ballsShot = 0;
        ElapsedTime shootTimer = new ElapsedTime();

        waitForStart();

        // START SHOOTER IMMEDIATELY - set velocity and angle once
        shooter.setVelocity(dynamicMode);
        shooter.setAngle(dynamicMode);
        shooter.homeServo();

        while (opModeIsActive()) {

            // Manual reset option - press options if you need to reset heading
            if (gamepad1.options) drive.resetHeading();

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

            // Toggle turret home position with Triangle button
            if (gamepad1.triangle && !trianglePressed) {
                if (turretTrackingEnabled) {
                    // If currently tracking, go to home (0 position)
                    turretTrackingEnabled = false;
                    turret.moveToAngle(0, 0.8);
                } else {
                    // If at home, resume tracking
                    turretTrackingEnabled = true;
                }
                trianglePressed = true;
            }
            else if (!gamepad1.triangle) trianglePressed = false;

            // Drive
            drive.drive(
                    gamepad1.left_stick_x * 1.1,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    gamepad1.left_trigger
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
            // DPAD UP - Smart shoot (first ball immediate, others wait for velocity)
            else if (gamepad1.dpad_up) {
                // Starting to shoot - initialize
                if (!isShooting) {
                    shooter.shoot();
                    isShooting = true;
                    ballsShot = 0;
                    shootTimer.reset();
                }

                // First ball: shoot immediately without waiting for velocity
                if (ballsShot == 0) {
                    intake.intakeIn();
                    // After 0.5 seconds, consider first ball shot
                    if (shootTimer.seconds() > 0.5) {
                        ballsShot = 1;
                        shootTimer.reset();
                    }
                }
                // All subsequent balls: wait for target velocity
                else {
                    if (shooter.isAtTargetVelocity(dynamicMode)) {
                        intake.intakeIn();
                    } else {
                        intake.stop();
                    }
                }
            }
            // Neither bumper nor dpad up pressed
            else {
                // Retract servo if we were shooting or force feeding
                if (isShooting || isForceFeeding) {
                    shooter.homeServo();
                    isShooting = false;
                    isForceFeeding = false;
                    ballsShot = 0;
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

            // Turret follow IMU heading only when tracking is enabled
            if (turretTrackingEnabled) {
                double imuHeading = drive.getHeading();
                double targetAngle = dynamicMode ? -44.5 - imuHeading : -72 - imuHeading;
                turret.moveToAngle(targetAngle, 0.8);
            }

            // Telemetry
            telemetry.addData("=== DRIVE ===", "");
            telemetry.addData("IMU Heading", "%.1f°", drive.getHeading());
            telemetry.addData("", "");
            telemetry.addData("=== TURRET ===", "");
            telemetry.addData("Tracking Mode", turretTrackingEnabled ? "IMU TRACKING" : "HOME POSITION (0°)");
            if (turretTrackingEnabled) {
                double imuHeading = drive.getHeading();
                double targetAngle = dynamicMode ? -44.5 - imuHeading : -72 - imuHeading;
                telemetry.addData("Target Angle", "%.1f°", targetAngle);
            } else {
                telemetry.addData("Target Angle", "0° (Home)");
            }
            telemetry.addData("Press B to Toggle Tracking", "");
            telemetry.addData("Press Triangle for Home/Resume", "");
            telemetry.addData("", "");
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Mode", dynamicMode ? "DYNAMIC" : "FIXED");
            telemetry.addData("Target Velocity", dynamicMode ? ShooterSystem.VELOCITY_DYNAMIC : ShooterSystem.VELOCITY_FIXED);
            telemetry.addData("Current Velocity", "%.0f", shooter.getVelocity());
            telemetry.addData("Shooting (DPAD UP)", isShooting ? "YES" : "NO");
            telemetry.addData("First Ball", ballsShot >= 1 ? "SHOT" : (isShooting ? "SHOOTING NOW" : "READY"));
            telemetry.addData("Additional Balls", ballsShot == 0 ? "Waiting..." : (shooter.isAtTargetVelocity(dynamicMode) ? "FEEDING" : "Waiting for RPM"));
            telemetry.addData("Force Feeding (L-BUMPER)", isForceFeeding ? "YES - Reduced power" : "NO");
            telemetry.addData("At Target RPM", shooter.isAtTargetVelocity(dynamicMode) ? "YES" : "NO");
            telemetry.addData("", "");
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("Right Bumper", "Normal Intake");
            telemetry.addData("DPAD Down", "Outtake");
            telemetry.addData("DPAD Up", "Smart Shoot (1st instant, rest wait)");
            telemetry.addData("Left Bumper", "Force Feed (no wait)");
            telemetry.addData("A", "Dynamic Mode");
            telemetry.addData("X", "Fixed Mode");
            telemetry.addData("B", "Toggle Tracking On/Off");
            telemetry.addData("Triangle", "Home Position / Resume Tracking");
            telemetry.addData("Options", "Reset IMU Heading");
            telemetry.update();
        }

        // Stop shooter when match ends
        shooter.stop();
    }
}