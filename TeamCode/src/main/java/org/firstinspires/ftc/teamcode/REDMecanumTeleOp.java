package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.teamcode.subsystems.*;
import java.util.List;

@TeleOp(name = "REDTeleOp")
public class REDMecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Enable bulk caching for all hubs - CRITICAL for performance
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);  // Changed to AUTO for better performance
        }

        // Initialize all subsystems
        DriveTrain drive = new DriveTrain(hardwareMap, false);
        ShooterSystem shooter = new ShooterSystem(hardwareMap);
        IntakeSystem intake = new IntakeSystem(hardwareMap);
        TurretSystem turret = new TurretSystem(hardwareMap);
        DistanceSensorSystem distanceSensors = new DistanceSensorSystem();

        // Initialize distance sensors
        boolean sensorsAvailable = false;
        try {
            distanceSensors.init(hardwareMap);
            sensorsAvailable = true;
            telemetry.addLine("Distance sensors initialized");
        } catch (Exception e) {
            telemetry.addLine("Distance sensors not found");
        }
        telemetry.update();

        boolean dynamicMode = true;
        boolean xPressed = false;
        boolean trianglePressed = false;
        boolean optionsPressed = false;
        boolean turretTrackingEnabled = true;

        // Performance optimization counters
        int loopCounter = 0;
        final int TELEMETRY_UPDATE_INTERVAL = 10;  // Update telemetry every 10 loops
        final int DISTANCE_CHECK_INTERVAL = 5;     // Check distance sensors every 5 loops
        final int TURRET_UPDATE_INTERVAL = 2;      // Update turret every 2 loops

        // Turret optimization
        double lastTurretTarget = 0;
        final double TURRET_DEADBAND = 3.0;  // Increased from 2.0 to reduce jitter

        // Cache frequently used values
        double cachedHeading = 0;
        int headingUpdateCounter = 0;
        final int HEADING_UPDATE_INTERVAL = 3;  // Update heading every 3 loops

        waitForStart();

        // START SHOOTER IMMEDIATELY
        shooter.setVelocity(dynamicMode);
        shooter.setAngle(dynamicMode);
        shooter.homeServo();

        while (opModeIsActive()) {
            loopCounter++;

            // Distance sensor check - only every N loops AND only if available
            if (sensorsAvailable && loopCounter % DISTANCE_CHECK_INTERVAL == 0) {
                try {
                    distanceSensors.checkThreeBallsAndVibrate(gamepad1);
                } catch (Exception e) {
                    // Silent fail
                }
            }

            // Reset IMU heading with Options button
            if (gamepad1.options && !optionsPressed) {
                drive.resetHeading();
                cachedHeading = 0;  // Reset cached heading
                telemetry.addLine(">>> IMU HEADING RESET! <<<");
                telemetry.update();
                sleep(300);
                optionsPressed = true;
            } else if (!gamepad1.options) {
                optionsPressed = false;
            }

            // Mode switching - X toggles dynamic/fixed mode
            if (gamepad1.x && !xPressed) {
                dynamicMode = !dynamicMode;
                shooter.setVelocity(dynamicMode);
                shooter.setAngle(dynamicMode);
                xPressed = true;
            } else if (!gamepad1.x) {
                xPressed = false;
            }

            // Toggle turret tracking with Triangle button
            if (gamepad1.triangle && !trianglePressed) {
                turretTrackingEnabled = !turretTrackingEnabled;
                if (!turretTrackingEnabled) {
                    turret.moveToAngle(0, 0.8);
                    lastTurretTarget = 0;
                }
                trianglePressed = true;
            } else if (!gamepad1.triangle) {
                trianglePressed = false;
            }

            // Boost mode
            double boostModeValue = gamepad1.left_bumper ? 0.0 : 2.0;

            // Drive - this runs every loop for responsive controls
            drive.drive(
                    gamepad1.left_stick_x * 1.1,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    boostModeValue
            );

            // SHOOTING: DPAD UP
            if (gamepad1.dpad_up) {
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
                    // Reset vibration when outtaking
                    if (sensorsAvailable) {
                        try {
                            distanceSensors.resetVibrationState();
                        } catch (Exception e) {
                            // Ignore
                        }
                    }
                } else {
                    intake.stop();
                }
            }

            // Update cached heading less frequently
            if (headingUpdateCounter++ % HEADING_UPDATE_INTERVAL == 0) {
                cachedHeading = drive.getHeading();
            }

            // Turret control - only update every N loops
            if (loopCounter % TURRET_UPDATE_INTERVAL == 0) {
                double targetAngle;
                if (turretTrackingEnabled) {
                    targetAngle = dynamicMode ? 43 - cachedHeading : 65 - cachedHeading;

                    // Only update turret if change is significant
                    if (Math.abs(targetAngle - lastTurretTarget) > TURRET_DEADBAND) {
                        turret.moveToAngle(targetAngle, 1);
                        lastTurretTarget = targetAngle;
                    }
                } else {
                    // Keep turret at 0 degrees (forward)
                    targetAngle = 0;
                    if (Math.abs(lastTurretTarget) > TURRET_DEADBAND) {
                        turret.moveToAngle(0, 0.8);
                        lastTurretTarget = 0;
                    }
                }
            }

            // Update telemetry only every N cycles
            if (loopCounter % TELEMETRY_UPDATE_INTERVAL == 0) {
                telemetry.addData("=== DRIVE ===", "");
                telemetry.addData("IMU Heading", "%.1f°", cachedHeading);
                telemetry.addData("Boost", gamepad1.left_bumper ? "ON" : "OFF");

                telemetry.addData("=== TURRET ===", "");
                telemetry.addData("Mode", turretTrackingEnabled ? "TRACKING" : "LOCKED 0°");
                telemetry.addData("Target", "%.1f°", lastTurretTarget);
                telemetry.addData("Current", "%.1f°", turret.getCurrentAngle());

                telemetry.addData("=== SHOOTER ===", "");
                telemetry.addData("Mode", dynamicMode ? "DYNAMIC" : "FIXED");
                telemetry.addData("RPM", "%.0f / %.0f", shooter.getVelocity(),
                        shooter.getTargetVelocity(dynamicMode));
                telemetry.addData("Ready", shooter.isAtTargetVelocity(dynamicMode) ? "YES" : "NO");

                telemetry.update();
            }
        }

        // Stop all systems when match ends
        shooter.stop();
        intake.stop();
    }
}