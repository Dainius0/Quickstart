    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.hardware.lynx.LynxModule;
    import org.firstinspires.ftc.teamcode.subsystems.*;
    import java.util.List;

    @TeleOp(name = "BLUETeleop")
    public class BLUEMecanumTeleOp extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
            // Enable bulk caching for all hubs - CRITICAL for performance
            List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);  // AUTO mode for better performance
            }

            // Initialize all subsystems
            DriveTrain drive = new DriveTrain(hardwareMap, false);
            ShooterSystem shooter = new ShooterSystem(hardwareMap);
            IntakeSystem intake = new IntakeSystem(hardwareMap);
            TurretSystem turret = new TurretSystem(hardwareMap);

            // DISTANCE SENSOR SYSTEM:
            // Creates the distance sensor system object that will monitor three sensors
            // to detect when the robot has collected three game balls
            DistanceSensorSystem distanceSensors = new DistanceSensorSystem();

            // DISTANCE SENSOR INITIALIZATION:
            // Try to initialize the three distance sensors from the hardware configuration.
            // sensorsAvailable flag tracks whether initialization succeeded - this allows
            // the program to continue running even if sensors aren't connected (graceful degradation)
            boolean sensorsAvailable = false;
            try {
                distanceSensors.init(hardwareMap);
                sensorsAvailable = true;  // Sensors found and initialized successfully
                telemetry.addLine("Distance sensors initialized");
            } catch (Exception e) {
                // If sensors aren't in the hardware config, catch the error and continue
                // Robot will function normally, just without ball detection feedback
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

            // DISTANCE SENSOR PERFORMANCE OPTIMIZATION:
            // Check distance sensors only every 5 loops instead of every loop.
            // This reduces I2C bus traffic and improves overall loop time.
            // 5 loops ≈ 50-100ms interval, which is fast enough for ball detection
            final int DISTANCE_CHECK_INTERVAL = 5;     // Check distance sensors every 5 loops

            final int TURRET_UPDATE_INTERVAL = 2;      // Update turret every 2 loops

            // Turret optimization
            double lastTurretTarget = 0;
            final double TURRET_DEADBAND = 3.0;  // Increased deadband to reduce jitter

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

                // DISTANCE SENSOR PERIODIC CHECK:
                // Only check sensors every DISTANCE_CHECK_INTERVAL loops (every 5th loop)
                // AND only if sensors were successfully initialized (sensorsAvailable = true)
                // This conditional check-in pattern optimizes performance while still providing
                // timely feedback when the robot collects its third ball
                if (sensorsAvailable && loopCounter % DISTANCE_CHECK_INTERVAL == 0) {
                    try {
                        // Call the vibration check method - this will:
                        // 1. Read all three distance sensors
                        // 2. Check if all three detect balls (< 5cm threshold)
                        // 3. Vibrate gamepad1 if transitioning to "all three detected" state
                        // 4. Use edge detection to prevent continuous vibration
                        distanceSensors.checkThreeBallsAndVibrate(gamepad1);
                    } catch (Exception e) {
                        // Silent fail - if sensors throw an error mid-match, don't crash
                        // This handles sensor disconnections or I2C communication errors gracefully
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

                        // DISTANCE SENSOR VIBRATION RESET ON OUTTAKE:
                        // When driver outtakes balls (dpad_down), reset the vibration state.
                        // This ensures that when the robot collects three balls again after
                        // outtaking, it will vibrate to notify the driver. Without this reset,
                        // the edge detection would prevent vibration on subsequent collections.
                        if (sensorsAvailable) {
                            try {
                                distanceSensors.resetVibrationState();
                            } catch (Exception e) {
                                // Ignore - fail silently if sensors have issues
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

                // Turret control - only update every N loops (BLUE SIDE: negative angles)
                if (loopCounter % TURRET_UPDATE_INTERVAL == 0) {
                    double targetAngle;
                    if (turretTrackingEnabled) {
                        targetAngle = dynamicMode ? -43 - cachedHeading : -65 - cachedHeading;

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