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
        // Enable bulk caching for all hubs
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize all subsystems
        DriveTrain drive = new DriveTrain(hardwareMap, false);
        ShooterSystem shooter = new ShooterSystem(hardwareMap);
        IntakeSystem intake = new IntakeSystem(hardwareMap);
        TurretSystem turret = new TurretSystem(hardwareMap, telemetry);
        DistanceSensorSystem distanceSensors = new DistanceSensorSystem();

        boolean sensorsAvailable = false;
        try {
            distanceSensors.init(hardwareMap);
            sensorsAvailable = true;
            telemetry.addLine("Distance sensors initialized");
        } catch (Exception e) {
            telemetry.addLine("Distance sensors not found");
        }

        telemetry.addLine("=== BLUE SIDE TELEOP ===");
        telemetry.addLine("Press INIT to reset IMU heading");
        telemetry.addLine("Ready to start!");
        telemetry.update();

        boolean dynamicMode = true;
        boolean xPressed = false;
        boolean trianglePressed = false;
        boolean optionsPressed = false;
        boolean turretTrackingEnabled = true;

        int loopCounter = 0;
        final int TELEMETRY_UPDATE_INTERVAL = 10;
        final int DISTANCE_CHECK_INTERVAL = 5;
        final int TURRET_UPDATE_INTERVAL = 2;

        double lastTurretTarget = 0;
        final double TURRET_DEADBAND = 3.0;

        // cachedHeading is now sourced from drive.getHeading() after drive.update()
        double cachedHeading = 0;
        int headingUpdateCounter = 0;
        final int HEADING_UPDATE_INTERVAL = 3;

        waitForStart();

        drive.resetHeading();
        cachedHeading = 0;

        telemetry.addLine(">>> IMU HEADING RESET TO 0° <<<");
        telemetry.addLine("Robot forward direction is now 0°");
        telemetry.update();
        sleep(300);

        shooter.setVelocity(dynamicMode);
        shooter.homeServo();

        while (opModeIsActive()) {
            loopCounter++;

            // FIX: Call update() ONCE at the top of every loop.
            // This is the only place pinpoint is polled, preventing heading drift.
            drive.update();

            // Distance sensor check
            if (sensorsAvailable && loopCounter % DISTANCE_CHECK_INTERVAL == 0) {
                try {
                    distanceSensors.checkThreeBallsAndVibrate(gamepad1);
                } catch (Exception e) {
                    // Silent fail
                }
            }

            // Manual IMU heading reset
            if (gamepad1.options && !optionsPressed) {
                drive.resetHeading();
                cachedHeading = 0;
                telemetry.addLine(">>> IMU HEADING MANUALLY RESET! <<<");
                telemetry.update();
                sleep(300);
                optionsPressed = true;
            } else if (!gamepad1.options) {
                optionsPressed = false;
            }

            // Mode switching
            if (gamepad1.x && !xPressed) {
                dynamicMode = !dynamicMode;
                shooter.setVelocity(dynamicMode);
                xPressed = true;
            } else if (!gamepad1.x) {
                xPressed = false;
            }

            // Toggle turret tracking
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

            // Drive - uses cached heading from drive.update() above
            drive.drive(
                    gamepad1.left_stick_x * 1.1,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    boostModeValue
            );

            // Shooting
            if (gamepad1.dpad_up) {
                shooter.shoot();
                intake.intakeInReduced();
            } else {
                shooter.homeServo();

                if (gamepad1.right_bumper) {
                    intake.intakeIn();
                } else if (gamepad1.dpad_down) {
                    intake.intakeOut();
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
                cachedHeading = drive.getHeading(); // Safe - just reads cached value, no extra update
            }

            // Turret control
            if (loopCounter % TURRET_UPDATE_INTERVAL == 0) {
                double targetAngle;
                if (turretTrackingEnabled) {
                    double relativeHeading = cachedHeading - 0;
                    targetAngle = dynamicMode ? -(42 + relativeHeading) : -(65 + relativeHeading);

                    if (Math.abs(targetAngle - lastTurretTarget) > TURRET_DEADBAND) {
                        turret.moveToAngle(targetAngle, 1);
                        lastTurretTarget = targetAngle;
                    }
                } else {
                    targetAngle = 0;
                    if (Math.abs(lastTurretTarget) > TURRET_DEADBAND) {
                        turret.moveToAngle(0, 0.8);
                        lastTurretTarget = 0;
                    }
                }
            }

            // Telemetry
            if (loopCounter % TELEMETRY_UPDATE_INTERVAL == 0) {
                telemetry.addData("=== DRIVE ===", "");
                telemetry.addData("IMU Heading", "%.1f°", cachedHeading);
                telemetry.addData("Boost", gamepad1.left_bumper ? "ON" : "OFF");

                turret.printTelemetry();

                telemetry.addData("=== SHOOTER ===", "");
                telemetry.addData("Mode", dynamicMode ? "DYNAMIC" : "FIXED");
                telemetry.addData("RPM", "%.0f / %.0f", shooter.getVelocity(),
                        shooter.getTargetVelocity(dynamicMode));
                telemetry.addData("Ready", shooter.isAtTargetVelocity(dynamicMode) ? "YES" : "NO");

                telemetry.update();
            }
        }

        shooter.stop();
        intake.stop();
    }
}