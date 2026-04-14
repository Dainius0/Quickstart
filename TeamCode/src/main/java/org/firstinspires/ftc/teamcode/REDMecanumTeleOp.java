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
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // ── INIT: IMU only ────────────────────────────────────────────────────
        DriveTrain drive = new DriveTrain(hardwareMap, false);
        drive.update();
        drive.resetHeading();

        telemetry.addLine("=== RED SIDE TELEOP ===");
        telemetry.addLine("IMU heading captured — facing direction is now 0°");
        telemetry.addLine("Ready to start!");
        telemetry.update();
        // ─────────────────────────────────────────────────────────────────────

        waitForStart();

        // ── START: initialise everything else ────────────────────────────────
        boolean dynamicMode = true;

        ShooterSystem shooter = new ShooterSystem(hardwareMap, telemetry);
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

        shooter.setVelocity(dynamicMode);
        shooter.homeServo();
        // ─────────────────────────────────────────────────────────────────────

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

        double cachedHeading = 0;
        int headingUpdateCounter = 0;
        final int HEADING_UPDATE_INTERVAL = 3;

        while (opModeIsActive()) {
            loopCounter++;

            drive.update();

            // ── Software PID update — must be called every loop ──────────────
            shooter.update();
            // ─────────────────────────────────────────────────────────────────

            if (sensorsAvailable && loopCounter % DISTANCE_CHECK_INTERVAL == 0) {
                try {
                    distanceSensors.checkThreeBallsAndVibrate(gamepad1);
                } catch (Exception e) { /* silent fail */ }
            }

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

            if (gamepad1.x && !xPressed) {
                dynamicMode = !dynamicMode;
                shooter.setVelocity(dynamicMode);
                xPressed = true;
            } else if (!gamepad1.x) {
                xPressed = false;
            }

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

            boolean slowMode = gamepad1.left_bumper;

            drive.drive(
                    gamepad1.left_stick_x * 1.1,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    slowMode
            );

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
                        try { distanceSensors.resetVibrationState(); }
                        catch (Exception e) { /* ignore */ }
                    }
                } else {
                    intake.stop();
                }
            }

            if (headingUpdateCounter++ % HEADING_UPDATE_INTERVAL == 0) {
                cachedHeading = drive.getHeading();
            }

            if (loopCounter % TURRET_UPDATE_INTERVAL == 0) {
                if (turretTrackingEnabled) {
                    // RED-specific constants and sign kept exactly as before
                    double targetAngle = dynamicMode ? 44 - cachedHeading : 65 - cachedHeading;
                    if (Math.abs(targetAngle - lastTurretTarget) > TURRET_DEADBAND) {
                        turret.moveToAngle(targetAngle, 1);
                        lastTurretTarget = targetAngle;
                    }
                } else {
                    if (Math.abs(lastTurretTarget) > TURRET_DEADBAND) {
                        turret.moveToAngle(0, 0.8);
                        lastTurretTarget = 0;
                    }
                }
            }

            if (loopCounter % TELEMETRY_UPDATE_INTERVAL == 0) {
                telemetry.addData("IMU Heading", "%.1f°", cachedHeading);
                telemetry.addData("Slow Mode", gamepad1.left_bumper ? "ON" : "OFF");

                turret.printTelemetry();

                shooter.addTelemetry(dynamicMode);

                telemetry.update();
            }
        }

        shooter.stop();
        intake.stop();
    }
}