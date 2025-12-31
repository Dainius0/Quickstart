package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        boolean xPressed = false;
        boolean trianglePressed = false;
        boolean optionsPressed = false;
        boolean turretTrackingEnabled = true;

        waitForStart();

        // START SHOOTER IMMEDIATELY - motors regulate RPM continuously
        shooter.setVelocity(dynamicMode);
        shooter.setAngle(dynamicMode);
        shooter.homeServo();

        while (opModeIsActive()) {
            // Reset IMU heading with Options button
            if (gamepad1.options && !optionsPressed) {
                drive.resetHeading();
                telemetry.addLine(">>> IMU HEADING RESET! <<<");
                telemetry.update();
                sleep(300); // Brief pause to show message
                optionsPressed = true;
            }
            else if (!gamepad1.options) optionsPressed = false;

            // Mode switching - X toggles dynamic/fixed mode
            if (gamepad1.x && !xPressed) {
                dynamicMode = !dynamicMode;
                shooter.setVelocity(dynamicMode);
                shooter.setAngle(dynamicMode);
                xPressed = true;
            }
            else if (!gamepad1.x) xPressed = false;

            // Toggle between IMU tracking and 0 degree turret angle with Triangle button
            if (gamepad1.triangle && !trianglePressed) {
                turretTrackingEnabled = !turretTrackingEnabled;
                if (!turretTrackingEnabled) {
                    turret.moveToAngle(0, 0.8);
                    turret.resetToHome(); // Reset turret's accumulated error when homing
                }
                trianglePressed = true;
            }
            else if (!gamepad1.triangle) trianglePressed = false;

            // INVERTED: Boost mode - 0.0 when not pressed (60% speed), 2.0 when pressed (100% speed)
            // Default is slower, pressing button gives full speed boost
            double boostModeValue = gamepad1.left_bumper ? 0.0 : 2.0;

            // Drive with left bumper as boost mode (inverted from slow mode)
            drive.drive(
                    gamepad1.left_stick_x * 1.1,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    boostModeValue
            );

            // SHOOTING: DPAD UP - Extend servo and run intake
            if (gamepad1.dpad_up) {
                shooter.shoot();
                // Use slower intake for fixed mode, normal speed for dynamic mode
                if (dynamicMode) {
                    intake.intakeIn();
                } else {
                    intake.intakeInReduced();
                }
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

            // Turret control - switch between IMU tracking and 0 degree position
            if (turretTrackingEnabled) {
                // IMU-based tracking
                double imuHeading = drive.getHeading();
                double targetAngle = dynamicMode ? -45 - imuHeading : -65 - imuHeading;
                turret.moveToAngle(targetAngle, 0.8);
            } else {
                // 0 degree position
                turret.moveToAngle(0, 0.8);
            }

            // Telemetry
            telemetry.addData("=== DRIVE ===", "");
            telemetry.addData("IMU Heading", "%.1f°", drive.getHeading());
            telemetry.addData("Boost Mode", gamepad1.left_bumper ? "BOOST (100%)" : "NORMAL (60%)");
            telemetry.addData("", "");
            telemetry.addData("=== TURRET ===", "");

            if (!turretTrackingEnabled) {
                telemetry.addData("Tracking Mode", "0° POSITION");
                telemetry.addData("Target Angle", "0°");
            }
            else {
                telemetry.addData("Tracking Mode", "IMU TRACKING");
                double imuHeading = drive.getHeading();
                double targetAngle = dynamicMode ? -45 - imuHeading : -65 - imuHeading;
                telemetry.addData("Target Angle", "%.1f°", targetAngle);
            }

            telemetry.addData("Press Triangle to Switch Mode", "");
            telemetry.addData("", "");
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Mode", dynamicMode ? "DYNAMIC" : "FIXED");
            telemetry.addData("Target Velocity", dynamicMode ? ShooterSystem.VELOCITY_DYNAMIC : ShooterSystem.VELOCITY_FIXED);
            telemetry.addData("Current Velocity", "%.0f", shooter.getVelocity());
            telemetry.addData("At Target RPM", shooter.isAtTargetVelocity(dynamicMode) ? "YES" : "NO");
            telemetry.addData("Servo Position", gamepad1.dpad_up ? "SHOOTING" : "HOME");
            telemetry.addData("Intake Speed", gamepad1.dpad_up ? (dynamicMode ? "NORMAL" : "SLOW") : "---");
            telemetry.addData("", "");
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("Left Bumper", "Boost Mode (100% speed)");
            telemetry.addData("Right Bumper", "Normal Intake");
            telemetry.addData("DPAD Down", "Outtake");
            telemetry.addData("DPAD Up", "Shoot (servo + intake)");
            telemetry.addData("X", "Toggle Dynamic/Fixed Mode");
            telemetry.addData("Triangle", "Switch: IMU Tracking ↔ 0° Position");
            telemetry.addData("Options", "Reset IMU Heading");
            telemetry.update();
        }

        // Stop shooter when match ends
        shooter.stop();
    }
}