package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.TurretSystem;

@TeleOp(name = "Turret Calibration", group = "Calibration")
public class TurretCalibration extends LinearOpMode {

    TurretSystem turret;

    @Override
    public void runOpMode() {
        turret = new TurretSystem(hardwareMap, telemetry);

        telemetry.addLine("Turret Calibration");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        double testAngle = 0;

        while (opModeIsActive()) {
            // Right stick X rotates slowly in real time
            testAngle += gamepad1.right_stick_x * 0.5;

            turret.moveToAngle(testAngle);
            turret.update();

            telemetry.addData("--- CALIBRATION ---", "");
            telemetry.addData("Commanded Angle", "%.1f°", testAngle);
            telemetry.addData("Actual Angle", "%.1f°", turret.getCurrentAngle());
            telemetry.addData("", "");
            telemetry.addData("INSTRUCTIONS", "");
            telemetry.addData("", "Use right stick X to rotate.");
            telemetry.addData("", "Slowly go CW until cables block.");
            telemetry.addData("", "Note the angle -> that is your MAX.");
            telemetry.addData("", "Then go CCW until cables block.");
            telemetry.addData("", "Note that angle -> that is your MIN.");
            telemetry.addData("", "");
            telemetry.addData("", "Put those two values into");
            telemetry.addData("", "MIN_TURRET_ANGLE and MAX_TURRET_ANGLE");
            telemetry.addData("", "in TurretSystem.java");
            telemetry.update();
        }
    }
}