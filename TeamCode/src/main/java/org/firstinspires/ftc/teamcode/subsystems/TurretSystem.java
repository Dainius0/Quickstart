package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSystem {
    private final DcMotor turret;
    private final Telemetry telemetry;
    private static final int TICKS_PER_ROTATION = 1879;
    private static final double TICKS_PER_DEGREE = (double) TICKS_PER_ROTATION / 360.0;

    private static final double RUN_TO_POSITION_POWER = 0.8;

    // The usable range. The dead zone (cables) is the gap between these two.
    // If you have ~37 degrees of dead space, total usable is ~323.
    // Centered around 0 that gives you roughly -161.5 to +161.5.
    // Adjust these if needed once you confirm the dead zone size.
    private static final double MIN_TURRET_ANGLE = -180;
    private static final double MAX_TURRET_ANGLE = 180;

    private double targetAngle = 0.0;

    public TurretSystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        turret = hwMap.dcMotor.get("turretRotator");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0); // Start with no power - only apply when moving
    }

    /**
     * Wraps an angle into the range (-180, 180].
     * Example:  181  ->  -179
     * Example: -200  ->   160
     * This is what prevents the turret from trying to go the long way around.
     */
    private static double wrapAngle(double angle) {
        while (angle > 180.0)  angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Clamps the angle to the physical travel range AFTER wrapping.
     * If the wrapped angle lands in the dead zone (beyond ±161.5),
     * it snaps to whichever limit is closer.
     */
    private static double clampToRange(double angle) {
        if (angle > MAX_TURRET_ANGLE) return MAX_TURRET_ANGLE;
        if (angle < MIN_TURRET_ANGLE) return MIN_TURRET_ANGLE;
        return angle;
    }

    public void moveToAngle(double angleDegrees) {
        // 1. Wrap into (-180, 180] so we never try to go the long way around
        angleDegrees = wrapAngle(angleDegrees);

        // 2. Clamp to physical cable limits
        angleDegrees = clampToRange(angleDegrees);

        // 3. Deadband — don't bother the motor for tiny changes
        if (Math.abs(angleDegrees - targetAngle) < 0.5) {
            return;
        }

        targetAngle = angleDegrees;
        int targetTicks = (int) Math.round(angleDegrees * TICKS_PER_DEGREE);
        turret.setTargetPosition(targetTicks);
        turret.setPower(RUN_TO_POSITION_POWER); // Apply power when we have a new target
    }

    public void moveToAngle(double angleDegrees, double power) {
        moveToAngle(angleDegrees);
    }

    public void update() {
        // Check if we're at target and cut power to prevent drift
        if (isAtTarget(2.0)) { // Within 2 degrees
            turret.setPower(0);
        }
    }

    public void printTelemetry() {
        telemetry.addData("=== TURRET ===", "");
        telemetry.addData("Target", "%.1f°", targetAngle);
        telemetry.addData("Current", "%.1f°", getCurrentAngle());
        telemetry.addData("Motor Power", "%.2f", turret.getPower());
        telemetry.addData("Limits", "%.1f° to %.1f°", MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    }

    public void resetToHome() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0); // Don't apply power until we get a move command
        targetAngle = 0.0;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getCurrentAngle() {
        return turret.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public boolean isAtTarget(double toleranceDegrees) {
        return Math.abs(getCurrentAngle() - targetAngle) < toleranceDegrees;
    }

    public boolean isCalibrating() {
        return false;
    }

    public DcMotor getMotor() {
        return turret;
    }
}