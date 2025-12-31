package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretSystem {
    private final DcMotor turret;
    private static final int TICKS_PER_ROTATION = 1879;
    private static final double TICKS_PER_DEGREE = (double) TICKS_PER_ROTATION / 360.0;

    private static final double MIN_TURRET_ANGLE = -180.0;
    private static final double MAX_TURRET_ANGLE = 180.0;

    // Store the precise target angle as a double to avoid drift
    private double preciseTargetAngle = 0.0;

    // Track accumulated rounding error
    private double accumulatedError = 0.0;

    public TurretSystem(HardwareMap hwMap) {
        turret = hwMap.dcMotor.get("turretRotator");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveToAngle(double angleDegrees, double power) {
        // Clamp the angle to valid range
        angleDegrees = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, angleDegrees));

        // Update our precise target
        preciseTargetAngle = angleDegrees;

        // Convert to ticks with accumulated error compensation
        double exactTicks = preciseTargetAngle * TICKS_PER_DEGREE + accumulatedError;
        int targetTicks = (int) Math.round(exactTicks);

        // Update accumulated error (difference between what we wanted and what we're commanding)
        accumulatedError = exactTicks - targetTicks;

        // Send command to motor
        turret.setTargetPosition(targetTicks);
        turret.setPower(Math.abs(power));
    }

    /**
     * Reset the turret to home position (0 degrees) and clear accumulated error
     * Call this when you manually reset or home the turret
     */
    public void resetToHome() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        preciseTargetAngle = 0.0;
        accumulatedError = 0.0;
    }

    /**
     * Get the current target angle (the precise value, not the rounded tick value)
     */
    public double getTargetAngle() {
        return preciseTargetAngle;
    }

    /**
     * Get the current actual angle based on encoder position
     */
    public double getCurrentAngle() {
        return turret.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Check if turret is close to target position
     */
    public boolean isAtTarget(double toleranceDegrees) {
        return Math.abs(getCurrentAngle() - preciseTargetAngle) < toleranceDegrees;
    }
}