package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretSystem {
    private final DcMotor turret;
    private static final int TICKS_PER_ROTATION = 1879;
    private static final double TICKS_PER_DEGREE = (double) TICKS_PER_ROTATION / 360.0;

    private static final double MIN_TURRET_ANGLE = -210.0;
    private static final double MAX_TURRET_ANGLE = 210.0;

    // Increased deadband to prevent drift
    private static final int POSITION_TOLERANCE_TICKS = 10; // Increased from 5

    // Power reduction when close to target
    private static final double HOLDING_POWER = 0.15; // Low power to hold position
    private static final double APPROACH_POWER_SCALE = 0.5; // Reduce power when close

    private double targetAngle = 0.0;
    private boolean isHolding = false;

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

        // Only update if the angle actually changed significantly
        if (Math.abs(angleDegrees - targetAngle) < 0.5) {
            return; // Skip update if change is tiny
        }

        // Store the target angle
        targetAngle = angleDegrees;
        isHolding = false;

        // Convert to ticks
        int targetTicks = (int) Math.round(angleDegrees * TICKS_PER_DEGREE);

        // Send command to motor
        turret.setTargetPosition(targetTicks);
        turret.setPower(Math.abs(power));
    }

    /**
     * CRITICAL: Call this every loop to prevent drift!
     */
    public void update() {
        int currentPos = turret.getCurrentPosition();
        int targetPos = turret.getTargetPosition();
        int error = Math.abs(targetPos - currentPos);

        if (error <= POSITION_TOLERANCE_TICKS) {
            // Within deadband - use minimal holding power
            if (!isHolding) {
                turret.setPower(HOLDING_POWER);
                isHolding = true;
            }
        } else if (error < 50) {
            // Close to target - reduce power to prevent overshoot
            double currentPower = turret.getPower();
            turret.setPower(currentPower * APPROACH_POWER_SCALE);
            isHolding = false;
        } else {
            // Far from target - maintain commanded power
            isHolding = false;
        }
    }

    public void resetToHome() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        targetAngle = 0.0;
        isHolding = false;
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

    public DcMotor getMotor() {
        return turret;
    }
}