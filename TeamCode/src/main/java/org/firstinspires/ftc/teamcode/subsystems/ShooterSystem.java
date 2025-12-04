package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterSystem {
    private final DcMotorEx shooter;
    private final Servo shooterServo, shooterAngle;

    public static final double VELOCITY_DYNAMIC = 1500;
    public static final double VELOCITY_FIXED = 2100;
    private static final double VELOCITY_TOLERANCE = 30;

    private static final double SERVO_HOME = 0.8;
    private static final double SERVO_SHOOT = 0.2;
    private static final double ANGLE_HOME = 0.59;
    private static final double ANGLE_DYNAMIC = 0.59;
    private static final double ANGLE_FIXED = 0.65;

    public ShooterSystem(HardwareMap hwMap) {
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        shooterServo = hwMap.servo.get("shooterServo");
        shooterAngle = hwMap.servo.get("shooterAngle");

        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Improved PIDF for faster spin-up
        PIDFCoefficients pidf = new PIDFCoefficients(
                25,      // P - increased for faster response
                0.045,   // I - helps reach target faster
                1,       // D - reduces overshoot
                12.5     // F - feedforward
        );
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        shooterServo.setPosition(SERVO_HOME);
        shooterAngle.setPosition(ANGLE_HOME);
    }

    // Separate methods - don't touch servo in spinUp
    public void setVelocity(boolean dynamicMode) {
        double velocity = dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
        shooter.setVelocity(velocity);
    }

    public void setAngle(boolean dynamicMode) {
        shooterAngle.setPosition(dynamicMode ? ANGLE_DYNAMIC : ANGLE_FIXED);
    }

    // Legacy method for compatibility
    public void spinUp(boolean dynamicMode) {
        setVelocity(dynamicMode);
        setAngle(dynamicMode);
    }

    public void stop() {
        shooter.setVelocity(0);
        shooterServo.setPosition(SERVO_HOME);
        shooterAngle.setPosition(ANGLE_HOME);
    }

    public double getVelocity() {
        return Math.abs(shooter.getVelocity());
    }

    public boolean isAtTargetVelocity(boolean dynamicMode) {
        double target = dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
        double v = getVelocity();
        return Math.abs(v - target) < VELOCITY_TOLERANCE;
    }

    public void shoot() {
        shooterServo.setPosition(SERVO_SHOOT);
    }

    public void homeServo() {
        shooterServo.setPosition(SERVO_HOME);
    }
}