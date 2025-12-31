package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterSystem {
    private final DcMotorEx shooter1, shooter2;
    private final Servo shooterServo, shooterServo2, shooterAngle;

    public static final double VELOCITY_DYNAMIC = 1480;
    public static final double VELOCITY_FIXED = 1900;
    private static final double VELOCITY_TOLERANCE = 30;

    private static final double SERVO_HOME = 0.85;
    private static final double SERVO_SHOOT = 0.62;
    private static final double SERVO2_HOME = 0.47;  // Reversed from SERVO_HOME (1.0 - 0.8)
    private static final double SERVO2_SHOOT = 0.70; // Reversed from SERVO_SHOOT (1.0 - 0.5)
    private static final double ANGLE_HOME = 0.59;
    private static final double ANGLE_DYNAMIC = 0.59;
    private static final double ANGLE_FIXED = 0.65;

    public ShooterSystem(HardwareMap hwMap) {
        shooter1 = hwMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hwMap.get(DcMotorEx.class, "shooter2");
        shooterServo = hwMap.servo.get("shooterServo");
        shooterServo2 = hwMap.servo.get("shooterServo2");
        shooterAngle = hwMap.servo.get("shooterAngle");

        // Initialize both motors
        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Improved PIDF for faster spin-up (apply to both motors)
        PIDFCoefficients pidf = new PIDFCoefficients(
                25,      // P - increased for faster response
                0.045,   // I - helps reach target faster
                1,       // D - reduces overshoot
                12.5     // F - feedforward
        );
        shooter1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        shooterServo.setPosition(SERVO_HOME);
        shooterServo2.setPosition(SERVO2_HOME);
        shooterAngle.setPosition(ANGLE_HOME);
    }

    public void setVelocity(boolean dynamicMode) {
        double velocity = dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
        // Motors spin in same direction
        shooter1.setVelocity(velocity);
        shooter2.setVelocity(velocity);
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
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
        shooterServo.setPosition(SERVO_HOME);
        shooterServo2.setPosition(SERVO2_HOME);
        shooterAngle.setPosition(ANGLE_HOME);
    }

    /**
     * Get average velocity of both shooter motors
     */
    public double getVelocity() {
        return (Math.abs(shooter1.getVelocity()) + Math.abs(shooter2.getVelocity())) / 2.0;
    }

    /**
     * Get velocity of shooter motor 1
     */
    public double getVelocity1() {
        return Math.abs(shooter1.getVelocity());
    }

    /**
     * Get velocity of shooter motor 2
     */
    public double getVelocity2() {
        return Math.abs(shooter2.getVelocity());
    }

    /**
     * Check if both motors are at target velocity
     */
    public boolean isAtTargetVelocity(boolean dynamicMode) {
        double target = dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
        double v1 = Math.abs(shooter1.getVelocity());
        double v2 = Math.abs(shooter2.getVelocity());
        return Math.abs(v1 - target) < VELOCITY_TOLERANCE &&
                Math.abs(v2 - target) < VELOCITY_TOLERANCE;
    }

    /**
     * Check if motor 1 is at target velocity
     */
    public boolean isMotor1AtTarget(boolean dynamicMode) {
        double target = dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
        double v1 = Math.abs(shooter1.getVelocity());
        return Math.abs(v1 - target) < VELOCITY_TOLERANCE;
    }

    /**
     * Check if motor 2 is at target velocity
     */
    public boolean isMotor2AtTarget(boolean dynamicMode) {
        double target = dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
        double v2 = Math.abs(shooter2.getVelocity());
        return Math.abs(v2 - target) < VELOCITY_TOLERANCE;
    }

    /**
     * Get target velocity based on mode
     */
    public double getTargetVelocity(boolean dynamicMode) {
        return dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
    }

    public void shoot() {
        shooterServo.setPosition(SERVO_SHOOT);
        shooterServo2.setPosition(SERVO2_SHOOT);
    }

    public void homeServo() {
        shooterServo.setPosition(SERVO_HOME);
        shooterServo2.setPosition(SERVO2_HOME);
    }

    /**
     * Get current angle servo position
     */
    public double getAnglePosition() {
        return shooterAngle.getPosition();
    }

    /**
     * Get current shooter servo 1 position
     */
    public double getServo1Position() {
        return shooterServo.getPosition();
    }

    /**
     * Get current shooter servo 2 position
     */
    public double getServo2Position() {
        return shooterServo2.getPosition();
    }
}