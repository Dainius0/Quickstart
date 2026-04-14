package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSystem {
    private final DcMotorEx shooter1, shooter2;
    private final Servo shooterServo, shooterServo2;
    private final Telemetry telemetry;   // may be null — all usages are null-safe

    // Target velocities in ticks/sec (28 CPR, no gearbox)
    public static final double VELOCITY_DYNAMIC   = 1150;
    public static final double VELOCITY_FIXED     = 1550;
    private static final double VELOCITY_TOLERANCE = 15;

    // Confirmed true max ticks/sec at full power
    private static final double MAX_VELOCITY = 2300;

    // Software PID — feedforward is simply targetVelocity / MAX_VELOCITY
    private static final double kP = 0.006;
    private static final double kI = 0.0000;
    private static final double kD = 0.00;

    // PID state — separate per motor
    private double integral1 = 0, integral2 = 0;
    private double lastError1 = 0, lastError2 = 0;
    private double power1 = 0, power2 = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private boolean running = false;
    private double targetVelocity = 0;

    private static final double SERVO_HOME   = 0.85;
    private static final double SERVO_SHOOT  = 0.62;
    private static final double SERVO2_HOME  = 0.47;
    private static final double SERVO2_SHOOT = 0.70;

    // =========================================================================
    // Constructors
    // =========================================================================

    /**
     * Primary constructor (Blue standard) — pass telemetry for addTelemetry() support.
     */
    public ShooterSystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shooter1 = hwMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hwMap.get(DcMotorEx.class, "shooter2");
        shooterServo  = hwMap.servo.get("shooterServo");
        shooterServo2 = hwMap.servo.get("shooterServo2");

        // RUN_WITHOUT_ENCODER — power set manually, velocity still readable via getVelocity()
        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorEx.Direction.REVERSE);

        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterServo.setPosition(SERVO_HOME);
        shooterServo2.setPosition(SERVO2_HOME);

        pidTimer.reset();
    }

    /**
     * Convenience constructor (Red / legacy) — no telemetry.
     * Chains to the primary constructor with null.
     * addTelemetry() calls are silently skipped.
     */
    public ShooterSystem(HardwareMap hwMap) {
        this(hwMap, null);
    }

    // =========================================================================
    // Velocity control
    // =========================================================================

    /** Call once to set the target velocity. Then call update() every loop. */
    public void setVelocity(boolean dynamicMode) {
        targetVelocity = dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
        running    = true;
        integral1  = 0;
        integral2  = 0;
        lastError1 = 0;
        lastError2 = 0;
        power1 = targetVelocity / MAX_VELOCITY;
        power2 = targetVelocity / MAX_VELOCITY;
        shooter1.setPower(power1);
        shooter2.setPower(power2);
        pidTimer.reset();
    }

    /**
     * MUST be called every loop iteration while the shooter is running.
     * Runs the software PID and updates motor power.
     */
    public void update() {
        if (!running) return;

        double dt = pidTimer.seconds();
        if (dt < 0.005) return; // skip if called faster than 5 ms
        pidTimer.reset();

        double v1 = Math.abs(shooter1.getVelocity());
        double v2 = Math.abs(shooter2.getVelocity());

        double error1 = targetVelocity - v1;
        double error2 = targetVelocity - v2;

        // Integral with anti-windup clamp
        integral1 = clamp(integral1 + error1 * dt, -0.3, 0.3);
        integral2 = clamp(integral2 + error2 * dt, -0.3, 0.3);

        // Derivative
        double derivative1 = (error1 - lastError1) / dt;
        double derivative2 = (error2 - lastError2) / dt;
        lastError1 = error1;
        lastError2 = error2;

        double ff = targetVelocity / MAX_VELOCITY;
        power1 = clamp(ff + kP * error1 + kI * integral1 + kD * derivative1, 0.0, 1.0);
        power2 = clamp(ff + kP * error2 + kI * integral2 + kD * derivative2, 0.0, 1.0);

        shooter1.setPower(power1);
        shooter2.setPower(power2);
    }

    public void stop() {
        running        = false;
        targetVelocity = 0;
        integral1      = 0;
        integral2      = 0;
        power1         = 0;
        power2         = 0;
        shooter1.setPower(0);
        shooter2.setPower(0);
        shooterServo.setPosition(SERVO_HOME);
        shooterServo2.setPosition(SERVO2_HOME);
    }

    // =========================================================================
    // Servo control
    // =========================================================================

    public void shoot() {
        shooterServo.setPosition(SERVO_SHOOT);
        shooterServo2.setPosition(SERVO2_SHOOT);
    }

    public void homeServo() {
        shooterServo.setPosition(SERVO_HOME);
        shooterServo2.setPosition(SERVO2_HOME);
    }

    // =========================================================================
    // Getters
    // =========================================================================

    public double getVelocity() {
        return (Math.abs(shooter1.getVelocity()) + Math.abs(shooter2.getVelocity())) / 2.0;
    }

    public double getVelocity1() { return Math.abs(shooter1.getVelocity()); }
    public double getVelocity2() { return Math.abs(shooter2.getVelocity()); }

    public boolean isAtTargetVelocity(boolean dynamicMode) {
        double target = dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
        return Math.abs(getVelocity1() - target) < VELOCITY_TOLERANCE &&
                Math.abs(getVelocity2() - target) < VELOCITY_TOLERANCE;
    }

    public boolean isMotor1AtTarget(boolean dynamicMode) {
        double target = dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
        return Math.abs(getVelocity1() - target) < VELOCITY_TOLERANCE;
    }

    public boolean isMotor2AtTarget(boolean dynamicMode) {
        double target = dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
        return Math.abs(getVelocity2() - target) < VELOCITY_TOLERANCE;
    }

    public double getTargetVelocity(boolean dynamicMode) {
        return dynamicMode ? VELOCITY_DYNAMIC : VELOCITY_FIXED;
    }

    public double getServo1Position() { return shooterServo.getPosition(); }
    public double getServo2Position() { return shooterServo2.getPosition(); }

    // =========================================================================
    // Telemetry (null-safe — silently skipped when constructed without telemetry)
    // =========================================================================

    /**
     * Add shooter data to telemetry. Call inside your OpMode's telemetry block.
     * Does NOT call telemetry.update() — let the OpMode do that.
     * No-ops silently if this instance was created without a Telemetry reference.
     */
    public void addTelemetry(boolean dynamicMode) {
        if (telemetry == null) return;

        double v1 = getVelocity1();
        double v2 = getVelocity2();

        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("Mode",    dynamicMode ? "DYNAMIC" : "FIXED");
        telemetry.addData("Target",  "%.0f ticks/sec", targetVelocity);
        telemetry.addData("Motor 1", "%.0f | err: %.0f | pwr: %.3f", v1, v1 - targetVelocity, power1);
        telemetry.addData("Motor 2", "%.0f | err: %.0f | pwr: %.3f", v2, v2 - targetVelocity, power2);
        telemetry.addData("Average", "%.0f ticks/sec", getVelocity());
        telemetry.addData("Ready",   isAtTargetVelocity(dynamicMode) ? "YES" : "NO");
        telemetry.addData("FF pwr",  "%.3f", targetVelocity / MAX_VELOCITY);
        telemetry.addData("kP",      "%.4f", kP);
    }

    // =========================================================================
    // Utility
    // =========================================================================

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}