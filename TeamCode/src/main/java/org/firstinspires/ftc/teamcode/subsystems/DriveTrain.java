package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveTrain {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private GoBildaPinpointDriver pinpoint;

    private static final double STRAFE_ROTATION_COMPENSATION = 0;
    private static final double INPUT_DEADBAND = 0.01;

    // ── Counter-Spin Braking ──────────────────────────────────────────────────
    private static final double BRAKE_POWER = 0.5;
    private static final long BRAKE_DURATION_MS = 100;
    private static final double BRAKE_TRIGGER_THRESHOLD = 0.08;

    private boolean isBraking = false;
    private long brakeStartTime = 0;
    private double lastFL = 0, lastBL = 0, lastFR = 0, lastBR = 0;
    // ─────────────────────────────────────────────────────────────────────────

    // ── Heading Lock PID ──────────────────────────────────────────────────────
    private static final double HEADING_KP = 0.01;
    private static final double HEADING_KD = 0.01;
    private static final double HEADING_LOCK_DEADBAND = 7;
    private static final double HEADING_MAX_CORRECTION = 1;
    private static final double ROTATION_INPUT_DEADBAND = 0.02;
    private static final double ROTATION_SPEED_MULTIPLIER = 1.0;
    private static final double SLOW_MODE_MULTIPLIER = 0.4;

    private double lockedHeading = 0;
    private boolean headingLockActive = false;
    private double lastHeadingError = 0;
    private long lastPIDTime = 0;
    // ─────────────────────────────────────────────────────────────────────────

    private double cachedHeadingDegrees = 0;
    private double headingOffset = 0;

    // ── Drift Reduction ───────────────────────────────────────────────────────
    private static final double DRIFT_REDUCTION_DEGREES = 0;
    private static final long DRIFT_REDUCTION_INTERVAL_MS = 1000;
    private long lastDriftReductionTime = 0;
    // ─────────────────────────────────────────────────────────────────────────

    public DriveTrain(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    public DriveTrain(HardwareMap hardwareMap, boolean resetYaw) {
        frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor   = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor  = hardwareMap.dcMotor.get("backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        if (resetYaw) {
            pinpoint.recalibrateIMU();
        }

        headingOffset = 0;
        cachedHeadingDegrees = 0;
        lastDriftReductionTime = System.currentTimeMillis();
        lastPIDTime = System.currentTimeMillis();
    }

    /**
     * Call ONCE at the top of every loop iteration.
     */
    public void update() {
        pinpoint.update();
        double rawHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        cachedHeadingDegrees = AngleUnit.normalizeDegrees(rawHeading - headingOffset);

        long now = System.currentTimeMillis();
        if (now - lastDriftReductionTime >= DRIFT_REDUCTION_INTERVAL_MS) {
            headingOffset += DRIFT_REDUCTION_DEGREES;
            cachedHeadingDegrees = AngleUnit.normalizeDegrees(rawHeading - headingOffset);
            lastDriftReductionTime = now;
        }

        if (isBraking && now - brakeStartTime >= BRAKE_DURATION_MS) {
            isBraking = false;
            stopMotors();
        }
    }

    public void drive(double x, double y, double rx, boolean slowMode) {
        if (slowMode) {
            headingLockActive = false;
            isBraking = false;
            double botHeadingRad = Math.toRadians(cachedHeadingDegrees);
            double rotX = (x * Math.cos(-botHeadingRad) - y * Math.sin(-botHeadingRad)) * 1.1 * SLOW_MODE_MULTIPLIER;
            double rotY = (x * Math.sin(-botHeadingRad) + y * Math.cos(-botHeadingRad)) * SLOW_MODE_MULTIPLIER;
            double effectiveRx = (rx * ROTATION_SPEED_MULTIPLIER) * SLOW_MODE_MULTIPLIER;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(effectiveRx), 1);
            lastFL = (rotY + rotX + effectiveRx) / denominator;
            lastBL = (rotY - rotX + effectiveRx) / denominator;
            lastFR = (rotY - rotX - effectiveRx) / denominator;
            lastBR = (rotY + rotX - effectiveRx) / denominator;
            frontLeftMotor.setPower(lastFL);
            backLeftMotor.setPower(lastBL);
            frontRightMotor.setPower(lastFR);
            backRightMotor.setPower(lastBR);
            return;
        }

        boolean noTranslation = Math.abs(x) < INPUT_DEADBAND && Math.abs(y) < INPUT_DEADBAND;
        boolean noRotation    = Math.abs(rx) < ROTATION_INPUT_DEADBAND;
        boolean noInput       = noTranslation && Math.abs(rx) < INPUT_DEADBAND;

        if (noInput) {
            if (!isBraking) {
                double maxLastPower = Math.max(
                        Math.max(Math.abs(lastFL), Math.abs(lastBL)),
                        Math.max(Math.abs(lastFR), Math.abs(lastBR))
                );
                if (maxLastPower >= BRAKE_TRIGGER_THRESHOLD) {
                    frontLeftMotor.setPower( -lastFL * BRAKE_POWER);
                    backLeftMotor.setPower(  -lastBL * BRAKE_POWER);
                    frontRightMotor.setPower(-lastFR * BRAKE_POWER);
                    backRightMotor.setPower( -lastBR * BRAKE_POWER);
                    isBraking = true;
                    brakeStartTime = System.currentTimeMillis();
                } else {
                    stopMotors();
                }
            }
            if (!isBraking) {
                headingLockActive = true;
                lockedHeading = cachedHeadingDegrees;
            }
            return;
        }

        isBraking = false;

        if (noRotation) {
            if (!headingLockActive) {
                headingLockActive = true;
                lockedHeading = cachedHeadingDegrees;
                lastHeadingError = 0;
                lastPIDTime = System.currentTimeMillis();
            }
        } else {
            headingLockActive = false;
            lockedHeading = cachedHeadingDegrees;
            lastHeadingError = 0;
        }

        double headingCorrection = 0;
        if (headingLockActive) {
            double error = AngleUnit.normalizeDegrees(lockedHeading - cachedHeadingDegrees);

            if (Math.abs(error) > HEADING_LOCK_DEADBAND) {
                long now = System.currentTimeMillis();
                double dt = Math.max((now - lastPIDTime) / 1000.0, 0.001);

                double derivative = (error - lastHeadingError) / dt;
                headingCorrection = (HEADING_KP * error) + (HEADING_KD * derivative);
                headingCorrection = Math.max(-HEADING_MAX_CORRECTION,
                        Math.min( HEADING_MAX_CORRECTION, headingCorrection));

                lastHeadingError = error;
                lastPIDTime = now;
            } else {
                lastHeadingError = 0;
                lastPIDTime = System.currentTimeMillis();
            }
        }

        double botHeadingRad = Math.toRadians(cachedHeadingDegrees);

        double rotX = x * Math.cos(-botHeadingRad) - y * Math.sin(-botHeadingRad);
        double rotY = x * Math.sin(-botHeadingRad) + y * Math.cos(-botHeadingRad);

        rotX = rotX * 1.1;

        double effectiveRx = (rx * ROTATION_SPEED_MULTIPLIER) + headingCorrection + (rotX * STRAFE_ROTATION_COMPENSATION);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(effectiveRx), 1);

        lastFL = (rotY + rotX + effectiveRx) / denominator;
        lastBL = (rotY - rotX + effectiveRx) / denominator;
        lastFR = (rotY - rotX - effectiveRx) / denominator;
        lastBR = (rotY + rotX - effectiveRx) / denominator;

        frontLeftMotor.setPower(lastFL);
        backLeftMotor.setPower(lastBL);
        frontRightMotor.setPower(lastFR);
        backRightMotor.setPower(lastBR);
    }

    public void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        lastFL = 0; lastBL = 0; lastFR = 0; lastBR = 0;
    }

    public void resetHeading() {
        pinpoint.update();
        headingOffset = pinpoint.getHeading(AngleUnit.DEGREES);
        cachedHeadingDegrees = 0;
        lockedHeading = 0;
        headingLockActive = false;
        lastHeadingError = 0;
        lastDriftReductionTime = System.currentTimeMillis();
        lastPIDTime = System.currentTimeMillis();
    }

    public double getHeading()            { return cachedHeadingDegrees; }
    public double getLockedHeading()      { return lockedHeading; }
    public boolean isHeadingLockActive()  { return headingLockActive; }
    public GoBildaPinpointDriver getPinpoint() { return pinpoint; }

    /**
     * Field X position in inches from the Pinpoint odometry pod.
     * Pinpoint returns position in mm — divide by 25.4 to convert to inches.
     *
     * NOTE: if driving toward the red wall increases X instead of Y,
     * swap getPosX() and getPosY() in both methods to match your pod orientation.
     */
    public double getFieldX() { return pinpoint.getPosX(DistanceUnit.INCH); }

    /**
     * Field Y position in inches from the Pinpoint odometry pod.
     */
    public double getFieldY() { return pinpoint.getPosY(DistanceUnit.INCH); }
}