package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue_Auto_21")
public class Blue_Auto_21 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer, shootingTimer;

    // =========================================================================
    // ██  TUNE THESE — SHOOTER  ███████████████████████████████████████████████
    // =========================================================================
    private static final double TARGET_VELOCITY    = 1150;  // ticks/sec — lower = shorter shot
    private static final double MAX_VELOCITY       = 2300;  // confirmed max at full power
    private static final double VELOCITY_TOLERANCE = 15;
    private static final double SHOOTER_KP         = 0.006;
    private static final double SHOOTER_KI         = 0.000;
    private static final double SHOOTER_KD         = 0.000;

    private static final double SERVO_HOME    = 0.85;
    private static final double SERVO_SHOOT   = 0.62;
    private static final double SERVO2_HOME   = 0.47;
    private static final double SERVO2_SHOOT  = 0.70;
    // =========================================================================

    // =========================================================================

    // =========================================================================
    private static final double TURRET_TARGET_ANGLE = -46.0; // degrees — BLUE side negative
    private static final double TURRET_DEADBAND     =  1.0;  // degrees
    // =========================================================================

    // Hardware — shooter
    private DcMotorEx shooter1, shooter2;
    private Servo     shooterServo, shooterServo2;

    // Shooter PID state
    private double  integral1, integral2;
    private double  lastError1, lastError2;
    private double  shooterPower1, shooterPower2;
    private boolean shooterRunning = false;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // Hardware — other
    private DcMotor               intakeMotor, turretRotator;
    private GoBildaPinpointDriver pinpoint;

    // Turret constants
    private static final int    TURRET_TICKS_PER_ROTATION = 1872;
    private static final double TURRET_TICKS_PER_DEGREE   = (double) TURRET_TICKS_PER_ROTATION / 360.0;
    private static final double MIN_TURRET_ANGLE          = -90.0;
    private static final double MAX_TURRET_ANGLE          =  90.0;
    private double lastTurretCommandedAngle = Double.MAX_VALUE;

    // Timing
    private static final long CONTINUOUS_SHOOT_DURATION = 900;

    // Drive power
    private static final double INTAKE_DRIVE_POWER = 1.0;
    private static final double NORMAL_DRIVE_POWER  = 1.0;

    // State
    private ShootingState shootingState = ShootingState.IDLE;
    private boolean intakeActive = false;

    public enum PathState {
        DRIVE_TO_SHOOT_POS,     // 1. Start → Shoot position
        SHOOT_PRELOAD,          // 2. Shoot pre-loaded balls
        DRIVE_TO_INTAKE_1,      // 3. Shoot pos → Intake 1
        RETURN_FROM_INTAKE_1,   // 4. Intake 1 → Shoot pos
        SHOOT_ROUND_1,          // 5. Shoot collected balls
        COMPLETE
    }
    public enum ShootingState { IDLE, SHOOTING }

    PathState pathState;

    // BLUE SIDE Poses
    private final Pose startPose      = new Pose(36, 130, Math.toRadians(180));
    private final Pose shootPose      = new Pose(48,  88, Math.toRadians(180));
    private final Pose intakeEndPoint = new Pose(11,  59, Math.toRadians(180));

    // Paths
    private PathChain driveToShootPos;
    private PathChain driveToIntake1;
    private PathChain returnFromIntake1;

    // =========================================================================
    // PATH BUILDING
    // =========================================================================
    public void buildPaths() {
        // Start → Shoot position (straight line)
        driveToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Shoot position → Intake 1 (curved to avoid field elements)
        Pose c1_intake = new Pose(48, 62, Math.toRadians(180));
        Pose c2_intake = new Pose(50.2, 59.7, Math.toRadians(180));
        driveToIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c1_intake, c2_intake, intakeEndPoint))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeEndPoint.getHeading())
                .build();

        // Intake 1 → Shoot position (curved back)
        Pose c1_return = new Pose(34, 68, Math.toRadians(180));
        Pose c2_return = new Pose(47, 74.7, Math.toRadians(180));
        returnFromIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(intakeEndPoint, c1_return, c2_return, shootPose))
                .setLinearHeadingInterpolation(intakeEndPoint.getHeading(), shootPose.getHeading())
                .build();
    }

    // =========================================================================
    // STATE MACHINE
    // =========================================================================
    public void statePathUpdate() {
        switch (pathState) {

            // ── 1. Drive from start to shoot position ─────────────────────────
            case DRIVE_TO_SHOOT_POS:
                shooterStart();
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(driveToShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            // ── 2. Shoot pre-loaded balls ──────────────────────────────────────
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startContinuousShooting();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_1);
                }
                telemetry.addData("Cycle", "Preload - Shooting");
                break;

            // ── 3. Drive to intake 1 while running intake ──────────────────────
            case DRIVE_TO_INTAKE_1:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake1, true);
                    setPathState(PathState.RETURN_FROM_INTAKE_1);
                }
                telemetry.addData("Cycle", "1 - To Intake 1");
                break;

            // ── 4. Return from intake 1 to shoot position ──────────────────────
            case RETURN_FROM_INTAKE_1:
                if (!follower.isBusy()) {
                    stopIntake();
                    shooterStart();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(returnFromIntake1, true);
                    setPathState(PathState.SHOOT_ROUND_1);
                }
                telemetry.addData("Cycle", "1 - Returning");
                break;

            // ── 5. Shoot collected balls ───────────────────────────────────────
            case SHOOT_ROUND_1:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startContinuousShooting();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        moveTurretToAngle(0, 0.8);
                        setPathState(PathState.COMPLETE);
                    }
                }
                telemetry.addData("Cycle", "1 - Shooting");
                break;

            // ── 6. Done ────────────────────────────────────────────────────────
            case COMPLETE:
                shooterStop();
                stopIntake();
                telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
                break;

            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    // =========================================================================
    // TURRET — continuously tracks target heading every loop
    // =========================================================================
    private void updateTurret() {
        if (pathState == PathState.COMPLETE) return;

        double imuHeading    = Math.toDegrees(follower.getPose().getHeading());
        double relativeHeading = AngleUnit.normalizeDegrees(imuHeading - 180);
        double targetAngle   = TURRET_TARGET_ANGLE - relativeHeading;
        targetAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, targetAngle));

        if (Math.abs(targetAngle - lastTurretCommandedAngle) > TURRET_DEADBAND) {
            moveTurretToAngle(targetAngle, 1.0);
            lastTurretCommandedAngle = targetAngle;
        }

        telemetry.addData("Turret Target", "%.1f°", targetAngle);
        telemetry.addData("IMU Heading",   "%.1f°", imuHeading);
    }

    private void moveTurretToAngle(double angleDegrees, double power) {
        int ticks = (int) Math.round(angleDegrees * TURRET_TICKS_PER_DEGREE);
        turretRotator.setTargetPosition(ticks);
        turretRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretRotator.setPower(Math.abs(power));
    }

    // =========================================================================
    // SHOOTER — inlined PID
    // =========================================================================
    private void shooterStart() {
        if (shooterRunning) return;
        shooterRunning = true;
        integral1 = integral2 = 0;
        lastError1 = lastError2 = 0;
        shooterPower1 = shooterPower2 = TARGET_VELOCITY / MAX_VELOCITY;
        shooter1.setPower(shooterPower1);
        shooter2.setPower(shooterPower2);
        pidTimer.reset();
    }

    private void shooterUpdate() {
        if (!shooterRunning) return;
        double dt = pidTimer.seconds();
        if (dt < 0.005) return;
        pidTimer.reset();

        double v1 = Math.abs(shooter1.getVelocity());
        double v2 = Math.abs(shooter2.getVelocity());
        double e1 = TARGET_VELOCITY - v1;
        double e2 = TARGET_VELOCITY - v2;

        integral1 = clamp(integral1 + e1 * dt, -0.3, 0.3);
        integral2 = clamp(integral2 + e2 * dt, -0.3, 0.3);

        double d1 = (e1 - lastError1) / dt;
        double d2 = (e2 - lastError2) / dt;
        lastError1 = e1;
        lastError2 = e2;

        double ff     = TARGET_VELOCITY / MAX_VELOCITY;
        shooterPower1 = clamp(ff + SHOOTER_KP * e1 + SHOOTER_KI * integral1 + SHOOTER_KD * d1, 0.0, 1.0);
        shooterPower2 = clamp(ff + SHOOTER_KP * e2 + SHOOTER_KI * integral2 + SHOOTER_KD * d2, 0.0, 1.0);

        shooter1.setPower(shooterPower1);
        shooter2.setPower(shooterPower2);
    }

    private void shooterStop() {
        shooterRunning = false;
        shooter1.setPower(0);
        shooter2.setPower(0);
        shooterServo.setPosition(SERVO_HOME);
        shooterServo2.setPosition(SERVO2_HOME);
    }

    private double getShooterVelocity() {
        return (Math.abs(shooter1.getVelocity()) + Math.abs(shooter2.getVelocity())) / 2.0;
    }

    // =========================================================================
    // INTAKE & SHOOTING HELPERS
    // =========================================================================
    private void startIntake() { intakeMotor.setPower(-1); intakeActive = true; }
    private void stopIntake()  { intakeMotor.setPower(0);  intakeActive = false; }

    private void startContinuousShooting() {
        shootingState = ShootingState.SHOOTING;
        shootingTimer.resetTimer();
        shooterStart();
        shooterServo.setPosition(SERVO_SHOOT);
        shooterServo2.setPosition(SERVO2_SHOOT);
        intakeMotor.setPower(-0.85);
    }

    private void updateShootingStateMachine() {
        if (shootingState != ShootingState.SHOOTING) return;
        long elapsed = (long)(shootingTimer.getElapsedTimeSeconds() * 1000);
        telemetry.addData("Shooting", elapsed + " / " + CONTINUOUS_SHOOT_DURATION + "ms");
        if (elapsed >= CONTINUOUS_SHOOT_DURATION) {
            intakeMotor.setPower(0);
            shooterServo.setPosition(SERVO_HOME);
            shooterServo2.setPosition(SERVO2_HOME);
            shooterStop();
            shootingState = ShootingState.IDLE;
        }
    }

    // =========================================================================
    // HARDWARE INIT
    // =========================================================================
    private void initializeHardware() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterServo  = hardwareMap.servo.get("shooterServo");
        shooterServo2 = hardwareMap.servo.get("shooterServo2");
        shooterServo.setPosition(SERVO_HOME);
        shooterServo2.setPosition(SERVO2_HOME);

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretRotator = hardwareMap.dcMotor.get("turretRotator");
        turretRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretRotator.setTargetPosition(0);
        turretRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretRotator.setPower(0.0);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
    }

    // =========================================================================
    // OP MODE LIFECYCLE
    // =========================================================================
    @Override
    public void init() {
        pathState     = PathState.DRIVE_TO_SHOOT_POS;
        pathTimer     = new Timer();
        opModeTimer   = new Timer();
        shootingTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        buildPaths();
        follower.setPose(startPose);

        telemetry.addLine("=== BLUE SIDE AUTONOMOUS 21 ===");
        telemetry.addData("Sequence", "Start → Shoot → Intake 1 → Shoot");
        telemetry.addData("Target Velocity", TARGET_VELOCITY + " ticks/sec");
        telemetry.addData("Turret Angle",    TURRET_TARGET_ANGLE + "°");
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        shooterStart(); // spin up immediately on start
    }

    @Override
    public void loop() {
        follower.update();
        shooterUpdate();
        updateTurret();
        statePathUpdate();

        telemetry.addData("Path State",       pathState.toString());
        telemetry.addData("Shooting State",   shootingState.toString());
        telemetry.addData("Shooter Velocity", "%.0f / %.0f ticks/s", getShooterVelocity(), TARGET_VELOCITY);
        telemetry.addData("Intake Active",    intakeActive);
        telemetry.addData("x",       "%.2f", follower.getPose().getX());
        telemetry.addData("y",       "%.2f", follower.getPose().getY());
        telemetry.addData("heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    // =========================================================================
    // UTILITY
    // =========================================================================
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}