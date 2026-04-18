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

@Autonomous(name = "Blue_Auto_15")
public class Blue_Auto_15 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer, shootingTimer, intermediatePauseTimer;

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
    // ██  TUNE THESE — TURRET  ████████████████████████████████████████████████
    // =========================================================================
    private static final double TURRET_TARGET_ANGLE = -46.0; // degrees — BLUE side negative
    private static final double TURRET_DEADBAND     =  1.0;  // degrees — smaller = more responsive
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
    private double lastTurretCommandedAngle = Double.MAX_VALUE; // force update on first call

    // Timing
    private static final long CONTINUOUS_SHOOT_DURATION   = 900;
    private static final long INTERMEDIATE_PAUSE_DURATION = 400;

    // Drive power
    private static final double INTAKE_DRIVE_POWER = 1.0;
    private static final double NORMAL_DRIVE_POWER  = 1.0;

    // State
    private ShootingState shootingState = ShootingState.IDLE;
    private boolean intakeActive          = false;
    private boolean pausingAtIntermediate = false;
    private Pose    holdPosition          = null;

    public enum PathState {
        DRIVE_TO_SHOOT_POS, SHOOT_PRELOAD,
        DRIVE_TO_INTAKE_END, DRIVE_TO_INTERMEDIATE, PAUSE_AT_INTERMEDIATE,
        RETURN_FROM_INTERMEDIATE, SHOOT_ROUND_1,
        DRIVE_TO_INTAKE_4, RETURN_FROM_INTAKE_4, SHOOT_ROUND_2,
        DRIVE_TO_INTAKE_5, RETURN_FROM_INTAKE_5, SHOOT_ROUND_3,
        DRIVE_TO_PARK, COMPLETE
    }
    public enum ShootingState { IDLE, SHOOTING }

    PathState pathState;

    // BLUE SIDE Poses
    private final Pose startPose         = new Pose(36, 130, Math.toRadians(180));
    private final Pose shootPose         = new Pose(48,  88, Math.toRadians(180));
    private final Pose intakeEndPoint    = new Pose(11,  59, Math.toRadians(180));
    private final Pose intermediatePose1 = new Pose(16,  64, Math.toRadians(180));
    private final Pose intakePoint4      = new Pose(16,  81, Math.toRadians(180));
    private final Pose intakePoint5      = new Pose( 9,  31, Math.toRadians(180));
    private final Pose parkPose          = new Pose(34,  72, Math.toRadians(180));

    private PathChain driveToShootPos;
    private PathChain driveToIntakeEnd, driveToIntermediate, returnFromIntermediate;
    private PathChain driveToIntake4, returnFromIntake4;
    private PathChain driveToIntake5, returnFromIntake5;
    private PathChain driveToPark;

    // =========================================================================
    // PATH BUILDING
    // =========================================================================
    public void buildPaths() {
        driveToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        Pose c1_1 = new Pose(48, 62, Math.toRadians(180));
        Pose c2_1 = new Pose(50.2, 59.7, Math.toRadians(180));
        driveToIntakeEnd = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c1_1, c2_1, intakeEndPoint))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeEndPoint.getHeading())
                .build();

        Pose c1_1i = new Pose(26, 60, Math.toRadians(180));
        driveToIntermediate = follower.pathBuilder()
                .addPath(new BezierCurve(intakeEndPoint, c1_1i, intermediatePose1))
                .setLinearHeadingInterpolation(intakeEndPoint.getHeading(), intermediatePose1.getHeading())
                .build();

        Pose c1_ir = new Pose(34, 68, Math.toRadians(180));
        Pose c2_ir = new Pose(47, 74.7, Math.toRadians(180));
        returnFromIntermediate = follower.pathBuilder()
                .addPath(new BezierCurve(intermediatePose1, c1_ir, c2_ir, shootPose))
                .setLinearHeadingInterpolation(intermediatePose1.getHeading(), shootPose.getHeading())
                .build();

        Pose c1_4 = new Pose(44.8, 87, Math.toRadians(180));
        Pose c2_4 = new Pose(42, 79, Math.toRadians(180));
        driveToIntake4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c1_4, c2_4, intakePoint4))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint4.getHeading())
                .build();

        Pose c1_4r = new Pose(27, 75, Math.toRadians(180));
        Pose c2_4r = new Pose(34, 68, Math.toRadians(180));
        Pose c3_4r = new Pose(47, 74.7, Math.toRadians(180));
        returnFromIntake4 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint4, c1_4r, c2_4r, c3_4r, shootPose))
                .setLinearHeadingInterpolation(intakePoint4.getHeading(), shootPose.getHeading())
                .build();

        Pose c1_5 = new Pose(49, 46, Math.toRadians(180));
        Pose c2_5 = new Pose(48.5, 32.8, Math.toRadians(180));
        driveToIntake5 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c1_5, c2_5, intakePoint5))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint5.getHeading())
                .build();

        Pose c1_5r = new Pose(30, 36, Math.toRadians(180));
        Pose c2_5r = new Pose(48, 56, Math.toRadians(180));
        returnFromIntake5 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint5, c1_5r, c2_5r, shootPose))
                .setLinearHeadingInterpolation(intakePoint5.getHeading(), shootPose.getHeading())
                .build();

        Pose c1_park = new Pose(42, 80, Math.toRadians(180));
        driveToPark = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c1_park, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    // =========================================================================
    // STATE MACHINE
    // =========================================================================
    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_TO_SHOOT_POS:
                shooterStart();
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(driveToShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startContinuousShooting();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_END);
                }
                break;

            case DRIVE_TO_INTAKE_END:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntakeEnd, true);
                    setPathState(PathState.DRIVE_TO_INTERMEDIATE);
                }
                telemetry.addData("Cycle", "1 - To intakeEndPoint");
                break;

            case DRIVE_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(driveToIntermediate, true);
                    setPathState(PathState.PAUSE_AT_INTERMEDIATE);
                }
                telemetry.addData("Cycle", "1 - To intermediatePose1");
                break;

            case PAUSE_AT_INTERMEDIATE:
                if (!follower.isBusy()) {
                    if (!pausingAtIntermediate) {
                        pausingAtIntermediate = true;
                        intermediatePauseTimer.resetTimer();
                        holdPosition = follower.getPose();
                    }
                    follower.holdPoint(holdPosition);
                    long elapsed = (long)(intermediatePauseTimer.getElapsedTimeSeconds() * 1000);
                    telemetry.addData("Cycle", "1 - PAUSING");
                    telemetry.addData("Pause Remaining", (INTERMEDIATE_PAUSE_DURATION - elapsed) + "ms");
                    if (elapsed >= INTERMEDIATE_PAUSE_DURATION) {
                        pausingAtIntermediate = false;
                        holdPosition = null;
                        shooterStart();
                        follower.followPath(returnFromIntermediate, true);
                        setPathState(PathState.RETURN_FROM_INTERMEDIATE);
                    }
                }
                break;

            case RETURN_FROM_INTERMEDIATE:
                if (!follower.isBusy()) setPathState(PathState.SHOOT_ROUND_1);
                telemetry.addData("Cycle", "1 - Returning");
                break;

            case SHOOT_ROUND_1:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startContinuousShooting();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_4);
                }
                telemetry.addData("Cycle", "1 - Shooting");
                break;

            case DRIVE_TO_INTAKE_4:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake4, true);
                    setPathState(PathState.RETURN_FROM_INTAKE_4);
                }
                telemetry.addData("Cycle", "2 - To intakePoint4");
                break;

            case RETURN_FROM_INTAKE_4:
                if (!follower.isBusy()) {
                    stopIntake();
                    shooterStart();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(returnFromIntake4, true);
                    setPathState(PathState.SHOOT_ROUND_2);
                }
                telemetry.addData("Cycle", "2 - Returning");
                break;

            case SHOOT_ROUND_2:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startContinuousShooting();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_5);
                }
                telemetry.addData("Cycle", "2 - Shooting");
                break;

            case DRIVE_TO_INTAKE_5:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake5, true);
                    setPathState(PathState.RETURN_FROM_INTAKE_5);
                }
                telemetry.addData("Cycle", "3 - To intakePoint5");
                break;

            case RETURN_FROM_INTAKE_5:
                if (!follower.isBusy()) {
                    stopIntake();
                    shooterStart();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(returnFromIntake5, true);
                    setPathState(PathState.SHOOT_ROUND_3);
                }
                telemetry.addData("Cycle", "3 - Returning");
                break;

            case SHOOT_ROUND_3:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startContinuousShooting();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        moveTurretToAngle(0, 0.8);
                        setPathState(PathState.DRIVE_TO_PARK);
                    }
                }
                telemetry.addData("Cycle", "3 - Shooting");
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy()) {
                    follower.followPath(driveToPark, true);
                    setPathState(PathState.COMPLETE);
                }
                telemetry.addData("Status", "Parking");
                break;

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

        // Use follower's pose heading — follower.update() already handles pinpoint,
        // calling pinpoint.update() separately causes double-update and heading spikes
        double imuHeading = Math.toDegrees(follower.getPose().getHeading());

        // Normalize to [-180, 180] to prevent wrap-around spikes near ±180°
        double relativeHeading = AngleUnit.normalizeDegrees(imuHeading - 180);
        double targetAngle = TURRET_TARGET_ANGLE - relativeHeading;
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
    // SHOOTER — inlined PID (tune TARGET_VELOCITY at the top of this file)
    // =========================================================================
    private void shooterStart() {
        if (shooterRunning) return; // already spinning — don't reset PID
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

        // pinpoint is kept for potential direct use but turret heading
        // is read via follower.getPose() to avoid double-update issues
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
    }

    // =========================================================================
    // OP MODE LIFECYCLE
    // =========================================================================
    @Override
    public void init() {
        pathState              = PathState.DRIVE_TO_SHOOT_POS;
        pathTimer              = new Timer();
        opModeTimer            = new Timer();
        shootingTimer          = new Timer();
        intermediatePauseTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        buildPaths();
        follower.setPose(startPose);

        telemetry.addLine("=== BLUE SIDE AUTONOMOUS ===");
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
        shooterUpdate();   // shooter PID — must be every loop
        updateTurret();    // turret continuously tracks heading — uses follower pose, not pinpoint directly
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