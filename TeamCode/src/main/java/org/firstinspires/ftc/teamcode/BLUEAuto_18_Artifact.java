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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

/**
 * BLUEAuto_18_Artifact
 *
 * 18-point autonomous routine:
 *   1. Drive to shooting position.
 *   2. Wait for flywheel to reach target velocity, then shoot preload.
 *   3. Run five intake → shoot cycles using curved Bézier paths.
 *   4. Park at final position.
 *
 * Uses DYNAMIC velocity mode (1400 ticks/s) for all shots.
 *
 * Hardware map names:
 *   DcMotorEx → "shooter1", "shooter2"          (inside ShooterSystem)
 *   Servo     → "shooterServo", "shooterServo2" (inside ShooterSystem)
 *   DcMotor   → "intakeMotor"
 *   DcMotor   → "turretRotator"
 *   IMU       → "imu"
 */
@Autonomous(name = "18 Artifact Auto Curved")
public class BLUEAuto_18_Artifact extends OpMode {

    // -----------------------------------------------------------------------
    // Pedro Pathing
    // -----------------------------------------------------------------------
    private Follower follower;
    private Timer pathTimer, opModeTimer, shootingTimer, intakeWaitTimer;

    // -----------------------------------------------------------------------
    // Subsystems & hardware
    // -----------------------------------------------------------------------
    private ShooterSystem shooterSystem;
    private DcMotor intakeMotor;
    private DcMotor turretRotator;
    private IMU imu;

    // -----------------------------------------------------------------------
    // Shooter mode — true = DYNAMIC (1400 t/s), false = FIXED (1900 t/s)
    // -----------------------------------------------------------------------
    private static final boolean DYNAMIC_MODE = true;

    // -----------------------------------------------------------------------
    // Timing constants (milliseconds)
    // -----------------------------------------------------------------------
    /** How long the pusher servos stay extended per shooting window. */
    private static final long SHOOT_DURATION       = 1000;

    /** How long to idle at the intake position before returning. */
    private static final long INTAKE_WAIT_DURATION = 300;

    /** Max ms to wait for flywheel to reach target velocity before shooting anyway. */
    private static final long SPINUP_TIMEOUT       = 2000;

    // -----------------------------------------------------------------------
    // Turret constants
    // -----------------------------------------------------------------------
    private static final int    TICKS_PER_ROTATION = 1872;
    private static final double TICKS_PER_DEGREE   = (double) TICKS_PER_ROTATION / 360.0;
    private static final double MIN_TURRET_ANGLE   = -90.0;
    private static final double MAX_TURRET_ANGLE   =  90.0;

    // -----------------------------------------------------------------------
    // Drive speed constants
    // -----------------------------------------------------------------------
    private static final double INTAKE_DRIVE_POWER = 0.6;
    private static final double NORMAL_DRIVE_POWER = 1.0;

    // -----------------------------------------------------------------------
    // State machines
    // -----------------------------------------------------------------------
    public enum PathState {
        DRIVE_TO_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_TO_INTAKE_1,
        WAIT_AND_INTAKE_1,
        RETURN_TO_SHOOT_1,
        SHOOT_SECOND_ROUND,
        DRIVE_TO_INTAKE_3,
        WAIT_AND_INTAKE_3,
        RETURN_TO_SHOOT_3,
        SHOOT_THIRD_ROUND,
        DRIVE_TO_INTAKE_4,
        WAIT_AND_INTAKE_4,
        RETURN_TO_SHOOT_4,
        SHOOT_FOURTH_ROUND,
        DRIVE_TO_INTAKE_5,
        WAIT_AND_INTAKE_5,
        RETURN_TO_SHOOT_5,
        SHOOT_FIFTH_ROUND,
        DRIVE_TO_FINAL_POSITION,
        COMPLETE
    }

    /**
     * Sub-states for each shooting sequence:
     *   IDLE     → not shooting
     *   SPINUP   → flywheel ramping; waiting for target velocity (or timeout)
     *   SHOOTING → pusher servos extended; timing the shot window
     */
    public enum ShootingState {
        IDLE,
        SPINUP,
        SHOOTING
    }

    private PathState     pathState;
    private ShootingState shootingState = ShootingState.IDLE;

    // -----------------------------------------------------------------------
    // Misc state flags
    // -----------------------------------------------------------------------
    private boolean intakeActive     = false;
    private boolean waitingForIntake = false;

    // -----------------------------------------------------------------------
    // Field poses
    // -----------------------------------------------------------------------
    private final Pose startPose      = new Pose(35, 131, Math.toRadians(180));
    private final Pose shootPose      = new Pose(48,  82, Math.toRadians(180));

    // Cycle 1
    private final Pose midPoint       = new Pose(48,  56, Math.toRadians(180));
    private final Pose intakeEndPoint = new Pose(14,  56, Math.toRadians(180));

    // Cycle 3
    private final Pose midPoint3      = new Pose(48,  60, Math.toRadians(180));
    private final Pose intakePoint3   = new Pose(10,  42, Math.toRadians(135));

    // Cycle 4
    private final Pose midPoint4      = new Pose(48,  80, Math.toRadians(180));
    private final Pose intakePoint4   = new Pose(17,  80, Math.toRadians(180));

    // Cycle 5
    private final Pose midPoint5      = new Pose(48,  33, Math.toRadians(180));
    private final Pose intakePoint5   = new Pose(17,  30, Math.toRadians(180));

    // Final park
    private final Pose finalPose      = new Pose(48,  60, Math.toRadians(180));

    // -----------------------------------------------------------------------
    // Path chains
    // -----------------------------------------------------------------------
    private PathChain driveToShootPos;
    private PathChain driveToIntake1,  returnToShoot1;
    private PathChain driveToIntake3,  returnToShoot3;
    private PathChain driveToIntake4,  returnToShoot4;
    private PathChain driveToIntake5,  returnToShoot5;
    private PathChain driveToFinalPosition;

    // -----------------------------------------------------------------------
    // buildPaths()
    // -----------------------------------------------------------------------
    public void buildPaths() {

        driveToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 1
        driveToIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, midPoint, intakeEndPoint))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeEndPoint.getHeading())
                .build();
        returnToShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(intakeEndPoint, midPoint, shootPose))
                .setLinearHeadingInterpolation(intakeEndPoint.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 3
        driveToIntake3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, midPoint3, intakePoint3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint3.getHeading())
                .build();
        returnToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint3, midPoint3, shootPose))
                .setLinearHeadingInterpolation(intakePoint3.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 4
        driveToIntake4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, midPoint4, intakePoint4))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint4.getHeading())
                .build();
        returnToShoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint4, midPoint4, shootPose))
                .setLinearHeadingInterpolation(intakePoint4.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 5
        driveToIntake5 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, midPoint5, intakePoint5))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint5.getHeading())
                .build();
        returnToShoot5 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint5, midPoint5, shootPose))
                .setLinearHeadingInterpolation(intakePoint5.getHeading(), shootPose.getHeading())
                .build();

        // FINAL PARK
        Pose controlPointFinal = new Pose(48, 70, Math.toRadians(180));
        driveToFinalPosition = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPointFinal, finalPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finalPose.getHeading())
                .build();
    }

    // -----------------------------------------------------------------------
    // statePathUpdate() — called every loop() tick
    // -----------------------------------------------------------------------
    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_TO_SHOOT_POS:
                follower.followPath(driveToShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startShootingSequence();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_1);
                }
                break;

            // ---- CYCLE 1 ---------------------------------------------------
            case DRIVE_TO_INTAKE_1:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake1, true);
                    setPathState(PathState.WAIT_AND_INTAKE_1);
                }
                telemetry.addData("Status", "Driving to intake 1");
                break;

            case WAIT_AND_INTAKE_1:
                if (!follower.isBusy()) {
                    if (!waitingForIntake) { waitingForIntake = true; intakeWaitTimer.resetTimer(); }
                    long e1 = msElapsed(intakeWaitTimer);
                    telemetry.addData("Wait Remaining", (INTAKE_WAIT_DURATION - e1) + "ms");
                    if (e1 >= INTAKE_WAIT_DURATION) finishIntakeAndReturn(returnToShoot1, PathState.RETURN_TO_SHOOT_1);
                }
                break;

            case RETURN_TO_SHOOT_1:
                if (!follower.isBusy()) setPathState(PathState.SHOOT_SECOND_ROUND);
                telemetry.addData("Status", "Returning to shoot");
                break;

            case SHOOT_SECOND_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startShootingSequence();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_3);
                }
                break;

            // ---- CYCLE 3 ---------------------------------------------------
            case DRIVE_TO_INTAKE_3:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake3, true);
                    setPathState(PathState.WAIT_AND_INTAKE_3);
                }
                telemetry.addData("Status", "Driving to intake 3");
                break;

            case WAIT_AND_INTAKE_3:
                if (!follower.isBusy()) {
                    if (!waitingForIntake) { waitingForIntake = true; intakeWaitTimer.resetTimer(); }
                    long e3 = msElapsed(intakeWaitTimer);
                    telemetry.addData("Wait Remaining", (INTAKE_WAIT_DURATION - e3) + "ms");
                    if (e3 >= INTAKE_WAIT_DURATION) finishIntakeAndReturn(returnToShoot3, PathState.RETURN_TO_SHOOT_3);
                }
                break;

            case RETURN_TO_SHOOT_3:
                if (!follower.isBusy()) setPathState(PathState.SHOOT_THIRD_ROUND);
                telemetry.addData("Status", "Returning to shoot");
                break;

            case SHOOT_THIRD_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startShootingSequence();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_4);
                }
                break;

            // ---- CYCLE 4 ---------------------------------------------------
            case DRIVE_TO_INTAKE_4:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake4, true);
                    setPathState(PathState.WAIT_AND_INTAKE_4);
                }
                telemetry.addData("Status", "Driving to intake 4");
                break;

            case WAIT_AND_INTAKE_4:
                if (!follower.isBusy()) {
                    if (!waitingForIntake) { waitingForIntake = true; intakeWaitTimer.resetTimer(); }
                    long e4 = msElapsed(intakeWaitTimer);
                    telemetry.addData("Wait Remaining", (INTAKE_WAIT_DURATION - e4) + "ms");
                    if (e4 >= INTAKE_WAIT_DURATION) finishIntakeAndReturn(returnToShoot4, PathState.RETURN_TO_SHOOT_4);
                }
                break;

            case RETURN_TO_SHOOT_4:
                if (!follower.isBusy()) setPathState(PathState.SHOOT_FOURTH_ROUND);
                telemetry.addData("Status", "Returning to shoot");
                break;

            case SHOOT_FOURTH_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startShootingSequence();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_5);
                }
                break;

            // ---- CYCLE 5 ---------------------------------------------------
            case DRIVE_TO_INTAKE_5:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake5, true);
                    setPathState(PathState.WAIT_AND_INTAKE_5);
                }
                telemetry.addData("Status", "Driving to intake 5");
                break;

            case WAIT_AND_INTAKE_5:
                if (!follower.isBusy()) {
                    if (!waitingForIntake) { waitingForIntake = true; intakeWaitTimer.resetTimer(); }
                    long e5 = msElapsed(intakeWaitTimer);
                    telemetry.addData("Wait Remaining", (INTAKE_WAIT_DURATION - e5) + "ms");
                    if (e5 >= INTAKE_WAIT_DURATION) finishIntakeAndReturn(returnToShoot5, PathState.RETURN_TO_SHOOT_5);
                }
                break;

            case RETURN_TO_SHOOT_5:
                if (!follower.isBusy()) setPathState(PathState.SHOOT_FIFTH_ROUND);
                telemetry.addData("Status", "Returning to shoot");
                break;

            case SHOOT_FIFTH_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startShootingSequence();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        returnTurretToHome();
                        setPathState(PathState.DRIVE_TO_FINAL_POSITION);
                    }
                }
                break;

            // ----------------------------------------------------------------
            case DRIVE_TO_FINAL_POSITION:
                if (!follower.isBusy()) {
                    follower.followPath(driveToFinalPosition, true);
                    setPathState(PathState.COMPLETE);
                }
                telemetry.addData("Status", "Driving to final position");
                break;

            case COMPLETE:
                if (!follower.isBusy()) {
                    shooterSystem.stop();
                    stopIntake();
                    telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
                    telemetry.addData("Final Position",  "x=48, y=60, heading=180°");
                    telemetry.addData("Turret Position", "Home (0 degrees)");
                }
                break;

            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }

    // -----------------------------------------------------------------------
    // Path state helper
    // -----------------------------------------------------------------------
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        telemetry.addData("New State", newState.toString());
    }

    // -----------------------------------------------------------------------
    // Shooting state machine
    // -----------------------------------------------------------------------

    /**
     * Begins a new shooting sequence. Tells the flywheel to spin up and
     * starts the timeout timer. Transitions to SPINUP.
     */
    private void startShootingSequence() {
        shooterSystem.setVelocity(DYNAMIC_MODE);
        shootingTimer.resetTimer();
        shootingState = ShootingState.SPINUP;
    }

    /**
     * Must be called every loop tick while shootingState != IDLE.
     *
     * SPINUP  → polls isAtTargetVelocity(); fires once ready (or timed out)
     * SHOOTING → times the pusher servo extension; homes servo when done
     */
    private void updateShootingStateMachine() {
        switch (shootingState) {

            case SPINUP:
                double v1     = shooterSystem.getVelocity1();
                double v2     = shooterSystem.getVelocity2();
                double target = shooterSystem.getTargetVelocity(DYNAMIC_MODE);
                long   spunMs = msElapsed(shootingTimer);

                telemetry.addData("Shooter",   "Spinning Up");
                telemetry.addData("Motor 1",   String.format("%.0f / %.0f tps", v1, target));
                telemetry.addData("Motor 2",   String.format("%.0f / %.0f tps", v2, target));
                telemetry.addData("Spinup ms", spunMs);

                boolean atSpeed  = shooterSystem.isAtTargetVelocity(DYNAMIC_MODE);
                boolean timedOut = spunMs >= SPINUP_TIMEOUT;

                if (atSpeed || timedOut) {
                    if (timedOut && !atSpeed) {
                        telemetry.addLine("WARNING: Spinup timed out — shooting anyway");
                    }
                    shooterSystem.shoot();           // extend both pusher servos
                    intakeMotor.setPower(-0.7);      // feed rings into the flywheel
                    shootingTimer.resetTimer();
                    shootingState = ShootingState.SHOOTING;
                }
                break;

            case SHOOTING:
                long shotMs = msElapsed(shootingTimer);
                telemetry.addData("Shooter",  "Shooting");
                telemetry.addData("Shot ms",  shotMs);

                if (shotMs >= SHOOT_DURATION) {
                    intakeMotor.setPower(0.0);
                    shooterSystem.homeServo();       // retract pushers; flywheel keeps spinning
                    shootingState = ShootingState.IDLE;
                }
                break;

            case IDLE:
                break;
        }
    }

    // -----------------------------------------------------------------------
    // Intake helpers
    // -----------------------------------------------------------------------
    private void startIntake() {
        intakeMotor.setPower(-1.0);
        intakeActive = true;
    }

    private void stopIntake() {
        intakeMotor.setPower(0.0);
        intakeActive = false;
    }

    /**
     * Shared cleanup at the end of every WAIT_AND_INTAKE state.
     * Stops the intake, restores full drive power, launches the return path.
     */
    private void finishIntakeAndReturn(PathChain returnPath, PathState nextState) {
        stopIntake();
        waitingForIntake = false;
        follower.setMaxPower(NORMAL_DRIVE_POWER);
        follower.followPath(returnPath, true);
        setPathState(nextState);
    }

    // -----------------------------------------------------------------------
    // Turret helpers
    // -----------------------------------------------------------------------
    private void alignTurretDynamic() {
        double imuHeading  = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetAngle = Math.max(MIN_TURRET_ANGLE,
                Math.min(MAX_TURRET_ANGLE, -45.0 + imuHeading));
        moveTurretToAngle(targetAngle, 0.8);
    }

    private void moveTurretToAngle(double angleDegrees, double power) {
        int targetTicks = (int) Math.round(angleDegrees * TICKS_PER_DEGREE);
        turretRotator.setTargetPosition(targetTicks);
        turretRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretRotator.setPower(Math.abs(power));
    }

    private void returnTurretToHome() {
        moveTurretToAngle(0.0, 0.8);
    }

    // -----------------------------------------------------------------------
    // Utility
    // -----------------------------------------------------------------------
    private long msElapsed(Timer timer) {
        return (long)(timer.getElapsedTimeSeconds() * 1000);
    }

    // -----------------------------------------------------------------------
    // Hardware initialisation
    // -----------------------------------------------------------------------
    private void initializeHardware() {
        // ShooterSystem manages shooter1, shooter2, shooterServo, shooterServo2
        shooterSystem = new ShooterSystem(hardwareMap);

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretRotator = hardwareMap.dcMotor.get("turretRotator");
        turretRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretRotator.setTargetPosition(0);
        turretRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretRotator.setPower(0.0);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    // -----------------------------------------------------------------------
    // OpMode lifecycle
    // -----------------------------------------------------------------------
    @Override
    public void init() {
        pathState = PathState.DRIVE_TO_SHOOT_POS;

        pathTimer       = new Timer();
        opModeTimer     = new Timer();
        shootingTimer   = new Timer();
        intakeWaitTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        buildPaths();
        follower.setPose(startPose);

        telemetry.addLine("=== 18-POINT CURVED PATH AUTONOMOUS ===");
        telemetry.addLine("Shooter: DcMotorEx + PIDF velocity control");
        telemetry.addData("Mode",           DYNAMIC_MODE ? "DYNAMIC (1400 t/s)" : "FIXED (1900 t/s)");
        telemetry.addData("Shoot Duration", SHOOT_DURATION + "ms");
        telemetry.addData("Spinup Timeout", SPINUP_TIMEOUT + "ms");
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        // Pre-spin the flywheel the moment the match starts so it reaches
        // target velocity by the time the robot arrives at shootPose.
        shooterSystem.setVelocity(DYNAMIC_MODE);

        // Aim the turret before the robot starts moving.
        alignTurretDynamic();
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State",    pathState.toString());
        telemetry.addData("Shoot State",   shootingState.toString());
        telemetry.addData("Intake Active", intakeActive);
        telemetry.addData("Shooter Vel",   String.format("%.0f / %.0f tps",
                shooterSystem.getVelocity(),
                shooterSystem.getTargetVelocity(DYNAMIC_MODE)));
        telemetry.addData("Servo 1",       String.format("%.2f", shooterSystem.getServo1Position()));
        telemetry.addData("Servo 2",       String.format("%.2f", shooterSystem.getServo2Position()));
        telemetry.addData("x",             follower.getPose().getX());
        telemetry.addData("y",             follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Op Time",       String.format("%.1fs", opModeTimer.getElapsedTimeSeconds()));
        telemetry.update();
    }
}