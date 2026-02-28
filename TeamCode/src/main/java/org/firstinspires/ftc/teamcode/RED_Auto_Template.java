package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

/**
 * RED_Auto_Template
 *
 * Red alliance autonomous template — 5-shot route (preload + 4 cycles).
 *
 * Route:
 *   Start
 *   → Shoot Position        (300ms pause → shoot preload)
 *   → Cycle 1: intakePoint1 → 200ms dwell (intake ON) → intermediate → 300ms pause → Shoot
 *   → Cycle 2: intakePoint3 (angled 40°)  → 2500ms dwell (intake ON) → 300ms pause → Shoot
 *   → Cycle 3: intakePoint4 → 200ms dwell (intake ON) → 300ms pause → Shoot
 *   → Cycle 4: intakePoint5 → 200ms dwell (intake ON) → 300ms pause → Shoot
 *   → Park
 *
 * Every shoot state is preceded by a 300ms PRE_SHOOT_PAUSE where the robot
 * settles and the flywheel stays spinning before the servo fires.
 */
@Autonomous(name = "RED Auto Template")
public class RED_Auto_Template extends OpMode {

    // =========================================================================
    // Pedro Pathing
    // =========================================================================
    private Follower follower;
    private Timer pathTimer;
    private Timer opModeTimer;
    private Timer shootingTimer;
    private Timer intakeWaitTimer;

    // =========================================================================
    // Subsystems & hardware
    // =========================================================================
    private ShooterSystem shooterSystem;
    private DcMotor intakeMotor;
    private DcMotor turretRotator;
    private IMU imu;

    // =========================================================================
    // Shooter constants
    // =========================================================================
    /** true = DYNAMIC mode (~1400 t/s),  false = FIXED mode (~1900 t/s) */
    private static final boolean DYNAMIC_MODE = true;

    /** How long the pusher servos stay extended each shooting window (ms). */
    private static final long SHOOT_DURATION = 1000;

    // =========================================================================
    // Timing constants
    // =========================================================================
    /** 200ms stop at normal (non-angled) intake points — intake keeps spinning. */
    private static final long INTAKE_DWELL_NORMAL = 200;

    /** 2500ms stop at angled intakePoint3 — intake keeps spinning. */
    private static final long INTAKE_DWELL_ANGLED = 2500;

    /** 300ms pause at intermediate settling point. */
    private static final long INTERMEDIATE_DWELL = 300;

    /** 300ms pause before every shot — robot settles, flywheel stays spinning. */
    private static final long PRE_SHOOT_PAUSE = 300;

    // =========================================================================
    // Turret constants
    // =========================================================================
    private static final int    TICKS_PER_ROTATION = 1872;
    private static final double TICKS_PER_DEGREE   = (double) TICKS_PER_ROTATION / 360.0;
    private static final double MIN_TURRET_ANGLE   = -90.0;
    private static final double MAX_TURRET_ANGLE   =  90.0;

    // =========================================================================
    // Drive power constants
    // =========================================================================
    private static final double NORMAL_DRIVE_POWER  = 1.0;
    private static final double INTAKE_DRIVE_POWER  = 0.9;
    private static final double INTAKE_POINT3_POWER = 0.75;

    // =========================================================================
    // Path state machine
    // =========================================================================
    public enum PathState {
        // Preload
        DRIVE_TO_SHOOT_POS,
        PRE_SHOOT_PAUSE_1,        // 300ms settle before preload shot
        SHOOT_PRELOAD,

        // Cycle 1: intakePoint1 → 200ms dwell → intermediate → shoot
        DRIVE_TO_INTAKE_1,
        DWELL_AT_INTAKE_1,        // 200ms stop, intake ON
        DRIVE_TO_INTERMEDIATE,
        DWELL_AT_INTERMEDIATE,    // 300ms stop, flywheel pre-spin
        RETURN_TO_SHOOT_1,
        PRE_SHOOT_PAUSE_2,        // 300ms settle before cycle 1 shot
        SHOOT_SECOND_ROUND,

        // Cycle 2: intakePoint3 angled 40° → 2500ms dwell → shoot
        DRIVE_TO_INTAKE_2,
        DWELL_AT_INTAKE_2,        // 2500ms stop, intake ON
        RETURN_TO_SHOOT_2,
        PRE_SHOOT_PAUSE_3,        // 300ms settle before cycle 2 shot
        SHOOT_THIRD_ROUND,

        // Cycle 3: intakePoint4 → 200ms dwell → shoot
        DRIVE_TO_INTAKE_3,
        DWELL_AT_INTAKE_3,        // 200ms stop, intake ON
        RETURN_TO_SHOOT_3,
        PRE_SHOOT_PAUSE_4,        // 300ms settle before cycle 3 shot
        SHOOT_FOURTH_ROUND,

        // Cycle 4: intakePoint5 → 200ms dwell → shoot
        DRIVE_TO_INTAKE_4,
        DWELL_AT_INTAKE_4,        // 200ms stop, intake ON
        RETURN_TO_SHOOT_4,
        PRE_SHOOT_PAUSE_5,        // 300ms settle before cycle 4 shot
        SHOOT_FIFTH_ROUND,

        // Park
        DRIVE_TO_FINAL_POSITION,
        COMPLETE
    }

    private PathState pathState;

    // =========================================================================
    // Shooting state machine
    // =========================================================================
    public enum ShootingState {
        IDLE,
        SHOOTING
    }

    private ShootingState shootingState = ShootingState.IDLE;

    // =========================================================================
    // State flags
    // =========================================================================
    private boolean intakeActive = false;
    private boolean dwelling     = false;

    // =========================================================================
    // Field poses  (Red side — robot faces 0°, mirrored: X_red = 144 - X_blue)
    // =========================================================================
    private final Pose startPose         = new Pose(108,   136, Math.toRadians(0));
    private final Pose shootPose         = new Pose( 94,    86, Math.toRadians(0));

    // Cycle 1 — straight intake + intermediate
    private final Pose intakePoint1      = new Pose(129.2,  60, Math.toRadians(0));
    private final Pose intermediatePoint = new Pose(125,    65, Math.toRadians(0));

    // Cycle 2 — angled intake (robot rotates to 40°)
    private final Pose intakePoint3      = new Pose(133.5,  62, Math.toRadians(35));

    // Cycle 3 — upper side
    private final Pose intakePoint4      = new Pose(128,    81, Math.toRadians(0));

    // Cycle 4 — lower side
    private final Pose intakePoint5      = new Pose(132,    32, Math.toRadians(0));

    // Park
    private final Pose finalPose         = new Pose( 96,    66, Math.toRadians(0));

    // =========================================================================
    // Path chains
    // =========================================================================
    private PathChain driveToShootPos;
    private PathChain driveToIntake1, driveToIntermediate, returnToShoot1;
    private PathChain driveToIntake2, returnToShoot2;
    private PathChain driveToIntake3, returnToShoot3;
    private PathChain driveToIntake4, returnToShoot4;
    private PathChain driveToFinalPosition;

    // =========================================================================
    // buildPaths()
    // =========================================================================
    public void buildPaths() {

        // Start → Shoot Position
        driveToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // --- CYCLE 1: shootPose → intakePoint1 --------------------------------
        Pose c1_1 = new Pose(96,   62,   Math.toRadians(0));
        Pose c2_1 = new Pose(93.8, 59.7, Math.toRadians(0));
        driveToIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c1_1, c2_1, intakePoint1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint1.getHeading())
                .build();

        // intakePoint1 → intermediate (curved inward)
        Pose intakeToInterCtrl = new Pose(110, 58, Math.toRadians(0));
        driveToIntermediate = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint1, intakeToInterCtrl, intermediatePoint))
                .setLinearHeadingInterpolation(intakePoint1.getHeading(), intermediatePoint.getHeading())
                .build();

        // intermediate → shootPose
        Pose c1_1r = new Pose(111, 61,   Math.toRadians(0));
        Pose c2_1r = new Pose( 97, 74.7, Math.toRadians(0));
        returnToShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(intermediatePoint, c1_1r, c2_1r, shootPose))
                .setLinearHeadingInterpolation(intermediatePoint.getHeading(), shootPose.getHeading())
                .build();

        // --- CYCLE 2: shootPose → intakePoint3 (angled 40°) ------------------
        Pose c1_2 = new Pose( 98.8, 57.3, Math.toRadians(0));
        Pose c2_2 = new Pose(120.5, 63,   Math.toRadians(20));
        driveToIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c1_2, c2_2, intakePoint3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint3.getHeading())
                .build();

        // intakePoint3 → shootPose
        Pose c1_2r = new Pose(116.5, 64.7, Math.toRadians(20));
        Pose c2_2r = new Pose(108,   66,   Math.toRadians(0));
        returnToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint3, c1_2r, c2_2r, shootPose))
                .setLinearHeadingInterpolation(intakePoint3.getHeading(), shootPose.getHeading())
                .build();

        // --- CYCLE 3: shootPose → intakePoint4 (upper side) ------------------
        Pose c1_3 = new Pose( 99.2, 87,   Math.toRadians(0));
        Pose c2_3 = new Pose(103.5, 83.3, Math.toRadians(0));
        driveToIntake3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c1_3, c2_3, intakePoint4))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint4.getHeading())
                .build();

        // intakePoint4 → shootPose
        Pose c1_3r = new Pose(103.5, 83.3, Math.toRadians(0));
        Pose c2_3r = new Pose( 99.4, 84.4, Math.toRadians(0));
        returnToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint4, c1_3r, c2_3r, shootPose))
                .setLinearHeadingInterpolation(intakePoint4.getHeading(), shootPose.getHeading())
                .build();

        // --- CYCLE 4: shootPose → intakePoint5 (lower side) ------------------
        Pose c1_4 = new Pose(95,   46,   Math.toRadians(0));
        Pose c2_4 = new Pose(95.5, 32.8, Math.toRadians(0));
        driveToIntake4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c1_4, c2_4, intakePoint5))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint5.getHeading())
                .build();

        // intakePoint5 → shootPose
        Pose c1_4r = new Pose(114, 36, Math.toRadians(0));
        Pose c2_4r = new Pose( 96, 56, Math.toRadians(0));
        returnToShoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint5, c1_4r, c2_4r, shootPose))
                .setLinearHeadingInterpolation(intakePoint5.getHeading(), shootPose.getHeading())
                .build();

        // Park
        Pose cFinal1 = new Pose(96, 81, Math.toRadians(0));
        Pose cFinal2 = new Pose(96, 71, Math.toRadians(0));
        driveToFinalPosition = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, cFinal1, cFinal2, finalPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finalPose.getHeading())
                .build();
    }

    // =========================================================================
    // statePathUpdate()
    // =========================================================================
    public void statePathUpdate() {
        switch (pathState) {

            // -----------------------------------------------------------------
            case DRIVE_TO_SHOOT_POS:
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(driveToShootPos, true);
                setPathState(PathState.PRE_SHOOT_PAUSE_1);
                break;

            case PRE_SHOOT_PAUSE_1:
                if (!follower.isBusy()) {
                    if (!dwelling) { dwelling = true; intakeWaitTimer.resetTimer(); }
                    if (msElapsed(intakeWaitTimer) >= PRE_SHOOT_PAUSE) {
                        dwelling = false;
                        setPathState(PathState.SHOOT_PRELOAD);
                    }
                    telemetry.addData("Pre-shoot pause", msElapsed(intakeWaitTimer) + " / " + PRE_SHOOT_PAUSE + "ms");
                }
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startContinuousShooting();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_1);
                }
                telemetry.addData("Status", "Shooting preload");
                break;

            // -----------------------------------------------------------------
            // CYCLE 1
            // -----------------------------------------------------------------
            case DRIVE_TO_INTAKE_1:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake1, true);
                    setPathState(PathState.DWELL_AT_INTAKE_1);
                }
                telemetry.addData("Status", "Cycle 1 — driving to intake");
                break;

            case DWELL_AT_INTAKE_1:
                if (!follower.isBusy()) {
                    if (!dwelling) { dwelling = true; intakeWaitTimer.resetTimer(); }
                    telemetry.addData("Intake Dwell", msElapsed(intakeWaitTimer) + " / " + INTAKE_DWELL_NORMAL + "ms");
                    if (msElapsed(intakeWaitTimer) >= INTAKE_DWELL_NORMAL) {
                        dwelling = false;
                        // intake keeps running while driving to intermediate
                        follower.setMaxPower(NORMAL_DRIVE_POWER);
                        follower.followPath(driveToIntermediate, true);
                        setPathState(PathState.DRIVE_TO_INTERMEDIATE);
                    }
                }
                telemetry.addData("Status", "Cycle 1 — dwelling at intake (intake ON)");
                break;

            case DRIVE_TO_INTERMEDIATE:
                if (!follower.isBusy()) setPathState(PathState.DWELL_AT_INTERMEDIATE);
                telemetry.addData("Status", "Cycle 1 — driving to intermediate");
                break;

            case DWELL_AT_INTERMEDIATE:
                if (!follower.isBusy()) {
                    if (!dwelling) {
                        dwelling = true;
                        intakeWaitTimer.resetTimer();
                        stopIntake();
                        shooterSystem.setVelocity(DYNAMIC_MODE);
                    }
                    telemetry.addData("Intermediate Dwell", msElapsed(intakeWaitTimer) + " / " + INTERMEDIATE_DWELL + "ms");
                    if (msElapsed(intakeWaitTimer) >= INTERMEDIATE_DWELL) {
                        dwelling = false;
                        follower.setMaxPower(NORMAL_DRIVE_POWER);
                        follower.followPath(returnToShoot1, true);
                        setPathState(PathState.RETURN_TO_SHOOT_1);
                    }
                }
                telemetry.addData("Status", "Cycle 1 — dwelling at intermediate");
                break;

            case RETURN_TO_SHOOT_1:
                if (!follower.isBusy()) setPathState(PathState.PRE_SHOOT_PAUSE_2);
                telemetry.addData("Status", "Cycle 1 — returning to shoot");
                break;

            case PRE_SHOOT_PAUSE_2:
                if (!follower.isBusy()) {
                    if (!dwelling) { dwelling = true; intakeWaitTimer.resetTimer(); }
                    if (msElapsed(intakeWaitTimer) >= PRE_SHOOT_PAUSE) {
                        dwelling = false;
                        setPathState(PathState.SHOOT_SECOND_ROUND);
                    }
                    telemetry.addData("Pre-shoot pause", msElapsed(intakeWaitTimer) + " / " + PRE_SHOOT_PAUSE + "ms");
                }
                break;

            case SHOOT_SECOND_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startContinuousShooting();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_2);
                }
                telemetry.addData("Status", "Cycle 1 — shooting");
                break;

            // -----------------------------------------------------------------
            // CYCLE 2 — angled intake
            // -----------------------------------------------------------------
            case DRIVE_TO_INTAKE_2:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_POINT3_POWER);
                    follower.followPath(driveToIntake2, true);
                    setPathState(PathState.DWELL_AT_INTAKE_2);
                }
                telemetry.addData("Status", "Cycle 2 — driving to angled intake");
                break;

            case DWELL_AT_INTAKE_2:
                if (!follower.isBusy()) {
                    if (!dwelling) { dwelling = true; intakeWaitTimer.resetTimer(); }
                    telemetry.addData("Intake Dwell", msElapsed(intakeWaitTimer) + " / " + INTAKE_DWELL_ANGLED + "ms");
                    if (msElapsed(intakeWaitTimer) >= INTAKE_DWELL_ANGLED) {
                        dwelling = false;
                        stopIntake();
                        follower.setMaxPower(NORMAL_DRIVE_POWER);
                        follower.followPath(returnToShoot2, true);
                        setPathState(PathState.RETURN_TO_SHOOT_2);
                    }
                }
                telemetry.addData("Status", "Cycle 2 — dwelling at angled intake (intake ON)");
                break;

            case RETURN_TO_SHOOT_2:
                if (!follower.isBusy()) setPathState(PathState.PRE_SHOOT_PAUSE_3);
                telemetry.addData("Status", "Cycle 2 — returning to shoot");
                break;

            case PRE_SHOOT_PAUSE_3:
                if (!follower.isBusy()) {
                    if (!dwelling) { dwelling = true; intakeWaitTimer.resetTimer(); }
                    if (msElapsed(intakeWaitTimer) >= PRE_SHOOT_PAUSE) {
                        dwelling = false;
                        setPathState(PathState.SHOOT_THIRD_ROUND);
                    }
                    telemetry.addData("Pre-shoot pause", msElapsed(intakeWaitTimer) + " / " + PRE_SHOOT_PAUSE + "ms");
                }
                break;

            case SHOOT_THIRD_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startContinuousShooting();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_3);
                }
                telemetry.addData("Status", "Cycle 2 — shooting");
                break;

            // -----------------------------------------------------------------
            // CYCLE 3 — upper side
            // -----------------------------------------------------------------
            case DRIVE_TO_INTAKE_3:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake3, true);
                    setPathState(PathState.DWELL_AT_INTAKE_3);
                }
                telemetry.addData("Status", "Cycle 3 — driving to intake");
                break;

            case DWELL_AT_INTAKE_3:
                if (!follower.isBusy()) {
                    if (!dwelling) { dwelling = true; intakeWaitTimer.resetTimer(); }
                    telemetry.addData("Intake Dwell", msElapsed(intakeWaitTimer) + " / " + INTAKE_DWELL_NORMAL + "ms");
                    if (msElapsed(intakeWaitTimer) >= INTAKE_DWELL_NORMAL) {
                        dwelling = false;
                        stopIntake();
                        follower.setMaxPower(NORMAL_DRIVE_POWER);
                        follower.followPath(returnToShoot3, true);
                        setPathState(PathState.RETURN_TO_SHOOT_3);
                    }
                }
                telemetry.addData("Status", "Cycle 3 — dwelling at intake (intake ON)");
                break;

            case RETURN_TO_SHOOT_3:
                if (!follower.isBusy()) setPathState(PathState.PRE_SHOOT_PAUSE_4);
                telemetry.addData("Status", "Cycle 3 — returning to shoot");
                break;

            case PRE_SHOOT_PAUSE_4:
                if (!follower.isBusy()) {
                    if (!dwelling) { dwelling = true; intakeWaitTimer.resetTimer(); }
                    if (msElapsed(intakeWaitTimer) >= PRE_SHOOT_PAUSE) {
                        dwelling = false;
                        setPathState(PathState.SHOOT_FOURTH_ROUND);
                    }
                    telemetry.addData("Pre-shoot pause", msElapsed(intakeWaitTimer) + " / " + PRE_SHOOT_PAUSE + "ms");
                }
                break;

            case SHOOT_FOURTH_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startContinuousShooting();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) setPathState(PathState.DRIVE_TO_INTAKE_4);
                }
                telemetry.addData("Status", "Cycle 3 — shooting");
                break;

            // -----------------------------------------------------------------
            // CYCLE 4 — lower side
            // -----------------------------------------------------------------
            case DRIVE_TO_INTAKE_4:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake4, true);
                    setPathState(PathState.DWELL_AT_INTAKE_4);
                }
                telemetry.addData("Status", "Cycle 4 — driving to intake");
                break;

            case DWELL_AT_INTAKE_4:
                if (!follower.isBusy()) {
                    if (!dwelling) { dwelling = true; intakeWaitTimer.resetTimer(); }
                    telemetry.addData("Intake Dwell", msElapsed(intakeWaitTimer) + " / " + INTAKE_DWELL_NORMAL + "ms");
                    if (msElapsed(intakeWaitTimer) >= INTAKE_DWELL_NORMAL) {
                        dwelling = false;
                        stopIntake();
                        alignTurretDynamic(); // pre-aim while driving back
                        follower.setMaxPower(NORMAL_DRIVE_POWER);
                        follower.followPath(returnToShoot4, true);
                        setPathState(PathState.RETURN_TO_SHOOT_4);
                    }
                }
                telemetry.addData("Status", "Cycle 4 — dwelling at intake (intake ON)");
                break;

            case RETURN_TO_SHOOT_4:
                if (!follower.isBusy()) setPathState(PathState.PRE_SHOOT_PAUSE_5);
                telemetry.addData("Status", "Cycle 4 — returning to shoot");
                break;

            case PRE_SHOOT_PAUSE_5:
                if (!follower.isBusy()) {
                    if (!dwelling) { dwelling = true; intakeWaitTimer.resetTimer(); }
                    if (msElapsed(intakeWaitTimer) >= PRE_SHOOT_PAUSE) {
                        dwelling = false;
                        setPathState(PathState.SHOOT_FIFTH_ROUND);
                    }
                    telemetry.addData("Pre-shoot pause", msElapsed(intakeWaitTimer) + " / " + PRE_SHOOT_PAUSE + "ms");
                }
                break;

            case SHOOT_FIFTH_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) startContinuousShooting();
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        returnTurretToHome();
                        setPathState(PathState.DRIVE_TO_FINAL_POSITION);
                    }
                }
                telemetry.addData("Status", "Cycle 4 — shooting");
                break;

            // -----------------------------------------------------------------
            case DRIVE_TO_FINAL_POSITION:
                if (!follower.isBusy()) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(driveToFinalPosition, true);
                    setPathState(PathState.COMPLETE);
                }
                telemetry.addData("Status", "Driving to park");
                break;

            case COMPLETE:
                if (!follower.isBusy()) {
                    shooterSystem.stop();
                    stopIntake();
                    telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
                    telemetry.addData("Final Pose",
                            String.format("x=%.1f  y=%.1f  h=%.1f°",
                                    follower.getPose().getX(),
                                    follower.getPose().getY(),
                                    Math.toDegrees(follower.getPose().getHeading())));
                }
                break;

            default:
                telemetry.addLine("Unknown state");
                break;
        }
    }

    // =========================================================================
    // Shooting helpers
    // =========================================================================
    private void startContinuousShooting() {
        shootingState = ShootingState.SHOOTING;
        shootingTimer.resetTimer();
        shooterSystem.shoot();
        intakeMotor.setPower(-0.7);
    }

    private void updateShootingStateMachine() {
        if (shootingState == ShootingState.SHOOTING) {
            long elapsed = msElapsed(shootingTimer);
            telemetry.addData("Shooter", "Shooting");
            telemetry.addData("Shot ms", elapsed);
            if (elapsed >= SHOOT_DURATION) {
                intakeMotor.setPower(0.0);
                shooterSystem.homeServo();
                shootingState = ShootingState.IDLE;
            }
        }
    }

    // =========================================================================
    // Intake helpers
    // =========================================================================
    private void startIntake() {
        intakeMotor.setPower(-1.0);
        intakeActive = true;
    }

    private void stopIntake() {
        intakeMotor.setPower(0.0);
        intakeActive = false;
    }

    // =========================================================================
    // Turret helpers
    // =========================================================================
    private void alignTurretDynamic() {
        double imuHeading  = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetAngle = Math.max(MIN_TURRET_ANGLE,
                Math.min(MAX_TURRET_ANGLE, 47.0 - imuHeading));
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

    // =========================================================================
    // Utility
    // =========================================================================
    private long msElapsed(Timer timer) {
        return (long)(timer.getElapsedTimeSeconds() * 1000);
    }

    // =========================================================================
    // Hardware initialisation
    // =========================================================================
    private void initializeHardware() {
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
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
    }

    // =========================================================================
    // setPathState()
    // =========================================================================
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        telemetry.addData("→ New State", newState.toString());
    }

    // =========================================================================
    // OpMode lifecycle
    // =========================================================================
    @Override
    public void init() {
        pathState       = PathState.DRIVE_TO_SHOOT_POS;
        pathTimer       = new Timer();
        opModeTimer     = new Timer();
        shootingTimer   = new Timer();
        intakeWaitTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        buildPaths();
        follower.setPose(startPose);

        telemetry.addLine("=== RED Auto Template ===");
        telemetry.addLine("Preload + 4 cycles: intake1+inter, angled(40°), upper, lower");
        telemetry.addData("Pre-shoot pause",     PRE_SHOOT_PAUSE     + "ms (every shot)");
        telemetry.addData("Normal intake dwell", INTAKE_DWELL_NORMAL + "ms (intake ON)");
        telemetry.addData("Angled intake dwell", INTAKE_DWELL_ANGLED + "ms (intake ON)");
        telemetry.addData("Intermediate dwell",  INTERMEDIATE_DWELL  + "ms");
        telemetry.addData("Shoot Duration",       SHOOT_DURATION      + "ms");
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        shooterSystem.setVelocity(DYNAMIC_MODE);
        alignTurretDynamic();
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State",    pathState.toString());
        telemetry.addData("Shoot State",   shootingState.toString());
        telemetry.addData("Intake Active", intakeActive);
        telemetry.addData("Dwelling",      dwelling);
        telemetry.addData("Shooter Vel",   String.format("%.0f tps", shooterSystem.getVelocity()));
        telemetry.addData("x",             String.format("%.2f", follower.getPose().getX()));
        telemetry.addData("y",             String.format("%.2f", follower.getPose().getY()));
        telemetry.addData("Heading (deg)", String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Op Time (s)",   String.format("%.1f", opModeTimer.getElapsedTimeSeconds()));
        telemetry.update();
    }
}