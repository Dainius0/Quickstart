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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

@Autonomous(name = "Red_Auto_15_Modified")
public class Red_Auto_15 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, shootingTimer, intermediatePauseTimer;

    // Subsystems
    private ShooterSystem shooterSystem;

    // Hardware
    private DcMotor intakeMotor, turretRotator;
    private GoBildaPinpointDriver pinpoint;

    // Continuous shooting constants
    private static final long CONTINUOUS_SHOOT_DURATION = 900;
    private static final long INTERMEDIATE_PAUSE_DURATION = 5000; // Pause at intermediatePose1

    // Turret constants
    private static final int TICKS_PER_ROTATION = 1872;
    private static final double TICKS_PER_DEGREE = (double) TICKS_PER_ROTATION / 360.0;
    private static final double MIN_TURRET_ANGLE = -90.0;
    private static final double MAX_TURRET_ANGLE = 90.0;

    // Drive speed constants
    private static final double INTAKE_DRIVE_POWER = 1;
    private static final double NORMAL_DRIVE_POWER = 1.0;

    // State variables
    private ShootingState shootingState = ShootingState.IDLE;
    private boolean intakeActive = false;
    private boolean pausingAtIntermediate = false;
    private Pose holdPosition = null;

    public enum PathState {
        DRIVE_TO_SHOOT_POS,
        SHOOT_PRELOAD,

        // Cycle 1: intakeEndPoint -> intermediatePose1 -> shootPose
        DRIVE_TO_INTAKE_END,
        DRIVE_TO_INTERMEDIATE,
        PAUSE_AT_INTERMEDIATE,
        RETURN_FROM_INTERMEDIATE,
        SHOOT_ROUND_1,

        // Cycle 2: intakePoint4 -> shootPose
        DRIVE_TO_INTAKE_4,
        RETURN_FROM_INTAKE_4,
        SHOOT_ROUND_2,

        // Cycle 3: intakePoint5 -> shootPose
        DRIVE_TO_INTAKE_5,
        RETURN_FROM_INTAKE_5,
        SHOOT_ROUND_3,

        // Park
        DRIVE_TO_PARK,
        COMPLETE
    }

    public enum ShootingState {
        IDLE,
        SHOOTING
    }

    PathState pathState;

    // RED SIDE Poses - Mirrored from BLUE (X = 144 - blue_x, heading = 0°)
    private final Pose startPose = new Pose(108, 135, Math.toRadians(0));          // Mirrored from BLUE (36, 135, 180°)
    private final Pose shootPose = new Pose(96, 88, Math.toRadians(0));            // Mirrored from BLUE (48, 88, 180°)
    private final Pose intakeEndPoint = new Pose(133, 59, Math.toRadians(0));      // Mirrored from BLUE (11, 59, 180°)
    private final Pose intermediatePose1 = new Pose(128, 64, Math.toRadians(0));   // Mirrored from BLUE (16, 64, 180°)
    private final Pose intakePoint4 = new Pose(128, 81, Math.toRadians(0));        // Mirrored from BLUE (16, 81, 180°)
    private final Pose intakePoint5 = new Pose(135, 31, Math.toRadians(0));        // Mirrored from BLUE (9, 31, 180°)
    private final Pose parkPose = new Pose(110, 72, Math.toRadians(0));            // Mirrored from BLUE (34, 72, 180°)

    // Path chains
    private PathChain driveToShootPos;
    private PathChain driveToIntakeEnd, driveToIntermediate, returnFromIntermediate;
    private PathChain driveToIntake4, returnFromIntake4;
    private PathChain driveToIntake5, returnFromIntake5;
    private PathChain driveToPark;

    public void buildPaths() {
        // Initial drive to shooting position
        driveToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 1: shootPose -> intakeEndPoint (mirrored control points)
        Pose control1_1 = new Pose(96, 62, Math.toRadians(0));          // Mirrored from BLUE (48, 62, 180°)
        Pose control2_1 = new Pose(93.8, 59.7, Math.toRadians(0));      // Mirrored from BLUE (50.2, 59.7, 180°)
        driveToIntakeEnd = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control1_1, control2_1, intakeEndPoint))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeEndPoint.getHeading())
                .build();

        // intakeEndPoint -> intermediatePose1 (mirrored control points)
        Pose control1_1i = new Pose(131, 61, Math.toRadians(0));        // Mirrored from BLUE (13, 61, 180°)
        driveToIntermediate = follower.pathBuilder()
                .addPath(new BezierCurve(intakeEndPoint, control1_1i, intermediatePose1))
                .setLinearHeadingInterpolation(intakeEndPoint.getHeading(), intermediatePose1.getHeading())
                .build();

        // intermediatePose1 -> shootPose (mirrored control points)
        Pose control1_ir = new Pose(110, 68, Math.toRadians(0));        // Mirrored from BLUE (34, 68, 180°)
        Pose control2_ir = new Pose(97, 74.7, Math.toRadians(0));       // Mirrored from BLUE (47, 74.7, 180°)
        returnFromIntermediate = follower.pathBuilder()
                .addPath(new BezierCurve(intermediatePose1, control1_ir, control2_ir, shootPose))
                .setLinearHeadingInterpolation(intermediatePose1.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 2: shootPose -> intakePoint4 (mirrored control points)
        Pose control1_4 = new Pose(99.2, 87, Math.toRadians(0));        // Mirrored from BLUE (44.8, 87, 180°)
        Pose control2_4 = new Pose(102, 79, Math.toRadians(0));         // Mirrored from BLUE (42, 79, 180°)
        driveToIntake4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control1_4, control2_4, intakePoint4))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint4.getHeading())
                .build();

        // Return: intakePoint4 -> shootPose (mirrored control points)
        Pose control1_4r = new Pose(117, 75, Math.toRadians(0));        // Mirrored from BLUE (27, 75, 180°)
        Pose control2_4r = new Pose(110, 68, Math.toRadians(0));        // Mirrored from BLUE (34, 68, 180°)
        Pose control3_4r = new Pose(97, 74.7, Math.toRadians(0));       // Mirrored from BLUE (47, 74.7, 180°)
        returnFromIntake4 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint4, control1_4r, control2_4r, control3_4r, shootPose))
                .setLinearHeadingInterpolation(intakePoint4.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 3: shootPose -> intakePoint5 (mirrored control points)
        Pose control1_5 = new Pose(95, 46, Math.toRadians(0));          // Mirrored from BLUE (49, 46, 180°)
        Pose control2_5 = new Pose(95.5, 32.8, Math.toRadians(0));      // Mirrored from BLUE (48.5, 32.8, 180°)
        driveToIntake5 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control1_5, control2_5, intakePoint5))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint5.getHeading())
                .build();

        // Return: intakePoint5 -> shootPose (mirrored control points)
        Pose control1_5r = new Pose(114, 36, Math.toRadians(0));        // Mirrored from BLUE (30, 36, 180°)
        Pose control2_5r = new Pose(96, 56, Math.toRadians(0));         // Mirrored from BLUE (48, 56, 180°)
        returnFromIntake5 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint5, control1_5r, control2_5r, shootPose))
                .setLinearHeadingInterpolation(intakePoint5.getHeading(), shootPose.getHeading())
                .build();

        // PARK: shootPose -> parkPose (mirrored control points)
        Pose control1_park = new Pose(102, 80, Math.toRadians(0));      // Mirrored from BLUE (42, 80, 180°)
        driveToPark = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control1_park, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_TO_SHOOT_POS:
                follower.followPath(driveToShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_INTAKE_END);
                    }
                }
                break;

            // ========== CYCLE 1: intakeEndPoint -> intermediatePose1 -> shootPose ==========
            case DRIVE_TO_INTAKE_END:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntakeEnd, true);
                    setPathState(PathState.DRIVE_TO_INTERMEDIATE);
                }
                telemetry.addData("Cycle", "1 - Driving to intakeEndPoint");
                break;

            case DRIVE_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(driveToIntermediate, true);
                    setPathState(PathState.PAUSE_AT_INTERMEDIATE);
                }
                telemetry.addData("Cycle", "1 - Driving to intermediatePose1");
                break;

            case PAUSE_AT_INTERMEDIATE:
                if (!follower.isBusy()) {
                    if (!pausingAtIntermediate) {
                        pausingAtIntermediate = true;
                        intermediatePauseTimer.resetTimer();
                        holdPosition = follower.getPose();
                    }

                    follower.holdPoint(holdPosition);

                    long pauseElapsedTime = (long)(intermediatePauseTimer.getElapsedTimeSeconds() * 1000);
                    telemetry.addData("Cycle", "1 - PAUSING at intermediatePose1");
                    telemetry.addData("Pause Time Remaining", (INTERMEDIATE_PAUSE_DURATION - pauseElapsedTime) + "ms");

                    if (pauseElapsedTime >= INTERMEDIATE_PAUSE_DURATION) {
                        pausingAtIntermediate = false;
                        holdPosition = null;
                        follower.followPath(returnFromIntermediate, true);
                        setPathState(PathState.RETURN_FROM_INTERMEDIATE);
                    }
                }
                break;

            case RETURN_FROM_INTERMEDIATE:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_ROUND_1);
                }
                telemetry.addData("Cycle", "1 - Returning to shoot position");
                break;

            case SHOOT_ROUND_1:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_INTAKE_4);
                    }
                }
                telemetry.addData("Cycle", "1 - Shooting");
                break;

            // ========== CYCLE 2: intakePoint4 -> shootPose ==========
            case DRIVE_TO_INTAKE_4:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake4, true);
                    setPathState(PathState.RETURN_FROM_INTAKE_4);
                }
                telemetry.addData("Cycle", "2 - Driving to intakePoint4");
                break;

            case RETURN_FROM_INTAKE_4:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(returnFromIntake4, true);
                    setPathState(PathState.SHOOT_ROUND_2);
                }
                telemetry.addData("Cycle", "2 - Returning to shoot position");
                break;

            case SHOOT_ROUND_2:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_INTAKE_5);
                    }
                }
                telemetry.addData("Cycle", "2 - Shooting");
                break;

            // ========== CYCLE 3: intakePoint5 -> shootPose ==========
            case DRIVE_TO_INTAKE_5:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(driveToIntake5, true);
                    setPathState(PathState.RETURN_FROM_INTAKE_5);
                }
                telemetry.addData("Cycle", "3 - Driving to intakePoint5");
                break;

            case RETURN_FROM_INTAKE_5:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(returnFromIntake5, true);
                    setPathState(PathState.SHOOT_ROUND_3);
                }
                telemetry.addData("Cycle", "3 - Returning to shoot position");
                break;

            case SHOOT_ROUND_3:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        returnTurretToHome();
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
                telemetry.addData("Status", "Driving to park position");
                break;

            case COMPLETE:
                shooterSystem.stop();
                stopIntake();
                telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
                telemetry.addData("Total Cycles", "3");
                telemetry.addData("Final Position", parkPose.toString());
                break;

            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        telemetry.addData("New State", newState.toString());
    }

    private void startIntake() {
        intakeMotor.setPower(-1);
        intakeActive = true;
    }

    private void stopIntake() {
        intakeMotor.setPower(0);
        intakeActive = false;
    }

    private void startContinuousShooting() {
        shootingState = ShootingState.SHOOTING;
        shootingTimer.resetTimer();
        shooterSystem.shoot();
        intakeMotor.setPower(-0.85);
    }

    private void updateShootingStateMachine() {
        switch (shootingState) {
            case SHOOTING:
                long elapsedTime = (long)(shootingTimer.getElapsedTimeSeconds() * 1000);
                telemetry.addData("Shooting State", "Continuous Shooting");
                telemetry.addData("Elapsed Time", elapsedTime + "ms");

                if (elapsedTime >= CONTINUOUS_SHOOT_DURATION) {
                    intakeMotor.setPower(0);
                    shooterSystem.homeServo();
                    shootingState = ShootingState.IDLE;
                }
                break;

            case IDLE:
                break;
        }
    }

    // RED SIDE: Turret angle is positive (opposite of BLUE)
    private void alignTurretDynamic() {
        pinpoint.update();
        double imuHeading = Math.toDegrees(pinpoint.getPosition().getHeading(AngleUnit.RADIANS));

        // For RED side, robot starts at 0° heading
        // relativeHeading is just the current heading
        double relativeHeading = imuHeading;

        // Target angle calculation: +44° relative to RED's perspective
        double targetAngle = 44 - relativeHeading;

        targetAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, targetAngle));
        moveTurretToAngle(targetAngle, 1);
    }

    private void moveTurretToAngle(double angleDegrees, double power) {
        int targetTicks = (int) Math.round(angleDegrees * TICKS_PER_DEGREE);
        turretRotator.setTargetPosition(targetTicks);
        turretRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretRotator.setPower(Math.abs(power));
    }

    private void returnTurretToHome() {
        moveTurretToAngle(0, 0.8);
    }

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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_TO_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootingTimer = new Timer();
        intermediatePauseTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        buildPaths();
        follower.setPose(startPose);

        telemetry.addLine("=== RED SIDE AUTONOMOUS - MODIFIED STRATEGY ===");
        telemetry.addLine("Cycle 1: intakeEndPoint -> intermediatePose1 (300ms pause) -> shootPose");
        telemetry.addLine("Cycle 2: intakePoint4 -> shootPose");
        telemetry.addLine("Cycle 3: intakePoint5 -> shootPose");
        telemetry.addLine("Park: Drive to (110, 72)");
        telemetry.addLine("No intake waits - continuous intake during movement");
        telemetry.addLine("Ready to start!");
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        alignTurretDynamic();
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Shooting State", shootingState.toString());
        telemetry.addData("Intake Active", intakeActive);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}