package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
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

@Autonomous(name = "18 Artifact Auto Extended")
public class Auto_18_Artifact extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, shootingTimer, intakeWaitTimer;

    // Subsystems
    private ShooterSystem shooterSystem;

    // Hardware
    private DcMotor intakeMotor, turretRotator;
    private IMU imu;

    // Continuous shooting constants
    private static final long CONTINUOUS_SHOOT_DURATION = 1300;
    private static final long INTAKE_WAIT_DURATION = 1500;

    // Turret constants
    private static final int TICKS_PER_ROTATION = 1872;
    private static final double TICKS_PER_DEGREE = (double) TICKS_PER_ROTATION / 360.0;
    private static final double MIN_TURRET_ANGLE = -90.0;
    private static final double MAX_TURRET_ANGLE = 90.0;

    // Drive speed constants
    private static final double INTAKE_DRIVE_POWER = 0.7;
    private static final double NORMAL_DRIVE_POWER = 1.0;

    // State variables
    private ShootingState shootingState = ShootingState.IDLE;
    private boolean intakeActive = false;
    private boolean waitingForIntake = false;

    public enum PathState {
        DRIVE_TO_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_TO_MIDPOINT,
        DRIVE_TO_INTAKE_START,
        INTAKE_FORWARD,
        RETURN_TO_MIDPOINT,
        DRIVE_TO_SHOOT_POS_2,
        SHOOT_SECOND_ROUND,
        DRIVE_TO_MIDPOINT_3,
        DRIVE_TO_INTAKE_3,
        WAIT_AND_INTAKE_3,
        RETURN_TO_MIDPOINT_3,
        DRIVE_TO_SHOOT_POS_3,
        SHOOT_THIRD_ROUND,
        DRIVE_TO_MIDPOINT_4,
        INTAKE_FORWARD_4,
        DRIVE_TO_SHOOT_POS_4,
        SHOOT_FOURTH_ROUND,
        DRIVE_TO_MIDPOINT_5,
        INTAKE_FORWARD_5,
        DRIVE_TO_SHOOT_POS_5,
        SHOOT_FIFTH_ROUND,
        DRIVE_TO_FINAL_POSITION,
        COMPLETE
    }

    public enum ShootingState {
        IDLE,
        SHOOTING
    }

    PathState pathState;

    // Positions
    private final Pose startPose = new Pose(36, 131, Math.toRadians(180));
    private final Pose shootPose = new Pose(48, 96, Math.toRadians(180));
    private final Pose midPoint = new Pose(48, 56, Math.toRadians(180));
    private final Pose intakeEndPoint = new Pose(14, 56, Math.toRadians(180));
    private final Pose midPoint3 = new Pose(48, 58.5, Math.toRadians(180));
    private final Pose intakePoint3 = new Pose(15.5, 58.5, Math.toRadians(150));
    private final Pose midPoint4 = new Pose(48, 80, Math.toRadians(180));
    private final Pose intakePoint4 = new Pose(17, 80, Math.toRadians(180));
    private final Pose midPoint5 = new Pose(48, 33, Math.toRadians(180));
    private final Pose intakePoint5 = new Pose(17, 33, Math.toRadians(180));
    private final Pose finalPose = new Pose(48, 60, Math.toRadians(180));

    // Path chains
    private PathChain driveToShootPos, driveToMidpoint, driveToIntakeStart;
    private PathChain intakeForward, returnToMidpoint, driveToShootPos2;
    private PathChain driveToMidpoint3, driveToIntake3, returnToMidpoint3, driveToShootPos3;
    private PathChain driveToMidpoint4, intakeForward4, driveToShootPos4;
    private PathChain driveToMidpoint5, intakeForward5, driveToShootPos5;
    private PathChain driveToFinalPosition;

    public void buildPaths() {
        // Initial drive to shooting position
        driveToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Drive to midpoint (48, 56)
        driveToMidpoint = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, midPoint))
                .setLinearHeadingInterpolation(shootPose.getHeading(), midPoint.getHeading())
                .build();

        // Prepare for intake
        driveToIntakeStart = follower.pathBuilder()
                .addPath(new BezierLine(midPoint, midPoint))
                .setLinearHeadingInterpolation(midPoint.getHeading(), midPoint.getHeading())
                .build();

        // Intake forward to (14, 56)
        intakeForward = follower.pathBuilder()
                .addPath(new BezierLine(midPoint, intakeEndPoint))
                .setLinearHeadingInterpolation(midPoint.getHeading(), intakeEndPoint.getHeading())
                .build();

        // Return to midpoint (48, 56)
        returnToMidpoint = follower.pathBuilder()
                .addPath(new BezierLine(intakeEndPoint, midPoint))
                .setLinearHeadingInterpolation(intakeEndPoint.getHeading(), midPoint.getHeading())
                .build();

        // Drive back to shooting position
        driveToShootPos2 = follower.pathBuilder()
                .addPath(new BezierLine(midPoint, shootPose))
                .setLinearHeadingInterpolation(midPoint.getHeading(), shootPose.getHeading())
                .build();

        // THIRD CYCLE
        driveToMidpoint3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, midPoint3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), midPoint3.getHeading())
                .build();

        driveToIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(midPoint3, intakePoint3))
                .setLinearHeadingInterpolation(midPoint3.getHeading(), intakePoint3.getHeading())
                .build();

        returnToMidpoint3 = follower.pathBuilder()
                .addPath(new BezierLine(intakePoint3, midPoint3))
                .setLinearHeadingInterpolation(intakePoint3.getHeading(), midPoint3.getHeading())
                .build();

        driveToShootPos3 = follower.pathBuilder()
                .addPath(new BezierLine(midPoint3, shootPose))
                .setLinearHeadingInterpolation(midPoint3.getHeading(), shootPose.getHeading())
                .build();

        // FOURTH CYCLE
        driveToMidpoint4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, midPoint4))
                .setLinearHeadingInterpolation(shootPose.getHeading(), midPoint4.getHeading())
                .build();

        intakeForward4 = follower.pathBuilder()
                .addPath(new BezierLine(midPoint4, intakePoint4))
                .setLinearHeadingInterpolation(midPoint4.getHeading(), intakePoint4.getHeading())
                .build();

        driveToShootPos4 = follower.pathBuilder()
                .addPath(new BezierLine(intakePoint4, shootPose))
                .setLinearHeadingInterpolation(intakePoint4.getHeading(), shootPose.getHeading())
                .build();

        // FIFTH CYCLE
        driveToMidpoint5 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, midPoint5))
                .setLinearHeadingInterpolation(shootPose.getHeading(), midPoint5.getHeading())
                .build();

        intakeForward5 = follower.pathBuilder()
                .addPath(new BezierLine(midPoint5, intakePoint5))
                .setLinearHeadingInterpolation(midPoint5.getHeading(), intakePoint5.getHeading())
                .build();

        driveToShootPos5 = follower.pathBuilder()
                .addPath(new BezierLine(intakePoint5, shootPose))
                .setLinearHeadingInterpolation(intakePoint5.getHeading(), shootPose.getHeading())
                .build();

        // FINAL POSITION
        driveToFinalPosition = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, finalPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finalPose.getHeading())
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
                        setPathState(PathState.DRIVE_TO_MIDPOINT);
                    }
                }
                break;

            case DRIVE_TO_MIDPOINT:
                if (!follower.isBusy()) {
                    follower.followPath(driveToMidpoint, true);
                    setPathState(PathState.DRIVE_TO_INTAKE_START);
                }
                break;

            case DRIVE_TO_INTAKE_START:
                if (!follower.isBusy()) {
                    telemetry.addLine("At midpoint (48, 56) - Starting intake");
                    setPathState(PathState.INTAKE_FORWARD);
                }
                break;

            case INTAKE_FORWARD:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(intakeForward, true);
                    setPathState(PathState.RETURN_TO_MIDPOINT);
                }
                telemetry.addData("Status", "Intaking forward to (14, 56)");
                break;

            case RETURN_TO_MIDPOINT:
                if (!follower.isBusy()) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    stopIntake();
                    follower.followPath(returnToMidpoint, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POS_2);
                }
                break;

            case DRIVE_TO_SHOOT_POS_2:
                if (!follower.isBusy()) {
                    follower.followPath(driveToShootPos2, true);
                    setPathState(PathState.SHOOT_SECOND_ROUND);
                }
                break;

            case SHOOT_SECOND_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_MIDPOINT_3);
                    }
                }
                break;

            case DRIVE_TO_MIDPOINT_3:
                if (!follower.isBusy()) {
                    follower.followPath(driveToMidpoint3, true);
                    setPathState(PathState.DRIVE_TO_INTAKE_3);
                }
                telemetry.addData("Status", "Driving to midpoint 3 (48, 60)");
                break;

            case DRIVE_TO_INTAKE_3:
                if (!follower.isBusy()) {
                    follower.followPath(driveToIntake3, true);
                    setPathState(PathState.WAIT_AND_INTAKE_3);
                }
                break;

            case WAIT_AND_INTAKE_3:
                if (!follower.isBusy()) {
                    if (!waitingForIntake) {
                        waitingForIntake = true;
                        intakeWaitTimer.resetTimer();
                        startIntake();
                    }

                    long elapsedTime = (long)(intakeWaitTimer.getElapsedTimeSeconds() * 1000);
                    telemetry.addData("Wait Time Remaining", (INTAKE_WAIT_DURATION - elapsedTime) + "ms");

                    if (elapsedTime >= INTAKE_WAIT_DURATION) {
                        stopIntake();
                        waitingForIntake = false;
                        follower.followPath(returnToMidpoint3, true);
                        setPathState(PathState.RETURN_TO_MIDPOINT_3);
                    }
                }
                break;

            case RETURN_TO_MIDPOINT_3:
                if (!follower.isBusy()) {
                    follower.followPath(driveToShootPos3, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POS_3);
                }
                break;

            case DRIVE_TO_SHOOT_POS_3:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_THIRD_ROUND);
                }
                break;

            case SHOOT_THIRD_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_MIDPOINT_4);
                    }
                }
                break;

            case DRIVE_TO_MIDPOINT_4:
                if (!follower.isBusy()) {
                    follower.followPath(driveToMidpoint4, true);
                    setPathState(PathState.INTAKE_FORWARD_4);
                }
                telemetry.addData("Status", "Driving to midpoint 4 (48, 82)");
                break;

            case INTAKE_FORWARD_4:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(intakeForward4, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POS_4);
                }
                telemetry.addData("Status", "Intaking forward to (16, 82)");
                break;

            case DRIVE_TO_SHOOT_POS_4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    stopIntake();
                    follower.followPath(driveToShootPos4, true);
                    setPathState(PathState.SHOOT_FOURTH_ROUND);
                }
                break;

            case SHOOT_FOURTH_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_MIDPOINT_5);
                    }
                }
                break;

            case DRIVE_TO_MIDPOINT_5:
                if (!follower.isBusy()) {
                    follower.followPath(driveToMidpoint5, true);
                    setPathState(PathState.INTAKE_FORWARD_5);
                }
                telemetry.addData("Status", "Driving to midpoint 5 (48, 36)");
                break;

            case INTAKE_FORWARD_5:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(intakeForward5, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POS_5);
                }
                telemetry.addData("Status", "Intaking forward to (16, 36)");
                break;

            case DRIVE_TO_SHOOT_POS_5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    stopIntake();
                    follower.followPath(driveToShootPos5, true);
                    setPathState(PathState.SHOOT_FIFTH_ROUND);
                }
                break;

            case SHOOT_FIFTH_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        returnTurretToHome();
                        setPathState(PathState.DRIVE_TO_FINAL_POSITION);
                    }
                }
                break;

            case DRIVE_TO_FINAL_POSITION:
                if (!follower.isBusy()) {
                    follower.followPath(driveToFinalPosition, true);
                    setPathState(PathState.COMPLETE);
                }
                telemetry.addData("Status", "Driving to final position (48, 60)");
                break;

            case COMPLETE:
                if (!follower.isBusy()) {
                    shooterSystem.stop();
                    stopIntake();
                    telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
                    telemetry.addData("Final Position", "x=48, y=60, heading=180°");
                    telemetry.addData("Turret Position", "Home (0 degrees)");
                }
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
        intakeMotor.setPower(1);
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
        intakeMotor.setPower(1);
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

    private void alignTurretDynamic() {
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetAngle = -37 + imuHeading;
        targetAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, targetAngle));
        moveTurretToAngle(targetAngle, 0.8);
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

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_TO_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootingTimer = new Timer();
        intakeWaitTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        buildPaths();
        follower.setPose(startPose);

        telemetry.addLine("=== EXTENDED AUTONOMOUS ===");
        telemetry.addLine("5 Shooting Cycles");
        telemetry.addLine("Ready to start!");
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        shooterSystem.spinUp(true);
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