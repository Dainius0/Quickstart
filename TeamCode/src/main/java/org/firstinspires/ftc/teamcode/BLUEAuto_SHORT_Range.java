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


public class BLUEAuto_SHORT_Range extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, shootingTimer;

    // Subsystems
    private ShooterSystem shooterSystem;

    // Hardware - Single intake motor
    private DcMotor intakeMotor, turretRotator;
    private IMU imu;

    // Continuous shooting constants
    private static final long CONTINUOUS_SHOOT_DURATION = 1500;  // 1.5 seconds of continuous shooting

    // Turret constants
    private static final int TICKS_PER_ROTATION = 1872;
    private static final double TICKS_PER_DEGREE = (double) TICKS_PER_ROTATION / 360.0;
    private static final double MIN_TURRET_ANGLE = -90.0;
    private static final double MAX_TURRET_ANGLE = 90.0;

    // Shooting state variables
    private ShootingState shootingState = ShootingState.IDLE;

    // Intake tracking
    private boolean intakeActive = false;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_TO_POINT_1,
        DRIVE_TO_POINT_2,
        DRIVE_TO_SHOOT_POS_2,
        SHOOT_SECOND_ROUND,
        DRIVE_TO_POINT_3,
        DRIVE_TO_POINT_4,
        DRIVE_TO_SHOOT_POS_3,
        SHOOT_THIRD_ROUND,
        DRIVE_TO_POINT_5,
        DRIVE_TO_POINT_6,
        DRIVE_TO_SHOOT_POS_4,
        SHOOT_FOURTH_ROUND,
        DRIVE_TO_PARK,
        COMPLETE
    }

    public enum ShootingState {
        IDLE,
        SHOOTING
    }

    PathState pathState;

    // UPDATED: New start and end positions
    private final Pose startPose = new Pose(34, 131, Math.toRadians(180));
    private final Pose shootPose = new Pose(48, 96, Math.toRadians(180));
    private final Pose point1Pose = new Pose(48, 82, Math.toRadians(180));
    private final Pose point2Pose = new Pose(20, 82, Math.toRadians(180));
    private final Pose point3Pose = new Pose(48, 58, Math.toRadians(180));
    private final Pose point4Pose = new Pose(20, 58, Math.toRadians(180));
    private final Pose point5Pose = new Pose(48, 33, Math.toRadians(180));
    private final Pose point6Pose = new Pose(15, 33, Math.toRadians(180));
    private final Pose parkPose = new Pose(48, 60, Math.toRadians(180));

    private PathChain driveStartPosShootPos, driveToPoint1, driveToPoint2;
    private PathChain driveToShootPos2, driveToPoint3, driveToPoint4;
    private PathChain driveToShootPos3, driveToPoint5, driveToPoint6;
    private PathChain driveToShootPos4, driveToPark;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveToPoint1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, point1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), point1Pose.getHeading())
                .build();

        driveToPoint2 = follower.pathBuilder()
                .addPath(new BezierLine(point1Pose, point2Pose))
                .setLinearHeadingInterpolation(point1Pose.getHeading(), point2Pose.getHeading())
                .build();

        driveToShootPos2 = follower.pathBuilder()
                .addPath(new BezierLine(point2Pose, shootPose))
                .setLinearHeadingInterpolation(point2Pose.getHeading(), shootPose.getHeading())
                .build();

        driveToPoint3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, point3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), point3Pose.getHeading())
                .build();

        driveToPoint4 = follower.pathBuilder()
                .addPath(new BezierLine(point3Pose, point4Pose))
                .setLinearHeadingInterpolation(point3Pose.getHeading(), point4Pose.getHeading())
                .build();

        driveToShootPos3 = follower.pathBuilder()
                .addPath(new BezierLine(point4Pose, shootPose))
                .setLinearHeadingInterpolation(point4Pose.getHeading(), shootPose.getHeading())
                .build();

        driveToPoint5 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, point5Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), point5Pose.getHeading())
                .build();

        driveToPoint6 = follower.pathBuilder()
                .addPath(new BezierLine(point5Pose, point6Pose))
                .setLinearHeadingInterpolation(point5Pose.getHeading(), point6Pose.getHeading())
                .build();

        driveToShootPos4 = follower.pathBuilder()
                .addPath(new BezierLine(point6Pose, shootPose))
                .setLinearHeadingInterpolation(point6Pose.getHeading(), shootPose.getHeading())
                .build();

        driveToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_POINT_1);
                    }
                }
                break;

            case DRIVE_TO_POINT_1:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(driveToPoint1, true);
                    setPathState(PathState.DRIVE_TO_POINT_2);
                }
                break;

            case DRIVE_TO_POINT_2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(driveToPoint2, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POS_2);
                }
                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                break;

            case DRIVE_TO_SHOOT_POS_2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    stopIntake();
                    follower.followPath(driveToShootPos2, true);
                    setPathState(PathState.SHOOT_SECOND_ROUND);
                }
                telemetry.addData("danius", "nubas");

                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                break;

            case SHOOT_SECOND_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_POINT_3);
                    }
                }
                break;

            case DRIVE_TO_POINT_3:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(driveToPoint3, true);
                    setPathState(PathState.DRIVE_TO_POINT_4);
                }
                break;

            case DRIVE_TO_POINT_4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(driveToPoint4, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POS_3);
                }
                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                break;

            case DRIVE_TO_SHOOT_POS_3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    stopIntake();
                    follower.followPath(driveToShootPos3, true);
                    setPathState(PathState.SHOOT_THIRD_ROUND);
                }
                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                break;

            case SHOOT_THIRD_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_POINT_5);
                    }
                }
                break;

            case DRIVE_TO_POINT_5:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(driveToPoint5, true);
                    setPathState(PathState.DRIVE_TO_POINT_6);
                }
                break;

            case DRIVE_TO_POINT_6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(driveToPoint6, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POS_4);
                }
                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                break;

            case DRIVE_TO_SHOOT_POS_4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    stopIntake();
                    follower.followPath(driveToShootPos4, true);
                    setPathState(PathState.SHOOT_FOURTH_ROUND);
                }
                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                break;

            case SHOOT_FOURTH_ROUND:
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
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy()) {
                    follower.followPath(driveToPark, true);
                    setPathState(PathState.COMPLETE);
                }
                break;

            case COMPLETE:
                if (!follower.isBusy()) {
                    shooterSystem.stop();
                    stopIntake();
                    returnTurretToHome();
                    telemetry.addLine("Autonomous Complete - Parked at (48, 60)!");
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
    }

    private void startIntake() {
        intakeMotor.setPower(1);
        intakeActive = true;
        telemetry.addData("Intake", "Started");
    }

    private void stopIntake() {
        intakeMotor.setPower(0);
        intakeActive = false;
        telemetry.addData("Intake", "Stopped");
    }

    /**
     * Starts continuous shooting for 1.5 seconds
     * Shooter spins, servo extends, intake runs continuously
     */
    private void startContinuousShooting() {
        shootingState = ShootingState.SHOOTING;
        shootingTimer.resetTimer();

        // Align turret for shooting
        alignTurretDynamic();

        // Shooter is already spinning - move servo to shoot position
        shooterSystem.shoot();

        // Start intake continuously
        intakeMotor.setPower(1);

        telemetry.addData("Status", "Starting continuous shooting for 1.5s");
    }

    /**
     * Simplified state machine for continuous shooting
     * Runs shooter + intake for 1.5 seconds, then stops
     */
    private void updateShootingStateMachine() {
        double currentVelocity = shooterSystem.getVelocity();

        switch (shootingState) {
            case SHOOTING:
                long elapsedTime = (long)(shootingTimer.getElapsedTimeSeconds() * 1000);

                telemetry.addData("Shooting State", "Continuous Shooting");
                telemetry.addData("Elapsed Time", elapsedTime + "ms");
                telemetry.addData("Remaining Time", (CONTINUOUS_SHOOT_DURATION - elapsedTime) + "ms");
                telemetry.addData("Current Velocity", currentVelocity);

                if (elapsedTime >= CONTINUOUS_SHOOT_DURATION) {
                    // Shooting duration complete - stop everything
                    intakeMotor.setPower(0);
                    shooterSystem.homeServo();
                    shootingState = ShootingState.IDLE;
                    telemetry.addData("Action", "Shooting Complete!");
                }
                break;

            case IDLE:
                // Do nothing
                break;
        }
    }

    private void alignTurretDynamic() {
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetAngle = -37 + imuHeading;  // Changed from minus to plus
        targetAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, targetAngle));
        moveTurretToAngle(targetAngle, 0.8);
        telemetry.addData("Turret Target Angle", targetAngle);
    }

    private void moveTurretToAngle(double angleDegrees, double power) {
        int targetTicks = (int) Math.round(angleDegrees * TICKS_PER_DEGREE);
        turretRotator.setTargetPosition(targetTicks);
        turretRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretRotator.setPower(Math.abs(power));
    }

    private void returnTurretToHome() {
        moveTurretToAngle(0, 0.8);
        telemetry.addData("Turret", "Returning to home (0 degrees)");
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
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootingTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        buildPaths();
        follower.setPose(startPose);

        telemetry.addLine("Initialization Complete");
        telemetry.addLine("Starting Position: (32, 136) @ 270°");
        telemetry.addLine("Parking Position: (48, 60) @ 270°");
        telemetry.addLine("Ready to start!");
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        // Start shooter spinning and keep it running
        shooterSystem.spinUp(true);
        telemetry.addLine("Shooter spinning up for continuous autonomous");
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Shooting State", shootingState.toString());
        telemetry.addData("Intake Active", intakeActive);
        telemetry.addData("Shooter Velocity", shooterSystem.getVelocity());
        telemetry.addData("Turret Position", turretRotator.getCurrentPosition());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}