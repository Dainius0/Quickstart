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

@Autonomous
public class BLUEPedroAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, shootingTimer;

    // Subsystems
    private ShooterSystem shooterSystem;

    // Hardware
    private DcMotor intakeMotor1, intakeMotor2, turretRotator;
    private IMU imu;

    // Velocity stability requirements for shooting
    private static final long VELOCITY_STABLE_TIME_REQUIRED = 150;  // ms
    private static final long INTAKE_PULSE_DURATION = 300;  // ms per shot

    // Turret constants
    private static final int TICKS_PER_ROTATION = 1872;
    private static final double TICKS_PER_DEGREE = (double) TICKS_PER_ROTATION / 360.0;
    private static final double MIN_TURRET_ANGLE = -90.0;
    private static final double MAX_TURRET_ANGLE = 90.0;

    // Shooting state variables
    private int shotsRemaining = 0;
    private int totalShots = 4;
    private ShootingState shootingState = ShootingState.IDLE;
    private boolean velocityWasInRange = false;
    private long velocityInRangeStartTime = 0;

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
        DRIVE_TO_POINT_5,      // NEW: Fourth collection sequence
        DRIVE_TO_POINT_6,      // NEW
        DRIVE_TO_SHOOT_POS_4,  // NEW
        SHOOT_FOURTH_ROUND,    // NEW
        DRIVE_TO_PARK,         // NEW: Park at (48, 96)
        COMPLETE
    }

    public enum ShootingState {
        IDLE,
        SPINNING_UP,
        WAITING_FOR_VELOCITY,
        SHOOTING,
        SHOT_COMPLETE
    }

    PathState pathState;

    private final Pose startPose = new Pose(22.5, 120.5, Math.toRadians(180));
    private final Pose shootPose = new Pose(48, 96, Math.toRadians(180));
    private final Pose point1Pose = new Pose(48, 82, Math.toRadians(180));
    private final Pose point2Pose = new Pose(20, 82, Math.toRadians(180));
    private final Pose point3Pose = new Pose(48, 58, Math.toRadians(180));
    private final Pose point4Pose = new Pose(20, 58, Math.toRadians(180));
    private final Pose point5Pose = new Pose(48, 33, Math.toRadians(180));  // NEW
    private final Pose point6Pose = new Pose(15, 33, Math.toRadians(180));  // NEW
    private final Pose parkPose = new Pose(48, 96, Math.toRadians(180));    // NEW

    private PathChain driveStartPosShootPos, driveToPoint1, driveToPoint2;
    private PathChain driveToShootPos2, driveToPoint3, driveToPoint4;
    private PathChain driveToShootPos3, driveToPoint5, driveToPoint6;      // NEW
    private PathChain driveToShootPos4, driveToPark;                        // NEW

    public void buildPaths() {
        // First shooting sequence paths
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

        // Second shooting sequence paths
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

        // Third shooting sequence path
        driveToShootPos3 = follower.pathBuilder()
                .addPath(new BezierLine(point4Pose, shootPose))
                .setLinearHeadingInterpolation(point4Pose.getHeading(), shootPose.getHeading())
                .build();

        // NEW: Fourth collection sequence paths
        driveToPoint5 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, point5Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), point5Pose.getHeading())
                .build();

        driveToPoint6 = follower.pathBuilder()
                .addPath(new BezierLine(point5Pose, point6Pose))
                .setLinearHeadingInterpolation(point5Pose.getHeading(), point6Pose.getHeading())
                .build();

        // NEW: Fourth shooting sequence path
        driveToShootPos4 = follower.pathBuilder()
                .addPath(new BezierLine(point6Pose, shootPose))
                .setLinearHeadingInterpolation(point6Pose.getHeading(), shootPose.getHeading())
                .build();

        // NEW: Park path
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
                    // Start shooting sequence
                    if (shootingState == ShootingState.IDLE) {
                        startDynamicShooting(totalShots);
                    }

                    // Update shooting state machine
                    updateShootingStateMachine();

                    // Move to next path when all shots are done
                    if (shootingState == ShootingState.IDLE && shotsRemaining == 0) {
                        setPathState(PathState.DRIVE_TO_POINT_1);
                    }
                }
                break;

            case DRIVE_TO_POINT_1:
                if (!follower.isBusy()) {
                    // Start intake before beginning the path to point 1
                    startIntake();
                    follower.followPath(driveToPoint1, true);
                    setPathState(PathState.DRIVE_TO_POINT_2);
                }
                break;

            case DRIVE_TO_POINT_2:
                // Intake should still be running from previous path
                if (!follower.isBusy()) {
                    // REDUCED SPEED for better ball collection
                    follower.setMaxPower(0.8);
                    follower.followPath(driveToPoint2, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POS_2);
                }

                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                telemetry.addData("Drive Speed", "REDUCED (70%)");
                break;

            case DRIVE_TO_SHOOT_POS_2:
                // Intake continues running during this path
                if (!follower.isBusy()) {
                    // RESTORE FULL SPEED and stop intake when arriving at shoot position
                    follower.setMaxPower(1.0);
                    stopIntake();
                    follower.followPath(driveToShootPos2, true);
                    setPathState(PathState.SHOOT_SECOND_ROUND);
                }

                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                break;

            case SHOOT_SECOND_ROUND:
                if (!follower.isBusy()) {
                    // Start shooting sequence
                    if (shootingState == ShootingState.IDLE) {
                        startDynamicShooting(totalShots);
                    }

                    // Update shooting state machine
                    updateShootingStateMachine();

                    // Move to next path when all shots are done
                    if (shootingState == ShootingState.IDLE && shotsRemaining == 0) {
                        setPathState(PathState.DRIVE_TO_POINT_3);
                    }
                }
                break;

            case DRIVE_TO_POINT_3:
                if (!follower.isBusy()) {
                    // Start intake before beginning the path to point 3
                    startIntake();
                    follower.followPath(driveToPoint3, true);
                    setPathState(PathState.DRIVE_TO_POINT_4);
                }
                break;

            case DRIVE_TO_POINT_4:
                // Intake should still be running from previous path
                if (!follower.isBusy()) {
                    // REDUCED SPEED for better ball collection
                    follower.setMaxPower(0.8);
                    follower.followPath(driveToPoint4, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POS_3);
                }

                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                telemetry.addData("Drive Speed", "REDUCED (70%)");
                break;

            case DRIVE_TO_SHOOT_POS_3:
                // Intake continues running during this path
                if (!follower.isBusy()) {
                    // RESTORE FULL SPEED and stop intake when arriving at shoot position
                    follower.setMaxPower(1.0);
                    stopIntake();
                    follower.followPath(driveToShootPos3, true);
                    setPathState(PathState.SHOOT_THIRD_ROUND);
                }

                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                break;

            case SHOOT_THIRD_ROUND:
                if (!follower.isBusy()) {
                    // Start shooting sequence
                    if (shootingState == ShootingState.IDLE) {
                        startDynamicShooting(totalShots);
                    }

                    // Update shooting state machine
                    updateShootingStateMachine();

                    // Move to next path when all shots are done
                    if (shootingState == ShootingState.IDLE && shotsRemaining == 0) {
                        setPathState(PathState.DRIVE_TO_POINT_5);
                    }
                }
                break;

            // NEW: Fourth collection sequence
            case DRIVE_TO_POINT_5:
                if (!follower.isBusy()) {
                    // Start intake before beginning the path to point 5
                    startIntake();
                    follower.followPath(driveToPoint5, true);
                    setPathState(PathState.DRIVE_TO_POINT_6);
                }
                break;

            case DRIVE_TO_POINT_6:
                // Intake should still be running from previous path
                if (!follower.isBusy()) {
                    // REDUCED SPEED for better ball collection
                    follower.setMaxPower(0.8);
                    follower.followPath(driveToPoint6, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POS_4);
                }

                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                telemetry.addData("Drive Speed", "REDUCED (70%)");
                break;

            case DRIVE_TO_SHOOT_POS_4:
                // Intake continues running during this path
                if (!follower.isBusy()) {
                    // RESTORE FULL SPEED and stop intake when arriving at shoot position
                    follower.setMaxPower(1.0);
                    stopIntake();
                    follower.followPath(driveToShootPos4, true);
                    setPathState(PathState.SHOOT_FOURTH_ROUND);
                }

                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                break;

            case SHOOT_FOURTH_ROUND:
                if (!follower.isBusy()) {
                    // Start shooting sequence
                    if (shootingState == ShootingState.IDLE) {
                        startDynamicShooting(totalShots);
                    }

                    // Update shooting state machine
                    updateShootingStateMachine();

                    // Move to park when all shots are done
                    if (shootingState == ShootingState.IDLE && shotsRemaining == 0) {
                        setPathState(PathState.DRIVE_TO_PARK);
                    }
                }
                break;

            // NEW: Drive to park position
            case DRIVE_TO_PARK:
                if (!follower.isBusy()) {
                    follower.followPath(driveToPark, true);
                    setPathState(PathState.COMPLETE);
                }
                break;

            case COMPLETE:
                if (!follower.isBusy()) {
                    // Stop all mechanisms
                    shooterSystem.stop();
                    stopIntake();
                    returnTurretToHome();
                    telemetry.addLine("Autonomous Complete - Parked!");
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

    /**
     * Starts the intake for collecting game elements
     */
    private void startIntake() {
        intakeMotor1.setPower(-0.9);
        intakeMotor2.setPower(-0.9);
        intakeActive = true;
        telemetry.addData("Intake", "Started");
    }

    /**
     * Stops the intake motors
     */
    private void stopIntake() {
        intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);
        intakeActive = false;
        telemetry.addData("Intake", "Stopped");
    }

    /**
     * Starts the dynamic shooting sequence
     * Note: Shooter is already spinning from start() method
     */
    private void startDynamicShooting(int numShots) {
        shotsRemaining = numShots;
        shootingState = ShootingState.SPINNING_UP;
        shootingTimer.resetTimer();

        // Align turret for dynamic shooting
        alignTurretDynamic();

        // Shooter is already spinning - just ensure servo is at home
        shooterSystem.homeServo();

        telemetry.addData("Status", "Starting dynamic shooting");
        telemetry.addData("Shots to fire", numShots);
    }

    /**
     * State machine for shooting process
     */
    private void updateShootingStateMachine() {
        double currentVelocity = shooterSystem.getVelocity();

        switch (shootingState) {
            case SPINNING_UP:
                // Wait a moment for shooter to start spinning
                if (shootingTimer.getElapsedTimeSeconds() > 0.3) {
                    shootingState = ShootingState.WAITING_FOR_VELOCITY;
                    velocityWasInRange = false;
                }
                telemetry.addData("Shooting State", "Spinning Up");
                telemetry.addData("Current Velocity", currentVelocity);
                break;

            case WAITING_FOR_VELOCITY:
                boolean velocityReady = shooterSystem.isAtTargetVelocity(true); // true = dynamic mode

                long currentTime = System.currentTimeMillis();

                if (velocityReady) {
                    if (!velocityWasInRange) {
                        velocityInRangeStartTime = currentTime;
                        velocityWasInRange = true;
                        // Move servo to shoot position when velocity is correct
                        shooterSystem.shoot();
                    }

                    long timeInRange = currentTime - velocityInRangeStartTime;

                    telemetry.addData("Shooting State", "Velocity Stabilizing");
                    telemetry.addData("Time in Range", timeInRange);

                    // Velocity stable long enough, start shooting
                    if (timeInRange >= VELOCITY_STABLE_TIME_REQUIRED) {
                        shootingState = ShootingState.SHOOTING;
                        shootingTimer.resetTimer();

                        // Execute shot - pulse intake motors
                        intakeMotor1.setPower(-1);
                        intakeMotor2.setPower(-1);
                    }
                } else {
                    // Velocity not ready yet - reset tracking but DON'T move servo
                    velocityWasInRange = false;
                    telemetry.addData("Shooting State", "Waiting for Velocity");
                }

                telemetry.addData("Current Velocity", currentVelocity);
                telemetry.addData("Target Velocity", ShooterSystem.VELOCITY_DYNAMIC);
                break;

            case SHOOTING:
                // Wait for intake pulse duration
                if (shootingTimer.getElapsedTimeSeconds() * 1000 >= INTAKE_PULSE_DURATION) {
                    // Stop intake
                    intakeMotor1.setPower(0.0);
                    intakeMotor2.setPower(0.0);

                    shotsRemaining--;
                    shootingState = ShootingState.SHOT_COMPLETE;
                    shootingTimer.resetTimer();
                }
                telemetry.addData("Shooting State", "Firing!");
                break;

            case SHOT_COMPLETE:
                // Wait a moment between shots
                if (shootingTimer.getElapsedTimeSeconds() > 0.2) {
                    if (shotsRemaining > 0) {
                        // Return servo to home before next shot
                        shooterSystem.homeServo();
                        // Start next shot (shooter still spinning)
                        shootingState = ShootingState.WAITING_FOR_VELOCITY;
                        velocityWasInRange = false;
                    } else {
                        // All shots complete - return servo to home
                        // Keep shooter spinning for next shooting sequence
                        shooterSystem.homeServo();
                        shootingState = ShootingState.IDLE;
                    }
                }
                telemetry.addData("Shooting State", "Shot Complete");
                telemetry.addData("Shots Remaining", shotsRemaining);
                break;

            case IDLE:
                // Do nothing
                break;
        }
    }

    /**
     * Aligns turret for dynamic shooting mode
     */
    private void alignTurretDynamic() {
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetAngle = -37 - imuHeading;

        // Clamp within safety limits
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
    }

    private void initializeHardware() {
        // Initialize ShooterSystem subsystem
        shooterSystem = new ShooterSystem(hardwareMap);

        // Initialize intake motors
        intakeMotor1 = hardwareMap.dcMotor.get("intakeMotor1");
        intakeMotor2 = hardwareMap.dcMotor.get("intakeMotor2");
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize turret
        turretRotator = hardwareMap.dcMotor.get("turretRotator");
        turretRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretRotator.setTargetPosition(0);
        turretRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretRotator.setPower(0.0);

        // Initialize IMU
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
        telemetry.addLine("Ready to start!");
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        // Start shooter spinning immediately and keep it running throughout autonomous
        shooterSystem.spinUp(true); // true = dynamic mode
        telemetry.addLine("Shooter spinning up for autonomous");
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Shooting State", shootingState.toString());
        telemetry.addData("Shots Remaining", shotsRemaining);
        telemetry.addData("Intake Active", intakeActive);
        telemetry.addData("Shooter Velocity", shooterSystem.getVelocity());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}