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
public class REDAuto_SHORT_Range extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, shootingTimer;

    // Subsystems
    private ShooterSystem shooterSystem;

    // Hardware
    private DcMotor intakeMotor1, intakeMotor2, turretRotator;
    private IMU imu;

    // Timed shooting constants (500ms between shots)
    private static final long SHOT_INTERVAL = 500;  // ms between shots
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
    private long lastShotTime = 0;
    private boolean isFirstShotEver = true;  // NEW: Track if this is the very first shot

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
        WAITING_FOR_INTERVAL,  // Wait for 500ms between shots
        SHOOTING
    }

    PathState pathState;

    // RED SIDE COORDINATES - Mirrored across Y-axis from blue side
    private final Pose startPose = new Pose(121.5, 120.5, Math.toRadians(0));
    private final Pose shootPose = new Pose(96, 96, Math.toRadians(0));
    private final Pose point1Pose = new Pose(96, 82, Math.toRadians(0));
    private final Pose point2Pose = new Pose(124, 82, Math.toRadians(0));
    private final Pose point3Pose = new Pose(96, 58, Math.toRadians(0));
    private final Pose point4Pose = new Pose(124, 58, Math.toRadians(0));
    private final Pose point5Pose = new Pose(96, 33, Math.toRadians(0));
    private final Pose point6Pose = new Pose(129, 33, Math.toRadians(0));
    // UPDATED: Mirrored park position from blue (96, 62) -> red (48, 62)
    private final Pose parkPose = new Pose(96, 62, Math.toRadians(0));

    private PathChain driveStartPosShootPos, driveToPoint1, driveToPoint2;
    private PathChain driveToShootPos2, driveToPoint3, driveToPoint4;
    private PathChain driveToShootPos3, driveToPoint5, driveToPoint6;
    private PathChain driveToShootPos4, driveToPark;

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

        // Park path to new position (48, 62)
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
                        startTimedShooting(totalShots);
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
                telemetry.addData("Intake Status", intakeActive ? "ACTIVE - Collecting" : "STOPPED");
                break;

            case SHOOT_SECOND_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startTimedShooting(totalShots);
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE && shotsRemaining == 0) {
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
                        startTimedShooting(totalShots);
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE && shotsRemaining == 0) {
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
                        startTimedShooting(totalShots);
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE && shotsRemaining == 0) {
                        // Return turret to home before parking
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
                    // Ensure turret is at home position
                    returnTurretToHome();
                    telemetry.addLine("Autonomous Complete - Parked at (48, 62)!");
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
        intakeMotor1.setPower(-0.9);
        intakeMotor2.setPower(-0.9);
        intakeActive = true;
        telemetry.addData("Intake", "Started");
    }

    private void stopIntake() {
        intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);
        intakeActive = false;
        telemetry.addData("Intake", "Stopped");
    }

    /**
     * Starts the timed shooting sequence
     * Shooter spins continuously, intake pulses every 500ms
     */
    private void startTimedShooting(int numShots) {
        shotsRemaining = numShots;
        shootingState = ShootingState.WAITING_FOR_INTERVAL;
        lastShotTime = System.currentTimeMillis();

        // Align turret for shooting
        alignTurretDynamic();

        // Shooter is already spinning - move servo to shoot position
        shooterSystem.shoot();

        telemetry.addData("Status", "Starting timed shooting");
        telemetry.addData("Shots to fire", numShots);
    }

    /**
     * Optimized state machine for timed shooting (500ms intervals)
     * PIDF continues running, shooter keeps spinning, only intake timing changes
     * Maximum efficiency - no stabilization delays
     * FIRST SHOT ONLY: 700ms delay (500ms + 200ms extra)
     */
    private void updateShootingStateMachine() {
        long currentTime = System.currentTimeMillis();
        double currentVelocity = shooterSystem.getVelocity();

        switch (shootingState) {
            case WAITING_FOR_INTERVAL:
                // Wait for interval since last shot
                // First shot ever gets 200ms extra delay (700ms total)
                long timeSinceLastShot = currentTime - lastShotTime;
                long requiredInterval = isFirstShotEver ? (SHOT_INTERVAL + 200) : SHOT_INTERVAL;

                telemetry.addData("Shooting State", "Waiting for Interval");
                telemetry.addData("Time Since Last Shot", timeSinceLastShot + "ms");
                telemetry.addData("Required Interval", requiredInterval + "ms");
                telemetry.addData("Next Shot In", (requiredInterval - timeSinceLastShot) + "ms");
                telemetry.addData("Current Velocity", currentVelocity);
                if (isFirstShotEver) {
                    telemetry.addData("FIRST SHOT", "Extra 200ms delay active");
                }

                if (timeSinceLastShot >= requiredInterval) {
                    // Time to shoot - pulse intake immediately
                    shootingState = ShootingState.SHOOTING;
                    shootingTimer.resetTimer();

                    intakeMotor1.setPower(-1);
                    intakeMotor2.setPower(-1);

                    // Clear first shot flag after the first shot begins
                    if (isFirstShotEver) {
                        isFirstShotEver = false;
                        telemetry.addData("Status", "FIRST SHOT FIRED - Normal timing resumes");
                    }

                    telemetry.addData("Action", "FIRING SHOT!");
                }
                break;

            case SHOOTING:
                // Wait for intake pulse duration
                if (shootingTimer.getElapsedTimeSeconds() * 1000 >= INTAKE_PULSE_DURATION) {
                    // Stop intake
                    intakeMotor1.setPower(0.0);
                    intakeMotor2.setPower(0.0);

                    shotsRemaining--;

                    if (shotsRemaining > 0) {
                        // Immediately continue to next shot - no pause
                        shootingState = ShootingState.WAITING_FOR_INTERVAL;
                        lastShotTime = currentTime;
                    } else {
                        // All shots complete
                        shooterSystem.homeServo();
                        shootingState = ShootingState.IDLE;
                    }
                }
                telemetry.addData("Shooting State", "Firing!");
                telemetry.addData("Current Velocity", currentVelocity);
                telemetry.addData("Shots Remaining", shotsRemaining);
                break;

            case IDLE:
                // Do nothing
                break;
        }
    }

    private void alignTurretDynamic() {
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        // RED SIDE: Positive 37° (mirrored from blue's -37°)
        double targetAngle = 32 - imuHeading; // kinda perfect

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

        intakeMotor1 = hardwareMap.dcMotor.get("intakeMotor1");
        intakeMotor2 = hardwareMap.dcMotor.get("intakeMotor2");
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        telemetry.addLine("Ready to start!");
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        // Start shooter spinning and keep it running (PIDF still active)
        shooterSystem.spinUp(true);
        telemetry.addLine("Shooter spinning up for timed autonomous");
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
        telemetry.addData("Turret Position", turretRotator.getCurrentPosition());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}