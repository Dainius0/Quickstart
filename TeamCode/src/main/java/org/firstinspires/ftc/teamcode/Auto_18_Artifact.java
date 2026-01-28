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

@Autonomous(name = "BLUE 18 Artifact Auto")
public class Auto_18_Artifact extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, shootingTimer, intakeWaitTimer;

    // Subsystems
    private ShooterSystem shooterSystem;

    // Hardware
    private DcMotor intakeMotor, turretRotator;
    private IMU imu;

    // Continuous shooting constants
    private static final long CONTINUOUS_SHOOT_DURATION = 1100;
    private static final long INTAKE_WAIT_DURATION = 1400;

    // Turret constants
    private static final int TICKS_PER_ROTATION = 1872;
    private static final double TICKS_PER_DEGREE = (double) TICKS_PER_ROTATION / 360.0;
    private static final double MIN_TURRET_ANGLE = -90.0;
    private static final double MAX_TURRET_ANGLE = 90.0;

    // Drive speed constants
    private static final double INTAKE_DRIVE_POWER = 0.9;
    private static final double INTAKE_POINT3_POWER = 0.75;  // Slower for intakePoint3
    private static final double NORMAL_DRIVE_POWER = 1.0;

    // State variables
    private ShootingState shootingState = ShootingState.IDLE;
    private boolean intakeActive = false;
    private boolean waitingForIntake = false;

    public enum PathState {
        DRIVE_TO_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_TO_INTAKE_1,
        WAIT_AND_INTAKE_1,
        RETURN_TO_SHOOT_1,
        SHOOT_SECOND_ROUND,
        DRIVE_TO_INTAKE_2,
        WAIT_AND_INTAKE_2,
        RETURN_TO_SHOOT_2,
        SHOOT_THIRD_ROUND,
        DRIVE_TO_INTAKE_3,
        WAIT_AND_INTAKE_3,
        RETURN_TO_SHOOT_3,
        SHOOT_FOURTH_ROUND,
        DRIVE_TO_INTAKE_4,
        WAIT_AND_INTAKE_4,
        RETURN_TO_SHOOT_4,
        SHOOT_FIFTH_ROUND,
        DRIVE_TO_INTAKE_5,
        WAIT_AND_INTAKE_5,
        RETURN_TO_SHOOT_5,
        SHOOT_SIXTH_ROUND,
        DRIVE_TO_FINAL_POSITION,
        COMPLETE
    }

    public enum ShootingState {
        IDLE,
        SHOOTING
    }

    PathState pathState;

    // BLUE SIDE - Original coordinates, heading = 180°
    private final Pose startPose = new Pose(36, 135, Math.toRadians(180));
    private final Pose shootPose = new Pose(48, 88, Math.toRadians(180));

    // Intake points
    private final Pose intakeEndPoint = new Pose(14.8, 60, Math.toRadians(180));
    private final Pose intakePoint3 = new Pose(11.2, 61.1, Math.toRadians(150));
    private final Pose intakePoint4 = new Pose(17, 82, Math.toRadians(180));
    private final Pose intakePoint5 = new Pose(17, 35, Math.toRadians(180));
    private final Pose finalPose = new Pose(48, 66, Math.toRadians(180));

    // Path chains
    private PathChain driveToShootPos;
    private PathChain driveToIntake1, returnToShoot1;
    private PathChain driveToIntake2, returnToShoot2;
    private PathChain driveToIntake3, returnToShoot3;
    private PathChain driveToIntake4, returnToShoot4;
    private PathChain driveToIntake5, returnToShoot5;
    private PathChain driveToFinalPosition;

    public void buildPaths() {

        // Initial drive to shooting position
        driveToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 1: shootPose -> intakeEndPoint
        Pose control1_1 = new Pose(48, 62, Math.toRadians(180));
        Pose control2_1 = new Pose(50.2, 59.7, Math.toRadians(180));
        driveToIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control1_1, control2_1, intakeEndPoint))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeEndPoint.getHeading())
                .build();

        // Return: intakeEndPoint -> shootPose
        Pose control1_1r = new Pose(33, 61, Math.toRadians(180));
        Pose control2_1r = new Pose(47, 74.7, Math.toRadians(180));
        returnToShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(intakeEndPoint, control1_1r, control2_1r, shootPose))
                .setLinearHeadingInterpolation(intakeEndPoint.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 2: shootPose -> intakePoint3
        Pose control1_2 = new Pose(45.2, 57.3, Math.toRadians(180));
        Pose control2_2 = new Pose(23.5, 63, Math.toRadians(160));
        driveToIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control1_2, control2_2, intakePoint3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint3.getHeading())
                .build();

        // Return: intakePoint3 -> shootPose
        Pose control1_2r = new Pose(27.5, 64.7, Math.toRadians(160));
        Pose control2_2r = new Pose(36, 66, Math.toRadians(180));
        returnToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint3, control1_2r, control2_2r, shootPose))
                .setLinearHeadingInterpolation(intakePoint3.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 3: shootPose -> intakePoint3
        Pose control1_3 = new Pose(45.2, 57.3, Math.toRadians(180));
        Pose control2_3 = new Pose(23.5, 63, Math.toRadians(160));
        driveToIntake3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control1_3, control2_3, intakePoint3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint3.getHeading())
                .build();

        // Return: intakePoint3 -> shootPose
        Pose control1_3r = new Pose(27.5, 64.7, Math.toRadians(160));
        Pose control2_3r = new Pose(36, 66, Math.toRadians(180));
        returnToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint3, control1_3r, control2_3r, shootPose))
                .setLinearHeadingInterpolation(intakePoint3.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 4: shootPose -> intakePoint4
        Pose control1_4 = new Pose(44.8, 87, Math.toRadians(180));
        Pose control2_4 = new Pose(40.5, 83.3, Math.toRadians(180));
        driveToIntake4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control1_4, control2_4, intakePoint4))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint4.getHeading())
                .build();

        // Return: intakePoint4 -> shootPose
        Pose control1_4r = new Pose(40.5, 83.3, Math.toRadians(180));
        Pose control2_4r = new Pose(44.6, 84.4, Math.toRadians(180));
        returnToShoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint4, control1_4r, control2_4r, shootPose))
                .setLinearHeadingInterpolation(intakePoint4.getHeading(), shootPose.getHeading())
                .build();

        // CYCLE 5: shootPose -> intakePoint5
        Pose control1_5 = new Pose(49, 47, Math.toRadians(180));
        Pose control2_5 = new Pose(48.5, 33.8, Math.toRadians(180));
        driveToIntake5 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control1_5, control2_5, intakePoint5))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePoint5.getHeading())
                .build();

        // Return: intakePoint5 -> shootPose
        Pose control1_5r = new Pose(30, 36, Math.toRadians(180));
        Pose control2_5r = new Pose(48, 56, Math.toRadians(180));
        returnToShoot5 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePoint5, control1_5r, control2_5r, shootPose))
                .setLinearHeadingInterpolation(intakePoint5.getHeading(), shootPose.getHeading())
                .build();

        // FINAL POSITION: shootPose -> finalPose
        Pose controlFinal1 = new Pose(48, 81, Math.toRadians(180));
        Pose controlFinal2 = new Pose(48, 71, Math.toRadians(180));
        driveToFinalPosition = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlFinal1, controlFinal2, finalPose))
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
                        setPathState(PathState.DRIVE_TO_INTAKE_1);
                    }
                }
                break;

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
                    stopIntake();  // Stop intake before returning
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(returnToShoot1, true);
                    setPathState(PathState.RETURN_TO_SHOOT_1);
                }
                break;

            case RETURN_TO_SHOOT_1:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SECOND_ROUND);
                }
                telemetry.addData("Status", "Returning to shoot position");
                break;

            case SHOOT_SECOND_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_INTAKE_2);
                    }
                }
                break;

            case DRIVE_TO_INTAKE_2:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_POINT3_POWER);  // Set to 0.75 for intakePoint3
                    follower.followPath(driveToIntake2, true);
                    setPathState(PathState.WAIT_AND_INTAKE_2);
                }
                telemetry.addData("Status", "Driving to intake 2 (Point 3 - SLOW)");
                break;

            case WAIT_AND_INTAKE_2:
                if (!follower.isBusy()) {
                    if (!waitingForIntake) {
                        waitingForIntake = true;
                        intakeWaitTimer.resetTimer();
                    }

                    long elapsedTime = (long)(intakeWaitTimer.getElapsedTimeSeconds() * 1000);
                    telemetry.addData("Wait Time Remaining (Point 2)", (INTAKE_WAIT_DURATION - elapsedTime) + "ms");
                    telemetry.addData("Status", "PAUSING at intake point 2");

                    if (elapsedTime >= INTAKE_WAIT_DURATION) {
                        waitingForIntake = false;
                        stopIntake();  // Stop intake before returning
                        follower.setMaxPower(NORMAL_DRIVE_POWER);
                        follower.followPath(returnToShoot2, true);
                        setPathState(PathState.RETURN_TO_SHOOT_2);
                    }
                }
                break;

            case RETURN_TO_SHOOT_2:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_THIRD_ROUND);
                }
                telemetry.addData("Status", "Returning to shoot position");
                break;

            case SHOOT_THIRD_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_INTAKE_3);
                    }
                }
                break;

            case DRIVE_TO_INTAKE_3:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.setMaxPower(INTAKE_POINT3_POWER);  // Set to 0.75 for intakePoint3
                    follower.followPath(driveToIntake3, true);
                    setPathState(PathState.WAIT_AND_INTAKE_3);
                }
                telemetry.addData("Status", "Driving to intake 3 (Point 3 - SLOW)");
                break;

            case WAIT_AND_INTAKE_3:
                if (!follower.isBusy()) {
                    if (!waitingForIntake) {
                        waitingForIntake = true;
                        intakeWaitTimer.resetTimer();
                    }

                    long elapsedTime = (long)(intakeWaitTimer.getElapsedTimeSeconds() * 1000);
                    telemetry.addData("Wait Time Remaining (Point 3)", (INTAKE_WAIT_DURATION - elapsedTime) + "ms");
                    telemetry.addData("Status", "PAUSING at intake point 3");

                    if (elapsedTime >= INTAKE_WAIT_DURATION) {
                        waitingForIntake = false;
                        stopIntake();  // Stop intake before returning
                        follower.setMaxPower(NORMAL_DRIVE_POWER);
                        follower.followPath(returnToShoot3, true);
                        setPathState(PathState.RETURN_TO_SHOOT_3);
                    }
                }
                break;

            case RETURN_TO_SHOOT_3:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FOURTH_ROUND);
                }
                telemetry.addData("Status", "Returning to shoot position");
                break;

            case SHOOT_FOURTH_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_INTAKE_4);
                    }
                }
                break;

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
                    stopIntake();  // Stop intake before returning
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(returnToShoot4, true);
                    setPathState(PathState.RETURN_TO_SHOOT_4);
                }
                break;

            case RETURN_TO_SHOOT_4:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIFTH_ROUND);
                }
                telemetry.addData("Status", "Returning to shoot position");
                break;

            case SHOOT_FIFTH_ROUND:
                if (!follower.isBusy()) {
                    if (shootingState == ShootingState.IDLE) {
                        startContinuousShooting();
                    }
                    updateShootingStateMachine();
                    if (shootingState == ShootingState.IDLE) {
                        setPathState(PathState.DRIVE_TO_INTAKE_5);
                    }
                }
                break;

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
                    stopIntake();  // Stop intake before returning
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(returnToShoot5, true);
                    setPathState(PathState.RETURN_TO_SHOOT_5);
                }
                break;

            case RETURN_TO_SHOOT_5:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SIXTH_ROUND);
                }
                telemetry.addData("Status", "Returning to shoot position");
                break;

            case SHOOT_SIXTH_ROUND:
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
                telemetry.addData("Status", "Driving to final position");
                break;

            case COMPLETE:
                if (!follower.isBusy()) {
                    shooterSystem.stop();
                    stopIntake();
                    telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
                    telemetry.addData("Final Position", "x=48, y=66, heading=180°");
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
        intakeMotor.setPower(-0.8);
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

    // BLUE SIDE: Turret angle is negative
    private void alignTurretDynamic() {
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetAngle = -46 + imuHeading;  // Negative for BLUE side
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

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Reset IMU yaw to 0
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

        telemetry.addLine("=== BLUE SIDE AUTONOMOUS ===");
        telemetry.addLine("6 Shooting Cycles");
        telemetry.addLine("Intake pauses at points 2 and 3 (1400ms each)");
        telemetry.addLine("IntakePoint3 paths set to 0.75 power");
        telemetry.addLine("IMU initialized and reset");
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