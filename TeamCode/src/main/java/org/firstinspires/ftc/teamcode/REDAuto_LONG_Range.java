package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class REDAuto_LONG_Range extends LinearOpMode {

    // Shooter velocity targets (ticks per second)
    private static final double SHOOTER_VELOCITY_DYNAMIC = 1420;
    private static final double SHOOTER_VELOCITY_FIXED = 2100;

    // Velocity tolerance for shooting activation
    private static final double VELOCITY_TOLERANCE_LOWER = 40;
    private static final double VELOCITY_TOLERANCE_UPPER = 20;

    // Velocity stability requirements for shooting
    private static final long VELOCITY_STABLE_TIME_REQUIRED = 200;  // ms
    private static final long INTAKE_PULSE_DURATION = 300;  // ms per shot

    // Turret constants
    private static final int TICKS_PER_ROTATION = 1872;
    private static final double TICKS_PER_DEGREE = (double) TICKS_PER_ROTATION / 360.0;
    private static final double MIN_TURRET_ANGLE = -90.0;
    private static final double MAX_TURRET_ANGLE = 90.0;

    // Shooting modes
    private static final int MODE_DYNAMIC = 0;
    private static final int MODE_FIXED = 1;

    // Hardware
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intakeMotor1, intakeMotor2, turretRotator;
    private DcMotorEx shooter;
    private Servo shooterServo, shooterAngle;
    private IMU imu;

    // Servo positions
    private double servoHome = 0.8;
    private double servoShoot = 0.2;
    private double angleHome = 0.58;
    private double angleRaisedDynamic = 0.58;
    private double angleRaisedFixed = 0.61;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();


        waitForStart();
        if (isStopRequested()) return;

        // Example autonomous routine for RED ALLIANCE
        // Shoot 6 preloaded discs in fixed mode
        shootSequence(MODE_FIXED, 6);

        // Drive forward (same direction as blue, field-centric handles orientation)
        driveForTime(-0.7, 0, 0, 600);

        // Return turret to starting position
        returnTurretToHome();

        // Stop all motors and brake shooter
        stopAllMotors();
        brakeShooter();
    }

    private void initializeHardware() {
        // Drivetrain
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Mechanisms
        intakeMotor1 = hardwareMap.dcMotor.get("intakeMotor1");
        intakeMotor2 = hardwareMap.dcMotor.get("intakeMotor2");
        turretRotator = hardwareMap.dcMotor.get("turretRotator");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Turret setup
        turretRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretRotator.setTargetPosition(0);
        turretRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretRotator.setPower(0.0);

        // Servos
        shooterServo = hardwareMap.servo.get("shooterServo");
        shooterAngle = hardwareMap.servo.get("shooterAngle");
        shooterServo.setPosition(servoHome);
        shooterAngle.setPosition(angleHome);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        telemetry.addLine("RED ALLIANCE Initialized");
        telemetry.update();
    }

    /**
     * Shoots a specified number of discs using velocity-based timing
     * @param mode MODE_DYNAMIC or MODE_FIXED
     * @param numShots Number of discs to shoot
     */
    private void shootSequence(int mode, int numShots) throws InterruptedException {
        // Determine shooter parameters based on mode
        double shooterVelocity = (mode == MODE_DYNAMIC) ? SHOOTER_VELOCITY_DYNAMIC : SHOOTER_VELOCITY_FIXED;
        double angleRaised = (mode == MODE_DYNAMIC) ? angleRaisedDynamic : angleRaisedFixed;

        // Align turret based on mode (RED ALLIANCE)
        alignTurret(mode);

        // Spin up shooter and raise angle
        shooter.setVelocity(shooterVelocity);
        shooterAngle.setPosition(angleRaised);

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Status", "Spinning up shooter");
        telemetry.addData("Target Velocity", shooterVelocity);
        telemetry.addData("Mode", mode == MODE_DYNAMIC ? "DYNAMIC" : "FIXED");
        telemetry.update();

        // Shoot each disc
        for (int i = 0; i < numShots; i++) {
            if (!opModeIsActive()) break;

            telemetry.addData("Alliance", "RED");
            telemetry.addData("Status", String.format("Shooting disc %d/%d", i + 1, numShots));
            telemetry.update();

            shootSingleDisc(shooterVelocity);
        }

        // Stop shooter
        shooter.setVelocity(0);
        shooterServo.setPosition(servoHome);
        shooterAngle.setPosition(angleHome);

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Status", "Shooting complete");
        telemetry.update();
    }

    /**
     * Shoots a single disc using the same velocity-based logic as teleop
     */
    private void shootSingleDisc(double targetVelocity) throws InterruptedException {
        boolean velocityWasInRange = false;
        long velocityInRangeStartTime = 0;

        // Wait for velocity to stabilize
        while (opModeIsActive()) {
            double currentVelocity = Math.abs(shooter.getVelocity());
            boolean velocityReady = currentVelocity >= (targetVelocity - VELOCITY_TOLERANCE_LOWER)
                    && currentVelocity <= (targetVelocity + VELOCITY_TOLERANCE_UPPER);

            long currentTime = System.currentTimeMillis();

            if (velocityReady) {
                if (!velocityWasInRange) {
                    velocityInRangeStartTime = currentTime;
                    velocityWasInRange = true;
                }

                long timeInRange = currentTime - velocityInRangeStartTime;

                telemetry.addData("Velocity", currentVelocity);
                telemetry.addData("Target", targetVelocity);
                telemetry.addData("Time in range", timeInRange);
                telemetry.update();

                // Velocity stable long enough, shoot
                if (timeInRange >= VELOCITY_STABLE_TIME_REQUIRED) {
                    break;
                }
            } else {
                velocityWasInRange = false;
                telemetry.addData("Velocity", currentVelocity);
                telemetry.addData("Target", targetVelocity);
                telemetry.addData("Status", "Waiting for velocity");
                telemetry.update();
            }

            sleep(10);
        }

        // Execute shot
        shooterServo.setPosition(servoShoot);
        intakeMotor1.setPower(-1);
        intakeMotor2.setPower(-1);

        sleep(INTAKE_PULSE_DURATION);

        // Stop intake and return servo
        intakeMotor1.setPower(0.0);
        intakeMotor2.setPower(0.0);

    }

    /**
     * Aligns turret based on shooting mode (RED ALLIANCE)
     * Turret angles are MIRRORED from blue side
     */
    private void alignTurret(int mode) {
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetAngle;

        if (mode == MODE_DYNAMIC) {
            // RED SIDE: Mirror of Blue's -34° = +34°
            targetAngle = 34 - imuHeading;
        } else {
            // RED SIDE: Mirror of Blue's -68° = +68°
            targetAngle = 68 - imuHeading;
        }

        // Clamp within safety limits
        targetAngle = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, targetAngle));

        moveTurretToAngle(targetAngle, 0.8);

        // Wait for turret to reach position
        while (opModeIsActive() && turretRotator.isBusy()) {
            telemetry.addData("Alliance", "RED");
            telemetry.addData("Turret Position", turretRotator.getCurrentPosition());
            telemetry.addData("Turret Target", turretRotator.getTargetPosition());
            telemetry.addData("Target Angle", String.format("%.1f°", targetAngle));
            telemetry.update();
            sleep(10);
        }
    }

    private void moveTurretToAngle(double angleDegrees, double power) {
        int targetTicks = (int) Math.round(angleDegrees * TICKS_PER_DEGREE);
        turretRotator.setTargetPosition(targetTicks);
        turretRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretRotator.setPower(Math.abs(power));
    }

    /**
     * Returns turret to starting position (0 degrees)
     */
    private void returnTurretToHome() {
        telemetry.addData("Alliance", "RED");
        telemetry.addData("Status", "Returning turret to home position");
        telemetry.update();

        moveTurretToAngle(0, 0.8);

        // Wait for turret to reach home position
        while (opModeIsActive() && turretRotator.isBusy()) {
            telemetry.addData("Alliance", "RED");
            telemetry.addData("Turret Position", turretRotator.getCurrentPosition());
            telemetry.addData("Turret Target", "0 (Home)");
            telemetry.update();
            sleep(10);
        }

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Status", "Turret at home position");
        telemetry.update();
    }

    /**
     * Drive robot using field-centric control for specified time
     * Field-centric automatically handles orientation differences
     * @param y Forward/backward (-1 to 1)
     * @param x Strafe left/right (-1 to 1)
     * @param rx Rotation (-1 to 1)
     * @param milliseconds Duration
     */
    private void driveForTime(double y, double x, double rx, long milliseconds) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        sleep(milliseconds);

        stopDrive();
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void stopAllMotors() {
        stopDrive();
        intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);
        shooter.setVelocity(0);
        turretRotator.setPower(0);
    }

    /**
     * Actively brakes the shooter motor to stop it quickly
     */
    private void brakeShooter() {
        // Set shooter to BRAKE mode for active braking
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setVelocity(0);
        shooter.setPower(0);

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Status", "Shooter braked");
        telemetry.update();
    }
}