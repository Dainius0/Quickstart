package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretSystem {
    private final DcMotor turret;
    private static final int TICKS_PER_ROTATION = 1879;
    private static final double TICKS_PER_DEGREE = (double) TICKS_PER_ROTATION / 360.0;

    private static final double MIN_TURRET_ANGLE = -180.0;
    private static final double MAX_TURRET_ANGLE = 180.0;

    public TurretSystem(HardwareMap hwMap) {
        turret = hwMap.dcMotor.get("turretRotator");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0); // Set target BEFORE switching mode
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Now switch to RUN_TO_POSITION
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveToAngle(double angleDegrees, double power) {
        angleDegrees = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, angleDegrees));
        int targetTicks = (int) Math.round(angleDegrees * TICKS_PER_DEGREE);
        turret.setTargetPosition(targetTicks);
        turret.setPower(Math.abs(power));
    }
}