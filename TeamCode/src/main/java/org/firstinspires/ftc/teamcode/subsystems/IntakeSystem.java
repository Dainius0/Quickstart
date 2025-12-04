package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSystem {
    private final DcMotor intake1, intake2;

    public IntakeSystem(HardwareMap hwMap) {
        intake1 = hwMap.dcMotor.get("intakeMotor1");
        intake2 = hwMap.dcMotor.get("intakeMotor2");
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intakeIn() {
        intake1.setPower(-0.9);
        intake2.setPower(-1);
    }

    // Reduced power for force feeding
    public void intakeInReduced() {
        intake1.setPower(-1
        );  // Reduced from -0.9
        intake2.setPower(-1); // Reduced from -1.0
    }

    public void intakeInSolo() {
        intake1.setPower(-0.9);
        intake2.setPower(0); // STOP the second motor explicitly
    }

    public void intakeOut() {
        intake1.setPower(1);
        intake2.setPower(1);
    }

    public void stop() {
        intake1.setPower(0);
        intake2.setPower(0);
    }
}