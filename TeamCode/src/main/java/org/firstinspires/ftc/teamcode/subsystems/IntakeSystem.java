package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSystem {
    private final DcMotor intakeMotor;

    public IntakeSystem(HardwareMap hwMap) {
        intakeMotor = hwMap.dcMotor.get("intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intakeIn() {
        intakeMotor.setPower(-1);
    }

    // Reduced power for shooting/force feeding
    public void intakeInReduced() {
        intakeMotor.setPower(-0.7);  // 50% power for controlled feeding
    }

    public void intakeOut() {
        intakeMotor.setPower(1);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}