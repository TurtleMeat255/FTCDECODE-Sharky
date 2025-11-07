package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeCode {

    // The single DcMotor variable for this class.
    private DcMotor intake1;
    double intakeSpeed = 0.6;


    public void init(HardwareMap hwMap) {
        // ✅ FIX 1: Assign to the class variable, don't declare a new one.
        // The "this." prefix makes it clear you're using the class-level variable.
        intake1 = hwMap.get(DcMotor.class, "intake");

        // Set the motor direction. You might need to change this to REVERSE.
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);

        // Run without encoders since we are just setting power.
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void ActivateIntake(boolean shouldRun, boolean shouldReverse)
    {
        // First, check if init() was called to prevent a crash.
        if (intake1 == null) {
            return;
        }

        if (shouldRun)
        {
            intake1.setPower(intakeSpeed);
        }
        else if (shouldReverse)
        {
            intake1.setPower(-intakeSpeed);
        } else {
            intake1.setPower(0);
        }
    }
}