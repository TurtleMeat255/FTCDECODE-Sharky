package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeCode {

    // The single DcMotor variable for this class.
    private DcMotor intake1;


    public void init(HardwareMap hwMap) {
        // ✅ FIX 1: Assign to the class variable, don't declare a new one.
        // The "this." prefix makes it clear you're using the class-level variable.
        this.intake1 = hwMap.get(DcMotor.class, "intake");

        // Set the motor direction. You might need to change this to REVERSE.
        this.intake1.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set brake mode, so the motor actively stops instead of coasting.
        this.intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run without encoders since we are just setting power.
        this.intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void ActivateIntake(boolean shouldRun, boolean shouldReverse) { // ✅ FIX 2: Use a clear parameter name.
        // First, check if init() was called to prevent a crash.
        if (this.intake1 == null) {
            return;
        }

        if (shouldRun) {
            // Run the intake at full power.
            this.intake1.setPower(1);
        } else if (shouldReverse){
            // Stop the intake.
            this.intake1.setPower(-1);
        } else {
            this.intake1.setPower(0);
        }
    }


    public void reverse() {
        if (this.intake1 != null) {
            this.intake1.setPower(-1.0);
        }
    }
}


