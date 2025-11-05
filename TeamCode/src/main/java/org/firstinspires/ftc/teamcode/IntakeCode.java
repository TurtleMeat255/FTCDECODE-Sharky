package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class IntakeCode {

    private DcMotor intake1;


    public void init(HardwareMap hwMap) {
        // Declare Intake Motor
        DcMotor intake1;

        intake1 = hwMap.get(DcMotor.class, "intake");
        // Set brake mode when power = 0
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void ActivateIntake(boolean intake1) {
        if (intake1) {
            this.intake1.setPower(1);
        }
        else {
            this.intake1.setPower(0);
        }
    }

}

