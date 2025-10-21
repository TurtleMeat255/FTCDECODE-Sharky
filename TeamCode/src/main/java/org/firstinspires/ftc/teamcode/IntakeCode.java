package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IntakeCode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare Intake Motor
        DcMotor intake1 = hardwareMap.get(DcMotor.class, "intake");

        // Set brake mode when power = 0
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.a) {
                // Run intake forward
                intake1.setPower(1.0);
            } else if (gamepad2.b) {
                // Run intake backward
                intake1.setPower(-1.0);
            } else {
                // Stop intake
                intake1.setPower(0.0);
            }
        }
    }
}

