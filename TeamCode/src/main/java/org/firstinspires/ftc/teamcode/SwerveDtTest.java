package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SwerveDtTest extends OpMode
{
    FTCSwerveDrive drivetrain = new FTCSwerveDrive();

    @Override
    public void init()
    {
        drivetrain.init(hardwareMap);
    }

    @Override
    public void loop()
    {
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        drivetrain.swerveDrive(leftStickY, leftStickX, rightStickX, false);
    }
}
