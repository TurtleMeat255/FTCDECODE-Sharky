package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class FullSwerveTestMode extends OpMode
{

    FTCSwerveDrive swerveDt = new FTCSwerveDrive();

    @Override
    public void init()
    {
        swerveDt.init(hardwareMap);
    }

    @Override
    public void loop()
    {
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;

        double rightStickX = gamepad1.right_stick_x;
        boolean reset = gamepad1.options;

        swerveDt.swerveDrive(leftStickY, leftStickX, rightStickX, reset);
    }
}
