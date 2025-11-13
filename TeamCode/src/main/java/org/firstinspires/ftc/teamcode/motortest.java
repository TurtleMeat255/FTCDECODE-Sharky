package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class motortest extends OpMode
{
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    @Override
    public void init()
    {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor"); // Port 0
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor"); // Port 1
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor"); // Port 2
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor"); // Port 3

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        if (gamepad1.a)
        {
            frontLeftMotor.setPower(1);
        }
        else
        {
            frontLeftMotor.setPower(0);
        }
        if (gamepad1.b)
        {
            frontRightMotor.setPower(1);
        }
        else
        {
            frontRightMotor.setPower(0);
        }
        if (gamepad1.x)
        {
            backRightMotor.setPower(1);
        }
        else
        {
            backRightMotor.setPower(0);
        }
        if (gamepad1.y)
        {
            backLeftMotor.setPower(1);
        }
        else
        {
            backLeftMotor.setPower(0);
        }
    }
}
