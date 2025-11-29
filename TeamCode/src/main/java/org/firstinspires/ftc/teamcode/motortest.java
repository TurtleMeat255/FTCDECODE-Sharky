package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class motortest extends OpMode
{
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo Hood1;
    Servo Hood2;

    @Override
    public void init()
    {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor"); // Port 0
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor"); // Port 1
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor"); // Port 2
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor"); // Port 3

        Hood1 = hardwareMap.get(Servo.class, "Hood1");
        Hood2 = hardwareMap.get(Servo.class, "Hood2");
        Hood2.setDirection(Servo.Direction.REVERSE);

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

        Hood1.setPosition(0);
        Hood2.setPosition(0);
    }
}
