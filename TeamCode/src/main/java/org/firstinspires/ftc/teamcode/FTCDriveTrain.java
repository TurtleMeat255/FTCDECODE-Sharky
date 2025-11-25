package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FTCDriveTrain
{
    double moveSpeed = 1;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    SparkFunOTOS otos;

    double kp = 0.2;
    double kd = 0.01;
    ElapsedTime dt = new ElapsedTime();
    double lastError = 0;
    double errorCrunchConstant = 3;

    public void init(HardwareMap hwMap) {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor"); // Port 0
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor"); // Port 1
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor"); // Port 2
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor"); // Port 3


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        otos = hwMap.get(SparkFunOTOS.class, "otos");
        otos.calibrateImu();
    }

    public void SwapToBrakeMode(boolean input)
    {
        if (input)
        {
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else
        {
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void RobotCentric(double x, double y, double rx)
    {
        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower * moveSpeed);
        backLeftMotor.setPower(backLeftPower * moveSpeed);
        frontRightMotor.setPower(frontRightPower * moveSpeed);
        backRightMotor.setPower(backRightPower * moveSpeed);
    }

    public void RobotCentricAlign(double x, double y, double tx)
    {
        double rx = PID(tx);

        RobotCentric(x,y,rx);
    }

    private double PID(double tx)
    {
        double error = (tx-0)/errorCrunchConstant;

        double derivative = (error - lastError)/dt.seconds();

        double output = error * kp + derivative * kd;
        output = Range.clip(output, -1, 1);

        lastError = error;
        dt.reset();

        return output;
    }

    private double angleWrap(double rad)
    {
        while (rad > Math.PI)
        {
            rad -= 2 * Math.PI;
        }
        while (rad < -Math.PI)
        {
            rad += 2 * Math.PI;
        }
        return rad;
    }

    public void Translate(double x, double y, double rx, boolean reset)
    {
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        SparkFunOTOS.Pose2D currentPose = otos.getPosition();

        currentPose.h = angleWrap(currentPose.h * Math.PI/180);

        if (reset) {
            otos.setPosition(new SparkFunOTOS.Pose2D(currentPose.x, currentPose.y, 0));
        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(currentPose.h) - y * Math.sin(currentPose.h);
        double rotY = x * Math.sin(currentPose.h) + y * Math.cos(currentPose.h);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower * moveSpeed);
        backLeftMotor.setPower(backLeftPower * moveSpeed);
        frontRightMotor.setPower(frontRightPower * moveSpeed);
        backRightMotor.setPower(backRightPower * moveSpeed);
    }

    public void FieldCentricAlign(double targetPowerX, double targetPowerY, double tx)
    {
        double rx = PID(tx);

        FieldOrientedTranslate(targetPowerX,targetPowerY,rx,false);
    }

    // taken from Sharkans INTO THE DEEP codebase
    public void FieldOrientedTranslate(double targetPowerX, double targetPowerY, double rotation, boolean reset)
    {
        SparkFunOTOS.Pose2D currentPose = otos.getPosition();

        currentPose.h = Math.toDegrees(angleWrap(Math.toRadians(currentPose.h)));

        if (reset) {
            otos.setPosition(new SparkFunOTOS.Pose2D(currentPose.x, currentPose.y, 0));
        }

        double yaw = currentPose.h;

        double stickRotation = 0;
        if (targetPowerY > 0 && targetPowerX < 0) //quad2
        {
            stickRotation = (Math.atan2(Math.abs(targetPowerY),Math.abs(targetPowerX)) + Math.PI/2) * 180/Math.PI;
        }
        else if (targetPowerY < 0 && targetPowerX < 0) //quad3
        {
            stickRotation = (Math.atan2(Math.abs(targetPowerY),Math.abs(targetPowerX)) + Math.PI) * 180/Math.PI;
        }
        else //quad1 and quad4
        {
            stickRotation = Math.atan2(targetPowerY,targetPowerX) * 180/Math.PI;
        }

        // angle of imu yaw supplemented by the stick's rotation, determined by atan
        double theta = (360-yaw) + stickRotation;
        double power = Math.hypot(targetPowerX,targetPowerY); //get hypotenuse of x and y tgt, getting the power

        // if at max power diag, limit to magnitude of 1
        // with the normalizing code, the diag movement had a bug where max power (being magnitude sqrt(2))--
        // --would cause wheels to flip polarity
        // to counteract this the power is limited to a proper magnitude
        if (power > 1)
        {
            power = 1;
        }
        else if (power < -1)
        {
            power = -1;
        }

        //get the sin and cos of theta
        //math.pi/4 represents 45 degrees, accounting for angular offset of mechanum
        double sin = Math.sin((theta * (Math.PI/180)) - (Math.PI/4));
        double cos = Math.cos((theta * (Math.PI/180)) - (Math.PI/4));
        //max of sin and cos, used to normalize the values for maximum efficiency
        double maxSinCos = Math.max(Math.abs(sin),Math.abs(cos));

        //same sign flip is to account for the inability of atan2, it typically only works for quadrants 1 and 4
        //by flipping the polarity when x < 0, we can use atan for every quadrant
        double flPower = 0;
        double frPower = 0;
        double blPower = 0;
        double brPower = 0;

        rotation *= -1;

        flPower = power * cos/maxSinCos+rotation;
        frPower = power * sin/maxSinCos-rotation;
        blPower = power * sin/maxSinCos+rotation;
        brPower = power * cos/maxSinCos-rotation;

//        double frontMax = Math.max(Math.abs(flPower),Math.abs(frPower));
//        double backMax = Math.max(Math.abs(blPower),Math.abs(brPower));

        //another normalization
        if ((power + Math.abs(rotation)) > 1)
        {
            flPower /= power + Math.abs(rotation);
            frPower /= power - Math.abs(rotation);
            blPower /= power + Math.abs(rotation);
            brPower /= power - Math.abs(rotation);
        }

        if (frontLeftMotor != null && frontRightMotor != null && backLeftMotor != null && backRightMotor != null)
        {
            frontLeftMotor.setPower(flPower * moveSpeed);
            frontRightMotor.setPower(frPower * moveSpeed);
            backLeftMotor.setPower(blPower * moveSpeed);
            backRightMotor.setPower(brPower * moveSpeed);
        }
    }
}