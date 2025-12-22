package org.firstinspires.ftc.teamcode;

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

public class NathanDriveTrain
{
    double moveSpeed = 1.0;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    IMU imu;

    double kp = 0.2;
    double kd = 0;
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

        // Retrieve the IMU from the hardware map
        imu = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
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

    public void Translate(double x, double y, double rx, boolean reset)
    {
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (reset) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

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
}
