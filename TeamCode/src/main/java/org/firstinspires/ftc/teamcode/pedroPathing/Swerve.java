package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.FTCSwerveDrive;

import java.util.List;

public class Swerve extends Drivetrain
{
    public SwerveConstants constants;
    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;
    CRServo frontLeftServo;
    CRServo backLeftServo;
    CRServo frontRightServo;
    CRServo backRightServo;
    SparkFunOTOS otos;
    AnalogInput frontLeftAnalog;
    AnalogInput backLeftAnalog;
    AnalogInput frontRightAnalog;
    AnalogInput backRightAnalog;

    //  PID Controller Instances
    FTCSwerveDrive.PIDController frPID;
    FTCSwerveDrive.PIDController flPID;
    FTCSwerveDrive.PIDController rlPID;
    FTCSwerveDrive.PIDController rrPID;

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {



        double[] wheelPowers = new double[4];

        double wheelPowerMax = Math.max(Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])), Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3])));

        if (wheelPowerMax > maxPowerScaling) {
            wheelPowers[0] = (wheelPowers[0] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[1] = (wheelPowers[1] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[2] = (wheelPowers[2] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[3] = (wheelPowers[3] / wheelPowerMax) * maxPowerScaling;
        }

        return wheelPowers;
    }

    @Override
    public void updateConstants() {

    }

    @Override
    public void breakFollowing() {

    }

    @Override
    public void runDrive(double[] drivePowers) {

    }

    @Override
    public void startTeleopDrive() {

    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {

    }

    @Override
    public double xVelocity() {
        return 0;
    }

    @Override
    public double yVelocity() {
        return 0;
    }

    @Override
    public void setXVelocity(double xMovement) {

    }

    @Override
    public void setYVelocity(double yMovement) {

    }

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public String debugString() {
        return "";
    }
}
