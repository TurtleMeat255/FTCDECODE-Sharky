package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class JacobSpindexer
{
    /*


    JACOB'S CODE - MEANT AS A REFERENCE, NOT AS WORKING DIRECTORY CODE


     */

    // motor ref
    DcMotor spinMotor;
    ColorSensor color;


    // pid
    ElapsedTime dt = new ElapsedTime();
    double kp,ki,kd = 0;
    double currentPIDOutput = 0;
    double lastError = 0;

    int huePurpleMin = 0;
    int saturationPurpleMin = 0;
    int valuePurpleMin = 0;

    int huePurpleMax = 0;

    // speed config
    double spindexerSpeed = 0.1;

    double integral = 0;

    public void init(HardwareMap hwMap)
    {
        spinMotor = hwMap.get(DcMotor.class, "spinMotor");
        color = hwMap.get(ColorSensor.class, "color");
        spinMotor.setDirection(DcMotor.Direction.FORWARD);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // not using i term
    private double PID(int setpoint, int process)
    {
        double error = setpoint - process;
        double derivative = (error - lastError)/dt.milliseconds();

        integral += error * dt.milliseconds();

        dt.reset();

        lastError = error;
        return error * kp + integral * ki + derivative * kd;
    }

    public void ColorShooter(boolean isPurple)
    {
        if (isPurple)
        {
//            Color colorOutputs = GetColorOutput();
        }
    }


    public void StopSpindexer()
    {
        currentPIDOutput = PID(spinMotor.getCurrentPosition(), spinMotor.getCurrentPosition());
    }

    private void SetTarget()
    {
        spinMotor.setPower(currentPIDOutput * spindexerSpeed);
    }

    public void SetSpindexerPower(double input)
    {
        spinMotor.setPower(input);
    }

    private int[] GetColorOutput()
    {
        // check out java.awt.color
        // search: java color rgb to hsv
        Color colVal = new Color();
//        colVal.

//        return rgbVals;
        return null;
    }
}
