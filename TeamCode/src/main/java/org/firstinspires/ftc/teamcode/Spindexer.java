package org.firstinspires.ftc.teamcode;

// In your RobotHardware.java or equivalent
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class Spindexer {

public DcMotorEx spindexerMotor = null;
    int ticks = 10; // <-- Fix this too
    // pid
    ElapsedTime dt = new ElapsedTime();
    double kp,kd = 0;
    double currentPIDOutput = 0;
    double lastError = 0;

    double spindexerSpeed = 0.1;


    // not using i term
    private double PID(int target, int setpoint)
    {
        double error = target - setpoint;
        double derivative = (error - lastError)/dt.milliseconds();

        dt.reset();

        lastError = error;
        double output = error * kp + derivative * kd;
        return output;
    }

    // ... other hardware ...
    public void init(HardwareMap hwMap) {

        // Initialize the motor
        spindexerMotor = hwMap.get(DcMotorEx.class, "spindexerMotor");

        // 1. Stop and reset the encoder to 0
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 2. Set the motor to run to a target position
        // It will now only move when given a target.
        spindexerMotor.setTargetPosition(ticks);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        /* Can I use an if statement to call a loop to do this?*/
        // 3. Set the motor to brake when it has zero power
        // This helps it hold its position firmly.
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setTargetPosition(67); // <-- Fix this later

    }

    private void SetTarget()
    {
        spindexerMotor.setPower(currentPIDOutput * spindexerSpeed);
    }

    public void SetSpindexerPower(double input)
    {
        spindexerMotor.setPower(input);
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


