package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
public class Spinindexer {
    public DcMotor spinner;
    public Servo nudger;
    public ColorSensor colorSensor;
    private final ElapsedTime PIDTimer = new ElapsedTime();
    private final ElapsedTime sinceMadeDown = new ElapsedTime();
    private int currentTicks = 0;
    private double currentAngle = 360 * currentTicks/145.6;
    private final double kp = 0.015; // 0.025
    private final double ki = 0;
    private double kd = 0; // 0.00075
    private double lastError = 0;
    private double integralSum = 0;
    private static double nudgyPosition = 0.05;
    private final double nudgerTime = 2;
    private final double greenHue = 0;
    private final double purpleHue = 0;
    private final double colorRange = 0;
    private double greenAngle = 0;
    private double purpleAngle = 0;
    double encoderResolution = 537.7;
    double spindexerSpeed = 0.5;

    public void init(HardwareMap hwMap) {
        spinner = hwMap.get(DcMotor.class, "spinner");
        nudger = hwMap.get(Servo.class, "nudger");
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int colorDetected() { // 1 = green, 2 = purple
        float[] hsvValues = {0F, 0F, 0F};
        android.graphics.Color.RGBToHSV(
                (int) (colorSensor.red() * 255),
                (int) (colorSensor.green() * 255),
                (int) (colorSensor.blue() * 255),
                hsvValues
        );
        double hue = hsvValues[0];
        greenAngle = Math.abs(hue - greenHue);
        purpleAngle = Math.abs(hue - purpleHue);
        if (greenAngle > 180) {
            greenAngle = 360 - greenAngle;
        }
        if (purpleAngle > 180) {
            purpleAngle = 360 - purpleAngle;
        }
        if (greenAngle <= colorRange) {
            return 1;
        } else if (purpleAngle <= colorRange) {
            return 2;
        } else {
            return 0;
        }
    }
    public void nudging(boolean nudgeUp) {
        if (nudgeUp) {
            nudgyPosition = 0.35;
        }
        else{
            nudgyPosition = 0.05;
            sinceMadeDown.reset();
        }
        nudger.setPosition(nudgyPosition);
    }
    public void nudge2(double nudgyPosition) {
        nudger.setPosition(nudgyPosition);
    }

    public boolean isItDown() {
        if (sinceMadeDown.milliseconds() > nudgerTime && Math.abs(0.05 - nudger.getPosition()) < 0.01) {
            return true;
        } else {
            return false;
        }
    }
    public boolean withinRange (double targetThing) {
        if (Math.abs(360 * spinner.getCurrentPosition()/encoderResolution - targetThing) <= 15) {
            return true;
        } else {
            return false;
        }
    }

    public void SpindexerPower(double power)
    {
        spinner.setPower(power);
    }

    public void PID(double targetAngle) {
        currentTicks = spinner.getCurrentPosition();
        currentAngle = 360 * currentTicks/encoderResolution;
        double error = currentAngle - targetAngle;
        double derivative;

//        if (withinRange(targetAngle))
//        {
//            spinner.setPower(0);
//            return;
//        }

        if (PIDTimer.seconds() > 0) {
            derivative = (error - lastError)/PIDTimer.seconds();
        } else {
            derivative = 0;
        }
        integralSum += error * PIDTimer.seconds();

        lastError = error;
        double power = kp * error + ki * integralSum + kd * derivative;
        if (power >= 1) {
            power = 1;
        }
        if (power <= -1) {
            power = -1;
        }
        spinner.setPower(power * -0.3);
        PIDTimer.reset();
    }
    public void runNudger(double input) {
        nudger.setPosition(input);
    }

    public double GetCurrentPosition()
    {
        return spinner.getCurrentPosition();
    }

    public double GetEncoderResolution()
    {
        return encoderResolution;
    }

    public void RapidFiring(boolean input)
    {
        if (input)
        {
            spindexerSpeed = 1;
        }
        else
        {
            spindexerSpeed = 0.5;
        }
    }

    public void BallKD(boolean input)
    {
        if (input)
        {
            kd = 0.00005;
        }
        else
        {
            kd = 0;
        }
    }

    public double GetKP()
    {
        return kp;
    }
}

