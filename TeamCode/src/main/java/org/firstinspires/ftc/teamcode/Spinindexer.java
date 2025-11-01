package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private final double kp = 0;
    private final double ki = 0;
    private final double kd = 0;
    private double lastError = 0;
    private double integralSum = 0;
    private double nudgyPosition = 0;
    private final double greenHue = 0;
    private final double purpleHue = 0;
    private final double colorRange = 0;
    private double greenAngle = 0;
    private double purpleAngle = 0;

    public void init(HardwareMap hwMap) {
        spinner = hwMap.get(DcMotor.class, "spinner");
        nudger = hwMap.get(Servo.class, "nudger");
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    public void nudging(boolean nudgeUp, boolean nudgeDown) {
        if (nudgeUp) {
            nudgyPosition = 1;
        }
        if (nudgeDown) {
            nudgyPosition = 0;
            sinceMadeDown.reset();
        }
        nudger.setPosition(nudgyPosition);
    }
    public boolean isItDown() {
        if (sinceMadeDown.milliseconds() > 2000 && nudgyPosition == 0) {
            return true;
        } else {
            return false;
        }
    }
    public boolean withinRange (double targetThing) {
        if (Math.abs(360 * spinner.getCurrentPosition()/145.6 - targetThing) <= 5) {
            return true;
        } else {
            return false;
        }
    }
    public void PID(double targetAngle) {
        currentTicks = spinner.getCurrentPosition();
        currentAngle = 360 * currentTicks/145.6;
        double error = currentAngle - targetAngle;
        double derivative;
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
        spinner.setPower(power);
        PIDTimer.reset();
    }
}
