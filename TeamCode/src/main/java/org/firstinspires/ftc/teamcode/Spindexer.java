package org.firstinspires.ftc.teamcode;

// In your RobotHardware.java or equivalent
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

public class Spindexer {
    public DcMotor spindexerMotor = null;

    // Color search variables
    public ColorSensor colorSensor = null;

    public Servo popupServo =  null;

    private static final double SERVO_INCREMENT = 0.025; // How much to move the servo on each press
    private static final double SERVO_MAX_POSITION = 0.25; // The highest the servo can go
    private static final double SERVO_MIN_POSITION = 0.0; // The lowest the servo can go.
    private double currentServoPosition;

    public boolean servoUp = false;

    /* State Machine for Spindexer control
     * State machine to manage different behaviors */

    private enum SpindexerState {
        Holding_Position, // Using PID to hod a specific encoder position
        Searching_For_Color, // Rotating while looking for color
        Manual_Control, // Being controlled by raw power input
        Moving_To_Position, // Moving to a specific position
        LeavingEntering_Intake, // Moving to intake
        EnteringOrLeaving,
        Move3


    }

    private SpindexerState currentState = SpindexerState.Holding_Position;

    // PID Control Variables
    private final ElapsedTime PIDTimer = new ElapsedTime();
    /* Things to do with Jacob
     * Tune PID coefficients for robot efficiency */
    public double kp = 0.1;
    public double ki = 0.0;
    public double kd = 0.0;

    private double integralSum = 0;
    private double lastError = 0;
    private int targetPosition = 0; // Encoder tick we want motor to hold.
    private double preciseTargetPosition = 0.0;

    // Color search variables
    private double searchPower = 0.3;
    /* To do with Jacob
     * Tune these HSV values for printing sensor output to telemetry
     * These are example values*/
    private static final float PURPLE_HUE = 270; // Hue range for purple
    private static final float GREEN_HUE = 120; // Hue range for green
    private float hueRange = 25; // Random number for hue range from google

    // Variable to hold the hue we are currently looking for.
    private float targetHue = PURPLE_HUE;

    private float encoderResolution = 145.6f;

    public boolean isIntake = true;

    double power = 0;

    /*
     * Initialize all spindexer hardware to set motor modes.*/

    public void init(HardwareMap hwMap) {
        spindexerMotor = hwMap.get(DcMotor.class, "spindexerMotor");
        spindexerMotor.setDirection(DcMotor.Direction.FORWARD); //Idk if forward works or not. Can reverse if needed
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // To use our own encoder
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Calculate power ourselves
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        popupServo = hwMap.get(Servo.class, "popupServo");
        currentServoPosition = SERVO_MIN_POSITION;
        popupServo.setPosition(currentServoPosition);

        // Initialize color sensor
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        // Sart in a holding position 0
        currentState = SpindexerState.Holding_Position;
        targetPosition = 0; // We can fix this I think.
        preciseTargetPosition = 0.0;
    }

    public void update() {
        switch (currentState) {


            case LeavingEntering_Intake:
                preciseTargetPosition += encoderResolution / 6.0;
                targetPosition = (int) Math.round(preciseTargetPosition);

                resetAndGo();

                currentState = SpindexerState.EnteringOrLeaving;
                break;

            case Move3:
                preciseTargetPosition += encoderResolution / 3.0;
                targetPosition = (int) Math.round(preciseTargetPosition);
                resetAndGo();

                currentState = SpindexerState.EnteringOrLeaving;
                break;


            case Searching_For_Color:
                preciseTargetPosition += encoderResolution / 3.0;
                // target position = currentposition + encoderresolution/3
                // if Math.abs(targetPosition - curentPostiion) < 3 then check color
                // if color correct, activate transfer
                // otherwise move one third again
                targetPosition = (int) Math.round(preciseTargetPosition);
                resetAndGo();

                currentState = SpindexerState.Moving_To_Position;
                break;


            case Moving_To_Position:
                power = runPID();
                spindexerMotor.setPower(power);

                if (Math.abs(targetPosition - spindexerMotor.getCurrentPosition()) < 10) {
                    // Check if the sensor sees the right color
                    if (isTargetColorDetected()) {
                        // If color is detected then hold
                        holdPosition(targetPosition);

                    } else {
                        currentState = SpindexerState.Searching_For_Color;
                    }
                }
                break;

            case EnteringOrLeaving:
                power = runPID();
                spindexerMotor.setPower(power);

                if (Math.abs(targetPosition - spindexerMotor.getCurrentPosition()) < 10)
                {
                    holdPosition(targetPosition);
                }
                break;

            case Holding_Position:
                // Run one iteration of the PID controller to have target position
                double powerToApply = runPID();

                spindexerMotor.setPower(powerToApply);
                break;

            case Manual_Control:
                // In this state the motor won't do anything until a command is given.
                // The power is set directly by the setPower() method.
                break;
        }
    }

    private void resetAndGo() {
        integralSum = 0;
        lastError = 0;
        PIDTimer.reset();
    }


    /* Puts the spindexer into Searching_For_Color mode.
     * Call this when you press the button to find a specific ball.
     * @param power the speed to spin while searching. Can be negative to search backwards. */

    public void searchForPurpleBall(double power) {
        this.targetHue = PURPLE_HUE;
        this.searchPower = power;
        this.currentState = SpindexerState.Searching_For_Color;
    }

    public void searchForGreenBall(double power) {
        this.targetHue = GREEN_HUE;
        this.searchPower = power;
        this.currentState = SpindexerState.Searching_For_Color;
    }

    /* Sets a new target position and engages the PID controller to hold it.*/
    private void holdPosition(int position) {
        targetPosition = position;
        preciseTargetPosition = position;
        // Reset PID variables for a clean start at a new target
        integralSum = 0;
        lastError = 0;
        PIDTimer.reset();
        currentState = SpindexerState.Holding_Position;
    }

    private double runPID() {
        int currentPosition = spindexerMotor.getCurrentPosition();
        double error = targetPosition - currentPosition;

        double dt = PIDTimer.seconds();   // ✅ store delta time once
        PIDTimer.reset();                 // ✅ reset for next iteration

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        integralSum += error * dt;

        lastError = error;

        double power = (kp * error) + (ki * integralSum) + (kd * derivative);

        power = Math.max(-1, Math.min(1, power));
        return power;
    }


    /* Checks if the color sensor currently sees the desired color.
     * Return true if the color is within the target hue range.*/
    private boolean isTargetColorDetected() {
        // Convert RGB sensor values to HSV color model.
        // Hue is a good reliable metric for color identification under varying light according to google.


        float[] hsvValues = {0F, 0F, 0F};
        android.graphics.Color.RGBToHSV(
                (int) (colorSensor.red()),
                (int) (colorSensor.green()),
                (int) (colorSensor.blue()),
                hsvValues
        );

        float hue = hsvValues[0]; // Hue is the first value, from 0 to 360.

        // Check if the detected hue is within our range.
        return (Math.abs(hue - targetHue) < hueRange);
    }

    /* Allows for direct manual control of the spindexer, bypassing PID and color search
     * @Param power to apply (-1.0 to 1.0) */
    public void setManualPower(double power) {
        currentState = SpindexerState.Manual_Control;
        spindexerMotor.setPower(power);
    }

    // Add manual SERVO control.

    public void nudgeServoUp() {
        // Add the increment to the current position
        servoUp = true;
        currentServoPosition += SERVO_INCREMENT;

        // Math.min to make sure the position never goes above the max limit
        currentServoPosition = Math.min(currentServoPosition, SERVO_MAX_POSITION);

        popupServo.setPosition(currentServoPosition);
    }

    public void nudgeServoDown() {
        // Add the increment to the current position
        currentServoPosition -= SERVO_INCREMENT;

        servoUp = false;

        // Math.min to make sure the position never goes above the max limit
        currentServoPosition = Math.max(currentServoPosition, SERVO_MIN_POSITION);

        popupServo.setPosition(currentServoPosition);
    }

    public double getServoPosition() {
        return this.currentServoPosition;
    }

    public void moveOneSixth() {
        this.currentState = SpindexerState.LeavingEntering_Intake;
    }

    public void Move3() {
        this.currentState = SpindexerState.Move3;
    }

    public String getCurrentState() {
        return currentState.toString();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public float[] getHsvValues() {
        float[] hsvValues = {0F, 0F, 0F};
        android.graphics.Color.RGBToHSV(
                (int) (colorSensor.red()),
                (int) (colorSensor.green()),
                (int) (colorSensor.blue()),
                hsvValues
        );
        return hsvValues;
    }
}


