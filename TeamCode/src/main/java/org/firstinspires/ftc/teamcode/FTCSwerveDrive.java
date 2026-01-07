package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class FTCSwerveDrive {

    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    Servo frontLeftServo, backLeftServo, frontRightServo, backRightServo;
    AnalogInput frontLeftAnalog, backLeftAnalog, frontRightAnalog, backRightAnalog;

//    SparkFunOTOS otos;

    final double L = 0.98;
    final double W = 1.0;

    // Servo offsets - adjust these to calibrate the "zero" position of each wheel
    double FL_SERVO_OFFSET = 0.0;
    double FR_SERVO_OFFSET = 0.0;
    double BL_SERVO_OFFSET = 0.0;
    double BR_SERVO_OFFSET = 0.0;

    double lastTargetFR = 0, lastTargetFL = 0, lastTargetRL = 0, lastTargetRR = 0;

    public void init(HardwareMap hwMap) {
        frontLeftMotor  = hwMap.get(DcMotorEx.class, "frontLeftMotor");
        frontLeftServo  = hwMap.get(Servo.class, "frontLeftServo");
        frontLeftAnalog = hwMap.get(AnalogInput.class, "frontLeftAnalog");

        frontRightMotor = hwMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightServo = hwMap.get(Servo.class, "frontRightServo");
        frontRightAnalog = hwMap.get(AnalogInput.class, "frontRightAnalog");

        backRightMotor  = hwMap.get(DcMotorEx.class, "backRightMotor");
        backRightServo  = hwMap.get(Servo.class, "backRightServo");
        backRightAnalog = hwMap.get(AnalogInput.class, "backRightAnalog");

        backLeftMotor   = hwMap.get(DcMotorEx.class, "backLeftMotor");
        backLeftServo   = hwMap.get(Servo.class, "backLeftServo");
        backLeftAnalog  = hwMap.get(AnalogInput.class, "backLeftAnalog");

//        otos = hwMap.get(SparkFunOTOS.class, "otos");
//        otos.initialize();
    }

//    private void configureOtos() {
//        otos.setLinearUnit(DistanceUnit.INCH);
//        otos.setAngularUnit(AngleUnit.DEGREES);
//        otos.calibrateImu();
//        otos.resetTracking();
//    }

    /**
     * @param y_cmd_field  Forward/Backward input (Joystick Y)
     * @param x_cmd_field  Strafing input (Joystick X)
     * @param turn_cmd     Turning input (Joystick RX)
     * @param reset        Button input to reset IMU (e.g., gamepad1.options)
     */
    public void swerveDrive(double y_cmd_field, double x_cmd_field, double turn_cmd, boolean reset) {
//        if (reset) {
//            otos.resetTracking();
//        }

//        SparkFunOTOS.Pose2D currentPose = otos.getPosition();
//        double heading_rad = Math.toRadians(currentPose.h);
        double heading_rad = Math.toRadians(0);


        double x_cmd = x_cmd_field * Math.cos(heading_rad) + y_cmd_field * Math.sin(heading_rad);
        double y_cmd = -x_cmd_field * Math.sin(heading_rad) + y_cmd_field * Math.cos(heading_rad);

        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stopDrive();
            return;
        }

        double y_fr = y_cmd - turn_cmd * L;
        double x_fr = x_cmd - turn_cmd * W;
        double y_fl = y_cmd - turn_cmd * L;
        double x_fl = x_cmd + turn_cmd * W;
        double y_rl = y_cmd + turn_cmd * L;
        double x_rl = x_cmd + turn_cmd * W;
        double y_rr = y_cmd + turn_cmd * L;
        double x_rr = x_cmd - turn_cmd * W;

        double speed_fr = Math.hypot(y_fr, x_fr);
        double speed_fl = Math.hypot(y_fr, x_fr);
        double speed_rl = Math.hypot(y_rl, x_rl);
        double speed_rr = Math.hypot(y_rr, x_rr);

        double angle_fr = Math.toDegrees(Math.atan2(x_fr, y_fr));
        double angle_fl = Math.toDegrees(Math.atan2(x_fl, y_fl));
        double angle_rl = Math.toDegrees(Math.atan2(x_rl, y_rl));
        double angle_rr = Math.toDegrees(Math.atan2(x_rr, y_rr));

        double max = Math.max(Math.max(speed_fr, speed_fl), Math.max(speed_rl, speed_rr));
        if (max > 1.0) {
            speed_fr /= max; speed_fl /= max; speed_rl /= max; speed_rr /= max;
        }

        double[] opt_fr = optimize(angle_fr, speed_fr);
        double[] opt_fl = optimize(angle_fl, speed_fl);
        double[] opt_rl = optimize(angle_rl, speed_rl);
        double[] opt_rr = optimize(angle_rr, speed_rr);

        frontRightMotor.setPower(opt_fr[1]); // We keep optimize because lowkey, you only need 180 degrees of rotation...
        frontLeftMotor.setPower(opt_fl[1]);
        backLeftMotor.setPower(opt_rl[1]);
        backRightMotor.setPower(opt_rr[1]);

        frontRightServo.setPosition(angleToServoPosition(opt_fr[0], FR_SERVO_OFFSET)); // Set the servo position and take the offset.
        frontLeftServo.setPosition(angleToServoPosition(opt_fl[0], FL_SERVO_OFFSET));
        backLeftServo.setPosition(angleToServoPosition(opt_rl[0], BL_SERVO_OFFSET));
        backRightServo.setPosition(angleToServoPosition(opt_rr[0], BR_SERVO_OFFSET));

        lastTargetFR = opt_fr[0]; // To prevent violent 0 snapping, we just set the position to the last one.
        lastTargetFL = opt_fl[0];
        lastTargetRL = opt_rl[0];
        lastTargetRR = opt_rr[0];
    }

    private void stopDrive() { // Stop the drive.
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // Hold the last target positions when stopped
        frontRightServo.setPosition(angleToServoPosition(lastTargetFR, FR_SERVO_OFFSET));
        frontLeftServo.setPosition(angleToServoPosition(lastTargetFL, FL_SERVO_OFFSET));
        backLeftServo.setPosition(angleToServoPosition(lastTargetRL, BL_SERVO_OFFSET));
        backRightServo.setPosition(angleToServoPosition(lastTargetRR, BR_SERVO_OFFSET));
    }

    // Little bit different from the last one. We still use offset, but instead clip it to 90 degrees because we only need to turn 180.
    private double angleToServoPosition(double angleDeg, double offset) {
        double adjustedAngle = angleDeg + offset;

        adjustedAngle = Range.clip(adjustedAngle, -90.0, 90.0);

        return (adjustedAngle + 90.0) / 180.0;
    }


    public double getAngle(AnalogInput sensor, double offset) {
        // The angle shall be gotten?
        double raw = (sensor.getVoltage() / 3.3) * 360.0;
        return normalizeAngle(raw - offset);
    }

    // Regular normalize angle function in case we need it
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }


    // optimizes the target angle and speed for MAXIUMUM TWEAK!!
    private double[] optimize(double target, double speed) {
        target = normalizeAngle(target);

        if (target > 90.0) {
            target = target - 180.0;
            speed *= -1.0;
        } else if (target < -90.0) {
            target = target + 180.0;
            speed *= -1.0;
        }

        return new double[]{target, speed};
    }
}

