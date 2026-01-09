package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    double FL_OFFSET = 0.0; // Test these offsets LMAO
    double FR_OFFSET = 0.0;
    double BL_OFFSET = 0.0;
    double BR_OFFSET = 0.0;

    final double GEARBOX_RATIO = 36.0 / 24.0;


    double lastTargetFR = 0, lastTargetFL = 0, lastTargetRL = 0, lastTargetRR = 0;

    public void init(HardwareMap hwMap) {
        frontLeftMotor  = hwMap.get(DcMotorEx.class, "frontLeftMotor");
        frontLeftServo  = hwMap.get(Servo.class,       "frontLeftServo");
        frontLeftAnalog = hwMap.get(AnalogInput.class, "frontLeftAnalog");

        frontRightMotor = hwMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightServo = hwMap.get(Servo.class,       "frontRightServo");
        frontRightAnalog = hwMap.get(AnalogInput.class, "frontRightAnalog");

        backRightMotor  = hwMap.get(DcMotorEx.class, "backRightMotor");
        backRightServo  = hwMap.get(Servo.class,       "backRightServo");
        backRightAnalog = hwMap.get(AnalogInput.class, "backRightAnalog");

        backLeftMotor   = hwMap.get(DcMotorEx.class, "backLeftMotor");
        backLeftServo   = hwMap.get(Servo.class,       "backLeftServo");
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

        double x_fr = x_cmd + turn_cmd * L;
        double y_fr = y_cmd - turn_cmd * W;
        double x_fl = x_cmd + turn_cmd * L;
        double y_fl = y_cmd + turn_cmd * W;
        double x_rl = x_cmd - turn_cmd * L;
        double y_rl = y_cmd + turn_cmd * W;
        double x_rr = x_cmd - turn_cmd * L;
        double y_rr = y_cmd - turn_cmd * W;

        double speed_fr = Math.hypot(x_fr, y_fr);
        double speed_fl = Math.hypot(x_fl, y_fl);
        double speed_rl = Math.hypot(x_rl, y_rl);
        double speed_rr = Math.hypot(x_rr, y_rr);

        double angle_fr = Math.toDegrees(Math.atan2(x_fr, y_fr));
        double angle_fl = Math.toDegrees(Math.atan2(x_fl, y_fl));
        double angle_rl = Math.toDegrees(Math.atan2(x_rl, y_rl));
        double angle_rr = Math.toDegrees(Math.atan2(x_rr, y_rr));

        double max = Math.max(Math.max(speed_fr, speed_fl), Math.max(speed_rl, speed_rr));
        if (max > 1.0) {
            speed_fr /= max; speed_fl /= max; speed_rl /= max; speed_rr /= max;
        }

        double current_fr = getAngle(frontRightAnalog, FR_OFFSET);
        double current_fl = getAngle(frontLeftAnalog, FL_OFFSET);
        double current_rl = getAngle(backLeftAnalog, BL_OFFSET);
        double current_rr = getAngle(backRightAnalog, BR_OFFSET);

        double[] opt_fr = optimize(angle_fr, speed_fr, current_fr);
        double[] opt_fl = optimize(angle_fl, speed_fl, current_fl);
        double[] opt_rl = optimize(angle_rl, speed_rl, current_rl);
        double[] opt_rr = optimize(angle_rr, speed_rr, current_rr);

        frontRightMotor.setPower(opt_fr[1]);
        frontLeftMotor.setPower(opt_fl[1]);
        backLeftMotor.setPower(opt_rl[1]);
        backRightMotor.setPower(opt_rr[1]);

        lastTargetFR = opt_fr[0];
        lastTargetFL = opt_fl[0];
        lastTargetRL = opt_rl[0];
        lastTargetRR = opt_rr[0];

        setServoPositions(opt_fr[0], opt_fl[0], opt_rl[0], opt_rr[0]);
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        setServoPositions(lastTargetFR, lastTargetFL, lastTargetRL, lastTargetRR);
    }

    private void setServoPositions(double tFR, double tFL, double tRL, double tRR) {
        frontRightServo.setPosition(angleToServoPosition(tFR));
        frontLeftServo.setPosition(angleToServoPosition(tFL));
        backLeftServo.setPosition(angleToServoPosition(tRL));
        backRightServo.setPosition(angleToServoPosition(tRR));

        lastTargetFR = tFR;
        lastTargetFL = tFL;
        lastTargetRL = tRL;
        lastTargetRR = tRR;
    }

    // BALRIGHHHHHTTTTTT
    private double angleToServoPosition(double wheelAngle) {
        double servoAngle = wheelAngle * GEARBOX_RATIO;
        double position = (servoAngle + 180.0) / 360.0;
        return Math.max(0.0, Math.min(1.0, position));
    }

    private double getAngle(AnalogInput sensor, double offset) {
        double raw = (sensor.getVoltage() / 3.3) * 360.0;

        double wheelAngle = raw / GEARBOX_RATIO;

        return normalizeAngle(wheelAngle - offset);
    }

    private double getAngleRaw(AnalogInput sensor, double offset) {
        double raw = (sensor.getVoltage() / 3.3) * 360.0;

        return raw / GEARBOX_RATIO;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    private double[] optimize(double target, double speed, double current) {
        double delta = normalizeAngle(target - current);
        if (Math.abs(delta) > 90.0) {
            target = normalizeAngle(target - Math.signum(delta) * 180.0);
            speed *= -1.0;
        }
        return new double[]{target, speed};
    }
}
