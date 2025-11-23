package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FTCSwerveDrive {

    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;
    Servo frontLeftServo;
    Servo backLeftServo;
    Servo frontRightServo;
    Servo backRightServo;

    SparkFunOTOS otos;

    final double L = 8.0;
    final double W = 8.0;

    double current_angle_fr = 0.0;
    double current_angle_fl = 0.0;
    double current_angle_rl = 0.0;
    double current_angle_rr = 0.0;

    public void init(HardwareMap hwMap) {
        frontLeftMotor  = hwMap.get(DcMotorEx.class, "frontLeftMotor");
        frontLeftServo  = hwMap.get(Servo.class,     "frontLeftServo");

        frontRightMotor = hwMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightServo = hwMap.get(Servo.class,     "frontRightServo");

        backRightMotor  = hwMap.get(DcMotorEx.class, "backRightMotor");
        backRightServo  = hwMap.get(Servo.class,     "backRightServo");

        backLeftMotor   = hwMap.get(DcMotorEx.class, "backLeftMotor");
        backLeftServo   = hwMap.get(Servo.class,     "backLeftServo");

        otos = hwMap.get(SparkFunOTOS.class, "sensor_otos");

        otos.calibrateImu();
    }

    public void swerveDrive(double y_cmd_field, double x_cmd_field, double turn_cmd, boolean reset) {

        /*
        y_cmd_field = the gamepad control for y-axis of bot. Up and down from the ctrl hub
        x_cmd_field = the gamepad control for x-axis of bot. Left and right from the ctrl hub / strafe
        turn_cmd = the gamepad control for turning of the bot.
         */
        // field centric
        SparkFunOTOS.Pose2D currentPose = otos.getPosition();
        double heading_rad = currentPose.h;


        // Correct field→robot transformation
        double x_cmd_robot = x_cmd_field * Math.cos(heading_rad) - y_cmd_field * Math.sin(heading_rad);
        double y_cmd_robot = x_cmd_field * Math.sin(heading_rad) + y_cmd_field * Math.cos(heading_rad);

        double y_fr = y_cmd_robot - turn_cmd * L;
        double x_fr = x_cmd_robot + turn_cmd * W;

        double y_fl = y_cmd_robot - turn_cmd * L;
        double x_fl = x_cmd_robot - turn_cmd * W;

        double y_rl = y_cmd_robot + turn_cmd * L;
        double x_rl = x_cmd_robot - turn_cmd * W;

        double y_rr = y_cmd_robot + turn_cmd * L;
        double x_rr = x_cmd_robot + turn_cmd * W; // WHY ARE YOU SICK JACOB!! - my bad :(

        /*
        Using some simple grade 9 METH!! We can do this trust me
         */
        double speed_fr = Math.hypot(x_fr, y_fr);
        double speed_fl = Math.hypot(x_fl, y_fl);
        double speed_rl = Math.hypot(x_rl, y_rl);
        double speed_rr = Math.hypot(x_rr, y_rr);

        /*
        Code use atan2(x, y) to make 0 degrees = forward
        We use the METH.atan2 so we get the 180 OR -180 degrees for the servo. SHII
         */
        double angle_fr_deg = Math.toDegrees(Math.atan2(x_fr, y_fr));
        double angle_fl_deg = Math.toDegrees(Math.atan2(x_fl, y_fl));
        double angle_rl_deg = Math.toDegrees(Math.atan2(x_rl, y_rl));
        double angle_rr_deg = Math.toDegrees(Math.atan2(x_rr, y_rr));

        // Normalize speeds
        double max = Math.max(
                Math.max(speed_fr, speed_fl),
                Math.max(speed_rl, speed_rr)
        );

        if (max > 1.0) {
            speed_fr /= max;
            speed_fl /= max;
            speed_rl /= max;
            speed_rr /= max;
        }

        // Compute angle errors
        double angle_error_fr = normalizeAngle(angle_fr_deg - current_angle_fr);
        double angle_error_fl = normalizeAngle(angle_fl_deg - current_angle_fl);
        double angle_error_rl = normalizeAngle(angle_rl_deg - current_angle_rl);
        double angle_error_rr = normalizeAngle(angle_rr_deg - current_angle_rr);

        // *** 60-degree flip optimization (YOUR REQUEST) ***
        if (Math.abs(angle_error_fr) > 60.0) {
            angle_fr_deg += (angle_error_fr > 0) ? -180.0 : 180.0;
            speed_fr *= -1.0;
        }

        if (Math.abs(angle_error_fl) > 60.0) {
            angle_fl_deg += (angle_error_fl > 0) ? -180.0 : 180.0;
            speed_fl *= -1.0;
        }

        if (Math.abs(angle_error_rl) > 60.0) {
            angle_rl_deg += (angle_error_rl > 0) ? -180.0 : 180.0;
            speed_rl *= -1.0;
        }

        if (Math.abs(angle_error_rr) > 60.0) {
            angle_rr_deg += (angle_error_rr > 0) ? -180.0 : 180.0;
            speed_rr *= -1.0;
        }

        // Servo mapping for 0–300° rotation
        double servo_pos_fr = mapAngleToServo(angle_fr_deg);
        double servo_pos_fl = mapAngleToServo(angle_fl_deg);
        double servo_pos_rl = mapAngleToServo(angle_rl_deg);
        double servo_pos_rr = mapAngleToServo(angle_rr_deg);

        // Apply angles + speeds
        current_angle_fr = angle_fr_deg;
        frontRightServo.setPosition(servo_pos_fr);
        frontRightMotor.setPower(speed_fr);

        current_angle_fl = angle_fl_deg;
        frontLeftServo.setPosition(servo_pos_fl);
        frontLeftMotor.setPower(speed_fl);

        current_angle_rl = angle_rl_deg;
        backLeftServo.setPosition(servo_pos_rl);
        backLeftMotor.setPower(speed_rl);

        current_angle_rr = angle_rr_deg;
        backRightServo.setPosition(servo_pos_rr);
        backRightMotor.setPower(speed_rr);

        // Optional IMU reset
        if (reset) {
            otos.setPosition(new SparkFunOTOS.Pose2D(currentPose.x, currentPose.y, 0));
        }
    }

    // [-180, 180]
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    // Map 0–360 wheel angle → 0–300° servo → 0.0–1.0
    private double mapAngleToServo(double angleDeg) {
        final double SERVO_RANGE = 300.0;

        double positive = (angleDeg % 360 + 360) % 360;

        double pos = positive / SERVO_RANGE;

        if (pos < 0.0) pos = 0.0;
        if (pos > 1.0) pos = 1.0;

        return pos;
    }
}
