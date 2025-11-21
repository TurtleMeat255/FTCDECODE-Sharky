package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

// Java Math
import java.lang.Math;

public class FTCSwerveDrive {

    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;
    Servo frontLeftServo;
    Servo backLeftServo;
    Servo frontRightServo;
    Servo backRightServo;
    IMU imu;

    final double L = 8.0;
    final double W = 8.0;
    double maxSped = 1.0;
    double current_angle_fr = 0.0;
    double current_angle_fl = 0.0;
    double current_angle_rl = 0.0;
    double current_angle_rr = 0.0;



    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotorEx.class, "frontLeftMotor");
        frontLeftServo = hwMap.get(Servo.class, "frontLeftServo");

        frontRightMotor = hwMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightServo = hwMap.get(Servo.class, "frontRightServo");

        backRightMotor = hwMap.get(DcMotorEx.class, "backRightMotor");
        backRightServo = hwMap.get(Servo.class, "backRightServo");

        backLeftMotor = hwMap.get(DcMotorEx.class, "backLeftMotor");
        backLeftServo = hwMap.get(Servo.class, "backLeftServo");

        imu = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    public void swerveDrive(double y_cmd_field, double x_cmd_field, double turn_cmd, boolean reset) {

        /*
        y_cmd_field = the gamepad control for y-axis of bot. Up and down from the ctrl hub
        x_cmd_field = the gamepad control for x-axis of bot. Left and right from the ctrl hub / strafe
        turn_cmd = the gamepad control for turning of the bot.
         */

        // field centric
        double heading_rad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double y_cmd_robot = y_cmd_field * Math.cos(heading_rad) - x_cmd_field * Math.sin(heading_rad);
        double x_cmd_robot = y_cmd_field * Math.sin(heading_rad) + x_cmd_field * Math.cos(heading_rad);

        double y_fr = y_cmd_robot - turn_cmd * L; // The rotation of the frontRight motor.
        double x_fr = x_cmd_robot + turn_cmd * W; // The horizontal rotation of the frontRight motor

        double y_fl = y_cmd_robot - turn_cmd * L; // The rotation of the frontLeft motor
        double x_fl = x_cmd_robot - turn_cmd * W; // The horizontal rotation of the frontLeft motor

        double y_rl = y_cmd_robot + turn_cmd * L; // The rotation of the backLeft Motor
        double x_rl = x_cmd_robot - turn_cmd * W; // The horizontal rotation of the backLeft motor

        double y_rr = y_cmd_robot + turn_cmd * L; // The rotation of the backRight motor
        double x_rr = x_cmd_robot + turn_cmd * W; // The horizontal rotation of the backRight motor... WHY ARE YOU SICK JACOB!!


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

        maxSped = Math.max(maxSped, Math.abs(speed_fr));
        maxSped = Math.max(maxSped, Math.abs(speed_fl));
        maxSped = Math.max(maxSped, Math.abs(speed_rl));
        maxSped = Math.max(maxSped, Math.abs(speed_rr));

        if (maxSped > 1.0) {
            speed_fr /= maxSped;
            speed_fl /= maxSped;
            speed_rl /= maxSped;
            speed_rr /= maxSped;
        }

        double angle_error_fr = angle_fr_deg - current_angle_fr;
        double angle_error_fl = angle_fl_deg - current_angle_fl;
        double angle_error_rl = angle_rl_deg - current_angle_rl;
        double angle_error_rr = angle_rr_deg - current_angle_rr;



        // Just some more METH!
        angle_error_fr = normalizeAngle(angle_error_fr);
        angle_error_fl = normalizeAngle(angle_error_fl);
        angle_error_rl = normalizeAngle(angle_error_rl);
        angle_error_rr = normalizeAngle(angle_error_rr);


        // Check for the 180-degree flip (Optimization)
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

        // (Map -180 to 180 to servo range)
        double servo_pos_fr = mapAngleToServo(angle_fr_deg);
        double servo_pos_fl = mapAngleToServo(angle_fl_deg);
        double servo_pos_rl = mapAngleToServo(angle_rl_deg);
        double servo_pos_rr = mapAngleToServo(angle_rr_deg);

        // Update the stored angle and set hardware
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

        if (reset) {
            imu.resetYaw();
        }

    }

    private double normalizeAngle(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    private double mapAngleToServo(double angle) {

        final double Servo_Range_Degrees = 300.0;

        double positive_angle = (angle + 360) % 360;

        double servo_position = 0.5 + (angle / 300.0);

        if (servo_position < 0.0) {
            servo_position = 0.0;
        } else if (servo_position > 1.0) {
            servo_position = 1.0;
        }

        return servo_position;
    }
}