package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class SwerveTest extends OpMode
{
//    FTCSwerveDrive swerveDt = new FTCSwerveDrive();
//
//    @Override
//    public void init()
//    {
//        swerveDt.init(hardwareMap);
//    }
//
//    @Override
//    public void loop()
//    {
//        double leftStickX = gamepad1.left_stick_x;
//        double leftStickY = gamepad1.left_stick_y;
//
//        double rightStickX = gamepad1.right_stick_x;
//        boolean reset = gamepad1.options;
//
//        swerveDt.swerveDrive(leftStickY, leftStickX, rightStickX, reset);
//    }

    final double L = 8.0;
    final double W = 8.0;

    DcMotorEx frontLeftMotor;
    AnalogInput frontLeftAnalog;
    CRServo frontLeftServo;
    PIDController flPID;

//    SparkFunOTOS otos;



    double FLkP = 0.03;
    double FLkI = 0.0;
    double FLkD = 0.0015;

    double inputAngle = 0;
    double turnSpeedDeg = 60;

    ElapsedTime dx = new ElapsedTime();


    @Override
    public void init()
    {
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontLeftServo  = hardwareMap.get(CRServo.class,     "frontLeftServo");
        frontLeftAnalog = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");

//        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
//        otos.calibrateImu();

        flPID = new PIDController(FLkP, FLkI, FLkD);
    }

    @Override
    public void loop()
    {
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;

        double rightStickX = gamepad1.right_stick_x;
        boolean reset = gamepad1.options;

//        inputAngle += dx.seconds() * rightStickX * turnSpeedDeg;

        dx.reset();

        swerveDrive(leftStickY, leftStickX, rightStickX, reset);

        telemetry.addData("servo angle: ", getAngle(frontLeftAnalog));
//        telemetry.addData("servo angle: ", getAngle(frontLeftAnalog));
//        telemetry.addData("servo angle: ", getAngle(frontLeftAnalog));
        telemetry.update();
    }

    public void swerveDrive(double y_cmd_field, double x_cmd_field, double turn_cmd, boolean reset) {

//        SparkFunOTOS.Pose2D currentPose = otos.getPosition();
//        double heading_rad = currentPose.h;
        double heading_rad = 0;

        // Field-centric conversion THIS IS NOT grad e 9 METH!
        double x_cmd_robot = x_cmd_field * Math.cos(heading_rad) + y_cmd_field * Math.sin(heading_rad);
        double y_cmd_robot = -x_cmd_field * Math.sin(heading_rad) + y_cmd_field * Math.cos(heading_rad);

        double y_fr = y_cmd_robot - turn_cmd * L;
        double x_fr = x_cmd_robot - turn_cmd * W;
        double y_fl = y_cmd_robot - turn_cmd * L;
        double x_fl = x_cmd_robot + turn_cmd * W;
        double y_rl = y_cmd_robot + turn_cmd * L;
        double x_rl = x_cmd_robot + turn_cmd * W;
        double y_rr = y_cmd_robot + turn_cmd * L;
        double x_rr = x_cmd_robot - turn_cmd * W;

        // Calculate Speed and Target Angle
        double speed_fr = Math.hypot(x_fr, y_fr);
        double speed_fl = Math.hypot(x_fl, y_fl);
        double speed_rl = Math.hypot(x_rl, y_rl);
        double speed_rr = Math.hypot(x_rr, y_rr);

        // atan2(x, y) makes 0 degrees = forward (Y-axis)
        double angle_fr_deg = Math.toDegrees(Math.atan2(x_fr, y_fr));
        double angle_fl_deg = Math.toDegrees(Math.atan2(x_fl, y_fl));
        double angle_rl_deg = Math.toDegrees(Math.atan2(x_rl, y_rl));
        double angle_rr_deg = Math.toDegrees(Math.atan2(x_rr, y_rr));

        // Normalize speeds to ensure maximum speed is 1.0
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

        //  90-Degree Optimization & PID Control

        // Get current angles from sensors
        double actual_angle_fl = getAngle(frontLeftAnalog);

        // Apply optimization to target angle and drive speed
        double[] optimized_fl = optimize(angle_fl_deg, speed_fl, actual_angle_fl);

        double target_fl = optimized_fl[0];
        speed_fl = optimized_fl[1];
        double steer_power_fl = flPID.calculate(target_fl, actual_angle_fl);


        telemetry.addData("steer power", steer_power_fl);

        telemetry.addData("target Angle", target_fl);

        frontLeftServo.setPower(steer_power_fl);
        frontLeftMotor.setPower(speed_fl);

        if (reset) {
//            otos.setPosition(new SparkFunOTOS.Pose2D(currentPose.x, currentPose.y, 0));
        }
    }

    private double getAngle(AnalogInput sensor) {
        final double VOLT_TO_DEG = 360.0 / 3.3; // Assuming 3.3V reference
        double rawAngle = sensor.getVoltage() * VOLT_TO_DEG;
        return rawAngle % 360.0;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    private double[] optimize(double targetAngle, double driveSpeed, double currentAngle) {
        double error = targetAngle - currentAngle;

        error = normalizeAngle(error);

        if (Math.abs(error) > 90.0) {
            targetAngle = targetAngle - Math.signum(error) * 180.0;

            driveSpeed *= -1.0;

            targetAngle = normalizeAngle(targetAngle);
        }

        return new double[]{targetAngle, driveSpeed};
    }

    public class PIDController {

        private double kP, kI, kD;
        private double integralSum = 0;
        private double lastError = 0;
        private ElapsedTime timer = new ElapsedTime();

        public PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            timer.reset();
        }

        public double calculate(double targetAngle, double currentAngle) {
            // Find the shortest error path [-180, 180]
            double error = targetAngle - currentAngle;
            error = normalizeAngle(error);

            error /= 10;

            telemetry.addData("error", error);

            double dt = timer.seconds();
            timer.reset();

            // P - Proportional Term
            double pTerm = kP * error;

            // I - Integral Term
            integralSum += error * dt;
            if (kI != 0) {
                integralSum = Math.max(Math.min(integralSum, 0.5 / kI), -0.5 / kI);
            }
            double iTerm = kI * integralSum;

            // D - Derivative Term
            double derivative = (error - lastError) / (dt > 0 ? dt : 0.001); // Avoid division by zero
            double dTerm = kD * derivative;

            lastError = error;

            double output = pTerm + iTerm + dTerm;

            // Clamp output power
            return Math.max(-1.0, Math.min(1.0, output));
        }

    }
}