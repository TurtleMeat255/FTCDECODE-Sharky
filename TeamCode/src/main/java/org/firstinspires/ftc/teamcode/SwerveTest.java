package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class SwerveTest extends OpMode {
    FTCSwerveDrive swerveDt = new FTCSwerveDrive();

//    @Override
//    public void init() {
//        swerveDt.init(hardwareMap);
//    }
//
//    @Override
//    public void loop() {
//        double leftStickX = gamepad1.left_stick_x;
//        double leftStickY = gamepad1.left_stick_y;
//
//        double rightStickX = gamepad1.right_stick_x;
//        boolean reset = gamepad1.options;
//
//        swerveDt.swerveDrive(leftStickY, leftStickX, rightStickX, reset);
//    }

    final double L = 0.98;
    final double W = 1.00;

    DcMotorEx frontLeftMotor;
    AnalogInput frontLeftAnalog;
    CRServo frontLeftServo;
    PIDController flPID;
    ElapsedTime angleTimer = new ElapsedTime();

//    SparkFunOTOS otos;



    double FLkP = 0.005;
    double FLkI = 0.0;
    double FLkD = 0.0;
    double FLkF = 0.0;


    double FL_OFFSET = 10.0;
    double FR_OFFSET = 0.0;
    double BL_OFFSET = 0.0;
    double BR_OFFSET = 0.0;


    final double GEARBOX_RATIO = 1/(36.0f / 24.0f);

    double minServoPower = 0.03;


    double lastTargetFR = 0, lastTargetFL = 0, lastTargetRL = 0, lastTargetRR = 0;


    ElapsedTime dx = new ElapsedTime();


    @Override
    public void init()
    {
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontLeftServo  = hardwareMap.get(CRServo.class,     "frontLeftServo");
        frontLeftAnalog = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");

//        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
//        otos.calibrateImu();

        flPID = new PIDController(FLkP, FLkI, FLkD, FLkF);
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

        telemetry.addData("servo angle: ", getAngle(frontLeftAnalog, FL_OFFSET));
//        telemetry.addData("servo angle: ", getAngle(frontLeftAnalog));
//        telemetry.addData("servo angle: ", getAngle(frontLeftAnalog));
        telemetry.update();
    }

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

//        double current_fr = getAngle(frontRightAnalog, FR_OFFSET);
        double current_fl = getAngle(frontLeftAnalog, FL_OFFSET);
//        double current_rl = getAngle(backLeftAnalog, BL_OFFS  ET);
//        double current_rr = getAngle(backRightAnalog, BR_OFFSET);

//        double[] opt_fr = optimize(angle_fr, speed_fr, current_fr);
        double[] opt_fl = optimize(angle_fl, speed_fl, current_fl);
//        double[] opt_rl = optimize(angle_rl, speed_rl, current_rl);
//        double[] opt_rr = optimize(angle_rr, speed_rr, current_rr);

//        frontRightMotor.setPower(opt_fr[1]);
        frontLeftMotor.setPower(opt_fl[1]);
//        backLeftMotor.setPower(opt_rl[1]);
//        backRightMotor.setPower(opt_rr[1]);

//        lastTargetFR = opt_fr[0];
        lastTargetFL = opt_fl[0];
//        lastTargetRL = opt_rl[0];
//        lastTargetRR = opt_rr[0];

        runPID(opt_fl[0]);
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        backRightMotor.setPower(0);

        runPID(lastTargetFL);
    }

    boolean flUnwinding = false;
    boolean frUnwinding = false;
    boolean rlUnwinding = false;
    boolean rrUnwinding = false;

    private void runPID(double tFL) {
        double dt = angleTimer.seconds();
        angleTimer.reset();
        if (dt < 0.001) dt = 0.001;

        // Convert Wheel Target to Servo Target (Control Space)
        double servoTargetFL = tFL * GEARBOX_RATIO;
        // double servoTargetFR = tFR * GEARBOX_RATIO;
        // double servoTargetRL = tRL * GEARBOX_RATIO;
        // double servoTargetRR = tRR * GEARBOX_RATIO;

        // Read raw sensor angle (Servo Space) for PID feedback
        double currentServoAngleFL = getRawServoAngle(frontLeftAnalog, FL_OFFSET * GEARBOX_RATIO);
        // double currentServoAngleFR = getRawServoAngle(frontRightAnalog, FR_OFFSET * GEARBOX_RATIO);
        // double currentServoAngleRL = getRawServoAngle(backLeftAnalog, BL_OFFSET * GEARBOX_RATIO);
        // double currentServoAngleRR = getRawServoAngle(backRightAnalog, BR_OFFSET * GEARBOX_RATIO);

        double velFL = normalizeAngle(servoTargetFL - (lastTargetFL * GEARBOX_RATIO)) / dt;
        // double velFR = normalizeAngle(servoTargetFR - (lastTargetFR * GEARBOX_RATIO)) / dt;
        // double velRL = normalizeAngle(servoTargetRL - (lastTargetRL * GEARBOX_RATIO)) / dt;
        // double velRR = normalizeAngle(servoTargetRR - (lastTargetRR * GEARBOX_RATIO)) / dt;

        if (getRawAngle(frontLeftAnalog, FL_OFFSET) > 360)
        {
            flUnwinding = true;
        }
        if (getRawAngle(frontRightAnalog, FR_OFFSET) > 360)
        {
            frUnwinding = true;
        }
        if (getRawAngle(backLeftAnalog, BL_OFFSET) > 360)
        {
            rlUnwinding = true;
        }
        if (getRawAngle(backRightAnalog, BR_OFFSET) > 360)
        {
            rrUnwinding = true;
        }

        if (flUnwinding)
        {
            servoTargetFL = 0;

            if (Math.abs(getRawAngle(frontLeftAnalog, FL_OFFSET)) < 360))
        }
        if (frUnwinding)
        {
            servoTargetFR = 0;
        }
        if (rlUnwinding)
        {
            servoTargetRL = 0;
        }
        if (rrUnwinding)
        {
            servoTargetRR = 0;
        }

        double powerFL = flPID.calculate(servoTargetFL, currentServoAngleFL, velFL);
        // double powerFR = flPID.calculate(servoTargetFR, currentServoAngleFR, velFR);
        // double powerRL = flPID.calculate(servoTargetRL, currentServoAngleRL, velRL);
        // double powerRR = flPID.calculate(servoTargetRR, currentServoAngleRR, velRR);

        frontLeftServo.setPower(powerFL);
        // frontRightServo.setPower(-powerFR);
        // backLeftServo.setPower(-powerRL);
        // backRightServo.setPower(-powerRR);


        lastTargetFL = tFL;
//        lastTargetFR = tFR;
//        lastTargetRL = tRL;
//        lastTargetRR = tRR;
    }

    //  (Servo Space) GUH
    private double getRawServoAngle(AnalogInput sensor, double offset) {
        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;
        return normalizeAngle(rawAngle - offset);
    }

    // (Wheel Space) BUH
    private double getAngle(AnalogInput sensor, double offset) {
        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;

        // Apply offset in servo space (before gearbox conversion)
        double wheelAngle = (rawAngle - offset) / GEARBOX_RATIO;

        return normalizeAngle(wheelAngle);
    }

    private double getRawAngle(AnalogInput sensor, double offset) {
        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;

        // Apply offset in servo space (before gearbox conversion)
        return (rawAngle - offset) / GEARBOX_RATIO;
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

    public class PIDController {
        private double kP, kI, kD, kF;
        private double lastError = 0;
        private ElapsedTime timer = new ElapsedTime();

        public PIDController(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            timer.reset();
        }

        public double calculate(double target, double current, double targetVel) {
            double error = normalizeAngle(target - current);
            double dt = timer.seconds();
            timer.reset();

            if (dt < 0.001) dt = 0.001;

            double pTerm = error * kP;
            double derivative = (error - lastError) / dt;
            double dTerm = derivative * kD;

            // PLEASE SPEED I NEED THIS!!!
            // MY FEED IS KINDA FORWARDLESS!!!
            double fTerm = targetVel * kF;

            lastError = error;

            double output = pTerm + dTerm + fTerm;

            if (Math.abs(error) < 15.00) return 0;

            output += Math.signum(output) * minServoPower;
            return Range.clip(output, -1.0, 1.0);
        }
    }
}