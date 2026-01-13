//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//@TeleOp
//public class SwerveTest extends OpMode {
////    @Override
////    public void init() {
////        swerveDt.init(hardwareMap);
////    }
////
////    @Override
////    public void loop() {
////        double leftStickX = gamepad1.left_stick_x;
////        double leftStickY = gamepad1.left_stick_y;
////
////        double rightStickX = gamepad1.right_stick_x;
////        boolean reset = gamepad1.options;
////
////        swerveDt.swerveDrive(leftStickY, leftStickX, rightStickX, reset);
////    }
//
//    final double L = 0.98;
//    final double W = 1.00;
//
//    DcMotorEx frontLeftMotor;
//    AnalogInput frontLeftAnalog;
//    CRServo frontLeftServo;
//
//    DcMotorEx frontRightMotor;
//    AnalogInput frontRightAnalog;
//    CRServo frontRightServo;
//
//    DcMotorEx backLeftMotor;
//    AnalogInput backLeftAnalog;
//    CRServo backLeftServo;
//
//    DcMotorEx backRightMotor;
//    AnalogInput backRightAnalog;
//    CRServo backRightServo;
//
//    PIDController flPID;
//    PIDController frPID;
//
//    PIDController rlPID;
//    PIDController rrPID;
//
//    ElapsedTime angleTimer = new ElapsedTime();
//
////    SparkFunOTOS otos;
//
//
//
//    double FLkP = 0.002;
//    double FLkI = 0.0;
//    double FLkD = 0.0001;
//    double FLkF = 0.0;
//
//    double FRkP = 0.002;
//    double FRkI = 0.0;
//    double FRkD = 0.0001;
//    double FRkF = 0.0;
//
//    double RLkP = 0.002;
//    double RLkI = 0.0;
//    double RLkD = 0.0001;
//    double RLkF = 0.0;
//
//    double RRkP = 0.002;
//    double RRkI = 0.0;
//    double RRkD = 0.0001;
//    double RRkF = 0.0;
//
//    double inputAngle = 0;
//    double turnSpeedDeg = 60;
//
//    double FL_OFFSET = 176; // Test these offsets LMAO
//    double FR_OFFSET = 162;
//    double BL_OFFSET = 63;
//    double BR_OFFSET = 48.6;
//
//
//    final double GEARBOX_RATIO = 1 / (36.0f / 24.0f);
//
//    double minServoPower = 0.03;
//
//
//    double lastTargetFR = 0, lastTargetFL = 0, lastTargetRL = 0, lastTargetRR = 0;
//    double lastAngleFR = 0, lastAngleFL = 0, lastAngleRL = 0, lastAngleRR = 0;
//
//    double speed = 0.35;
//
//    boolean wasFlippedFL, wasFlippedFR, wasFlippedBL, wasFlippedBR = false;
//    double flSpeed, frSpeed, blSpeed, brSpeed = 0;
//
//    ElapsedTime dx = new ElapsedTime();
//
//
//    @Override
//    public void init()
//    {
//        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
//        frontLeftServo  = hardwareMap.get(CRServo.class,     "frontLeftServo");
//        frontLeftAnalog = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");
//
//        frontRightMotor  = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
//        frontRightServo  = hardwareMap.get(CRServo.class,     "frontRightServo");
//        frontRightAnalog = hardwareMap.get(AnalogInput.class, "frontRightAnalog");
//
//        backLeftMotor  = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
//        backLeftServo  = hardwareMap.get(CRServo.class,     "backLeftServo");
//        backLeftAnalog = hardwareMap.get(AnalogInput.class, "backLeftAnalog");
//
//        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
//        backRightServo  = hardwareMap.get(CRServo.class,     "backRightServo");
//        backRightAnalog = hardwareMap.get(AnalogInput.class, "backRightAnalog");
//
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//
//
////        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
////        otos.calibrateImu();
//
//        flPID = new PIDController(FLkP, FLkI, FLkD, FLkF);
//        frPID = new PIDController(FRkP, FRkI, FRkD, FRkF);
//        rlPID = new PIDController(RLkP, RLkI, RLkD, RLkF);
//        rrPID = new PIDController(RRkP, RRkI, RRkD, RRkF);
//    }
//
//    @Override
//    public void loop()
//    {
//        double leftStickX = -gamepad1.left_stick_x;
//        double leftStickY = gamepad1.left_stick_y;
//
//        double rightStickX = gamepad1.right_stick_x;
//        boolean reset = gamepad1.options;
//
////        inputAngle += dx.seconds() * rightStickX * turnSpeedDeg;
//
//        dx.reset();
//
//        swerveDrive(leftStickY, leftStickX, rightStickX, reset);
//
////        telemetry.addData("servo angleL: ", getAngle(backLeftAnalog, BL_OFFSET));
////        telemetry.addData("servo angleR: ", getAngle(backRightAnalog, BR_OFFSET));
////        telemetry.addData("servo angle: ", getAngle(frontLeftAnalog));
////        telemetry.addData("servo angle: ", getAngle(frontLeftAnalog));
//
//        telemetry.addData("servo FL:", filter(lastAngleFL, getAngle(frontLeftAnalog, FL_OFFSET)));
//        telemetry.addData("servo FR :", filter(lastAngleFR, getAngle(frontRightAnalog, FR_OFFSET)));
//        telemetry.addData("servo BL :", filter(lastAngleRL, getAngle(backLeftAnalog, BL_OFFSET)));
//        telemetry.addData("servo BR :", filter(lastAngleRR, getAngle(backRightAnalog, BR_OFFSET)));
//
////        double rawFL = (frontLeftAnalog.getVoltage() / 3.3) * 360.0;
////        telemetry.addData("FL Raw Sensor", "%.1f°", rawFL);
////        telemetry.addData("FL Wheel Angle", "%.1f°", getAngle(frontLeftAnalog, FL_OFFSET));
//        telemetry.addData("FL Target", "%.1f°", lastTargetFL);
//        telemetry.addData("FR Target", "%.1f°", lastTargetFR);
//        telemetry.addData("RL Target", "%.1f°", lastTargetRL);
//        telemetry.addData("RR Target", "%.1f°", lastTargetRR);
//
//        telemetry.addData("FR Flipped", wasFlippedFR);
//        telemetry.addData("BR Flipped", wasFlippedBR);
//        telemetry.addData("BL Flipped", wasFlippedBL);
//        telemetry.addData("FL Flipped", wasFlippedFL);
//
//        telemetry.addData("FL Speed", flSpeed);
//        telemetry.addData("FR Speed", frSpeed);
//        telemetry.addData("RL Speed", blSpeed);
//        telemetry.addData("RR Speed", brSpeed);
//
//        //        telemetry.addData("---", "---");
//
////        telemetry.addData("FR Wheel", "%.1f°", getAngle(frontRightAnalog, FR_OFFSET));
////        telemetry.addData("BL Wheel", "%.1f°", getAngle(backLeftAnalog, BL_OFFSET));
////        telemetry.addData("BR Wheel", "%.1f°", getAngle(backRightAnalog, BR_OFFSET));
////        telemetry.addData("FR Flipped", wasFlippedFR);
////        telemetry.addData("BR Flipped", wasFlippedBR);
////        telemetry.addData("BL Flipped", wasFlippedBL);
//
//
//
//        telemetry.update();
//    }
//
//    public void swerveDrive(double y_cmd_field, double x_cmd_field, double turn_cmd, boolean reset) {
////        if (reset) {
////            otos.resetTracking();
////        }
//
////        SparkFunOTOS.Pose2D currentPose = otos.getPosition();
////        double heading_rad = Math.toRadians(currentPose.h);
//        double heading_rad = Math.toRadians(0);
//
//
//        double x_cmd = x_cmd_field * Math.cos(heading_rad) + y_cmd_field * Math.sin(heading_rad);
//        double y_cmd = -x_cmd_field * Math.sin(heading_rad) + y_cmd_field * Math.cos(heading_rad);
//
//        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
//            stopDrive();
//            return;
//        }
//
//        double R = Math.sqrt(L*L + W*W);
//
//        double y_fr = y_cmd - turn_cmd * L;
//        double x_fr = x_cmd - turn_cmd * W;
//        double y_fl = y_cmd - turn_cmd * L;
//        double x_fl = x_cmd + turn_cmd * W;
//        double y_rl = y_cmd + turn_cmd * L;
//        double x_rl = x_cmd + turn_cmd * W;
//        double y_rr = y_cmd + turn_cmd * L;
//        double x_rr = x_cmd - turn_cmd * W;
//
//        double speed_fr = Math.hypot(x_fr, y_fr);
//        double speed_fl = Math.hypot(x_fl, y_fl);
//        double speed_rl = Math.hypot(x_rl, y_rl);
//        double speed_rr = Math.hypot(x_rr, y_rr);
//
//        double angle_fr = Math.toDegrees(Math.atan2(x_fr, y_fr));
//        double angle_fl = Math.toDegrees(Math.atan2(x_fl, y_fl));
//        double angle_rl = Math.toDegrees(Math.atan2(x_rl, y_rl));
//        double angle_rr = Math.toDegrees(Math.atan2(x_rr, y_rr));
//
//        double max = Math.max(Math.max(speed_fr, speed_fl), Math.max(speed_rl, speed_rr));
//        if (max > 1.0) {
//            speed_fr /= max; speed_fl /= max; speed_rl /= max; speed_rr /= max;
//        }
//
//        double current_fr = getAngle(frontRightAnalog, FR_OFFSET);
//        double current_fl = getAngle(frontLeftAnalog, FL_OFFSET);
//        double current_rl = getAngle(backLeftAnalog, BL_OFFSET);
//        double current_rr = getAngle(backRightAnalog, BR_OFFSET);
//
//        current_fr = filter(lastAngleFR, current_fr);
//        current_fl = filter(lastAngleFL, current_fl);
//        current_rl = filter(lastAngleRL, current_rl);
//        current_rr = filter(lastAngleRR, current_rr);
//
//        lastAngleFR = current_fr;
//        lastAngleFL = current_fl;
//        lastAngleRL = current_rl;
//        lastAngleRR = current_rr;
//
//
//        double[] opt_fr, opt_fl, opt_rl, opt_rr;
//        if (speed_fr < 0.05)
//        {
//            opt_fr = new double[]{lastTargetFR, 0.0, wasFlippedFR ? 1 : 0};
//        }
//        else
//        {
//            opt_fr = optimize(angle_fr, speed_fr, current_fr, wasFlippedFR);
//        }
//
//        if (speed_fl < 0.05)
//        {
//            opt_fl = new double[]{lastTargetFL, 0.0, wasFlippedFL ? 1 : 0};
//        }
//        else
//        {
//            opt_fl = optimize(angle_fl, speed_fl, current_fl, wasFlippedFL);
//        }
//
//        if (speed_rl < 0.05)
//        {
//            opt_rl = new double[]{lastTargetRL, 0.0, wasFlippedBL ? 1 : 0};
//        }
//        else
//        {
//            opt_rl = optimize(angle_rl, speed_rl, current_rl, wasFlippedBL);
//        }
//        if (speed_rr < 0.05)
//        {
//            opt_rr = new double[]{lastTargetRR, 0.0, wasFlippedBR ? 1 : 0};
//        }
//        else
//        {
//            opt_rr = optimize(angle_rr, speed_rr, current_rr, wasFlippedBR);
//        }
//
//        wasFlippedFR = opt_fr[2] > 0;
//        wasFlippedFL = opt_fl[2] > 0;
//        wasFlippedBL = opt_rl[2] > 0;
//        wasFlippedBR = opt_rr[2] > 0;
//
//        frSpeed = opt_fr[1] * speed;
//        flSpeed = opt_fl[1] * speed;
//        blSpeed = opt_rl[1] * speed;
//        brSpeed = opt_rr[1] * speed;
//
//        frontRightMotor.setPower(-frSpeed);
//        frontLeftMotor.setPower(flSpeed);
//        backLeftMotor.setPower(blSpeed);
//        backRightMotor.setPower(brSpeed);
//
////        lastTargetFR = opt_fr[0];
////        lastTargetFL = opt_fl[0];
////        lastTargetRL = opt_rl[0];
////        lastTargetRR = opt_rr[0];
//
//        runPID(opt_fl[0], opt_fr[0], opt_rl[0], opt_rr[0]);
//    }
//
//    private void stopDrive() {
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        backRightMotor.setPower(0);
//
//        runPID(lastTargetFL,lastTargetFR,lastTargetRL,lastTargetRR);
//    }
//
//    private void runPID(double tFL ,double tFR, double tRL, double tRR) {
//        double dt = angleTimer.seconds();
//        if (dt < 0.001) dt = 0.001;
//
//        double servoTargetFL = tFL;
//        double servoTargetFR = tFR;
//        double servoTargetRL = tRL;
//        double servoTargetRR = tRR;
//
//        double currentServoAngleFL = filter(lastAngleFL, getAngle(frontLeftAnalog, FL_OFFSET));
//        double currentServoAngleFR = filter(lastAngleFR, getAngle(frontRightAnalog, FR_OFFSET));
//        double currentServoAngleRL = filter(lastAngleRL, getAngle(backLeftAnalog, BL_OFFSET));
//        double currentServoAngleRR = filter(lastAngleRR, getAngle(backRightAnalog, BR_OFFSET));
//
//        double velFL = normalizeAngle(servoTargetFL - lastTargetFL) * dt;
//        double velFR = normalizeAngle(servoTargetFR - lastTargetFR) * dt;
//        double velRL = normalizeAngle(servoTargetRL - lastTargetRL) * dt;
//        double velRR = normalizeAngle(servoTargetRR - lastTargetRR) * dt;
//
//        double powerFL = flPID.calculate(servoTargetFL, currentServoAngleFL, velFL);
//        double powerFR = frPID.calculate(servoTargetFR, currentServoAngleFR, velFR);
//        double powerRL = rlPID.calculate(servoTargetRL, currentServoAngleRL, velRL);
//        double powerRR = rrPID.calculate(servoTargetRR, currentServoAngleRR, velRR);
//
//        angleTimer.reset();
//
////        powerFL = 0.15 * gamepad1.left_stick_x;
//
//        frontLeftServo.setPower(powerFL);
//        frontRightServo.setPower(powerFR);
//        backLeftServo.setPower(powerRL);
//        backRightServo.setPower(powerRR);
//
//
//        lastTargetFL = tFL;
//        lastTargetFR = tFR;
//        lastTargetRL = tRL;
//        lastTargetRR = tRR;
//    }
//
//    private double getAngle(AnalogInput sensor, double offset) {
//        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;
//
//        double adjustedAngle = (rawAngle/GEARBOX_RATIO) - offset; // Coordinate missmatch between servo and wheel space!
//
//        return normalizeAngle(adjustedAngle);
//    }
//
//    double alpha = 0.8;
//
//    private double filter(double prev, double angle)
//    {
//        return (angle * alpha + prev * (1-alpha));
//    }
//
//    private double normalizeAngle(double angle) {
//
//        angle = (angle + 180.0) % 360.0;
//        if (angle < 0) angle += 360.0;
//        return angle - 180.0;
//    }
//
//    double FLIP_CENTER = 90;
//    double FLIP_BAND = 5;
//
//    private double[] optimize(
//            double target,
//            double speed,
//            double current,
//            boolean wasFlipped
//    ) {
//        double error = normalizeAngle(target - current);
//        double absError = Math.abs(error);
//
//        // hysteresis around 90 degrees
//        if (!wasFlipped) {
//            if (absError > FLIP_CENTER + FLIP_BAND) wasFlipped = true;
//        } else {
//            if (absError < FLIP_CENTER - FLIP_BAND) wasFlipped = false;
//        }
//
//        if (wasFlipped) {
//            target = normalizeAngle(target + 180.0);
//            speed *= -1;
//        }
//
//        return new double[]{target, speed, wasFlipped ? 1 : 0};
//    }
//
//
//    public class PIDController {
//        private double kP, kI, kD, kF;
//        private double lastError = 0;
//        private ElapsedTime timer = new ElapsedTime();
//
//        public PIDController(double kP, double kI, double kD, double kF) {
//            this.kP = kP;
//            this.kI = kI;
//            this.kD = kD;
//            this.kF = kF;
//            timer.reset();
//        }
//
//        public double calculate(double target, double current, double targetVel) {
//            double error = normalizeAngle(target - current);
//            double dt = timer.seconds();
//
//            if (dt < 0.001) dt = 0.001;
//
//            double pTerm = error * kP;
//            double derivative = (error - lastError) / dt;
//            double dTerm = derivative * kD;
//
//            // PLEASE SPEED I NEED THIS!!!
//            // MY FEED IS KINDA FORWARDLESS!!!
////            double fTerm = targetVel * kF;
//
//            lastError = error;
//            timer.reset();
//
//            double output = pTerm + dTerm;
//
//            if (Math.abs(output) < 1e-6) output = 0;
//            else output += Math.signum(output) * minServoPower;
//
//            return Range.clip(output, -1, 1);
//        }
//    }
//}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class SwerveTest extends OpMode {
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

    DcMotorEx frontRightMotor;
    AnalogInput frontRightAnalog;
    CRServo frontRightServo;

    DcMotorEx backLeftMotor;
    AnalogInput backLeftAnalog;
    CRServo backLeftServo;

    DcMotorEx backRightMotor;
    AnalogInput backRightAnalog;
    CRServo backRightServo;

    PIDController flPID;
    PIDController frPID;

    PIDController rlPID;
    PIDController rrPID;

    ElapsedTime angleTimer = new ElapsedTime();

//    SparkFunOTOS otos;



    double FLkP = 0.002;
    double FLkI = 0.0;
    double FLkD = 0.0001;
    double FLkF = 0.0;

    double FRkP = 0.002;
    double FRkI = 0.0;
    double FRkD = 0.0001;
    double FRkF = 0.0;

    double RLkP = 0.002;
    double RLkI = 0.0;
    double RLkD = 0.0001;
    double RLkF = 0.0;

    double RRkP = 0.002;
    double RRkI = 0.0;
    double RRkD = 0.0001;
    double RRkF = 0.0;

    double inputAngle = 0;
    double turnSpeedDeg = 60;

    double FL_OFFSET = 0; // Test these offsets LMAO
    double FR_OFFSET = 0;
    double BL_OFFSET = 0;
    double BR_OFFSET = 0;


    final double GEARBOX_RATIO = 1 / (36.0f / 24.0f);

    double minServoPower = 0.03;


    double lastTargetFR = 0, lastTargetFL = 0, lastTargetRL = 0, lastTargetRR = 0;

    double speed = 0.4;

//    boolean wasFlippedFL, wasFlippedFR, wasFlippedBL, wasFlippedBR = false;

    double flSpeed, frSpeed, blSpeed, brSpeed = 0;

    double angle_fl, angle_fr, angle_rl, angle_rr = 0;

    double initFR = 0;
    double initFL = 0;
    double initRL = 0;
    double initRR = 0;

    // ==================== CUMULATIVE ANGLE TRACKING ====================
    //
    // THE FACKING RAHHHHHHH PROBLEM:
    // Our analog encoder reads 0-360° on the SERVO, but due to the gear ratio (0.75 or whatevers),
    // the WHEEL rotates only 240° for each full 360° servo rotation or 540 or whatever for a full rotation..
    // When the servo crosses 360°→0°, the sensor jumps, causing the PID to freak out and our wheel to flip thinking it's right but it's wrong yatayat.
    //
    // THE FACKING RAHHHHHHH SOLUTION:
    // Instead of using the raw rAAAHHHH servo angle, we track how much the wheel has ACTUALLY rotated
    // since startup. This cumulative RAAAAHHHHH angle can go beyond ±180° (e.g., 400°, -300°, etc.)
    // and never has sudden jumps.
    //
    // How my genius brain has done it......
    // 1. Each loop, we read the current rAAAHHHH servo angle
    // 2. We DELTA!!!
    // 3. If delta is huge (>180°), we know the sensor wrapped, so we correct it
    // 4. We add the corrected DELTA to our cumulative RAAAAHHHHH angle
    //
    // Example: If servo goes 359° → 1°, the raw DELTA is -358°.
    //          We detect this wrap and correct it to +2° (the actual FACKING MOVEMENTS!!!!!!). HUEWHIOWOIFEJW
    // ======================================================================

    // Cumulative wheel angles - these keep counting up/down and never reset at 360°
    // Can be any value: -500°, 0°, 720°, 1 GOOGL PLEXX!!!!!!!
    double cumulativeAngleFL = 0;
    double cumulativeAngleFR = 0;
    double cumulativeAngleRL = 0;
    double cumulativeAngleRR = 0;

    // Previous RAAAAHHHHH SERVO readings (0-360°) - used to detect Harvey's Buffalo Chicken Wraps
    double lastRawFL = 0;
    double lastRawFR = 0;
    double lastRawRL = 0;
    double lastRawRR = 0;

    //  UNWIND MY FHISHWIEFWO
    //
    // BACKUP PLAN... SOOO MYSTERIOO: If the cumulative tracking gets confused (wheel thinks it's
    // at correct position but is actually flipped 180°), we can "unwind" by:
    // Rotate the servo GUH!!!
    // When we eat the Buffalo Chicken Wrap we know the servo angle yuhhh!!
    // EEEEE ERRRRR Recalibrate the cum angle to known position
    //
    // Each module can unwind independently.

    // Current state for each swerve module
    SwerveState stateFL = SwerveState.MAIN_STATE;
    SwerveState stateFR = SwerveState.MAIN_STATE;
    SwerveState stateRL = SwerveState.MAIN_STATE;
    SwerveState stateRR = SwerveState.MAIN_STATE;

    // Direction to unwind: +1 = clockwise, -1 = counter-clockwise... WHERRRREEEEEEE HAVVVVVEEEEEEE YOUUUUUU BEEEEEEENNN
    double unwindDirectionFL = 1;
    double unwindDirectionFR = 1;
    double unwindDirectionRL = 1;
    double unwindDirectionRR = 1;

    // Unwind servo power (slow and steady... OR FREAKY AND FAST!!!)
    double UNWIND_POWER = 0.2;

    // Threshold for detecting Buffalo Chicken Wrap (how close to 0° or 360° triggers recalibration)
    double WRAP_THRESHOLD = 20;

    ElapsedTime dx = new ElapsedTime();

    ElapsedTime waitForCalibration = new ElapsedTime();

    @Override
    public void init()
    {
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontLeftServo  = hardwareMap.get(CRServo.class,     "frontLeftServo");
        frontLeftAnalog = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");

        frontRightMotor  = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightServo  = hardwareMap.get(CRServo.class,     "frontRightServo");
        frontRightAnalog = hardwareMap.get(AnalogInput.class, "frontRightAnalog");

        backLeftMotor  = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backLeftServo  = hardwareMap.get(CRServo.class,     "backLeftServo");
        backLeftAnalog = hardwareMap.get(AnalogInput.class, "backLeftAnalog");

        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        backRightServo  = hardwareMap.get(CRServo.class,     "backRightServo");
        backRightAnalog = hardwareMap.get(AnalogInput.class, "backRightAnalog");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


//        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
//        otos.calibrateImu();

        flPID = new PIDController(FLkP, FLkI, FLkD, FLkF);
        frPID = new PIDController(FRkP, FRkI, FRkD, FRkF);
        rlPID = new PIDController(RLkP, RLkI, RLkD, RLkF);
        rrPID = new PIDController(RRkP, RRkI, RRkD, RRkF);
    }

    boolean canSetInit = true;
    boolean startupUnwindTriggered = false;  // Track if we've done the startup unwind

    @Override
    public void loop()
    {
        // PHASE 1 OF ACTION PLANN!!!!!: Wait for sensors to stabilize (first 0.5 seconds or whatevers)
        if (waitForCalibration.seconds() > 0.5 && canSetInit)
        {
            // Initialize cumu angles from first reading
            double rawFL = getRawServoAngle(frontLeftAnalog);
            double rawFR = getRawServoAngle(frontRightAnalog);
            double rawRL = getRawServoAngle(backLeftAnalog);
            double rawRR = getRawServoAngle(backRightAnalog);

            // Store initial RAAAAAHHHHHH readings
            lastRawFL = rawFL;
            lastRawFR = rawFR;
            lastRawRL = rawRL;
            lastRawRR = rawRR;

            // Convert to wheel space and apply offset for initial cum angle
            cumulativeAngleFL = (rawFL / GEARBOX_RATIO) - FL_OFFSET;
            cumulativeAngleFR = (rawFR / GEARBOX_RATIO) - FR_OFFSET;
            cumulativeAngleRL = (rawRL / GEARBOX_RATIO) - BL_OFFSET;
            cumulativeAngleRR = (rawRR / GEARBOX_RATIO) - BR_OFFSET;

            initFL = normalizeAngle(cumulativeAngleFL);
            initFR = normalizeAngle(cumulativeAngleFR);
            initRL = normalizeAngle(cumulativeAngleRL);
            initRR = normalizeAngle(cumulativeAngleRR);

            canSetInit = false;

            // PHASE 2 OF ACTION PLANN!!!!!: Trigger automatic startup unwind
            // Basically unwind to the point where wheel is 0 and known location
            // giving us a known reference point for calibration
            triggerUnwind(1);  // Unwind clockwise
            startupUnwindTriggered = true;
        }
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;

        double rightStickX = gamepad1.right_stick_x;
        boolean reset = gamepad1.options;

        // UNWIND TRIGGERS... Cause i'm too lazy to make it auto...:
        // Press Y to unwind clockwise (if wheels seem flipped)
        // Press A to unwind counter-clockwise
        if (gamepad1.y) {
            triggerUnwind(1);  // Clockwise
        }
        if (gamepad1.a) {
            triggerUnwind(-1);  // Counter-clockwise
        }

        dx.reset();

        swerveDrive(leftStickY, leftStickX, rightStickX, reset);

        // TELEMETRY RAHHHHH
        telemetry.addData("State FL", stateFL);
        telemetry.addData("State FR", stateFR);
        telemetry.addData("State RL", stateRL);
        telemetry.addData("State RR", stateRR);

        telemetry.addData("servo FL:", normalizeAngle(getAngle(frontLeftAnalog, FL_OFFSET)));
        telemetry.addData("servo FR :", normalizeAngle(getAngle(frontRightAnalog, FR_OFFSET)));
        telemetry.addData("servo BL :", normalizeAngle(getAngle(backLeftAnalog, BL_OFFSET)));
        telemetry.addData("servo BR :", normalizeAngle(getAngle(backRightAnalog, BR_OFFSET)));

        // Also show cumulative angles to help debug
        telemetry.addData("Cumulative FL", "%.1f°", cumulativeAngleFL);
        telemetry.addData("Cumulative FR", "%.1f°", cumulativeAngleFR);
        telemetry.addData("Cumulative RL", "%.1f°", cumulativeAngleRL);
        telemetry.addData("Cumulative RR", "%.1f°", cumulativeAngleRR);

        telemetry.addData("FL Speed", flSpeed);
        telemetry.addData("FR Speed", frSpeed);
        telemetry.addData("RL Speed", blSpeed);
        telemetry.addData("RR Speed", brSpeed);

        telemetry.addData("FL Angle", getAngle(frontLeftAnalog, FL_OFFSET));
        telemetry.addData("FR Angle", getAngle(frontRightAnalog, FR_OFFSET));
        telemetry.addData("RL Angle", getAngle(backLeftAnalog, BL_OFFSET));
        telemetry.addData("RR Angle", getAngle(backRightAnalog, BR_OFFSET));
//        telemetry.addData("FL Target", "%.1f°", lastTargetFL);;
        telemetry.update();
    }

    // No need for wrap detection... Optimize is now in charge of the motors buddy...

    public enum SwerveState
    {
        UNWINDING_STATE,  // Rotating servo to find the wrap point
        MAIN_STATE        // Normal swerve operation
    }

    /**
     * Triggers an unwind for all modules.
     * Call this if you lowkey thunking it up and the wheels are flipped 180° from where they should be.
     * @param direction +1 for clockwise unwind, -1 for counter-clockwise
     */
    public void triggerUnwind(double direction) {
        stateFL = SwerveState.UNWINDING_STATE;
        stateFR = SwerveState.UNWINDING_STATE;
        stateRL = SwerveState.UNWINDING_STATE;
        stateRR = SwerveState.UNWINDING_STATE;
        unwindDirectionFL = direction;
        unwindDirectionFR = direction;
        unwindDirectionRL = direction;
        unwindDirectionRR = direction;
    }

    /**
     * Nahndles the unwinding stute for mama modules!!!!.
     * Returns true if still unwinding, false if done.
     */
    private boolean processUnwind(CRServo servo, AnalogInput analog, double offset,
                                  double direction, double lastRaw,
                                  int moduleIndex) {
        double rawAngle = getRawServoAngle(analog);

        // Check if we've crossed the Buffalo Chicken wrap point (near 0° or near 360°)
        // We detect a Buffalo Chickenwrap by seeing if we've moved from one side to the other
        boolean wrappedForward = (lastRaw > 360 - WRAP_THRESHOLD && rawAngle < WRAP_THRESHOLD);
        boolean wrappedBackward = (lastRaw < WRAP_THRESHOLD && rawAngle > 360 - WRAP_THRESHOLD);

        if (wrappedForward || wrappedBackward) {
            // BUFFALO CHICKENWRAP DETECTED!!!!!!! OMMMM NONMMMM NOMMMM Recalibrate the cum angle
            // The servo is now at approximately 0° (or 360°)
            double wheelAngle = (rawAngle / GEARBOX_RATIO) - offset;

            // Update the appropriate cumulative angle based on module index
            switch (moduleIndex) {
                case 0: cumulativeAngleFL = wheelAngle; lastRawFL = rawAngle; break;
                case 1: cumulativeAngleFR = wheelAngle; lastRawFR = rawAngle; break;
                case 2: cumulativeAngleRL = wheelAngle; lastRawRL = rawAngle; break;
                case 3: cumulativeAngleRR = wheelAngle; lastRawRR = rawAngle; break;
            }

            servo.setPower(0);
            return false;
        }

        // Still unwinding?? - FAAAAAHHHHH
        servo.setPower(UNWIND_POWER * direction);
        return true;
    }

    public void swerveDrive(double y_cmd_field, double x_cmd_field, double turn_cmd, boolean reset) {
        // ==================== UNWIND STATE HANDLING ====================
        // If any module is unwinding, handle that first before normal swerve
        boolean anyUnwinding = false;

        if (stateFL == SwerveState.UNWINDING_STATE) {
            boolean stillUnwinding = processUnwind(frontLeftServo, frontLeftAnalog, FL_OFFSET,
                    unwindDirectionFL, lastRawFL, 0);
            if (!stillUnwinding) stateFL = SwerveState.MAIN_STATE;
            else anyUnwinding = true;
        }
        if (stateFR == SwerveState.UNWINDING_STATE) {
            boolean stillUnwinding = processUnwind(frontRightServo, frontRightAnalog, FR_OFFSET,
                    unwindDirectionFR, lastRawFR, 1);
            if (!stillUnwinding) stateFR = SwerveState.MAIN_STATE;
            else anyUnwinding = true;
        }
        if (stateRL == SwerveState.UNWINDING_STATE) {
            boolean stillUnwinding = processUnwind(backLeftServo, backLeftAnalog, BL_OFFSET,
                    unwindDirectionRL, lastRawRL, 2);
            if (!stillUnwinding) stateRL = SwerveState.MAIN_STATE;
            else anyUnwinding = true;
        }
        if (stateRR == SwerveState.UNWINDING_STATE) {
            boolean stillUnwinding = processUnwind(backRightServo, backRightAnalog, BR_OFFSET,
                    unwindDirectionRR, lastRawRR, 3);
            if (!stillUnwinding) stateRR = SwerveState.MAIN_STATE;
            else anyUnwinding = true;
        }

        // If any module is still unwinding, stop the drive motors and skip normal swerve... NOOOOO
        if (anyUnwinding) {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

            // Still need to update lastRAAAAHHHHH for modules that aren't unwinding
            // to keep cum tracking working
            updateCumulativeAngles();
            return;
        }

        double heading_rad = Math.toRadians(0);

        double x_cmd = x_cmd_field * Math.cos(heading_rad) + y_cmd_field * Math.sin(heading_rad);
        double y_cmd = -x_cmd_field * Math.sin(heading_rad) + y_cmd_field * Math.cos(heading_rad);

        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stopDrive();
            return;
        }

        double y_fr = y_cmd - turn_cmd * W;
        double x_fr = x_cmd - turn_cmd * L;
        double y_fl = y_cmd - turn_cmd * W;
        double x_fl = x_cmd + turn_cmd * L;
        double y_rl = y_cmd + turn_cmd * W;
        double x_rl = x_cmd + turn_cmd * L;
        double y_rr = y_cmd + turn_cmd * W;
        double x_rr = x_cmd - turn_cmd * L;

        double speed_fr = Math.hypot(x_fr, y_fr);
        double speed_fl = Math.hypot(x_fl, y_fl);
        double speed_rl = Math.hypot(x_rl, y_rl);
        double speed_rr = Math.hypot(x_rr, y_rr);

        angle_fr = Math.toDegrees(Math.atan2(x_fr, y_fr));
        angle_fl = Math.toDegrees(Math.atan2(x_fl, y_fl));
        angle_rl = Math.toDegrees(Math.atan2(x_rl, y_rl));
        angle_rr = Math.toDegrees(Math.atan2(x_rr, y_rr));

        double max = Math.max(Math.max(speed_fr, speed_fl), Math.max(speed_rl, speed_rr));
        if (max > 1.0) {
            speed_fr /= max; speed_fl /= max; speed_rl /= max; speed_rr /= max;
        }

        // Update cum angles (handle buffalo chicken wrap)
        updateCumulativeAngles();

        // Use cum angles for current position??
        double current_fr = cumulativeAngleFR;
        double current_fl = cumulativeAngleFL;
        double current_rl = cumulativeAngleRL;
        double current_rr = cumulativeAngleRR;

        double[] opt_fr = optimize(angle_fr, speed_fr, current_fr);
        double[] opt_fl = optimize(angle_fl, speed_fl, current_fl);
        double[] opt_rl = optimize(angle_rl, speed_rl, current_rl);
        double[] opt_rr = optimize(angle_rr, speed_rr, current_rr);

        // opt_*[1] already has the correct sign (negative if flipped)
        frSpeed = opt_fr[1] * speed;
        flSpeed = opt_fl[1] * speed;
        blSpeed = opt_rl[1] * speed;
        brSpeed = opt_rr[1] * speed;

        frontRightMotor.setPower(frSpeed);
        frontLeftMotor.setPower(flSpeed);
        backLeftMotor.setPower(blSpeed);
        backRightMotor.setPower(brSpeed);

        lastTargetFR = opt_fr[0];
        lastTargetFL = opt_fl[0];
        lastTargetRL = opt_rl[0];
        lastTargetRR = opt_rr[0];

        runPID(opt_fl[0], opt_fr[0], opt_rl[0], opt_rr[0]);
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        runPID(lastTargetFL,lastTargetFR,lastTargetRL,lastTargetRR);
    }

    private void runPID(double tFL ,double tFR, double tRL, double tRR) {
        double dt = angleTimer.seconds();
        if (dt < 0.001) dt = 0.001;

        double servoTargetFL = tFL;
        double servoTargetFR = tFR;
        double servoTargetRL = tRL;
        double servoTargetRR = tRR;

        double currentServoAngleFL = cumulativeAngleFL;
        double currentServoAngleFR = cumulativeAngleFR;
        double currentServoAngleRL = cumulativeAngleRL;
        double currentServoAngleRR = cumulativeAngleRR;

        double velFL = (servoTargetFL - lastTargetFL) * dt;
        double velFR = (servoTargetFR - lastTargetFR) * dt;
        double velRL = (servoTargetRL - lastTargetRL) * dt;
        double velRR = (servoTargetRR - lastTargetRR) * dt;

        // For cumulative angles, we use direct difference (not normalized)
        double powerFL = flPID.calculateCumulative(servoTargetFL, currentServoAngleFL, velFL);
        double powerFR = frPID.calculateCumulative(servoTargetFR, currentServoAngleFR, velFR);
        double powerRL = rlPID.calculateCumulative(servoTargetRL, currentServoAngleRL, velRL);
        double powerRR = rrPID.calculateCumulative(servoTargetRR, currentServoAngleRR, velRR);

        angleTimer.reset();

//        frontLeftServo.setPower(powerFL);
//        frontRightServo.setPower(powerFR);
//        backLeftServo.setPower(powerRL);
//        backRightServo.setPower(powerRR);

        frontLeftServo.setPower(gamepad1.left_stick_x);
        frontRightServo.setPower(0);
        backLeftServo.setPower(0);
        backRightServo.setPower(0);

        lastTargetFL = tFL;
        lastTargetFR = tFR;
        lastTargetRL = tRL;
        lastTargetRR = tRR;
    }

    /**
     * Gets the RAAHAHHHHH servo angle in degrees (0-360), before poopy gear ratio
     */
    private double getRawServoAngle(AnalogInput sensor) {
        return (sensor.getVoltage() / 3.3) * 360.0;
    }

    /**
     * Updates cum angles for buffalo chicken wrap-around detection.
     * Call this once per poop iteration, before using cumulative angles.
     *
     * GUH DETECTION ESPXOAJIO:
     * - The servo sensor reads 0-360°, so when it rotates past 360°, it jumps back to 0°
     * - If the servo was at 350° last loop and is now at 10°, the raw delta is -340°
     * - But physically, the servo only moved +20° (350° → 360° → 10°)
     * - We detect this by checking: if |delta| > 180°, a wrap occurred
     * - To correct: if delta > 180, subtract 360; if delta < -180, add 360
     * - This turns -340° into +20° (THE RAHHHHHH MOVEMENT)
     */
    private void updateCumulativeAngles() {
        // STEP 1: Read current raw servo angles (in servo space, 0-360°)
        double rawFL = getRawServoAngle(frontLeftAnalog);
        double rawFR = getRawServoAngle(frontRightAnalog);
        double rawRL = getRawServoAngle(backLeftAnalog);
        double rawRR = getRawServoAngle(backRightAnalog);

        // DELTA = THE CHANGE IN RAHHHHHHHH
        double deltaFL = rawFL - lastRawFL;
        double deltaFR = rawFR - lastRawFR;
        double deltaRL = rawRL - lastRawRL;
        double deltaRR = rawRR - lastRawRR;

        // IF THE DELTA RAHHHHH THEN FLIP
        if (deltaFL > 180) deltaFL -= 360;
        if (deltaFL < -180) deltaFL += 360;
        if (deltaFR > 180) deltaFR -= 360;
        if (deltaFR < -180) deltaFR += 360;
        if (deltaRL > 180) deltaRL -= 360;
        if (deltaRL < -180) deltaRL += 360;
        if (deltaRR > 180) deltaRR -= 360;
        if (deltaRR < -180) deltaRR += 360;

        // ADD DELTA RAHHHHH TO CUMULATIVE RAHHHHHH
        cumulativeAngleFL += deltaFL / GEARBOX_RATIO;
        cumulativeAngleFR += deltaFR / GEARBOX_RATIO;
        cumulativeAngleRL += deltaRL / GEARBOX_RATIO;
        cumulativeAngleRR += deltaRR / GEARBOX_RATIO;

        // REMEMBER THE LAST RAHHHHHHHH
        lastRawFL = rawFL;
        lastRawFR = rawFR;
        lastRawRL = rawRL;
        lastRawRR = rawRR;
    }

    private double getAngle(AnalogInput sensor, double offset)
    {
        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;

        double adjustedAngle = (rawAngle/GEARBOX_RATIO) - offset; // Coordinate mismatch between servo and wheel space!

        return adjustedAngle;
    }

    // No longer used RAHHHHH
    private double getRawAngle(AnalogInput sensor, double offset) {
        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;

        if (rawAngle < 0)
        {
            rawAngle *= -1;
            rawAngle = 360 - rawAngle;
        }

        // Apply offset in servo space (before gearbox conversion)
        return (rawAngle - offset) / GEARBOX_RATIO;
    }


    private double normalizeAngle(double angle)
    {
        angle = (angle + 180.0) % 360.0;
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }

    /**
     * OPTIMIZE FUNCTION... MAAAAAHHHHH BREEADDD AND BUTTTTERRRRRRR - Minimizes wheel travel by potentially flipping 180°?? GUH?
     *
     * THE PROBLEM WITH CUM angles... GUH... Okay lock in on comments
     * - Target comes from atan2, so it's always -180° to +180°
     * - But cumulative angle can be anything: -500°, 0°, 400°, 720°, etc.
     * - We need to find the equivalent target in the cumulative angle's "neighborhood"
     *
     * EXAMPLE: GUH
     * - Joystick says "go to 45°" (target = 45°)
     * - Wheel is currently at 380° (cumulative, which is equivalent to 20°)
     * - We need to adjust target to 405° (45° + 360°) to be in the same range
     * - Then delta = 405° - 380° = 25°, which is the actual travel needed
     *
     * THE FLIP OPTIMIZATION: GUH
     * - If the wheel would need to rotate more than 90°, it's faster to:
     *   1. Flip the target by 180° (point the wheel the opposite way)
     *   2. Reverse the RAAAAHHHH MOTOR direction (negative speed)
     * - This way the wheel never needs to rotate more than 90° to reach any target
     *
     * @param target  The desired wheel angle (-180° to +180° from joystick)
     * @param speed   The desired RAAAAHHH motor speed
     * @param current The current cumulative wheel angle (can be any value)
     * @return [adjustedTarget, adjustedSpeed] - target in cum space??, speed may be negated
     */
    private double[] optimize(
            double target,
            double speed,
            double current
    ) {
        // STEP 1 of SECOND ACTION PLAAAAANNNNNN: Figure out which "rotation pluh" the wheel is currently in
        // If current = 380°, rotations = floor((380+180)/360) = floor(1.56) = 1... lowkey learned about the floor in like 5 mins
        // This means we're in the "first" full rotation past zero
        double rotations = Math.floor((current + 180) / 360.0);

        // STEP 2 of SECOND ACTION PLAAAAANNNNNN: Bring target up to the same rotation level
        // If target = 45° and rotations = 1, adjustedTarget = 45° + 360° = 405°
        double adjustedTarget = target + (rotations * 360);

        // STEP 3 of SECOND ACTION PLAAAAANNNNNN: Calculate how far we need to rotate my ballsack... Okay that's not okay imma get rid of that later
        double delta = adjustedTarget - current;

        // STEP 4 of SECOND ACTION PLAAAAANNNNNN: If we need to rotate more than 90°, flip instead!! GUH!!!
        if (Math.abs(delta) > 90) {
            // Flip the target by 180° (in the direction that minimizes travel)
            if (delta > 0) {
                adjustedTarget -= 180;  // Target was ahead, bring it back... Who decided that???
            } else {
                adjustedTarget += 180;  // Target was behind, push it forward... Who decided that???
            }
            // Reverse motor direction since wheel is now pointing "backwards"... Who decided that???
            speed *= -1;
        }

        return new double[]{adjustedTarget, speed};
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

            if (dt < 0.001) dt = 0.001;

            double pTerm = error * kP;
            double derivative = (error - lastError) / dt;
            double dTerm = derivative * kD;

            lastError = error;

            double output = pTerm + dTerm;

            if (Math.abs(error) < 5.0)
                return Math.signum(error) * minServoPower;

            output += Math.signum(output) * minServoPower;

            timer.reset();

            return Range.clip(output, -1.0, 1.0);
        }

        /**
         * Calculate PID output for cumulative (unwrapped) angles.
         * Uses direct error instead of normalized, since cumulative angles don't wrap.
         */
        public double calculateCumulative(double target, double current, double targetVel) {
            double error = target - current;  // Direct difference for cumulative angles
            double dt = timer.seconds();

            if (dt < 0.001) dt = 0.001;

            double pTerm = error * kP;
            double derivative = (error - lastError) / dt;
            double dTerm = derivative * kD;

            lastError = error;

            double output = pTerm + dTerm;

            if (Math.abs(error) < 5.0)
                return Math.signum(error) * minServoPower;

            output += Math.signum(output) * minServoPower;

            timer.reset();

            return Range.clip(output, -1.0, 1.0);
        }
    }
}