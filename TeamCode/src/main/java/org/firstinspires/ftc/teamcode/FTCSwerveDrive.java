package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime; // Needed for PID


public class FTCSwerveDrive extends SubsystemBase {

    Motor frontLeftMotor;
    Motor backLeftMotor;
    Motor frontRightMotor;
    Motor backRightMotor;
    CRServo frontLeftServo;
    CRServo backLeftServo;
    CRServo frontRightServo;
    CRServo backRightServo;

    SparkFunOTOS otos;

    AnalogInput frontLeftAnalog;
    AnalogInput backLeftAnalog;
    AnalogInput frontRightAnalog;
    AnalogInput backRightAnalog;

    //  PID Controller Instances
    PIDController frPID;
    PIDController flPID;
    PIDController rlPID;
    PIDController rrPID;

    final double L = 8.0;
    final double W = 8.0;

    double FRkP = 0.03;
    double FRkI = 0.0; // Adjusted for FTCLib likely needing non-zero I if used, or keep 0
    double FRkD = 0.001;

    double FLkP = 0.03;
    double FLkI = 0.0;
    double FLkD = 0.001;

    double RLkP = 0.03;
    double RLkI = 0.0;
    double RLkD = 0.001;

    double RRkP = 0.03;
    double RRkI = 0.0;
    double RRkD = 0.001;


    public void init(HardwareMap hwMap) {
        frontLeftMotor  = new Motor(hwMap, "frontLeftMotor");
        frontLeftServo  = new CRServo(hwMap, "frontLeftServo");
        frontLeftAnalog = hwMap.get(AnalogInput.class, "frontLeftAnalog");

        frontRightMotor = new Motor(hwMap, "frontRightMotor");
        frontRightServo = new CRServo(hwMap, "frontRightServo");
        frontRightAnalog = hwMap.get(AnalogInput.class, "frontRightAnalog");

        backRightMotor  = new Motor(hwMap, "backRightMotor");
        backRightServo  = new CRServo(hwMap, "backRightServo");
        backRightAnalog = hwMap.get(AnalogInput.class, "backRightAnalog");

        backLeftMotor   = new Motor(hwMap, "backLeftMotor");
        backLeftServo   = new CRServo(hwMap, "backLeftServo");
        backLeftAnalog  = hwMap.get(AnalogInput.class, "backLeftAnalog");

        otos = hwMap.get(SparkFunOTOS.class, "otos");
        otos.calibrateImu();

        frPID = new PIDController(FRkP, FRkI, FRkD);
        flPID = new PIDController(FLkP, FLkI, FLkD);
        rlPID = new PIDController(RLkP, RLkI, RLkD);
        rrPID = new PIDController(RRkP, RRkI, RRkD);
    }

    public void swerveDrive(double fwd_cmd_field, double strafe_cmd_field, double turn_cmd, boolean reset) {

        SparkFunOTOS.Pose2D currentPose = otos.getPosition();
        double heading_rad = Math.toRadians(currentPose.h);

        // Field-centric conversion NOT GRADE 9 METH
        double x_cmd_robot = fwd_cmd_field * Math.sin(heading_rad) + strafe_cmd_field * Math.cos(heading_rad);
        double y_cmd_robot = fwd_cmd_field * Math.cos(heading_rad) - strafe_cmd_field * Math.sin(heading_rad);
        


        double y_fr = y_cmd_robot - turn_cmd * W; 
        double x_fr = x_cmd_robot + turn_cmd * L;
        
        double y_fl = y_cmd_robot + turn_cmd * W;
        double x_fl = x_cmd_robot + turn_cmd * L;
        
        double y_rl = y_cmd_robot + turn_cmd * W;
        double x_rl = x_cmd_robot - turn_cmd * L;
        
        double y_rr = y_cmd_robot - turn_cmd * W;
        double x_rr = x_cmd_robot - turn_cmd * L;

        double speed_fr = Math.hypot(x_fr, y_fr);
        double speed_fl = Math.hypot(x_fl, y_fl);
        double speed_rl = Math.hypot(x_rl, y_rl);
        double speed_rr = Math.hypot(x_rr, y_rr);
        double angle_fr_deg = Math.toDegrees(Math.atan2(x_fr, y_fr));
        double angle_fl_deg = Math.toDegrees(Math.atan2(x_fl, y_fl));
        double angle_rl_deg = Math.toDegrees(Math.atan2(x_rl, y_rl));
        double angle_rr_deg = Math.toDegrees(Math.atan2(x_rr, y_rr));
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

        double actual_angle_fr = getAngle(frontRightAnalog);
        double actual_angle_fl = getAngle(frontLeftAnalog);
        double actual_angle_rl = getAngle(backLeftAnalog);
        double actual_angle_rr = getAngle(backRightAnalog);

        double[] optimized_fr = optimize(angle_fr_deg, speed_fr, actual_angle_fr);
        double[] optimized_fl = optimize(angle_fl_deg, speed_fl, actual_angle_fl);
        double[] optimized_rl = optimize(angle_rl_deg, speed_rl, actual_angle_rl);
        double[] optimized_rr = optimize(angle_rr_deg, speed_rr, actual_angle_rr);

        double target_fr = optimized_fr[0];
        speed_fr = optimized_fr[1];

        double target_fl = optimized_fl[0];
        speed_fl = optimized_fl[1];

        double target_rl = optimized_rl[0];
        speed_rl = optimized_rl[1];

        double target_rr = optimized_rr[0];
        speed_rr = optimized_rr[1];

        double steer_power_fr = frPID.calculate(target_fr, actual_angle_fr);
        double steer_power_fl = flPID.calculate(target_fl, actual_angle_fl);
        double steer_power_rl = rlPID.calculate(target_rl, actual_angle_rl);
        double steer_power_rr = rrPID.calculate(target_rr, actual_angle_rr);

        frontRightServo.set(steer_power_fr);
        frontRightMotor.set(speed_fr);

        frontLeftServo.set(steer_power_fl);
        frontLeftMotor.set(speed_fl);

        backLeftServo.set(steer_power_rl);
        backLeftMotor.set(speed_rl);

        backRightServo.set(steer_power_rr);
        backRightMotor.set(speed_rr);

        if (reset) {
            otos.setPosition(new SparkFunOTOS.Pose2D(currentPose.x, currentPose.y, 0));
        }
    }

    private double getAngle(AnalogInput sensor) {
        final double VOLT_TO_DEG = 360.0 / 3.3; // Yup... Just yup
        double rawAngle = sensor.getVoltage() * VOLT_TO_DEG;
        return rawAngle % 360.0;
    }

    //angle flip kinda stuff?
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

    // Lowkey, its the most beautiful thing i've ever seen in my life, its a PID controller, INSIDE OF A PID CONTROLLER
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
            double derivative = (error - lastError) / (dt > 0 ? dt : 0.001); // Avoid division by zeroo
            double dTerm = kD * derivative;

            lastError = error;

            double output = pTerm + iTerm + dTerm;

            // Clamp output powerrrrrr... yup
            return Math.max(-1.0, Math.min(1.0, output));
        }

    }
}
