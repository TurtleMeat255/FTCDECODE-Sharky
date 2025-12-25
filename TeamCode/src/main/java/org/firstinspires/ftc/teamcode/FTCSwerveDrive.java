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

    // PID Controller Instances
    PIDController frPID;
    PIDController flPID;
    PIDController rlPID;
    PIDController rrPID;

    final double L = 8.0; // These are uhh place holders... TRUTH NUKE!!
    final double W = 8.0; // Same with these!! MUWAHAAHAHAHAHA!!
    final double GEAR_RATIO = 2.0 / 3.0;

    double FRkP = 0.03;
    double FRkI = 0.0;
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
        // otos.setOffset(new SparkFunOTOS.Pose2D(1.5,-7,Math.PI)); I need the CAD!!!
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
        double x_cmd_robot = fwd_cmd_field * Math.cos(heading_rad) + strafe_cmd_field * Math.sin(heading_rad);
        double y_cmd_robot = -fwd_cmd_field * Math.sin(heading_rad) + strafe_cmd_field * Math.cos(heading_rad);

        // OKAY... So basically for rotation I need distance the wheel is from the center so it's like
        // Pythagorean Theorem. Right...
        double R = Math.hypot(L, W);

        // Then I assign each of the X and Y of a cardinal plane direction a letter cause why not.
        double a = x_cmd_robot - turn_cmd * (L / R);
        double b = x_cmd_robot + turn_cmd * (L / R);
        double c = y_cmd_robot - turn_cmd * (W / R);
        double d = y_cmd_robot + turn_cmd * (W / R);

        double x_fr = b, y_fr = c;
        double x_fl = b, y_fl = d;
        double x_rl = a, y_rl = d;
        double x_rr = a, y_rr = c;

        // Then just stuff those into the wheels. So NOW it should work correctly.
        double speed_fr = Math.hypot(x_fr, y_fr);
        double speed_fl = Math.hypot(x_fl, y_fl);
        double speed_rl = Math.hypot(x_rl, y_rl);
        double speed_rr = Math.hypot(x_rr, y_rr);

        // lowkey, atan2?
        double angle_fr_deg = Math.toDegrees(Math.atan2(y_fr, x_fr));
        double angle_fl_deg = Math.toDegrees(Math.atan2(y_fl, x_fl));
        double angle_rl_deg = Math.toDegrees(Math.atan2(y_rl, x_rl));
        double angle_rr_deg = Math.toDegrees(Math.atan2(y_rr, x_rr));

        double max = Math.max(
                Math.max(speed_fr, speed_fl),
                Math.max(speed_rl, speed_rr)
        );

        // Normalize the spedos...
        if (max > 1.0) {
            speed_fr /= max;
            speed_fl /= max;
            speed_rl /= max;
            speed_rr /= max;
        }

        // 90-Degree Optimization & PID Control
        double actual_fr = getAngle(frontRightAnalog);
        double actual_fl = getAngle(frontLeftAnalog);
        double actual_rl = getAngle(backLeftAnalog);
        double actual_rr = getAngle(backRightAnalog);

        double[] fr = optimize(angle_fr_deg, speed_fr, actual_fr);
        double[] fl = optimize(angle_fl_deg, speed_fl, actual_fl);
        double[] rl = optimize(angle_rl_deg, speed_rl, actual_rl);
        double[] rr = optimize(angle_rr_deg, speed_rr, actual_rr);

        // We drive, we steer. WE DRIVE-STEER
        frontRightServo.set(frPID.calculate(fr[0], actual_fr));
        frontRightMotor.set(fr[1]);

        frontLeftServo.set(flPID.calculate(fl[0], actual_fl));
        frontLeftMotor.set(fl[1]);

        backLeftServo.set(rlPID.calculate(rl[0], actual_rl));
        backLeftMotor.set(rl[1]);

        backRightServo.set(rrPID.calculate(rr[0], actual_rr));
        backRightMotor.set(rr[1]);

        // buh
        if (reset) {
            otos.setPosition(new SparkFunOTOS.Pose2D(currentPose.x, currentPose.y, 0));
        }
    }

    private double getAngle(AnalogInput sensor) {
        // The voltage shall become degrees??
        double deg = (sensor.getVoltage() * (360.0 / 3.3)) / GEAR_RATIO;
        deg %= 360;
        if (deg > 180) deg -= 360;
        return deg;
    }

    private double normalize(double angle) {
        //
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double[] optimize(double target, double speed, double current) {
        // 90-degree optimization: flip wheel if it’s shorter? guh
        double error = normalize(target - current);
        if (Math.abs(error) > 90) {
            target -= Math.signum(error) * 180;
            speed *= -1;
        }
        return new double[]{normalize(target), speed};
    }

    public class PIDController {

        double kP, kI, kD;
        double integral = 0, lastError = 0;
        ElapsedTime timer = new ElapsedTime();

        PIDController(double p, double i, double d) {
            kP = p; kI = i; kD = d;
            timer.reset();
        }

        double calculate(double target, double current) {
            // I'm a genius. I added dead band because WPIlib is lowkey a fraud and FTC is different.
            double error = normalize(target - current);
            double dt = timer.seconds();
            timer.reset();

            integral += error * dt;
            double derivative = (error - lastError) / (dt > 0 ? dt : 0.001);
            lastError = error;

            double output = kP * error + kI * integral + kD * derivative;

            // Smol deadband to prevent the servo from tweaking out.
            if (Math.abs(output) < 0.05) output = 0;
            return Math.max(-1.0, Math.min(1.0, output));
        }
    }
}

