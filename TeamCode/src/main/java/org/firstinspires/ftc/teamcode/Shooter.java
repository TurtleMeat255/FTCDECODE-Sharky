package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    Servo Hood1;
    Servo Hood2;
    boolean LastUp = false;
    boolean LastDown = false;
    double position = 0;

    double shooterSpeed = 1;

    double lastEncoder = 0;
    double lastRPMERR = 0;

    double kp = 0.1;
    double kd = 0;


    ElapsedTime dt = new ElapsedTime();
    double encoderResolution = 28;

    public void init(HardwareMap hwMap) {
        shooter1 = hwMap.get(DcMotorEx.class, "Shooter1"); //
        shooter2 = hwMap.get(DcMotorEx.class, "Shooter2");
        Hood1 = hwMap.get(Servo.class, "Hood1");
        Hood2 = hwMap.get(Servo.class, "Hood2");

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kp,0,kd,0.2));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kp,0,kd,0.2));

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
    //    Hood1.setDirection(Servo.Direction.REVERSE);
    }
    public void ActivateShooter(boolean Shoot) {
        if (Shoot)
        {
            shooter1.setPower(shooterSpeed);
            shooter2.setPower(shooterSpeed);
        }
        else
        {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
    }

    public void FireAtRPM(int rpm)
    {
        double outputVel = rpm * encoderResolution/60;
        shooter1.setVelocity(outputVel);
        shooter2.setVelocity(outputVel);
    }

//    public void FireAtRPM(int rpm)
//    {
//        double dx = dt.milliseconds();
//
//        double currentRPM = (shooter1.getCurrentPosition() / encoderResolution - lastEncoder) / dx;
//        double error = rpm - currentRPM;
//
//        double derivative = (error - lastRPMERR) / dx;
//        double proportional = error;
//
//        lastEncoder = shooter1.getCurrentPosition() / encoderResolution;
//        lastRPMERR = error;
//
//        double output = proportional * kp + derivative * kd;
//
//        SetPowerPID(output);
//
//        dt.reset();
//    }

//    private void SetPowerPID(double input)
//    {
//        shooter1.setPower(input);
//        shooter2.setPower(input);
//    }
//    public void HoodStuff(boolean HoodUp, boolean HoodDown) {
//        if (HoodUp && !LastUp && position <= 0.6) {
//            position += 0.2;
//        }
//        if (HoodDown && !LastDown && position >= 0.2) {
//            position -= 0.2;
//        }
//        // Random comment
//
//        LastDown = HoodDown;
//        LastUp = HoodUp;
//        Hood1.setPosition(position);
//        Hood2.setPosition(position);
//    }
}