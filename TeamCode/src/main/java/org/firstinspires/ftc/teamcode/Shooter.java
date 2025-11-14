package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    Servo Hood1;
    Servo Hood2;
    boolean LastUp = false;
    boolean LastDown = false;
    double position = 0.2;

    double shooterSpeed = 0.7;

    double lastEncoder = 0;
    double lastRPMERR = 0;

    double kp = 1;
    double kd = 0;

    final double COUNTS_PER_MOTOR_REV = 28.0;
    final double GEAR_REDUCTION = 1;
    final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;

    ElapsedTime dt = new ElapsedTime();
    double encoderResolution = 28;

    public void init(HardwareMap hwMap) {
        shooter1 = hwMap.get(DcMotorEx.class, "Shooter1"); //
        shooter2 = hwMap.get(DcMotorEx.class, "Shooter2");
        Hood1 = hwMap.get(Servo.class, "Hood1");
        Hood2 = hwMap.get(Servo.class, "Hood2");
        Hood2.setDirection(Servo.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(kp,0,kd,0));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(kp,0,kd,0));
    }
    public void ActivateShooter(boolean Shoot, boolean backShoot) {
        if (Shoot)
        {
            shooter1.setPower(shooterSpeed);
            shooter2.setPower(shooterSpeed);
        }

        else if (backShoot) {
            shooter1.setPower(-shooterSpeed * 0.35);
            shooter2.setPower(-shooterSpeed * 0.35);
        }
        else
        {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
    }

    public void FireAtRPM(int rpm)
    {
        double dx = dt.milliseconds();

        double currentRPM = (shooter1.getCurrentPosition() / encoderResolution - lastEncoder) / dx;

        double error = rpm-currentRPM;

        double derivative = (error - lastRPMERR) / dx;
        double proportional = error;

        lastEncoder = shooter1.getCurrentPosition() / encoderResolution;
        lastRPMERR = error;

        double output = proportional * kp + derivative * kd;

        output /= 8400; // normalize into 0-1 range

        // creates feedforward just to make sure it still moves
        if (output < 0.2)
        {
            output = 0.2;
        }
        else if (output > 1)
        {
            output = 1;
        }

        SetPowerPID(output);

        dt.reset();
    }

    public void SetShooterRPM(double targetRPM)
    {
        double tps = (targetRPM/60) * COUNTS_PER_WHEEL_REV;
        shooter1.setVelocity(tps);
        shooter2.setVelocity(tps);
    }

    private void SetPowerPID(double input)
    {
        shooter1.setPower(input);
        shooter2.setPower(input);
    }

    public void HoodStuff(boolean HoodUp, boolean HoodDown) {
        if (HoodUp && !LastUp && position <= 0.8) {
            position += 0.1;
        }
        if (HoodDown && !LastDown && position >= 0.1) {
            position -= 0.1;
        }
        // Random comment

        LastDown = HoodDown;
        LastUp = HoodUp;
        Hood1.setPosition(position);
        Hood2.setPosition(position);
    }

    public void SetShooterPower(double power)
    {
        shooterSpeed = power;
    }

    public double GetHoodPosition()
    {
        return position;
    }

    public double GetShooterRPM()
    {
        return shooter1.getVelocity() * 60 / COUNTS_PER_WHEEL_REV;
    }

    public boolean RPMCorrect(double target)
    {
        return (Math.abs(GetShooterRPM() - target) < 100);
    }
}