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

    /* working specifications
     far shot
     66 inches from score
     lob
     3300 rpm
     40 degrees hood
     hard
     3100 rpm
     20 degrees hood

     medium shot
     45 inches from score
     3000 rpm
     30 degrees

     close shot
     30 inches from score
     2700 rpm
     40 degree

    far zone
    3700 rpm
    30 degree


    Thanks Jacob!!
     */

    public enum ShootState {
        FAR_LOB_SHOT,
        FAR_HARD_SHOT,
        MEDIUM_SHOT,
        CLOSE_SHOT,
        NO_SHOT,
    }


    DcMotorEx shooter1;
    DcMotorEx shooter2;
    Servo Hood1;
    Servo Hood2;
    AprilTagLimeLight limelight = new AprilTagLimeLight();
    boolean LastUp = false;
    boolean LastDown = false;

    boolean IS_FAR_LOB_SHOT = false;

    boolean IS_FAR_HARD_SHOT = false;

    boolean IS_MEDIUM_SHOT = true;

    boolean IS_CLOSE_SHOT = false;

    double position = 0.2;

    double shooterSpeed = 0.7;

    double lastEncoder = 0;
    double lastRPMERR = 0;

    double kp = 1;
    double kd = 0.01;

    final double COUNTS_PER_MOTOR_REV = 28.0;
    final double GEAR_REDUCTION = 1;
    final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;

    private boolean wasShootButtonPressed = false;

    ElapsedTime dt = new ElapsedTime();
    double encoderResolution = 28;
    private Shooter.ShootState currentState = Shooter.ShootState.MEDIUM_SHOT;

    public void init(HardwareMap hwMap) {
        shooter1 = hwMap.get(DcMotorEx.class, "Shooter1"); //
        shooter2 = hwMap.get(DcMotorEx.class, "Shooter2");
        Hood1 = hwMap.get(Servo.class, "Hood1");
        Hood2 = hwMap.get(Servo.class, "Hood2");
        Hood2.setDirection(Servo.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.REVERSE);
        limelight.init(hwMap);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(kp,0,kd,0));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(kp,0,kd,0));
    }

    public void update(boolean isShootButtonPressed, boolean isHardShotPressed) {

        double currentDistance = limelight.GetDistance();

        final double CLOSE_LIMIT = 40.0;
        final double MEDIUM_LIMIT = 55.0;

        if (isShootButtonPressed) {

            if (currentDistance > 0 && currentDistance < CLOSE_LIMIT) {
                currentState = ShootState.CLOSE_SHOT;
            } else if (currentDistance >= CLOSE_LIMIT && currentDistance < MEDIUM_LIMIT) {
                currentState = ShootState.MEDIUM_SHOT;
            } else if (currentDistance >= MEDIUM_LIMIT) {

                if (isHardShotPressed) {
                    currentState = ShootState.FAR_HARD_SHOT;
                } else {
                    currentState = ShootState.FAR_LOB_SHOT;
                }
            } else {

                currentState = ShootState.NO_SHOT;
            }
        } else {

            currentState = ShootState.NO_SHOT;
        }

        updateShooter();
    }

    public void updateShooter() {
        switch(currentState) {

            case FAR_LOB_SHOT:
                SetShooterPower(3300);
                SetHoodPosition(0.40);

                IS_FAR_LOB_SHOT = true;
                IS_FAR_HARD_SHOT = false;
                IS_MEDIUM_SHOT = false;
                IS_CLOSE_SHOT = false;
                break;

            case FAR_HARD_SHOT:
                SetShooterPower(3100);
                SetHoodPosition(0.2);

                IS_FAR_LOB_SHOT = false;
                IS_FAR_HARD_SHOT = true;
                IS_MEDIUM_SHOT = false;
                IS_CLOSE_SHOT = false;
                break;

            case MEDIUM_SHOT:
                SetShooterPower(3000);
                SetHoodPosition(0.30);

                IS_FAR_LOB_SHOT = false;
                IS_FAR_HARD_SHOT = false;
                IS_MEDIUM_SHOT = true;
                IS_CLOSE_SHOT = false;
                break;

            case CLOSE_SHOT:
                SetShooterPower(2700);
                SetHoodPosition(0.40);

                IS_FAR_LOB_SHOT = false;
                IS_FAR_HARD_SHOT = false;
                IS_MEDIUM_SHOT = false;
                IS_CLOSE_SHOT = true;
                break;

            case NO_SHOT:
                SetShooterPower(0);
                SetHoodPosition(0.30);
                break;
        }
    }

    public void ActivateShooter(boolean Shoot, boolean backShoot) {
        if (Shoot)
        {
            shooter1.setPower(shooterSpeed);
            shooter2.setPower(shooterSpeed);
        }

        else if (backShoot) {
            shooter1.setPower(-shooterSpeed * 0.15);
            shooter2.setPower(-shooterSpeed * 0.15);
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

        double error = Math.abs(rpm)-currentRPM;

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

        SetPowerPID(output * rpm/Math.abs(rpm));

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

    public void SetHoodPosition(double value)
    {
        position = value;
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