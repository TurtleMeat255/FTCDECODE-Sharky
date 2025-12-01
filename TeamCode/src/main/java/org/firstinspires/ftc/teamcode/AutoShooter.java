package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoShooter {
    AprilTagLimeLight limeLight = new AprilTagLimeLight();
    DcMotorEx leftShooter = null;
    DcMotorEx rightShooter = null;
    DcMotorEx turretMotor = null;
    Servo leftHood = null;
    Servo righthood = null;

    double lastRPMERROR = 0;
    ElapsedTime timer = new ElapsedTime();
    double integralSum = 0;
    double lastError = 0;
    double kP = 0.02;
    double kI = 0.0001;
    double kD = 0.0015;

    boolean randomVariable = false;

    double filterStrength = 0.7; // we can tune this to make it smooth & slow, or kinda FREAKY & fast.
    double deadband = 0.67; // This just says if the error is less than this. Just don't bother moving. It's going to save power and still maintain accuracy.
    double maxPower = 0.67;
    double maxDeltaPower = 0.03; // Basically prevents the turret from imploding into a black hole. (See me for more info).

    double lastFilteredTx = 0;
    double lastOutput = 0;

    public void init(HardwareMap hwMap) {

        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");

        leftShooter = hwMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hwMap.get(DcMotorEx.class, "rightShooter");

        leftHood = hwMap.get(Servo.class, "leftHood");
        righthood = hwMap.get(Servo.class, "righthood");

        timer.reset();
    }

    public void turnTurretBLUE() {
        double tx = limeLight.GetTX();

        if (limeLight.GetLimelightId() == 20 && randomVariable) {
            turretMotor.setPower(0);
            integralSum = 0;
            lastError = 0;
            timer.reset();
            return;
        }

        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 0.001;

        double filteredTx = filterStrength * lastFilteredTx + (1 - filterStrength) * tx;
        lastFilteredTx = filteredTx; // lowkey, all the stupid spinners I see are super smooth. I want ours to be too!!!! :)
        // TS filtered TX basically just micro correct how the PID is going to oscilate between the april tag since we MOVE!!


        if (Math.abs(filteredTx) < deadband) {
            turretMotor.setPower(0);
            lastOutput = 0;
            integralSum = 0;
            return;
        }

        double error = filteredTx;

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kD * derivative;

        double delta = output - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            output = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = output;

        output = Math.max(-maxPower, Math.min(maxPower, output));

        // lowkey, if it's backwards, just reverse the output!!
        turretMotor.setPower(-output);
    }
}


