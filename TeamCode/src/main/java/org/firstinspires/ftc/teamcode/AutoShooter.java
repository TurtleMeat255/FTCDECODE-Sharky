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
    Servo rightHood = null;

    double lastRPMERROR = 0;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime dt = new ElapsedTime();
    ElapsedTime shootdt = new ElapsedTime();
    double integralSum = 0.0;
    double lastError = 0.0;
    double kP = 0.02;
    double kI = 0.0001;
    double kD = 0.0015;

    // make my feedforward!
    double kV = 1.0 / 6000.0;
    double kS = 0.0;   // Usually smol
    double kA = 0.0;

    double shootkP = 0.02;
    double shootkI = 0.0001;
    double shootkD = 0.0015;
    double shootintegralSum = 0.0;
    double shootlastError = 0.0;


    boolean randomVariable = false;

    double filterStrength = 0.7; // we can tune this to make it smooth & slow, or kinda FREAKY & fast.
    double deadband = 0.67; // This just says if the error is less than this. Just don't bother moving. It's going to save power and still maintain accuracy.
    double maxPower = 0.67;
    double maxDeltaPower = 0.03; // Basically prevents the turret from imploding into a black hole. (See me for more info).

    double lastFilteredTx = 0;
    double lastOutput = 0;

    // Safety Limits - PLACEHOLDER VALUES, MUST BE TUNED
    final int TURRET_MAX_TICKS = 1000;
    final int TURRET_MIN_TICKS = -1000;

    final double TICKS_PER_REV = 537.7; 
    final double GEAR_RATIO = 5.0;    
    final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
    
    enum TurretState {
        TRACKING,
        UNWINDING
    }
    TurretState currentState = TurretState.TRACKING;
    double unwindTargetAngle = 0;

    public void init(HardwareMap hwMap) {

        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftShooter = hwMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hwMap.get(DcMotorEx.class, "rightShooter");

        leftHood = hwMap.get(Servo.class, "leftHood");
        rightHood = hwMap.get(Servo.class, "rightHood");

        timer.reset();
    }

    public void turnTurretBlue() {
        double tx = limeLight.GetTX();

        // should make it just stop if it sees wrong april tag
        if (limeLight.GetLimelightId() != 20) { // Lowkey not sure what this does for sorting APRILTAGLOL!!
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

        integralSum += error * dt;

        double output = kP * error + kI * integralSum + kD * derivative;

        double delta = output - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            output = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = output;

        // Maybe unwind?
        if (currentState == TurretState.UNWINDING) {
            turnToAngle(unwindTargetAngle);
            // We track or we UNWIND
            if (Math.abs(getTurretAngle() - unwindTargetAngle) < 5.0) {
                currentState = TurretState.TRACKING;
                timer.reset();
            }
            return;
        }

        output = Math.max(-maxPower, Math.min(maxPower, output));

        // Onn skibidi?
        int currentPos = turretMotor.getCurrentPosition();
        boolean hitMax = (currentPos > TURRET_MAX_TICKS && -output > 0);
        boolean hitMin = (currentPos < TURRET_MIN_TICKS && -output < 0);

        if (hitMax || hitMin) {
            output = 0;
            
            // Maybe unwind again?
            double currentAngle = getTurretAngle();
            double alternativeAngle = hitMax ? (currentAngle - 360) : (currentAngle + 360);
            
            double altTicks = alternativeAngle * TICKS_PER_DEGREE;
            if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                currentState = TurretState.UNWINDING;
                unwindTargetAngle = alternativeAngle;
                return; 
            }
        }

        // lowkey, if it's backwards, just reverse the output!!
        turretMotor.setPower(-output);
    }
    public void turnTurretRED() {
        double tx = limeLight.GetTX();

        // should make it just stop if it sees wrong april tag
        if (limeLight.GetLimelightId() != 24) { // Lowkey not sure what this does for sorting APRILTAGLOL!!
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

        integralSum += error * dt;

        double output = kP * error + kI * integralSum + kD * derivative;

        double delta = output - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            output = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = output;

        // UNWIND HIS SHIIIII
        if (currentState == TurretState.UNWINDING) {
            turnToAngle(unwindTargetAngle);
            if (Math.abs(getTurretAngle() - unwindTargetAngle) < 5.0) {
                currentState = TurretState.TRACKING;
                timer.reset();
            }
            return;
        }

        // Normul jrrrking logic
        output = Math.max(-maxPower, Math.min(maxPower, output));

        // Soft or HARD limit check ya know?
        int currentPos = turretMotor.getCurrentPosition();
        boolean hitMax = (currentPos > TURRET_MAX_TICKS && -output > 0);
        boolean hitMin = (currentPos < TURRET_MIN_TICKS && -output < 0);

        if (hitMax || hitMin) {
            output = 0;
            
            // Ong the other side is reachable!!
            double currentAngle = getTurretAngle();
            double alternativeAngle = hitMax ? (currentAngle - 360) : (currentAngle + 360);
            
            // Who decided that!?
            double altTicks = alternativeAngle * TICKS_PER_DEGREE;
            if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                currentState = TurretState.UNWINDING;
                unwindTargetAngle = alternativeAngle;
                return; 
            }
        }

        // lowkey, if it's backwards, just reverse the output!!
        turretMotor.setPower(-output);
    }
    public void flyWheelRPM(double targetRPM) {
        double currentRPM = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2.0;

        double dtSeconds = shootdt.seconds();
        shootdt.reset();
        if (dtSeconds <= 0) dtSeconds = 0.001;

        double shootError = targetRPM - currentRPM;

        shootintegralSum += shootError * dtSeconds;
        double maxIntegral = 1000; // tune this onskibidi
        if (shootintegralSum > maxIntegral) shootintegralSum = maxIntegral;
        if (shootintegralSum < -maxIntegral) shootintegralSum = -maxIntegral;

        // Derivative
        double derivative = (shootError - shootlastError) / dtSeconds;
        shootlastError = shootError;

        double pid = shootkP * shootError + shootkI * shootintegralSum + shootkD * derivative;

        // Please SPEED I NEED THIS!!
        // My forward is kinda feedless!!!
        double feedForward = kS + kV * targetRPM;

        double output = pid + feedForward;


        output = Math.max(0, Math.min(maxPower, output));

        double delta = output - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            output = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = output;

        /*
        Have you ever heard the tragedy of Darth Plagueis the rizzler?
        No,
        I thought so, it's not a story the Betas' would tell you.
        He was a mog lord of the sigmas.
        It is said that he could influence the betas' to... show their gyats...
        He... Could actually rizz up Livvy Dune?!?
        He was so powerful that... the only thing he was afraid of was... losing his rizz.
        Which in the end he did...
        Where could you learn this power?
        Not. From. A Beta...
         */

        leftShooter.setPower(output);
        rightShooter.setPower(output);
    }

    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public void turnToAngle(double targetAngle) {
        int targetTicks = (int) (targetAngle * TICKS_PER_DEGREE);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretMotor.setPower(0.67);
    }
}


