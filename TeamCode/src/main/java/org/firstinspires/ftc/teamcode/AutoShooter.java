package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class AutoShooter {
    AprilTagLimeLight limeLight = new AprilTagLimeLight();
    DcMotorEx leftShooter = null;
    DcMotorEx rightShooter = null;
    DcMotorEx turretMotor = null;
    Servo leftHood = null;
    Servo rightHood = null;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime dt = new ElapsedTime();

    public enum ShootState {
        FAR_LOB_SHOT,
        FAR_HARD_SHOT,
        MEDIUM_SHOT,
        CLOSE_SHOT,
        NO_SHOT,
    }
    double integralSum = 0.0;
    double lastError = 0.0;
    double kP = 0.02;
    double kI = 0.0001;
    double kD = 0.0015;




    double hoodPosition = 0.2;

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

    // Shooter PLUH
    final double COUNTS_PER_MOTOR_REV = 28.0;
    final double GEAR_REDUCTION = 1;
    final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;

    private AutoShooter.ShootState currentHoodState = AutoShooter.ShootState.NO_SHOT;


    enum TurretState {
        TRACKING,
        UNWINDING
    }
    TurretState currentState = TurretState.TRACKING;
    double unwindTargetAngle = 0;
    double unwindStartHeading = 0; // To track robot rotation
    SparkFunOTOS otos;

    public void init(HardwareMap hwMap) {
        otos = hwMap.get(SparkFunOTOS.class, "otos");

        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor"); // Ctrl Hub 3
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        leftShooter = hwMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hwMap.get(DcMotorEx.class, "rightShooter"); // Exp Motor Port 3
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0.01, 0, 0, 0));

        leftHood = hwMap.get(Servo.class, "leftHood"); // Exp Port 2
        rightHood = hwMap.get(Servo.class, "rightHood"); // Exp Port 3

        timer.reset();
    }

    public void update(boolean isShootButtonPressed, boolean isHardShotPressed) {

        double currentDistance = limeLight.GetDistance();

        final double CLOSE_LIMIT = 40.0;
        final double MEDIUM_LIMIT = 55.0;

        if (isShootButtonPressed) {

            if (currentDistance > 0 && currentDistance < CLOSE_LIMIT) {
                currentHoodState = AutoShooter.ShootState.CLOSE_SHOT;
            } else if (currentDistance >= CLOSE_LIMIT && currentDistance < MEDIUM_LIMIT) {
                currentHoodState = AutoShooter.ShootState.MEDIUM_SHOT;
            } else if (currentDistance >= MEDIUM_LIMIT) {

                if (isHardShotPressed) {
                    currentHoodState = AutoShooter.ShootState.FAR_HARD_SHOT;
                } else {
                    currentHoodState = AutoShooter.ShootState.FAR_LOB_SHOT;
                }
            } else {

                currentHoodState = AutoShooter.ShootState.NO_SHOT;
            }
        } else {

            currentHoodState = AutoShooter.ShootState.NO_SHOT;
        }

        updateShooter();
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
        timer.reset();

        // Maybe unwind?
        if (currentState == TurretState.UNWINDING) {
            // Field-Centric Unwinding
            double currentHeading = otos.getPosition().h;
            double headingDelta = currentHeading - unwindStartHeading;
            double dynamicTarget = unwindTargetAngle - headingDelta;

            turnToAngle(dynamicTarget);

            // We track or we UNWIND
            if (Math.abs(getTurretAngle() - dynamicTarget) < 5.0) {
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
                unwindStartHeading = otos.getPosition().h; // Capture start heading
                return;
            }
        }

        // lowkey, if it's backwards, just reverse the output!!
        turretMotor.setPower(-output);
    }
    public void turnTurretRed() {
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
        timer.reset();

        // UNWIND HIS SHIIIII
        if (currentState == TurretState.UNWINDING) {
            // Field-Centric Unwinding
            // Field-Centric Unwinding
            double currentHeading = otos.getPosition().h;
            double headingDelta = currentHeading - unwindStartHeading;
            double dynamicTarget = unwindTargetAngle - headingDelta;

            turnToAngle(dynamicTarget);

            if (Math.abs(getTurretAngle() - dynamicTarget) < 5.0) {
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
                unwindStartHeading = otos.getPosition().h; // Capture start heading
                return; 
            }
        }

        // lowkey, if it's backwards, just reverse the output!!
        turretMotor.setPower(-output);
    }

    public void setFlywheelRPM(double targetRPM) {
        double ticksPerSecond = (targetRPM / 60.0) * COUNTS_PER_WHEEL_REV;
        rightShooter.setVelocity(ticksPerSecond);
    }

    public double getFlywheelRPM() {
        return (rightShooter.getVelocity() / COUNTS_PER_WHEEL_REV) * 60.0;
    }

    public boolean isFlywheelAtSpeed(double targetRPM, double tolerance) {
        return Math.abs(getFlywheelRPM() - targetRPM) < tolerance;
    }

    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public void turnToAngle(double targetAngle) {

        // SOOOo... I just realized that using RUN_TO_POSITION with my own PID and the FTC one is going to basically ruin things.
        // So i'm basically just making my own new PID for new things.
        double currentAngle = getTurretAngle();

        double error = targetAngle - currentAngle;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double dtSeconds = timer.seconds();
        timer.reset();
        if (dtSeconds <= 0) dtSeconds = 0.001;

        integralSum += error * dtSeconds;
        double maxIntegral = 1.0; // tune as needed yuh?
        integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));

        double derivative = (error - lastError) / dtSeconds;
        lastError = error;

        double output = kP * error + kI * integralSum + kD * derivative;

        double delta = output - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            output = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = output;

        output = Math.max(-maxPower, Math.min(maxPower, output));

        turretMotor.setPower(output);
    }


    public void updateShooter() {
        switch(currentHoodState) {
            case FAR_LOB_SHOT:
                SetHoodPosition(0.40);
                // setFlywheelRPM(3150);
                break;

            case FAR_HARD_SHOT:
                SetHoodPosition(0.2);
                // setFlywheelRPM(3100);
                break;

            case MEDIUM_SHOT:
                SetHoodPosition(0.30);
                // setFlywheelRPM(3000);
                break;

            case CLOSE_SHOT:
                SetHoodPosition(0.40);
                // setFlywheelRPM(2700);
                break;

            case NO_SHOT:
                SetHoodPosition(0.30);
                // setFlywheelRPM(0);
                break;
        }
    }

    public void SetHoodPosition(double value)
    {
        double position = value;
        leftHood.setPosition(position);
        rightHood.setPosition(position);
    }
}


