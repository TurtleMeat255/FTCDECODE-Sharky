//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//public class FTCDriveTrain extends SubsystemBase
//{
//    double moveSpeed = 1.0;
//    private final Motor frontLeftMotor;
//    private final Motor backLeftMotor;
//    private final Motor frontRightMotor;
//    private final Motor backRightMotor;
//
//    SparkFunOTOS otos;
//
//    double kp = 0.025;
//    double kd = 0;
//    ElapsedTime dt = new ElapsedTime();
//    double lastError = 0;
//    double errorCrunchConstant = 3;
//
//    public FTCDriveTrain(HardwareMap hwMap) {
//        // Declare our motors
//        // Make sure your ID's match your configuration
//        frontLeftMotor = new Motor(hwMap, "frontLeftMotor"); // Port 0
//        backLeftMotor = new Motor(hwMap, "backLeftMotor"); // Port 1
//        frontRightMotor = new Motor(hwMap, "frontRightMotor"); // Port 2
//        backRightMotor = new Motor(hwMap, "backRightMotor"); // Port 3
//
//        frontLeftMotor.setRunMode(Motor.RunMode.RawPower);
//        backLeftMotor.setRunMode(Motor.RunMode.RawPower);
//        frontRightMotor.setRunMode(Motor.RunMode.RawPower);
//        backRightMotor.setRunMode(Motor.RunMode.RawPower);
//
//
//        frontLeftMotor.setInverted(true);
//        backLeftMotor.setInverted(true);
//        frontRightMotor.setInverted(false);
//        backRightMotor.setInverted(false);
//
//        otos = hwMap.get(SparkFunOTOS.class, "otos");
//        otos.setOffset(new SparkFunOTOS.Pose2D(1.5,-7,Math.PI));
//        otos.calibrateImu();
//    }
//
//    public void SwapToBrakeMode(boolean input)
//    {
//        if (input)
//        {
//            frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//            backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//            frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//            backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        }
//        else
//        {
//            frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//            backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//            frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//            backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        }
//    }
//
//    public void RobotCentric(double x, double y, double rx)
//    {
//        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//        frontLeftMotor.set(frontLeftPower * moveSpeed);
//        backLeftMotor.set(backLeftPower * moveSpeed);
//        frontRightMotor.set(frontRightPower * moveSpeed);
//        backRightMotor.set(backRightPower * moveSpeed);
//    }
//
//    public void RobotCentricAlign(double x, double y, double tx)
//    {
//        double rx = PID(tx);
//
//        RobotCentric(x,y,rx);
//    }
//
//    private double PID(double tx)
//    {
//        double error = (tx-0)/errorCrunchConstant;
//
//        double derivative = (error - lastError)/dt.seconds();
//
//        double output = error * kp + derivative * kd;
//        output = Range.clip(output, -1, 1);
//
//        lastError = error;
//        dt.reset();
//
//        return output;
//    }
//
//    private double angleWrap(double rad)
//    {
//        while (rad > Math.PI)
//        {
//            rad -= 2 * Math.PI;
//        }
//        while (rad < -Math.PI)
//        {
//            rad += 2 * Math.PI;
//        }
//        return rad;
//    }
//
//    public void Translate(double x, double y, double rx, boolean reset)
//    {
//        SparkFunOTOS.Pose2D currentPose = otos.getPosition();
//
//        currentPose.h = -angleWrap(currentPose.h * Math.PI/180);
//
//        if (reset) {
//            otos.setPosition(new SparkFunOTOS.Pose2D(currentPose.x, currentPose.y, 0));
//        }
//
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = x * Math.cos(currentPose.h) - y * Math.sin(currentPose.h);
//        double rotY = x * Math.sin(currentPose.h) + y * Math.cos(currentPose.h);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        double frontLeftPower = (rotY + rotX + rx) / denominator;
//        double backLeftPower = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower = (rotY + rotX - rx) / denominator;
//
//        frontLeftMotor.set(frontLeftPower * moveSpeed);
//        backLeftMotor.set(backLeftPower * moveSpeed);
//        frontRightMotor.set(frontRightPower * moveSpeed);
//        backRightMotor.set(backRightPower * moveSpeed);
//    }
//
//    public void FieldCentricAlign(double targetPowerX, double targetPowerY, double tx)
//    {
//        double rx = PID(tx);
//
//        FieldOrientedTranslate(targetPowerX,targetPowerY,rx,false);
//    }
//
//    // taken from Sharkans INTO THE DEEP codebase
//    public void FieldOrientedTranslate(double targetPowerX, double targetPowerY, double rotation, boolean reset)
//    {
//        SparkFunOTOS.Pose2D currentPose = otos.getPosition();
//
//        currentPose.h = Math.toDegrees(angleWrap(Math.toRadians(currentPose.h)));
//
//        if (reset) {
//            otos.setPosition(new SparkFunOTOS.Pose2D(currentPose.x, currentPose.y, 0));
//        }
//
//        double yaw = currentPose.h;
//
//        double stickRotation = 0;
//        if (targetPowerY > 0 && targetPowerX < 0) //quad2
//        {
//            stickRotation = (Math.atan2(Math.abs(targetPowerY),Math.abs(targetPowerX)) + Math.PI/2) * 180/Math.PI;
//        }
//        else if (targetPowerY < 0 && targetPowerX < 0) //quad3
//        {
//            stickRotation = (Math.atan2(Math.abs(targetPowerY),Math.abs(targetPowerX)) + Math.PI) * 180/Math.PI;
//        }
//        else //quad1 and quad4
//        {
//            stickRotation = Math.atan2(targetPowerY,targetPowerX) * 180/Math.PI;
//        }
//
//        // angle of imu yaw supplemented by the stick's rotation, determined by atan
//        double theta = (360-yaw) + stickRotation;
//        double power = Math.hypot(targetPowerX,targetPowerY); //get hypotenuse of x and y tgt, getting the power
//
//        // if at max power diag, limit to magnitude of 1
//        // with the normalizing code, the diag movement had a bug where max power (being magnitude sqrt(2))--
//        // --would cause wheels to flip polarity
//        // to counteract this the power is limited to a proper magnitude
//        if (power > 1)
//        {
//            power = 1;
//        }
//        else if (power < -1)
//        {
//            power = -1;
//        }
//
//        //get the sin and cos of theta
//        //math.pi/4 represents 45 degrees, accounting for angular offset of mechanum
//        double sin = Math.sin((theta * (Math.PI/180)) - (Math.PI/4));
//        double cos = Math.cos((theta * (Math.PI/180)) - (Math.PI/4));
//        //max of sin and cos, used to normalize the values for maximum efficiency
//        double maxSinCos = Math.max(Math.abs(sin),Math.abs(cos));
//
//        //same sign flip is to account for the inability of atan2, it typically only works for quadrants 1 and 4
//        //by flipping the polarity when x < 0, we can use atan for every quadrant
//        double flPower = 0;
//        double frPower = 0;
//        double blPower = 0;
//        double brPower = 0;
//
////        rotation *= -1;
//
//        flPower = power * cos/maxSinCos+rotation;
//        frPower = power * sin/maxSinCos-rotation;
//        blPower = power * sin/maxSinCos+rotation;
//        brPower = power * cos/maxSinCos-rotation;
//
////        double frontMax = Math.max(Math.abs(flPower),Math.abs(frPower));
////        double backMax = Math.max(Math.abs(blPower),Math.abs(brPower));
//
//        //another normalization
//        if ((power + Math.abs(rotation)) > 1)
//        {
//            flPower /= power + Math.abs(rotation);
//            frPower /= power - Math.abs(rotation);
//            blPower /= power + Math.abs(rotation);
//            brPower /= power - Math.abs(rotation);
//        }
//
//        if (frontLeftMotor != null && frontRightMotor != null && backLeftMotor != null && backRightMotor != null)
//        {
//            frontLeftMotor.set(flPower * moveSpeed);
//            frontRightMotor.set(frPower * moveSpeed);
//            backLeftMotor.set(blPower * moveSpeed);
//            backRightMotor.set(brPower * moveSpeed);
//        }
//    }
//
//    public double GetRotation()
//    {
//        return otos.getPosition().h;
//    }
//}