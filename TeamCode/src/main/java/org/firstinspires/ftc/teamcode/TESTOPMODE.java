package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TESTOPMODE extends LinearOpMode{
    FTCDriveTrain drivetrain = new FTCDriveTrain();
    Spinindexer spinindexer = new Spinindexer();
    Shooter shooter = new Shooter();
    IntakeCode intake = new IntakeCode();
    ColorSensor colorSensor = new ColorSensor();

    Servo nudger = null;

    DcMotor intake1 = null;

    ElapsedTime dt = new ElapsedTime();

    public double inputAngle = 0;
    public boolean intakeAligned = false;
    boolean lastShooterLeft = false;
    boolean lastShooterRight = false;
    boolean lastIntakeLeft = false;
    boolean lastIntakeRight = false;
    boolean coloringRn = false;
    ColorSensor.DetectedColor colorIWant = ColorSensor.DetectedColor.UNKNOWN; // 0 = no color, 1 = green, 2 = purple
    int ticker = 0;
 // And these for the nudger
    boolean pushUp; // This will be set by your logic
    boolean pushDown;

    boolean canMove = true;
    boolean canSwapMode = true;
    boolean swapping = false;
    double servoPos = 0;

    ElapsedTime pushUpTimer = new ElapsedTime();
    double pushUpMaxTime = 0.6;

    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
//        intake.init(hardwareMap);
        spinindexer.init(hardwareMap);
        shooter.init(hardwareMap);
        colorSensor.init(hardwareMap);
        nudger = hardwareMap.get(Servo.class, "nudger");
        nudger.setDirection(Servo.Direction.REVERSE);

        intake1 = hardwareMap.get(DcMotor.class, "intake");

        shooter.SetShooterPower(0.2);

        // I wanna know if initialization is complete.
        telemetry.addData("Status", "Robot is Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            double xPos = gamepad1.left_stick_x;
            double yPos = -gamepad1.left_stick_y;
            double rot = gamepad1.right_stick_x;

            boolean resetButton = gamepad1.a;

            boolean spindexSixth = gamepad2.dpad_left;
            boolean spindexThird = gamepad2.dpad_up;

            double spindexerManual = gamepad2.right_stick_x;

            // You will also need these for the color selection logic
            boolean shootAPurple = gamepad2.x;
            boolean shootAGreen = gamepad2.b;
            boolean activateNudger = gamepad2.y;

            boolean hoodUp = gamepad2.right_bumper;
            boolean hoodDown = gamepad2.left_bumper;

            double intakePressed = gamepad2.right_trigger;
            double shooterPressed = gamepad2.left_trigger;

            drivetrain.RobotCentric(xPos,yPos,rot);

            if (intakePressed > 0.3)
            {
                intake1.setPower(-0.6);
            }
            else
            {
                intake1.setPower(0);
            }

            if (gamepad1.b)
            {
                intake1.setPower(0.6);
            }

            if (shooterPressed > 0.3) {
                shooter.ActivateShooter(true, false);
            }
            else
            {
                shooter.ActivateShooter(false, false);
            }

            if (Math.abs(spindexerManual) > 0.3)
            {
                inputAngle += spindexerManual/Math.abs(spindexerManual) * 120 * dt.seconds();
            }

            shooter.HoodStuff(hoodUp,hoodDown);

            if (spindexThird && canMove && canSwapMode) {
                inputAngle += 120;
                canMove = false;
            }
            else if (!spindexThird)
            {
                canMove = true;
            }

            if (spindexSixth && canSwapMode) {
                inputAngle += 60;
                intakeAligned = !intakeAligned;
                canSwapMode = false;
            }
            else if (!spindexSixth)
            {
                canSwapMode = true;
            }

            if (!coloringRn && spinindexer.isItDown()) {
                if (shootAPurple && !intakeAligned && spinindexer.withinRange(inputAngle)) {
                    colorIWant = ColorSensor.DetectedColor.PURPLE;
                    coloringRn = true;
                    ticker = 0;
                }
                if (shootAGreen && !intakeAligned && spinindexer.withinRange(inputAngle)) {
                    colorIWant = ColorSensor.DetectedColor.GREEN;
                    coloringRn = true;
                    ticker = 0;
                }
            }

            if (coloringRn) {
                if (spinindexer.withinRange(inputAngle)) {
                    if (colorSensor.GetDetectedColor() == colorIWant) {
                        pushUp = true;
                        coloringRn = false;
                        pushUpTimer.reset();
                    } else {
                        pushUp = false;
                        ticker += 1;
                        if (ticker == 2) {
                            coloringRn = false;
                        } else {
                            inputAngle += 120;
                        }
                    }
                }
            }

            spinindexer.PID(inputAngle);
            spinindexer.nudging(pushUp);

            dt.reset();

            if (pushUpTimer.seconds() > pushUpMaxTime)
            {
                pushUp = false;
            }

            telemetry.addData("nudger position", nudger.getPosition());
            telemetry.addData("spindexer pos", spinindexer.GetCurrentPosition());
            telemetry.addData("input angle:", inputAngle);
            telemetry.addData("hood position", shooter.GetHoodPosition());
            telemetry.addData("color sensor detection", colorSensor.GetDetectedColor());
            telemetry.update();
        }
    }
}