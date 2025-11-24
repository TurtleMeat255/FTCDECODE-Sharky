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
    AprilTagLimeLight limelight = new AprilTagLimeLight();

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

    int rpmShiftAmount = 100;
    int firingRPM = 1000;

    boolean canShiftSpeedUp = true;
    boolean canShiftSpeedDown = true;

    ElapsedTime pushUpTimer = new ElapsedTime();
    double pushUpMaxTime = 0.6;
    boolean readyToShoot = false;

    boolean autoShoot = true;

    int rpmIncrement = 200;

    boolean autoTargeting = false;
    boolean rapidFiring = false;

    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        spinindexer.init(hardwareMap);
        shooter.init(hardwareMap);
        colorSensor.init(hardwareMap);
        limelight.init(hardwareMap);
        nudger = hardwareMap.get(Servo.class, "nudger");
        nudger.setDirection(Servo.Direction.REVERSE);

        intake1 = hardwareMap.get(DcMotor.class, "intake");

        shooter.SetShooterPower(1);

        // I wanna know if initialization is complete.
        telemetry.addData("Status", "Robot is Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {

            // Gamepad 1
            double xPos = gamepad1.left_stick_x;
            double yPos = -gamepad1.left_stick_y;
            double rot = gamepad1.right_stick_x;

            boolean shiftRpmDown = gamepad1.left_bumper;
            boolean shiftRpmUp = gamepad1.right_bumper;

            boolean activateAutoTarget = gamepad1.dpad_right;
            boolean deactivateAutoTarget = gamepad1.dpad_down;

            boolean reverseIntakeShooter = gamepad1.dpad_left;

            boolean resetButton = gamepad1.a;

            boolean reverseIntakeFront = gamepad1.b;

            // Gamepad 2

                //Spindexer

            boolean spindexSixth = gamepad2.dpad_left;
            boolean spindexThird = gamepad2.dpad_up;
            double spindexerManual = gamepad2.right_stick_x;

                // Shooter

            boolean activateAutoShoot = gamepad2.dpad_right;
            boolean activateManualShoot = gamepad2.dpad_down;

            // You will also need these for the color selection logic
            boolean shootAPurple = gamepad2.x;
            boolean shootAGreen = gamepad2.b;
            boolean rapidFire = gamepad2.a;

                // hood

            boolean hoodUp = gamepad2.right_bumper;
            boolean hoodDown = gamepad2.left_bumper;

                // Intake

            double intakePressed = gamepad2.right_trigger;
            double shooterPressed = gamepad2.left_trigger;

                // Limelight

            double distance = limelight.GetDistance();

                // Nudger

            boolean raiseNudger = gamepad2.y;

            if (autoTargeting)
            {
                drivetrain.RobotCentricAlign(xPos,yPos,limelight.GetTX());
            }
            else
            {
                drivetrain.RobotCentric(xPos,yPos,rot);
            }


            if (raiseNudger)
            {
                pushUp = true;
                pushUpTimer.reset();
            }

            if (intakePressed > 0.3)
            {
                intake1.setPower(-0.6);
            }
            else
            {
                intake1.setPower(0);
            }

            if (reverseIntakeFront)
            {
                intake1.setPower(0.6);
            }

            // Reverse intake code

            if (reverseIntakeShooter && !rapidFiring) {
                shooter.ActivateShooter(false, true);
            }

            if (activateAutoShoot)
            {
                autoShoot = true;
            }
            else if (activateManualShoot)
            {
                autoShoot = false;
            }

            if (activateAutoTarget)
            {
                autoTargeting = true;
            }
            else if (deactivateAutoTarget)
            {
                autoTargeting = false;
            }

            if (autoShoot)
            {
                if (distance > 66)
                {
                    firingRPM = 3300;
                    shooter.SetHoodPosition(0.8);
                }
                else if (distance > 45)
                {
                    firingRPM = 3000;
                    shooter.SetHoodPosition(0.45);
                }
                else
                {
                    firingRPM = 2700;
                    shooter.SetHoodPosition(0.8);
                }
            }
            else
            {
                if (shiftRpmUp && canShiftSpeedUp)
                {
                    if (firingRPM + rpmIncrement < 8400)
                    {
                        canShiftSpeedUp = false;
                        firingRPM += rpmIncrement;
                    }
                }
                else if (!shiftRpmUp)
                {
                    canShiftSpeedUp = true;
                }

                if (shiftRpmDown && canShiftSpeedDown)
                {
                    if (firingRPM - rpmIncrement > 1000)
                    {
                        canShiftSpeedDown = false;
                        firingRPM -= rpmIncrement;
                    }
                }
                else if (!shiftRpmDown)
                {
                    canShiftSpeedDown = true;
                }

                shooter.HoodStuff(hoodUp, hoodDown);
            }

            if (shooterPressed > 0.3) {
                shooter.SetShooterRPM(firingRPM);
            }
            else
            {
                if (!rapidFiring && ! coloringRn)
                {
                    shooter.SetShooterRPM(0);
                }
            }

            if (Math.abs(spindexerManual) > 0.3)
            {
                inputAngle += spindexerManual/Math.abs(spindexerManual) * 120 * dt.seconds();
            }

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
                        readyToShoot = true;
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

            if (!pushUp && !rapidFiring)
            {
                spinindexer.PID(inputAngle);
            }

            if (rapidFire)
            {
                if (inputAngle % 120 != 0)
                {
                    inputAngle += 60;
                }
                spinindexer.PID(inputAngle);
                rapidFiring = true;

                if (spinindexer.withinRange(inputAngle))
                {
                    spinindexer.BallKD(true);
                    for (int i = 0; i < 3; i ++)
                    {
                        if (i > 0)
                        {
                            inputAngle += 120;
                        }
                        while (!shooter.RPMCorrect(firingRPM) || !spinindexer.withinRange(inputAngle))
                        {
                            shooter.SetShooterRPM(firingRPM);
                            spinindexer.PID(inputAngle);

                            if (autoShoot)
                            {
                                if (distance > 66)
                                {
                                    firingRPM = 3300;
                                    shooter.SetHoodPosition(0.8);
                                }
                                else if (distance > 45)
                                {
                                    firingRPM = 3000;
                                    shooter.SetHoodPosition(0.45);
                                }
                                else
                                {
                                    firingRPM = 2700;
                                    shooter.SetHoodPosition(0.8);
                                }
                            }
                        }

                        spinindexer.nudging(true);
                        sleep(300);
                        spinindexer.nudging(false);
                        sleep(200);
                    }
                    rapidFiring = false;
                    spinindexer.BallKD(false);
                }
            }

            if (!rapidFiring)
            {
                spinindexer.RapidFiring(false);
            }

            dt.reset();

            if (readyToShoot)
            {
                shooter.SetShooterRPM(firingRPM);

                if (shooter.RPMCorrect(firingRPM))
                {
                    pushUp = true;
                }
            }

            if (pushUpTimer.seconds() > pushUpMaxTime/2)
            {
                pushUp = false;
            }

            if (pushUpTimer.seconds() > pushUpMaxTime)
            {
                readyToShoot = false;
            }

            spinindexer.nudging(pushUp);

            telemetry.addData("nudger position", nudger.getPosition());
            telemetry.addData("spindexer pos", spinindexer.GetCurrentPosition());
            telemetry.addData("input angle:", inputAngle);
            telemetry.addData("hood position", shooter.GetHoodPosition());
            telemetry.addData("color sensor detection", colorSensor.GetDetectedColor());
            telemetry.addData("current shooter tgt rpm", firingRPM);
            telemetry.addData("current shooter actual rpm", shooter.GetShooterRPM());
            telemetry.addData("shooter at rpm", shooter.RPMCorrect(firingRPM));
            telemetry.addData("distance", limelight.GetDistance());
            telemetry.addData("GetPositionalCorrectness", spinindexer.GetPositionalCorrectness());
            telemetry.addData("GetKP", spinindexer.GetKP());
            telemetry.update();
        }
    }
}

// added comment to annoy natey boy