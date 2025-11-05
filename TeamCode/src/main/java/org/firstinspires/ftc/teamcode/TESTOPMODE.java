package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TESTOPMODE extends LinearOpMode{
//    FTCDriveTrain drivetrain = new FTCDriveTrain();
//    Spinindexer spinindexer = new Spinindexer();
    Shooter shooter = new Shooter();
    IntakeCode intake = new IntakeCode();

    Servo nudger = null;
    DcMotor spindexer = null;

    DcMotor intake1 = null;

    public double inputAngle = 0;
    public boolean intakeAligned = true;
    boolean lastShooterLeft = false;
    boolean lastShooterRight = false;
    boolean lastIntakeLeft = false;
    boolean lastIntakeRight = false;
    boolean coloringRn = false;
    int colorIWant = 0; // 0 = no color, 1 = green, 2 = purple
    int ticker = 0;

    private double encoderResolution = 145.6f;
    boolean canMove = true;
    boolean canSwapMode = true;

    @Override
    public void runOpMode() {
//        drivetrain.init(hardwareMap);
//        spinindexer.init(hardwareMap);
        shooter.init(hardwareMap);
        nudger = hardwareMap.get(Servo.class, "nudger");
        nudger.setDirection(Servo.Direction.REVERSE);

        spindexer = hardwareMap.get(DcMotor.class, "spindexer");

        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        intake = hardwareMap.get(DcMotor.class, "intake");

        // I wanna know if initialization is complete.
        telemetry.addData("Status", "Robot is Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {

            // Intake stuff ahhh code.
            if (gamepad1.a) {
                intake.ActivateIntake(true);
            }
            else {
                intake.ActivateIntake(false);
            }

            // Shoot stuff ahhh code.
            if (gamepad2.a) {
                shooter.ActivateShooter(true);
            }
            else if (gamepad1.a)
            {
                shooter.FireAtRPM(6000);
            }
            else {
                shooter.ActivateShooter(false);
            }

            if (gamepad2.y){
                nudger.setPosition(0.25);
            }
            else {
                nudger.setPosition(0);
            }

            telemetry.addData("nudger position", nudger.getPosition());
            telemetry.update();

            telemetry.addData("nudger position", nudger.getPosition());
            telemetry.update();

//            spindexer.setPower(0.1);
//            if (gamepad2.x && canMove && canSwapMode) {
//                spindexer.setTargetPosition(spindexer.getCurrentPosition() + (int)encoderResolution / 3);
//                canMove = false;
//            }
//            else if (!gamepad2.x)
//            {
//                canMove = true;
//            }

            if (gamepad2.b && canSwapMode) {
                spindexer.setTargetPosition(spindexer.getCurrentPosition() + (int)encoderResolution / 6);
                canSwapMode = false;
            }
            else if (!gamepad2.b)
            {
                canSwapMode = true;
            }

//            intake.setPower(0.3);

//            if (!coloringRn && spinindexer.isItDown()) {
//                if (shootAPurple && !intakeAligned && spinindexer.withinRange(inputAngle)) {
//                    colorIWant = 2;
//                    coloringRn = true;
//                    ticker = 0;
//                }
//                if (shootAGreen && !intakeAligned && spinindexer.withinRange(inputAngle)) {
//                    colorIWant = 1;
//                    coloringRn = true;
//                    ticker = 0;
//                }
//            }
//            if (!coloringRn) {
//                if (spinindexer.isItDown()) {
//                    if (shooterLeft && !lastShooterLeft) {
//                        if (intakeAligned) {
//                            inputAngle += 60;
//                            intakeAligned = !intakeAligned;
//                        } else {
//                            inputAngle += 120;
//                        }
//                    }
//                    if (shooterRight && !lastShooterRight) {
//                        if (intakeAligned) {
//                            inputAngle -= 60;
//                            intakeAligned = !intakeAligned;
//                        } else {
//                            inputAngle -= 120;
//                        }
//                    }
//                    if (intakeLeft && !lastIntakeLeft) {
//                        if (intakeAligned) {
//                            inputAngle += 120;
//                        } else {
//                            inputAngle += 60;
//                            intakeAligned = !intakeAligned;
//                        }
//                    }
//                    if (intakeRight && !lastIntakeRight) {
//                        if (intakeAligned) {
//                            inputAngle -= 120;
//                        } else {
//                            inputAngle -= 60;
//                            intakeAligned = !intakeAligned;
//                        }
//                    }
//                }
//            }
//            if (coloringRn) {
//                if (spinindexer.withinRange(inputAngle)) {
//                    if (spinindexer.colorDetected() == colorIWant) {
//                            pushUp = true;
//                            coloringRn = false;
//                    } else {
//                        pushUp = false;
//                        ticker += 1;
//                        if (ticker == 2) {
//                            coloringRn = false;
//                        } else {
//                            inputAngle += 120;
//                        }
//                    }
//                }
//            }



//            lastIntakeRight = intakeRight;
//            lastShooterLeft = shooterLeft;
//            lastIntakeLeft = intakeLeft;
//            lastShooterRight = shooterRight;
//            spinindexer.PID(inputAngle);

//            if (!spinindexer.withinRange(inputAngle)) {
//                pushUp = false;
//            }
//            if (intakeAligned) {
//                pushUp = false;
//            }
//            spinindexer.nudging(pushUp, pushDown);
        }
    }
}