package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class DriverOperator extends LinearOpMode{
    FTCDriveTrain drivetrain = new FTCDriveTrain();
    Spinindexer spinindexer = new Spinindexer();
    Shooter shooter = new Shooter();
    public double inputAngle = 0;
    public boolean intakeAligned = true;
    boolean lastShooterLeft = false;
    boolean lastShooterRight = false;
    boolean lastIntakeLeft = false;
    boolean lastIntakeRight = false;
    boolean coloringRn = false;
    int colorIWant = 0; // 0 = no color, 1 = green, 2 = purple
    int ticker = 0;

    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
        spinindexer.init(hardwareMap);
        shooter.init(hardwareMap);

        // I wanna know if initialization is complete.
        telemetry.addData("Status", "Robot is Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            // Driver controls
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean options = gamepad1.options;

            // Shooter controls
            boolean shooterLeft = gamepad2.x;
            boolean shooterRight = gamepad2.y;
            boolean intakeLeft = gamepad2.a;
            boolean intakeRight = gamepad2.b;
            boolean pushUp = gamepad2.dpad_up;
            boolean pushDown = gamepad2.dpad_down;
            boolean Hoodup = gamepad2.right_bumper;
            boolean Hooddown = gamepad2.left_bumper;
            boolean shootAGreen = gamepad2.dpad_left;
            boolean shootAPurple = gamepad2.dpad_right;

            // Drivetrain
            drivetrain.Translate(x,y,rx,options);
            shooter.HoodStuff(Hoodup, Hooddown);

            // Shoot stuff ahhh code.
            if (gamepad2.a) {
                shooter.ShootStuff(true);
            }
            else {
                shooter.ShootStuff(false);
            }
            if (!coloringRn && spinindexer.isItDown()) {
                if (shootAPurple && !intakeAligned && spinindexer.withinRange(inputAngle)) {
                    colorIWant = 2;
                    coloringRn = true;
                    ticker = 0;
                }
                if (shootAGreen && !intakeAligned && spinindexer.withinRange(inputAngle)) {
                    colorIWant = 1;
                    coloringRn = true;
                    ticker = 0;
                }
            }
            if (!coloringRn) {
                if (spinindexer.isItDown()) {
                    if (shooterLeft && !lastShooterLeft) {
                        if (intakeAligned) {
                            inputAngle += 60;
                            intakeAligned = !intakeAligned;
                        } else {
                            inputAngle += 120;
                        }
                    }
                    if (shooterRight && !lastShooterRight) {
                        if (intakeAligned) {
                            inputAngle -= 60;
                            intakeAligned = !intakeAligned;
                        } else {
                            inputAngle -= 120;
                        }
                    }
                    if (intakeLeft && !lastIntakeLeft) {
                        if (intakeAligned) {
                            inputAngle += 120;
                        } else {
                            inputAngle += 60;
                            intakeAligned = !intakeAligned;
                        }
                    }
                    if (intakeRight && !lastIntakeRight) {
                        if (intakeAligned) {
                            inputAngle -= 120;
                        } else {
                            inputAngle -= 60;
                            intakeAligned = !intakeAligned;
                        }
                    }
                }
            }
            if (coloringRn) {
                if (spinindexer.withinRange(inputAngle)) {
                    if (spinindexer.colorDetected() == colorIWant) {
                            pushUp = true;
                            coloringRn = false;
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



            lastIntakeRight = intakeRight;
            lastShooterLeft = shooterLeft;
            lastIntakeLeft = intakeLeft;
            lastShooterRight = shooterRight;
            spinindexer.PID(inputAngle);

            if (!spinindexer.withinRange(inputAngle)) {
                pushUp = false;
            }
            if (intakeAligned) {
                pushUp = false;
            }
            spinindexer.nudging(pushUp, pushDown);
        }
    }
}