package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class DriverOperator extends LinearOpMode{
    FTCDriveTrain drivetrain = new FTCDriveTrain();
    Spindexer spindexer = new Spindexer();
    Shooter shooter = new Shooter();


    private boolean yButton2Pressed = false;
    private boolean xButton2Pressed = false;

    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
        spindexer.init(hardwareMap);
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
            boolean dpadUp2 = gamepad2.dpad_up;
            boolean dpadDown2 = gamepad2.dpad_down;
            double leftStick2Y = gamepad2.left_stick_y;
            boolean y2 = gamepad2.y;
            boolean x2 = gamepad2.x;
            boolean Hoodup = gamepad2.right_bumper;
            boolean Hooddown = gamepad2.left_bumper;
            // Drivetrain
//            drivetrain.Translate(x,y,rx,options);
            shooter.HoodStuff(Hoodup, Hooddown);


            // Spindexer manual servo
            if (dpadUp2)
            {
                spindexer.nudgeServoUp();
            }
            else if (dpadDown2)
            {
                spindexer.nudgeServoDown();
            }

            // Spindexer color search (If button pressed search for purple)
            if (y2 && !yButton2Pressed) {
                // When Y is pressed for the first time call searchForColor
                // The spindexer class will handle spinning, stopping, and raising the servo
                spindexer.searchForPurpleBall(0.4); // Just 40% power
            }

            yButton2Pressed =  y2;

            // Seach for Green ball with the X Button
            if (x2 && !xButton2Pressed) {
                spindexer.searchForGreenBall(0.4);
            }

            xButton2Pressed = x2;

            // Spindexer manual motor override
            // So operator can manually spin the spindexer with a left stick

            if (Math.abs(leftStick2Y) > 0.1) {
                spindexer.setManualPower(leftStick2Y);
            }

            spindexer.update();


            // Shoot stuff ahhh code.
            if (gamepad2.a) {
                shooter.ShootStuff(true);
            }
            else {
                shooter.ShootStuff(false);
            }

            // Hood stuff ahhh code.

        }


            // Telemetry for debugging
            telemetry.addData("Spindexer State", spindexer.getCurrentState());
            telemetry.addData("Servo Position", "%.2f", spindexer.getServoPosition());
            float[] hsv = spindexer.getHsvValues();
            telemetry.addData("Hue", "%.1f", hsv[0]);
            telemetry.addData("ServoState", spindexer.getServoPosition());
            telemetry.update();


    }
}