package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class DriverOperator extends LinearOpMode{
    FTCDriveTrain drivetrain = new FTCDriveTrain();
    Spindexer spindexer = new Spindexer();

    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
        spindexer.init(hardwareMap);

        waitForStart();
        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean options = gamepad1.options;

            boolean dpadUp2 = gamepad2.dpad_up;
            boolean dpadDown2 = gamepad2.dpad_down;
            double leftStick2Y = gamepad2.left_stick_y;

            drivetrain.Translate(x,y,rx,options);

            if (dpadUp2)
            {
                spindexer.nudgeServoUp();
            }
            else if (dpadDown2)
            {
                spindexer.nudgeServoDown();
            }
            // drivetrain translate stufffff

            spindexer.setManualPower(leftStick2Y);




            spindexer.update();
        }
    }
}