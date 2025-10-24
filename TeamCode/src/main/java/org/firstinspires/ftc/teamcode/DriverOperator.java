package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class DriverOperator extends LinearOpMode{
    FTCDriveTrain drivetrain = new FTCDriveTrain();

    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);

        waitForStart();
        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean options = gamepad1.options;

            drivetrain.Translate(x,y,rx,options);

            // drivetrain translate stufffff
        }
    }
}