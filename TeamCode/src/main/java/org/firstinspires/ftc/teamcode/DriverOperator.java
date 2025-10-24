package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class DriverOperator extends LinearOpMode{
    FTCDriveTrain drivetrain = new FTCDriveTrain();
    Shooter shooter = new Shooter();
    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
        shooter.init(hardwareMap);
        waitForStart();
        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean options = gamepad1.options;
            boolean ShootButton = gamepad2.a;
            boolean DownButton = gamepad2.x;
            boolean UpButton = gamepad2.y;

            drivetrain.Translate(x,y,rx,options);
            shooter.HoodStuff(UpButton, DownButton);
            shooter.ShootStuff(ShootButton);
            // drivetrain translate stufffff
        }
    }
}