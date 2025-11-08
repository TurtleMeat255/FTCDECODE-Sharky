package org.firstinspires.ftc.teamcode.pedroPathing;

//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.IntakeCode;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Spinindexer;

@Autonomous
public class Auton3 extends LinearOpMode {
    Spinindexer spindexer = new Spinindexer();
    Shooter shooter = new Shooter();

    Servo nudger;

    double firingAngle = 0;
    /**
     * This method is called once at the init of the OpMode.
     **/
    boolean continuing = true;
    @Override
    public void runOpMode()
    {
        spindexer.init(hardwareMap);
        shooter.init(hardwareMap);

        nudger = hardwareMap.get(Servo.class, "nudger");
        nudger.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive())
        {
            if (continuing)
            {
                shooter.ActivateShooter(true);
                sleep(1000);
                nudger.setPosition(0.35);
//                spindexer.nudging(true, false);
                sleep(1000);
                nudger.setPosition(0.05);
//                spindexer.nudging(false, true);

                sleep(1000);
                while (!spindexer.withinRange(120))
                {
                    spindexer.PID(120);
                }
                spindexer.SpindexerPower(0);

                nudger.setPosition(0.35);
                sleep(1000);
                nudger.setPosition(0.05);

                sleep(1000);
                while (!spindexer.withinRange(240))
                {
                    spindexer.PID(240);
                }
                spindexer.SpindexerPower(0);

                nudger.setPosition(0.35);
                sleep(1000);
                nudger.setPosition(0.05);
                sleep(1000);
                continuing = false;
            }
        }
    }
}