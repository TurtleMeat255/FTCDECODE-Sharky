//package org.firstinspires.ftc.teamcode.pedroPathing;
//
////import com.pedropathing.follower.Follower;
////import com.pedropathing.geometry.BezierLine;
////import com.pedropathing.geometry.Pose;
////import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.FTCDriveTrain;
//import org.firstinspires.ftc.teamcode.Shooter;
//import org.firstinspires.ftc.teamcode.Spinindexer;
//
//@Autonomous
//public class Auton4 extends LinearOpMode {
//    FTCDriveTrain dt = new FTCDriveTrain();
//    Spinindexer spindexer = new Spinindexer();
//    Shooter shooter = new Shooter();
//
//    Servo nudger;
//
//    double firingAngle = 0;
//    /**
//     * This method is called once at the init of the OpMode.
//     **/
//    boolean continuing = true;
//    @Override
//    public void runOpMode()
//    {
//        spindexer.init(hardwareMap);
//        shooter.init(hardwareMap);
//        dt = new FTCDriveTrain(hardwareMap);
//
//        nudger = hardwareMap.get(Servo.class, "nudger");
//        nudger.setDirection(Servo.Direction.REVERSE);
//        shooter.SetShooterPower(0.75);
//
//        waitForStart();
//        while (opModeIsActive())
//        {
//            if (continuing)
//            {
//                dt.Translate(0,0,0,true);
//                shooter.ActivateShooter(true, false);
//                sleep(1000);
//                nudger.setPosition(0.35);
////                spindexer.nudging(true, false);
//                sleep(1000);
//                nudger.setPosition(0.05);
////                spindexer.nudging(false, true);
//
//                sleep(1000);
//                while (!spindexer.withinRange(100))
//                {
//                    spindexer.PID(100);
//                }
//                spindexer.SpindexerPower(0);
//                sleep(500);
//
//                nudger.setPosition(0.35);
//                sleep(1000);
//                nudger.setPosition(0.05);
//
//                sleep(1000);
//                while (!spindexer.withinRange(220))
//                {
//                    spindexer.PID(220);
//                }
//                spindexer.SpindexerPower(0);
//                sleep(1000);
//
//                nudger.setPosition(0.35);
//                sleep(1000);
//                nudger.setPosition(0.05);
//                sleep(1000);
//
//                dt.Translate(-0.3,0.3,0,false);
//                sleep(2000);
//                dt.Translate(0,0,0,false);
//                sleep(1000);
//
//                continuing = false;
//            }
//        }
//    }
//}