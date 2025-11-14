package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ColorSensor;
import org.firstinspires.ftc.teamcode.IntakeCode;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Spinindexer;

@Autonomous(name = "AUTON PEDRO", group = "PEDRO TESTING")
public class Auton5 extends OpMode {

    Spinindexer spindexer = new Spinindexer();
    Shooter shooter = new Shooter();
    IntakeCode intake = new IntakeCode();
    ColorSensor colorSensor = new ColorSensor();


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private Path scorePreload;
    private PathChain prepareGrab1, grab1, score2, prepareGrab2, grab2, score3;

    int motif = 21;
    /*
    21 = gpp
    22 = pgp
    23 = ppg
     */



    ElapsedTime pushUpTimer = new ElapsedTime();
    double pushUpMaxTime = 1;

    boolean coloringRn = false;
    int ticker = 0;
    boolean pushUp;

    public double inputAngle = 0;

    private final Pose startPose = new Pose(20, 130, Math.toRadians(155)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(155)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose grab1StartPose = new Pose(60, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose grab1Pose = new Pose(10, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose grab2StartPose = new Pose(60, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose grab2Pose = new Pose(10, 60, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        prepareGrab1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, grab1StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grab1StartPose.getHeading())
                .build();
        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(grab1StartPose, grab1Pose))
                .setLinearHeadingInterpolation(grab1StartPose.getHeading(), grab1Pose.getHeading())
                .setBrakingStart(1)
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(grab1Pose, scorePose))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), scorePose.getHeading())
                .build();
        prepareGrab2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, grab2StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grab2StartPose.getHeading())
                .build();
        grab2 = follower.pathBuilder()
                .addPath(new BezierLine(grab2StartPose, grab2Pose))
                .setLinearHeadingInterpolation(grab2StartPose.getHeading(), grab2Pose.getHeading())
                .setBrakingStart(1)
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(grab2Pose, scorePose))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void ShootGreen()
    {
        coloringRn = true;
        ticker = 0;
        shooter.FireAtRPM(1000);

        ShootBall(ColorSensor.DetectedColor.GREEN,1000);
    }

    public void ShootPurple()
    {
        coloringRn = true;
        ticker = 0;
        shooter.FireAtRPM(1000);
        ShootBall(ColorSensor.DetectedColor.PURPLE,1000);
    }

    public void ShootBall(ColorSensor.DetectedColor color, double rpm)
    {
        boolean continuing = true;
        while (continuing)
        {
            if (!shooter.RPMCorrect(rpm)) {continue;}
            if (coloringRn) {
                if (spindexer.withinRange(inputAngle)) {
                    if (colorSensor.GetDetectedColor() == color) {
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

            if (pushUpTimer.seconds() > pushUpMaxTime - 0.5)
            {
                pushUp = false;
            }

            spindexer.PID(inputAngle);
            spindexer.nudging(pushUp);

            if (pushUpTimer.seconds() > pushUpMaxTime)
            {
                continuing = false;
            }
        }
    }

    public void scoringSequence(int pathState)
    {
        // swap scoring based on motif
        switch (motif)
        {
            case 21:
                ShootGreen();
                ShootPurple();
                ShootPurple();
                break;
            case 22:
                ShootPurple();
                ShootGreen();
                ShootPurple();
                break;
            case 23:
                ShootPurple();
                ShootPurple();
                ShootGreen();
                break;
            default: // error handling :(
                break;
        }

        setPathState(pathState);
        shooter.FireAtRPM(0);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy())
                {
                    scoringSequence(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(prepareGrab1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(grab1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(score2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy())
                {
                    scoringSequence(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(prepareGrab2,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(grab2,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(score3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy())
                {
                    scoringSequence(10);
                }
            case 10:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        spindexer.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        colorSensor.init(hardwareMap);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
//        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}