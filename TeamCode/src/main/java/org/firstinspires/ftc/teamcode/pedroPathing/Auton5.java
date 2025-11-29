package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagLimeLight;
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
    Spinindexer spinindexer = new Spinindexer();

    AprilTagLimeLight limelight = new AprilTagLimeLight();


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shooterAccel, gobbleTimer, sinceLowered, sinceHighered;
    private int pathState = -1;
    private int timesShot = 0;
    private int timesChecked = 0;
    private int colorIWant = 0;
    private int colorISee = 0;
    private int noClueWhatToCallThis = 0;
    int[] number = {3, 3, 2, 100};
    private boolean intakeOn = true;
    private boolean shooterOn = true;
    private boolean hold = false;
    private boolean pushed = false;
    private double targetAngle = 0;
    private double nudgePosition = 0;


    private Path scorePreload;
    private PathChain prepareGrab1, grab1, score2, prepareGrab2, grab2, score3;

    int motif = 24;
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

    private double firingRPM = 3300;

    private double distance = 0;
    private double startOffset = 30;

    private final Pose startPose = new Pose(130, 20, Math.toRadians(155)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(84, 60, Math.toRadians(155)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose grab1StartPose = new Pose(84, 60, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose grab1Pose = new Pose(84, 10, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose grab2StartPose = new Pose(60, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose grab2Pose = new Pose(60, 10, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    ElapsedTime waitTimer = new ElapsedTime();

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
        shooter.FireAtRPM(2000);

        ShootBall(ColorSensor.DetectedColor.GREEN,2000);
    }

    public void ShootPurple()
    {
        coloringRn = true;
        ticker = 0;
        shooter.FireAtRPM(2000);
        ShootBall(ColorSensor.DetectedColor.PURPLE,2000);
    }

    private void RapidFire()
    {
        if (spindexer.withinRange(inputAngle))
        {
            spindexer.RapidFiring(true);
            spindexer.BallKD(true);
            for (int i = 0; i < 3; i ++)
            {
                if (i > 0)
                {
                    inputAngle += 120;
                }
                while (!shooter.RPMCorrect(firingRPM) || !spindexer.withinRange(inputAngle))
                {
                    shooter.SetShooterRPM(firingRPM);
                    spindexer.PID(inputAngle);

                    firingRPM = 3300;
                    shooter.SetHoodPosition(0.8);
                }

                double time_start = System.currentTimeMillis();
                while (System.currentTimeMillis() - time_start <= 0.3)
                {
                    spindexer.nudging(true);
                }
                time_start = System.currentTimeMillis();
                while (System.currentTimeMillis() - time_start <= 0.2)
                {
                    spindexer.nudging(false);
                }
            }
            spindexer.BallKD(false);
        }
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
            case 24:
                RapidFire();
            default: // error handling :(
                break;
        }

        setPathState(pathState);
        shooter.FireAtRPM(0);
    }

    public void autonomousPathUpdate() {
        if (nudgePosition == 0.35 && sinceHighered.getElapsedTimeSeconds() > 0.5) {
            nudgePosition = 0.05;
            sinceLowered.resetTimer();
            timesShot++;
        }

        switch (pathState) {
            case -1:
                if (shooterAccel.getElapsedTimeSeconds() > 2) {
                    setPathState(0);
                }
                break;
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
                runWhileShooting();

                if(!follower.isBusy() && timesShot == 3) {
                    follower.followPath(prepareGrab1,true);
                    setPathState(3);
                    targetAngle += 60;
                }
                break;
            case 3:

                if(!follower.isBusy()) {
                    follower.followPath(grab1,true);
                    setPathState(4);

                }
                break;
            case 4:
                runToPickUp();
                if(!follower.isBusy()) {
                    follower.followPath(score2,true);
                    setPathState(5);
                    targetAngle += 60;
                    timesShot = 0;
                    timesChecked = 0;
                    hold = false;
                }
                break;
            case 5:
                runToDropOff();
                if (!follower.isBusy())
                {
                    scoringSequence(6);
                    pushed = false;
                    noClueWhatToCallThis = 0;
                }
                break;
            case 6:
                runWhileShooting();

                if(!follower.isBusy() && timesShot == 3) {
                    follower.followPath(prepareGrab2,true);
                    setPathState(7);
                    targetAngle += 60;
                }
                break;
            case 7:

                if(!follower.isBusy()) {
                    follower.followPath(grab2,true);
                    setPathState(8);

                }


                break;
            case 8:
                runToPickUp();

                if(!follower.isBusy()) {
                    follower.followPath(score3, true);
                    setPathState(9);
                    targetAngle += 60;
                    timesShot = 0;
                    timesChecked = 0;
                    hold = false;
                }
                break;
            case 9:
                runToDropOff();

                if (!follower.isBusy())
                {
                    scoringSequence(10);
                    pushed = false;
                    noClueWhatToCallThis = 0;
                }
            case 10:
                runWhileShooting();

                if(!follower.isBusy() && timesShot == 3) {
                    setPathState(-100);
                }
                break;
        }
        shooter.ActivateShooter(shooterOn, false);
        intake.ActivateIntake(intakeOn, false);
        spinindexer.PID(targetAngle);
        spinindexer.nudge2(nudgePosition);
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
        limelight.init(hardwareMap);
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
    public void runToPickUp() {
        shooterOn = false;
        if (gobbleTimer.getElapsedTimeSeconds() > 0.5 && nudgePosition == 0.05 && sinceLowered.getElapsedTimeSeconds() > 0.5) {
            targetAngle += 120;
            gobbleTimer.resetTimer();
        }
    }
    public void runWhileShooting() {
        if (spinindexer.withinRange(targetAngle) && nudgePosition == 0.05 && sinceLowered.getElapsedTimeSeconds() > 0.5 && timesShot != 3) {
            if (timesChecked >= number[timesShot]) {
                if (pushed) {
                    if (noClueWhatToCallThis == 2) {
                        targetAngle -= 120;
                    } else {
                        targetAngle += 120;
                    }
                    pushed = false;
                } else {
                    nudgePosition = 0.35;
                    sinceHighered.resetTimer();
                    pushed = true;
                }
            } else {
                if (colorISee == colorIWant) {
                    nudgePosition = 0.35;
                    sinceHighered.resetTimer();
                    timesChecked = 0;
                } else {
                    timesChecked++;
                    if (timesChecked >= number[timesShot]) {
                    } else {
                        if (noClueWhatToCallThis == 2) {
                            targetAngle -= 120;
                        } else {
                            targetAngle += 120;
                        }
                        if (timesShot == 1) {
                            noClueWhatToCallThis++;
                        }
                    }
                }
            }
        }
    }
    public void runToDropOff() {
        shooterOn = true;
        /// //green
        if (!hold) {
            if (spinindexer.withinRange(targetAngle) && nudgePosition == 0.05 && sinceLowered.getElapsedTimeSeconds() > 0.5) {
                if (colorISee == colorIWant) {
                    hold = true;
                } else {
                    timesChecked++;
                    targetAngle += 120;
                }
            }
        }
    }
}