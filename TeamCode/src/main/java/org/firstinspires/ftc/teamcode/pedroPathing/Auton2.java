package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.FTCDriveTrain;
import org.firstinspires.ftc.teamcode.IntakeCode;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Spinindexer;

@Autonomous(name = "Example Auto", group = "Examples")
public class Auton2 extends OpMode
{
    Spinindexer spinindexer = new Spinindexer();
    Shooter shooter = new Shooter();
    IntakeCode intake = new IntakeCode();

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

    private final Pose startPose = new Pose(67, 69, Math.toRadians(180)); // Start Pose of our robot.
    //private final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(42, 61, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(13, 7, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private PathChain shoot1, grabPickup1, scorePickup1, shoot2, grabPickup2, scorePickup2, shoot3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())
                .build();

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, startPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), startPose.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup2Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, startPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), startPose.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        if (nudgePosition == 0.25 && sinceHighered.getElapsedTimeSeconds() > 0.5) {
            nudgePosition = 0;
            sinceLowered.resetTimer();
            timesShot++;
        }


        switch (pathState) {
            case-1:
                if (shooterAccel.getElapsedTimeSeconds() > 2) {
                    setPathState(0);
                }
                break;
            case 0:
                follower.followPath(shoot1);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                if (spinindexer.withinRange(targetAngle) && nudgePosition == 0 && sinceLowered.getElapsedTimeSeconds() > 0.5 && timesShot != 3) {
                    if (timesChecked >= number[timesShot]) {
                        if (pushed) {
                            if (noClueWhatToCallThis == 2) {
                                targetAngle -= 120;
                            } else {
                                targetAngle += 120;
                            }
                            pushed = false;
                        } else {
                            nudgePosition = 0.25;
                            sinceHighered.resetTimer();
                            pushed = true;
                        }
                    } else {
                        if (colorISee == colorIWant) {
                            nudgePosition = 0.25;
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
                if(!follower.isBusy() && timesShot == 3) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                    targetAngle += 60;
                }
                break;
            case 2:
                shooterOn = false;
                if (gobbleTimer.getElapsedTimeSeconds() > 0.5 && nudgePosition == 0 && sinceLowered.getElapsedTimeSeconds() > 0.5) {
                    targetAngle += 120;
                    gobbleTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                    targetAngle += 60;
                    timesShot = 0;
                    timesChecked = 0;
                    hold = false;
                }
                break;
            case 3:
                shooterOn = true;
                /// //green
                if (!hold) {
                    if (spinindexer.withinRange(targetAngle) && nudgePosition == 0 && sinceLowered.getElapsedTimeSeconds() > 0.5) {
                        if (colorISee == colorIWant) {
                            hold = true;
                        } else {
                            timesChecked++;
                            targetAngle += 120;
                        }
                    }
                }
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(shoot2,true);
                    setPathState(4);
                    pushed = false;
                    noClueWhatToCallThis = 0;
                }
                break;
            case 4:
                if (spinindexer.withinRange(targetAngle) && nudgePosition == 0 && sinceLowered.getElapsedTimeSeconds() > 0.5 && timesShot != 3) {
                    if (timesChecked >= number[timesShot]) {
                        if (pushed) {
                            if (noClueWhatToCallThis == 2) {
                                targetAngle -= 120;
                            } else {
                                targetAngle += 120;
                            }
                            pushed = false;
                        } else {
                            nudgePosition = 0.25;
                            sinceHighered.resetTimer();
                            pushed = true;
                        }
                    } else {
                        if (colorISee == colorIWant) {
                            nudgePosition = 0.25;
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


                if(!follower.isBusy() && timesShot == 3) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(5);
                    targetAngle += 60;
                }
                break;
            case 5:
                shooterOn = false;
                if (gobbleTimer.getElapsedTimeSeconds() > 0.5 && nudgePosition == 0 && sinceLowered.getElapsedTimeSeconds() > 0.5) {
                    targetAngle += 120;
                    gobbleTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(6);
                    targetAngle += 60;
                    timesShot = 0;
                    timesChecked = 0;
                    hold = false;
                }
                break;
            case 6:
                shooterOn = true;
                /// ///green
                if (!hold) {
                    if (spinindexer.withinRange(targetAngle) && nudgePosition == 0 && sinceLowered.getElapsedTimeSeconds() > 0.5) {
                        if (colorISee == colorIWant) {
                            hold = true;
                        } else {
                            timesChecked++;
                            targetAngle += 120;
                        }
                    }
                }
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(shoot3, true);
                    setPathState(7);
                    pushed = false;
                    noClueWhatToCallThis = 0;
                }
                break;
            case 7:
                if (spinindexer.withinRange(targetAngle) && nudgePosition == 0 && sinceLowered.getElapsedTimeSeconds() > 0.5 && timesShot != 3) {
                    if (timesChecked >= number[timesShot]) {
                        if (pushed) {
                            if (noClueWhatToCallThis == 2) {
                                targetAngle -= 120;
                            } else {
                                targetAngle += 120;
                            }
                            pushed = false;
                        } else {
                            nudgePosition = 0.25;
                            sinceHighered.resetTimer();
                            pushed = true;
                        }
                    } else {
                        if (colorISee == colorIWant) {
                            nudgePosition = 0.25;
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
                if(!follower.isBusy() && timesShot == 3) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
        shooter.ActivateShooter(shooterOn);
        intake.ActivateIntake(intakeOn, false);
        spinindexer.PID(targetAngle);
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
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}