//package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.AprilTagLimeLight;
//import org.firstinspires.ftc.teamcode.ColorSensor;
//import org.firstinspires.ftc.teamcode.Shooter;
//import org.firstinspires.ftc.teamcode.Spinindexer;
//
//@Autonomous(name = "AUTON PEDRO BLUE", group = "PEDRO TESTING")
//public class Auton5 extends OpMode {
//
//    Spinindexer spindexer = new Spinindexer();
//    Shooter shooter = new Shooter();
//    IntakeSubsystem intake;
//    ColorSensor colorSensor = new ColorSensor();
//
//    AprilTagLimeLight limelight = new AprilTagLimeLight();
//
//    public Servo nudger;
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer, shooterAccel, gobbleTimer, sinceLowered, sinceHighered;
//
//    private int pathState = -1;
//    private int timesShot = 0;
//    private int timesChecked = 0;
//    private int colorIWant = 0;
//    private int colorISee = 0;
//    private int noClueWhatToCallThis = 0;
//    int[] number = {3, 3, 2, 100};
//    private boolean intakeOn = true;
//    private boolean shooterOn = true;
//    private boolean holding = false;
//    private boolean pushed = false;
//    private double targetAngle = 0;
//    private double nudgePosition = 0.05;
//
//    private Path scorePreload;
//    private PathChain prepareGrab1, grab11, grab12, grab13, hold, score2, prepareGrab2, grab21, grab22, grab23, prepareScore3, score3, leave;
//
//    int motif = 24;
//    /*
//    21 = gpp
//    22 = pgp
//    23 = ppg
//     */
//
//    ElapsedTime pushUpTimer = new ElapsedTime();
//    double pushUpMaxTime = 1;
//
//    boolean coloringRn = false;
//    int ticker = 0;
//    boolean pushUp;
//
//    public double inputAngle = 0;
//
//    private double firingRPM = 3300;
//
//    private double distance = 0;
//    private double startOffset = 30;
//
//    private final Pose startPose = new Pose(24, 120, Math.toRadians(135)); // Start Pose of our robot.
//    private final Pose scorePose = new Pose(40, 96, Math.toRadians(130)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//    private final Pose scorePose3 = new Pose(48, 88, Math.toRadians(130)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//
//    private final Pose grab1StartPose = new Pose(55, 76, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose grab1Pose1 = new Pose(35, 76, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose grab1Pose2 = new Pose(24,   76, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose grab1Pose3 = new Pose(17, 76, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
//
//    private final Pose grab2StartPose = new Pose(55, 52, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose grab2Pose1 = new Pose(35, 52, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose grab2Pose2 = new Pose(24, 52, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose grab2Pose3 = new Pose(6, 52, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose leavePose = new Pose(20, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
//
//
//
//    ElapsedTime waitTimer = new ElapsedTime();
//    boolean canSwap = true;
//
//    private final ColorSensor.DetectedColor[] motif21 = {ColorSensor.DetectedColor.GREEN, ColorSensor.DetectedColor.PURPLE, ColorSensor.DetectedColor.PURPLE};
//    private final ColorSensor.DetectedColor[] motif22 = {ColorSensor.DetectedColor.PURPLE, ColorSensor.DetectedColor.GREEN, ColorSensor.DetectedColor.PURPLE};
//    private final ColorSensor.DetectedColor[] motif23 = {ColorSensor.DetectedColor.PURPLE, ColorSensor.DetectedColor.PURPLE, ColorSensor.DetectedColor.GREEN};
//
//    public void buildPaths() {
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//        prepareGrab1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, grab1StartPose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), grab1StartPose.getHeading())
//                .build();
//        hold = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose,scorePose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), grab1StartPose.getHeading())
//                .setVelocityConstraint(0)
//                .build();
//        grab11 = follower.pathBuilder()
//                .addPath(new BezierLine(grab1StartPose, grab1Pose1))
//                .setLinearHeadingInterpolation(grab1StartPose.getHeading(), grab1Pose1.getHeading())
//                .setVelocityConstraint(0.1)
//                .setBrakingStart(1)
//                .setBrakingStrength(1)
//
//                .build();
//        grab12 = follower.pathBuilder()
//                .addPath(new BezierLine(grab1Pose1, grab1Pose2))
//                .setLinearHeadingInterpolation(grab1Pose1.getHeading(), grab1Pose2.getHeading())
//                .setVelocityConstraint(0.1)
//                .setBrakingStart(1)
//                .setBrakingStrength(1)
//
//                .build();
//        grab13 = follower.pathBuilder()
//                .addPath(new BezierLine(grab1Pose2, grab1Pose3))
//                .setLinearHeadingInterpolation(grab1Pose2.getHeading(), grab1Pose3.getHeading())
//                .setVelocityConstraint(0.1)
//                .setBrakingStart(1)
//                .setBrakingStrength(1)
//
//                .build();
//        score2 = follower.pathBuilder()
//                .addPath(new BezierLine(grab1Pose3, scorePose))
//                .setLinearHeadingInterpolation(grab1Pose3.getHeading(), scorePose.getHeading())
//                .setBrakingStrength(1)
//                .setBrakingStart(1)
//                .setVelocityConstraint(0)
//                .build();
//        prepareGrab2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, grab2StartPose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), grab2StartPose.getHeading())
//                .build();
//        grab21 = follower.pathBuilder()
//                .addPath(new BezierLine(grab2StartPose, grab2Pose1))
//                .setLinearHeadingInterpolation(grab2StartPose.getHeading(), grab2Pose1.getHeading())
//                .setVelocityConstraint(0.1)
//                .setBrakingStart(1)
//                .setBrakingStrength(1)
//                .build();
//        grab22 = follower.pathBuilder()
//                .addPath(new BezierLine(grab2StartPose, grab2Pose2))
//                .setLinearHeadingInterpolation(grab2Pose1.getHeading(), grab2Pose2.getHeading())
//                .setVelocityConstraint(0.1)
//                .setBrakingStart(1)
//                .setBrakingStrength(1)
//                .build();
//        grab23 = follower.pathBuilder()
//                .addPath(new BezierLine(grab2StartPose, grab2Pose3))
//                .setLinearHeadingInterpolation(grab2Pose2.getHeading(), grab2Pose3.getHeading())
//                .setVelocityConstraint(0.1)
//                .setBrakingStart(1)
//                .setBrakingStrength(1)
//                .build();
//        prepareScore3 = follower.pathBuilder()
//                .addPath(new BezierLine(grab2Pose3, grab2StartPose))
//                .setLinearHeadingInterpolation(grab2Pose3.getHeading(), grab2StartPose.getHeading())
//                .setVelocityConstraint(0.1)
//                .setBrakingStart(1)
//                .setBrakingStrength(1)
//                .build();
//        score3 = follower.pathBuilder()
//                .addPath(new BezierLine(grab2StartPose, scorePose3))
//                .setLinearHeadingInterpolation(grab2StartPose.getHeading(), scorePose3.getHeading())
//                .setBrakingStrength(1)
//                .setVelocityConstraint(0)
//                .setBrakingStart(1)
//                .build();
//        leave = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose3, leavePose))
//                .setLinearHeadingInterpolation(scorePose3.getHeading(), leavePose.getHeading())
//                .setBrakingStrength(1)
//                .setVelocityConstraint(0)
//                .setBrakingStart(1)
//                .build();
//    }
//
//    public void ShootGreen()
//    {
//        coloringRn = true;
//        ticker = 0;
//        shooter.FireAtRPM(2000);
//
//        ShootBall(ColorSensor.DetectedColor.GREEN,2000);
//    }
//
//    public void ShootPurple()
//    {
//        coloringRn = true;
//        ticker = 0;
//        shooter.FireAtRPM(2000);
//        ShootBall(ColorSensor.DetectedColor.PURPLE,2000);
//    }
//
//    private void RapidFire()
//    {
//        if (spindexer.withinRange(inputAngle))
//        {
//            spindexer.RapidFiring(true);
//            spindexer.BallKD(true);
//            for (int i = 0; i < 3; i ++)
//            {
//                if (i > 0)
//                {
//                    inputAngle += 120;
//                }
//                while (!shooter.RPMCorrect(firingRPM) || !spindexer.withinRange(inputAngle))
//                {
//                    shooter.SetShooterRPM(firingRPM);
//                    spindexer.PID(inputAngle);
//
//                    firingRPM = 3300;
//                    shooter.SetHoodPosition(0.8);
//                }
//
//                double time_start = System.currentTimeMillis();
//                while (System.currentTimeMillis() - time_start <= 0.3)
//                {
//                    spindexer.nudging(true);
//                }
//                time_start = System.currentTimeMillis();
//                while (System.currentTimeMillis() - time_start <= 0.2)
//                {
//                    spindexer.nudging(false);
//                }
//            }
//            spindexer.BallKD(false);
//        }
//    }
//
//    public void ShootBall(ColorSensor.DetectedColor color, double rpm)
//    {
//        boolean continuing = true;
//        while (continuing)
//        {
//            if (!shooter.RPMCorrect(rpm)) {continue;}
//            if (coloringRn) {
//                if (spindexer.withinRange(inputAngle)) {
//                    if (colorSensor.GetDetectedColor() == color) {
//                        pushUp = true;
//                        coloringRn = false;
//                        pushUpTimer.reset();
//                    } else {
//                        pushUp = false;
//                        ticker += 1;
//                        if (ticker == 2) {
//                            coloringRn = false;
//                        } else {
//                            inputAngle += 120;
//                        }
//                    }
//                }
//            }
//
//            if (pushUpTimer.seconds() > pushUpMaxTime - 0.5)
//            {
//                pushUp = false;
//            }
//
//            spindexer.PID(inputAngle);
//            spindexer.nudging(pushUp);
//
//            if (pushUpTimer.seconds() > pushUpMaxTime)
//            {
//                continuing = false;
//            }
//        }
//    }
//
//    private ColorSensor.DetectedColor getTargetColor() {
//        if (timesShot < 0 || timesShot >= 3) {
//            return ColorSensor.DetectedColor.UNKNOWN;
//        }
//
//        switch (motif) {
//            case 21:
//                return motif21[timesShot];
//            case 22:
//                return motif22[timesShot];
//            case 23:
//                return motif23[timesShot];
//            default:
//                return ColorSensor.DetectedColor.UNKNOWN;
//        }
//    }
//
//    public void scoringSequence(int pathState)
//    {
//        // swap scoring based on motif
//        switch (motif)
//        {
//            case 21:
//                ShootGreen();
//                ShootPurple();
//                ShootPurple();
//                break;
//            case 22:
//                ShootPurple();
//                ShootGreen();
//                ShootPurple();
//                break;
//            case 23:
//                ShootPurple();
//                ShootPurple();
//                ShootGreen();
//                break;
//            case 24:
//                RapidFire();
//            default: // error handling :(
//                break;
//        }
//
//        setPathState(pathState);
//        shooter.FireAtRPM(0);
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                spindexer.BallKD(true);
//                follower.followPath(scorePreload,true);
//                setPathState(1);
//                break;
//            case 1:
//                if (!follower.isBusy())
//                {
////                    follower.followPath(hold);
//                    scoringSequence(2);
//                }
//                break;
//            case 2:
//                if(!follower.isBusy()) {
//                    inputAngle += 60;
//                    while(!spindexer.withinRange(inputAngle))
//                    {
//                        spindexer.PID(inputAngle);
//                    }
//                    follower.followPath(prepareGrab1,true);
//                    setPathState(3);
//                    waitTimer.reset();
//                }
//                break;
//            case 3:
//                if(!follower.isBusy()) {
//                    intake.outtake();
//
//                    follower.followPath(grab11,true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if(!follower.isBusy()) {
//                    intake.outtake();
//
//                    inputAngle += 120;
//                    while(!spindexer.withinRange(inputAngle))
//                    {
//                        spindexer.PID(inputAngle);
//                    }
//
//                    follower.followPath(grab12,true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if(!follower.isBusy()) {
//                    intake.outtake();
//
//                    inputAngle += 120;
//                    while(!spindexer.withinRange(inputAngle))
//                    {
//                        spindexer.PID(inputAngle);
//                    }
//
//                    follower.followPath(grab13,true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if(!follower.isBusy()) {
////                    intake.ActivateIntake(false, false);
////
////                    inputAngle += 60;
////                    while(!spindexer.withinRange(inputAngle))
////                    {
////                        spindexer.PID(inputAngle);
////                    }
//
//                    follower.followPath(score2,true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
////                setPathState(6);
//                if (!follower.isBusy())
//                {
//                    inputAngle += 120;
//                    while(!spindexer.withinRange(inputAngle))
//                    {
//                        spindexer.PID(inputAngle);
//                    }
//
//                    inputAngle += 60;
//                    while(!spindexer.withinRange(inputAngle))
//                    {
//                        spindexer.PID(inputAngle);
//                    }
//
//                    scoringSequence(8);
//                }
//                break;
//            case 8:
//                if(!follower.isBusy()) {
//                    intake.outtake();
//
//                    inputAngle += 120;
//                    while(!spindexer.withinRange(inputAngle))
//                    {
//                        spindexer.PID(inputAngle);
//                    }
//                    follower.followPath(prepareGrab2,true);
//                    setPathState(9);
//                    waitTimer.reset();
//                }
//                break;
//            case 9:
//                if(!follower.isBusy()) {
//                    intake.outtake();
//
//                    follower.followPath(grab21,true);
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                if(!follower.isBusy()) {
//                    intake.outtake();
//
//                    inputAngle += 120;
//                    while(!spindexer.withinRange(inputAngle))
//                    {
//                        spindexer.PID(inputAngle);
//                    }
//
//                    follower.followPath(grab22,true);
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                if(!follower.isBusy()) {
//                    inputAngle += 60;
//                    while(!spindexer.withinRange(inputAngle))
//                    {
//                        spindexer.PID(inputAngle);
//                    }
//
//                    follower.followPath(grab23,true);
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                if (!follower.isBusy())
//                {
////                    intake.ActivateIntake(false, false);
//
//                    follower.followPath(prepareScore3, true);
//                    setPathState(13);
//                }
//                break;
//            case 13:
//                if(!follower.isBusy()) {
////                    inputAngle += 60;
////                    while(!spindexer.withinRange(inputAngle))
////                    {
////                        spindexer.PID(inputAngle);
////                    }
//                    follower.followPath(score3, true);
//                    setPathState(14);
//                }
//                break;
//            case 14:
//                if (!follower.isBusy())
//                {
//                    inputAngle += 60;
//                    while(!spindexer.withinRange(inputAngle))
//                    {
//                        spindexer.PID(inputAngle);
//                    }
//
//                    scoringSequence(15);
//                }
//                break;
//            case 15:
//                if(!follower.isBusy()) {
//                    follower.followPath(leave, true);
//                    setPathState(16);
//                }
//                break;
//            case 16:
//                if (!follower.isBusy())
//                {
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//
//    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        autonomousPathUpdate();
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("servo position", nudger.getPosition());
//        telemetry.addData("spindexer position", inputAngle);
//        telemetry.update();
//    }
//
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//
//        spindexer.init(hardwareMap);
//        shooter.init(hardwareMap);
//        intake = new IntakeSubsystem(hardwareMap, "intake");
//        colorSensor.init(hardwareMap);
//        limelight.init(hardwareMap);
//    }
//
//    /** This method is called continuously after Init while waiting for "play". **/
//    @Override
//    public void init_loop() {}
//
//    /** This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
////        setPathState(0);
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {}
//
//    public void runToPickUp() {
//        shooterOn = false;
//        if (gobbleTimer.getElapsedTimeSeconds() > 0.5 && nudgePosition == 0.05 && sinceLowered.getElapsedTimeSeconds() > 0.5) {
//            targetAngle += 120;
//            gobbleTimer.resetTimer();
//        }
//    }
//    public void runWhileShooting() {
//        if (spindexer.withinRange(targetAngle) && nudgePosition == 0.05 && sinceLowered.getElapsedTimeSeconds() > 0.5 && timesShot != 3) {
//            if (timesChecked >= number[timesShot]) {
//                if (pushed) {
//                    if (noClueWhatToCallThis == 2) {
//                        targetAngle -= 120;
//                    } else {
//                        targetAngle += 120;
//                    }
//                    pushed = false;
//                } else {
//                    nudgePosition = 0.35;
//                    sinceHighered.resetTimer();
//                    pushed = true;
//                }
//            } else {
//                if (colorSensor.GetDetectedColor() == getTargetColor()) {
//                    nudgePosition = 0.35;
//                    sinceHighered.resetTimer();
//                    timesChecked = 0;
//                } else {
//                    timesChecked++;
//                    if (timesChecked >= number[timesShot]) {
//                    } else {
//                        if (noClueWhatToCallThis == 2) {
//                            targetAngle -= 120;
//                        } else {
//                            targetAngle += 120;
//                        }
//                        if (timesShot == 1) {
//                            noClueWhatToCallThis++;
//                        }
//                    }
//                }
//            }
//        }
//    }
//    public void runToDropOff() {
//        shooterOn = true;
//        /// //green
//        if (!holding) {
//            if (spindexer.withinRange(targetAngle) && nudgePosition == 0.05 && sinceLowered.getElapsedTimeSeconds() > 0.5) {
//                if (colorISee == colorIWant) {
//                    holding = true;
//                } else {
//                    timesChecked++;
//                    targetAngle += 120;
//                }
//            }
//        }
//    }
//}