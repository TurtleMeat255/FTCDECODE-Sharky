package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp
public class AprilTagLimeLight extends OpMode {
    private Limelight3A limeLight;
    private IMU Imu;

    private final double LIMELIGHTANGLECONST_D = 21;

    private final double APRILTAGH_M = 749.3;
    private final double LIMELIGHTDISTBOTTOM_M = 245.7;
    private final double LIMELIGHTDISTCONST_M = APRILTAGH_M - LIMELIGHTDISTBOTTOM_M;

    @Override
    public void init() {
        limeLight = hardwareMap.get(Limelight3A.class, "limelight");
        Imu = hardwareMap.get(IMU.class, "imu");
        limeLight.pipelineSwitch(0); // ASK JACOB HOW TO FIX. IDK WHAT THIS IS. WE MIGHT NEED TWO FOR BOTH COLOR SENSING AND APRIL TAG DETECTION
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        Imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limeLight.start(); // Uses large  amount of battery btw... IF DELAY MOVE TO INIT METHOD.
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = Imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limeLight.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducial = llresult.getFiducialResults();

        if (llresult != null && llresult.isValid()) {
//            Pose3D botPose = llresult.getBotpose_MT2();
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ty", llresult.getTy());
            telemetry.addData("Ta", llresult.getTa());
            telemetry.addData("Target Area", llresult.getTa());
            telemetry.addData("id", fiducial.get(0).getFiducialId());

            double theta = llresult.getTy();
            theta += LIMELIGHTANGLECONST_D;

            theta = Math.toRadians(theta);

            double txTheta = llresult.getTx();

            txTheta = Math.toRadians(txTheta);

            // limelight physical angle - 21 degrees
            // lens dist to bottom - 245.7mm = const
            // dist = const/tan theta

            double distance = LIMELIGHTDISTCONST_M / Math.tan(theta);
            distance /= Math.cos(txTheta);

            telemetry.addData("distance", distance);

            if (fiducial.get(0).getFiducialId() == 21) {
                //run code
            } else if (fiducial.get(0).getFiducialId() == 22) {
                // run code
            } else if (fiducial.get(0).getFiducialId() == 23) {
                // run code
            }
        }

        telemetry.update();
    }
}
