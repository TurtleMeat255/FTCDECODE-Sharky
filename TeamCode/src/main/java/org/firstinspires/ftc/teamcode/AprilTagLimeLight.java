package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class AprilTagLimeLight {
    private Limelight3A limeLight;
    private IMU Imu;

    private final double LIMELIGHTANGLECONST_D = 21;

    private final double APRILTAGH_M = 749.3;
    private final double LIMELIGHTDISTBOTTOM_M = 245.7;
    private final double LIMELIGHTDISTCONST_M = APRILTAGH_M - LIMELIGHTDISTBOTTOM_M;

    public void init(HardwareMap hwMap) {
        limeLight = hwMap.get(Limelight3A.class, "limelight");
        Imu = hwMap.get(IMU.class, "imu");
        limeLight.pipelineSwitch(0); // ASK JACOB HOW TO FIX. IDK WHAT THIS IS. WE MIGHT NEED TWO FOR BOTH COLOR SENSING AND APRIL TAG DETECTION
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        Imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limeLight.start(); // Uses large  amount of battery btw... IF DELAY MOVE TO INIT METHOD.

    }

    private LLResult GetResult()
    {
        YawPitchRollAngles orientation = Imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limeLight.getLatestResult();
        return llresult;
    }

    public double GetDistance()
    {
        LLResult llresult = GetResult();

        if (llresult != null && llresult.isValid()) {

            double theta = llresult.getTy();
            theta += LIMELIGHTANGLECONST_D;

            theta = Math.toRadians(theta);

            double DIST_M = LIMELIGHTDISTCONST_M / Math.tan(theta);
            double DIST_I = DIST_M / 25.4;

            return DIST_I;
        }
        return -1;
    }

    public double GetTX()
    {
        LLResult llresult = GetResult();
        return llresult.getTx();
    }

    public int GetLimelightId()
    {
        LLResult llresult = GetResult();
        List<LLResultTypes.FiducialResult> fiducial = llresult.getFiducialResults();

        if (fiducial.isEmpty())
        {
            return 0;
        }
        return fiducial.get(0).getFiducialId();
    }
}
