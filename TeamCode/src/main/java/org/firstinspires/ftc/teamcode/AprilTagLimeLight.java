package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class AprilTagLimeLight extends OpMode {
    private Limelight3A limeLight;
    private IMU Imu;

    @Override
    public void init() {
        limeLight = hardwareMap.get(Limelight3A.class, "limelight");
        Imu = hardwareMap.get(IMU.class, "Imu");
        limeLight.pipelineSwitch(8); // ASK JACOB HOW TO FIX. IDK WHAT THIS IS
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

        if (llresult != null && llresult.isValid()) {
            Pose3D botPose = llresult.getBotpose_MT2();
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ty", llresult.getTy());
            telemetry.addData("Target Area", llresult.getTa());
        }
    }
}
