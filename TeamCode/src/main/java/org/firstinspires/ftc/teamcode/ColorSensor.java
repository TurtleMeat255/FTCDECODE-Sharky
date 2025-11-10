package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

// resource used:
// https://www.youtube.com/watch?v=pyIeknIcT8M
// coach prat color sensor

@TeleOp
public class ColorSensor extends OpMode {
    private NormalizedColorSensor colorSensor;

    public enum DetectedColor
    {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    /*
    green
    RED =
    GREEN =
    BLUE =

    purple
    RED =
    GREEN =
    BLUE =
     */

    @Override
    public void init()
    {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(20);
    }

    @Override
    public void loop() {
        DetectedColor detection = GetDetectedColor();
        telemetry.update();
    }

    private DetectedColor GetDetectedColor()
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;

        telemetry.addData("norm red", normRed);
        telemetry.addData("norm green", normGreen);
        telemetry.addData("norm blue", normBlue);

        telemetry.addData("red", colors.red);
        telemetry.addData("green", colors.green);
        telemetry.addData("blue", colors.blue);

        return DetectedColor.UNKNOWN;
    }
}
