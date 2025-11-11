package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

// resource used:
// https://www.youtube.com/watch?v=pyIeknIcT8M
// coach prat color sensor

public class ColorSensor{
    private NormalizedColorSensor colorSensor;

    HSVColor purpleMin = new HSVColor(190, 0.45, 0.18);
    HSVColor purpleMax = new HSVColor(240, 1, 1);

    HSVColor greenMin = new HSVColor(150, 0.45, 0.18);
    HSVColor greenMax = new HSVColor(175, 1, 1);

    public enum DetectedColor
    {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public static class HSVColor
    {
        public double hue;
        double saturation;
        double value;
        public HSVColor(double hue, double saturation, double value)
        {
            this.hue = hue;
            this.saturation = saturation;
            this.value = value;
        }

    }

    public void init(HardwareMap hwMap)
    {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(20);
    }

    public DetectedColor GetDetectedColor()
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;

        float[] hsvValues = {0F, 0F, 0F};
        android.graphics.Color.RGBToHSV(
                (int) (normRed * 255),
                (int) (normGreen * 255),
                (int) (normBlue * 255),
                hsvValues
        );

        if (hsvValues[0] > purpleMin.hue && hsvValues[0] < purpleMax.hue &&
            hsvValues[1] > purpleMin.saturation && hsvValues[2] > purpleMin.value)
        {
            return DetectedColor.PURPLE;
        }

        if (hsvValues[0] > greenMin.hue && hsvValues[0] < greenMax.hue &&
                hsvValues[1] > greenMin.saturation && hsvValues[2] > greenMin.value)
        {
            return DetectedColor.GREEN;
        }

        return DetectedColor.UNKNOWN;
    }
}
