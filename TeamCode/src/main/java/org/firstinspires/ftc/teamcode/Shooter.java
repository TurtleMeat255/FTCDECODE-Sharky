package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class Shooter {
DcMotor Shooter1;
DcMotor Shooter2;
Servo Hood1;
Servo Hood2;
boolean LastUp = false;
boolean LastDown = false;
double position = 0;
public void init(HardwareMap hwMap) {
    Shooter1 = hwMap.get(DcMotor.class, "Shooter1"); //
    Shooter2 = hwMap.get(DcMotor.class, "Shooter2");
    Hood1 = hwMap.get(Servo.class, "Hood1");
    Hood2 = hwMap.get(Servo.class, "Hood2");
    Shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
    Hood1.setDirection(Servo.Direction.REVERSE);
    }
    public void ShootStuff(boolean Shoot) {
        if (Shoot) {
        Shooter1.setPower(1);
        } else {
        Shooter1.setPower(0);
        }
    }
    public void HoodStuff(boolean HoodUp, boolean HoodDown) {
    if (HoodUp && !LastUp && position <= 0.6) {
        position += 0.2;
    }
    if (HoodDown && !LastDown && position >= 0.2) {
        position -= 0.2;
    }
    // Random comment

    LastDown = HoodDown;
    LastUp = HoodUp;
    Hood1.setPosition(position);
    Hood2.setPosition(position);
    }


}



















// 67