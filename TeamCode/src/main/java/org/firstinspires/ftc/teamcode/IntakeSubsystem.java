package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final Motor intakeMotor;
    private final double INTAKE_SPEED = 0.6;

    public IntakeSubsystem(HardwareMap hardwareMap, String intake) {
        intakeMotor = new Motor(hardwareMap, intake);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
    }

    public void intake() {
        intakeMotor.set(INTAKE_SPEED);
    }

    public void outtake() {
        intakeMotor.set(-INTAKE_SPEED);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}
