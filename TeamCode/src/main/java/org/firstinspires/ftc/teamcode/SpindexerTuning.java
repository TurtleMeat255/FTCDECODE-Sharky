package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp
public class SpindexerTuning extends OpMode {

    Spinindexer spindexer = new Spinindexer();

    @Override
    public void init() {
        spindexer.init(hardwareMap);
    }

    @Override
    public void loop() {

    }
}