package org.firstinspires.ftc.teamcode.base.programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.base.presets.GenericPositionFinder;
@TeleOp
public class TestServoPositionFinder extends GenericPositionFinder {
    @Override
    public void runOpMode() {
        RunConfigs.TestServo.init(hardwareMap,telemetry);
        super.runOpMode();
    }
}
