package org.firstinspires.ftc.teamcode.base.programs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Components.PartsConfig;
import org.firstinspires.ftc.teamcode.base.Components.BotServo;

public abstract class RunConfigs {
    public static class TestServo extends PartsConfig {
        public static BotServo testServo;
        public static void init(HardwareMap hardwareMap, Telemetry telemetry){
            initialize(hardwareMap, telemetry);
            testServo=new BotServo(
                    "test", new String[]{"test"},
                    ()->(270.0),()->(0.0),
                    270, 5,new String[]{},new double[]{},
                    new Servo.Direction[]{Servo.Direction.FORWARD},
                    270,
                    0
            );
        }
    }
}
