package org.firstinspires.ftc.teamcode.base.programs;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.Components.PartsConfig;
import org.firstinspires.ftc.teamcode.base.Components.BotServo;

public abstract class RunConfigs {
    public static class TestServo extends PartsConfig {
        public BotServo testServo;
        @Override
        public void initParts() {
            testServo=new BotServo(
                    "test", new String[]{"test"},
                    ()->(0.0),()->(270.0),
                    422, new String[]{},new double[]{},
                    new Servo.Direction[]{Servo.Direction.FORWARD},
                    270,
                    0
            );
        }
    }
}
