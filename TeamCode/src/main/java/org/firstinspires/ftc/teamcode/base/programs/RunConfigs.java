package org.firstinspires.ftc.teamcode.base.programs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.Components.PartsConfig;
import org.firstinspires.ftc.teamcode.base.Components.BotServo;
import org.firstinspires.ftc.teamcode.base.presets.PresetControl;

import java.util.Arrays;
import java.util.List;

public abstract class RunConfigs {
    public static class TestServo extends PartsConfig {
        public static BotServo testServo;
        public static Components.BotMotor testMotor;
        public static void init(HardwareMap hardwareMap, Telemetry telemetry){
            initialize(hardwareMap, telemetry);
            testServo=new BotServo(
                    "test", new String[]{"test"},
                    ()->(180.0),()->(0.0),
                    700, 2,
                    new Servo.Direction[]{Servo.Direction.FORWARD},
                    180,
                    0
            );
            testServo.setOffsetBoundFuncs(()->(180.0),()->(0.0));
            PresetControl.TrapezoidalMotionProfile<Components.BotMotor> profile = new PresetControl.TrapezoidalMotionProfile<>(10000,5000);
            testMotor=new Components.BotMotor(
                    "motor", new String[]{"intake"},
                    ()->(1500.0),()->(0.0),
                    20,3,
                    new DcMotor.Direction[]{DcMotor.Direction.FORWARD},
                    new String[]{"MotionProfile","PID"},
                    Arrays.asList(profile,new PresetControl.PIDF<>(profile,new PresetControl.PIDF.PIDFConstants(0.015, 0, 0))),
                    List.of(new PresetControl.PIDF<>(profile,new PresetControl.PIDF.PIDFConstants(0.015, 0, 0)))
            );
        }
    }
}
