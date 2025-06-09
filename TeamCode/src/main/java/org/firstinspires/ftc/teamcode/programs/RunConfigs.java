package org.firstinspires.ftc.teamcode.programs;

public abstract class RunConfigs {
    /*
    public static class TestServo extends PartsConfig {
        public static BotServo testServo;
        public static Components.BotMotor testMotor;
        public static PresetControl.TrapezoidalMotionProfile<Components.BotMotor> profile;
        public static void init(HardwareMap hardwareMap, Telemetry telemetry){
            initialize(hardwareMap, telemetry);
            testServo=new BotServo(
                    "test", new String[]{"test","test1"},
                    ()->(180.0),()->(0.0),
                    700, 2,
                    new Servo.Direction[]{Servo.Direction.FORWARD, Servo.Direction.REVERSE},
                    180,
                    0
            );
            testServo.setOffsetBoundFuncs(()->(180.0),()->(0.0));
            profile=new PresetControl.TrapezoidalMotionProfile<>(10000,2000);
            testMotor=new Components.BotMotor(
                    "motor", new String[]{"intake"},
                    ()->(Double.POSITIVE_INFINITY),()->(Double.NEGATIVE_INFINITY),
                    20,3,
                    new DcMotor.Direction[]{DcMotor.Direction.FORWARD},
                    new String[]{"MotionProfile","PID"},
                    Arrays.asList(profile,new PresetControl.PIDF<>(profile,new PresetControl.PIDF.PIDFConstants(0.015, 0, 0))),
                    List.of(new PresetControl.PIDF<>(profile,new PresetControl.PIDF.PIDFConstants(0.015, 0, 0)))
            );
        }
    }
    */
}
