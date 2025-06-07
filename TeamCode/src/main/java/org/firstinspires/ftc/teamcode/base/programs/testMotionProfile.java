package org.firstinspires.ftc.teamcode.base.programs;

import static org.firstinspires.ftc.teamcode.base.Components.timer;

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.presets.PresetControl;

public class testMotionProfile {
    public static void main(String[] args){
        timer.reset();
        PresetControl.TrapezoidalMotionProfile<Components.BotMotor> profile = new PresetControl.TrapezoidalMotionProfile<>(10000,5000);
        double position=0;
        double target=0;
        while (timer.time()<10){
            target+=0.1;
            position=profile.runMotionProfileOnce();
            profile.createMotionProfile(target,position);
            System.out.print(timer.time()); System.out.print("     "); System.out.print(position); System.out.print("     "); System.out.print(target); System.out.print("     "); System.out.print(profile.getProfileData());
        }
    }
}
