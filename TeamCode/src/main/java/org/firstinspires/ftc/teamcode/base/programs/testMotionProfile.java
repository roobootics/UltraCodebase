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
        double prevTime=0;
        while (timer.time()<2){
            double time=timer.time();
            target+=0.1;
            position=profile.runMotionProfileOnce();
            profile.createMotionProfile(target,position);
            if (time-prevTime>0.005){
                prevTime=time;
                System.out.print(profile.getPhase()); System.out.print("     "); System.out.print(timer.time()); System.out.print("     "); System.out.print(position); System.out.print("     "); System.out.print(target); System.out.print("     "); System.out.print(profile.getProfileData());
                System.out.println();
                System.out.println();
            }
        }
    }
}
