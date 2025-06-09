package org.firstinspires.ftc.teamcode.base.programs;

import static org.firstinspires.ftc.teamcode.base.Components.timer;

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.presets.PresetControl;

public class testMotionProfile {
    public static void main(String[] args){
        timer.reset();
        PresetControl.TrapezoidalMotionProfile<Components.BotMotor> profile = new PresetControl.TrapezoidalMotionProfile<>(10000,5000);
        double targetposition=0;
        double target=20000;
        double prevTime=0;
        boolean switched=false;
        profile.createMotionProfile(target);
        while (timer.time()<4){
            double time=timer.time();
            if (targetposition>10000 && !switched){
                target=0;
                profile.createMotionProfile(target);
            }
            targetposition=profile.runMotionProfileOnce();
            if (time-prevTime>0.005){
                prevTime=time;
                System.out.print(profile.getPhase()); System.out.print("     "); System.out.print(timer.time()); System.out.print("     "); System.out.print(targetposition); System.out.print("     "); System.out.print(target); System.out.print("     "); System.out.print(profile.getProfileData());
                System.out.println();
                System.out.println();
            }
        }
    }
}
