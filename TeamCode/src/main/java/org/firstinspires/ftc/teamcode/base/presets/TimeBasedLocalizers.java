package org.firstinspires.ftc.teamcode.base.presets;

import static org.firstinspires.ftc.teamcode.base.Components.timer;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.Components;

public abstract class TimeBasedLocalizers{
    public static class ServoTimeBasedLocalizer{ //Calculates position of a servo based on time
        public double ABS_SERVO_SPEED;
        public double prevPosition;
        public double prevTime;
        public double prevTarget;
        public Components.BotServo actuator;
        public ServoTimeBasedLocalizer(double servoSpeed,Components.BotServo actuator){
            this.ABS_SERVO_SPEED=servoSpeed;
            this.actuator=actuator;
        }
        public double getCurrentPosition(Servo servo){
            if (!Double.isNaN(servo.getPosition())){
                double servoSpeed = Math.signum(prevTarget-prevPosition)*ABS_SERVO_SPEED;
                double time=timer.time();
                double change = actuator.positionConversionInverse.apply(servoSpeed)*(time-prevTime);
                if (Math.signum(change)==1){
                    prevPosition=Math.min(prevTarget,prevPosition+change);
                }
                else if (Math.signum(change)==-1){
                    prevPosition=Math.max(prevTarget,prevPosition+change);
                }
                prevTime=time;
                prevTarget=actuator.positionConversionInverse.apply(actuator.getPosition());
            }
            return prevPosition;
        }
    }
    public static class CRTimeBasedLocalizer<E extends DcMotorSimple>{ //Calculates position of a continuous rotation actuator based on time
        public double ABS_SERVO_SPEED;
        public double prevPosition;
        public double prevTime;
        public double prevPower;
        public CRTimeBasedLocalizer(double servoSpeed){
            this.ABS_SERVO_SPEED=servoSpeed;
        }
        public double getCurrentPosition(E actuator){
            double servoSpeed = prevPower*ABS_SERVO_SPEED;
            double time=timer.time();
            prevPosition=prevPosition+servoSpeed*(time-prevTime);
            prevTime=time;
            prevPower=actuator.getPower();
            return prevPosition;
        }
    }
}
