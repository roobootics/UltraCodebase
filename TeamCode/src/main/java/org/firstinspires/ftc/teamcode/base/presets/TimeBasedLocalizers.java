package org.firstinspires.ftc.teamcode.base.presets;

import static org.firstinspires.ftc.teamcode.base.Components.timer;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
public abstract class TimeBasedLocalizers{
    public static class ServoTimeBasedLocalizer{ //Calculates position of a servo based on time
        public double ABS_SERVO_SPEED;
        public double prevPosition;
        public double prevTime;
        public ServoTimeBasedLocalizer(double servoSpeed){
            this.ABS_SERVO_SPEED=servoSpeed;
        }
        public double getCurrentPosition(Servo servo){
            if (!Double.isNaN(servo.getPosition())){
                double servoSpeed = Math.signum(servo.getPosition()-prevPosition)*ABS_SERVO_SPEED;
                double time=timer.time();
                double change = servoSpeed*(time-prevTime);
                if (Math.signum(change)==1){
                    prevPosition=Math.min(servo.getPosition(),prevPosition+change);
                }
                else if (Math.signum(change)==-1){
                    prevPosition=Math.max(servo.getPosition(),prevPosition+change);
                }
                prevTime=time;
            }
            return prevPosition;
        }
    }
    public static class CRTimeBasedLocalizer<E extends DcMotorSimple>{ //Calculates position of a continuous rotation actuator based on time
        public double ABS_SERVO_SPEED;
        public double prevPosition;
        public double prevTime;
        public CRTimeBasedLocalizer(double servoSpeed){
            this.ABS_SERVO_SPEED=servoSpeed;
        }
        public double getCurrentPosition(E actuator){
            double servoSpeed = actuator.getPower()*ABS_SERVO_SPEED;
            double time=timer.time();
            prevPosition=prevPosition+servoSpeed*(time-prevTime);
            prevTime=time;
            return prevPosition;
        }
    }
}
