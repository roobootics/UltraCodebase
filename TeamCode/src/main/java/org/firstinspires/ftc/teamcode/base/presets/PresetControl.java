package org.firstinspires.ftc.teamcode.base.presets;

import static org.firstinspires.ftc.teamcode.base.Components.timer;

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.Components.Actuator;
import org.firstinspires.ftc.teamcode.base.Components.BotServo;
import org.firstinspires.ftc.teamcode.base.Components.ControlFunction;
import org.firstinspires.ftc.teamcode.base.Components.CRActuator;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.ReturningFunc;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class PresetControl { //Holds control functions that actuators can use. Note that more control functions, like other types of motion profiling, can be coded and used.
    public static class PIDF<E extends CRActuator<?>> extends ControlFunction<E>{
        public static class PIDFConstants{ //Stores PIDF coefficients
            public double kP;
            public double kI;
            public double kD;
            public double kF;
            public ReturningFunc<Double> feedForwardFunc; //From my understanding, feedforward is just a custom way to boost power based on a certain factor. For example, it could be used to counter gravity. feedForwardFunc returns a certain metric, and that will be multiplied by kF.
            public PIDFConstants(double kP, double kI, double kD, double kF, ReturningFunc<Double> feedForwardFunc){
                this.kP=kP;
                this.kI=kI;
                this.kD=kD;
                this.kF=kF;
                this.feedForwardFunc = feedForwardFunc;
            }
            public PIDFConstants(double kP, double kI, double kD){
                this.kP=kP;
                this.kI=kI;
                this.kD=kD;
                this.kF=0;
                this.feedForwardFunc = ()->(0.0);
            }
        }
        public double[] integralSums;
        public double[] previousErrors;
        public double prevLoopTime;
        public double integralIntervalTime;
        public ArrayList<PIDFConstants> constants;
        public PIDF(PIDFConstants...constants){ //The PIDF can accept multiple sets of coefficients, since if two synchronized CR components have different loads, they will need to produce different power outputs
            this.constants=new ArrayList<>(Arrays.asList(constants));
        }
        @Override
        public void registerToParent(E actuator){
            super.registerToParent(actuator);
            if (actuator.parts.values().size()>this.constants.size()){
                for (int i=0;i<actuator.parts.values().size()-this.constants.size();i++){
                    constants.add(constants.get(constants.size()-1));
                }
            }
        }
        @Override
        protected void runProcedure() {
            if (isStart){
                prevLoopTime=timer.time();
                integralIntervalTime=timer.time();
            }
            for (int i=0;i<parentActuator.partNames.length;i++){
                double currentPosition = parentActuator.getCurrentPosition(parentActuator.partNames[i]);
                if (timer.time()-integralIntervalTime>0.1){ //Limited integration history; once every 100 ms
                    integralSums[i] += parentActuator.getTarget()-currentPosition;
                    integralIntervalTime=timer.time();
                }
                parentActuator.setPower(
                        constants.get(i).kP * parentActuator.instantTarget-currentPosition +
                                constants.get(i).kI * integralSums[i] * timer.time()-prevLoopTime +
                                constants.get(i).kD * ((parentActuator.instantTarget-currentPosition)-previousErrors[i])/(timer.time()-prevLoopTime) +
                                constants.get(i).kF * constants.get(i).feedForwardFunc.call(),
                        parentActuator.partNames[i]
                );
                previousErrors[i]=parentActuator.instantTarget-currentPosition;
            }
            prevLoopTime=timer.time();
        }
        @Override
        public void stopProcedure(){
            parentActuator.setPower(0);
        }
    }
    public static class TrapezoidalMotionProfile<E extends Actuator<?>> extends ControlFunction<E>{
        boolean newParams=true;
        boolean resetting=true;
        double firstResetPosition;
        public double currentMaxVelocity;
        public double currentAcceleration;
        public double currentDeceleration;
        public double accelDT;
        public double decelDT;
        public double cruiseDT;
        public double accelDistance;
        public double decelDistance;
        public double cruiseDistance;
        public double MAX_VELOCITY;
        public double ACCELERATION;
        public double profileStartTime;
        public double profileStartPos;
        double startVelocity;
        public TrapezoidalMotionProfile(double maxVelocity, double acceleration){
            this.MAX_VELOCITY=maxVelocity;
            this.ACCELERATION=acceleration;
        }
        public void setNewParams(double maxVelocity, double acceleration){
            this.MAX_VELOCITY=maxVelocity;
            this.ACCELERATION=acceleration;
            newParams=true;
        }
        @Override
        protected void runProcedure() {
            if (parentActuator.newTarget||newParams||isStart){
                newParams=false;
                resetting=true;
                profileStartTime=timer.time();
                firstResetPosition=parentActuator.getCurrentPosition();
            }
            //When the profile needs to be reset, it will reset not in the current, but in the next loop iteration to allow for velocity calculation and avoid an issue with loop-time discrepancies
            else if (resetting){
                resetting=false;
                createMotionProfile();
                parentActuator.instantTarget=runMotionProfileOnce();
            }
            else{
                parentActuator.instantTarget=runMotionProfileOnce();
            }
        }
        public void createMotionProfile(){
            profileStartPos=parentActuator.getCurrentPosition();
            double distance = parentActuator.getTarget() - profileStartPos;
            if (distance!=0) {
                if (parentActuator instanceof Components.BotMotor) {
                    startVelocity = ((Components.BotMotor) parentActuator).getVelocity();
                } else {
                    startVelocity = (profileStartPos - firstResetPosition) / (timer.time() - profileStartTime);
                }
                //If the actuator is a motor, we can use getVelocity to find the velocity at the start of the profile. Otherwise we calculate it manually
                currentMaxVelocity = MAX_VELOCITY * Math.signum(distance);
                currentAcceleration = ACCELERATION * Math.signum(currentMaxVelocity - startVelocity);
                currentDeceleration = -ACCELERATION * Math.signum(distance);
                accelDT = (currentMaxVelocity - startVelocity) / currentAcceleration;
                decelDT = (0 - currentMaxVelocity) / currentDeceleration;
                accelDistance = startVelocity * accelDT + 0.5 * currentAcceleration * accelDT * accelDT;
                decelDistance = currentMaxVelocity * decelDT + 0.5 * currentDeceleration * decelDT * decelDT;
                cruiseDistance = Math.abs(distance - accelDistance - decelDistance) * Math.signum(currentMaxVelocity);
                if (Math.abs(accelDistance + cruiseDistance + decelDistance) > Math.abs(distance)) {
                    double halfExceededDistance = (distance - accelDistance - decelDistance) / 2;
                    accelDistance = accelDistance + halfExceededDistance;
                    accelDT = Math.max(
                            (-startVelocity + Math.sqrt(Math.abs(startVelocity * startVelocity + 2 * currentAcceleration * accelDistance))) / (currentAcceleration),
                            (-startVelocity - Math.sqrt(Math.abs(startVelocity * startVelocity + 2 * currentAcceleration * accelDistance))) / (currentAcceleration)
                    );
                    currentMaxVelocity = currentAcceleration * accelDT + startVelocity;
                    decelDistance = decelDistance + halfExceededDistance;
                    decelDT = Math.max(
                            (-currentMaxVelocity + Math.sqrt(Math.abs(currentMaxVelocity * currentMaxVelocity + 2 * currentDeceleration * decelDistance))) / (currentDeceleration),
                            (-currentMaxVelocity - Math.sqrt(Math.abs(currentMaxVelocity * currentMaxVelocity + 2 * currentDeceleration * decelDistance))) / (currentDeceleration)
                    );
                }
                cruiseDistance = Math.abs(distance - accelDistance - decelDistance) * Math.signum(currentMaxVelocity);
                cruiseDT = cruiseDistance / currentMaxVelocity;
                if (Double.isNaN(accelDT) || Double.isNaN(accelDistance) || Double.isNaN(decelDT) || Double.isNaN(decelDistance) || Double.isNaN(cruiseDT) || Double.isNaN(cruiseDistance) || accelDT < 0 || decelDT < 0 || cruiseDT < 0) {
                    accelDT = 0;
                    cruiseDT = 0;
                    decelDT = 0;
                    accelDistance = 0;
                    cruiseDistance = 0;
                    decelDistance = 0;
                }
            }
            else{
                accelDT=0;
                cruiseDT=0;
                decelDT=0;
                accelDistance=0;
                cruiseDistance=0;
                decelDistance=0;
            }
        }
        public double runMotionProfileOnce(){
            double elapsedTime = timer.time()-profileStartTime;
            if (elapsedTime < accelDT){
                return profileStartPos + startVelocity * elapsedTime + 0.5 * currentAcceleration * elapsedTime*elapsedTime;
            }
            else if (elapsedTime < accelDT+cruiseDT){
                double cruiseCurrentDT = elapsedTime - accelDT;
                return profileStartPos + accelDistance + currentMaxVelocity * cruiseCurrentDT;
            }

            else if (elapsedTime < accelDT+cruiseDT+decelDT){
                double decelCurrentDT = elapsedTime - accelDT - cruiseDT;
                return profileStartPos + accelDistance + cruiseDistance + currentMaxVelocity * decelCurrentDT + 0.5 * currentDeceleration * decelCurrentDT*decelCurrentDT;
            }
            else{
                return parentActuator.getTarget();
            }
        }
    }


    public static class ServoControl extends ControlFunction<BotServo>{ //Control function to get servos to their targets
        @Override
        protected void runProcedure() {
            parentActuator.setPosition(parentActuator.instantTarget);
        }
    }
    public static class CRBangBangControl<E extends CRActuator<?>> extends ControlFunction<E>{ //Likely will be used to get CRServos to their targets if they have no encoders with them
        ReturningFunc<Double> powerFunc; //This control function moves the CRActuator to the target at a given power, which can change. That is stored here.
        public CRBangBangControl(double power){
            this.powerFunc=()->(power);
        }
        public CRBangBangControl(ReturningFunc<Double> powerFunc){
            this.powerFunc=powerFunc;
        }
        @Override
        protected void runProcedure() {
            double currentPosition = parentActuator.getCurrentPosition();
            if (Math.abs(parentActuator.instantTarget-currentPosition)>1){
                parentActuator.setPower(powerFunc.call()*Math.signum(parentActuator.instantTarget-currentPosition));
            }
        }
    }
}
