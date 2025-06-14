package org.firstinspires.ftc.teamcode.presets;

import static org.firstinspires.ftc.teamcode.base.Components.timer;

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.Components.Actuator;
import org.firstinspires.ftc.teamcode.base.Components.BotServo;
import org.firstinspires.ftc.teamcode.base.Components.ControlFunction;
import org.firstinspires.ftc.teamcode.base.Components.CRActuator;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.ReturningFunc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Objects;

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
        private double[] integralSums = new double[]{};
        private double[] previousErrors = new double[]{};
        private double prevLoopTime;
        private double integralIntervalTime;
        private final ArrayList<PIDFConstants> constants;
        private ReturningFunc<Integer> shouldApplyDerivative = ()->(1);
        public PIDF(PIDFConstants...constants){ //The PIDF can accept multiple sets of coefficients, since if two synchronized CR components have different loads, they will need to produce different power outputs
            this.constants=new ArrayList<>(Arrays.asList(constants));
        }
        public PIDF(TrapezoidalMotionProfile<?> profile, PIDFConstants...constants){
            this(constants);
            shouldApplyDerivative=()->{
                if (profile.getPhase().equals(TrapezoidalMotionProfile.Phase.IDLE) || profile.getPhase().equals(TrapezoidalMotionProfile.Phase.OFF) || profile.getPhase().equals(TrapezoidalMotionProfile.Phase.DECEL)){
                    return 1;
                }
                else{return 0;}
            };
        }
        @Override
        public void registerToParent(E actuator){
            super.registerToParent(actuator);
            if (actuator.parts.values().size()>this.constants.size()){
                for (int i=0;i<actuator.parts.values().size()-this.constants.size();i++){
                    constants.add(constants.get(constants.size()-1));
                }
            }
            integralSums=new double[parentActuator.partNames.length];
            previousErrors=new double[parentActuator.partNames.length];
        }
        @Override
        protected void runProcedure() {
            if (isStart()){
                prevLoopTime=timer.time();
                integralIntervalTime=timer.time();
            }
            if (isStart()||parentActuator.isNewTarget()){
                Arrays.setAll(integralSums,(int i)->(0.0));
            }
            for (int i=0;i<parentActuator.partNames.length;i++){
                double currentPosition = parentActuator.getCurrentPosition(parentActuator.partNames[i]);
                if (timer.time()-integralIntervalTime>0.1){ //Limited integration history; once every 100 ms
                    integralSums[i] += parentActuator.getTarget()-currentPosition;
                    integralIntervalTime=timer.time();
                }
                double dTerm;
                if (!(parentActuator instanceof Components.BotMotor)){
                    dTerm=shouldApplyDerivative.call() * ((parentActuator.getInstantTarget() - currentPosition) - previousErrors[i])/(timer.time()-prevLoopTime);
                }
                else{
                    dTerm=shouldApplyDerivative.call() * -((Components.BotMotor) parentActuator).getVelocity(parentActuator.partNames[i]);
                }
                parentActuator.setPower(
                        constants.get(i).kP * (parentActuator.getInstantTarget()-currentPosition) +
                                constants.get(i).kI * integralSums[i] * (timer.time()-prevLoopTime) +
                                constants.get(i).kD * dTerm +
                                constants.get(i).kF * constants.get(i).feedForwardFunc.call(),
                        parentActuator.partNames[i]
                );
                previousErrors[i]=parentActuator.getInstantTarget()-currentPosition;
            }
            prevLoopTime=timer.time();
        }
        @Override
        public void stopProcedure(){
            parentActuator.setPower(0);
        }
    }
    public static class TrapezoidalMotionProfile<E extends Actuator<?>> extends ControlFunction<E>{
        public enum Phase{
            ACCEL,
            CRUISE,
            DECEL,
            IDLE,
            OFF
        }
        private double currentTarget;
        private boolean newParams=true;
        private double currentMaxVelocity;
        private double currentAcceleration;
        private double currentDeceleration;
        private double accelDT;
        private double decelDT;
        private double cruiseDT;
        private double accelDistance;
        private double decelDistance;
        private double cruiseDistance;
        private double MAX_VELOCITY;
        private double ACCELERATION;
        private double lastLoopTime=0;
        private double profileStartPos;
        private double startVelocity;
        private double targetVelocity;
        private double instantTarget;
        private Phase phase = Phase.IDLE;
        private double elapsedTime;
        public TrapezoidalMotionProfile(double maxVelocity, double acceleration){
            this.MAX_VELOCITY=maxVelocity;
            this.ACCELERATION=acceleration;
        }
        @Override
        public void registerToParent(E actuator){
            super.registerToParent(actuator);
            currentTarget=actuator.getTarget();
        }
        public void setNewParams(double maxVelocity, double acceleration){
            this.MAX_VELOCITY=maxVelocity;
            this.ACCELERATION=acceleration;
            newParams=true;
        }
        @Override
        protected void runProcedure() {
            if (parentActuator.isNewTarget()||newParams||isStart()){
                if (isStart()){
                    instantTarget=parentActuator.getCurrentPosition();
                }
                newParams=false;
                createMotionProfile(parentActuator.getTarget());
            }
            parentActuator.setInstantTarget(runMotionProfileOnce());
        }
        public void createMotionProfile(double target){
            elapsedTime=0;
            currentTarget = target;
            profileStartPos = instantTarget;
            double distance = target - profileStartPos;
            if (distance!=0) {
                startVelocity=targetVelocity;
                currentMaxVelocity = MAX_VELOCITY * Math.signum(distance);
                currentAcceleration = ACCELERATION * Math.signum(currentMaxVelocity - startVelocity);
                currentDeceleration = -ACCELERATION * Math.signum(distance);
                accelDT = (currentMaxVelocity - startVelocity) / currentAcceleration;
                decelDT = (0 - currentMaxVelocity) / currentDeceleration;
                accelDistance = startVelocity * accelDT + 0.5 * currentAcceleration * accelDT * accelDT;
                decelDistance = currentMaxVelocity * decelDT + 0.5 * currentDeceleration * decelDT * decelDT;
                cruiseDistance = Math.abs(distance - accelDistance - decelDistance) * Math.signum(currentMaxVelocity);
                if (Math.abs(accelDistance + cruiseDistance + decelDistance) > Math.abs(distance)) {
                    cruiseDistance=0;
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
            double time=timer.time();
            elapsedTime+=time-lastLoopTime;
            lastLoopTime=time;
            if (elapsedTime < accelDT){
                phase=Phase.ACCEL;
                targetVelocity = startVelocity + currentAcceleration * elapsedTime;
                instantTarget = profileStartPos + startVelocity * elapsedTime + 0.5 * currentAcceleration * elapsedTime*elapsedTime;
            }
            else if (elapsedTime < accelDT+cruiseDT){
                phase=Phase.CRUISE;
                double cruiseCurrentDT = elapsedTime - accelDT;
                targetVelocity = currentMaxVelocity;
                instantTarget = profileStartPos + accelDistance + currentMaxVelocity * cruiseCurrentDT;
            }
            else if (elapsedTime < accelDT+cruiseDT+decelDT){
                phase=Phase.DECEL;
                double decelCurrentDT = elapsedTime - accelDT - cruiseDT;
                targetVelocity = currentMaxVelocity + currentDeceleration * decelCurrentDT;
                instantTarget = profileStartPos + accelDistance + cruiseDistance + currentMaxVelocity * decelCurrentDT + 0.5 * currentDeceleration * decelCurrentDT*decelCurrentDT;
            }
            else{
                phase=Phase.IDLE;
                targetVelocity=0;
                instantTarget = currentTarget;
            }
            return instantTarget;
        }
        @Override
        public void stopProcedure() {
            phase=Phase.OFF; targetVelocity=0; instantTarget=0;
        }
        public HashMap<String,Double> getProfileData(){
            HashMap<String,Double> data = new HashMap<>();
            data.put("currentMaxVelocity",currentMaxVelocity);
            data.put("accelDistance",accelDistance);
            data.put("decelDistance",decelDistance);
            data.put("cruiseDistance",cruiseDistance);
            data.put("accelDT",accelDT);
            data.put("cruiseDT",cruiseDT);
            data.put("decelDT",decelDT);
            data.put("targetVelocity",targetVelocity);
            data.put("elapsedTime",elapsedTime);
            return data;
        }
        public double getProfileValue(String label){
            return Objects.requireNonNull(getProfileData().get(label));
        }
        public Phase getPhase(){
            return phase;
        }
    }


    public static class ServoControl extends ControlFunction<BotServo>{ //Control function to get servos to their targets
        @Override
        protected void runProcedure() {
            parentActuator.setPosition(parentActuator.getInstantTarget());
        }
    }
    public static class CRBangBangControl<E extends CRActuator<?>> extends ControlFunction<E>{ //Likely will be used to get CRServos to their targets if they have no encoders with them
        private final ReturningFunc<Double> powerFunc; //This control function moves the CRActuator to the target at a given power, which can change. That is stored here.
        public CRBangBangControl(double power){
            this.powerFunc=()->(power);
        }
        public CRBangBangControl(ReturningFunc<Double> powerFunc){
            this.powerFunc=powerFunc;
        }
        @Override
        protected void runProcedure() {
            double currentPosition = parentActuator.getCurrentPosition();
            if (Math.abs(parentActuator.getInstantTarget()-currentPosition)>parentActuator.getErrorTol()){
                parentActuator.setPower(powerFunc.call()*Math.signum(parentActuator.getInstantTarget()-currentPosition));
            }
        }
    }
}
