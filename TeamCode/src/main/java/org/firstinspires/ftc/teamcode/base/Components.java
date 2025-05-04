package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.NonLinearSequentialAction;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.SleepUntilTrue;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.InstantAction;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.CompoundAction;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.NonLinearAction;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.ConditionalAction;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.PressTrigger;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.IfThen;

import java.lang.annotation.ElementType;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;

import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.ReturningFunc;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.Condition;

import org.firstinspires.ftc.teamcode.base.presets.PresetControl.ServoControl;
import org.firstinspires.ftc.teamcode.base.presets.TimeBasedLocalizers;

public abstract class Components {
    public static HardwareMap hardwareMap;
    public static Telemetry telemetry;
    public static ElapsedTime timer = new ElapsedTime(); //Central timer used by everything (e.g. sleep action, motion profile)
    public static HashMap<String,Actuator<?>> actuators = new HashMap<>(); //Map of all actuators, each accessible through its name
    @Target(ElementType.METHOD)
    public @interface Actuate{} //Used to denote methods that actually move a part, like setPower or setPosition
    public abstract static class PartsConfig{ //Classes overriding PartsConfig will have static fields that hold all the actuators for a build. Similar to Mr. Nayal's JSON files that held the components and data on each component of a build.
        //Create field of type Actuator here to hold the actuators
        public static void initialize(HardwareMap hardwareMap, Telemetry telemetry){ //Inner method to initialize hardwareMap and telemetry, common for all PartsConfigs
            Components.hardwareMap=hardwareMap;
            Components.telemetry=telemetry;
            timer.reset(); //Static variables are preserved between runs, so timer needs to be reset
        }
    }
    public abstract static class ControlFunction<E extends Actuator<?>>{ //The subclasses of this are methods that are called to control actuators and get them to the target, such as PID or motion profiles. Each function works with a specific type of actuator. Multiple can run at once
        public E parentActuator; //Each function has access to the actuator it runs on
        public boolean isStart; //Indicates if the control function has just started running
        public void registerToParent(E parentActuator){
            this.parentActuator=parentActuator;
        }
        public void run(){
            runProcedure();
            isStart=false;
        }
        protected abstract void runProcedure(); //This method is where the control function does its job
        public void stopAndReset(){stopProcedure(); isStart=true;} //The function can stop running if the control mode of the actuator is switched
        public void stopProcedure(){} //Takes care of anything that needs to occur when the control function stops
    }
    public abstract static class Actuator<E extends HardwareDevice>{ //Actuators are enhanced hardware classes that have more state and functionality. Each Actuator instance is parametrized with a specific type, like DcMotorEx or Servo/.
        public String name;
        public HashMap<String,E> parts = new HashMap<>(); public String[] partNames; //Since two hardware devices can be synchronized on one mechanism, Actuators can have multiple inner parts, each referenced by its hardwareMap name

        double target;
        public double instantTarget;
        public boolean newTarget=false; //Set to true when setTarget is called. Set to false after the end of each loop.

        double offset; //In case a part skips or something, this allows us to offset all the targets we set to compensate

        public ReturningFunc<Double> maxTargetFunc;
        public ReturningFunc<Double> minTargetFunc;
        //Max and min targets. They are dynamic functions since the max position for an actuator may not be the same. An in-game extension limit may not apply based on the direction of the actuator, for example.
        public double errorTol; //Error tolerance for when the actuator is commanded to a position
        public double defaultTimeout; //Default time waited when an actuator is commanded to a position before ending the action.
        public boolean lockActuationState = true;

        public HashMap<String,Double> keyPositions = new HashMap<>(); //Stores key positions, like 'transferPosition,' etc.

        public HashMap<String,ReturningFunc<Double>> getCurrentPositions = new HashMap<>(); //Map of methods to get the current positions of each of the actuator's parts. (They may have slightly different positions each)
        public ControlFuncRegister<?> funcRegister;
        public String currControlFuncKey;
        public String defaultControlKey;
        public Function<Double,Double> positionConversion = (Double pos)->(pos); //Allows one to apply unit conversion on the getCurrentPosition method to return it in a different unit (e.g ticks to inches)
        public Function<Double,Double> positionConversionInverse = (Double pos)->(pos);
        public boolean timeBasedLocalization; //Indicates whether the getCurrentPosition method of the actuator calculates the position based on time as opposed to an encoder, which is important to know.
        public boolean dynamicTargetBoundaries=false; //Indicates whether the max and min targets can change. Useful to know if they don't

        public class ControlFuncRegister<T extends Actuator<E>>{ //Registers control functions. Parametrized to the subclass of Actuator that is using it. The functions cannot be stored directly in the actuator because of generic type erasure and generic invariance. This approach is cleaner
            public HashMap<String, List<ControlFunction<T>>> controlFuncsMap = new HashMap<>(); //Map with lists of control functions paired with names.
            @SafeVarargs
            public ControlFuncRegister(T instance, //The function to find the current position of the actuator accepts one of the actuator's parts
                                       String[] controlFuncKeys, List<ControlFunction<T>>... controlFuncs){

                for (int i=0;i<controlFuncKeys.length;i++) {
                    for (ControlFunction<T> func : controlFuncs[i]) {
                        func.registerToParent(instance);
                    }
                    controlFuncsMap.put(controlFuncKeys[i],controlFuncs[i]);
                }
                controlFuncsMap.put("controlOff",new ArrayList<>()); //Add a control mode without any control functions.
                currControlFuncKey="controlOff";
                defaultControlKey=controlFuncKeys[0];
            }
        }
        public Actuator(String actuatorName, Class<E> type,
                        String[] partNames, Function<E, Double> getCurrentPosition,
                        ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc,
                        double errorTol, double defaultTimeout,
                        String[] keyPositionKeys,
                        double[] keyPositionValues){
            this.name=actuatorName;
            this.partNames=partNames;
            this.maxTargetFunc = ()->(maxTargetFunc.call()+offset);
            this.minTargetFunc = ()->(minTargetFunc.call()+offset);
            this.defaultTimeout = defaultTimeout;
            this.errorTol=errorTol;
            for (int i=0; i<keyPositionKeys.length; i++){
                keyPositions.put(keyPositionKeys[i],keyPositionValues[i]);
            }
            for (String name:partNames){
                this.parts.put(name,Components.hardwareMap.get(type,name)); //Can't do E.class instead of using the parameter 'type' because of generic type erasure
            }
            for (String name: partNames){
                this.getCurrentPositions.put(name,()->(positionConversion.apply(getCurrentPosition.apply(parts.get(name))))); //The getCurrentPosition function is copied, one for each of the actuator's parts. Position conversion is also applied
            }
            actuators.put(name,this);
        }
        public void setTarget(double target){
            target=target+offset;
            target=Math.max(minTargetFunc.call(),Math.min(target, maxTargetFunc.call()));
            if (target!=this.target) {
                this.target = target;
                this.instantTarget = target;
                newTarget = true;
            }
        }
        public double getTarget(){
            return target;
        }
        public double getCurrentPosition(String name){ //Gets the position of a specific part
            return Objects.requireNonNull(getCurrentPositions.get(name)).call();
        }
        public double getCurrentPosition(){ //Gets the avg position of all synchronized parts
            double avg = 0;
            for (ReturningFunc<Double> func: getCurrentPositions.values()){
                avg+=func.call();
            }
            return avg/getCurrentPositions.values().size();
        }
        public void setOffset(double offset){
            this.offset=offset;
            setTarget(target);
        }
        public void runControl(){
            for (ControlFunction<? extends Actuator<E>> func : Objects.requireNonNull(this.funcRegister.controlFuncsMap.get(currControlFuncKey))) {
                func.run();
            }
        }
        public double getPos(String key){ //Returns one of the key positions based on the inputted key
            return Objects.requireNonNull(keyPositions.get(key));
        }
        public void switchControl(String key){
            for (ControlFunction<?> func: Objects.requireNonNull(this.funcRegister.controlFuncsMap.get(currControlFuncKey))){
                func.stopAndReset();
            }
            currControlFuncKey=key;
        }
        public class SetTargetAction extends CompoundAction { //Action to set the target, then wait until the position of the actuator is a certain distance from the target, or until a set timeout
            public SetTargetAction(ReturningFunc<Double> targetFunc, double timeout){
                sequence = new NonLinearSequentialAction(
                        new InstantAction(()-> setTarget(targetFunc.call())),
                        new SleepUntilTrue(
                                ()->(Math.abs(getCurrentPosition()-target)<errorTol),
                                timeout
                        )
                );
            }
            public SetTargetAction(double target, double timeout){
                this(()->(target), timeout);
            }
            public SetTargetAction(ReturningFunc<Double> targetFunc){
                this(targetFunc, defaultTimeout);
            }
            public SetTargetAction(double target){
                this(()->(target), defaultTimeout);
            }
            @Override
            public void stopProcedure(){
                setTarget(getCurrentPosition());
            }
        }
        public class SetOffsetAction extends CompoundAction { //Action to set the offset
            public SetOffsetAction(ReturningFunc<Double> offsetFunc, double timeout){
                sequence = new NonLinearSequentialAction(
                        new InstantAction(()-> setOffset(offsetFunc.call())),
                        new SleepUntilTrue(
                                ()->(Math.abs(getCurrentPosition()-target)<errorTol),
                                timeout
                        )
                );
            }
            public SetOffsetAction(double offset, double timeout){
                this(()->(offset), timeout);
            }
            public SetOffsetAction(ReturningFunc<Double> offsetFunc){
                this(offsetFunc, defaultTimeout);
            }
            public SetOffsetAction(double offset){
                this(()->(offset), defaultTimeout);
            }
            @Override
            public void stopProcedure(){
                setTarget(getCurrentPosition());
            }
        }
        public SetTargetAction setTargetAction(double target){
            return new SetTargetAction(target);
        }
        public SetTargetAction setTargetAction(ReturningFunc<Double> targetFunc){
            return new SetTargetAction(targetFunc);
        }
        public SetTargetAction setTargetAction(double target, double timeout){
            return new SetTargetAction(target,timeout);
        }
        public SetTargetAction setTargetAction(ReturningFunc<Double> targetFunc, double timeout){
            return new SetTargetAction(targetFunc);
        }
        public SetTargetAction toggleAction(double target1, double target2){
            return setTargetAction(()->{
                if (this.target==target1) return target2; else if (this.target==target2) return target1; else return this.target;
            });
        }
        public SetTargetAction upwardFSMAction(double...targets){
            Arrays.sort(targets);
            return setTargetAction(()->{
                for (double target: targets){
                    if (this.target<target){
                        return target;
                    }
                }
                return this.target;
            });
        }
        public SetTargetAction downwardFSMAction(double...targets){
            Arrays.sort(targets);
            return setTargetAction(()->{
                for (int i = targets.length-1; i>=0; i--){
                    if (this.target>targets[i]){
                        return targets[i];
                    }
                }
                return this.target;
            });
        }
        public SetOffsetAction setOffsetAction(double offset){
            return new SetOffsetAction(offset);
        }
        public SetOffsetAction setOffsetAction(ReturningFunc<Double> offsetFunc){
            return new SetOffsetAction(offsetFunc);
        }
        public PressTrigger triggeredSetTargetAction(Condition condition, double target){
            return new PressTrigger(new IfThen(condition, new SetTargetAction(target)));
        }
        public PressTrigger triggeredSetTargetAction(Condition condition, ReturningFunc<Double> targetFunc) {
            return new PressTrigger(new IfThen(condition, setTargetAction(targetFunc)));
        }
        public PressTrigger triggeredSetTargetAction(Condition condition, double target, double timeout){
                return new PressTrigger(new IfThen(condition, setTargetAction(target,timeout)));
        }
        public PressTrigger triggeredSetTargetAction(Condition condition, ReturningFunc<Double> targetFunc, double timeout){
                return new PressTrigger(new IfThen(condition, setTargetAction(targetFunc,timeout)));
        }
        public PressTrigger triggeredToggleAction(Condition condition, double target1, double target2){
            return new PressTrigger(new IfThen(condition, toggleAction(target1,target2)));
        }
        public ConditionalAction triggeredDynamicAction(Condition upCondition, Condition downCondition, double change){
            return new ConditionalAction(new IfThen(upCondition, setTargetAction(()->(target+change))),new IfThen(downCondition, setTargetAction(()->(target-change))));
        }
        public PressTrigger triggeredFSMAction(Condition upCondition, Condition downCondition, double...targets){
            return new PressTrigger(new IfThen(upCondition, upwardFSMAction(targets)),new IfThen(downCondition, downwardFSMAction(targets)));
        }
        public PressTrigger triggeredSetOffsetAction(Condition condition, double offset){
            return new PressTrigger(new IfThen(condition, new SetOffsetAction(offset)));
        }
        public PressTrigger triggeredDynamicOffsetAction(Condition upCondition, Condition downCondition, double offsetChange){
            return new PressTrigger(new IfThen(upCondition, setOffsetAction(()->(offset+offsetChange))),new IfThen(downCondition, setOffsetAction(()->(target-offsetChange))));
        }
        public InstantAction switchControlAction(String controlKey){
            return new InstantAction(()->this.switchControl(controlKey));
        }
    }
    //Each of the subclasses of Actuator will have some generic constructors and some constructors where information is preset.
    public abstract static class CRActuator<E extends DcMotorSimple> extends Actuator<E>{ //Type of Actuator that works for continuous rotation parts, like DcMotorEx and CRServo
        HashMap<String,Double> powers = new HashMap<>(); //Stores the powers each of the parts are set to. Synchronized parts can have different powers because the load on one may be larger than on the other
        ReturningFunc<Double> maxPowerFunc;
        ReturningFunc<Double> minPowerFunc;
        //Max and min power boundaries
        public boolean dynamicPowerBoundaries=false; //Indicates if the power boundaries can change, which is useful to know
        public CRActuator(String name, Class<E> type, String[] names, Function<E, Double> getCurrentPosition, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues,
                           DcMotorSimple.Direction[] directions) {
            super(name, type, names, getCurrentPosition, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues);
            this.maxPowerFunc=maxPowerFunc;
            this.minPowerFunc=minPowerFunc;
            for (int i=0;i<names.length;i++){
                Objects.requireNonNull(parts.get(names[i])).setDirection(directions[i]);
                powers.put(names[i],0.0);
            }
            this.target=0;
        }
        public CRActuator(String name, Class<E> type, String[] names, Function<E, Double> getCurrentPosition, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues,
                          DcMotorSimple.Direction[] directions) {
            this(name,type,names,getCurrentPosition,maxTargetFunc,minTargetFunc,()->(1.0),()->(-1.0),errorTol,defaultTimeout,keyPositionKeys,keyPositionValues,directions);
        }
        public CRActuator(String name, Class<E> type, String[] names, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, DcMotorSimple.Direction[] directions) { //For CRActuators that don't set targets and only use setPower, like drivetrain motors.
            this(name,type,names,(E e)->(0.0), ()->(Double.POSITIVE_INFINITY),()->(Double.NEGATIVE_INFINITY),()->(1.0),()->(-1.0),0,0,new String[]{},new double[]{},directions);
        }
        @Actuate
        public void setPower(double power, String name){ //Sets power to a specific part
            if (lockActuationState){
                power=Math.max(Math.min(power, maxPowerFunc.call()), minPowerFunc.call());
                E part = parts.get(name);
                assert part != null;
                if (Math.abs(power-part.getPower())>0.05) {
                    part.setPower(power);
                    powers.put(name,power);
                    if (timeBasedLocalization){ //If current position is calculated by time, it needs to be updated everytime the actuator moves
                        getCurrentPosition(name);
                    }
                }
            }
        }
        @Actuate
        public void setPower(double power){ //Sets power to all synchronized parts at once
            if (lockActuationState) {
                power=Math.max(Math.min(power, maxPowerFunc.call()), minPowerFunc.call());
                if (Math.abs(power-Objects.requireNonNull(this.powers.get(partNames[0])))>0.05) {
                    for (String name:partNames) {
                        this.powers.put(name,power);
                        Objects.requireNonNull(parts.get(name)).setPower(power);
                    }
                    if (timeBasedLocalization){
                        getCurrentPosition();
                    }
                }
            }
        }
        public class SetPowerAction extends InstantAction{ //Action to set the power of all synchronized parts
            public SetPowerAction(ReturningFunc<Double> powerFunc) {
                super(()-> setPower(powerFunc.call()));
            }
            public SetPowerAction(double power) {
                super(()-> setPower(power));
            }
        }
        public SetPowerAction setPowerAction(ReturningFunc<Double> powerFunc){
            return new SetPowerAction(powerFunc);
        }
        public SetPowerAction setPowerAction(double power){
            return new SetPowerAction(power);
        }
        public SetPowerAction togglePowerAction(double power1, double power2){
            return new SetPowerAction(()->{
                if (Objects.requireNonNull(powers.get(partNames[0]))==power1) return power2; else if (Objects.requireNonNull(powers.get(partNames[0]))==power2) return power1; else return Objects.requireNonNull(powers.get(partNames[0]));
            });
        }
        public SetPowerAction upwardFSMPowerAction(double...powersList){
            Arrays.sort(powersList);
            return setPowerAction(()->{
                for (double power: powersList){
                    if (Objects.requireNonNull(powers.get(partNames[0]))<power){
                        return power;
                    }
                }
                return Objects.requireNonNull(powers.get(partNames[0]));
            });
        }
        public SetPowerAction downwardFSMPowerAction(double...powersList){
            Arrays.sort(powersList);
            return setPowerAction(()->{
                for (int i = powersList.length-1; i>=0; i--){
                    if (Objects.requireNonNull(powers.get(partNames[0]))>powersList[i]){
                        return powersList[i];
                    }
                }
                return Objects.requireNonNull(powers.get(partNames[0]));
            });
        }
        public PressTrigger triggeredSetPowerAction(Condition condition, ReturningFunc<Double> powerFunc){
            return new PressTrigger(new IfThen(condition, new SetPowerAction(powerFunc)));
        }
        public PressTrigger triggeredSetPowerAction(Condition condition, double power){
            return new PressTrigger(new IfThen(condition, new SetPowerAction(power)));
        }
        public ConditionalAction triggeredDynamicPowerAction(Condition upCondition, Condition downCondition, double change){
            return new ConditionalAction(new IfThen(upCondition, setPowerAction(()->(Objects.requireNonNull(powers.get(partNames[0]))+change))),new IfThen(downCondition, setPowerAction(()->(Objects.requireNonNull(powers.get(partNames[0]))-change))));
        }
        public PressTrigger triggeredTogglePowerAction(Condition condition, double power1, double power2){
            return new PressTrigger(new IfThen(condition, togglePowerAction(power1,power2)));
        }
        public PressTrigger triggeredFSMPowerAction(Condition upCondition, Condition downCondition, double...powers){
            return new PressTrigger(
                    new IfThen(upCondition, upwardFSMPowerAction(powers)),
                    new IfThen(downCondition, downwardFSMPowerAction(powers))
            );
        }
    }
    //Each of the bottom-level subclass constructors will accept getCurrentPosition functions and control functions, since those cater to a specific subclass.
    public static class BotMotor extends CRActuator<DcMotorEx>{
        public boolean isStallResetting;
        @SafeVarargs
        public BotMotor(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<BotMotor>>... controlFuncs) {
            super(name, DcMotorEx.class, names, (DcMotorEx motor)->((double) motor.getCurrentPosition()), maxTargetFunc, minTargetFunc, maxPowerFunc,minPowerFunc,errorTol, defaultTimeout, keyPositionKeys, keyPositionValues, directions);
            for (DcMotorEx part:parts.values()){
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            this.funcRegister=new ControlFuncRegister<BotMotor>(this,controlFuncKeys, controlFuncs);
        }
        @SafeVarargs
        public BotMotor(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<BotMotor>>... controlFuncs) {
            super(name, DcMotorEx.class, names, (DcMotorEx motor)->((double) motor.getCurrentPosition()), maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues, directions);
            for (DcMotorEx part:parts.values()){
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            this.funcRegister=new ControlFuncRegister<BotMotor>(this,controlFuncKeys, controlFuncs);
        }
        public BotMotor(String name, String[] names, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, DcMotorSimple.Direction[] directions) {
            super(name, DcMotorEx.class, names, maxPowerFunc, minPowerFunc, directions);
            for (DcMotorEx part:parts.values()){
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            this.funcRegister=new ControlFuncRegister<BotMotor>(this,new String[]{},new ArrayList<>());
        }
        public double getVelocity(){ //Returns avg velocity of all parts
            double avg=0;
            for (DcMotorEx part:parts.values()){
                avg+=part.getVelocity();
            }
            return avg/parts.size();
        }
        public void resetEncoders(){
            for (DcMotorEx part:parts.values()){
                part.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        public void setZeroPowerFloat(){
            for (DcMotorEx part:parts.values()){
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
        public void setZeroPowerBrake(){
            for (DcMotorEx part:parts.values()){
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        public double getCurrentAmps(){
            double maxCurrent=0;
            for (DcMotorEx part:parts.values()){
                double current=part.getCurrent(CurrentUnit.AMPS);
                if (current>maxCurrent){
                    maxCurrent=current;
                }
            }
            return maxCurrent;
        }
        public class StallResetAction extends NonLinearAction { //Stall resets encoders, and offsets the position if you want to reset at a non-zero position.
            double resetPosition;
            double stallVolts;
            public StallResetAction(double resetPosition, double stallVolts) {
                this.resetPosition=resetPosition;
                this.stallVolts=stallVolts;
            }
            @Override
            boolean runProcedure() {
                if (isStart){
                    isStallResetting=true;
                    setPower(-0.2);
                }
                if (getCurrentAmps()>stallVolts){
                    for (DcMotorEx part:parts.values()){
                        part.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    setOffset(-resetPosition);
                    setTarget(resetPosition);
                    setPower(0);
                    isStallResetting=false;
                }
                return isStallResetting;
            }
        }
        public StallResetAction stallResetAction(double resetPosition,double stallVolts){
            return new StallResetAction(resetPosition,stallVolts);
        }
        public PressTrigger triggeredStallResetAction(Condition condition, double resetPosition,double stallVolts){
            return new PressTrigger(new IfThen(condition,stallResetAction(resetPosition,stallVolts)));
        }
    }
    public static class BotServo extends Actuator<Servo>{
        private double currCommandedPos;
        @SafeVarargs
        public BotServo(String name, String[] names, Function<Servo, Double> getCurrentPosition, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues, Servo.Direction[] directions, double range, double initialTarget, String[] controlFuncKeys, List<ControlFunction<BotServo>>... controlFuncs) {
            super(name, Servo.class, names, getCurrentPosition, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues);
            this.positionConversion=(Double pos)->(pos*range);
            this.positionConversionInverse=(Double pos)->(pos/range);
            for (int i=0;i<directions.length;i++){
                Objects.requireNonNull(parts.get(names[i])).setDirection(directions[i]);
            }
            this.funcRegister=new ControlFuncRegister<BotServo>(this,controlFuncKeys, controlFuncs);
        }
        public BotServo(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double servoSpeed, String[] keyPositionKeys, double[] keyPositionValues, Servo.Direction[] directions, double range, double initialTarget) {
            this(name,names,new TimeBasedLocalizers.ServoTimeBasedLocalizer(servoSpeed)::getCurrentPosition,maxTargetFunc,minTargetFunc,0.01,0,keyPositionKeys,keyPositionValues,directions,range, initialTarget, new String[]{"setPos"}, new ArrayList<>(Collections.singleton(new ServoControl())));
        }
        @Actuate
        public void setPosition(double position){
            position=Math.max(minTargetFunc.call(),Math.min(position, maxTargetFunc.call()));
            if (lockActuationState && position!=currCommandedPos){
                currCommandedPos=position;
                for (Servo part:parts.values()){part.setPosition(positionConversionInverse.apply(position));}
                if (timeBasedLocalization){
                    getCurrentPosition();
                }
            }
        }
        public double getPosition(){
            return currCommandedPos;
        }
    }
    public static class CRBotServo extends CRActuator<CRServo>{
        @SafeVarargs
        public CRBotServo(String name, String[] names, Function<CRServo, Double> getCurrentPosition, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            super(name, CRServo.class, names, getCurrentPosition, maxTargetFunc, minTargetFunc, maxPowerFunc, minPowerFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues, directions);
            this.funcRegister=new ControlFuncRegister<CRBotServo>(this, controlFuncKeys, controlFuncs);
        }
        @SafeVarargs
        public CRBotServo(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, double servoSpeed, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            super(name, CRServo.class, names, new TimeBasedLocalizers.CRTimeBasedLocalizer<CRServo>(servoSpeed)::getCurrentPosition, maxTargetFunc, minTargetFunc, maxPowerFunc,minPowerFunc,0, Double.POSITIVE_INFINITY, keyPositionKeys, keyPositionValues, directions);
            this.funcRegister=new ControlFuncRegister<CRBotServo>(this, controlFuncKeys, controlFuncs);
        }
        @SafeVarargs
        public CRBotServo(String name, String[] names, Function<CRServo, Double> getCurrentPosition, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            super(name, CRServo.class, names, getCurrentPosition, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues, directions);
            this.funcRegister=new ControlFuncRegister<CRBotServo>(this, controlFuncKeys, controlFuncs);
        }
        @SafeVarargs
        public CRBotServo(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double servoSpeed, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            super(name, CRServo.class, names, new TimeBasedLocalizers.CRTimeBasedLocalizer<CRServo>(servoSpeed)::getCurrentPosition, maxTargetFunc, minTargetFunc,0.05, Double.POSITIVE_INFINITY, keyPositionKeys, keyPositionValues, directions);
            this.funcRegister=new ControlFuncRegister<CRBotServo>(this, controlFuncKeys, controlFuncs);
        }
    }
}
