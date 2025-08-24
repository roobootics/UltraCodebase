package org.firstinspires.ftc.teamcode.base;

import static org.firstinspires.ftc.teamcode.base.NonLinearActions.executor;

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
import org.firstinspires.ftc.teamcode.base.NonLinearActions.RunResettingLoop;

import java.lang.annotation.ElementType;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;

import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.ReturningFunc;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.Condition;

import org.firstinspires.ftc.teamcode.presets.PresetControl.ServoControl;
import org.firstinspires.ftc.teamcode.presets.TimeBasedLocalizers;

public abstract class Components {
    private static HardwareMap hardwareMap;
    public static HardwareMap getHardwareMap(){
        return hardwareMap;
    }
    private static Telemetry telemetry;
    private static final LinkedHashMap<String,Object> telemetryOutput=new LinkedHashMap<>();
    private static LinkedHashMap<String,Object> prevTelemetryOutput = new LinkedHashMap<>();
    public static void telemetryAddData(String caption, Object data){
        telemetryOutput.put(caption, data);
    }
    public static void telemetryAddLine(String line){
        telemetryOutput.put(line,null);
    }
    public static void updateTelemetry(){
        if (!prevTelemetryOutput.equals(telemetryOutput)){
            prevTelemetryOutput=new LinkedHashMap<>(telemetryOutput);
            for (String caption: telemetryOutput.keySet()){
                if (Objects.isNull(telemetryOutput.get(caption))){
                    telemetry.addLine(caption);
                }
                else{
                    telemetry.addData(caption,telemetryOutput.get(caption));
                }
            }
            telemetry.update();
        }
        telemetryOutput.clear();
    }
    public static final ElapsedTime timer = new ElapsedTime(); //Central timer used by everything (e.g. sleep action, motion profile)
    public static final HashMap<String,Actuator<?>> actuators = new HashMap<>(); //Map of all actuators, each accessible through its name
    public static void activateActuatorControl(){
        for (Actuator<?> actuator: actuators.values()){
            actuator.switchControl(actuator.getDefaultControlKey());
        }
    }
    public static void resetMotorEncoders(){
        for (Actuator<?> actuator: actuators.values()){
            if (actuator instanceof BotMotor){
                BotMotor castedActuator=(BotMotor) actuator;
                castedActuator.resetEncoders();
            }
        }
    }
    public static class CachedReader<E>{
        //Allows for the optimized reading of values. The return of a read is cached and re-returned every time the read is called, until the cache is cleared so fresh values can be obtained.
        private static final ArrayList<CachedReader<?>> readers = new ArrayList<>(); //Contains all instances
        private final ReturningFunc<E> read;
        private int resetCacheCounter = 0;
        private final int resetCacheLoopInterval; //If this is set to n, the cache is reset every nth iteration.
        private E storedReadValue = null;
        public CachedReader(ReturningFunc<E> read, int resetCacheLoopInterval){
            this.read=read;
            this.resetCacheLoopInterval = resetCacheLoopInterval;
            readers.add(this);
        }
        public E cachedRead(){
            if (Objects.isNull(storedReadValue)){
                storedReadValue=read.call();
            }
            return storedReadValue;
        }
        public void resetCache(){
            resetCacheCounter=0;
            storedReadValue=null;
        }
        protected static void resetAllCaches(){
            for (CachedReader<?> reader : readers){
                reader.resetCacheCounter+=1;
                if (reader.resetCacheCounter>= reader.resetCacheLoopInterval){
                    reader.resetCache();
                }
            }
        }
    }
    @Target(ElementType.METHOD)
    public @interface Actuate{} //Used to denote methods that actually move a part, like setPower or setPosition
    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry){ //Method to initialize hardwareMap and telemetry.
        Components.hardwareMap=hardwareMap;
        Components.telemetry=telemetry;
        timer.reset(); //Static variables are preserved between runs, so timer needs to be reset
        actuators.clear(); //Static variables are preserved between runs, so its better for actuators to be cleared
        telemetryOutput.clear();
        prevTelemetryOutput.clear();
        executor.clearActions();
        CachedReader.readers.clear();
    }
    public abstract static class ControlFunction<E extends Actuator<?>>{ //The subclasses of this are methods that are called to control actuators and get them to the target, such as PID or motion profiles. Each function works with a specific type of actuator. Multiple can run at once
        protected E parentActuator; //Each function has access to the actuator it runs on
        private boolean isStart; //Indicates if the control function has just started running
        public boolean isStart(){
            return isStart;
        }
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
        private final String name;
        public final HashMap<String,E> parts = new HashMap<>(); public final String[] partNames; //Since two hardware devices can be synchronized on one mechanism, Actuators can have multiple inner parts, each referenced by its hardwareMap name
        private double target;
        private double instantTarget;
        private boolean newTarget=false; //Set to true when setTarget is called. Set to false after the end of each loop.
        private boolean newActuation=false; //Set to true when a method tagged with @Actuator is called. Set to false after the end of each loop.
        private double offset; //In case a part skips or something, this allows us to offset all the targets we set to compensate
        public final ReturningFunc<Double> maxTargetFunc;
        public final ReturningFunc<Double> minTargetFunc;
        //Max and min targets. They are dynamic functions since the max position for an actuator may not be the same. An in-game extension limit may not apply based on the direction of the actuator, for example.
        private ReturningFunc<Double> maxOffsetFunc = ()->(Double.POSITIVE_INFINITY);
        private ReturningFunc<Double> minOffsetFunc = ()->(Double.NEGATIVE_INFINITY);
        //Max and min offsets. They are dynamic functions since the max position for an actuator may not be the same.
        private final double errorTol; //Error tolerance for when the actuator is commanded to a position
        private final double defaultMovementTimeout; //Default time waited when an actuator is commanded to a position before ending the action.
        protected boolean actuationStateUnlocked = true; //If set to false, methods tagged with @Actuate should not have an effect; it locks the actuator in whatever power/position state it's in.
        private boolean targetStateUnlocked = true; //If set to false, the actuator's target cannot change.
        private final HashMap<String,Double> keyPositions = new HashMap<>(); //Stores key positions, like 'transferPosition,' etc.
        private final HashMap<String,ReturningFunc<Double>> getCurrentPositions = new HashMap<>(); //Map of methods to get the current positions of each of the actuator's parts. (They may have slightly different positions each)
        private final LambdaInterfaces.Procedure resetCurrentPositionCaches;
        protected ControlFuncRegister<?> funcRegister;
        private String currControlFuncKey;
        private String defaultControlKey;
        private boolean timeBasedLocalization = false; //Indicates whether the getCurrentPosition method of the actuator calculates the position based on time as opposed to an encoder, which is important to know.
        private boolean isBroken = false; //Flag for whether the actuator is broken or not
        public class ControlFuncRegister<T extends Actuator<E>>{ //Registers control functions. Parametrized to the subclass of Actuator that is using it. The functions cannot be stored directly in the actuator because of generic type erasure and generic invariance. This approach is cleaner
            private final HashMap<String, List<ControlFunction<T>>> controlFuncsMap = new HashMap<>(); //Map with lists of control functions paired with names.
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
                if (controlFuncKeys.length>0){
                    defaultControlKey=controlFuncKeys[0];
                }
                else{
                    defaultControlKey="controlOff";
                }
            }
        }
        public Actuator(String actuatorName, Class<E> type,
                        String[] partNames, Function<E, Double> getCurrentPosition, int currentPosPollingInterval,
                        ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc,
                        double errorTol, double defaultMovementTimeout){
            currentPosPollingInterval=Math.max(0,currentPosPollingInterval);
            this.name=actuatorName;
            this.partNames=partNames;
            this.maxTargetFunc = ()->(maxTargetFunc.call()+offset);
            this.minTargetFunc = ()->(minTargetFunc.call()+offset);
            this.defaultMovementTimeout = defaultMovementTimeout;
            this.errorTol=errorTol;
            ArrayList<CachedReader<Double>> readers = new ArrayList<>();
            for (String name:partNames){
                this.parts.put(name,Components.hardwareMap.get(type,name)); //Can't do E.class instead of using the parameter 'type' because of generic type erasure
                CachedReader<Double> reader = new CachedReader<>(()->(getCurrentPosition.apply(parts.get(name))),currentPosPollingInterval);
                readers.add(reader);
                this.getCurrentPositions.put(name,reader::cachedRead); //The getCurrentPosition function is copied, one for each of the actuator's parts. Position conversion is also applied
            }
            resetCurrentPositionCaches = ()->{
                for (CachedReader<Double> reader:readers){
                    reader.resetCache();
                }
            };
            actuators.put(name,this);
        }
        public String getName(){
            return name;
        }
        public void setOffsetBoundFuncs(ReturningFunc<Double> maxFunc,ReturningFunc<Double> minFunc){
            this.maxOffsetFunc=maxFunc;
            this.minOffsetFunc=minFunc;
        }
        public double getErrorTol(){
            return errorTol;
        }
        public String getCurrControlFuncKey(){
            return currControlFuncKey;
        }
        public String getDefaultControlKey(){
            return defaultControlKey;
        }
        public void setTimeBasedLocalization(boolean timeBasedLocalization){
            this.timeBasedLocalization=timeBasedLocalization;
        }
        public boolean getTimeBasedLocalization(){
            return this.timeBasedLocalization;
        }
        public boolean isNewTarget(){
            return newTarget;
        }
        public void resetNewTarget(){
            newTarget=false;
        }
        public boolean isNewActuation(){
            return newActuation;
        }
        public void resetNewActuation(){
            newActuation=false;
        }
        public void setNewActuation(){
            newActuation=true;
        }
        public void setTarget(double target){
            if (targetStateUnlocked){
                target=target+offset;
                target=Math.max(minTargetFunc.call(),Math.min(target, maxTargetFunc.call()));
                if (target!=this.target) {
                    this.target = target;
                    this.instantTarget = target;
                    newTarget = true;
                }
            }
        }
        public double getTargetMinusOffset(){
            return target-offset;
        }
        public double getTarget(){
            return target;
        }
        public void setInstantTarget(double instantTarget){
            this.instantTarget=Math.max(minTargetFunc.call(),Math.min(instantTarget, maxTargetFunc.call()));
        }
        public double getInstantTarget(){
            return instantTarget;
        }
        public double getCurrentPosition(String name){ //Gets the position of a specific part
            return Objects.requireNonNull(getCurrentPositions.get(name)).call();
        }
        public double getCurrentPosition(){ //Gets the avg position of all synchronized parts
            double avg = 0;
            for (String name : partNames){
                avg+=getCurrentPosition(name);
            }
            return avg/getCurrentPositions.values().size();
        }
        public void setOffset(double offset){
            offset=Math.max(minOffsetFunc.call(),Math.min(maxOffsetFunc.call(),offset));
            if (offset!=this.offset){
                double oldOffset=this.offset;
                this.offset=offset;
                setTarget(target-oldOffset);
            }
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
        public void lockActuationState(){
            actuationStateUnlocked=false;
        }
        public void unlockActuationState(){
            actuationStateUnlocked=true;
        }
        public void lockTargetState(){
            targetStateUnlocked=false;
        }
        public void unlockTargetState(){
            targetStateUnlocked=true;
        }
        protected void resetCurrentPositions(){
            resetCurrentPositionCaches.call();
        }
        public void setKeyPositions(String[] keyPositionKeys, double[] keyPositionValues){
            for (int i=0; i<keyPositionKeys.length; i++){
                keyPositions.put(keyPositionKeys[i],keyPositionValues[i]);
            }
        }
        public E getPart(String partName){
            return Objects.requireNonNull(this.parts.get(partName));
        }
        public Collection<E> getParts(){
            return this.parts.values();
        }
        public String[] getPartNames(){
            return this.partNames;
        }
        public void setAsBroken(){
            this.isBroken=true;
        }
        public void setAsUnbroken(){
            this.isBroken=false;
        }
        public boolean isBroken(){
            return this.isBroken;
        }
        public class MoveToTargetAction extends CompoundAction { //Action to set the target, then wait until the position of the actuator is a certain distance from the target, or until a set timeout
            public MoveToTargetAction(ReturningFunc<Double> targetFunc, double timeout){
                group = new NonLinearSequentialAction(
                        new InstantAction(()-> setTarget(targetFunc.call())),
                        new SleepUntilTrue(
                                ()->(isBroken||Math.abs(getCurrentPosition()-target)<errorTol),
                                timeout
                        )
                );
            }
            public MoveToTargetAction(double target, double timeout){
                this(()->(target), timeout);
            }
            public MoveToTargetAction(ReturningFunc<Double> targetFunc){
                this(targetFunc, defaultMovementTimeout);
            }
            public MoveToTargetAction(double target){
                this(()->(target), defaultMovementTimeout);
            }
        }
        public class SetOffsetAction extends CompoundAction { //Action to set the offset
            public SetOffsetAction(ReturningFunc<Double> offsetFunc, double timeout){
                group = new NonLinearSequentialAction(
                        new InstantAction(()-> setOffset(offsetFunc.call())),
                        new SleepUntilTrue(
                                ()->(isBroken||Math.abs(getCurrentPosition()-target)<errorTol),
                                timeout
                        )
                );
            }
            public SetOffsetAction(double offset, double timeout){
                this(()->(offset), timeout);
            }
            public SetOffsetAction(ReturningFunc<Double> offsetFunc){
                this(offsetFunc, defaultMovementTimeout);
            }
            public SetOffsetAction(double offset){
                this(()->(offset), defaultMovementTimeout);
            }
        }
        public InstantAction instantSetTargetAction(double target){
            return new InstantAction(()->setTarget(target));
        }
        public InstantAction instantSetTargetAction(ReturningFunc<Double> targetFunc){
            return new InstantAction(()->setTarget(targetFunc.call()));
        }
        public InstantAction instantSetTargetAction(String position){
            return new InstantAction(()->setTarget(getPos(position)));
        }
        public MoveToTargetAction moveToTargetAction(double target){
            return new MoveToTargetAction(target);
        }
        public MoveToTargetAction moveToTargetAction(ReturningFunc<Double> targetFunc){
            return new MoveToTargetAction(targetFunc);
        }
        public MoveToTargetAction moveToTargetAction(double target, double timeout){
            return new MoveToTargetAction(target,timeout);
        }
        public MoveToTargetAction moveToTargetAction(ReturningFunc<Double> targetFunc, double timeout){
            return new MoveToTargetAction(targetFunc);
        }
        public MoveToTargetAction moveToTargetAction(String position){
            return new MoveToTargetAction(getPos(position));
        }
        public MoveToTargetAction moveToTargetAction(String position,double timeout){
            return new MoveToTargetAction(getPos(position),timeout);
        }
        public MoveToTargetAction toggleTargetAction(double target1, double target2){
            return moveToTargetAction(()->{
                if (getTargetMinusOffset()==target1) return target2; else if (getTargetMinusOffset()==target2) return target1; else return getTargetMinusOffset();
            });
        }
        public MoveToTargetAction upwardFSMTargetAction(double...targets){
            Arrays.sort(targets);
            return moveToTargetAction(()->{
                for (double target: targets){
                    if (getTargetMinusOffset()<target){
                        return target;
                    }
                }
                return getTargetMinusOffset();
            });
        }
        public MoveToTargetAction downwardFSMTargetAction(double...targets){
            Arrays.sort(targets);
            return moveToTargetAction(()->{
                for (int i = targets.length-1; i>=0; i--){
                    if (getTargetMinusOffset()>targets[i]){
                        return targets[i];
                    }
                }
                return getTargetMinusOffset();
            });
        }
        public SetOffsetAction setOffsetAction(double offset){
            return new SetOffsetAction(offset);
        }
        public SetOffsetAction setOffsetAction(ReturningFunc<Double> offsetFunc){
            return new SetOffsetAction(offsetFunc);
        }
        public RunResettingLoop triggeredMoveToTargetAction(Condition condition, double target){
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, new MoveToTargetAction(target))));
        }
        public RunResettingLoop triggeredMoveToTargetAction(Condition condition, ReturningFunc<Double> targetFunc) {
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, moveToTargetAction(targetFunc))));
        }
        public RunResettingLoop triggeredMoveToTargetAction(Condition condition, double target, double timeout){
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, moveToTargetAction(target,timeout))));
        }
        public RunResettingLoop triggeredMoveToTargetAction(Condition condition, ReturningFunc<Double> targetFunc, double timeout){
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, moveToTargetAction(targetFunc,timeout))));
        }
        public RunResettingLoop triggeredMoveToTargetAction(Condition condition, String position){
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, moveToTargetAction(position))));
        }
        public RunResettingLoop triggeredMoveToTargetAction(Condition condition, String position,double timeout){
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, moveToTargetAction(position,timeout))));
        }
        public RunResettingLoop triggeredToggleTargetAction(Condition condition, double target1, double target2){
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, toggleTargetAction(target1,target2))));
        }
        public RunResettingLoop triggeredDynamicTargetAction(Condition upCondition, Condition downCondition, double change){
            return new RunResettingLoop(new ConditionalAction(new IfThen(upCondition, moveToTargetAction(()->(getTargetMinusOffset()+change))),new IfThen(downCondition, moveToTargetAction(()->(getTargetMinusOffset()-change)))));
        }
        public RunResettingLoop triggeredFSMTargetAction(Condition upCondition, Condition downCondition, double...targets){
            return new RunResettingLoop(new PressTrigger(new IfThen(upCondition, upwardFSMTargetAction(targets)),new IfThen(downCondition, downwardFSMTargetAction(targets))));
        }
        public RunResettingLoop triggeredSetOffsetAction(Condition condition, double offset){
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, new SetOffsetAction(offset))));
        }
        public RunResettingLoop triggeredDynamicOffsetAction(Condition upCondition, Condition downCondition, double offsetChange){
            return new RunResettingLoop(new ConditionalAction(new IfThen(upCondition, setOffsetAction(()->(offset+offsetChange))),new IfThen(downCondition, setOffsetAction(()->(offset-offsetChange)))));
        }
        public InstantAction switchControlAction(String controlKey){
            return new InstantAction(()->this.switchControl(controlKey));
        }
        public InstantAction lockActuationAction(){
            return new InstantAction(this::lockActuationState);
        }
        public InstantAction unlockActuationAction(){
            return new InstantAction(this::unlockActuationState);
        }
        public InstantAction lockTargetAction(){
            return new InstantAction(this::lockTargetState);
        }
        public InstantAction unlockTargetAction(){
            return new InstantAction(this::unlockTargetState);
        }
    }
    //Each of the subclasses of Actuator will have some generic constructors and some constructors where information is preset.
    public abstract static class CRActuator<E extends DcMotorSimple> extends Actuator<E>{ //Type of Actuator that works for continuous rotation parts, like DcMotorEx and CRServo
        private final HashMap<String,Double> powers = new HashMap<>(); //Stores the powers each of the parts are set to. Synchronized parts can have different powers because the load on one may be larger than on the other
        private final ReturningFunc<Double> maxPowerFunc;
        private final ReturningFunc<Double> minPowerFunc;
        //Max and min power boundaries
        private final HashMap<String,Double> keyPowers = new HashMap<>(); //Stores key powers, like 'intakePower,' etc.
        public CRActuator(String name, Class<E> type, String[] names, Function<E, Double> getCurrentPosition, int pollingRate, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, double errorTol, double defaultTimeout,
                           DcMotorSimple.Direction[] directions) {
            super(name, type, names, getCurrentPosition, pollingRate, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout);
            this.maxPowerFunc=maxPowerFunc;
            this.minPowerFunc=minPowerFunc;
            for (int i=0;i<names.length;i++){
                Objects.requireNonNull(parts.get(names[i])).setDirection(directions[i]);
                powers.put(names[i],0.0);
            }
            this.setTarget(0);
        }
        public CRActuator(String name, Class<E> type, String[] names, Function<E, Double> getCurrentPosition, int pollingRate, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout,
                          DcMotorSimple.Direction[] directions) {
            this(name,type,names,getCurrentPosition,pollingRate,maxTargetFunc,minTargetFunc,()->(1.0),()->(-1.0),errorTol,defaultTimeout,directions);
        }
        public CRActuator(String name, Class<E> type, String[] names, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, DcMotorSimple.Direction[] directions) { //For CRActuators that don't set targets and only use setPower, like drivetrain motors.
            this(name,type,names,(E e)->(0.0), 1,()->(Double.POSITIVE_INFINITY),()->(Double.NEGATIVE_INFINITY),()->(1.0),()->(-1.0),0,0,directions);
        }
        public double getKeyPower(String key){
            return Objects.requireNonNull(keyPowers.get(key));
        }
        public void setKeyPowers(String[] keyPowerKeys, double[] keyPowerValues){
            for (int i=0; i<keyPowerKeys.length; i++){
                keyPowers.put(keyPowerKeys[i],keyPowerValues[i]);
            }
        }
        @Actuate
        public void setPower(double power, String name){ //Sets power to a specific part
            if (actuationStateUnlocked){
                power=Math.max(Math.min(power, maxPowerFunc.call()), minPowerFunc.call());
                E part = parts.get(name);
                assert part != null;
                if (Math.abs(power-part.getPower())>0.03) {
                    part.setPower(power);
                    powers.put(name,power);
                    if (getTimeBasedLocalization()){ //If current position is calculated by time, it needs to be updated everytime the actuator moves
                        resetCurrentPositions();
                        getCurrentPosition(name);
                    }
                    setNewActuation();
                }
            }
        }
        @Actuate
        public void setPower(double power){ //Sets power to all synchronized parts at once
            if (actuationStateUnlocked) {
                for (String name:partNames) {
                    setPower(power,name);
                }
            }
        }
        public double getPower(String name){
            return Objects.requireNonNull(powers.get(name));
        }
        public double getPower(){
            double avg=0;
            for (String name:partNames){
                avg+=getPower(name);
            }
            return avg/partNames.length;
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
        public SetPowerAction setPowerAction(String key){
            return new SetPowerAction(getKeyPower(key));
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
        public RunResettingLoop triggeredSetPowerAction(Condition condition, ReturningFunc<Double> powerFunc){
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, new SetPowerAction(powerFunc))));
        }
        public RunResettingLoop triggeredSetPowerAction(Condition condition, String key){
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, new SetPowerAction(getKeyPower(key)))));
        }
        public RunResettingLoop triggeredSetPowerAction(Condition condition, double power){
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, new SetPowerAction(power))));
        }
        public RunResettingLoop triggeredDynamicPowerAction(Condition upCondition, Condition downCondition, double change){
            return new RunResettingLoop(new ConditionalAction(new IfThen(upCondition, setPowerAction(()->(Objects.requireNonNull(powers.get(partNames[0]))+change))),new IfThen(downCondition, setPowerAction(()->(Objects.requireNonNull(powers.get(partNames[0]))-change)))));
        }
        public RunResettingLoop triggeredTogglePowerAction(Condition condition, double power1, double power2){
            return new RunResettingLoop(new PressTrigger(new IfThen(condition, togglePowerAction(power1,power2))));
        }
        public RunResettingLoop triggeredFSMPowerAction(Condition upCondition, Condition downCondition, double...powers){
            return new RunResettingLoop(new PressTrigger(
                    new IfThen(upCondition, upwardFSMPowerAction(powers)),
                    new IfThen(downCondition, downwardFSMPowerAction(powers))
            ));
        }
    }
    //Each of the bottom-level subclass constructors will accept getCurrentPosition functions and control functions, since those cater to a specific subclass.
    public static class BotMotor extends CRActuator<DcMotorEx> {
        private boolean isStallResetting;
        private final HashMap<String, ReturningFunc<Double>> velocityReaders = new HashMap<>();
        private final HashMap<String, ReturningFunc<Double>> currentReaders = new HashMap<>();

        @SafeVarargs
        public BotMotor(String name, String[] names, Function<DcMotorEx, Double> getCurrentPosition, int currentPosPollingInterval, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, double errorTol, double defaultTimeout, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<BotMotor>>... controlFuncs) {
            super(name, DcMotorEx.class, names, getCurrentPosition, currentPosPollingInterval, maxTargetFunc, minTargetFunc, maxPowerFunc, minPowerFunc, errorTol, defaultTimeout, directions);
            for (String partName : getPartNames()) {
                velocityReaders.put(partName, new CachedReader<>(Objects.requireNonNull(parts.get(name))::getVelocity, 1)::cachedRead);
                currentReaders.put(partName, new CachedReader<>(() -> Objects.requireNonNull(parts.get(name)).getCurrent(CurrentUnit.AMPS), 3)::cachedRead);
            }
            for (DcMotorEx part : parts.values()) {
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            this.funcRegister = new ControlFuncRegister<BotMotor>(this, controlFuncKeys, controlFuncs);
        }

        @SafeVarargs
        public BotMotor(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, double errorTol, double defaultTimeout, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<BotMotor>>... controlFuncs) {
            super(name, DcMotorEx.class, names, (DcMotorEx motor) -> ((double) motor.getCurrentPosition()), 1, maxTargetFunc, minTargetFunc, maxPowerFunc, minPowerFunc, errorTol, defaultTimeout, directions);
            for (String partName : getPartNames()) {
                velocityReaders.put(partName, new CachedReader<>(Objects.requireNonNull(parts.get(name))::getVelocity, 1)::cachedRead);
                currentReaders.put(partName, new CachedReader<>(() -> Objects.requireNonNull(parts.get(name)).getCurrent(CurrentUnit.AMPS), 3)::cachedRead);
            }
            for (DcMotorEx part : parts.values()) {
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            this.funcRegister = new ControlFuncRegister<BotMotor>(this, controlFuncKeys, controlFuncs);
        }

        @SafeVarargs
        public BotMotor(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<BotMotor>>... controlFuncs) {
            super(name, DcMotorEx.class, names, (DcMotorEx motor) -> ((double) motor.getCurrentPosition()), 1, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, directions);
            for (String partName : getPartNames()) {
                velocityReaders.put(partName, new CachedReader<>(Objects.requireNonNull(parts.get(name))::getVelocity, 1)::cachedRead);
                currentReaders.put(partName, new CachedReader<>(() -> Objects.requireNonNull(parts.get(name)).getCurrent(CurrentUnit.AMPS), 3)::cachedRead);
            }
            for (DcMotorEx part : parts.values()) {
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            this.funcRegister = new ControlFuncRegister<BotMotor>(this, controlFuncKeys, controlFuncs);
        }

        public BotMotor(String name, String[] names, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, DcMotorSimple.Direction[] directions) {
            super(name, DcMotorEx.class, names, maxPowerFunc, minPowerFunc, directions);
            for (String partName : getPartNames()) {
                velocityReaders.put(partName, new CachedReader<>(Objects.requireNonNull(parts.get(name))::getVelocity, 1)::cachedRead);
                currentReaders.put(partName, new CachedReader<>(() -> Objects.requireNonNull(parts.get(name)).getCurrent(CurrentUnit.AMPS), 3)::cachedRead);
            }
            for (DcMotorEx part : parts.values()) {
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            this.funcRegister = new ControlFuncRegister<BotMotor>(this, new String[]{}, new ArrayList<>());
        }

        public double getVelocity(String name) {
            return Objects.requireNonNull(velocityReaders.get(name)).call();
        }

        public double getVelocity() { //Returns avg velocity of all parts
            double avg = 0;
            for (String name : partNames) {
                avg += getVelocity(name);
            }
            return avg / parts.size();
        }

        public void resetEncoders() {
            for (DcMotorEx part : parts.values()) {
                part.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        public void setZeroPowerFloat() {
            for (DcMotorEx part : parts.values()) {
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        public void setZeroPowerBrake() {
            for (DcMotorEx part : parts.values()) {
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        public double getCurrentAmps(String partName) {
            return Objects.requireNonNull(currentReaders.get(partName)).call();
        }

        public double getCurrentAmps() {
            double maxCurrent = 0;
            for (String partName : getPartNames()) {
                double current = getCurrentAmps(partName);
                if (current > maxCurrent) {
                    maxCurrent = current;
                }
            }
            return maxCurrent;
        }

        public boolean isStallResetting() {
            return isStallResetting;
        }

        public void setMode(DcMotorEx.RunMode mode) {
            for (DcMotorEx part : parts.values()) {
                part.setMode(mode);
            }
        }

        public class StallResetAction extends NonLinearAction { //Stall resets encoders, and offsets the position if you want to reset at a non-zero position.
            double resetPosition;
            double stallVolts;

            public StallResetAction(double resetPosition, double stallVolts) {
                this.resetPosition = resetPosition;
                this.stallVolts = stallVolts;
            }

            @Override
            protected boolean runProcedure() {
                if (isStart()) {
                    isStallResetting = true;
                    setPower(-0.2);
                }
                if (getCurrentAmps() > stallVolts) {
                    setPower(0);
                    setOffset(getCurrentPosition() - resetPosition);
                    setTarget(resetPosition);
                    isStallResetting = false;
                }
                return isStallResetting;
            }
        }

        public StallResetAction stallResetAction(double resetPosition, double stallVolts) {
            return new StallResetAction(resetPosition, stallVolts);
        }

        public PressTrigger triggeredStallResetAction(Condition condition, double resetPosition, double stallVolts) {
            return new PressTrigger(new IfThen(condition, stallResetAction(resetPosition, stallVolts)));
        }
        public class SetPowerForDistance extends CompoundAction{ //Makes the motor set a power until it travels a certain distance.
            private double startPosition;
            public SetPowerForDistance(double power, double distance){
                setGroup(new NonLinearSequentialAction(
                        new InstantAction(()->startPosition=getCurrentPosition()),
                        new NonLinearActions.NonLinearParallelAction(
                                setPowerAction(power)
                        ),
                        new SleepUntilTrue(()->(getCurrentPosition()-startPosition)>50),
                        new NonLinearActions.NonLinearParallelAction(
                                setPowerAction(0)
                        )
                ));
            }
        }
        public SetPowerForDistance setPowerForDistance(double power, double distance){
            return new SetPowerForDistance(power,distance);
        }
    }
    public static class BotServo extends Actuator<Servo>{
        private double currCommandedPos;
        private boolean ignoreSetPosCaching = false;
        private final double range;
        private Function<Double, Double> setPositionConversion;
        @SafeVarargs
        public BotServo(String name, String[] names, Function<Servo, Double> getCurrentPosition, int currentPosPollingInterval, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, Servo.Direction[] directions, double range, //Degree range that servo is programmed to
                        double initialTarget, String[] controlFuncKeys, List<ControlFunction<BotServo>>... controlFuncs) {
            super(name, Servo.class, names, getCurrentPosition, currentPosPollingInterval, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout);
            setTarget(initialTarget);
            for (int i=0;i<directions.length;i++){
                Objects.requireNonNull(parts.get(names[i])).setDirection(directions[i]);
            }
            this.range=range;
            this.setPositionConversion=(Double pos)->pos/range;
            this.funcRegister=new ControlFuncRegister<BotServo>(this,controlFuncKeys, controlFuncs);
        }
        public BotServo(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double servoSpeedDPS, double defaultTimeout, Servo.Direction[] directions, double range, double initialTarget) {
            this(name,names,new TimeBasedLocalizers.ServoTimeBasedLocalizer(servoSpeedDPS/range,initialTarget,range)::getCurrentPosition,1,maxTargetFunc,minTargetFunc,1.5,defaultTimeout,directions,range, initialTarget, new String[]{"setPos"}, new ArrayList<>(Collections.singleton(new ServoControl())));
            setTimeBasedLocalization(true);
        }
        public void setPositionConversion(Function<Double,Double> setPositionConversion){
            this.setPositionConversion=setPositionConversion;
        }
        @Actuate
        public void setPosition(double position){ //Accepts position in degrees
            position=Math.max(minTargetFunc.call(),Math.min(position, maxTargetFunc.call()));
            if (actuationStateUnlocked && (Math.abs(currCommandedPos-position)>0.07||ignoreSetPosCaching)){
                currCommandedPos=position;
                for (Servo part:parts.values()){part.setPosition(setPositionConversion.apply(position));}
                if (getTimeBasedLocalization()){
                    resetCurrentPositions();
                    getCurrentPosition();
                }
                setNewActuation();
            }
        }
        public double getPosition(){
            return currCommandedPos;
        }
        public double getRange(){return range;}
        public boolean isIgnoreSetPosCaching(){
            return ignoreSetPosCaching;
        }
        public void setIgnoreSetPosCaching(boolean bool){
            ignoreSetPosCaching=bool;
        }
        public InstantAction toggleIgnoreSetPosCaching(){
            return new InstantAction(()->setIgnoreSetPosCaching(!isIgnoreSetPosCaching()));
        }
    }
    public static class CRBotServo extends CRActuator<CRServo>{
        @SafeVarargs
        public CRBotServo(String name, String[] names, Function<CRServo, Double> getCurrentPosition, int pollingRate,ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, double errorTol, double defaultTimeout, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            super(name, CRServo.class, names, getCurrentPosition, pollingRate, maxTargetFunc, minTargetFunc, maxPowerFunc, minPowerFunc, errorTol, defaultTimeout, directions);
            this.funcRegister=new ControlFuncRegister<CRBotServo>(this, controlFuncKeys, controlFuncs);
        }
        @SafeVarargs
        public CRBotServo(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, ReturningFunc<Double> maxPowerFunc, ReturningFunc<Double> minPowerFunc, double servoSpeed, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            super(name, CRServo.class, names, new TimeBasedLocalizers.CRTimeBasedLocalizer<CRServo>(servoSpeed)::getCurrentPosition, 1,maxTargetFunc, minTargetFunc, maxPowerFunc,minPowerFunc,0, Double.POSITIVE_INFINITY, directions);
            this.funcRegister=new ControlFuncRegister<CRBotServo>(this, controlFuncKeys, controlFuncs);
        }
        @SafeVarargs
        public CRBotServo(String name, String[] names, Function<CRServo, Double> getCurrentPosition, int pollingRate, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            super(name, CRServo.class, names, getCurrentPosition, pollingRate, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, directions);
            this.funcRegister=new ControlFuncRegister<CRBotServo>(this, controlFuncKeys, controlFuncs);
        }
        @SafeVarargs
        public CRBotServo(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double servoSpeed, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, List<ControlFunction<CRBotServo>>... controlFuncs) {
            super(name, CRServo.class, names, new TimeBasedLocalizers.CRTimeBasedLocalizer<CRServo>(servoSpeed)::getCurrentPosition, 1,maxTargetFunc, minTargetFunc,0.05, Double.POSITIVE_INFINITY, directions);
            this.funcRegister=new ControlFuncRegister<CRBotServo>(this, controlFuncKeys, controlFuncs);
        }
    }
}
