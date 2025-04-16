package org.firstinspires.ftc.teamcode.base;

import static org.firstinspires.ftc.teamcode.base.Components.actuators;
import static org.firstinspires.ftc.teamcode.base.Components.telemetry;
import static org.firstinspires.ftc.teamcode.base.Components.timer;

import static org.firstinspires.ftc.teamcode.base.Components.BotMotor;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.Procedure;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.ReturningFunc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Objects;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.Condition;
import java.util.stream.Collectors;
public abstract class NonLinearActions { //Command-based (or action-based) system
    public abstract static class NonLinearAction { //Base class for any action
        public boolean isBusy = false; //Indicates whether the action is active or not
        public boolean isStart = true; //Actions know if they've just started running or not
        public Procedure removeFromGroup; //If the action is part of an action group (everything but the scheduler,) this allows it to remove itself from the group.

        public void reset() {
            isStart = true;
        }

        final public boolean run() { //Actual method called to run the action
            isBusy = runProcedure();
            isStart = false;
            if (!isBusy) {
                reset(); //Auto-reset after action is completed or stopped.
            }
            return isBusy;
        }

        abstract boolean runProcedure(); //This is where one codes what the action does

        public void stopProcedure() {
        } //This is where code is made for if the action is interrupted

        final public void stop() { //Actual method called to stop the action
            if (isBusy) {
                stopProcedure();
                isBusy = false;
                reset();
            }
        }

        public void registerRemoveFromGroup(Procedure removeFromGroup) {
            this.removeFromGroup = removeFromGroup;
        }

        public void removeFromGroup() { //Allows the action to remove itself from an action group
            if (Objects.nonNull(removeFromGroup)) {
                removeFromGroup.call();
            }
            removeFromGroup = null;
        }
    }

    public interface MappedActionGroup<K> { //NonLinearActions that run a group of other actions, and stores them with indexes or keys, should implement this. (SequentialActions, ConditionalActions)
        default void addAction(K key, NonLinearAction action) { //Actual method called to add actions
            action.registerRemoveFromGroup(() -> this.removeAction(action));
            addActionProcedure(key, action);
        }

        void addActionProcedure(K key, NonLinearAction action); //Allows action groups to add actions to the group

        default void removeAction(NonLinearAction action){
            action.stop();
            removeActionProcedure(action);
        }
        void removeActionProcedure(NonLinearAction action); //Allows action groups to remove actions from the group
        default void registerActions(NonLinearAction...actions){
            for (NonLinearAction action : actions) {
                action.registerRemoveFromGroup(() -> this.removeAction(action));
            }
        }
    }

    public interface UnmappedActionGroup { //NonLinearActions that run an unmapped set of other actions should implement this. (ParallelActions)
        default void addAction(NonLinearAction action) {
            action.registerRemoveFromGroup(() -> this.removeAction(action));
            addActionProcedure(action);
        }
        void addActionProcedure(NonLinearAction action);

        default void removeAction(NonLinearAction action){
            action.stop();
            removeActionProcedure(action);
        }
        void removeActionProcedure(NonLinearAction action); //Allows action groups to remove actions from the group
        default void registerActions(NonLinearAction...actions){
            for (NonLinearAction action : actions) {
                action.registerRemoveFromGroup(() -> this.removeAction(action));
            }
        }
    }
    public enum ScheduleType{
        ADD,
        REMOVE
    }

    //TEMPLATE ACTIONS

    public abstract static class PersistentNonLinearAction extends NonLinearAction { //NonLinearAction but it can't be reset if not completed
        @Override
        public void reset() {
            if (!isBusy) {
                isStart = true;
            }
        }
    }

    public static class InstantAction extends NonLinearAction { //Action that completes in one loop iteration
        public Procedure procedure;

        public InstantAction(Procedure procedure) {
            this.procedure = procedure;
        }

        @Override
        public boolean runProcedure() {
            procedure.call();
            return false;
        }
    }

    public static class ContinuousAction extends NonLinearAction { //Action that constantly runs the same way each loop iteration
        public Procedure procedure;

        public ContinuousAction(Procedure procedure) {
            this.procedure = procedure;
        }

        @Override
        public boolean runProcedure() {
            procedure.call();
            return true;
        }
    }

    public static class LambdaAction extends NonLinearAction { //This allows one to create a NonLinearAction without making it its own class
        public ReturningFunc<Boolean> action;

        public LambdaAction(ReturningFunc<Boolean> action) {
            this.action = action;
        }

        @Override
        public boolean runProcedure() {
            return action.call();
        }
    }

    public abstract static class CompoundAction extends NonLinearAction { //Allows one to represent a sequence of actions as one atomic action. The main difference between this and a sequential action is that you can code custom stop functionality.
        public NonLinearAction sequence;

        @Override
        boolean runProcedure() {
            if (isStart) {
                sequence.reset();
            }
            return sequence.run();
        }

        @Override
        public void stopProcedure() {
            sequence.stop();
        }
    }

    //PRELOADED ACTIONS
    public static class UnmappedScheduleAction extends CompoundAction{ //Allows one to schedule the adding or removing of an action from an unmapped action group when a condition is met, or if a timeout is reached
        public UnmappedScheduleAction(UnmappedActionGroup group, Condition condition, NonLinearAction action, ScheduleType type, double timeout){
            Procedure schedule;
            if (type==ScheduleType.ADD) schedule = ()->group.addAction(action); else schedule=()->group.removeAction(action);
            sequence = new NonLinearSequentialAction(
                    new SleepUntilTrue(condition,timeout),
                    new InstantAction(schedule),
                    new InstantAction(this::removeFromGroup)
            );
        }
        public UnmappedScheduleAction(UnmappedActionGroup group, Condition condition, NonLinearAction action, ScheduleType type){
            Procedure schedule;
            if (type==ScheduleType.ADD) schedule = ()->group.addAction(action); else schedule=()->group.removeAction(action);
            sequence = new NonLinearSequentialAction(
                    new SleepUntilTrue(condition),
                    new InstantAction(schedule),
                    new InstantAction(this::removeFromGroup)
            );
        }
    }
    public static class MappedScheduleAction<E> extends CompoundAction{ //Allows one to schedule the adding or removing of an action from a mapped action group when a condition is met, or if a timeout is reached
        public MappedScheduleAction(MappedActionGroup<E> group, Condition condition, E key, NonLinearAction action, ScheduleType type, double timeout){
            Procedure schedule;
            if (type==ScheduleType.ADD) schedule = ()->group.addAction(key,action); else schedule=()->group.removeAction(action);
            sequence = new NonLinearSequentialAction(
                    new SleepUntilTrue(condition,timeout),
                    new InstantAction(schedule),
                    new InstantAction(this::removeFromGroup)
            );
        }
        public MappedScheduleAction(MappedActionGroup<E> group, Condition condition, E key, NonLinearAction action, ScheduleType type){
            Procedure schedule;
            if (type==ScheduleType.ADD) schedule = ()->group.addAction(key,action); else schedule=()->group.removeAction(action);
            sequence = new NonLinearSequentialAction(
                    new SleepUntilTrue(condition),
                    new InstantAction(schedule),
                    new InstantAction(this::removeFromGroup)
            );
        }
    }
    public static class RunLoopRoutine extends ContinuousAction { //This action runs each actuator's control functions and updates the telemetry using the updateTelemetry function it is provided
        public RunLoopRoutine(Procedure updateTelemetry) {
            super(() -> {
                for (Components.Actuator<?> actuator : actuators.values()) {
                    if (actuator.dynamicTargetBoundaries) { //If the actuator's target boundaries can change, this will ensure that the actuator's target never falls outside of the boundaries
                        actuator.setTarget(actuator.target);
                    }
                    if (actuator instanceof Components.CRActuator && ((Components.CRActuator<?>) actuator).dynamicPowerBoundaries) { //If the CRActuator's power boundaries can change, this will ensure that the CRActuator's power never falls outside of the boundaries
                        Components.CRActuator<?> castedActuator = ((Components.CRActuator<?>) actuator);
                        castedActuator.setPower(Objects.requireNonNull(castedActuator.powers.get(castedActuator.partNames[0])));
                    }
                    actuator.runControl();
                    actuator.newTarget = false;
                }
                updateTelemetry.call();
            });
        }

        public RunLoopRoutine() {
            super(() -> {
                for (Components.Actuator<?> actuator : actuators.values()) {
                    if (actuator.dynamicTargetBoundaries) {
                        actuator.setTarget(actuator.target);
                    }
                    if (actuator instanceof Components.CRActuator && ((Components.CRActuator<?>) actuator).dynamicPowerBoundaries) {
                        Components.CRActuator<?> castedActuator = ((Components.CRActuator<?>) actuator);
                        castedActuator.setPower(Objects.requireNonNull(castedActuator.powers.get(castedActuator.partNames[0])));
                    }
                    actuator.runControl();
                    actuator.newTarget = false;
                }
                for (Components.Actuator<?> actuator : actuators.values()) {
                    telemetry.addData(actuator.name + " target", actuator.target);
                    telemetry.addData(actuator.name + " instant target", actuator.instantTarget);
                    telemetry.addData(actuator.name + " current position", actuator.getCurrentPosition());
                    telemetry.addData("", "");
                }
                telemetry.update();
            });
        }
    }

    public static class PowerOnCommand extends NonLinearAction { //This action automatically activates each actuator's default control functions when they are first commanded
        public HashMap<String, Boolean> actuatorsCommanded = new HashMap<>();

        @Override
        boolean runProcedure() {
            if (actuatorsCommanded.containsValue(false))
                for (String key : actuators.keySet()) {
                    if (Objects.requireNonNull(actuators.get(key)).newTarget && Boolean.FALSE.equals(actuatorsCommanded.get(key))) {
                        Objects.requireNonNull(actuators.get(key)).switchControl(Objects.requireNonNull(actuators.get(key)).defaultControlKey);
                        actuatorsCommanded.put(key, true);
                    }
                }
            else{
                removeFromGroup();
            }
            return true;
        }
    }

    public static class SleepUntilTrue extends NonLinearAction { //Sleeps until a condition is met or until an optional timeout time is reached
        public ReturningFunc<Boolean> condition;
        public double timeout;
        public double startTime;

        public SleepUntilTrue(ReturningFunc<Boolean> condition, double timeout) {
            this.condition = condition;
            this.timeout = timeout;
        }

        public SleepUntilTrue(ReturningFunc<Boolean> condition) {
            this.condition = condition;
            this.timeout = Double.POSITIVE_INFINITY;
        }

        @Override
        boolean runProcedure() {
            if (isStart && timeout != Double.POSITIVE_INFINITY) {
                startTime = timer.time();
            }
            return !condition.call() && (timer.time() - startTime) < timeout;
        }
    }

    public static class NonLinearSleepAction extends NonLinearAction { //Sleeps for a set time
        double time;
        double startTime;

        public NonLinearSleepAction(double time) {
            this.time = time;
        }

        @Override
        boolean runProcedure() {
            if (isStart) {
                startTime = timer.time();
            }
            return (timer.time() - startTime) < time;
        }
    }
    public static class InterruptOnTimeout extends NonLinearAction{ //Action to run the action passed to it, but interrupt it after it's been running for a given time
        NonLinearAction action;
        NonLinearSleepAction sleepAction;
        boolean actionEnded=false;
        public InterruptOnTimeout(double time, NonLinearAction action){
            this.action=action;
            this.sleepAction=new NonLinearSleepAction(time);
        }
        @Override
        boolean runProcedure() {
            if (isStart){
                sleepAction.reset();
                action.reset();
                actionEnded=false;
            }
            if (sleepAction.run()){
                if (!actionEnded && !action.run()){
                    actionEnded=true;
                }
            }
            else{
                actionEnded=true;
                action.stop();
            }
            return !actionEnded;
        }
        @Override
        public void stopProcedure(){
            action.stop();
        }
    }
    public static class InterruptOnCondition extends NonLinearAction{ //Action to run the action passed to it, but interrupt it after a given condition is met
        NonLinearAction action;
        SleepUntilTrue sleepAction;
        boolean actionEnded=false;
        public InterruptOnCondition(Condition condition, NonLinearAction action){
            this.action=action;
            this.sleepAction=new SleepUntilTrue(condition);
        }
        @Override
        boolean runProcedure() {
            if (isStart){
                sleepAction.reset();
                action.reset();
                actionEnded=false;
            }
            if (sleepAction.run()){
                if (!actionEnded && !action.run()){
                    actionEnded=true;
                }
            }
            else{
                actionEnded=true;
                action.stop();
            }
            return !actionEnded;
        }
        @Override
        public void stopProcedure(){
            action.stop();
        }
    }
    public static class NonLinearSequentialAction extends NonLinearAction implements MappedActionGroup<Integer> { //Runs actions sequentially
        public List<NonLinearAction> remainingActions;
        public List<NonLinearAction> actions;
        public ArrayList<Boolean> isStarts;

        public NonLinearSequentialAction(NonLinearAction... actions) {
            this.actions = Arrays.asList(actions);
            this.remainingActions = new ArrayList<>(this.actions);
            isStarts = new ArrayList<>(actions.length);
            for (NonLinearAction action : actions) {
                isStarts.add(true);
            }
            registerActions(actions);
        }

        @Override
        public boolean runProcedure() {
            if (isStart) {
                remainingActions = actions;
                for (int i = 0; i < actions.size(); i++) {
                    isStarts.set(i,true);
                }
            }
            if (isStarts.get(actions.size() - remainingActions.size())) {
                remainingActions.get(0).reset();
                isStarts.set(actions.size() - remainingActions.size(),false);
            }
            if (!remainingActions.get(0).run()) {
                remainingActions.remove(0);
            }
            return !remainingActions.isEmpty();
        }

        @Override
        public void stopProcedure() {
            remainingActions.get(0).stop();
        }

        @Override
        public void addActionProcedure(Integer key, NonLinearAction action) {
            actions.add(key,action);
            remainingActions.add(key,action);
            isStarts.add(key,true);
        }

        @Override
        public void removeActionProcedure(NonLinearAction action) {
            isStarts.remove(actions.indexOf(action));
            actions.remove(action);
            remainingActions.remove(action);
        }
    }

    public static class NonLinearParallelAction extends NonLinearAction implements UnmappedActionGroup { //Runs actions in parallel
        public List<NonLinearAction> remainingActions;
        public List<NonLinearAction> actions;

        public NonLinearParallelAction(NonLinearAction... actions) {
            this.actions = Arrays.asList(actions);
            this.remainingActions = new ArrayList<>(this.actions);
            registerActions(actions);
        }

        @Override
        public boolean runProcedure() {
            if (isStart) {
                remainingActions = actions;
                for (NonLinearAction action : remainingActions) {
                    action.reset();
                }
            }
            remainingActions = remainingActions.stream().filter(NonLinearAction::run).collect(Collectors.toList());
            return !remainingActions.isEmpty();
        }

        @Override
        public void stopProcedure() {
            for (NonLinearAction action : remainingActions) {
                action.stop();
            }
        }

        @Override
        public void addActionProcedure(NonLinearAction action) {
            actions.add(action);
            remainingActions.add(action);
        }

        @Override
        public void removeActionProcedure(NonLinearAction action) {
            actions.remove(action);
            remainingActions.remove(action);
        }
    }

    public static class IfThen { //Holds a condition and an action to be executed if it is met
        public ReturningFunc<Boolean> condition;
        public NonLinearAction action;

        public IfThen(ReturningFunc<Boolean> condition, NonLinearAction action) {
            this.condition = condition;
            this.action = action;
        }
    }

    public static class ConditionalAction extends NonLinearAction implements MappedActionGroup<ReturningFunc<Boolean>> { //Executes actions if their respective conditions are met, in an if,else-if,else manner. Only one action can run at a time. If one action's condition stops being met, it will finish, unless another action's condition starts being met, in which case it will stop and switch to that action
        public LinkedHashMap<ReturningFunc<Boolean>, NonLinearAction> actions = new LinkedHashMap<>();
        public NonLinearAction currentAction = null;
        public ConditionalAction(IfThen... conditionalPairs) {
            for (IfThen conditionalPair : conditionalPairs) {
                actions.put(conditionalPair.condition, conditionalPair.action);
            }
            registerActions(actions.values().toArray(new NonLinearAction[0]));
        }

        @Override
        boolean runProcedure() {
            if (isStart) {
                for (ReturningFunc<Boolean> condition : actions.keySet()) {
                    if (condition.call()) {
                        if (actions.get(condition) != currentAction) {
                            currentAction.stop();
                            currentAction = actions.get(condition);
                        }
                        break;
                    }
                }
                if (Objects.nonNull(currentAction)) {
                    currentAction.reset();
                }
            } else {
                for (ReturningFunc<Boolean> condition : actions.keySet()) {
                    if (condition.call()) {
                        if (actions.get(condition) != currentAction) {
                            currentAction.stop();
                            currentAction = actions.get(condition);
                            assert currentAction != null;
                            currentAction.reset();
                        }
                        break;
                    }
                }
            }
            if (Objects.nonNull(currentAction)) {
                if (!currentAction.run()) {
                    currentAction = null;
                    return false;
                } else {
                    return true;
                }
            } else return false;
        }

        @Override
        public void addActionProcedure(ReturningFunc<Boolean> key, NonLinearAction action) {
            actions.put(key,action);
        }
        @Override
        public void removeActionProcedure(NonLinearAction action) {
            for (ReturningFunc<Boolean> key: actions.keySet()){
                if (actions.get(key)==action){
                    if (currentAction==actions.get(key)){
                        currentAction=null;
                    }
                    actions.remove(key);
                }
            }
        }
    }

    public static class PersistentConditionalAction extends PersistentNonLinearAction implements MappedActionGroup<ReturningFunc<Boolean>>{ //ConditionalAction, but an action cannot be interrupted
        public LinkedHashMap<ReturningFunc<Boolean>, NonLinearAction> actions = new LinkedHashMap<>();
        public NonLinearAction currentAction = null;

        public PersistentConditionalAction(IfThen... conditionalPairs) {
            for (IfThen conditionalPair : conditionalPairs) {
                actions.put(conditionalPair.condition, conditionalPair.action);
            }
            registerActions(actions.values().toArray(new NonLinearAction[0]));
        }

        @Override
        boolean runProcedure() {
            if (Objects.isNull(currentAction)) {
                for (ReturningFunc<Boolean> condition : actions.keySet()) {
                    if (condition.call()) {
                        currentAction = actions.get(condition);
                        assert currentAction != null;
                        currentAction.reset();
                        break;
                    }
                }
            }
            if (Objects.nonNull(currentAction)) {
                if (!currentAction.run()) {
                    currentAction = null;
                    return false;
                } else {
                    return true;
                }
            } else return false;
        }
        @Override
        public void addActionProcedure(ReturningFunc<Boolean> key, NonLinearAction action) {
            actions.put(key,action);
        }
        @Override
        public void removeActionProcedure(NonLinearAction action) {
            for (ReturningFunc<Boolean> key: actions.keySet()){
                if (actions.get(key)==action){
                    if (currentAction==actions.get(key)){
                        currentAction=null;
                    }
                    actions.remove(key);
                }
            }
        }
    }

    public static class SemiPersistentConditionalAction extends NonLinearAction implements MappedActionGroup<ReturningFunc<Boolean>> { //ConditionalAction, but an action can only be interrupted if the SemiPersistentConditionalAction is reset()
        public LinkedHashMap<ReturningFunc<Boolean>, NonLinearAction> actions = new LinkedHashMap<>();
        public NonLinearAction currentAction = null;

        public SemiPersistentConditionalAction(IfThen... conditionalPairs) {
            for (IfThen conditionalPair : conditionalPairs) {
                actions.put(conditionalPair.condition, conditionalPair.action);
            }
            registerActions(actions.values().toArray(new NonLinearAction[0]));
        }

        @Override
        boolean runProcedure() {
            if (isStart) {
                for (ReturningFunc<Boolean> condition : actions.keySet()) {
                    if (condition.call()) {
                        if (actions.get(condition) != currentAction) {
                            if (Objects.nonNull(currentAction)) {
                                currentAction.stop();
                            }
                            currentAction = actions.get(condition);
                        }
                        break;
                    }
                }
                if (Objects.nonNull(currentAction)) {
                    currentAction.reset();
                }
            }
            if (Objects.nonNull(currentAction)) {
                if (!currentAction.run()) {
                    currentAction = null;
                    return false;
                } else {
                    return true;
                }
            } else return false;
        }
        @Override
        public void addActionProcedure(ReturningFunc<Boolean> key, NonLinearAction action) {
            actions.put(key,action);
        }
        @Override
        public void removeActionProcedure(NonLinearAction action) {
            for (ReturningFunc<Boolean> key: actions.keySet()){
                if (actions.get(key)==action){
                    if (currentAction==actions.get(key)){
                        currentAction=null;
                    }
                    actions.remove(key);
                }
            }
        }
    }

    public static class PressTrigger extends ConditionalAction { //ConditionalAction, but all conditions are converted into button-presses, such that they will not return 'true' two loop-iterations in a row.
        public ArrayList<Boolean> isPressed;

        public PressTrigger(IfThen... conditionalPairs) {
            super(conditionalPairs);
            actions.clear();
            isPressed = new ArrayList<>(conditionalPairs.length);
            for (int i = 0; i < conditionalPairs.length; i++) {
                int finalI = i;
                actions.put(
                        () -> {
                            if (conditionalPairs[finalI].condition.call()) {
                                isPressed.set(finalI,true);
                                return !isPressed.get(finalI);
                            } else {
                                isPressed.set(finalI,false);
                                return false;
                            }
                        },
                        conditionalPairs[i].action
                );
            }
        }
        @Override
        public void addAction(ReturningFunc<Boolean> condition, NonLinearAction action){
            actions.put(condition,action);
            isPressed.add(false);
        }
        @Override
        public void removeAction(NonLinearAction action){
            ReturningFunc<Boolean> matchingKey = null;
            int matchingIndex=0;
            for (ReturningFunc<Boolean> key:actions.keySet()){
                if (actions.get(key)==action){
                    matchingKey=key;
                }
                else{
                    matchingIndex++;
                }
            }
            actions.remove(matchingKey);
            isPressed.remove(matchingIndex);
        }
    }

    public static class PersistentPressTrigger extends PersistentConditionalAction { //PressTrigger but persistent
        public ArrayList<Boolean> isPressed;
        public PersistentPressTrigger(IfThen... conditionalPairs) {
            super(conditionalPairs);
            actions.clear();
            isPressed = new ArrayList<>(conditionalPairs.length);
            for (int i = 0; i < conditionalPairs.length; i++) {
                int finalI = i;
                actions.put(
                        () -> {
                            if (conditionalPairs[finalI].condition.call()) {
                                isPressed.set(finalI,true);
                                return !isPressed.get(finalI);
                            } else {
                                isPressed.set(finalI,false);
                                return false;
                            }
                        },
                        conditionalPairs[i].action
                );
            }
        }
        @Override
        public void addAction(ReturningFunc<Boolean> condition, NonLinearAction action){
            actions.put(condition,action);
            isPressed.add(false);
        }
        @Override
        public void removeAction(NonLinearAction action){
            ReturningFunc<Boolean> matchingKey = null;
            int matchingIndex=0;
            for (ReturningFunc<Boolean> key:actions.keySet()){
                if (actions.get(key)==action){
                    matchingKey=key;
                }
                else{
                    matchingIndex++;
                }
            }
            actions.remove(matchingKey);
            isPressed.remove(matchingIndex);
        }
    }

    public static class LoopForDuration extends NonLinearAction { //Loops an action for a certain duration
        double startTime;
        double duration;
        NonLinearAction action;

        public LoopForDuration(double duration, NonLinearAction action) {
            this.duration = duration;
            this.action = action;
        }

        @Override
        boolean runProcedure() {
            if (isStart) {
                startTime = timer.time();
                action.reset();
            }
            if ((timer.time() - startTime) < duration) {
                action.run();
                return true;
            } else {
                action.stop();
                return false;
            }
        }
    }

    public abstract static class SleepUntilPose extends SleepUntilTrue { //Sleeps until the drivetrain and heading get a certain distance from a desired position and heading, or until an optional timeout is reached
        public static ReturningFunc<double[]> getPose;
        public static void setGetPose(ReturningFunc<double[]> getPose){
            SleepUntilPose.getPose=getPose;
        }
        public SleepUntilPose(double x, double y, double heading, double poseDistance, double headingDistance, double timeout) {
            super(() -> {
                double[] pose = getPose.call();
                return Math.sqrt((x - pose[0]) * (x - pose[0]) + (y - pose[1]) * (y - pose[1])) < poseDistance &&
                        Math.abs(heading - pose[2]) < headingDistance;
            }, timeout);
        }

        public SleepUntilPose(double x, double y, double heading, double poseDistance, double headingDistance) {
            super(() -> {
                double[] pose = getPose.call();
                return Math.sqrt((x - pose[0]) * (x - pose[0]) + (y - pose[1]) * (y - pose[1])) < poseDistance &&
                        Math.abs(heading - pose[2]) < headingDistance;
            });
        }
    }

    public abstract static class PathAction<E> extends NonLinearAction { //Action for autonomous pathing. Must be subclassed to create an implementation for a specific autonomous library. Parameterized to the actual path object it is based off of.
        ReturningFunc<E> buildPath;
        public E path; //Stores the path this action follows. For RR it would be a TrajectoryAction, for Pedro it would be a PathChain
        public PathAction(ReturningFunc<E> buildPath) {
            this.buildPath = buildPath;
        }
        @Override
        boolean runProcedure() {
            if (isStart) {
                preBuild();
                path = buildPath.call(); //Path is built when the action needs to run (useful for RoadRunner)
            }
            return followPath();
        }

        abstract boolean followPath(); //Here, one implements the autonomous library's method of following paths. The function must return true if the path is still being followed, and false if it has finished

        public void preBuild() {
        } //Here, one can code anything that must occur right before the path is built.
    }

    public static class RobotCentricMecanumAction extends NonLinearAction { //Action for robot-centric TeleOp drivetrain control
        private final ReturningFunc<Double> xFun;
        private final ReturningFunc<Double> yFun;
        private final ReturningFunc<Double> rxFun;
        private final Condition slowDownFun;
        private final BotMotor[] motors;

        public RobotCentricMecanumAction(BotMotor[] motors, ReturningFunc<Double> xFun, ReturningFunc<Double> yFun, ReturningFunc<Double> rxFun, Condition slowDownFun) {
            this.xFun = xFun;
            this.yFun = yFun;
            this.rxFun = rxFun;
            this.slowDownFun = slowDownFun;
            this.motors = motors;
        }

        public RobotCentricMecanumAction(BotMotor[] motors, ReturningFunc<Double> xFun, ReturningFunc<Double> yFun, ReturningFunc<Double> rxFun) {
            this(motors, xFun, yFun, rxFun, null);
        }

        @Override
        public boolean runProcedure() {
            double y = -yFun.call();
            double x = xFun.call();
            double rx = -rxFun.call();

            double botHeading = 0;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (slowDownFun != null && slowDownFun.call()) { // Checks for left trigger input, slows all motors by 25%
                frontLeftPower = 0.75 * (rotY + rotX + rx) / denominator;
                backLeftPower = 0.75 * (rotY - rotX + rx) / denominator;
                frontRightPower = 0.75 * (rotY - rotX - rx) / denominator;
                backRightPower = 0.75 * (rotY + rotX - rx) / denominator;
            }

            motors[0].setPower(frontLeftPower);
            motors[1].setPower(backLeftPower);
            motors[2].setPower(frontRightPower);
            motors[3].setPower(backRightPower);
            return false;
        }
    }

    public static class FieldCentricMecanumAction extends NonLinearAction { //Action for field-centric TeleOp drivetrain control
        private final ReturningFunc<Double> xFun;
        private final ReturningFunc<Double> yFun;
        private final ReturningFunc<Double> rxFun;
        private final Condition slowDownFun;
        private final BotMotor[] motors;
        private final IMU imu;

        public FieldCentricMecanumAction(BotMotor[] motors, IMU imu, ReturningFunc<Double> xFun, ReturningFunc<Double> yFun, ReturningFunc<Double> rxFun, Condition slowDownFun) {
            this.xFun = xFun;
            this.yFun = yFun;
            this.rxFun = rxFun;
            this.slowDownFun = slowDownFun;
            this.motors = motors;
            this.imu = imu;
        }

        public FieldCentricMecanumAction(BotMotor[] motors, IMU imu, ReturningFunc<Double> xFun, ReturningFunc<Double> yFun, ReturningFunc<Double> rxFun) {
            this(motors, imu, xFun, yFun, rxFun, null);
        }

        @Override
        public boolean runProcedure() {
            double y = -yFun.call();
            double x = xFun.call();
            double rx = -rxFun.call();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (slowDownFun != null && slowDownFun.call()) { // Checks for left trigger input, slows all motors by 25%
                frontLeftPower = 0.75 * (rotY + rotX + rx) / denominator;
                backLeftPower = 0.75 * (rotY - rotX + rx) / denominator;
                frontRightPower = 0.75 * (rotY - rotX - rx) / denominator;
                backRightPower = 0.75 * (rotY + rotX - rx) / denominator;
            }

            motors[0].setPower(frontLeftPower);
            motors[1].setPower(backLeftPower);
            motors[2].setPower(frontRightPower);
            motors[3].setPower(backRightPower);
            return false;
        }
    }
    public static class LoopActionScheduler extends NonLinearParallelAction{ //Group of actions that runs actions in parallel in a while loop (used for TeleOp)
        public LoopActionScheduler(NonLinearAction...actions){
            super(actions);
        }
        public void runOnce(){
            reset(); run();
        }
        public void runLoop(Condition loopCondition) {
            while (loopCondition.call()) {
                runOnce();
            }
            stop();
        }
    }
    public static class LinearActionScheduler extends NonLinearSequentialAction { //Group of actions that runs actions sequentially (used for Autonomous)
        public LinearActionScheduler(NonLinearAction... actions) {
            super(actions);
        }
        public void runLinear() {
            while (run()){}
        }
    }
}
