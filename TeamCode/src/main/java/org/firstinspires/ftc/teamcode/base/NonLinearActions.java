package org.firstinspires.ftc.teamcode.base;

import static org.firstinspires.ftc.teamcode.base.Components.actuators;
import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.Components.timer;

import static org.firstinspires.ftc.teamcode.base.Components.BotMotor;
import static org.firstinspires.ftc.teamcode.base.Components.updateTelemetry;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.Procedure;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.ReturningFunc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Objects;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.Condition;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;

public abstract class NonLinearActions { //Command-based (or action-based) system
    public abstract static class NonLinearAction { //Base class for any action
        private boolean isBusy = false; //Indicates whether the action is active or not
        private boolean isStart = true; //Actions know if they've just started running or not
        private Procedure removeFromGroup; //If the action is part of an action group (everything but the scheduler,) this allows it to remove itself from the group.

        public void reset() {
            isStart = true;
        }

        final public boolean run() { //Actual method called to run the action, called repeatedly in a loop. Returns true if the action is not complete and false if it is.
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
        final public boolean isBusy(){
            return isBusy;
        }
        final public boolean isStart(){
            return isStart;
        }
    }
    public interface MappedActionGroup<K> { //NonLinearActions that run a group of other actions, and stores them with indexes or keys, should implement this. (SequentialActions, ConditionalActions)
        ArrayList<Procedure> scheduledRemovals = new ArrayList<>();
        ArrayList<Procedure> scheduledAdditions = new ArrayList<>();
        default void addAction(K key, NonLinearAction action) { //Method that schedules the adding of an action for the end of the loop (required to avoid concurrent modifications)
            action.registerRemoveFromGroup(() -> this.removeAction(key));
            action.reset();
            scheduledAdditions.add(()->addActionProcedure(key, action));
        }

        void addActionProcedure(K key, NonLinearAction action); //Allows action groups to add actions to the group

        default void removeAction(K key){ //Method that schedules the removal of an action for the end of the loop (required to avoid concurrent modifications)
            scheduledRemovals.add(()->removeActionProcedure(key));
        }
        void removeActionProcedure(K key); //Allows action groups to remove actions from the group
        default InstantAction addToGroupAction(K key, NonLinearAction action){
            return new InstantAction(()->addAction(key,action));
        }
        default InstantAction removeFromGroupAction(K key){
            return new InstantAction(()->removeAction(key));
        }
    }

    public interface UnmappedActionGroup { //NonLinearActions that run an unmapped set of other actions should implement this. (ParallelActions)
        ArrayList<Procedure> scheduledRemovals = new ArrayList<>();
        ArrayList<Procedure> scheduledAdditions = new ArrayList<>();
        default void addAction(NonLinearAction action) { //Method that schedules the adding of an action for the end of the loop (required to avoid concurrent modifications)
            action.registerRemoveFromGroup(() -> this.removeAction(action));
            scheduledAdditions.add(()->addActionProcedure(action));
        }
        void addActionProcedure(NonLinearAction action); //Allows action groups to add actions to the group

        default void removeAction(NonLinearAction action){ //Method that schedules the removal of an action for the end of the loop (required to avoid concurrent modifications)
            action.stop();
            scheduledRemovals.add(()->removeActionProcedure(action));
        }
        void removeActionProcedure(NonLinearAction action); //Allows action groups to remove actions from the group
        default void registerActions(NonLinearAction...actions){
            for (NonLinearAction action : actions) {
                action.registerRemoveFromGroup(() -> this.removeAction(action));
            }
        }
        default InstantAction addToGroupAction(NonLinearAction action){
            return new InstantAction(()->addAction(action));
        }
        default InstantAction removeFromGroupAction(NonLinearAction action){
            return new InstantAction(()->removeAction(action));
        }
    }
    public static void conductActionModifications(){ //Adds and removes all actions that need to be added or removed. Called by top-level action schedulers at the end of each loop iteration
        for (Procedure add:MappedActionGroup.scheduledAdditions){
            add.call();
        }
        for (Procedure remove:MappedActionGroup.scheduledRemovals){
            remove.call();
        }
        for (Procedure add:UnmappedActionGroup.scheduledAdditions){
            add.call();
        }
        for (Procedure remove:UnmappedActionGroup.scheduledAdditions){
            remove.call();
        }
        MappedActionGroup.scheduledAdditions.clear();
        MappedActionGroup.scheduledRemovals.clear();
        UnmappedActionGroup.scheduledAdditions.clear();
        UnmappedActionGroup.scheduledRemovals.clear();
    }

    //TEMPLATE ACTIONS

    public abstract static class PersistentNonLinearAction extends NonLinearAction { //NonLinearAction but it can't be reset if not completed
        @Override
        public void reset() {
            if (!isBusy()) {
                super.reset();
            }
        }
    }

    public static class InstantAction extends NonLinearAction { //Action that completes in one loop iteration
        private final Procedure procedure;

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
        private final Procedure procedure;

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
        private final ReturningFunc<Boolean> action;

        public LambdaAction(ReturningFunc<Boolean> action) {
            this.action = action;
        }

        @Override
        public boolean runProcedure() {
            return action.call();
        }
    }

    public abstract static class CompoundAction extends NonLinearAction { //Allows one to represent a sequence of actions as one atomic action. The main difference between this and a sequential action is that you can code custom stop functionality.
        public NonLinearAction group; //A subclass will assign an action to this.
        @Override
        boolean runProcedure() {
            if (isStart()) {
                group.reset();
            }
            return group.run();
        }

        @Override
        public void stopProcedure() {
            group.stop();
        }
    }

    //PRELOADED ACTIONS
    public static class WriteToTelemetry extends ContinuousAction { //This action runs each actuator's control functions and updates the telemetry using the updateTelemetry function it is provided
        public WriteToTelemetry(Procedure updateTelemetry) {
            super(updateTelemetry);
        }
        public WriteToTelemetry() {
            super(() -> {
                for (Components.Actuator<?> actuator : actuators.values()) {
                    telemetryAddData(actuator.getName() + " target", actuator.getTarget());
                    telemetryAddData(actuator.getName() + " instant target", actuator.getInstantTarget());
                    telemetryAddData(actuator.getName() + " current position", actuator.getCurrentPosition());
                    telemetryAddData("", "");
                }
            });
        }
    }

    public static class PowerOnCommand extends NonLinearAction { //This action automatically activates each actuator's default control functions when they are first commanded
        private final HashMap<String, Boolean> actuatorsCommanded = new HashMap<>();
        @Override
        boolean runProcedure() {
            if (actuatorsCommanded.size()<actuators.size()) {
                for (String key : actuators.keySet()) {
                    if (Objects.requireNonNull(actuators.get(key)).isNewTarget() && !actuatorsCommanded.containsKey(key)) {
                        Objects.requireNonNull(actuators.get(key)).switchControl(Objects.requireNonNull(actuators.get(key)).getDefaultControlKey());
                        actuatorsCommanded.put(key, true);
                    }
                }
                return true;
            }
            else{
                removeFromGroup();
                return false;
            }

        }
    }
    public static class SleepUntilTrue extends NonLinearAction { //Sleeps until a condition is met or until an optional timeout time is reached
        private final ReturningFunc<Boolean> condition;
        private final double timeout;
        private double startTime;

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
            if (isStart() && timeout != Double.POSITIVE_INFINITY) {
                startTime = timer.time();
            }
            return !condition.call() && (timer.time() - startTime) < timeout;
        }
    }

    public static class NonLinearSleepAction extends NonLinearAction { //Sleeps for a set time
        private final double time;
        private double startTime;

        public NonLinearSleepAction(double time) {
            this.time = time;
        }

        @Override
        boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
            }
            return (timer.time() - startTime) < time;
        }
    }
    public static class InterruptOnTimeout extends NonLinearAction{ //Action to run the action passed to it, but interrupt it after it's been running for a given time
        private final NonLinearAction action;
        private final NonLinearSleepAction sleepAction;
        boolean actionEnded=false;
        public InterruptOnTimeout(double time, NonLinearAction action){
            this.action=action;
            this.sleepAction=new NonLinearSleepAction(time);
        }
        @Override
        boolean runProcedure() {
            if (isStart()){
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
        private final NonLinearAction action;
        private final SleepUntilTrue sleepAction;
        private boolean actionEnded=false;
        public InterruptOnCondition(Condition condition, NonLinearAction action){
            this.action=action;
            this.sleepAction=new SleepUntilTrue(condition);
        }
        @Override
        boolean runProcedure() {
            if (isStart()){
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
        private ArrayList<NonLinearAction> remainingActions;
        private final ArrayList<NonLinearAction> actions;
        private final ArrayList<Boolean> isStarts;

        public NonLinearSequentialAction(NonLinearAction... actions) {
            this.actions = new ArrayList<>(Arrays.asList(actions));
            this.remainingActions = new ArrayList<>(this.actions);
            isStarts = new ArrayList<>(actions.length);
            for (NonLinearAction action : actions) {
                isStarts.add(true);
            }
            for (int i=0;i<actions.length;i++){
                int finalI = i;
                this.actions.get(i).registerRemoveFromGroup(()->removeAction(finalI));
            }
        }

        @Override
        public boolean runProcedure() {
            if (isStart()) {
                remainingActions = new ArrayList<>(actions);
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
        public void removeActionProcedure(Integer key) {
            Objects.requireNonNull(actions.get(key)).stop();
            isStarts.remove((int) key);
            actions.remove((int) key);
            remainingActions.remove((int) key);
        }
    }

    public static class NonLinearParallelAction extends NonLinearAction implements UnmappedActionGroup { //Runs actions in parallel
        private ArrayList<NonLinearAction> remainingActions;
        protected final ArrayList<NonLinearAction> actions;
        public NonLinearParallelAction(NonLinearAction... actions) {
            this.actions = new ArrayList<>(Arrays.asList(actions));
            this.remainingActions = new ArrayList<>(this.actions);
            registerActions(actions);
        }

        @Override
        public boolean runProcedure() {
            if (isStart()) {
                remainingActions = new ArrayList<>(actions);
                for (NonLinearAction action : remainingActions) {
                    action.reset();
                }
            }
            ArrayList<NonLinearAction> removingActions = new ArrayList<>();
            for (NonLinearAction action:remainingActions){
                if(!action.run()){
                    removingActions.add(action);
                }
            }
            for (NonLinearAction action:removingActions){
                remainingActions.remove(action);
            }
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
        private final ReturningFunc<Boolean> condition;
        private final NonLinearAction action;

        public IfThen(ReturningFunc<Boolean> condition, NonLinearAction action) {
            this.condition = condition;
            this.action = action;
        }
    }

    public static class ConditionalAction extends NonLinearAction implements MappedActionGroup<ReturningFunc<Boolean>> { //Executes actions if their respective conditions are met, in an if,else-if,else manner. Only one action can run at a time. If one action's condition stops being met, it will finish, unless another action's condition starts being met, in which case it will stop and switch to that action
        protected final LinkedHashMap<ReturningFunc<Boolean>, NonLinearAction> actions = new LinkedHashMap<>();
        protected NonLinearAction currentAction = null;
        public ConditionalAction(IfThen... conditionalPairs) {
            for (IfThen conditionalPair : conditionalPairs) {
                actions.put(conditionalPair.condition, conditionalPair.action);
                conditionalPair.action.registerRemoveFromGroup(()->removeAction(conditionalPair.condition));
            }
        }

        @Override
        boolean runProcedure() {
            if (isStart()) {
                for (ReturningFunc<Boolean> condition : actions.keySet()) {
                    if (condition.call()) {
                        if (actions.get(condition) != currentAction) {
                            if (Objects.nonNull(currentAction)) {
                                currentAction.stop();
                            }
                            currentAction=actions.get(condition);
                        }
                        if (Objects.nonNull(currentAction)) {
                            currentAction.reset();
                        }
                        break;
                    }
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
        public void removeActionProcedure(ReturningFunc<Boolean> condition) {
            Objects.requireNonNull(actions.get(condition)).stop();
            if (currentAction==actions.get(condition)){
                currentAction=null;
            }
            actions.remove(condition);
        }
    }

    public static class PersistentConditionalAction extends PersistentNonLinearAction implements MappedActionGroup<ReturningFunc<Boolean>>{ //ConditionalAction, but an action cannot be interrupted
        protected final LinkedHashMap<ReturningFunc<Boolean>, NonLinearAction> actions = new LinkedHashMap<>();
        protected NonLinearAction currentAction = null;

        public PersistentConditionalAction(IfThen... conditionalPairs) {
            for (IfThen conditionalPair : conditionalPairs) {
                actions.put(conditionalPair.condition, conditionalPair.action);
                conditionalPair.action.registerRemoveFromGroup(()->removeAction(conditionalPair.condition));
            }
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
        public void removeActionProcedure(ReturningFunc<Boolean> condition) {
            Objects.requireNonNull(actions.get(condition)).stop();
            if (currentAction==actions.get(condition)){
                currentAction=null;
            }
            actions.remove(condition);
        }
    }

    public static class SemiPersistentConditionalAction extends NonLinearAction implements MappedActionGroup<ReturningFunc<Boolean>> { //ConditionalAction, but an action can only be interrupted if the SemiPersistentConditionalAction is reset()
        private final LinkedHashMap<ReturningFunc<Boolean>, NonLinearAction> actions = new LinkedHashMap<>();
        private NonLinearAction currentAction = null;

        public SemiPersistentConditionalAction(IfThen... conditionalPairs) {
            for (IfThen conditionalPair : conditionalPairs) {
                actions.put(conditionalPair.condition, conditionalPair.action);
                conditionalPair.action.registerRemoveFromGroup(()->removeAction(conditionalPair.condition));
            }
        }

        @Override
        boolean runProcedure() {
            if (isStart()) {
                for (ReturningFunc<Boolean> condition : actions.keySet()) {
                    if (condition.call()) {
                        if (actions.get(condition) != currentAction) {
                            if (Objects.nonNull(currentAction)) {
                                currentAction.stop();
                            }
                            currentAction = actions.get(condition);
                            if (Objects.nonNull(currentAction)) {
                                currentAction.reset();
                            }
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
        public void removeActionProcedure(ReturningFunc<Boolean> condition) {
            Objects.requireNonNull(actions.get(condition)).stop();
            if (currentAction==actions.get(condition)){
                currentAction=null;
            }
            actions.remove(condition);
        }
    }

    public static class PressTrigger extends ConditionalAction { //ConditionalAction, but all conditions are converted into button-presses, such that they will not return 'true' two loop-iterations in a row.
        private final ArrayList<Boolean> isPressed;

        public PressTrigger(IfThen... conditionalPairs) {
            super(conditionalPairs);
            actions.clear();
            isPressed = new ArrayList<>();
            for (int i = 0; i < conditionalPairs.length; i++) {
                int finalI = i;
                isPressed.add(false);
                actions.put(
                        () -> {
                            if (conditionalPairs[finalI].condition.call()) {
                                if (!isPressed.get(finalI)){
                                    isPressed.set(finalI,true);
                                    return true;
                                }
                                else{
                                    return false;
                                }
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
            isPressed.add(false);
            int index=isPressed.size()-1;
            actions.put(() -> {
                if (condition.call()) {
                    if (!isPressed.get(index)){
                        isPressed.set(index,true);
                        return true;
                    }
                    else{
                        return false;
                    }
                } else {
                    isPressed.set(index,false);
                    return false;
                }
            },action);
        }
        @Override
        public void removeAction(ReturningFunc<Boolean> condition){
            Objects.requireNonNull(actions.get(condition)).stop();
            if (currentAction==actions.get(condition)){
                currentAction=null;
            }
            ReturningFunc<Boolean> matchingKey = null;
            int matchingIndex=0;
            for (ReturningFunc<Boolean> key:actions.keySet()){
                if (key.equals(condition)){
                    break;
                }
                matchingIndex++;
            }
            actions.remove(condition);
            isPressed.remove(matchingIndex);
        }
    }

    public static class PersistentPressTrigger extends PersistentConditionalAction { //PressTrigger but persistent
        private final ArrayList<Boolean> isPressed;
        public PersistentPressTrigger(IfThen... conditionalPairs) {
            super(conditionalPairs);
            actions.clear();
            isPressed = new ArrayList<>();
            for (int i = 0; i < conditionalPairs.length; i++) {
                int finalI = i;
                isPressed.add(false);
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
            isPressed.add(false);
            int index=isPressed.size()-1;
            actions.put(() -> {
                if (condition.call()) {
                    if (!isPressed.get(index)){
                        isPressed.set(index,true);
                        return true;
                    }
                    else{
                        return false;
                    }
                } else {
                    isPressed.set(index,false);
                    return false;
                }
            },action);
        }
        @Override
        public void removeAction(ReturningFunc<Boolean> condition){
            Objects.requireNonNull(actions.get(condition)).stop();
            if (currentAction==actions.get(condition)){
                currentAction=null;
            }
            ReturningFunc<Boolean> matchingKey = null;
            int matchingIndex=0;
            for (ReturningFunc<Boolean> key:actions.keySet()){
                if (key.equals(condition)){
                    break;
                }
                matchingIndex++;
            }
            actions.remove(condition);
            isPressed.remove(matchingIndex);
        }
    }
    public static class LoopForDuration extends NonLinearAction { //Loops an action for a certain duration
        private double startTime;
        private final double duration;
        private final NonLinearAction action;

        public LoopForDuration(double duration, NonLinearAction action) {
            this.duration = duration;
            this.action = action;
        }

        @Override
        boolean runProcedure() {
            if (isStart()) {
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
    public static class LoopUntilTrue extends NonLinearAction { //Loops an action until a condition is met, or until an optional timeout is reached
        private double startTime;
        private final Condition condition;
        private final double timeout;
        private final NonLinearAction action;

        public LoopUntilTrue(Condition condition, NonLinearAction action, double timeout) {
            this.condition = condition;
            this.action = action;
            this.timeout=timeout;
        }
        public LoopUntilTrue(Condition condition, NonLinearAction action) {
            this(condition,action,Double.POSITIVE_INFINITY);
        }
        @Override
        boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
                action.reset();
            }
            if (!condition.call() && (timer.time() - startTime) < timeout) {
                action.run();
                return true;
            } else {
                action.stop();
                return false;
            }
        }
    }
    public static class ResetAndLoopForDuration extends NonLinearAction { //Loops an action for a certain duration
        private double startTime;
        private final double duration;
        private final NonLinearAction action;

        public ResetAndLoopForDuration(double duration, NonLinearAction action) {
            this.duration = duration;
            this.action = action;
        }

        @Override
        boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
            }
            if ((timer.time() - startTime) < duration) {
                action.reset(); action.run();
                return true;
            } else {
                action.stop();
                return false;
            }
        }
    }
    public static class ResetAndLoopUntilTrue extends NonLinearAction { //Loops an action until a condition is met, or until an optional timeout is reached
        private double startTime;
        private final Condition condition;
        private final double timeout;
        private final NonLinearAction action;

        public ResetAndLoopUntilTrue(Condition condition, NonLinearAction action, double timeout) {
            this.condition = condition;
            this.action = action;
            this.timeout=timeout;
        }
        public ResetAndLoopUntilTrue(Condition condition, NonLinearAction action) {
            this(condition,action,Double.POSITIVE_INFINITY);
        }
        @Override
        boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
            }
            if (!condition.call() && (timer.time() - startTime) < timeout) {
                action.reset(); action.run();
                return true;
            } else {
                action.stop();
                return false;
            }
        }
    }
    public abstract static class SleepUntilPose extends SleepUntilTrue { //Sleeps until the drivetrain and heading get a certain distance from a desired position and heading, or until an optional timeout is reached. Meant to be subclassed depending on the pathing library.
        public static ReturningFunc<double[]> getPose; //Subclasses assign this to a method that can get the drivetrain position, returning x, y, and heading.
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
        private final ReturningFunc<E> buildPath;
        private E path; //Stores the path this action follows. For RR it would be a TrajectoryAction, for Pedro it would be a PathChain
        public PathAction(ReturningFunc<E> buildPath) {
            this.buildPath = buildPath;
        }
        @Override
        boolean runProcedure() {
            if (isStart()) {
                preBuild();
                path = buildPath.call(); //Path is built when the action needs to run (useful for RoadRunner)
            }
            return followPath();
        }

        abstract boolean followPath(); //Here, one implements the autonomous library's method of following paths. The function must return true if the path is still being followed, and false if it has finished

        public void preBuild() {
        } //Here, one can code anything that must occur right before the path is built.
        public E getPath(){
            return path;
        }
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
    public static PressTrigger triggeredToggleAction(Condition condition, NonLinearAction action1, NonLinearAction action2){
        AtomicBoolean state = new AtomicBoolean(true);
        action1 = new NonLinearSequentialAction(new InstantAction(()->state.set(!state.get())), action1);
        action2 = new NonLinearSequentialAction(new InstantAction(()->state.set(!state.get())), action2);
        return new PressTrigger(
                new IfThen(condition,
                        new SemiPersistentConditionalAction(
                                new IfThen(state::get,action1),
                                new IfThen(()->(!state.get()),action2)
                        )
                )
        );
    }
    public static PressTrigger triggeredFSMAction(Condition upCondition, Condition downCondition, NonLinearAction...actions){
        AtomicInteger state = new AtomicInteger(-1);
        IfThen[] upIfThens = new IfThen[actions.length];
        for (int i=0;i<upIfThens.length;i++){
            int finalI = i;
            upIfThens[i]=new IfThen(
                    ()->(state.get()==finalI),
                    actions[i]
            );
        }
        IfThen[] downIfThens = new IfThen[actions.length];
        for (int i=0;i<downIfThens.length;i++){
            int finalI = i;
            downIfThens[i]=new IfThen(
                    ()->(state.get()==finalI),
                    actions[i]
            );
        }
        return new PressTrigger(
                new IfThen(upCondition,
                        new NonLinearSequentialAction(
                                new InstantAction(()->{if (state.get()<actions.length-1){state.set(state.get()+1);}}),
                                new SemiPersistentConditionalAction(
                                        upIfThens
                                )
                        )
                ),
                new IfThen(downCondition,
                    new NonLinearSequentialAction(
                            new InstantAction(()->{if (state.get()>0){state.set(state.get()-1);}}),
                            new SemiPersistentConditionalAction(
                                    downIfThens
                            )
                    )
                )
        );
    }
    public static PressTrigger triggeredCycleAction(Condition condition, NonLinearAction...actions){
        AtomicInteger state = new AtomicInteger(0);
        IfThen[] ifThens=new IfThen[actions.length];
        for (int i=0;i< ifThens.length;i++){
            int finalI = i;
            ifThens[i]=new IfThen(
                    ()->(state.get()==finalI),
                    new NonLinearSequentialAction(new InstantAction(()->{
                        if (state.get()==actions.length-1){
                            state.set(0);
                        }
                        else{state.set(state.get()+1);}
                    }), actions[finalI])
            );
        }
        return new PressTrigger(
                new IfThen(condition,
                    new SemiPersistentConditionalAction(
                        ifThens
                    )
                )
        );
    }
    public static class RunResettingLoop extends NonLinearParallelAction{ //Group of actions that runs actions in parallel in a while loop and resets them each iteration (used for TeleOp)
        public RunResettingLoop(NonLinearAction...actions){
            super(actions);
        }
        public boolean runProcedure(){
            reset(); super.runProcedure();
            return true;
        }
        public void stopProcedure(){
            for (NonLinearAction action : actions){
                action.stop();
            }
        }
    }
    public static class RunLoop extends NonLinearParallelAction{ //Group of actions that runs actions in parallel in a while loop
        public RunLoop(NonLinearAction...actions){
            super(actions);
        }
        public boolean runProcedure(){
            for (NonLinearAction action : actions){
                action.run();
            }
            return true;
        }
        public void stopProcedure(){
            for (NonLinearAction action : actions){
                action.stop();
            }
        }
    }
    public static class ParallelActionExecutor implements UnmappedActionGroup{
        private ArrayList<NonLinearAction> commandGroups;
        public ParallelActionExecutor(NonLinearAction...commandGroups){
            this.commandGroups=new ArrayList<>(Arrays.asList(commandGroups));
            registerActions(commandGroups);
        }
        public void runOnce(){
            conductActionModifications();
            this.commandGroups=commandGroups.stream().filter(NonLinearAction::run).collect(Collectors.toCollection(ArrayList::new));
            for (Components.Actuator<?> actuator : actuators.values()) {
                if (actuator.getDynamicTargetBoundaries()) { //If the actuator's target boundaries can change, this will ensure that the actuator's target never falls outside of the boundaries
                    actuator.setTarget(actuator.getTargetMinusOffset());
                }
                if (actuator instanceof Components.CRActuator && ((Components.CRActuator<?>) actuator).dynamicPowerBoundaries) { //If the CRActuator's power boundaries can change, this will ensure that the CRActuator's power never falls outside of the boundaries
                    Components.CRActuator<?> castedActuator = ((Components.CRActuator<?>) actuator);
                    castedActuator.setPower(castedActuator.getPower(castedActuator.partNames[0]));
                }
                actuator.runControl();
                actuator.resetCurrentPositions();
                actuator.resetNewTarget();
            }
            updateTelemetry();
        }
        public void runLoop(Condition condition){
            while (condition.call()){
                runOnce();
            }
            stop();
        }
        public void stop(){
            for (NonLinearAction commandGroup : commandGroups){
                commandGroup.stop();
            }
        }
        @Override
        public void addActionProcedure(NonLinearAction commandGroup) {
            commandGroups.add(commandGroup);
        }
        @Override
        public void removeActionProcedure(NonLinearAction commandGroup) {
            commandGroups.remove(commandGroup);
        }
    }
}
