package org.firstinspires.ftc.teamcode.base;

import static org.firstinspires.ftc.teamcode.base.Components.actuators;
import static org.firstinspires.ftc.teamcode.base.Components.timer;

import static org.firstinspires.ftc.teamcode.base.Components.BotMotor;
import static org.firstinspires.ftc.teamcode.base.Components.updateTelemetry;

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
    public static final ParallelActionExecutor executor=new ParallelActionExecutor(); //This runs NonLinearActions
    private static final HashMap<String, Double> initialActuatorTargets = new HashMap<>();
    public abstract static class NonLinearAction { //Base class for any action
        private boolean isBusy = false; //Indicates whether the action is active or not
        private boolean isStart = true; //Actions know if they've just started running or not
        private boolean isEnabled = true;
        public void reset() {
            isStart = true;
        }

        final public boolean run() { //Actual method called to run the action, called repeatedly in a loop. Returns true if the action is not complete and false if it is.
            if (isEnabled){
                isBusy = runProcedure();
                isStart = false;
                if (!isBusy) {
                    reset(); //Auto-reset after action is completed or stopped.
                }
                return isBusy;
            }
            else{
                return false;
            }
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
        final public boolean isBusy(){
            return isBusy;
        }
        final public boolean isStart(){
            return isStart;
        }
        final public boolean isEnabled(){
            return isEnabled;
        }
        final public void enable(){
            isEnabled=true;
        }
        final public void disable(){
            if (isBusy) {
                stopProcedure();
                isBusy=false;
            }
            isEnabled=false;
        }
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
    public static class ActionHolder extends NonLinearAction{ //An action that can run a given action when run. The action it runs can be changed.
        //The idea behind this is that you can build sequences using an ActionHolder, and set, change, or remove the action inside the ActionHolder later on without having to rebuild the entire sequence again.
        private NonLinearAction action = null;
        public ActionHolder(NonLinearAction initialAction){
            setAction(initialAction);
        }
        public ActionHolder(){}
        @Override
        boolean runProcedure() {
            if (Objects.nonNull(action)){
                return action.run();
            }
            else{
                return false;
            }
        }
        @Override
        public void stopProcedure(){
            action.stop();
        }
        public void setAction(NonLinearAction action){
            if (Objects.nonNull(this.action)){
                this.action.stop();
            }
            action.reset();
            this.action=action;
        }
        public NonLinearAction getAction(){
            return this.action;
        }
        public void removeAction(){
            if (Objects.nonNull(this.action)){
                action.stop();
                action=null;
            }
        }
    }
    public static class PowerOnCommand extends NonLinearAction { //This action automatically activates each actuator's default control functions when they are first commanded
        private final HashMap<String, Boolean> actuatorsCommanded = new HashMap<>();
        @Override
        boolean runProcedure() {
            if (actuatorsCommanded.size()<actuators.size()) {
                for (String key : actuators.keySet()) {
                    if (Objects.requireNonNull(actuators.get(key)).getTarget()!=Objects.requireNonNull(initialActuatorTargets.get(key)) && !actuatorsCommanded.containsKey(key)) {
                        Objects.requireNonNull(actuators.get(key)).switchControl(Objects.requireNonNull(actuators.get(key)).getDefaultControlKey());
                        actuatorsCommanded.put(key, true);
                    }
                }
                return true;
            }
            else{
                disable();
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
    public static class NonLinearSequentialAction extends NonLinearAction{ //Runs actions sequentially
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
    }

    public static class NonLinearParallelAction extends NonLinearAction{ //Runs actions in parallel
        private ArrayList<NonLinearAction> remainingActions;
        protected final ArrayList<NonLinearAction> actions;
        public NonLinearParallelAction(NonLinearAction... actions) {
            this.actions = new ArrayList<>(Arrays.asList(actions));
            this.remainingActions = new ArrayList<>(this.actions);
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
    }

    public static class IfThen { //Holds a condition and an action to be executed if it is met
        private final ReturningFunc<Boolean> condition;
        private final NonLinearAction action;

        public IfThen(ReturningFunc<Boolean> condition, NonLinearAction action) {
            this.condition = condition;
            this.action = action;
        }
    }

    public static class ConditionalAction extends NonLinearAction{ //Executes actions if their respective conditions are met, in an if,else-if,else manner. Only one action can run at a time. If one action's condition stops being met, it will finish, unless another action's condition starts being met, in which case it will stop and switch to that action
        protected final LinkedHashMap<ReturningFunc<Boolean>, NonLinearAction> actions = new LinkedHashMap<>();
        protected NonLinearAction currentAction = null;
        public ConditionalAction(IfThen... conditionalPairs) {
            for (IfThen conditionalPair : conditionalPairs) {
                actions.put(conditionalPair.condition, conditionalPair.action);
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
        public NonLinearAction getCurrentAction(){
            return currentAction;
        }
        @Override
        public void stopProcedure(){
            if (Objects.nonNull(currentAction)){
                currentAction.stop();
            }
        }
    }

    public static class PersistentConditionalAction extends PersistentNonLinearAction{ //ConditionalAction, but an action cannot be interrupted
        protected final LinkedHashMap<ReturningFunc<Boolean>, NonLinearAction> actions = new LinkedHashMap<>();
        protected NonLinearAction currentAction = null;

        public PersistentConditionalAction(IfThen... conditionalPairs) {
            for (IfThen conditionalPair : conditionalPairs) {
                actions.put(conditionalPair.condition, conditionalPair.action);
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
        public NonLinearAction getCurrentAction(){
            return currentAction;
        }
        @Override
        public void stopProcedure(){
            if (Objects.nonNull(currentAction)){
                currentAction.stop();
            }
        }
    }

    public static class SemiPersistentConditionalAction extends NonLinearAction{ //ConditionalAction, but an action can only be interrupted if the SemiPersistentConditionalAction is reset()
        private final LinkedHashMap<ReturningFunc<Boolean>, NonLinearAction> actions = new LinkedHashMap<>();
        private NonLinearAction currentAction = null;

        public SemiPersistentConditionalAction(IfThen... conditionalPairs) {
            for (IfThen conditionalPair : conditionalPairs) {
                actions.put(conditionalPair.condition, conditionalPair.action);
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
        public NonLinearAction getCurrentAction(){
            return currentAction;
        }
        @Override
        public void stopProcedure(){
            if (Objects.nonNull(currentAction)){
                currentAction.stop();
            }
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
    }
    public static class LoopForDuration extends NonLinearParallelAction { //Loops an action for a certain duration
        private double startTime;
        private final double duration;
        public LoopForDuration(double duration, NonLinearAction...actions) {
            super(actions);
            this.duration = duration;
        }

        @Override
        public boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
                reset();
            }
            if ((timer.time() - startTime) < duration) {
                for (NonLinearAction action : actions){
                    action.run();
                }
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public static class LoopUntilTrue extends NonLinearParallelAction { //Loops an action until a condition is met, or until an optional timeout is reached
        private double startTime;
        private final Condition condition;
        private final double timeout;

        public LoopUntilTrue(Condition condition, double timeout, NonLinearAction... actions) {
            super(actions);
            this.condition = condition;
            this.timeout=timeout;
        }
        public LoopUntilTrue(Condition condition, NonLinearAction... actions) {
            this(condition,Double.POSITIVE_INFINITY,actions);
        }
        @Override
        public boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
                reset();
            }
            if (!condition.call() && (timer.time() - startTime) < timeout) {
                for (NonLinearAction action : actions){
                    action.run();
                }
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public static class ResetAndLoopForDuration extends NonLinearParallelAction { //Loops an action for a certain duration
        private double startTime;
        private final double duration;

        public ResetAndLoopForDuration(double duration, NonLinearAction... actions) {
            super(actions);
            this.duration = duration;
        }

        @Override
        public boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
            }
            if ((timer.time() - startTime) < duration) {
                reset(); super.runProcedure();
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public static class ResetAndLoopUntilTrue extends NonLinearParallelAction { //Loops an action until a condition is met, or until an optional timeout is reached
        private double startTime;
        private final Condition condition;
        private final double timeout;

        public ResetAndLoopUntilTrue(Condition condition, double timeout, NonLinearAction...actions) {
            super(actions);
            this.condition = condition;
            this.timeout=timeout;
        }
        public ResetAndLoopUntilTrue(Condition condition, NonLinearAction... actions) {
            this(condition,Double.POSITIVE_INFINITY,actions);
        }
        @Override
        public boolean runProcedure() {
            if (isStart()) {
                startTime = timer.time();
            }
            if (!condition.call() && (timer.time() - startTime) < timeout) {
                reset(); super.runProcedure();
                return true;
            } else {
                stop();
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
        private final ReturningFunc<Double> slowDownFun;
        private final BotMotor[] motors;

        public RobotCentricMecanumAction(BotMotor[] motors, ReturningFunc<Double> xFun, ReturningFunc<Double> yFun, ReturningFunc<Double> rxFun, ReturningFunc<Double> slowDownFun) {
            this.xFun = xFun;
            this.yFun = yFun;
            this.rxFun = rxFun;
            this.slowDownFun = slowDownFun;
            this.motors = motors;
        }

        public RobotCentricMecanumAction(BotMotor[] motors, ReturningFunc<Double> xFun, ReturningFunc<Double> yFun, ReturningFunc<Double> rxFun) {
            this(motors, xFun, yFun, rxFun, ()->(1.0));
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
            double slowDownAmount=slowDownFun.call();
            double frontLeftPower = slowDownAmount*(rotY + rotX + rx) / denominator;
            double backLeftPower = slowDownAmount*(rotY - rotX + rx) / denominator;
            double frontRightPower = slowDownAmount*(rotY - rotX - rx) / denominator;
            double backRightPower = slowDownAmount*(rotY + rotX - rx) / denominator;

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
        private final ReturningFunc<Double> slowDownFun;
        private final BotMotor[] motors;
        private final ReturningFunc<Double> getHeading;

        public FieldCentricMecanumAction(BotMotor[] motors, ReturningFunc<Double> getHeading, int imuPollingRate, ReturningFunc<Double> xFun, ReturningFunc<Double> yFun, ReturningFunc<Double> rxFun, ReturningFunc<Double> slowDownFun) {
            this.xFun = xFun;
            this.yFun = yFun;
            this.rxFun = rxFun;
            this.slowDownFun = slowDownFun;
            this.motors = motors;
            this.getHeading = new Components.CachedReader<>(getHeading,imuPollingRate)::cachedRead;
        }

        public FieldCentricMecanumAction(BotMotor[] motors, ReturningFunc<Double> getHeading, int imuPollingRate, ReturningFunc<Double> xFun, ReturningFunc<Double> yFun, ReturningFunc<Double> rxFun) {
            this(motors, getHeading, imuPollingRate, xFun, yFun, rxFun, ()->(1.0));
        }

        @Override
        public boolean runProcedure() {
            double y = -yFun.call();
            double x = xFun.call();
            double rx = -rxFun.call();

            double botHeading = getHeading.call();

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double slowDownAmount=slowDownFun.call();
            double frontLeftPower = slowDownAmount*(rotY + rotX + rx) / denominator;
            double backLeftPower = slowDownAmount*(rotY - rotX + rx) / denominator;
            double frontRightPower = slowDownAmount*(rotY - rotX - rx) / denominator;
            double backRightPower = slowDownAmount*(rotY + rotX - rx) / denominator;

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
    public static PressTrigger triggeredFSMAction(Condition upCondition, Condition downCondition, int startingState, NonLinearAction...actions){
        if (startingState>actions.length){
            startingState= actions.length;
        }
        else if (startingState<0){
            startingState=0;
        }
        AtomicInteger state = new AtomicInteger(-1+startingState);
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
    public static class ParallelActionExecutor{
        private ArrayList<NonLinearAction> actions;
        private final ArrayList<NonLinearAction> actionsToAdd = new ArrayList<>();
        private final ArrayList<NonLinearAction> actionsToRemove = new ArrayList<>();
        private boolean isStartOfProgram = true;
        private Procedure writeToTelemetry = ()->{};
        private ParallelActionExecutor(){
        }
        public void setActions(NonLinearAction...commandGroups){
            this.actions =new ArrayList<>(Arrays.asList(commandGroups));
        }
        public ArrayList<NonLinearAction> getActions(){
            return new ArrayList<>(this.actions);
        }
        public void setWriteToTelemetry(Procedure procedure){
            this.writeToTelemetry=procedure;
        }
        public void runOnce(){
            if (isStartOfProgram){
                initialActuatorTargets.clear();
                for (String key:actuators.keySet()){
                    initialActuatorTargets.put(key, Objects.requireNonNull(actuators.get(key)).getTarget());
                }
                isStartOfProgram=false;
            }
            this.actions.addAll(actionsToAdd);
            this.actions.removeAll(actionsToRemove);
            actionsToAdd.clear();
            actionsToRemove.clear();
            this.actions = actions.stream().filter(NonLinearAction::run).collect(Collectors.toCollection(ArrayList::new));
            for (Components.Actuator<?> actuator : actuators.values()) {
                if (actuator.getDynamicTargetBoundaries()) { //If the actuator's target boundaries can change, this will ensure that the actuator's target never falls outside of the boundaries
                    actuator.setTarget(actuator.getTargetMinusOffset());
                }
                if (actuator instanceof Components.CRActuator && ((Components.CRActuator<?>) actuator).dynamicPowerBoundaries) { //If the CRActuator's power boundaries can change, this will ensure that the CRActuator's power never falls outside of the boundaries
                    Components.CRActuator<?> castedActuator = ((Components.CRActuator<?>) actuator);
                    castedActuator.setPower(castedActuator.getPower(castedActuator.partNames[0]));
                }
                actuator.runControl();
                actuator.resetNewTarget(); actuator.resetNewActuation();
            }
            Components.CachedReader.resetAllCaches();
            writeToTelemetry.call();
            updateTelemetry();
        }
        public void runLoop(Condition condition){
            while (condition.call()){
                runOnce();
            }
            stop();
        }
        public void stop(){
            for (NonLinearAction action : actions){
                action.stop();
            }
        }
        public void addAction(NonLinearAction action){
            action.reset();
            actionsToAdd.add(action);
        }
        public void removeAction(NonLinearAction action){
            action.stop();
            if (this.actions.contains(action)) {
                actionsToRemove.add(action);
            }
        }
    }
}
