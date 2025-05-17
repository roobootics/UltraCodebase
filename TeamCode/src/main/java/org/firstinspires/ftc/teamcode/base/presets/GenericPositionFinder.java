package org.firstinspires.ftc.teamcode.base.presets;

import static org.firstinspires.ftc.teamcode.base.Components.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Components;

import org.firstinspires.ftc.teamcode.base.NonLinearActions;
import static org.firstinspires.ftc.teamcode.base.programs.RunConfigs.TestServo.testServo;

import java.util.ArrayList;
import java.util.Objects;
public abstract class GenericPositionFinder extends LinearOpMode { //Used to find the specific positions that we will end up setting actuators to. Allows one to select an actuator and move it.
    public int selectedActuatorIndex = 0;
    public ArrayList<String> actuatorNames = new ArrayList<>();
    @Override
    public void runOpMode(){
        this.run();
    }
    public void updateTelemetry(){
        Components.telemetryAddLine(actuatorNames.get(selectedActuatorIndex));
        Components.telemetryAddData("position", Objects.requireNonNull(actuators.get(actuatorNames.get(selectedActuatorIndex))).getCurrentPosition());
        Components.telemetryAddData("target",testServo.getPosition());
        Components.telemetryAddData("e",testServo.parts.get("test").getPosition());
    }
    public void shiftSelectionRight(){
        if (selectedActuatorIndex<actuatorNames.size()-1){
            selectedActuatorIndex+=1;
        }
    }
    public void shiftSelectionLeft(){
        if (selectedActuatorIndex>0){
            selectedActuatorIndex-=1;
        }
    }
    public void run(){
        for (String name:actuators.keySet()){
            if (!(actuators.get(name) instanceof Components.BotMotor)){
                Objects.requireNonNull(actuators.get(name)).switchControl(Objects.requireNonNull(actuators.get(name)).defaultControlKey);
            }
            else{
                Components.BotMotor motor = Objects.requireNonNull((Components.BotMotor) actuators.get(name));
                motor.resetEncoders();
                motor.setZeroPowerFloat();
                //Motors are disabled; to find their positions you move them manually and read the encoder
            }
            actuatorNames.add(name);
        }
        NonLinearActions.IfThen[] conditions = new NonLinearActions.IfThen[actuatorNames.size()];
        for (int i=0;i<actuatorNames.size();i++){
            int finalI = i;
            conditions[i]=new NonLinearActions.IfThen(
                    ()->(selectedActuatorIndex==finalI),
                    Objects.requireNonNull(actuators.get(actuatorNames.get(i))).triggeredDynamicAction(()->(gamepad1.left_bumper),()->(gamepad1.right_bumper),0.1)
            );
        }

        waitForStart();
        new NonLinearActions.ParallelActionExecutor(
                new NonLinearActions.RunResettingLoop(
                        new NonLinearActions.PressTrigger(new NonLinearActions.IfThen(
                                ()->(gamepad1.dpad_left),
                                new NonLinearActions.InstantAction(this::shiftSelectionLeft)
                        )),
                        new NonLinearActions.PressTrigger(new NonLinearActions.IfThen(
                                ()->(gamepad1.dpad_right),
                                new NonLinearActions.InstantAction(this::shiftSelectionRight)
                        )),
                        new NonLinearActions.ConditionalAction(
                                conditions
                        )
                ),
                new NonLinearActions.WriteToTelemetry(this::updateTelemetry),
                new NonLinearActions.PowerOnCommand()
        ).runLoop(this::opModeIsActive);
    }
}
