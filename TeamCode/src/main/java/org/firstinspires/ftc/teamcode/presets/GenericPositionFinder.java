package org.firstinspires.ftc.teamcode.presets;

import static org.firstinspires.ftc.teamcode.base.Components.actuators;
import static org.firstinspires.ftc.teamcode.base.NonLinearActions.executor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Components;

import org.firstinspires.ftc.teamcode.base.NonLinearActions;

import java.util.ArrayList;
import java.util.Objects;
//Dpad left/right to switch between actuators. If the actuator is a servo, use left/right bumper to move it and see the position.
public abstract class GenericPositionFinder extends LinearOpMode { //Used to find the specific positions that we will end up setting actuators to. Allows one to select an actuator and move it.
    //Subclass it to work with a specific PartsConfig. Call the init function of that PartsConfig before calling the runOpMode of this class.
    private int selectedActuatorIndex = 0;
    private final ArrayList<String> actuatorNames = new ArrayList<>();
    public void updateTelemetry(){
        Components.telemetryAddLine(actuatorNames.get(selectedActuatorIndex));
        Components.telemetryAddData("position", Objects.requireNonNull(actuators.get(actuatorNames.get(selectedActuatorIndex))).getCurrentPosition());
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
    @Override
    public void runOpMode(){
        for (String name:actuators.keySet()){
            if (!(actuators.get(name) instanceof Components.BotMotor)){
                Objects.requireNonNull(actuators.get(name)).switchControl(Objects.requireNonNull(actuators.get(name)).getDefaultControlKey());
            }
            else{
                Components.BotMotor motor = Objects.requireNonNull((Components.BotMotor) actuators.get(name));
                motor.resetEncoders();
                motor.setZeroPowerFloat();
                motor.lockTargetState();
                //Motors are disabled; to find their positions you move them manually and read the encoder
            }
            actuatorNames.add(name);
        }
        NonLinearActions.IfThen[] conditions = new NonLinearActions.IfThen[actuatorNames.size()];
        for (int i=0;i<actuatorNames.size();i++){
            int finalI = i;
            conditions[i]=new NonLinearActions.IfThen(
                    ()->(selectedActuatorIndex==finalI),
                    Objects.requireNonNull(actuators.get(actuatorNames.get(i))).triggeredDynamicTargetAction(()->(gamepad1.left_bumper),()->(gamepad1.right_bumper),0.1)
            );
        }

        waitForStart();
        executor.setWriteToTelemetry(this::updateTelemetry);
        executor.setActions(
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
                new NonLinearActions.PowerOnCommand()
        );
        executor.runLoop(this::opModeIsActive);
    }
}
