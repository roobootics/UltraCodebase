package org.firstinspires.ftc.teamcode.base.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.programs.RunConfigs.TestServo.testServo;

import org.firstinspires.ftc.teamcode.base.NonLinearActions;

@TeleOp
public class tets extends LinearOpMode {
    @Override
    public void runOpMode(){
        RunConfigs.TestServo.init(hardwareMap,telemetry);
        waitForStart();
        NonLinearActions.ParallelActionExecutor executor = new NonLinearActions.ParallelActionExecutor(
                new NonLinearActions.RunLoop(
                    testServo.setTargetAction(()->(270-testServo.getTarget()))
                ),
                new NonLinearActions.PowerOnCommand(),
                new NonLinearActions.WriteToTelemetry(
                    ()->{
                        telemetryAddData("a",testServo.getTarget());
                        telemetryAddData("e",testServo.getCurrentPosition());
                    }
                )
        );
        executor.runLoop(this::opModeIsActive);
    }
}
