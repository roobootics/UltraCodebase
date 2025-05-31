package org.firstinspires.ftc.teamcode.base.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.programs.RunConfigs.TestServo.testMotor;

import org.firstinspires.ftc.teamcode.base.NonLinearActions;

@TeleOp
public class tets extends LinearOpMode {
    @Override
    public void runOpMode(){
        RunConfigs.TestServo.init(hardwareMap,telemetry);
        testMotor.resetEncoders();
        waitForStart();
        NonLinearActions.ParallelActionExecutor executor = new NonLinearActions.ParallelActionExecutor(
                new NonLinearActions.ResetAndLoopForDuration(
                        10,
                       NonLinearActions.triggeredFSMAction(
                               ()->(gamepad1.right_bumper),
                               ()->(gamepad1.left_bumper),
                               2,
                               testMotor.moveToTargetAction(50),
                               testMotor.moveToTargetAction(200),
                               testMotor.moveToTargetAction(500)
                       )
                ),
                new NonLinearActions.PowerOnCommand(),
                new NonLinearActions.WriteToTelemetry(
                    ()->{
                        telemetryAddData("a",testMotor.getInstantTarget());
                        telemetryAddData("e",testMotor.getCurrentPosition());
                        telemetryAddData("vel",testMotor.getVelocity());
                    }
                )
        );
        executor.runLoop(this::opModeIsActive);
    }
}
