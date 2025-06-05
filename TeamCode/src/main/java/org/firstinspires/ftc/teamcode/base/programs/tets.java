package org.firstinspires.ftc.teamcode.base.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.NonLinearActions.executor;
import static org.firstinspires.ftc.teamcode.base.programs.RunConfigs.TestServo.testMotor;
import static org.firstinspires.ftc.teamcode.base.programs.RunConfigs.TestServo.testServo;

import org.firstinspires.ftc.teamcode.base.NonLinearActions;

@TeleOp
public class tets extends LinearOpMode {
    @Override
    public void runOpMode(){
        RunConfigs.TestServo.init(hardwareMap,telemetry);
        testMotor.resetEncoders();
        NonLinearActions.NonLinearAction sequence = new NonLinearActions.RunLoop(new NonLinearActions.NonLinearSequentialAction(
                testServo.moveToTargetAction(180),
                testServo.moveToTargetAction(0)
        ));
        NonLinearActions.ActionHolder holder = new NonLinearActions.ActionHolder();
        waitForStart();
        executor.setActions(
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
                new NonLinearActions.RunResettingLoop(
                        NonLinearActions.triggeredToggleAction(
                                ()->(gamepad1.a),
                                new NonLinearActions.InstantAction(()->holder.setAction(
                                        NonLinearActions.triggeredToggleAction(
                                                () -> (gamepad1.b),
                                                new NonLinearActions.InstantAction(()->executor.addAction(sequence)),
                                                new NonLinearActions.InstantAction(()->executor.removeAction(sequence))
                                        )
                                )),
                                new NonLinearActions.InstantAction(holder::removeAction)
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
