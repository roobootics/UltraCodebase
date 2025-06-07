package org.firstinspires.ftc.teamcode.base.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.base.Components.telemetryAddData;
import static org.firstinspires.ftc.teamcode.base.Components.timer;
import static org.firstinspires.ftc.teamcode.base.NonLinearActions.executor;
import static org.firstinspires.ftc.teamcode.base.programs.RunConfigs.TestServo.profile;
import static org.firstinspires.ftc.teamcode.base.programs.RunConfigs.TestServo.testMotor;
import static org.firstinspires.ftc.teamcode.base.programs.RunConfigs.TestServo.testServo;

import org.firstinspires.ftc.teamcode.base.NonLinearActions;

@TeleOp
public class tets extends LinearOpMode {
    public double prevTime;
    @Override
    public void runOpMode(){
        RunConfigs.TestServo.init(hardwareMap,telemetry);
        testMotor.resetEncoders();
        waitForStart();
        executor.setWriteToTelemetry(()->{
            telemetryAddData("a",testMotor.getInstantTarget());
            telemetryAddData("target",testMotor.getTarget());
            telemetryAddData("e",testMotor.getCurrentPosition());
            telemetryAddData("vel",testMotor.getVelocity());
            telemetryAddData("control",testMotor.getCurrControlFuncKey());
            telemetryAddData("targetVel", profile.getProfileData().get("targetVelocity"));
            telemetryAddData("phase", profile.getPhase());
        });
        executor.setActions(
                new NonLinearActions.RunResettingLoop(testMotor.triggeredDynamicTargetAction(()->(gamepad1.right_bumper),()->(gamepad1.left_bumper),2)),
                new NonLinearActions.PowerOnCommand()
        );
        executor.runLoop(this::opModeIsActive);
    }
}
