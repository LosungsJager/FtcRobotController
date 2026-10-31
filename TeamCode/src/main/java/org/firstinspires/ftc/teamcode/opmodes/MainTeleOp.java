package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.teamundefined.defined.ftc.RobotOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.actions.IntakeActions;

/** Primary TeleOp for the new Defined-based architecture. */
@TeleOp(name = "Lösungsjäger - TeleOp", group = "Competition")
public final class MainTeleOp extends RobotOpMode<Robot> {
    @Override
    protected Robot createRobot() {
        return new Robot(hardwareMap);
    }

    @Override
    protected void onRobotInit() {
        robot = createRobot();
        runner.addMonitor(IntakeActions.toggle(robot, () -> gamepad1.aWasReleased()));
    }

    @Override
    protected void onLoop(long nowMs) {
        robot.drive.driveMecanum(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
    }
}
