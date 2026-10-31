package org.firstinspires.ftc.teamcode.actions;

import com.teamundefined.defined.Action;
import com.teamundefined.defined.generic.ToggleAction;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.function.BooleanSupplier;

/** Reusable Defined actions for the intake. */
public final class IntakeActions {
    private IntakeActions() {}

    public static Action start(Robot robot, boolean reversed) {
        return Action.oneShot("intake_start", now -> robot.intake.start(reversed))
                .requires(Subsystem.INTAKE);
    }

    public static Action stop(Robot robot) {
        return Action.oneShot("intake_stop", now -> robot.intake.stop())
                .requires(Subsystem.INTAKE);
    }

    public static Action toggle(Robot robot, BooleanSupplier button) {
        return ToggleAction.onPress(
                "intake_toggle",
                button,
                start(robot, false),
                stop(robot)
        ).withStateProbe(robot.intake::isRunning);
    }
}
