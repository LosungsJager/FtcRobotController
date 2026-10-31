package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Intake motor subsystem; does not read gamepad input. */
public final class Intake {
    private final DcMotorEx motor;
    private double requestedPower = RobotConfig.Intake.STOP_POWER;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConfig.Hardware.INTAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0.0);
    }

    public void start(boolean reversed) {
        requestedPower = reversed
                ? RobotConfig.Intake.OUT_POWER
                : RobotConfig.Intake.IN_POWER;
    }

    public void stop() {
        requestedPower = RobotConfig.Intake.STOP_POWER;
    }

    public boolean isRunning() {
        return Math.abs(requestedPower) > 0.001;
    }

    public void update() {
        motor.setPower(requestedPower);
    }
}
