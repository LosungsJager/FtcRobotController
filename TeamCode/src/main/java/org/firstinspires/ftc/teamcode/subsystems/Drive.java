package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Four-motor mecanum drivetrain. */
public final class Drive {
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;

    public Drive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, RobotConfig.Hardware.LEFT_FRONT);
        leftRear = hardwareMap.get(DcMotorEx.class, RobotConfig.Hardware.LEFT_REAR);
        rightFront = hardwareMap.get(DcMotorEx.class, RobotConfig.Hardware.RIGHT_FRONT);
        rightRear = hardwareMap.get(DcMotorEx.class, RobotConfig.Hardware.RIGHT_REAR);

        leftFront.setDirection(RobotConfig.Drive.LEFT_DIRECTION);
        leftRear.setDirection(RobotConfig.Drive.LEFT_DIRECTION);
        rightFront.setDirection(RobotConfig.Drive.RIGHT_DIRECTION);
        rightRear.setDirection(RobotConfig.Drive.RIGHT_DIRECTION);

        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public void driveMecanum(double forward, double strafe, double turn) {
        double denominator = Math.max(1.0,
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));

        double lf = (forward + strafe + turn) / denominator;
        double lr = (forward - strafe + turn) / denominator;
        double rf = (forward - strafe - turn) / denominator;
        double rr = (forward + strafe - turn) / denominator;

        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);
    }

    public void stop() {
        driveMecanum(0.0, 0.0, 0.0);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        rightRear.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftRear.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightRear.setZeroPowerBehavior(behavior);
    }
}
