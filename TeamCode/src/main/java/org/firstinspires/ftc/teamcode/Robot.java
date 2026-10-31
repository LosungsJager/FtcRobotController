package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PinpointLocalizer;

import java.util.List;

/**
 * Composition root for the 2026-2027 robot.
 * Keep hardware initialization here and gamepad logic inside OpModes.
 */
public final class Robot extends com.teamundefined.defined.ftc.Robot {
    public final Drive drive;
    public final Intake intake;
    public final PinpointLocalizer localization;

    private final List<LynxModule> hubs;

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        localization = new PinpointLocalizer(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void init() {
        // As in the previous Perry OpMode: zero pose and recalibrate IMU during INIT.
        // The robot must remain completely stationary during this calibration.
        localization.resetPositionAndImu();
    }

    @Override
    public void initUpdate() {
        // Keep this non-blocking; use it for sensor warm-up when needed.
    }

    @Override
    public void start(boolean isTeleOp) {
        drive.stop();
        intake.stop();
    }

    @Override
    public void preUpdate(long nowMs) {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        localization.update();
    }

    @Override
    public void setOpModeTime(double seconds) {
        // Available for match-time-dependent behavior.
    }

    @Override
    public void update(long nowMs) {
        intake.update();
    }

    @Override
    public void stop() {
        drive.stop();
        intake.stop();
    }
}
