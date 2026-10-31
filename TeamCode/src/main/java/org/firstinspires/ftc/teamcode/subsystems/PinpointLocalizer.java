package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Thin adapter around the FTC SDK goBILDA Pinpoint driver.
 *
 * Encoder resolution, pod offsets, and encoder directions are deliberately
 * not guessed here. Configure them only after measuring the installed pods.
 */
public final class PinpointLocalizer {
    private final GoBildaPinpointDriver pinpoint;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                RobotConfig.Hardware.PINPOINT
        );
    }

    /** Read a fresh sample. Call once per robot loop. */
    public void update() {
        pinpoint.update();
    }

    /** Current heading in degrees. */
    public double getHeadingDegrees() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    /** Raw forward-pod encoder count; useful before calibration. */
    public int getForwardEncoderTicks() {
        return pinpoint.getEncoderX();
    }

    /** Raw strafe-pod encoder count; useful before calibration. */
    public int getStrafeEncoderTicks() {
        return pinpoint.getEncoderY();
    }

    public double getFrequencyHz() {
        return pinpoint.getFrequency();
    }

    public int getLoopTimeMicros() {
        return pinpoint.getLoopTime();
    }

    /**
     * Call only while the robot is completely stationary.
     * Resets the local pose and recalibrates the internal IMU.
     */
    public void resetPositionAndImu() {
        pinpoint.resetPosAndIMU();
    }

    /**
     * Exposes the SDK driver for a future calibration step after the pod model,
     * measured offsets, and encoder directions have been confirmed.
     */
    public GoBildaPinpointDriver getDriver() {
        return pinpoint;
    }
}
