package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Pinpoint adapter using the settings already present in the team's Perry.java:
 * 4-Bar pods, zero offsets, and both encoder directions FORWARD.
 *
 * Re-measure the physical offsets if the pod mounting or robot geometry changed.
 */
public final class PinpointLocalizer {
    private final GoBildaPinpointDriver pinpoint;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                RobotConfig.Hardware.PINPOINT
        );

        // These are inherited from the team's previous Pinpoint implementation.
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }

    /** Read a fresh sample. Call once per robot loop. */
    public void update() {
        pinpoint.update();
    }

    public double getXMillimeters() {
        return pinpoint.getPosX(DistanceUnit.MM);
    }

    public double getYMillimeters() {
        return pinpoint.getPosY(DistanceUnit.MM);
    }

    public double getHeadingDegrees() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public int getForwardEncoderTicks() {
        return pinpoint.getEncoderX();
    }

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
}
