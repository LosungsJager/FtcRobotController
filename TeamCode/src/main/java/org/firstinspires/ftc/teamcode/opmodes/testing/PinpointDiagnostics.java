package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PinpointLocalizer;

/**
 * Sensor-only diagnostic. Does not initialize or move the drivetrain/intake.
 * Keep the robot stationary during startup so the Pinpoint IMU can calibrate.
 */
@TeleOp(name = "TEST - Pinpoint Diagnostics", group = "Testing")
public final class PinpointDiagnostics extends LinearOpMode {
    @Override
    public void runOpMode() {
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap);

        telemetry.addLine("Pinpoint found. Keep robot stationary.");
        telemetry.addLine("This test does not move any motors.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            localizer.update();

            telemetry.addData("X (mm)", "%.1f", localizer.getXMillimeters());
            telemetry.addData("Y (mm)", "%.1f", localizer.getYMillimeters());
            telemetry.addData("Heading (deg)", "%.2f", localizer.getHeadingDegrees());
            telemetry.addData("Forward encoder (ticks)", localizer.getForwardEncoderTicks());
            telemetry.addData("Strafe encoder (ticks)", localizer.getStrafeEncoderTicks());
            telemetry.addData("Pinpoint frequency (Hz)", "%.1f", localizer.getFrequencyHz());
            telemetry.addData("Pinpoint loop (us)", localizer.getLoopTimeMicros());
            telemetry.update();
            idle();
        }
    }
}
