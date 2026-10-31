# Defined migration — Lösungsjäger

## Scope of this branch

- Preserve the repository's FTC SDK 12.0.0 dependency versions.
- Add Defined Core and FTC 0.3.1 dependencies.
- Move 29 legacy Java OpModes outside the Android Gradle main source set.
- Add a Defined-based Robot lifecycle, Drive and Intake subsystems, reusable Intake Actions, and primary TeleOp.
- Add a Pinpoint adapter and a sensor-only diagnostics OpMode.

## Pinpoint calibration still required

The HardwareMap name `Pinpoint` is confirmed. The following physical values are not yet known and are intentionally not guessed:

- Odometry pod model (goBILDA Swingarm or 4-Bar, or a different pod).
- X pod offset from robot tracking center (mm).
- Y pod offset from robot tracking center (mm).
- X encoder direction and Y encoder direction.

Until these are configured, diagnostics reports raw encoder ticks and heading only. Do not use X/Y displacement for autonomous navigation yet.

## Compatibility and build

Defined 0.3.1 is added alongside the existing FTC SDK 12.0.0 dependencies. The upstream Defined Quickstart uses SDK 11.2.0, so this combination needs an actual Gradle compile check. The branch includes an Android build workflow to run that check on GitHub Actions.

## Safety

- Pinpoint IMU reset/recalibration must happen while the robot is stationary.
- Verify motor direction with wheels lifted before driving.
- Test each actuator separately before using a full TeleOp or Autonomous.
- No hardware test has been performed by this change.
