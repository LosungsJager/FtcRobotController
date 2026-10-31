# Defined migration — Lösungsjäger

## Scope of this branch

- Preserve the repository's FTC SDK 12.0.0 dependency versions.
- Add Defined Core and FTC 0.3.1 dependencies.
- Move 29 legacy Java OpModes outside the Android Gradle main source set.
- Add a Defined-based Robot lifecycle, Drive and Intake subsystems, reusable Intake Actions, and primary TeleOp.
- Add a Pinpoint adapter and a sensor-only diagnostics OpMode.
- Add GitHub Actions compilation of the TeamCode module.

## Pinpoint configuration migrated from Perry.java

The old team OpMode used HardwareMap name `pinpoint`, goBILDA 4-Bar pods, offsets 0.0 mm / 0.0 mm, and both encoder directions FORWARD. These settings are now centralized in `PinpointLocalizer`.

The settings are inherited from existing code, not newly measured. Re-check the mechanical installation and offsets before relying on field coordinates. The diagnostics OpMode displays X/Y in millimeters, heading, raw encoder counts, frequency, and loop time.

## Compatibility and build

Defined 0.3.1 is added alongside the existing FTC SDK 12.0.0 dependencies. The upstream Defined Quickstart uses SDK 11.2.0, so this combination is validated by the GitHub Actions compile workflow in this branch rather than assumed compatible.

## Safety

- Pinpoint IMU reset/recalibration must happen while the robot is stationary.
- Verify motor direction with wheels lifted before driving.
- Test each actuator separately before using a full TeleOp or Autonomous.
- No hardware test has been performed by this change.
