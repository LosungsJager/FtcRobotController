# Lösungsjäger — Defined architecture

This package is the active starting point for the 2026–2027 robot code.

## Structure

- `Robot.java`: composition root and Defined lifecycle.
- `config/RobotConfig.java`: HardwareMap names and tuning constants.
- `subsystems/Drive.java`: four-motor mecanum drivetrain.
- `subsystems/Intake.java`: collector motor.
- `subsystems/PinpointLocalizer.java`: goBILDA Pinpoint localization adapter.
- `actions/IntakeActions.java`: reusable Defined actions.
- `opmodes/MainTeleOp.java`: primary TeleOp.
- `opmodes/testing/PinpointDiagnostics.java`: sensor-only diagnostic.

## HardwareMap names

| Device | Name |
|---|---|
| Left front drive motor | `esq_f` |
| Left rear drive motor | `esq_t` |
| Right front drive motor | `dir_f` |
| Right rear drive motor | `dir_t` |
| Intake motor | `coletor` |
| goBILDA Pinpoint | `pinpoint` |

The Pinpoint name and its legacy configuration were read from the team's previous `Perry.java`.

Drive motors: REV HD Hex 41 (1291), external 5:1 reduction. The reduction is documented; wheel distance conversion is not yet used by the drivetrain.

## Pinpoint configuration

The adapter currently reuses the previous team settings:

- Pod type: goBILDA 4-Bar.
- X/Y offsets: 0.0 mm / 0.0 mm.
- X/Y encoder directions: FORWARD / FORWARD.

These are inherited settings, not newly measured values. Confirm the pod installation and offsets on the physical robot. The Pinpoint diagnostics OpMode displays X, Y, heading, raw encoder ticks, frequency, and loop time.

## Defined dependencies

Defined Core and FTC 0.3.1 are added from the library's Maven repository. The existing FTC SDK 12.0.0 version is retained. GitHub Actions compiles the TeamCode module; hardware behavior still requires bench testing.

## Safety checks before first run

1. Lift the drivetrain so all wheels are clear of the floor.
2. Verify each HardwareMap name in the Robot Configuration.
3. Keep the robot stationary during Pinpoint IMU reset/calibration.
4. Verify motor directions one motor at a time at low power.
5. Confirm the intake direction and stop behavior.
6. Keep the emergency stop accessible.
