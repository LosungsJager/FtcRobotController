# Lösungsjäger — Defined architecture

This package is the active starting point for the 2026–2027 robot code.

## Structure

- `Robot.java`: composition root and Defined lifecycle.
- `config/RobotConfig.java`: HardwareMap names and tuning constants.
- `subsystems/Drive.java`: four-motor mecanum drivetrain.
- `subsystems/Intake.java`: collector motor.
- `actions/IntakeActions.java`: reusable Defined actions.
- `opmodes/MainTeleOp.java`: primary TeleOp.

## Confirmed HardwareMap names

| Device | Name |
|---|---|
| Left front drive motor | `esq_f` |
| Left rear drive motor | `esq_t` |
| Right front drive motor | `dir_f` |
| Right rear drive motor | `dir_t` |
| Intake motor | `coletor` |
| goBILDA Pinpoint | `Pinpoint` |

Drive motors: REV HD Hex 41 (1291), external 5:1 reduction. The reduction is documented but no encoder-to-distance conversion is enabled yet.

## Pinpoint integration

The HardwareMap name is recorded, but the localizer is not yet initialized in code. Before enabling it, confirm the pod model, encoder directions, X/Y offsets, and heading orientation. These are physical calibration values and must not be guessed.

## Defined dependencies

Defined Core and FTC 0.3.1 are added from the library's Maven repository. The existing FTC SDK 12.0.0 version is retained. A Gradle build is required to confirm compatibility between those versions; this change has not been hardware-tested.

## Safety checks before first run

1. Lift the drivetrain so all wheels are clear of the floor.
2. Verify each HardwareMap name in the Robot Configuration.
3. Verify motor directions one motor at a time at low power.
4. Confirm the intake direction and stop behavior.
5. Keep the emergency stop accessible.
