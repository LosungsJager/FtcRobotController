# Defined migration — Lösungsjäger

## Scope of this branch

- Preserve the repository's FTC SDK 12.0.0 dependency versions.
- Add Defined Core and FTC 0.3.1 dependencies.
- Move 29 legacy Java OpModes outside the Android Gradle main source set.
- Add a first active Robot, Drive, Intake, IntakeActions, and TeleOp structure.
- Keep the `master` branch unchanged.

## Deliberately not implemented yet

- Pinpoint driver initialization and localization actions.
- Autonomous routes and path following.
- Encoder distance conversion.
- Any unconfirmed servo, launcher, or other mechanism.
- Automated build validation in this environment.

## Compatibility note

The Defined Quickstart uses FTC SDK 11.2.0 while this repository currently uses 12.0.0. This branch keeps 12.0.0 and adds Defined 0.3.1, but a Gradle build must verify binary/API compatibility before merging.

## Hardware assumptions requiring bench validation

- Left drive motors are configured REVERSE and right drive motors FORWARD.
- Intake uses the `coletor` motor name.
- The drivetrain uses four REV HD Hex 41 (1291) motors with an external 5:1 reduction.
- Pinpoint is named `Pinpoint`; physical pod offsets and encoder directions remain unprovided.
