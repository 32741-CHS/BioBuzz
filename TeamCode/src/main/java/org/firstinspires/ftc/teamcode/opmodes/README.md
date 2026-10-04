# OpModes

(This is an AI Generated README.md file and is correct as of 4/10 3:10pm)
This directory contains the FTC OpModes used to operate and test the robot.

## Structure

* `teleop/` — Driver-controlled programs.
* `auto/` — Autonomous programs.

## TeleOp

The main driver-controlled OpMode is:

`MainTeleOp`

It coordinates the drivetrain, intake, shooter, turret and vision systems.

## Autonomous

The autonomous routines use the same subsystem classes as TeleOp.

Current autonomous programs include:

* `DriveAndShootClose`
* `DriveAndShootFar`
* `DriveForTime`

Each autonomous routine is intended for a specific movement or scoring sequence.

## Design

OpModes are responsible for coordinating robot behaviour. Mechanism-specific logic should generally
remain inside the relevant subsystem rather than being duplicated across multiple OpModes.
