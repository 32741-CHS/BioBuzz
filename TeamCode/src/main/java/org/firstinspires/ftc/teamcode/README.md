# TeamCode

(This is an AI Generated README.md file and is correct as of 4/10 3:10pm)
This directory contains the robot control code for the FTC robot.

## Project Structure

* `configs/` — Robot hardware configuration and global constants.
* `opmodes/` — FTC OpModes, including TeleOp and autonomous routines.
* `pedroPathing/` — Pedro Pathing configuration and tuning code.
* `subsystems/` — Reusable classes that control individual robot mechanisms.
* `utils/` — General-purpose utility classes used throughout the project.

## Main Robot Systems

The robot is controlled through several independent subsystems:

* **Drivetrain** — Four-wheel mecanum drive with optional field-relative driving.
* **Intake** — Controls the intake motor and intake direction.
* **Shooter** — Controls the flywheel and feeder, including flywheel velocity control.
* **Turret** — Controls the shooting turret using position feedback.
* **Vision** — Detects AprilTags using the robot's webcam.
* **Ballistics** — Converts AprilTag information into turret angles and flywheel speeds.

## Control Structure

`MainTeleOp` coordinates the subsystems during driver-controlled operation. Each subsystem handles
its own hardware and control logic, while the OpMode determines when each subsystem should be used.

Autonomous OpModes use the same subsystem classes rather than directly controlling individual
motors.

## Hardware Configuration

All hardware mappings are contained in:

`configs/RobotHardwareGroup.java`

If a hardware device is renamed in the Robot Controller configuration, its name should be updated in
this file rather than throughout the rest of the project.

## External Libraries

The project uses FTC SDK functionality and additional libraries including:

* Pedro Pathing
* Panels telemetry/graphing
* Panels camera streaming
* Configurables

## Important Notes

Before running the robot, ensure that the configured hardware names match those in
`RobotHardwareGroup.java`.

Subsystem constants and tuning values are generally kept inside the subsystem where they are used.
