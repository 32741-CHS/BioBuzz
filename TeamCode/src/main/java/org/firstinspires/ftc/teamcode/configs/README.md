# Configs

(This is an AI Generated README.md file and is correct as of 4/10 3:10pm)
This directory contains global configuration and hardware-mapping classes.

## Files

### `RobotHardwareGroup.java`

Contains references to every piece of robot hardware used by the code.

Hardware currently mapped includes:

* Front-left drive motor: `flDrive`
* Front-right drive motor: `frDrive`
* Back-left drive motor: `blDrive`
* Back-right drive motor: `brDrive`
* Turret motor: `turret`
* Flywheel motor: `flywheel`
* Feeder motor: `feeder`
* Intake motor: `intake`
* IMU: `imu`
* Webcam: `webcam`

All hardware is initialised through `RobotHardwareGroup.init()`.

If the hardware configuration names change, update this file.

### `Globals.java`

Contains constants and shared values used across the project.

Examples include:

* Motor encoder tick rates
* Wheel diameter
* Joystick deadband
* AprilTag IDs for the goals
* Vision frame rate
* Shooter tolerance
* Trigger threshold

The shared `robotHardwareGroup` instance is also defined here.

## Hardware Mapping Rule

Hardware should be accessed through `RobotHardwareGroup` rather than being mapped independently
inside each subsystem. This keeps hardware configuration in one location and makes wiring or
configuration changes easier to manage.
