# Subsystems

(This is an AI Generated README.md file and is correct as of 4/10 3:10pm)
This directory contains the reusable mechanism-control classes used by the robot.

Each subsystem is responsible for controlling a particular part of the robot rather than putting all
robot logic inside the OpModes.

## Drivetrain

`Drivetrain.java`

Controls the four mecanum drive motors.

Features include:

* Forward/backward movement
* Strafing
* Rotation
* Joystick deadband
* Speed limiting
* Field-relative driving
* IMU reset

## Intake

`Intake.java`

Controls the intake motor.

The intake can run normally or in reverse. Commands are stored until `update()` applies the
requested motor power.

## Shooter

`Shooter.java`

Controls the flywheel and feeder.

The flywheel uses velocity control with configurable proportional and feed-forward constants.

The feeder is only activated when the flywheel is within the configured speed tolerance of its
target.

## Turret

`Turret.java`

Controls the shooting turret.

The turret uses its encoder position to estimate its current angle and uses proportional control
with feed-forward to move toward the desired angle.

The turret also has:

* Minimum and maximum angle limits
* A soft limit zone
* Limited acceleration
* A home position
* Manual and automatic control support

## Vision

`Vision.java`

Uses the robot webcam and FTC AprilTag detection to identify targets.

The camera is configured at 640 × 480 resolution and uses camera exposure and gain settings intended
for AprilTag detection.

The class provides access to all current detections and allows a detection to be retrieved by its
tag ID.

## Ballistics

`Ballistics.java`

Contains calculations used by the shooting system.

It currently provides:

* Turret angle calculation from target bearing
* Flywheel speed calculation from target distance

Flywheel speed is selected using a linear interpolation lookup table.

## GamepadEx

`GamepadEx.java`

Provides a wrapper around the FTC gamepad and exposes higher-level button and trigger handling.

It works with the utility classes in `utils/`.

## General Design

Subsystems should contain mechanism-specific behaviour. OpModes should primarily coordinate these
subsystems rather than directly manipulating hardware.
