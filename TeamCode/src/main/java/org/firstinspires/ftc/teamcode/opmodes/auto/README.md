# Autonomous OpModes

(This is an AI Generated README.md file and is correct as of 4/10 3:10pm)
This directory contains the autonomous routines currently available for the robot.

## `DriveAndShootClose`

Drives toward the shooting position, spins up the flywheel, tracks the goal using AprilTags, shoots
for a set period, and then drives away.

The close routine uses a longer initial drive period before shooting.

Key configurable values include:

* Drive time
* Drive power
* Shooting time
* Maximum flywheel spin-up time

## `DriveAndShootFar`

Starts from the far shooting position, spins up the flywheel while tracking the goal, shoots for a
set period, and then drives forward.

Unlike the close routine, the far routine does not perform the initial drive before spinning up the
shooter.

## `DriveForTime`

A simple movement routine used to drive the robot for a specified amount of time.

It includes an initial waiting period followed by a timed drive.

This OpMode is useful for simple autonomous movement and testing.

## Alliance Selection

The shooting autonomous routines allow the alliance to be selected during the initialization period.

* Gamepad X selects Red.
* Gamepad A selects Blue.

The selected alliance determines which goal AprilTag is used for targeting.

## Vision and Ballistics

The shooting routines use the `Vision` subsystem to detect the selected goal AprilTag.

The detected:

* **Bearing** is used to calculate the turret angle.
* **Distance** is used to calculate the flywheel target speed.

The calculations are handled by the `Ballistics` subsystem.

## Tuning

Most autonomous timing and power values are defined as constants near the top of each OpMode. These
values can be changed when tuning autonomous behaviour.
