# TeleOp

(This is an AI Generated README.md file and is correct as of 4/10 3:10pm)

## Main TeleOp

`MainTeleOp` is the primary driver-controlled OpMode.

Two gamepads are used:

* **Driver gamepad** — drivetrain and field-driving controls.
* **Operator gamepad** — intake, shooter and turret controls.

## Driver Controls

| Control       | Function                             |
|---------------|--------------------------------------|
| Left stick    | Drive and strafe                     |
| Right stick X | Rotate                               |
| Left bumper   | Increase drivetrain speed while held |
| Y             | Toggle field-relative driving        |
| X during init | Select Red alliance                  |
| A during init | Select Blue alliance                 |

The drivetrain normally operates at reduced speed and can be switched to a higher speed using the
left bumper.

## Operator Controls

| Control           | Function                       |
|-------------------|--------------------------------|
| D-pad Up          | Increase flywheel target speed |
| D-pad Down        | Decrease flywheel target speed |
| X                 | Toggle flywheel                |
| Right trigger     | Feed balls into the shooter    |
| Left bumper       | Run intake                     |
| A                 | Reverse intake / feeder        |
| B                 | Reset turret encoder           |
| Y                 | Reset IMU                      |
| Left bumper press | Toggle flywheel lookup control |

## Turret

The turret normally tracks the selected goal's AprilTag automatically.

Moving the operator's right stick horizontally temporarily switches the turret into manual control.

If the goal AprilTag has not been detected for a configured period, the turret returns toward its
home position.

## Shooter

The flywheel uses velocity control. When automatic lookup control is enabled, the detected AprilTag
distance is used to select an appropriate flywheel speed.

The feeder only operates when the flywheel is sufficiently close to its target speed.

## Telemetry

Runtime information is sent through Panels telemetry and graphs, including:

* Flywheel speed and error
* Feeder power
* Intake power
* Turret angle and error
* Drivetrain speed
* Field-relative mode
* AprilTag distance and bearing
