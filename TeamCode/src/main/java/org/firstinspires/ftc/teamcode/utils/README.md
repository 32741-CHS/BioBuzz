# Utilities

(This is an AI Generated README.md file and is correct as of 4/10 3:10pm)
This directory contains reusable helper classes that are not specific to one robot mechanism.

## Binder

`Binder.java`

Provides a simple callback system.

Multiple `Runnable` callbacks can be associated with a string key and executed together using
`call()`.

This is used by the gamepad classes to implement button callbacks.

## GamepadButton

`GamepadButton.java`

Tracks the state of a gamepad button and provides methods for detecting:

* A button being pressed
* A button being held
* A button being released
* A button not being held

Callbacks can also be registered for these states.

## GamepadTrigger

`GamepadTrigger.java`

Extends `GamepadButton` to handle analogue triggers.

It stores the trigger's percentage of activation while also allowing it to be treated as a button
when it passes the configured trigger threshold.

## DataInterpolation

`DataInterpolation.java`

Performs linear interpolation between supplied data points.

Values outside the supplied range are clamped to the nearest endpoint.

This is currently used by `Ballistics` to convert measured target distance into a flywheel speed.

## Vector2D

`Vector2D.java`

Represents an immutable two-dimensional vector.

It supports:

* Magnitude
* Angle
* Addition
* Subtraction
* Normalisation
* Construction from magnitude and angle

## Vector3D

`Vector3D.java`

Represents an immutable three-dimensional vector.

It supports:

* Magnitude
* Horizontal angle
* Vertical angle
* Addition
* Subtraction
* Normalisation
* Construction from magnitude and two angles

## Adding Utilities

Utility classes should be general enough to be reused by multiple parts of the project.
Mechanism-specific code should remain in `subsystems/`.
