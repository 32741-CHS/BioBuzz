# HELP

For help.

### Contents?

* Websites
* Our Code Terminology
* Downloading Code into the robot (using Android Studio)
* When It's Next Season

### Websites

Note: the IP adress 192.168.43.1 is the robot A local website is hosted on the robot, so that mean
you need to be connected to the robot Wi-Fi to accsss it. Note that when connected to the robot
Wi-Fi you will never have internet connection.

* https://pedropathing.com/docs/pathing/installation --- All you need to know about Pedro Pathing. I
  mean EVERYTHING (see the tabs).
* http://192.168.43.1:10158 --- Pedro Pathing Tuning Local Website
* https://visualizer.pedropathing.com/ --- Pedro Pathing nice opcode creator using a fancy robot
  simulation
* http://192.168.43.1:8080 --- The FTC "Control and Manager" website for OnBot Java and other stuff.
  Explore it.
* http://192.168.43.1:8001 --- For Panels, a better logging software that we use with GRAPHS!

### Code Terminology

##### Only our Terms

* Subsystem --- An indecent system designed to handle one very specific task e.g make sure that the
  flywheel runs at the correct speed.
* FlyWheel --- The very fast spinning wheel that shoots the ball into space.
* Shooter --- The system that runs the flywheel
* Intake --- The contraption that sucks in balls into the robot.

##### FTC Terms

* Panels --- The graphing/logging software. Think like a supercharged print statement in python with
  graphs
* Pedro Pathing & Ivy (or just Pedro for short)---
* OpMode --- Aka Operation Mode, it being different scripts that we can choose in the Driver Hub
* Auto/Autonomus Opcode --- An Opcode designed to run without any human input
* Tele Op --- With human input OpMode
* Driver Hub --- The fancy (android) phone like thing
* Controller Hub --- The thing that 1) Has all the ports for all the robot hardware 2) Hosts the
  robot Wi-Fi 3) where the code goes to and runs
* Expansion Hub --- Think of power strip/bar, it just gives us more ports to plug in more hardware
* Android Studio --- An IDE (Intigrated Develoupment Enviorment) aka a place to write, compile,
  debug and run code

##### General Java/Code Terms

* Gradle sync --- Gradle downloads dependencies (external libraries from the internet) for java
  files that are dictated by the .gradle files that act as a sort of check list for what the code
  needs to run.
* Compile code --- Convert code from human-readable text to binary machine code (JVM byte code for
  java)
* Assemble/Build/Compile Code --- Android Studio downloads dependencies from the internet, compile
  code into computer language and something... I don't know the difference between these terms
  myself.

### Putting Code onto the robot

A step by step, by step guide.

##### 1. Make sure the code builds

1. Press the hammer icon in Android Studio while connected to the internet
2. Wait for it to finish doing its thing... It can take up to a minute on the first run.
3. Check for a success message like "BUILD COMPLETE" found in the bottom window or as a notification
   within Android Studio.

##### 2. Power on all the hardware

1. Plug in battery to Control hub
2. Power on the Driver hub
3. Make sure that your computer is on (I mean, it's hardware too...)

##### 3. In your terminal run `./connectw.bat` (Windows) or `./connectw.sh ` (Unix/Linux/macOS)

Runs a few commands for you, using ADB (Android Debug Bridge) to connect to the robot.

##### 4. Press the green play button and WAIT until loading finishes with success message

It builds and deploys your code onto the robot.

##### 5. Close the app (FULLY) and open it again. Wait a bit and if you hear a sound it good.

### Next Season Set Up & Updates (By Eric)

* Go to the GitHub for FTC and find the relevant season you are competing (I'm in BioBuzz)
* Merge their code with yours (Too long explanation, just google it.), noting that the `TeamCode`
  folder holds all your code, and the `FtcRobotController` holds the stuff for the next season.
* Assemble* the code
* Download the Rev Hardware Client 2, connect to the Wi-Fi with the Driver Hub, Control and
  Expansion Hub all on.
* Find

### Ivy

##### Ivy — FTC BioBuzz feature reference

Ivy is Pedro Pathing's command-based framework for FTC. It provides a way to organise robot
behaviour into reusable commands, manage subsystem requirements, and execute multiple operations
sequentially or concurrently.

For BioBuzz, the important distinction is:

- Ivy: Command scheduling, subsystem organisation and command composition.
- Pedro Pathing: Robot localisation, path following and movement control.
- FTC SDK: Hardware access, OpModes, motors, servos and sensors.

###### 1. Core command lifecycle

Every Ivy command follows four lifecycle methods.

| Name        | Class / Method / Inputs / Outputs                     | Description                                                                                          |
|-------------|-------------------------------------------------------|------------------------------------------------------------------------------------------------------|
| `start()`   | `Command` method; no inputs; `void` output            | Called once when the command begins. Initialise targets, reset counters or configure mechanisms.     |
| `execute()` | `Command` method; no inputs; `void` output            | Called every scheduler cycle while the command is active.                                            |
| `done()`    | `Command` method; no inputs; `boolean` output         | Returns `true` when the command should finish.                                                       |
| `end()`     | `Command` method; `EndCondition` input; `void` output | Called when the command ends, is interrupted or suspended. Perform cleanup, such as stopping motors. |

## 2. Creating commands

| Name                        | Class / Method / Inputs / Outputs                  | Description                                                                           |
|-----------------------------|----------------------------------------------------|---------------------------------------------------------------------------------------|
| `Command.build()`           | Static method; no inputs; returns `CommandBuilder` | Creates a command using the builder API. Recommended for most BioBuzz commands.       |
| `.setStart()`               | `Runnable`; no inputs; `void`                      | Defines the command's start behaviour.                                                |
| `.setExecute()`             | `Runnable`; no inputs; `void`                      | Defines what happens every loop.                                                      |
| `.setDone()`                | `BooleanSupplier`; returns `boolean`               | Defines the completion condition.                                                     |
| `.setEnd()`                 | `Consumer<EndCondition>`; receives end reason      | Defines cleanup behaviour.                                                            |
| `.requiring()`              | `Object...` inputs; returns builder                | Declares which resources/subsystems the command uses.                                 |
| `.setPriority()`            | `int` input; returns builder                       | Sets command priority. Default is `0`.                                                |
| `.setInterruptedBehavior()` | `InterruptedBehavior` input                        | Determines what happens when a higher-priority command interrupts it.                 |
| `.setBlockedBehavior()`     | `BlockedBehavior` input                            | Determines what happens when a command cannot start due to a higher-priority command. |
| `.setConflictBehavior()`    | `ConflictBehavior` input                           | Determines what happens when commands have equal-priority conflicts.                  |

Recommendation: Use `Command.build()` for most subsystem operations. You generally do not need to
create a separate Java class for every command.

## 3. Command compositions

These allow multiple commands to be combined into larger operations.

| Name           | Class / Method / Inputs / Outputs                                | Description                                                                |
|----------------|------------------------------------------------------------------|----------------------------------------------------------------------------|
| `sequential()` | Static method; `Command...`; returns `Command`                   | Executes commands one after another.                                       |
| `.then()`      | Instance method; `Command...`; returns `Command`                 | Appends commands sequentially.                                             |
| `parallel()`   | Static method; `Command...`; returns `Command`                   | Executes commands concurrently. Finishes when all finish.                  |
| `.with()`      | Instance method; `Command...`; returns `Command`                 | Adds parallel commands.                                                    |
| `race()`       | Static method; `Command...`; returns `Command`                   | Runs commands concurrently until the first finishes. Cancels the others.   |
| `.raceWith()`  | Instance method; `Command...`; returns `Command`                 | Adds commands to a race composition.                                       |
| `deadline()`   | Static method; `Command deadline, Command...`; returns `Command` | Runs commands concurrently until the designated deadline command finishes. |
| `repeat()`     | Static method; `Command, int/IntSupplier`; returns `Command`     | Repeats a command a specified number of times.                             |
| `loop()`       | Static method; `Command`; returns `Command`                      | Repeats a command indefinitely.                                            |

## 4. Utility commands

These are prebuilt commands for common operations.

| Name            | Class / Method / Inputs / Outputs                     | Description                                           |
|-----------------|-------------------------------------------------------|-------------------------------------------------------|
| `instant()`     | Static method; `Runnable`; returns `Command`          | Executes an action once and immediately finishes.     |
| `infinite()`    | Static method; `Runnable`; returns `Command`          | Executes continuously until cancelled or interrupted. |
| `waitMs()`      | Static method; `long` milliseconds; returns `Command` | Waits for a specified duration.                       |
| `waitUntil()`   | Static method; `BooleanSupplier`; returns `Command`   | Waits until a condition becomes true.                 |
| `onInterrupt()` | Static method; `Runnable`; returns `Command`          | Performs a callback when interrupted.                 |

## 5. Control flow commands

| Name            | Class / Method / Inputs / Outputs                                           | Description                                                                                     |
|-----------------|-----------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------|
| `conditional()` | Static method; `BooleanSupplier, Command, Command`; returns `Command`       | Selects between two commands using an if/else condition.                                        |
| `branch()`      | Static method; `LinkedHashMap<BooleanSupplier, Command>`; returns `Command` | Selects a command based on the first matching condition.                                        |
| `match()`       | Static method; `Supplier<T>, EnumMap<T, Command>`; returns `Command`        | Selects a command based on an enum value, similar to a switch statement.                        |
| `lazy()`        | Static method; `Supplier<Command>`; returns `Command`                       | Creates the command only when it starts. Useful when its target depends on current robot state. |

## 6. Command decorators

Decorators modify existing commands without changing the original command.

| Name        | Class / Method / Inputs / Outputs                     | Description                                                    |
|-------------|-------------------------------------------------------|----------------------------------------------------------------|
| `.until()`  | Instance method; `BooleanSupplier`; returns `Command` | Runs a command until a condition becomes true.                 |
| `.unless()` | Instance method; `BooleanSupplier`; returns `Command` | Skips the command if a condition is true when it starts.       |
| `.proxy()`  | Instance method; no inputs; returns `Command`         | Schedules the command independently of its parent composition. |

## 7. Requirements and priorities

| Name                  | Class / Method / Inputs / Outputs       | Description                                                                                            |
|-----------------------|-----------------------------------------|--------------------------------------------------------------------------------------------------------|
| `requirements()`      | `Command` method; returns `Set<Object>` | Returns resources claimed by the command. Override when implementing the `Command` interface directly. |
| `priority()`          | `Command` method; returns `int`         | Returns command priority. Override in class-based commands.                                            |
| `InterruptedBehavior` | Enum                                    | `END` terminates the interrupted command; `SUSPEND` pauses it so it can resume later.                  |
| `BlockedBehavior`     | Enum                                    | `CANCEL` cancels a blocked command; `QUEUE` queues it.                                                 |
| `ConflictBehavior`    | Enum                                    | `CANCEL`, `QUEUE` or `OVERRIDE` determine equal-priority conflict handling.                            |

Requirements are optional, but I recommend using them for BioBuzz mechanisms.

For example, an intake command should claim the intake subsystem so another command cannot
unintentionally control it simultaneously.

## 8. Scheduler

The scheduler manages command execution.

| Name                      | Class / Method / Inputs / Outputs | Description                                                        |
|---------------------------|-----------------------------------|--------------------------------------------------------------------|
| `Scheduler`               | Static class                      | Central command manager. Does not need to be instantiated.         |
| `Scheduler.schedule()`    | `Command...`; `void`              | Schedules commands.                                                |
| `Scheduler.execute()`     | No inputs; `void`                 | Executes one scheduler cycle. Must be called once per OpMode loop. |
| `Scheduler.isRunning()`   | `Command`; `boolean`              | Checks whether a command is actively executing.                    |
| `Scheduler.isScheduled()` | `Command`; `boolean`              | Checks whether a command is running, queued or suspended.          |
| `Scheduler.cancel()`      | `Command`; `void`                 | Cancels a command.                                                 |
| `Scheduler.reset()`       | No inputs; `void`                 | Clears all running, queued and suspended commands.                 |

Important: Ivy uses cooperative multitasking. Commands execute on the same thread, one scheduler
cycle at a time. It does not require you to create threads.

## 9. Pedro Pathing integration

These are Ivy commands specifically designed to work with Pedro Pathing 3.

| Name       | Class / Method / Inputs / Outputs   | Description                                                       |
|------------|-------------------------------------|-------------------------------------------------------------------|
| `follow()` | `Follower, Path`; returns `Command` | Follows a Pedro path. Finishes when the follower stops following. |
| `hold()`   | `Follower`; returns `Command`       | Holds the robot's current position.                               |
| `hold()`   | `Follower, Pose`; returns `Command` | Moves to and holds a specified position.                          |

Pedro commands can be combined with normal Ivy commands.

For example, you can follow a path while running an intake command in parallel.

## 10. Class API — what needs overriding?

If you implement `Command` directly instead of using `Command.build()`, you must implement the
following methods.

| Method                  | Must override? | Purpose                        |
|-------------------------|----------------|--------------------------------|
| `start()`               | Yes            | Initialise command.            |
| `execute()`             | Yes            | Run command logic.             |
| `done()`                | Yes            | Determine completion.          |
| `end()`                 | Yes            | Cleanup.                       |
| `requirements()`        | Yes            | Declare resource requirements. |
| `priority()`            | Yes            | Set priority.                  |
| `interruptedBehavior()` | Yes            | Define interruption handling.  |
| `blockedBehavior()`     | Yes            | Define blocked handling.       |
| `conflictBehavior()`    | Yes            | Define conflict handling.      |

The builder API supplies defaults for methods you don't configure.

### Misc

* OnBot java and the code by Android Studio are completely different, when you use on bot you wipe
  the code pushed by Android Studio, and vise versa
* Sleep is very important
* Flywheel speed not adapting? Probably the tuned PID values are out of whack. Good luck.
* Want to add a dependency? Put it in the `build.dependencies.gradle` file!
* Try coding in java 8, but check with your fellow coders and google.
