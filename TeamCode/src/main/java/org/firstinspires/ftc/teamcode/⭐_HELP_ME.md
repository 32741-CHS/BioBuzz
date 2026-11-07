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

### Misc

* OnBot java and the code by Android Studio are completely different, when you use on bot you wipe
  the code pushed by Android Studio, and vise versa
* Sleep is very important
* Flywheel speed not adapting? Probably the tuned PID values are out of whack. Good luck.
* Want to add a dependency? Put it in the `build.dependencies.gradle` file!
* Try coding in java 8, but check with your fellow coders and google.
