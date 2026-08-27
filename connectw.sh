#!/run/current-system/sw/bin/bash
# wrapper for connecting to the robot
# must be connected to the network beforehand
adb connect 192.168.43.1:5555
echo "connecting to robot filesystem"
