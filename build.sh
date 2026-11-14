#!/run/current-system/sw/bin/bash
# builds, pls beforehand test in ./tests
./gradlew :TeamCode:installDebug && adb shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper
adb logcat *:E
