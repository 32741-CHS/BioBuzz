@echo off
./gradlew :TeamCode:installDebug && adb shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper
@echo on
adb logcat -T 50 *:E
pause
