@echo off
./gradlew clean :TeamCode:installDebug
adb logcat -T 50 *:E
pause
