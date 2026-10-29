#!/run/current-system/sw/bin/bash
# builds, pls beforehand test in ./tests
./gradlew clean :TeamCode:installDebug
adb logcat -T 50 *:E
