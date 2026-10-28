#!/run/current-system/sw/bin/bash
# builds, pls beforehand test in ./tests
./gradlew clean installDebug
adb logcat *:E
