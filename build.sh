#!/run/current-system/sw/bin/bash
<<<<<<< Updated upstream
# builds, pls beforehand test in ./tests
./gradlew clean installDebug
adb logcat *:E
=======
./gradlew clean :TeamCode:installDebug
adb logcat -T 50 *:E
>>>>>>> Stashed changes
