#!/run/current-system/sw/bin/bash
# this pushes the TeamCode/src/main/java/ to /sdcard/FIRST/java/src
adb push --sync TeamCode/src/main/java/ /sdcard/FIRST/java/src/
echo "finished pushing"
