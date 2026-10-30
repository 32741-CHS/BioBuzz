@echo off
adb connect 192.168.43.1:5555
adb push --sync TeamCode/src/main/java/ /sdcard/FIRST/java/src/
echo finished pushing
pause
