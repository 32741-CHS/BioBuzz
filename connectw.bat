@echo off

netsh wlan connect name="32741-RC"

adb disconnect

@REM The line below is to make sure all works fine
echo.
echo [Code] Restarting ADB server (Prevents random bugs)
adb kill-server && adb start-server

pause
@REM Connect to the device (Note echo. is new line for a spacer)
echo.
echo [Code] connecting to 192.168.43.1:5555 (Robot)...
adb connect 192.168.43.1:5555

@REM Just to make sure...
echo.
echo [Code] Current ADB Devices:
adb devices
echo.
echo [Code] Finished!!! Press the Play button on the top right and wait for it to finish loading.