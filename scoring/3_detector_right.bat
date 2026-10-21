@echo off
cd /d "%~dp0detector"
set SERVER_IP=
set CAM=
set /p SERVER_IP=Server laptop IP (Enter = this laptop):
if "%SERVER_IP%"=="" set SERVER_IP=127.0.0.1
set /p CAM=Camera number (Enter = 0):
if "%CAM%"=="" set CAM=0
python goal_detector.py --goal right --server http://%SERVER_IP%:8000 --camera %CAM%
pause
