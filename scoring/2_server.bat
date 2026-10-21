@echo off
cd /d "%~dp0server"
python server.py --port 8000
pause
