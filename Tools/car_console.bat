@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"

rem Check a working Python 3, not just a Windows Store PATH alias.
py -3 -c "import sys; sys.exit(sys.version_info < (3, 8))" >nul 2>nul
if not errorlevel 1 goto use_py
python -c "import sys; sys.exit(sys.version_info < (3, 8))" >nul 2>nul
if not errorlevel 1 goto use_python
echo [ERROR] Python 3.8+ was not found. Install Python 3 and enable PATH.
pause
exit /b 1

:use_py
py -3 "%~dp0car_console_server.py" %*
goto finished

:use_python
python "%~dp0car_console_server.py" %*

:finished
set "car_exit=%errorlevel%"
if "%car_exit%"=="0" exit /b 0
echo [ERROR] Console stopped with code %car_exit%. See the message above.
pause
exit /b %car_exit%
