@echo off
REM Run script for VOR/ASRACS/SAAF Airport Monitoring System v5.2
REM Run from source (requires Python and dependencies installed)

echo.
echo ==========================================
echo VOR/ASRACS/SAAF Monitoring System v5.2
echo Starting Application...
echo ==========================================
echo.

REM Check if Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo.
    echo ERROR: Python not found!
    echo.
    echo Please ensure Python 3.8+ is installed and available in PATH
    echo.
    echo Installation options:
    echo   1. Install from python.org (https://www.python.org/downloads/)
    echo   2. Install from Microsoft Store
    echo   3. Add Python to PATH environment variable
    echo.
    pause
    exit /b 1
)

echo Verifying dependencies...
python -m pip list | findstr "PyQt5 numpy pyserial PyYAML PyOpenGL" >nul
if errorlevel 1 (
    echo.
    echo WARNING: Some dependencies are missing!
    echo.
    echo Installing required packages...
    python -m pip install PyQt5 PyQtChart PyOpenGL PyOpenGL_accelerate pyserial numpy PyYAML
    echo.
)

echo Starting VOR Monitoring System...
echo.
python CVOR1.py

if errorlevel 1 (
    echo.
    echo ERROR: Application failed to start
    echo Check vor_monitor.log for details
    echo.
    pause
)
