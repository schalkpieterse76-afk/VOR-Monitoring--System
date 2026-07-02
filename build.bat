@echo off
REM Build script for VOR/ASRACS/SAAF Airport Monitoring System v5.2
REM Requires: PyInstaller, PyQt5, numpy, pyserial, PyYAML, PyOpenGL

echo.
echo ==========================================
echo VOR/ASRACS/SAAF Monitoring System v5.2
echo Build Script for Windows
echo ==========================================
echo.

echo [1/5] Checking Python installation...
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python not found in PATH
    echo Please install Python 3.8+ and add it to PATH
    pause
    exit /b 1
)

echo [2/5] Installing/updating dependencies...
pip install --upgrade pip setuptools wheel
pip install PyQt5 PyQtChart PyOpenGL PyOpenGL_accelerate pyserial numpy PyYAML PyInstaller

echo [3/5] Removing old build artifacts...
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist
if exist CVOR1.egg-info rmdir /s /q CVOR1.egg-info

echo [4/5] Building executable...
pyinstaller CVOR1.spec --clean

echo [5/5] Build complete!
echo.
echo Executable location: .\dist\CVOR1.exe
echo.
echo To run the application:
echo   Option 1: Double-click .\dist\CVOR1.exe
echo   Option 2: Run from command line: .\dist\CVOR1.exe
echo.
pause
