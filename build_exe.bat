@echo off
REM ============================================================================
REM VOR / ASRACS / SAAF Monitoring System - Build EXE Batch Script
REM ============================================================================
REM This batch file builds a standalone Windows executable from CVOR1.py
REM using PyInstaller
REM
REM Prerequisites:
REM   - Python 3.9+ installed and in PATH
REM   - pip install PyInstaller
REM   - All dependencies installed
REM
REM Usage:
REM   1. Open Command Prompt in this directory
REM   2. Run: build_exe.bat
REM   3. Wait for compilation (2-3 minutes)
REM   4. Executable will be in: dist\CVOR1\CVOR1.exe
REM
REM ============================================================================

setlocal enabledelayedexpansion

echo.
echo ============================================================================
echo VOR / ASRACS / SAAF Monitoring System v5.2 - Build Script
echo ============================================================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.9+ from https://www.python.org/
    pause
    exit /b 1
)

echo [*] Python found:
python --version

REM Check if PyInstaller is installed
pip show pyinstaller >nul 2>&1
if errorlevel 1 (
    echo.
    echo [!] PyInstaller not found. Installing...
    pip install PyInstaller
    if errorlevel 1 (
        echo ERROR: Failed to install PyInstaller
        pause
        exit /b 1
    )
)

echo.
echo [*] Checking dependencies...
python -c "import PyQt5; import PyQtChart; import OpenGL; import serial; import numpy; import yaml" >nul 2>&1
if errorlevel 1 (
    echo.
    echo [!] Missing dependencies. Installing...
    pip install PyQt5 PyQtChart PyOpenGL PyOpenGL_accelerate pyserial numpy PyYAML
    if errorlevel 1 (
        echo ERROR: Failed to install dependencies
        pause
        exit /b 1
    )
)

echo [+] All dependencies available

REM Remove old build artifacts
echo.
echo [*] Cleaning previous builds...
if exist dist (
    rmdir /s /q dist >nul 2>&1
)
if exist build (
    rmdir /s /q build >nul 2>&1
)

REM Build the executable
echo.
echo [*] Building executable with PyInstaller...
echo.

pyinstaller ^
    --onefile ^
    --windowed ^
    --name "CVOR1" ^
    --icon "CVOR1.ico" ^
    --specpath "." ^
    --distpath "dist" ^
    --buildpath "build" ^
    --collect-all "PyQt5" ^
    --collect-all "PyQtChart" ^
    --collect-all "OpenGL" ^
    --hidden-import="PyQt5.sip" ^
    --hidden-import="serial" ^
    --hidden-import="OpenGL.GL" ^
    --hidden-import="OpenGL.GLU" ^
    --clean ^
    CVOR1.py

if errorlevel 1 (
    echo.
    echo ERROR: Build failed
    pause
    exit /b 1
)

echo.
echo ============================================================================
echo [+] Build successful!
echo ============================================================================
echo.
echo Executable location: dist\CVOR1\CVOR1.exe
echo.
echo To run the application:
echo   1. Navigate to: dist\CVOR1\
echo   2. Double-click: CVOR1.exe
echo.
echo To create an installer:
echo   1. Install Inno Setup: https://jrsoftware.org/isdl.php
echo   2. Run: build_installer.bat
echo.
pause
exit /b 0
