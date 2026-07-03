@echo off
REM ============================================================================
REM VOR / ASRACS / SAAF Monitoring System - Build Installer Batch Script
REM ============================================================================
REM This batch file builds a Windows installer using Inno Setup
REM
REM Prerequisites:
REM   - Inno Setup 6.0+ installed from: https://jrsoftware.org/isdl.php
REM   - CVOR1.exe already built (run build_exe.bat first)
REM
REM Usage:
REM   1. Open Command Prompt in this directory
REM   2. Run: build_installer.bat
REM   3. Wait for compilation (~30 seconds)
REM   4. Installer will be in: Output\CVOR1_Setup_v5.2.exe
REM
REM ============================================================================

setlocal enabledelayedexpansion

echo.
echo ============================================================================
echo VOR / ASRACS / SAAF Monitoring System v5.2 - Installer Build
echo ============================================================================
echo.

REM Check if executable exists
if not exist "dist\CVOR1\CVOR1.exe" (
    echo ERROR: CVOR1.exe not found
    echo Please run: build_exe.bat first
    pause
    exit /b 1
)

echo [+] CVOR1.exe found

REM Check if Inno Setup is installed
if exist "C:\Program Files (x86)\Inno Setup 6\ISCC.exe" (
    set "INNO_PATH=C:\Program Files (x86)\Inno Setup 6\ISCC.exe"
) else if exist "C:\Program Files\Inno Setup 6\ISCC.exe" (
    set "INNO_PATH=C:\Program Files\Inno Setup 6\ISCC.exe"
) else if exist "C:\Program Files (x86)\Inno Setup 5\ISCC.exe" (
    set "INNO_PATH=C:\Program Files (x86)\Inno Setup 5\ISCC.exe"
) else (
    echo ERROR: Inno Setup not found
    echo Please install from: https://jrsoftware.org/isdl.php
    pause
    exit /b 1
)

echo [+] Inno Setup found: !INNO_PATH!

REM Create output directory
if not exist "Output" mkdir Output

echo.
echo [*] Building installer...
echo.

"!INNO_PATH!" "CVOR1.iss" /Q

if errorlevel 1 (
    echo.
    echo ERROR: Installer build failed
    pause
    exit /b 1
)

echo.
echo ============================================================================
echo [+] Installer build successful!
echo ============================================================================
echo.
echo Installer location: Output\CVOR1_Setup_v5.2.exe
echo.
echo Redistribution:
echo   - Share Output\CVOR1_Setup_v5.2.exe with end users
echo   - Users run the installer to install CVOR1
echo.
pause
exit /b 0
