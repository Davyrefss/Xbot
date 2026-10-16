@echo off
setlocal EnableExtensions

REM ============================================================
REM  Flash core_control firmware via ST-Link (SWD)
REM
REM  Usage:
REM     flash.bat              flash build\Debug\core_control.hex
REM     flash.bat check        only resolve tool & firmware paths
REM ============================================================

set "PROJECT_DIR=%~dp0"
set "PROG_BASE=%LOCALAPPDATA%\stm32cube\bundles\programmer"

REM ---- 1) locate STM32_Programmer_CLI.exe (newest version) ----
set "CLI="
for /f "delims=" %%v in ('dir /b /o-n /ad "%PROG_BASE%" 2^>nul') do (
    if not defined CLI if exist "%PROG_BASE%\%%v\bin\STM32_Programmer_CLI.exe" set "CLI=%PROG_BASE%\%%v\bin\STM32_Programmer_CLI.exe"
)

if not defined CLI (
    echo [ERROR] STM32_Programmer_CLI.exe not found under:
    echo         %PROG_BASE%
    pause
    exit /b 1
)

REM ---- 2) locate firmware hex (prefer build\Debug) ----
set "HEX=%PROJECT_DIR%build\Debug\core_control.hex"
if not exist "%HEX%" (
    set "HEX="
    for /r "%PROJECT_DIR%build" %%f in (core_control.hex) do if not defined HEX set "HEX=%%f"
)

if not defined HEX (
    echo [ERROR] core_control.hex not found.
    echo         Build the project first; artifact should be under build\Debug\.
    pause
    exit /b 1
)

echo Tool : %CLI%
echo Hex  : %HEX%

if /i "%~1"=="check" (
    echo [check] resolved paths only; nothing flashed.
    exit /b 0
)

REM ---- 3) connect ST-Link, program + verify + reset ----
echo.
echo Flashing...
"%CLI%" -c port=SWD -w "%HEX%" -v -rst
if errorlevel 1 (
    echo.
    echo [FAILED] Check:
    echo   1. ST-Link wired: SWDIO=PA13  SWCLK=PA14  GND  3.3V
    echo   2. Target board is powered
    echo   3. ST-Link not held by another tool
    pause
    exit /b 1
)

echo.
echo [OK] Flash + verify done. MCU reset and running.
exit /b 0
