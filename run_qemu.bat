@echo on
REM Run script for STM32F407 Multi-Zone Temperature Control System in QEMU

echo STM32F407 Multi-Zone Temperature Control System
echo ==============================================
echo Starting QEMU simulation...

REM Check if QEMU is in PATH
where qemu-system-arm >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo ERROR: qemu-system-arm not found in PATH
    echo Please install QEMU from:
    echo https://www.qemu.org/download/#windows
    goto end
)

REM Check if build files exist
if not exist build\master.elf (
    echo ERROR: build\master.elf not found
    echo Please run build.bat first
    goto end
)

if not exist build\sensor.elf (
    echo ERROR: build\sensor.elf not found
    echo Please run build.bat first
    goto end
)

echo Starting master controller...
echo Use Ctrl+A, X to exit QEMU

REM Start master controller in QEMU
start "Master Controller" cmd /k "qemu-system-arm -machine olimex-stm32-h405 ^
    -kernel build/master.elf ^
    -nographic"

REM Wait briefly
timeout /t 2 /nobreak

echo Starting sensor...

REM Start a temperature sensor in QEMU
start "Temperature Sensor" cmd /k "qemu-system-arm -machine olimex-stm32-h405 ^
    -kernel build/sensor.elf ^
    -nographic"

echo Simulation started successfully!

:end
echo Press any key to exit this window...
pause > nul