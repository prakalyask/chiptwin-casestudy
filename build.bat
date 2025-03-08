@echo on
REM Build script for STM32F407 Multi-Zone Temperature Control System

REM Create directories
if not exist build mkdir build
if not exist logs mkdir logs

echo STM32F407 Multi-Zone Temperature Control System
echo ==============================================
echo Building project...

REM Check if arm-none-eabi-gcc is in PATH
where arm-none-eabi-gcc >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo ERROR: arm-none-eabi-gcc not found in PATH
    echo Please install ARM toolchain from:
    echo https://developer.arm.com/downloads/-/gnu-rm
    goto error
)

echo Compiling master controller...

REM Compile the master controller
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 -g ^
    -T stm32f4.ld ^
    -o build/master.elf ^
    startup.c system_stm32f4xx.c main_master.c ^
    -lm -lc --specs=rdimon.specs

if %ERRORLEVEL% neq 0 (
    echo ERROR: Failed to compile master controller
    goto error
)

echo Compiling temperature sensor...

REM Compile the temperature sensor
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 -g ^
    -T stm32f4.ld ^
    -o build/sensor.elf ^
    startup.c system_stm32f4xx.c main_sensor.c ^
    -lm -lc --specs=rdimon.specs

if %ERRORLEVEL% neq 0 (
    echo ERROR: Failed to compile temperature sensor
    goto error
)

echo Build successful!
echo.
echo To run the simulation, use:
echo   run_qemu.bat      - Run with QEMU emulation
echo   run_simulation.bat - Run visual simulation
goto end

:error
echo.
echo Build failed! Please check the errors above.

:end
pause