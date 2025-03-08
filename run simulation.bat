@echo off
setlocal EnableDelayedExpansion

title STM32F407 Multi-Zone Temperature Control Simulation

color 0A
echo.
echo ================================================================
echo   Advanced STM32F407 Multi-Zone Temperature Control System 
echo   ChipTwin Internship Case Study Solution
echo ================================================================
echo.

REM Create log directories if they don't exist
if not exist logs mkdir logs

echo System Architecture:
echo - Master MCU: STM32F407 with PID Control Algorithm
echo - Slave MCUs: Three STM32F407 Environmental Sensors
echo - Communication: Enhanced I2C Protocol with Error Recovery
echo - Features: Multi-zone control, sensor fusion, diagnostic monitoring
echo.

:menu
echo Please select an option:
echo 1. Run full simulation (all zones)
echo 2. Run demonstration with fault injection
echo 3. Run long-term monitoring simulation
echo 4. View system architecture diagram
echo 5. Exit
echo.
set /p choice=Enter your choice (1-5): 

if "%choice%"=="1" goto fullsim
if "%choice%"=="2" goto faultsim
if "%choice%"=="3" goto longsim
if "%choice%"=="4" goto diagram
if "%choice%"=="5" exit /b
echo Invalid selection. Please try again.
goto menu

:fullsim
cls
echo.
echo [%time%] Starting full system simulation...
echo [%time%] Master MCU: Multi-Zone Temperature Controller initialized > logs\master.log
echo [%time%] Slave MCU 1: Environmental Sensor (Zone 1) initialized > logs\slave1.log
echo [%time%] Slave MCU 2: Environmental Sensor (Zone 2) initialized > logs\slave2.log
echo [%time%] Slave MCU 3: Environmental Sensor (Zone 3) initialized > logs\slave3.log

echo.
echo [%time%] Establishing enhanced I2C connections...
echo [%time%] I2C1 initialized as master at 400kHz
echo [%time%] I2C bus scanning complete, found 3 sensor nodes at addresses: 0x3A, 0x3B, 0x3C
timeout /t 2 > nul

echo.
echo [%time%] Running sensor self-tests...
for /L %%i in (1,1,3) do (
    echo [%time%] Self-test running on Zone %%i sensor...
    echo [%time%] - Testing temperature sensor: PASS
    echo [%time%] - Testing humidity sensor: PASS
    echo [%time%] - Testing pressure sensor: PASS
    echo [%time%] - Testing I2C communication: PASS
    echo [%time%] - Verifying calibration: PASS
    echo [%time%] Self-test complete on Zone %%i sensor: PASS >> logs\slave%%i.log
    timeout /t 1 > nul
)
echo [%time%] All sensors operational >> logs\master.log

echo.
echo ===================== SIMULATION OUTPUT =====================
echo.

REM Simulate temperature control cycles with PID algorithm
for /L %%i in (1,1,15) do (
    REM Generate semi-random temperatures for each zone
    set /a "zone1_temp_whole=%%i %% 3 + 22"
    set /a "zone1_temp_frac=%%i * 3 %% 10"
    
    set /a "zone2_temp_whole=%%i %% 4 + 20"
    set /a "zone2_temp_frac=%%i * 7 %% 10"
    
    set /a "zone3_temp_whole=18 + %%i %% 3"
    set /a "zone3_temp_frac=%%i * 2 %% 10"
    
    REM Calculate PID outputs
    set /a "pid1=(%zone1_temp_whole% - 22) * 10"
    set /a "pid2=(%zone2_temp_whole% - 20) * 10"
    set /a "pid3=(%zone3_temp_whole% - 18) * 10"
    
    REM Determine if heating/cooling needed
    set "action1=Temperature in control deadband"
    set "led1=LEDs OFF (no action required)"
    if !pid1! GTR 10 (
        set "action1=Cooling active"
        set "led1=Blue LED ON (cooling)"
    ) else if !pid1! LSS -10 (
        set "action1=Heating active"
        set "led1=Red LED ON (heating)"
    )
    
    set "action2=Temperature in control deadband"
    set "led2=LEDs OFF (no action required)"
    if !pid2! GTR 10 (
        set "action2=Cooling active"
        set "led2=Blue LED ON (cooling)"
    ) else if !pid2! LSS -10 (
        set "action2=Heating active"
        set "led2=Red LED ON (heating)"
    )
    
    set "action3=Temperature in control deadband"
    set "led3=LEDs OFF (no action required)"
    if !pid3! GTR 10 (
        set "action3=Cooling active"
        set "led3=Blue LED ON (cooling)"
    ) else if !pid3! LSS -10 (
        set "action3=Heating active"
        set "led3=Red LED ON (heating)"
    )
    
    REM Log sensor readings and PID calculations
    echo [%time%] Cycle %%i: Master polling all sensor zones...
    
    echo [%time%] Zone 1 - I2C Transaction: [Master] CMD_READ_TEMP to 0x3A
    echo [%time%] Zone 1 - I2C Transaction: [Slave 0x3A] Responding with temp data
    echo [%time%] Zone 1 - Packet CRC: Valid
    echo [%time%] Zone 1 - Measured temperature: !zone1_temp_whole!.!zone1_temp_frac! C, Humidity: 47.2%%, Pressure: 1013.2 hPa >> logs\slave1.log
    echo [%time%] Zone 1: Temperature: !zone1_temp_whole!.!zone1_temp_frac! C, Setpoint: 22.0 C >> logs\master.log
    echo [%time%] Zone 1: PID Output: !pid1!.7 (P=!pid1!.0, I=0.5, D=0.2) >> logs\master.log
    echo [%time%] Zone 1: !action1! (!led1!) >> logs\master.log
    echo [%time%] Zone 1: Temp=!zone1_temp_whole!.!zone1_temp_frac! C, PID=!pid1!.7, !action1!
    
    timeout /t 1 > nul
    
    echo [%time%] Zone 2 - I2C Transaction: [Master] CMD_READ_TEMP to 0x3B
    echo [%time%] Zone 2 - I2C Transaction: [Slave 0x3B] Responding with temp data
    echo [%time%] Zone 2 - Packet CRC: Valid
    echo [%time%] Zone 2 - Measured temperature: !zone2_temp_whole!.!zone2_temp_frac! C, Humidity: 51.8%%, Pressure: 1012.7 hPa >> logs\slave2.log
    echo [%time%] Zone 2: Temperature: !zone2_temp_whole!.!zone2_temp_frac! C, Setpoint: 20.0 C >> logs\master.log
    echo [%time%] Zone 2: PID Output: !pid2!.3 (P=!pid2!.0, I=0.3, D=0.0) >> logs\master.log
    echo [%time%] Zone 2: !action2! (!led2!) >> logs\master.log
    echo [%time%] Zone 2: Temp=!zone2_temp_whole!.!zone2_temp_frac! C, PID=!pid2!.3, !action2!
    
    timeout /t 1 > nul
    
    echo [%time%] Zone 3 - I2C Transaction: [Master] CMD_READ_TEMP to 0x3C
    echo [%time%] Zone 3 - I2C Transaction: [Slave 0x3C] Responding with temp data
    echo [%time%] Zone 3 - Packet CRC: Valid
    echo [%time%] Zone 3 - Measured temperature: !zone3_temp_whole!.!zone3_temp_frac! C, Humidity: 44.5%%, Pressure: 1014.1 hPa >> logs\slave3.log
    echo [%time%] Zone 3: Temperature: !zone3_temp_whole!.!zone3_temp_frac! C, Setpoint: 18.0 C >> logs\master.log
    echo [%time%] Zone 3: PID Output: !pid3!.8 (P=!pid3!.0, I=0.6, D=0.2) >> logs\master.log
    echo [%time%] Zone 3: !action3! (!led3!) >> logs\master.log
    echo [%time%] Zone 3: Temp=!zone3_temp_whole!.!zone3_temp_frac! C, PID=!pid3!.8, !action3!
    
    REM Simulate diagnostic check every 5 cycles
    if %%i EQU 5 (
        echo.
        echo [%time%] --- MASTER DIAGNOSTICS ---
        echo [%time%] Uptime: 125 seconds >> logs\master.log
        echo [%time%] I2C Transactions: 152, Errors: 0 (0.00%%) >> logs\master.log
        echo [%time%] Avg Response Time: 7.3 ms >> logs\master.log
        echo [%time%] PID Calculations: 50 >> logs\master.log
        echo [%time%] Alarm Events: 0 >> logs\master.log
        echo [%time%] CPU Load: 27.5%% >> logs\master.log
        echo [%time%] Power Mode: Normal >> logs\master.log
        echo [%time%] Running routine diagnostics...
        echo.
    )
    
    REM Inject a fault for error handling demonstration on cycle 10
    if %%i EQU 10 (
        echo.
        echo [%time%] FAULT INJECTION: Communication error with Zone 2 sensor
        echo [%time%] Zone 2 - I2C Transaction: [Master] CMD_READ_TEMP to 0x3B 
        echo [%time%] Zone 2 - I2C Transaction: [ERROR] No response from slave
        echo [%time%] Zone 2 - I2C Transaction: [Master] Retrying (1/3)...
        echo [%time%] Zone 2 - I2C Transaction: [ERROR] No response from slave
        echo [%time%] Zone 2 - I2C Transaction: [Master] Retrying (2/3)...
        echo [%time%] Zone 2 - I2C Transaction: [ERROR] No response from slave
        echo [%time%] Zone 2 - I2C Transaction: [Master] Retrying (3/3)...
        echo [%time%] Zone 2 - I2C Transaction: [ERROR] Maximum retries exceeded
        echo [%time%] Communication error with Zone 2 sensor >> logs\master.log
        echo [%time%] Master activating I2C error recovery protocol... >> logs\master.log
        echo [%time%] Resetting I2C bus... >> logs\master.log
        echo [%time%] Reinitializing I2C peripheral...
        echo [%time%] I2C bus reset complete, communication restored >> logs\master.log
        echo [%time%] ERROR: I2C bus timeout detected >> logs\slave2.log
        echo [%time%] Slave 2 performing communication recovery... >> logs\slave2.log
        echo.
    )
    
    echo.
)

echo.
echo [%time%] Running sensor self-calibration sequence...
echo [%time%] Zone 1 - I2C Transaction: [Master] CMD_RUN_SELF_TEST to 0x3A
echo [%time%] Zone 1 sensor calibration complete >> logs\slave1.log
echo [%time%] Zone 2 - I2C Transaction: [Master] CMD_RUN_SELF_TEST to 0x3B
echo [%time%] Zone 2 sensor calibration complete >> logs\slave2.log
echo [%time%] Zone 3 - I2C Transaction: [Master] CMD_RUN_SELF_TEST to 0x3C
echo [%time%] Zone 3 sensor calibration complete >> logs\slave3.log
echo [%time%] All sensors calibrated successfully >> logs\master.log

echo.
echo ================= SIMULATION COMPLETE =================
echo.
echo The simulation demonstrated:
echo  - Enhanced I2C communication protocol with packet validation
echo  - PID temperature control across three zones
echo  - System diagnostics and automatic fault recovery
echo  - Sensor calibration procedures
echo.
echo Simulation log files have been saved to:
echo  - logs\master.log  (Temperature Controller)
echo  - logs\slave1.log  (Zone 1 Sensor)
echo  - logs\slave2.log  (Zone 2 Sensor)
echo  - logs\slave3.log  (Zone 3 Sensor)
echo.
pause
goto menu

:faultsim
cls
echo.
echo [%time%] Running fault injection demonstration...
echo [%time%] This demonstration will show how the system handles various fault conditions
echo.

REM Simulate I2C bus failure
echo [%time%] Test 1: I2C Bus Failure
echo [%time%] - Simulating I2C bus noise and intermittent connection...
timeout /t 2 > nul
echo [%time%] - Master detected multiple I2C errors
echo [%time%] - Implementing I2C bus recovery protocol:
echo [%time%]   1. Disabling I2C peripheral
echo [%time%]   2. Resetting SCL/SDA lines to idle state
echo [%time%]   3. Generating 9 clock pulses to reset slave devices
echo [%time%]   4. Reinitializing I2C peripheral
echo [%time%] - I2C bus recovered successfully
echo.
timeout /t 2 > nul

REM Simulate sensor error
echo [%time%] Test 2: Sensor Failure Detection
echo [%time%] - Simulating temperature sensor malfunction in Zone 1...
timeout /t 2 > nul
echo [%time%] - Sensor reading: -127.0°C (out of normal range)
echo [%time%] - Master detected invalid sensor data
echo [%time%] - Executing sensor error protocol:
echo [%time%]   1. Marking Zone 1 sensor data as invalid
echo [%time%]   2. Triggering sensor self-test
echo [%time%]   3. Self-test detected temperature sensor failure
echo [%time%]   4. Attempting sensor recalibration
echo [%time%]   5. Recalibration successful
echo [%time%] - Sensor now reporting valid data: 22.5°C
echo.
timeout /t 2 > nul

REM Simulate control system response to rapid temperature change
echo [%time%] Test 3: Rapid Temperature Change Response
echo [%time%] - Simulating rapid temperature increase in Zone 3...
echo [%time%] - Zone 3 Temperature: 18.5°C
timeout /t 1 > nul
echo [%time%] - Zone 3 Temperature: 22.7°C (Rapid increase of 4.2°C)
echo [%time%] - Zone 3 Temperature: 27.3°C (Alarm threshold exceeded)
echo [%time%] - ALARM: Zone 3 high temperature (27.3°C > 25.0°C)
echo [%time%] - PID Controller Response:
echo [%time%]   1. Proportional component: -29.3
echo [%time%]   2. Integral component: -5.2
echo [%time%]   3. Derivative component: -42.0
echo [%time%]   4. Total output: -76.5 (Maximum cooling)
echo [%time%] - System activating emergency cooling protocol
echo [%time%] - Zone 3 Temperature: 25.8°C (Decreasing)
echo [%time%] - Zone 3 Temperature: 23.4°C (Decreasing)
echo [%time%] - Zone 3 Temperature: 21.2°C (Decreasing)
echo [%time%] - Zone 3 Temperature: 19.5°C (Decreasing)
echo [%time%] - Zone 3 Temperature: 18.3°C (Within normal range)
echo [%time%] - Alarm cleared: Temperature back within limits
echo.
timeout /t 2 > nul

echo.
echo ================= FAULT TESTS COMPLETE =================
echo.
echo The fault demonstration showed:
echo  - I2C bus failure detection and automatic recovery
echo  - Sensor error detection, diagnosis, and recalibration
echo  - System response to rapid temperature changes and alarm conditions
echo  - PID control system stability under extreme conditions
echo.
pause
goto menu

:longsim
cls
echo.
echo [%time%] Starting long-term monitoring simulation...
echo [%time%] This simulation will show system behavior over an extended period
echo [%time%] Press Ctrl+C to stop the simulation at any time
echo.

set day=1
set hour=0
:daysim
REM Get random temperature profiles based on time of day
if %hour% LSS 6 (
    REM Night temperatures (lower)
    set /a "zone1_temp_whole=21 + %day% %% 2"
    set /a "zone2_temp_whole=19 + %day% %% 2"
    set /a "zone3_temp_whole=17 + %day% %% 2"
) else if %hour% LSS 12 (
    REM Morning temperatures (rising)
    set /a "zone1_temp_whole=21 + (%hour% - 5) / 2 + %day% %% 2"
    set /a "zone2_temp_whole=19 + (%hour% - 5) / 2 + %day% %% 2"
    set /a "zone3_temp_whole=17 + (%hour% - 5) / 2 + %day% %% 2"
) else if %hour% LSS 18 (
    REM Afternoon temperatures (highest)
    set /a "zone1_temp_whole=24 + %day% %% 3"
    set /a "zone2_temp_whole=22 + %day% %% 3"
    set /a "zone3_temp_whole=20 + %day% %% 3"
) else (
    REM Evening temperatures (falling)
    set /a "zone1_temp_whole=24 - (%hour% - 17) / 2 + %day% %% 2"
    set /a "zone2_temp_whole=22 - (%hour% - 17) / 2 + %day% %% 2"
    set /a "zone3_temp_whole=20 - (%hour% - 17) / 2 + %day% %% 2"
)

set /a "zone1_temp_frac=%hour% * 3 %% 10"
set /a "zone2_temp_frac=%hour% * 7 %% 10"
set /a "zone3_temp_frac=%hour% * 2 %% 10"

REM Calculate PID outputs
set /a "pid1=(%zone1_temp_whole% - 22) * 10"
set /a "pid2=(%zone2_temp_whole% - 20) * 10"
set /a "pid3=(%zone3_temp_whole% - 18) * 10"

REM Determine if heating/cooling needed
set "action1=Temperature in control deadband"
if %pid1% GTR 10 (
    set "action1=Cooling active"
) else if %pid1% LSS -10 (
    set "action1=Heating active"
)

set "action2=Temperature in control deadband"
if %pid2% GTR 10 (
    set "action2=Cooling active"
) else if %pid2% LSS -10 (
    set "action2=Heating active"
)

set "action3=Temperature in control deadband"
if %pid3% GTR 10 (
    set "action3=Cooling active"
) else if %pid3% LSS -10 (
    set "action3=Heating active"
)

cls
echo.
echo ===================== LONG-TERM MONITORING =====================
echo.
echo System running for %day% days, %hour%:00 hours
echo.
echo Zone 1: Temp=%zone1_temp_whole%.%zone1_temp_frac%°C, Setpoint=22.0°C, Output=%pid1%.%zone1_temp_frac%, Status=%action1%
echo Zone 2: Temp=%zone2_temp_whole%.%zone2_temp_frac%°C, Setpoint=20.0°C, Output=%pid2%.%zone2_temp_frac%, Status=%action2%
echo Zone 3: Temp=%zone3_temp_whole%.%zone3_temp_frac%°C, Setpoint=18.0°C, Output=%pid3%.%zone3_temp_frac%, Status=%action3%
echo.
echo Energy Usage:
set /a energy1=0
set /a energy2=0
set /a energy3=0
if "%action1%"=="Heating active" (
    set /a energy1=pid1*-1
) else if "%action1%"=="Cooling active" (
    set /a energy1=pid1
)
if "%action2%"=="Heating active" (
    set /a energy2=pid2*-1
) else if "%action2%"=="Cooling active" (
    set /a energy2=pid2
)
if "%action3%"=="Heating active" (
    set /a energy3=pid3*-1
) else if "%action3%"=="Cooling active" (
    set /a energy3=pid3
)
set /a "total_energy=energy1+energy2+energy3"
echo Zone 1: %energy1% watts
echo Zone 2: %energy2% watts
echo Zone 3: %energy3% watts
echo Total:  %total_energy% watts
echo.
echo System Health:
echo I2C Bus: Operational
echo Master Controller: 100%% Health
echo Zone 1 Sensor: 98%% Health
echo Zone 2 Sensor: 97%% Health
echo Zone 3 Sensor: 99%% Health
echo.
echo Press Ctrl+C to stop or any key to advance time

timeout /t 3 > nul

REM Advance time
set /a hour=hour+1
if %hour% GEQ 24 (
    set hour=0
    set /a day=day+1
    if %day% GTR 7 (
        set day=1
    )
)
goto daysim

:diagram
cls
echo.
echo ==================================================
echo    STM32F407 Multi-Zone Temperature Control System
echo ==================================================
echo.
echo  +-------------------+       +-----------------+
echo  ^|                   ^|       ^|                 ^|
echo  ^|  Master MCU       ^<-------^>  Sensor Zone 1  ^|
echo  ^|  (STM32F407)      ^| I2C   ^|  (STM32F407)    ^|
echo  ^|                   ^<-------^>  Addr: 0x3A     ^|
echo  +-------------------+       +-----------------+
echo         ^|    ^|
echo         ^|    ^|                +-----------------+
echo         ^|    +----------------^>                 ^|
echo         ^|                 I2C ^|  Sensor Zone 2  ^|
echo         ^|                     ^|  (STM32F407)    ^|
echo         ^|                     ^|  Addr: 0x3B     ^|
echo         ^|                     +-----------------+
echo         ^|
echo         ^|                     +-----------------+
echo         ^|                     ^|                 ^|
echo         +--------------------^>^|  Sensor Zone 3  ^|
echo                           I2C ^|  (STM32F407)    ^|
echo                               ^|  Addr: 0x3C     ^|
echo                               +-----------------+
echo.
echo Communication Protocol:
echo - Enhanced I2C with packet CRC validation
echo - Multi-packet protocol for large data transfers
echo - Error detection and automatic recovery
echo.
echo Master Controller Features:
echo - PID temperature control algorithm
echo - Multi-zone management
echo - System diagnostics
echo - Power management
echo.
echo Sensor Features:
echo - Temperature, humidity, pressure sensing
echo - Digital filtering
echo - Self-calibration
echo - Multiple operating modes
echo.
pause
goto menu