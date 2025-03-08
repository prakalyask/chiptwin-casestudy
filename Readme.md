# STM32F407 Multi-Zone Temperature Control System

## Project Overview

This project implements a sophisticated multi-zone temperature control system using STM32F407 microcontrollers. The system consists of a master controller that communicates with multiple temperature sensor nodes via I2C protocol. Each zone maintains its target temperature using a PID control algorithm.

## System Architecture

The system follows a master-slave architecture:

- **Master Controller**: A central STM32F407 MCU that implements PID control algorithms for multiple temperature zones
- **Slave Sensors**: Three STM32F407 MCUs functioning as environmental sensors that measure temperature, humidity, and pressure
- **Communication**: Enhanced I2C protocol with error detection and recovery mechanisms
- **Control**: PID (Proportional-Integral-Derivative) algorithm for precise temperature regulation

## Key Components

### Hardware (Simulated)

- **Master Controller MCU**: STM32F407 (ARM Cortex-M4 core, 168MHz)
- **Sensor Node MCUs**: STM32F407 (same specifications)
- **Sensors**:
  - Temperature sensors (all zones)
  - Humidity sensors (standard and premium models)
  - Pressure sensors (premium model only)
- **Actuators**: Heating and cooling systems (simulated)
- **Communication**: I2C bus operating at 400kHz

### Software Components

1. **Master Controller**:

   - Multi-zone PID control implementation
   - I2C master communication
   - System diagnostics and error recovery
   - Power management

2. **Sensor Nodes**:

   - Digital filtering of sensor readings
   - Self-test and calibration functions
   - Multiple operating modes (normal, low-power, high-precision)
   - Alarm management

3. **Communication Protocol**:
   - Enhanced I2C with packet validation (CRC16)
   - Multi-packet transfers for larger data
   - Error detection and recovery mechanisms
   - Command-response architecture

## Implementation Details

### PID Control Algorithm

The system employs a PID control algorithm to maintain target temperatures:

```c
float CalculatePIDOutput(TempZone_TypeDef *zone)
{
    // Calculate error
    float error = zone->pidParams.setpoint - zone->temperature;

    // Proportional term
    float proportional = zone->pidParams.kp * error;

    // Integral term with anti-windup
    zone->integral += zone->pidParams.ki * error;
    if (zone->integral > zone->pidParams.maxOutput) {
        zone->integral = zone->pidParams.maxOutput;
    } else if (zone->integral < zone->pidParams.minOutput) {
        zone->integral = zone->pidParams.minOutput;
    }

    // Derivative term
    float derivative = zone->pidParams.kd * (error - zone->lastError);
    zone->lastError = error;

    // Calculate and clamp output
    float output = proportional + zone->integral + derivative;
    if (output > zone->pidParams.maxOutput) {
        output = zone->pidParams.maxOutput;
    } else if (output < zone->pidParams.minOutput) {
        output = zone->pidParams.minOutput;
    }

    return output;
}
```

### I2C Communication Protocol

The system uses an enhanced I2C protocol with command/response structure:

1. **Command Structure**:

   - Command code (1 byte)
   - Data length (1 byte)
   - Data payload (0-12 bytes)
   - CRC16 (2 bytes)

2. **Command Set**:
   - `CMD_READ_TEMP (0x01)`: Read temperature
   - `CMD_SET_TEMP_PARAMS (0x02)`: Set temperature parameters
   - `CMD_READ_HUMIDITY (0x03)`: Read humidity
   - `CMD_READ_PRESSURE (0x04)`: Read pressure
   - `CMD_SET_OPERATING_MODE (0x05)`: Change sensor operating mode
   - `CMD_RUN_SELF_TEST (0x06)`: Run sensor self-test
   - `CMD_GET_DIAGNOSTICS (0x07)`: Get sensor diagnostic information
   - `CMD_SET_ALARM_THRESHOLD (0x08)`: Set temperature alarm thresholds
   - `CMD_RESET_DEVICE (0xFF)`: Reset sensor device

### Sensor Filtering

The system implements sophisticated filtering for sensor readings:

1. **Outlier Detection and Removal**:

   - Statistical analysis of readings to identify outliers
   - Configurable threshold based on standard deviations

2. **Digital Filtering**:
   - IIR (Infinite Impulse Response) filter
   - Moving average filter
   - Noise reduction based on operating mode

### Error Detection and Recovery

The system includes comprehensive error handling:

1. **Communication Errors**:

   - CRC validation for packet integrity
   - I2C bus reset and recovery procedures
   - Retry mechanisms with exponential backoff

2. **Sensor Errors**:
   - Sensor reading validation
   - Self-test capabilities
   - Automatic recalibration

## Building and Running

### Prerequisites

- ARM GCC Toolchain (`arm-none-eabi-gcc`)
- QEMU with STM32 support (`qemu-system-arm`) or a Windows system for visual simulation

### Build Instructions

1. Install the required tools
2. Clone or download this repository
3. Run the build script:
   ```
   build.bat
   ```

### Running the Simulation

Two options are available:

1. **QEMU Simulation** (hardware emulation):

   ```
   run_qemu.bat
   ```

2. **Visual Simulation** (interactive demonstration):
   ```
   run_simulation.bat
   ```

## System Analysis

### Strengths

1. **Robust Temperature Control**:

   - PID algorithm provides precise temperature regulation
   - Anti-windup protection prevents integral term saturation
   - Configurable parameters per zone

2. **Communication Reliability**:

   - Packet validation with CRC
   - Automatic error recovery
   - Retry mechanisms

3. **Flexibility and Scalability**:

   - Multiple sensor models (basic, standard, premium)
   - Various operating modes for different scenarios
   - Extensible command set

4. **Diagnostics and Monitoring**:
   - Comprehensive system diagnostics
   - Performance monitoring
   - Fault detection and logging

### Challenges and Solutions

1. **Challenge**: Sensor Reading Noise

   - **Solution**: Multiple filtering techniques including outlier removal, IIR filtering, and moving average

2. **Challenge**: I2C Bus Reliability

   - **Solution**: Enhanced protocol with CRC validation and automatic bus recovery

3. **Challenge**: Temperature Control Precision

   - **Solution**: PID algorithm with zone-specific tuning and anti-windup protection

4. **Challenge**: Power Efficiency

   - **Solution**: Multiple operating modes including low-power mode with adjustable sample rates

## Future Improvements

1. **Network Connectivity**:

   - Add wireless communication for remote monitoring and control
   - Implement a web interface for system management

2. **Advanced Control Algorithms**:

   - Implement predictive control based on weather forecasts
   - Add learning capabilities to optimize PID parameters automatically

3. **Data Logging and Analysis**:
   - Add permanent storage for historical data
   - Implement data analysis for energy optimization

## Conclusion

This multi-zone temperature control system demonstrates robust embedded software design with sophisticated control algorithms, reliable communication protocols, and comprehensive error handling. The implementation balances control precision with power efficiency, making it suitable for various applications from home climate control to industrial temperature management.
