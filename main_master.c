/**
 * STM32F407 Master Controller - Multi-Zone Temperature Control System
 * QEMU compatible version for olimex-stm32-h405
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

// Command set for I2C communication
typedef enum
{
    CMD_READ_TEMP = 0x01,           // Read temperature
    CMD_SET_TEMP_PARAMS = 0x02,     // Set temperature parameters
    CMD_READ_HUMIDITY = 0x03,       // Read humidity
    CMD_READ_PRESSURE = 0x04,       // Read pressure
    CMD_SET_OPERATING_MODE = 0x05,  // Change sensor operating mode
    CMD_RUN_SELF_TEST = 0x06,       // Run sensor self-test
    CMD_GET_DIAGNOSTICS = 0x07,     // Get sensor diagnostic information
    CMD_SET_ALARM_THRESHOLD = 0x08, // Set temperature alarm thresholds
    CMD_RESET_DEVICE = 0xFF         // Reset sensor device
} Command_TypeDef;

// Operating modes
typedef enum
{
    MODE_NORMAL = 0x00,    // Normal operation
    MODE_LOW_POWER = 0x01, // Low power mode
    MODE_HIGH_PREC = 0x02, // High precision mode
    MODE_BURST = 0x03,     // Burst measurement mode
    MODE_STANDBY = 0x04    // Standby mode (minimal power)
} Operating_Mode_TypeDef;

// PID parameters structure
typedef struct
{
    float setpoint;      // Target temperature
    float kp;            // Proportional gain
    float ki;            // Integral gain
    float kd;            // Derivative gain
    float minOutput;     // Minimum output limit
    float maxOutput;     // Maximum output limit
    uint32_t sampleTime; // PID calculation interval in ms
} PIDParameters_TypeDef;

// Temperature zone structure
typedef struct
{
    uint8_t zoneId;                  // Zone identifier
    float temperature;               // Current temperature
    float humidity;                  // Current humidity
    float pressure;                  // Current pressure
    PIDParameters_TypeDef pidParams; // PID control parameters
    float lastError;                 // Last error for derivative calculation
    float integral;                  // Integral accumulator
    float output;                    // Current control output
    bool heatingActive;              // Heating status
    bool coolingActive;              // Cooling status
    bool alarmActive;                // Alarm status
    float alarmHighThreshold;        // High temperature alarm threshold
    float alarmLowThreshold;         // Low temperature alarm threshold
} TempZone_TypeDef;

// Diagnostic data
typedef struct
{
    uint32_t i2cTransactions;
    uint32_t i2cErrors;
    uint32_t controllerUptime;
    uint32_t pidCalculations;
    uint32_t alarmEvents;
    float cpuLoadPercent;
    float avgResponseTime;
} DiagnosticData_TypeDef;

// System variables
volatile uint32_t systemTicks = 0;
volatile uint8_t currentPowerMode = 0; // 0: Normal, 1: Low-power

// Temperature zones (multiple sensor channels)
#define MAX_ZONES 3
TempZone_TypeDef tempZones[MAX_ZONES];

// Diagnostics
DiagnosticData_TypeDef diagnostics = {0};

// Function prototypes
void InitTempZones(void);
void ProcessSensorData(uint8_t zoneId);
float CalculatePIDOutput(TempZone_TypeDef *zone);
void ApplyControl(TempZone_TypeDef *zone);
void CheckAlarms(TempZone_TypeDef *zone);
void UpdateDiagnostics(void);
void PrintDiagnostics(void);
bool SendCommand(uint8_t slaveAddress, Command_TypeDef command, uint8_t *data, uint8_t dataLength);
bool ReceiveData(uint8_t slaveAddress, uint8_t *data, uint8_t dataLength);

/**
 * Main function
 */
int main(void)
{
    printf("\n\n");
    printf("=====================================================\n");
    printf("  STM32F407 Multi-Zone Temperature Control System\n");
    printf("  Master Controller Starting\n");
    printf("=====================================================\n\n");

    // Initialize temperature zones
    InitTempZones();

    printf("Master Controller: Multi-Zone Temperature Controller initialized\n");
    printf("Controller version: 2.1.0, PID control enabled\n");
    printf("Supported sensor modes: Normal, Low-Power, High-Precision\n\n");

    uint32_t lastDiagnosticsTime = 0;
    uint32_t lastSampleTime = 0;

    // Main control loop
    for (int cycle = 0; cycle < 30; cycle++)
    {
        // Update system ticks (in real system this would be done by SysTick_Handler)
        systemTicks += 1000; // Increment by 1000ms for simulation speed

        printf("\n----- Cycle %d -----\n", cycle + 1);

        // Process each temperature zone
        for (int i = 0; i < MAX_ZONES; i++)
        {
            TempZone_TypeDef *zone = &tempZones[i];

            // Check if it's time to sample this zone
            if ((systemTicks - lastSampleTime) >= zone->pidParams.sampleTime)
            {
                lastSampleTime = systemTicks;

                // Request temperature data from the sensor
                printf("\nPolling Zone %d (I2C addr: 0x%02X)\n", i + 1, 0x3A + i);
                uint8_t cmdData[4] = {0};
                bool success = SendCommand(0x3A + i, CMD_READ_TEMP, cmdData, 0);

                if (success)
                {
                    uint8_t respData[4] = {0};
                    if (ReceiveData(0x3A + i, respData, sizeof(float)))
                    {
                        // Simulate temperature evolution - slight drifting
                        float drift = (float)((rand() % 20) - 10) / 10.0f;
                        zone->temperature += drift;

                        // Constrain to realistic values based on setpoint
                        if (zone->temperature < zone->pidParams.setpoint - 5.0f)
                        {
                            zone->temperature += 1.0f;
                        }
                        else if (zone->temperature > zone->pidParams.setpoint + 5.0f)
                        {
                            zone->temperature -= 1.0f;
                        }

                        // Process the temperature reading
                        ProcessSensorData(i);

                        // Calculate PID output and apply control
                        zone->output = CalculatePIDOutput(zone);
                        ApplyControl(zone);

                        // Check for alarm conditions
                        CheckAlarms(zone);

                        // Update diagnostic statistics
                        UpdateDiagnostics();
                    }
                }
                else
                {
                    // Communication error handling
                    diagnostics.i2cErrors++;
                    printf("Communication error with Zone %d sensor\n", i + 1);
                }
            }
        }

        // Print diagnostics periodically
        if ((systemTicks - lastDiagnosticsTime >= 10000) || (cycle == 29))
        { // Every 10 seconds
            PrintDiagnostics();
            lastDiagnosticsTime = systemTicks;
        }
    }

    printf("\nSimulation complete. System ran for %lu seconds.\n", systemTicks / 1000);
    return 0;
}

/**
 * Initialize the temperature zones with default settings
 */
void InitTempZones(void)
{
    printf("Initializing temperature zones...\n");

    // Zone 1 - Main room
    tempZones[0].zoneId = 1;
    tempZones[0].temperature = 22.0f;
    tempZones[0].humidity = 45.0f;
    tempZones[0].pressure = 1013.25f;
    tempZones[0].pidParams.setpoint = 22.0f;    // 22°C target
    tempZones[0].pidParams.kp = 3.0f;           // Proportional gain
    tempZones[0].pidParams.ki = 0.1f;           // Integral gain
    tempZones[0].pidParams.kd = 1.0f;           // Derivative gain
    tempZones[0].pidParams.minOutput = -100.0f; // Minimum output (full cooling)
    tempZones[0].pidParams.maxOutput = 100.0f;  // Maximum output (full heating)
    tempZones[0].pidParams.sampleTime = 1000;   // 1 second sample time
    tempZones[0].lastError = 0.0f;
    tempZones[0].integral = 0.0f;
    tempZones[0].output = 0.0f;
    tempZones[0].heatingActive = false;
    tempZones[0].coolingActive = false;
    tempZones[0].alarmActive = false;
    tempZones[0].alarmHighThreshold = 30.0f; // High temperature alarm at 30°C
    tempZones[0].alarmLowThreshold = 15.0f;  // Low temperature alarm at 15°C

    // Zone 2 - Secondary room
    tempZones[1].zoneId = 2;
    tempZones[1].temperature = 20.0f;
    tempZones[1].humidity = 50.0f;
    tempZones[1].pressure = 1012.0f;
    tempZones[1].pidParams.setpoint = 20.0f; // 20°C target
    tempZones[1].pidParams.kp = 2.5f;
    tempZones[1].pidParams.ki = 0.05f;
    tempZones[1].pidParams.kd = 0.8f;
    tempZones[1].pidParams.minOutput = -100.0f;
    tempZones[1].pidParams.maxOutput = 100.0f;
    tempZones[1].pidParams.sampleTime = 1500; // 1.5 second sample time
    tempZones[1].lastError = 0.0f;
    tempZones[1].integral = 0.0f;
    tempZones[1].output = 0.0f;
    tempZones[1].heatingActive = false;
    tempZones[1].coolingActive = false;
    tempZones[1].alarmActive = false;
    tempZones[1].alarmHighThreshold = 28.0f;
    tempZones[1].alarmLowThreshold = 17.0f;

    // Zone 3 - Critical equipment
    tempZones[2].zoneId = 3;
    tempZones[2].temperature = 18.0f;
    tempZones[2].humidity = 40.0f;
    tempZones[2].pressure = 1014.0f;
    tempZones[2].pidParams.setpoint = 18.0f; // 18°C target (cooler)
    tempZones[2].pidParams.kp = 4.0f;        // More aggressive control
    tempZones[2].pidParams.ki = 0.2f;
    tempZones[2].pidParams.kd = 1.2f;
    tempZones[2].pidParams.minOutput = -100.0f;
    tempZones[2].pidParams.maxOutput = 100.0f;
    tempZones[2].pidParams.sampleTime = 800; // 0.8 second sample time (faster)
    tempZones[2].lastError = 0.0f;
    tempZones[2].integral = 0.0f;
    tempZones[2].output = 0.0f;
    tempZones[2].heatingActive = false;
    tempZones[2].coolingActive = false;
    tempZones[2].alarmActive = false;
    tempZones[2].alarmHighThreshold = 25.0f; // Tighter temperature range
    tempZones[2].alarmLowThreshold = 15.0f;

    printf("Zone 1: Setpoint=%.1f°C, Kp=%.1f, Ki=%.1f, Kd=%.1f\n",
           tempZones[0].pidParams.setpoint,
           tempZones[0].pidParams.kp,
           tempZones[0].pidParams.ki,
           tempZones[0].pidParams.kd);
    printf("Zone 2: Setpoint=%.1f°C, Kp=%.1f, Ki=%.1f, Kd=%.1f\n",
           tempZones[1].pidParams.setpoint,
           tempZones[1].pidParams.kp,
           tempZones[1].pidParams.ki,
           tempZones[1].pidParams.kd);
    printf("Zone 3: Setpoint=%.1f°C, Kp=%.1f, Ki=%.1f, Kd=%.1f\n",
           tempZones[2].pidParams.setpoint,
           tempZones[2].pidParams.kp,
           tempZones[2].pidParams.ki,
           tempZones[2].pidParams.kd);
}

/**
 * Process sensor data for a temperature zone
 */
void ProcessSensorData(uint8_t zoneId)
{
    TempZone_TypeDef *zone = &tempZones[zoneId];

    // Print temperature reading
    printf("Zone %d: Temperature: %.2f°C, Setpoint: %.2f°C\n",
           zone->zoneId, zone->temperature, zone->pidParams.setpoint);
}

/**
 * Calculate PID control output
 */
float CalculatePIDOutput(TempZone_TypeDef *zone)
{
    // Calculate error
    float error = zone->pidParams.setpoint - zone->temperature;

    // Proportional term
    float proportional = zone->pidParams.kp * error;

    // Integral term with anti-windup
    zone->integral += zone->pidParams.ki * error;
    if (zone->integral > zone->pidParams.maxOutput)
    {
        zone->integral = zone->pidParams.maxOutput;
    }
    else if (zone->integral < zone->pidParams.minOutput)
    {
        zone->integral = zone->pidParams.minOutput;
    }

    // Derivative term
    float derivative = zone->pidParams.kd * (error - zone->lastError);
    zone->lastError = error;

    // Calculate output
    float output = proportional + zone->integral + derivative;

    // Clamp output to limits
    if (output > zone->pidParams.maxOutput)
    {
        output = zone->pidParams.maxOutput;
    }
    else if (output < zone->pidParams.minOutput)
    {
        output = zone->pidParams.minOutput;
    }

    // Update diagnostics
    diagnostics.pidCalculations++;

    printf("Zone %d: PID Output: %.2f (P=%.2f, I=%.2f, D=%.2f)\n",
           zone->zoneId, output, proportional, zone->integral, derivative);

    return output;
}

/**
 * Apply control output to actuators
 */
void ApplyControl(TempZone_TypeDef *zone)
{
    // Check if heating or cooling needed
    if (zone->output > 5.0f)
    { // Positive output = heating
        // Activate heating, deactivate cooling
        zone->heatingActive = true;
        zone->coolingActive = false;
        printf("Zone %d: Heating active, output=%.2f%%\n", zone->zoneId, zone->output);

        // Simulate temperature increasing due to heating
        zone->temperature += (zone->output / 500.0f); // Slow increase
    }
    else if (zone->output < -5.0f)
    { // Negative output = cooling
        // Activate cooling, deactivate heating
        zone->heatingActive = false;
        zone->coolingActive = true;
        printf("Zone %d: Cooling active, output=%.2f%%\n", zone->zoneId, -zone->output);

        // Simulate temperature decreasing due to cooling
        zone->temperature -= (-zone->output / 500.0f); // Slow decrease
    }
    else
    { // Within deadband, no action needed
        // Deactivate both
        zone->heatingActive = false;
        zone->coolingActive = false;
        printf("Zone %d: Temperature in control deadband\n", zone->zoneId);
    }
}

/**
 * Check for alarm conditions
 */
void CheckAlarms(TempZone_TypeDef *zone)
{
    bool prevAlarmState = zone->alarmActive;

    // Check if temperature is outside of alarm thresholds
    if (zone->temperature > zone->alarmHighThreshold)
    {
        // High temperature alarm
        zone->alarmActive = true;
        printf("ALARM: Zone %d high temperature (%.2f°C > %.2f°C)\n",
               zone->zoneId, zone->temperature, zone->alarmHighThreshold);
    }
    else if (zone->temperature < zone->alarmLowThreshold)
    {
        // Low temperature alarm
        zone->alarmActive = true;
        printf("ALARM: Zone %d low temperature (%.2f°C < %.2f°C)\n",
               zone->zoneId, zone->temperature, zone->alarmLowThreshold);
    }
    else
    {
        // Temperature within normal range
        zone->alarmActive = false;
    }

    // Count new alarm events
    if (!prevAlarmState && zone->alarmActive)
    {
        diagnostics.alarmEvents++;
    }
}

/**
 * Update diagnostic information
 */
void UpdateDiagnostics(void)
{
    // Update controller uptime
    diagnostics.controllerUptime = systemTicks / 1000; // Convert to seconds

    // Simulate CPU load
    diagnostics.cpuLoadPercent = 25.0f + (rand() % 100) / 10.0f; // 25-35% load

    // Update response time metric
    diagnostics.avgResponseTime = (diagnostics.avgResponseTime * 0.9f) +
                                  (rand() % 5 + 5) * 0.1f; // 5-10ms response time

    // Increment I2C transaction counter
    diagnostics.i2cTransactions++;
}

/**
 * Print diagnostic information
 */
void PrintDiagnostics(void)
{
    printf("\n--- SYSTEM DIAGNOSTICS ---\n");
    printf("Uptime: %lu seconds\n", diagnostics.controllerUptime);
    printf("I2C Transactions: %lu, Errors: %lu (%.2f%%)\n",
           diagnostics.i2cTransactions,
           diagnostics.i2cErrors,
           diagnostics.i2cTransactions > 0 ? (float)diagnostics.i2cErrors / diagnostics.i2cTransactions * 100.0f : 0.0f);
    printf("Avg Response Time: %.2f ms\n", diagnostics.avgResponseTime);
    printf("PID Calculations: %lu\n", diagnostics.pidCalculations);
    printf("Alarm Events: %lu\n", diagnostics.alarmEvents);
    printf("CPU Load: %.2f%%\n", diagnostics.cpuLoadPercent);
    printf("Power Mode: %s\n", currentPowerMode == 0 ? "Normal" : "Low Power");

    // Zone status summary
    printf("\nZone Status Summary:\n");
    for (int i = 0; i < MAX_ZONES; i++)
    {
        TempZone_TypeDef *zone = &tempZones[i];
        printf("Zone %d: Temp=%.2f°C, Setpoint=%.2f°C, Output=%.2f, Status=%s\n",
               zone->zoneId,
               zone->temperature,
               zone->pidParams.setpoint,
               zone->output,
               zone->heatingActive ? "Heating" : (zone->coolingActive ? "Cooling" : "Idle"));
    }
    printf("-------------------------\n");
}

/**
 * Send command to a slave device (QEMU-compatible simulation)
 */
bool SendCommand(uint8_t slaveAddress, Command_TypeDef command, uint8_t *data, uint8_t dataLength)
{
    // For simulation, just print what would happen
    printf("I2C TX: [Master -> Slave 0x%02X] Command 0x%02X\n", slaveAddress, command);

    // Simulate success - in reality would return status of I2C transfer
    bool success = (rand() % 100) < 98; // 98% success rate
    if (!success)
    {
        printf("I2C ERROR: No acknowledgment from slave 0x%02X\n", slaveAddress);
    }
    return success;
}

/**
 * Receive data from a slave device (QEMU-compatible simulation)
 */
bool ReceiveData(uint8_t slaveAddress, uint8_t *data, uint8_t dataLength)
{
    // For simulation, create a realistic response
    printf("I2C RX: [Slave 0x%02X -> Master] Data received\n", slaveAddress);

    // Simulate a temperature reading
    float temp = 0.0f;

    // Generate different temperatures for different zones
    switch (slaveAddress)
    {
    case 0x3A: // Zone 1
        temp = tempZones[0].temperature;
        break;
    case 0x3B: // Zone 2
        temp = tempZones[1].temperature;
        break;
    case 0x3C: // Zone 3
        temp = tempZones[2].temperature;
        break;
    default:
        temp = 25.0f;
        break;
    }

    // Copy the temperature to the data buffer
    memcpy(data, &temp, sizeof(float));

    // Simulate success
    bool success = (rand() % 100) < 95; // 95% success rate
    if (!success)
    {
        printf("I2C ERROR: Receive error from slave 0x%02X\n", slaveAddress);
    }
    return success;
}