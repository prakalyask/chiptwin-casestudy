/**
 * STM32F407 Slave Sensor - Multi-Zone Temperature Control System
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

// Self-test result
typedef enum
{
    SELF_TEST_PASS = 0x00,
    SELF_TEST_FAIL_TEMP = 0x01,
    SELF_TEST_FAIL_HUM = 0x02,
    SELF_TEST_FAIL_PRES = 0x04,
    SELF_TEST_FAIL_COM = 0x08,
    SELF_TEST_FAIL_CAL = 0x10
} SelfTest_Result_TypeDef;

// Sensor model
typedef enum
{
    SENSOR_MODEL_BASIC = 0,    // Basic temperature only
    SENSOR_MODEL_STANDARD = 1, // Temperature and humidity
    SENSOR_MODEL_PREMIUM = 2   // Temperature, humidity, pressure
} Sensor_Model_TypeDef;

// Sensor constants
#define TEMP_SENSOR_MIN -40.0f
#define TEMP_SENSOR_MAX 125.0f
#define HUMIDITY_MIN 0.0f
#define HUMIDITY_MAX 100.0f
#define PRESSURE_MIN 300.0f
#define PRESSURE_MAX 1100.0f

// Sensor data structure
typedef struct
{
    float rawTemp;
    float filteredTemp;
    float rawHumidity;
    float filteredHumidity;
    float rawPressure;
    float filteredPressure;
    float batteryVoltage;
    uint32_t readingCount;
    uint32_t lastReadTime;
    bool tempValid;
    bool humidityValid;
    bool pressureValid;
} SensorData_TypeDef;

// Calibration data structure
typedef struct
{
    float tempOffset;
    float tempScale;
    float humOffset;
    float humScale;
    float presOffset;
    float presScale;
    uint32_t lastCalibrationTime;
} Calibration_TypeDef;

// Diagnostic data structure
typedef struct
{
    uint32_t powerOnTime;
    uint32_t commandCount;
    uint32_t errorCount;
    uint32_t selfTestCount;
    uint32_t lastErrorCode;
    SelfTest_Result_TypeDef lastSelfTestResult;
    float minTempRecorded;
    float maxTempRecorded;
    float avgTempRecorded;
    float sensorNoiseLevel;
    uint8_t sensorHealth;
} Diagnostic_TypeDef;

// Filter settings
typedef struct
{
    float alpha;            // IIR filter coefficient (0-1)
    uint8_t samples;        // Number of samples for moving average
    bool outlierRemoval;    // Whether to remove outliers
    float outlierThreshold; // Threshold for outlier detection (std deviations)
} FilterSettings_TypeDef;

// System state
typedef struct
{
    uint8_t address;
    Operating_Mode_TypeDef mode;
    Sensor_Model_TypeDef model;
    SensorData_TypeDef data;
    Calibration_TypeDef calibration;
    Diagnostic_TypeDef diagnostics;
    FilterSettings_TypeDef filter;
    float alarmHighThreshold;
    float alarmLowThreshold;
    bool alarmEnabled;
    bool alarmTriggered;
    uint8_t sampleRateDivider; // Divides the base sample rate
    uint8_t noiseLevelPercent; // Simulated noise level (0-100%)
    float driftRate;           // Temperature drift in °C per hour
} SystemState_TypeDef;

// System variables
volatile uint32_t systemTicks = 0;
SystemState_TypeDef sysState = {0};

// Moving average filter buffer
#define MAX_FILTER_SAMPLES 10
float tempHistory[MAX_FILTER_SAMPLES] = {0};
float humHistory[MAX_FILTER_SAMPLES] = {0};
float presHistory[MAX_FILTER_SAMPLES] = {0};
uint8_t historyIndex = 0;

// Function prototypes
void InitSensor(void);
void ProcessCommand(Command_TypeDef command, uint8_t *data, uint8_t dataLength);
float ReadTemperature(void);
float ReadHumidity(void);
float ReadPressure(void);
float ReadBatteryVoltage(void);
void UpdateSensorReadings(void);
float ApplyFilter(float newValue, float *history, uint8_t sampleCount);
void RunSelfTest(void);
void CheckAlarms(void);
void UpdateDiagnostics(void);
float CalculateMovingAverage(float *history, uint8_t count);
void UpdateMovingAverage(float newValue, float *history, uint8_t sampleIndex);

// Choose a sensor address based on instance number
uint8_t getSensorAddress(void)
{
    // Demonstration: Just return one of the three addresses
    return 0x3A + (rand() % 3);
}

/**
 * Main function
 */
int main(void)
{
    // Seed the random generator
    srand(time(NULL));

    printf("\n\n");
    printf("=====================================================\n");
    printf("  STM32F407 Multi-Zone Temperature Control System\n");
    printf("  Temperature Sensor Starting\n");
    printf("=====================================================\n\n");

    // Initialize the sensor
    InitSensor();

    printf("Sensor Model: %s, Operating Mode: %s\n",
           sysState.model == SENSOR_MODEL_PREMIUM ? "Premium" : sysState.model == SENSOR_MODEL_STANDARD ? "Standard"
                                                                                                        : "Basic",
           sysState.mode == MODE_NORMAL ? "Normal" : sysState.mode == MODE_LOW_POWER ? "Low Power"
                                                 : sysState.mode == MODE_HIGH_PREC   ? "High Precision"
                                                 : sysState.mode == MODE_BURST       ? "Burst"
                                                                                     : "Standby");
    printf("I2C Address: 0x%02X\n\n", sysState.address);

    uint32_t lastSampleTime = 0;
    uint32_t lastDiagTime = 0;

    // Run initial self-test
    RunSelfTest();

    // Main loop - run for a fixed number of cycles for demonstration
    for (int cycle = 0; cycle < 30; cycle++)
    {
        // Update system ticks (in real system this would be done by SysTick_Handler)
        systemTicks += 1000; // Increment by 1000ms for simulation speed

        printf("\n----- Cycle %d -----\n", cycle + 1);

        // Simulate receiving a command from master
        if (rand() % 100 < 33)
        { // 33% chance to receive a command per cycle
            Command_TypeDef commands[] = {CMD_READ_TEMP, CMD_READ_HUMIDITY, CMD_READ_PRESSURE,
                                          CMD_RUN_SELF_TEST, CMD_GET_DIAGNOSTICS};
            Command_TypeDef command = commands[rand() % 5];
            uint8_t data[4] = {0};
            printf("I2C RX: [Master -> Slave 0x%02X] Command received: 0x%02X\n",
                   sysState.address, command);
            ProcessCommand(command, data, 0);
        }

        // Check if it's time to update sensor readings based on mode
        uint32_t sampleInterval = 1000; // Base interval: 1 second

        // Adjust interval based on operating mode
        switch (sysState.mode)
        {
        case MODE_LOW_POWER:
            sampleInterval = 5000; // 5 seconds in low power mode
            break;
        case MODE_HIGH_PREC:
            sampleInterval = 500; // 0.5 seconds in high precision mode
            break;
        case MODE_BURST:
            sampleInterval = 100; // 0.1 seconds in burst mode
            break;
        case MODE_STANDBY:
            sampleInterval = 30000; // 30 seconds in standby mode
            break;
        default:
            break;
        }

        // Apply the sample rate divider
        sampleInterval *= sysState.sampleRateDivider;

        // Update sensor readings periodically
        if (systemTicks - lastSampleTime >= sampleInterval)
        {
            UpdateSensorReadings();
            CheckAlarms();
            lastSampleTime = systemTicks;
        }

        // Update diagnostics every 10 seconds
        if (systemTicks - lastDiagTime >= 10000)
        {
            UpdateDiagnostics();
            lastDiagTime = systemTicks;
        }

        // Simulate I2C interrupt happening
        if (rand() % 100 < 20)
        { // 20% chance of I2C interrupt
            printf("I2C Interrupt: Address match detected\n");
            printf("I2C Interrupt: Process transaction with master\n");
        }
    }

    printf("\nSimulation complete. Sensor ran for %lu seconds.\n", systemTicks / 1000);
    return 0;
}

/**
 * Initialize sensor state and calibration
 */
void InitSensor(void)
{
    printf("Initializing environmental sensor...\n");

    // Set initial system state
    sysState.address = getSensorAddress();
    sysState.mode = MODE_NORMAL;
    sysState.model = SENSOR_MODEL_PREMIUM;
    sysState.sampleRateDivider = 1;
    sysState.noiseLevelPercent = 5; // 5% noise level
    sysState.driftRate = 0.05f;     // 0.05°C drift per hour

    // Initialize alarm settings
    sysState.alarmHighThreshold = 40.0f;
    sysState.alarmLowThreshold = 0.0f;
    sysState.alarmEnabled = true;
    sysState.alarmTriggered = false;

    // Initialize filter settings
    sysState.filter.alpha = 0.8f;            // IIR filter coefficient (0-1)
    sysState.filter.samples = 5;             // 5 samples for moving average
    sysState.filter.outlierRemoval = true;   // Remove outliers
    sysState.filter.outlierThreshold = 3.0f; // 3 standard deviations

    // Initialize calibration data
    sysState.calibration.tempOffset = 0.0f;
    sysState.calibration.tempScale = 1.0f;
    sysState.calibration.humOffset = -2.0f; // Slight humidity offset
    sysState.calibration.humScale = 0.98f;  // Slight humidity scaling
    sysState.calibration.presOffset = 0.0f;
    sysState.calibration.presScale = 1.0f;
    sysState.calibration.lastCalibrationTime = 0;

    // Initialize sensor data with realistic starting values based on address/zone
    float baseTemp;
    switch (sysState.address)
    {
    case 0x3A: // Zone 1
        baseTemp = 22.5f;
        break;
    case 0x3B: // Zone 2
        baseTemp = 20.3f;
        break;
    case 0x3C: // Zone 3
        baseTemp = 18.7f;
        break;
    default:
        baseTemp = 25.0f;
        break;
    }

    sysState.data.rawTemp = baseTemp;
    sysState.data.filteredTemp = baseTemp;
    sysState.data.rawHumidity = 50.0f;
    sysState.data.filteredHumidity = 50.0f;
    sysState.data.rawPressure = 1013.25f;
    sysState.data.filteredPressure = 1013.25f;
    sysState.data.batteryVoltage = 3.3f;
    sysState.data.readingCount = 0;
    sysState.data.tempValid = true;
    sysState.data.humidityValid = true;
    sysState.data.pressureValid = true;

    // Initialize diagnostics
    sysState.diagnostics.powerOnTime = 0;
    sysState.diagnostics.commandCount = 0;
    sysState.diagnostics.errorCount = 0;
    sysState.diagnostics.selfTestCount = 0;
    sysState.diagnostics.lastErrorCode = 0;
    sysState.diagnostics.lastSelfTestResult = SELF_TEST_PASS;
    sysState.diagnostics.minTempRecorded = baseTemp;
    sysState.diagnostics.maxTempRecorded = baseTemp;
    sysState.diagnostics.avgTempRecorded = baseTemp;
    sysState.diagnostics.sensorNoiseLevel = 0.0f;
    sysState.diagnostics.sensorHealth = 100;

    // Initialize history arrays
    for (int i = 0; i < MAX_FILTER_SAMPLES; i++)
    {
        tempHistory[i] = baseTemp;
        humHistory[i] = 50.0f;
        presHistory[i] = 1013.25f;
    }

    printf("Sensor 0x%02X initialized with following parameters:\n", sysState.address);
    printf("Initial temperature: %.1f°C\n", baseTemp);
    printf("Calibration: Temp offset=%.2f, scale=%.2f\n",
           sysState.calibration.tempOffset, sysState.calibration.tempScale);
    printf("Filter settings: alpha=%.2f, samples=%d, outlier removal=%s\n",
           sysState.filter.alpha, sysState.filter.samples,
           sysState.filter.outlierRemoval ? "enabled" : "disabled");
}

/**
 * Process received command
 */
void ProcessCommand(Command_TypeDef command, uint8_t *data, uint8_t dataLength)
{
    // Track command count
    sysState.diagnostics.commandCount++;

    // Process command based on type
    switch (command)
    {
    case CMD_READ_TEMP:
        printf("Processing CMD_READ_TEMP command\n");
        printf("I2C TX: [Slave 0x%02X -> Master] Temperature: %.2f°C\n",
               sysState.address, sysState.data.filteredTemp);
        break;

    case CMD_READ_HUMIDITY:
        printf("Processing CMD_READ_HUMIDITY command\n");
        if (sysState.model == SENSOR_MODEL_BASIC)
        {
            printf("Error: Humidity sensor not available on basic model\n");
            sysState.diagnostics.errorCount++;
            sysState.diagnostics.lastErrorCode = 0x01;
            printf("I2C TX: [Slave 0x%02X -> Master] ERROR 0x01\n", sysState.address);
        }
        else
        {
            printf("I2C TX: [Slave 0x%02X -> Master] Humidity: %.2f%%\n",
                   sysState.address, sysState.data.filteredHumidity);
        }
        break;

    case CMD_READ_PRESSURE:
        printf("Processing CMD_READ_PRESSURE command\n");
        if (sysState.model != SENSOR_MODEL_PREMIUM)
        {
            printf("Error: Pressure sensor not available on this model\n");
            sysState.diagnostics.errorCount++;
            sysState.diagnostics.lastErrorCode = 0x02;
            printf("I2C TX: [Slave 0x%02X -> Master] ERROR 0x02\n", sysState.address);
        }
        else
        {
            printf("I2C TX: [Slave 0x%02X -> Master] Pressure: %.2f hPa\n",
                   sysState.address, sysState.data.filteredPressure);
        }
        break;

    case CMD_SET_OPERATING_MODE:
    {
        printf("Processing CMD_SET_OPERATING_MODE command\n");
        Operating_Mode_TypeDef newMode = MODE_NORMAL;
        if (dataLength > 0)
        {
            newMode = (Operating_Mode_TypeDef)data[0];
        }
        else
        {
            newMode = (Operating_Mode_TypeDef)(rand() % 5); // Random mode for simulation
        }

        // Set operating mode
        const char *modeNames[] = {"Normal", "Low Power", "High Precision", "Burst", "Standby"};
        printf("Operating mode changing from %s to %s\n",
               modeNames[sysState.mode], modeNames[newMode]);
        sysState.mode = newMode;
        printf("I2C TX: [Slave 0x%02X -> Master] Mode set to %d\n",
               sysState.address, sysState.mode);
    }
    break;

    case CMD_RUN_SELF_TEST:
        printf("Processing CMD_RUN_SELF_TEST command\n");
        RunSelfTest();
        printf("I2C TX: [Slave 0x%02X -> Master] Self-test result: 0x%02X\n",
               sysState.address, sysState.diagnostics.lastSelfTestResult);
        break;

    case CMD_GET_DIAGNOSTICS:
        printf("Processing CMD_GET_DIAGNOSTICS command\n");
        printf("I2C TX: [Slave 0x%02X -> Master] Sending diagnostic data\n", sysState.address);
        printf("        Health: %d%%, Errors: %lu, Battery: %.2fV\n",
               sysState.diagnostics.sensorHealth,
               sysState.diagnostics.errorCount,
               sysState.data.batteryVoltage);
        break;

    case CMD_SET_ALARM_THRESHOLD:
    {
        printf("Processing CMD_SET_ALARM_THRESHOLD command\n");
        // For simulation, set some random thresholds
        float highThreshold = 30.0f + (rand() % 10);
        float lowThreshold = 10.0f + (rand() % 10);

        sysState.alarmHighThreshold = highThreshold;
        sysState.alarmLowThreshold = lowThreshold;
        sysState.alarmEnabled = true;

        printf("Alarm thresholds set: Low=%.2f°C, High=%.2f°C\n",
               sysState.alarmLowThreshold, sysState.alarmHighThreshold);
        printf("I2C TX: [Slave 0x%02X -> Master] Thresholds set\n", sysState.address);
    }
    break;

    case CMD_RESET_DEVICE:
        printf("Processing CMD_RESET_DEVICE command\n");
        printf("Resetting device...\n");

        // Simulate a reset by re-initializing sensor (but preserve the address)
        uint8_t savedAddress = sysState.address;
        InitSensor();
        sysState.address = savedAddress;

        printf("I2C TX: [Slave 0x%02X -> Master] Device reset complete\n", sysState.address);
        break;

    default:
        printf("Error: Unknown command: 0x%02X\n", command);
        sysState.diagnostics.errorCount++;
        sysState.diagnostics.lastErrorCode = 0xFF;
        printf("I2C TX: [Slave 0x%02X -> Master] ERROR 0xFF (Unknown command)\n",
               sysState.address);
        break;
    }
}

/**
 * Read temperature from sensor
 */
float ReadTemperature(void)
{
    // Base temperature with slow drift based on address/zone
    float baseTemp;

    switch (sysState.address)
    {
    case 0x3A: // Zone 1
        baseTemp = 22.5f;
        break;
    case 0x3B: // Zone 2
        baseTemp = 20.3f;
        break;
    case 0x3C: // Zone 3
        baseTemp = 18.7f;
        break;
    default:
        baseTemp = 25.0f;
        break;
    }

    // Add drift based on system time
    float driftFactor = sinf((float)systemTicks / 10000.0f) * sysState.driftRate;

    // Add random noise based on noise level setting
    float noise = (((float)rand() / RAND_MAX) * 2.0f - 1.0f) *
                  (sysState.noiseLevelPercent / 100.0f);

    // Adjust noise level based on operating mode
    if (sysState.mode == MODE_HIGH_PREC)
    {
        noise *= 0.5f; // Reduced noise in high precision mode
    }
    else if (sysState.mode == MODE_LOW_POWER)
    {
        noise *= 2.0f; // Increased noise in low power mode
    }

    // Apply calibration
    float temp = (baseTemp + driftFactor + noise) * sysState.calibration.tempScale +
                 sysState.calibration.tempOffset;

    // Clamp to sensor range
    if (temp < TEMP_SENSOR_MIN)
        temp = TEMP_SENSOR_MIN;
    if (temp > TEMP_SENSOR_MAX)
        temp = TEMP_SENSOR_MAX;

    return temp;
}

/**
 * Read humidity from sensor
 */
float ReadHumidity(void)
{
    // Base humidity (varies inversely with temperature)
    float tempFactor = (sysState.data.rawTemp - 5.0f) / 35.0f; // 0-1 scale (5-40°C)
    float baseHumidity = 80.0f - tempFactor * 30.0f;           // 50-80% range, lower when hotter

    // Add random noise
    float noise = (((float)rand() / RAND_MAX) * 2.0f - 1.0f) * 5.0f; // ±5% noise

    // Apply calibration
    float humidity = (baseHumidity + noise) * sysState.calibration.humScale +
                     sysState.calibration.humOffset;

    // Clamp to sensor range
    if (humidity < HUMIDITY_MIN)
        humidity = HUMIDITY_MIN;
    if (humidity > HUMIDITY_MAX)
        humidity = HUMIDITY_MAX;

    return humidity;
}

/**
 * Read pressure from sensor
 */
float ReadPressure(void)
{
    // Base pressure (tends to revert to 1013.25 hPa)
    static float basePressure = 1013.25f;

    // Slow random walk for pressure
    float walk = (((float)rand() / RAND_MAX) * 2.0f - 1.0f) * 0.1f; // ±0.1 hPa per reading
    basePressure += walk;

    // Tend to revert to normal pressure
    basePressure = basePressure * 0.9995f + 1013.25f * 0.0005f;

    // Add noise
    float noise = (((float)rand() / RAND_MAX) * 2.0f - 1.0f) * 0.5f; // ±0.5 hPa noise

    // Apply calibration
    float pressure = (basePressure + noise) * sysState.calibration.presScale +
                     sysState.calibration.presOffset;

    // Clamp to sensor range
    if (pressure < PRESSURE_MIN)
        pressure = PRESSURE_MIN;
    if (pressure > PRESSURE_MAX)
        pressure = PRESSURE_MAX;

    return pressure;
}

/**
 * Read battery voltage
 */
float ReadBatteryVoltage(void)
{
    // For simulation, generate realistic battery voltage
    static float voltage = 3.3f;

    // Slow discharge rate (depends on operating mode)
    float dischargeRate = 0.00001f; // Base discharge rate

    // Adjust discharge rate based on operating mode
    if (sysState.mode == MODE_HIGH_PREC)
    {
        dischargeRate *= 2.0f; // Faster discharge in high precision mode
    }
    else if (sysState.mode == MODE_LOW_POWER)
    {
        dischargeRate *= 0.5f; // Slower discharge in low power mode
    }
    else if (sysState.mode == MODE_STANDBY)
    {
        dischargeRate *= 0.1f; // Very slow discharge in standby mode
    }

    // Apply discharge
    voltage -= dischargeRate;

    // Add small noise
    float noise = (((float)rand() / RAND_MAX) * 2.0f - 1.0f) * 0.02f; // ±0.02V noise

    // Ensure voltage stays within realistic limits (2.0V to 3.3V)
    if (voltage < 2.0f)
        voltage = 2.0f;
    if (voltage > 3.3f)
        voltage = 3.3f;

    return voltage + noise;
}

/**
 * Update all sensor readings
 */
void UpdateSensorReadings(void)
{
    // Read raw sensor values
    sysState.data.rawTemp = ReadTemperature();

    // Only read humidity and pressure for supported models
    if (sysState.model >= SENSOR_MODEL_STANDARD)
    {
        sysState.data.rawHumidity = ReadHumidity();
    }

    if (sysState.model == SENSOR_MODEL_PREMIUM)
    {
        sysState.data.rawPressure = ReadPressure();
    }

    // Read battery voltage
    sysState.data.batteryVoltage = ReadBatteryVoltage();

    // Apply filtering
    sysState.data.filteredTemp = ApplyFilter(
        sysState.data.rawTemp, tempHistory, sysState.filter.samples);

    if (sysState.model >= SENSOR_MODEL_STANDARD)
    {
        sysState.data.filteredHumidity = ApplyFilter(
            sysState.data.rawHumidity, humHistory, sysState.filter.samples);
    }

    if (sysState.model == SENSOR_MODEL_PREMIUM)
    {
        sysState.data.filteredPressure = ApplyFilter(
            sysState.data.rawPressure, presHistory, sysState.filter.samples);
    }

    // Update reading count and time
    sysState.data.readingCount++;
    sysState.data.lastReadTime = systemTicks;

    // Update diagnostics
    if (sysState.data.filteredTemp < sysState.diagnostics.minTempRecorded)
    {
        sysState.diagnostics.minTempRecorded = sysState.data.filteredTemp;
    }

    if (sysState.data.filteredTemp > sysState.diagnostics.maxTempRecorded)
    {
        sysState.diagnostics.maxTempRecorded = sysState.data.filteredTemp;
    }

    // Update average temperature using exponential moving average
    sysState.diagnostics.avgTempRecorded = sysState.diagnostics.avgTempRecorded * 0.99f +
                                           sysState.data.filteredTemp * 0.01f;

    // Calculate sensor noise level as standard deviation between raw and filtered
    float diff = sysState.data.rawTemp - sysState.data.filteredTemp;
    sysState.diagnostics.sensorNoiseLevel =
        sysState.diagnostics.sensorNoiseLevel * 0.95f + fabsf(diff) * 0.05f;

    printf("Sensor 0x%02X readings updated:\n", sysState.address);
    printf("Temperature: %.2f°C", sysState.data.filteredTemp);

    if (sysState.model >= SENSOR_MODEL_STANDARD)
    {
        printf(", Humidity: %.2f%%", sysState.data.filteredHumidity);
    }

    if (sysState.model == SENSOR_MODEL_PREMIUM)
    {
        printf(", Pressure: %.2f hPa", sysState.data.filteredPressure);
    }

    printf(", Battery: %.2fV\n", sysState.data.batteryVoltage);
}

/**
 * Apply filtering to sensor readings
 */
float ApplyFilter(float newValue, float *history, uint8_t sampleCount)
{
    // First check for outliers if enabled
    if (sysState.filter.outlierRemoval)
    {
        // Calculate mean of history
        float mean = CalculateMovingAverage(history, sampleCount);

        // Calculate standard deviation
        float sumSquaredDiff = 0.0f;
        for (uint8_t i = 0; i < sampleCount; i++)
        {
            float diff = history[i] - mean;
            sumSquaredDiff += diff * diff;
        }
        float stdDev = sqrtf(sumSquaredDiff / sampleCount);

        // Check if new value is an outlier
        float deviation = fabsf(newValue - mean);
        if (deviation > sysState.filter.outlierThreshold * stdDev)
        {
            printf("Outlier detected: %.2f (mean=%.2f, stdDev=%.2f)\n",
                   newValue, mean, stdDev);
            // Replace outlier with mean
            newValue = mean;
        }
    }

    // Apply IIR filter (exponential moving average)
    float filteredValue = sysState.filter.alpha * newValue +
                          (1.0f - sysState.filter.alpha) * history[historyIndex];

    // Update history buffer
    UpdateMovingAverage(newValue, history, historyIndex);
    historyIndex = (historyIndex + 1) % sampleCount;

    return filteredValue;
}

/**
 * Update moving average buffer
 */
void UpdateMovingAverage(float newValue, float *history, uint8_t sampleIndex)
{
    history[sampleIndex] = newValue;
}

/**
 * Calculate moving average from history buffer
 */
float CalculateMovingAverage(float *history, uint8_t count)
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++)
    {
        sum += history[i];
    }
    return sum / count;
}

/**
 * Run sensor self-test
 */
void RunSelfTest(void)
{
    printf("Running sensor self-test...\n");
    sysState.diagnostics.selfTestCount++;

    // Simulate self-test process
    // In a real system, this would test hardware components

    // For simulation, determine result based on sensor health and random chance
    SelfTest_Result_TypeDef result = SELF_TEST_PASS;

    // 10% chance of failure during testing for simulation realism
    if (rand() % 100 < 10)
    {
        // Pick a random failure type
        uint8_t failureType = 1 << (rand() % 5);
        result = (SelfTest_Result_TypeDef)failureType;
    }

    // Store result
    sysState.diagnostics.lastSelfTestResult = result;

    // Update sensor health based on self-test
    if (result == SELF_TEST_PASS)
    {
        // Improve health with successful tests (max 100)
        if (sysState.diagnostics.sensorHealth < 100)
        {
            sysState.diagnostics.sensorHealth += 1;
        }
    }
    else
    {
        // Reduce health with failed tests
        uint8_t reduction = 0;
        if (result & SELF_TEST_FAIL_TEMP)
            reduction += 5;
        if (result & SELF_TEST_FAIL_HUM)
            reduction += 3;
        if (result & SELF_TEST_FAIL_PRES)
            reduction += 3;
        if (result & SELF_TEST_FAIL_COM)
            reduction += 2;
        if (result & SELF_TEST_FAIL_CAL)
            reduction += 5;

        if (reduction > sysState.diagnostics.sensorHealth)
        {
            sysState.diagnostics.sensorHealth = 0;
        }
        else
        {
            sysState.diagnostics.sensorHealth -= reduction;
        }
    }

    printf("Self-test completed with result: 0x%02X, Sensor health: %d%%\n",
           result, sysState.diagnostics.sensorHealth);

    // Print specific test results
    const char *testNames[] = {"Temperature", "Humidity", "Pressure", "Communication", "Calibration"};
    for (int i = 0; i < 5; i++)
    {
        bool testPassed = !(result & (1 << i));
        printf("- %s test: %s\n", testNames[i], testPassed ? "PASS" : "FAIL");
    }
}

/**
 * Check for alarm conditions
 */
void CheckAlarms(void)
{
    if (!sysState.alarmEnabled)
    {
        return;
    }

    // Check temperature against thresholds
    if (sysState.data.filteredTemp > sysState.alarmHighThreshold)
    {
        if (!sysState.alarmTriggered)
        {
            printf("ALARM: High temperature detected (%.2f°C > %.2f°C)\n",
                   sysState.data.filteredTemp, sysState.alarmHighThreshold);
            sysState.alarmTriggered = true;
        }
    }
    else if (sysState.data.filteredTemp < sysState.alarmLowThreshold)
    {
        if (!sysState.alarmTriggered)
        {
            printf("ALARM: Low temperature detected (%.2f°C < %.2f°C)\n",
                   sysState.data.filteredTemp, sysState.alarmLowThreshold);
            sysState.alarmTriggered = true;
        }
    }
    else if (sysState.alarmTriggered)
    {
        // Temperature back within limits, clear alarm
        printf("Alarm cleared: Temperature back within limits\n");
        sysState.alarmTriggered = false;
    }
}

/**
 * Update diagnostic information
 */
void UpdateDiagnostics(void)
{
    // Update uptime
    sysState.diagnostics.powerOnTime = systemTicks / 1000; // Convert to seconds

    // For simulation, print diagnostics summary
    printf("\n--- SENSOR 0x%02X DIAGNOSTICS ---\n", sysState.address);
    printf("Uptime: %lu seconds\n", sysState.diagnostics.powerOnTime);
    printf("Operating mode: %d\n", sysState.mode);
    printf("Battery voltage: %.2fV\n", sysState.data.batteryVoltage);
    printf("Temperature range: %.2f°C to %.2f°C (avg: %.2f°C)\n",
           sysState.diagnostics.minTempRecorded,
           sysState.diagnostics.maxTempRecorded,
           sysState.diagnostics.avgTempRecorded);
    printf("Commands processed: %lu\n", sysState.diagnostics.commandCount);
    printf("Errors: %lu\n", sysState.diagnostics.errorCount);
    printf("Self-tests: %lu (last result: 0x%02X)\n",
           sysState.diagnostics.selfTestCount,
           sysState.diagnostics.lastSelfTestResult);
    printf("Sensor health: %d%%\n", sysState.diagnostics.sensorHealth);
    printf("Sensor noise: %.3f°C\n", sysState.diagnostics.sensorNoiseLevel);
    printf("-------------------------\n");
}