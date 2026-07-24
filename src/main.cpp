/**
 * @file main.cpp
 * @brief Main firmware for Garden Guardian Temperature & Humidity Monitor
 *
 * ESP32-based greenhouse monitoring system that reads DHT sensor data, manages
 * network connectivity (WiFi/AP modes), publishes data via MQTT/HTTP, and supports
 * deep sleep power management for battery operation.
 *
 * System supports multiple operating modes:
 * - INITIALIZING: Boot and initial configuration
 * - NORMAL_OPERATION: Regular sensor reading and data publishing
 * - CONFIG_MODE: Access point mode for WiFi configuration
 * - WAKE_UP: Recovery mode after deep sleep
 * - SERIAL_MODE: Serial configuration interface (future feature)
 * - ERROR: Error handling state
 */

// System Libraries
#include <Arduino.h>
#include <time.h>
#include "esp_sleep.h"
#include <Preferences.h>

// Include System Libraries
#include "networkConnections.h"
#include "mqttConnection.h"

// Device Libraries
#include "dhtSensor.h"

// Config
#include "config.h"
#include "secrets.h"

// Base configuration system
#include "state.h"
#include "base/deviceConfig.h"
#include "base/sysLogs.h"
#include "base/serialConfig.h"

#include "tempHumDeviceConfig.h"

// Runtime DEBUG_MODE variable (replaces compile-time constant)
bool DEBUG_MODE = true;

// NVS namespace for payload counters
static Preferences payloadPrefs;

/**
 * @brief Global system state object
 */
SystemState state;

/**
 * @brief Sensor data manager for greenhouse category
 */
SensorDataManager sensorDataManager("greenhouse");

/**
 * @brief Network management object
 */
NetworkConnections network;

/**
 * @brief MQTT connection object
 */
MqttConnection mqtt;

/**
 * @brief DHT temperature and humidity sensor object
 */
DHTSensor dhtSensor(DHTPIN, DHTTYPE);

/**
 * @brief Latest sensor readings for web display
 */
LatestReadings latestReadings;

// Forward Declarations
void loadDeviceSettings();
void setupNetwork();
void initializeMQTT();

bool readSensorData(bool discardReading = false);
bool readDHTData(bool discardReading = false);

bool publishDataWithHTTP(const String &jsonPayload);
bool publishDataWithMQTT(const String &jsonPayload);

/**
 * @brief Load batch_id and wake_count from NVS, increment them, save back, and update state.
 */
void incrementAndPersistPayloadCounters()
{
  payloadPrefs.begin("payload", false);
  state.batch_id   = payloadPrefs.getUInt("batch_id",   0) + 1;
  state.wake_count = payloadPrefs.getUInt("wake_count",  0) + 1;
  payloadPrefs.putUInt("batch_id",   state.batch_id);
  payloadPrefs.putUInt("wake_count", state.wake_count);
  payloadPrefs.end();
}

/**
 * @brief Load and apply device-specific settings from storage
 *
 * Uses the base configuration system with a device-specific applier
 * to load and apply temperature/humidity monitor settings.
 */
void loadDeviceSettings()
{
  // Create device-specific settings applier
  TempHumDeviceSettingsApplier applier(state);

  // Use the base configuration system to load and apply settings
  BaseConfig::loadAndApplyDeviceSettings(network, &applier);
}

/**
 * @brief Setup network connections (WiFi/AP mode)
 *
 * Initializes WiFi connection with stored credentials, performs NTP time
 * synchronization, and falls back to RTC if NTP fails. Configures access
 * point mode if WiFi connection fails.
 */
void setupNetwork()
{
  WiFiCredentials credentials = network.loadWiFiCredentials();
  network.setupWiFi(credentials, state.idCode, state.apAlwaysOn);

  if (network.isConnected())
  {
    // Attempt NTP synchronization
    unsigned long ntpTime = network.getTime();
    if (ntpTime == 0)
    {
      SysLogs::logWarning("NTP synchronization failed. Continuing with RTC time if available.");
      // Try to get RTC time as fallback
      unsigned long rtcTime = network.getRTCTime();
      if (rtcTime == 0)
      {
        SysLogs::logWarning("No valid time source available (neither NTP nor RTC)");
        SysLogs::logWarning("Timestamps in sensor data may be inaccurate");
      }
      else
      {
        SysLogs::logInfo("NETWORK", "Using RTC time as fallback");
      }
    }
    else
    {
      SysLogs::logSuccess("NETWORK", "System time synchronized successfully via NTP");
    }

    // Start the web server when connected to WiFi
    // network.startWebServer();
  }
}

/**
 * @brief Initialize MQTT connection to AWS IoT Core
 *
 * Configures and connects to AWS IoT Core if WiFi is connected and not in AP mode.
 * Combines device ID with ID code for full device identifier.
 */
void initializeMQTT()
{
  if (!network.isAPMode() && WiFi.status() == WL_CONNECTED)
  {
    // Combine device_id with idCode for the full device identifier
    String fullDeviceID = state.deviceID + state.idCode;
    mqtt.initializeMQTT(fullDeviceID.c_str());
    mqtt.connectMQTT();
  }
}

/**
 * @brief Read sensor data from all connected sensors
 * @param discardReading If true, updates latest readings but doesn't store for publishing
 * @return true if all sensor reads successful, false if any failed
 *
 * Acts as a wrapper for specific sensor reading functions. Currently only reads DHT data
 * but designed to support multiple sensor types in the future.
 */
bool readSensorData(bool discardReading)
{

  SysLogs::println();
  SysLogs::printSectionHeader("Data Collection Starting");
  SysLogs::logInfo("SENSOR", "Reading sensor data at t=" + String(millis()));

  if (!readDHTData(discardReading))
  {
    SysLogs::logError("Failed to read DHT data");
    state.sensorError = true;
    return false;
  }

  sensorDataManager.printAllSensorData();

  SysLogs::printSectionHeader("Data Collection Complete");
  SysLogs::println();

  state.sensorError = false;
  return true;
}

/**
 * @brief Read temperature and humidity data from DHT sensor
 * @param discardReading If true, updates latest readings but doesn't store for publishing
 * @return true if readings are valid, false if sensor read failed
 *
 * Reads temperature and humidity from DHT sensor, updates latest readings for web display,
 * and optionally stores data for publishing. Handles sensor errors and invalid readings.
 */
bool readDHTData(bool discardReading)
{

  // Read temperature and humidity
  float temp = dhtSensor.readTemperature();
  float hum = dhtSensor.readHumidity();
  int status = 200; // Default status

  // Check if readings are valid
  if (isnan(temp) || isnan(hum))
  {
    SysLogs::logError("Failed to read from DHT sensor!");
    state.sensorError = true;
    state.lastErrorTime = millis();
    status = 503; // Sensor fault status per company payload spec

    // Update latest readings with error status
    latestReadings.temperatureStatus = 503;
    latestReadings.humidityStatus = 503;
  }
  else
  {
    // Always update latest readings for web display (even if discarding for publishing)
    latestReadings.temperature = temp;
    latestReadings.humidity = hum;
    latestReadings.temperatureTimestamp = state.currentTime;
    latestReadings.humidityTimestamp = state.currentTime;
    latestReadings.temperatureStatus = status;
    latestReadings.humidityStatus = status;
    latestReadings.hasValidData = true;
  }

  // Only update the stored values for publishing if we're not discarding the reading
  if (!discardReading)
  {

    sensorDataManager.addSensorData({.sensorID = "DHT-" + state.idCode,
                                     .sensorType = "Air Temperature",
                                     .status = status,
                                     .unit = "C",
                                     .timestamp = state.currentTime,
                                     .values = {temp}});

    sensorDataManager.addSensorData({.sensorID = "DHT-" + state.idCode,
                                     .sensorType = "Air Humidity",
                                     .status = status,
                                     .unit = "%",
                                     .timestamp = state.currentTime,
                                     .values = {hum}});
  }
  else
  {
    SysLogs::logInfo("SENSOR", "Reading discarded for publishing (device in stabilization period)");
    SysLogs::logInfo("SENSOR", "But latest readings updated for web display");
  }

  return !isnan(temp) && !isnan(hum);
}

/**
 * @brief Publish sensor data via MQTT to AWS IoT Core
 * @return true if all data published successfully, false if any publish failed
/**
 * @brief Publish the pre-built payload via MQTT to AWS IoT Core.
 * @param jsonPayload Full company-spec JSON envelope string
 * @return true if published successfully
 */
bool publishDataWithMQTT(const String &jsonPayload)
{
  SysLogs::logInfo("MQTT", "Publishing sensor payload via MQTT...");

  if (!mqtt.isConnected())
  {
    SysLogs::logWarning("MQTT not connected, skipping publish");
    return false;
  }

  if (!mqtt.publishMessage(jsonPayload))
  {
    SysLogs::logError("Failed to publish payload via MQTT");
    return false;
  }

  SysLogs::logSuccess("MQTT", "Payload published successfully");
  return true;
}

/**
 * @brief Publish the pre-built payload via HTTP POST to the server endpoint.
 * @param jsonPayload Full company-spec JSON envelope string
 * @return true if published successfully
 */
bool publishDataWithHTTP(const String &jsonPayload)
{
  SysLogs::printSectionHeader("HTTP");
  SysLogs::logInfo("HTTP", "Publishing sensor payload via HTTP...");

  bool publishSuccess = network.publishSensorData(jsonPayload);

  if (publishSuccess)
  {
    SysLogs::logSuccess("HTTP", "Payload published successfully");
  }
  else
  {
    SysLogs::logError("Failed to publish payload via HTTP");
  }

  return publishSuccess;
}

/**
 * @brief Enter deep sleep mode with proper WiFi cleanup
 * @param currentMillis Current system time in milliseconds
 *
 * Calculates optimal sleep duration based on next scheduled sensor reading or publish event.
 * Properly disconnects WiFi and MQTT before entering ESP32 light sleep mode. Wakes up
 * via timer interrupt.
 */
void sleep(unsigned long currentMillis)
{
  // Calculate which interval is next (reading or publishing)
  unsigned long timeToNextReading = state.sensorRead_interval - (currentMillis - state.lastReadingTime);
  unsigned long timeToNextPublish = state.httpPublishInterval - (currentMillis - state.lastHTTPPublishTime);
  unsigned long sleepDuration = min(timeToNextReading, timeToNextPublish); // Sleep until the next scheduled event

  // Ensure minimum wake duration is respected
  // if (sleepDuration < state.MIN_WAKE_DURATION)
  // {
  //   sleepDuration = state.MIN_WAKE_DURATION;
  // }

  SysLogs::logInfo("SYSTEM", "Entering sleep for " + String(sleepDuration) + " ms");

  // Properly disconnect WiFi and MQTT before sleep
  if (network.isConnected())
  {
    SysLogs::logInfo("SYSTEM", "Disconnecting MQTT and WiFi before sleep...");
    mqtt.disconnect();
    delay(100);
    network.disconnectWiFi();
    delay(200);
  }

  // Brief delay for serial to complete and to prevent busy-waiting
  delay(100);

  // Enter light sleep
  esp_sleep_enable_timer_wakeup(sleepDuration * 1000); // Convert ms to us
  esp_light_sleep_start();
  state.currentMode = SystemMode::WAKE_UP;
  SysLogs::logInfo("SYSTEM", "Woke up from sleep");
}

/**
 * @brief Main system setup function
 *
 * Initializes serial communication, sensors, network connections, and MQTT.
 * Called once at system boot. Loads device settings from storage and configures
 * all system components before entering the main loop.
 */
void setup()
{
  Serial.begin(115200);
  delay(5000); // Allow time for serial to initialize
  SysLogs::logInfo("SYSTEM", "Starting Garden Guardian - Temperature & Humidity Monitor");

  // Record start time for stabilization tracking
  state.deviceStartTime = millis();
  SysLogs::logInfo("SYSTEM", "Device start time: t=" + String(state.deviceStartTime));

  SysLogs::logInfo("SENSOR", "Initializing sensors...");

  // Initialize DHT sensor
  if (!dhtSensor.begin())
  {
    state.sensorError = true;
    state.lastErrorTime = millis();
    SysLogs::logError("Failed to connect to DHT sensor!");
  }
  else
  {
    SysLogs::logInfo("SENSOR", "DHT sensor initialized successfully.");
  }
  delay(1000);

  // Load and apply device settings BEFORE network initialization
  loadDeviceSettings();

  // Load persisted payload counters
  payloadPrefs.begin("payload", true);
  state.batch_id   = payloadPrefs.getUInt("batch_id",   0);
  state.wake_count = payloadPrefs.getUInt("wake_count",  0);
  payloadPrefs.end();
  SysLogs::logInfo("SYSTEM", "Payload counters loaded — batch_id=" + String(state.batch_id) + " wake_count=" + String(state.wake_count));

  // Initialize Network
  SysLogs::logInfo("SYSTEM", "Initializing network connections...");
  setupNetwork();

  // Initialize MQTT if connected to network
  SysLogs::logInfo("SYSTEM", "Initializing MQTT connection...");
  initializeMQTT();

  SysLogs::logInfo("SYSTEM", "Setup complete.");
}

/**
 * @brief Main system loop function
 *
 * Implements a state machine that handles different operating modes:
 * - INITIALIZING: Determines initial operating mode (normal or config)
 * - NORMAL_OPERATION: Regular sensor reading, data publishing, and sleep cycles
 * - CONFIG_MODE: Access point mode for WiFi configuration
 * - WAKE_UP: Recovery and reconnection after sleep
 * - SERIAL_MODE: Serial configuration interface
 * - ERROR: Error handling state
 *
 * Runs continuously after setup() completes.
 */
void loop()
{
  unsigned long currentMillis = millis();

  // Check for serial access password in any mode except SERIAL_MODE
  // Note: checkForSerialAccess() is non-blocking and only processes available serial data
  if (state.currentMode != SystemMode::SERIAL_MODE && SerialCLI::checkForSerialAccess())
  {
    state.previousMode = state.currentMode;
    state.currentMode = SystemMode::SERIAL_MODE;
  }

  switch (state.currentMode)
  {
  case SystemMode::INITIALIZING:
  {
    if (network.isConnected())
    {
      state.currentMode = SystemMode::NORMAL_OPERATION;

      // Initialize system state: Time
      state.currentTime = network.getRTCTime();
      state.lastTimeSyncEpoch = state.currentTime;

      // Start web server when connected to WiFi
      // network.startWebServer();
    }
    else if (network.isAPMode())
    {
      state.currentMode = SystemMode::CONFIG_MODE;
      SysLogs::logInfo("SYSTEM", "Entering Configuration Mode, Awaiting Network Configuration...");
      state.currentTime = millis();
    }
    break;
  }

  case SystemMode::NORMAL_OPERATION:
  {
    // Update current time
    if (network.isConnected())
    {
      state.currentTime = network.getRTCTime();
      mqtt.checkConnection();
    }
    else
    {
      state.currentTime = millis();
    }

    // Check device stabilization status
    // if (!state.deviceStabilized && currentMillis - state.deviceStartTime >= state.SENSOR_STABILIZATION_TIME)
    // {
    //   state.deviceStabilized = true;
    //   SysLogs::logInfo("SYSTEM", "DHT Sensor stabilized at t=" + String(currentMillis) + ", DHT Sensor readings will begin.");
    // }

    // Check if it's time to read Data from connected Sensors
    if (currentMillis - state.lastReadingTime >= state.sensorRead_interval)
    {
      state.lastReadingTime = currentMillis;

      // Read sensor, but discard data if not yet stabilized
      // bool discardReading = !state.deviceStabilized;
      // if (discardReading)
      // {
      //   SysLogs::logInfo("SYSTEM", "Device not stabilized yet, reading will be discarded");
      // }

      bool readSuccess = readSensorData(false); // use DiscardReading as param for stabilization
      state.lastSensorRead = state.currentTime;

      if (!readSuccess)
      {
        SysLogs::logError("Sensor read failed during normal operation at t=" + String(currentMillis));
      }
    }

    // Check if it's time to publish data (non-blocking timer)
    if (state.httpPublishEnabled && network.isConnected() &&
        (currentMillis - state.lastHTTPPublishTime >= state.httpPublishInterval))
    {
      SysLogs::logInfo("SYSTEM", "Time to publish sensor data at t=" + String(currentMillis));
      state.lastHTTPPublishTime = currentMillis;

      if (sensorDataManager.getSensorDataCount() > 0)
      {
        // Increment and persist payload counters before building the envelope
        incrementAndPersistPayloadCounters();

        String fullDeviceID = state.deviceID + state.idCode;
        unsigned long uptimeSeconds = millis() / 1000;

        String payload = sensorDataManager.buildFullPayload(
            fullDeviceID,
            state.batch_id,
            state.wake_count,
            state.currentTime,
            uptimeSeconds,
            state.locationLat,
            state.locationLon,
            FIRMWARE_VERSION);

        SysLogs::logDebug("SYSTEM", "Built payload: " + payload);

        // HTTP first, then MQTT — each gated by its transport flag
        if (USE_HTTP)
        {
          publishDataWithHTTP(payload);
        }

        if (USE_MQTT)
        {
          publishDataWithMQTT(payload);
        }

        // Clear sensor data after publishing
        sensorDataManager.resetSensorData();
      }
      else
      {
        SysLogs::logInfo("SYSTEM", "No data to publish yet");
      }
    }

    // Handle web server requests when connected to WiFi
    if (network.isConnected())
    {
      network.handleClientRequestsWithSensorData(latestReadings);
    }

    // Sleep Mode - only if not connected via USB Serial
    if (!Serial)
    {
      sleep(currentMillis);
    }

    break;
  }

  case SystemMode::ERROR:
  {
    break; // System Mode: Error handling (future implementation)
  }

  case SystemMode::CONFIG_MODE:
  {
    // Check device stabilization status
    // if (!state.deviceStabilized && currentMillis - state.deviceStartTime >= state.SENSOR_STABILIZATION_TIME)
    // {
    //   state.deviceStabilized = true;
    //   SysLogs::logInfo("SYSTEM", "DHT Sensor stabilized at t=" + String(currentMillis) + ", DHT Sensor readings will begin.");
    // }

    // Check if it's time to read Data from connected Sensors (even in config mode)
    if (currentMillis - state.lastReadingTime >= state.sensorRead_interval)
    {
      SysLogs::logInfo("SYSTEM", "Time to take a sensor reading at t=" + String(currentMillis));
      state.lastReadingTime = currentMillis;

      // Read sensor, but discard data if not yet stabilized
      // bool discardReading = !state.deviceStabilized;
      // if (discardReading)
      // {
      //   SysLogs::logInfo("SYSTEM", "Device not stabilized yet, reading will be discarded");
      // }

      bool readSuccess = readSensorData(false);
      state.lastSensorRead = state.currentTime;
    }

    // Process DNS first
    // network.processDNSRequests();
    // network.handleClientRequestsWithSensorData(latestReadings);
    break;
  }

  case SystemMode::WAKE_UP:
  {

    // We need to reconnect network after wake-up
    SysLogs::logInfo("SYSTEM", "Re-initializing network connections after wake-up...");

    // Try reconnection with retry logic first
    bool reconnected = network.reconnectToNetwork(3);

    if (!reconnected)
    {
      SysLogs::logInfo("SYSTEM", "Reconnection failed, trying full WiFi setup...");
      WiFiCredentials credentials = network.loadWiFiCredentials();
      network.setupWiFi(credentials, state.idCode, state.apAlwaysOn);
    }

    // Reconnect MQTT if WiFi is connected
    if (network.isConnected())
    {
      SysLogs::logInfo("SYSTEM", "Reconnecting MQTT...");
      mqtt.checkConnection();
      state.currentMode = SystemMode::NORMAL_OPERATION;

      // Update time after reconnection
      state.currentTime = network.getRTCTime();
      SysLogs::logInfo("SYSTEM", "WiFi reconnected successfully, resuming normal operation");
    }
    else
    {
      SysLogs::logInfo("SYSTEM", "WiFi reconnection failed, entering configuration mode");
      state.currentMode = SystemMode::CONFIG_MODE;
    }

    break;
  }

  case SystemMode::SERIAL_MODE:
  {
    SysLogs::logInfo("SYSTEM", "Entering Serial Configuration Mode");

    // Serial configuration mode for user interaction
    // This mode disables debug logging and presents an interactive menu
    // for device configuration, diagnostics, and system management
    SerialCLI::enterSerialMode(state, network, dhtSensor, latestReadings);
    // Mode will be restored to previous state by enterSerialMode()
    break;
  }
  }

  // Add a debug statement every 30 seconds to show the system is still running
  static unsigned long lastHeartbeat = 0;
  if (currentMillis - lastHeartbeat > 30000)
  {
    SysLogs::logInfo("SYSTEM", "Heartbeat at t=" + String(currentMillis) + ", stabilized=" + String(state.deviceStabilized ? "true" : "false"));
    lastHeartbeat = currentMillis;

    // Print Current time from RTC
    SysLogs::logInfo("SYSTEM", "Current time from RTC: Unix Epoch: " + String(state.currentTime));

    // Print Local Time
    time_t rawtime = static_cast<time_t>(state.currentTime);
    struct tm *timeinfo = localtime(&rawtime);
    SysLogs::logInfo("SYSTEM", "Local time: " + String(asctime(timeinfo)));

    // Network Status
    SysLogs::logInfo("NETWORK", "Connected: " + String(network.isConnected() ? "Yes" : "No"));
    SysLogs::logInfo("NETWORK", "AP Mode: " + String(network.isAPMode() ? "Yes" : "No"));

    SysLogs::logDebug("DEBUG", "Time since last HTTP publish: " + String(currentMillis - state.lastHTTPPublishTime) + " ms, Interval: " + String(state.httpPublishInterval) + " ms");

    // add time remaining and is time to publish boolean
    unsigned long timeSinceLastPublish = currentMillis - state.lastHTTPPublishTime;
    unsigned long timeRemaining = (timeSinceLastPublish >= state.httpPublishInterval) ? 0 : (state.httpPublishInterval - timeSinceLastPublish);
    SysLogs::logDebug("DEBUG", "Time remaining until next HTTP publish: " + String(timeRemaining) + " ms");
    SysLogs::logDebug("DEBUG", "Is it time to publish? " + String((timeSinceLastPublish >= state.httpPublishInterval) ? "Yes" : "No"));
  }

  // Short delay to prevent busy-waiting
  delay(100);
}
