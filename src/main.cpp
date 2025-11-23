// System Libraries
#include <Arduino.h>
#include <time.h>
#include "esp_sleep.h"

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
#include "tempHumDeviceConfig.h"

/*****************************************
 * Global Objects
 *****************************************/
SystemState state;
SensorDataManager sensorDataManager("greenhouse"); // Sensor data manager for greenhouse category
NetworkConnections network;                        // Network management object
MqttConnection mqtt;                               // MQTT connection object
DHTSensor dhtSensor(DHTPIN, DHTTYPE);
LatestReadings latestReadings; // Store latest readings for web display

/*****************************************
 * Forward Declarations
 *****************************************/
void loadDeviceSettings();
void setupNetwork();
bool readSensorData(bool discardReading = false);
bool readDHTData(bool discardReading = false);
void logError(const char *message);

/*****************************************
 * Configuration Functions
 *****************************************/
void loadDeviceSettings()
{
  // Create device-specific settings applier
  TempHumDeviceSettingsApplier applier(state);

  // Use the base configuration system to load and apply settings
  BaseConfig::loadAndApplyDeviceSettings(network, &applier);
}

/*****************************************
 * Utility Functions
 *****************************************/
void logError(const char *message)
{
  state.lastErrorMessage = message;
  state.lastErrorTime = state.currentTime;
  if (DEBUG_MODE)
  {
    Serial.print("ERROR: ");
    Serial.println(message);
  }
}

/*****************************************
 * System Functions
 *****************************************/

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
      Serial.println("NTP synchronization failed. Continuing with RTC time if available.");
      // Try to get RTC time as fallback
      unsigned long rtcTime = network.getRTCTime();
      if (rtcTime == 0)
      {
        Serial.println("WARNING: No valid time source available (neither NTP nor RTC)");
        Serial.println("Timestamps in sensor data may be inaccurate");
      }
      else
      {
        Serial.println("Using RTC time as fallback");
      }
    }
    else
    {
      Serial.println("System time synchronized successfully via NTP");
    }

    // Start the web server when connected to WiFi
    // network.startWebServer();
  }
}

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

bool readSensorData(bool discardReading)
{
  if (!readDHTData(discardReading))
  {
    logError("Sensor Error: Failed to read DHT data");
    state.sensorError = true;
    return false;
  }
  else
  {
    state.sensorError = false;
    return true;
  }
}

bool readDHTData(bool discardReading)
{
  Serial.println();
  Serial.println("--------- Data Collection Starting ---------");
  Serial.print("[SENSOR] Reading sensor data at t=");
  Serial.println(millis());

  // Read temperature and humidity
  float temp = dhtSensor.readTemperature();
  float hum = dhtSensor.readHumidity();
  int status = 200; // Default status

  // Check if readings are valid
  if (isnan(temp) || isnan(hum))
  {
    Serial.println("[SENSOR] ERROR: Failed to read from DHT sensor!");
    state.sensorError = true;
    state.lastErrorTime = millis();
    status = 500; // Update status to indicate error

    // Update latest readings with error status
    latestReadings.temperatureStatus = 500;
    latestReadings.humidityStatus = 500;
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

    Serial.print("[SENSOR] Latest readings updated - Temp: ");
    Serial.print(temp);
    Serial.print("°C, Humidity: ");
    Serial.print(hum);
    Serial.println("%");
  }

  // Only update the stored values for publishing if we're not discarding the reading
  if (!discardReading)
  {
    Serial.println("[SENSOR] Reading stored and ready for transmission");

    sensorDataManager.addSensorData({.sensorID = "DHT-" + state.idCode,
                                     .sensorType = {"airTemperature"},
                                     .sensorName = "DHT",
                                     .status = status,
                                     .unit = {"°C"},
                                     .timestamp = state.currentTime,
                                     .values = {temp}});

    sensorDataManager.addSensorData({.sensorID = "DHT-" + state.idCode,
                                     .sensorType = {"airHumidity"},
                                     .sensorName = "DHT",
                                     .status = status,
                                     .unit = {"%"},
                                     .timestamp = state.currentTime,
                                     .values = {hum}});
  }
  else
  {
    Serial.println("[SENSOR] Reading discarded for publishing (device in stabilization period)");
    Serial.println("[SENSOR] But latest readings updated for web display");
  }

  Serial.println("--------- Data Collection Complete ---------");
  Serial.println();

  return !isnan(temp) && !isnan(hum);
}

// Function to publish the Data
bool publishDataWithMQTT()
{
  Serial.print("[MQTT] Publishing sensor data via MQTT...");
  Serial.print(sensorDataManager.getSensorDataCount());
  Serial.println(" sensor data items...");

  // Combine device_id with idCode for the full device identifier
  String fullDeviceID = state.deviceID + state.idCode;

  const std::vector<sensorData> &dataList = sensorDataManager.getAllSensorData();
  bool allPublished = true;

  for (const auto &data : dataList)
  {

    // Print the sensor being published
    Serial.print("Publishing sensor data for: ");
    Serial.println(data.sensorID);

    // Convert sensor data to JSON format
    String jsonPayload = sensorDataManager.convertSensorDataToJSON(data, fullDeviceID);

    // Publish to MQTT
    if (mqtt.isConnected())
    {
      // Publish the JSON data to the MQTT Broker
      if (!mqtt.publishMessage(jsonPayload))
      {
        Serial.println("Failed to publish to MQTT: " + data.sensorID);
        allPublished = false;
      }
    }
    else
    {
      Serial.println("MQTT not connected, skipping publish for: " + data.sensorID);
      allPublished = false;
    }

    // Delay for 1 second between each publish
    delay(1000);
  }
  return allPublished;
}

bool publishDataWithHTTP()
{
  Serial.print("[HTTP] Publishing ");
  Serial.print(sensorDataManager.getSensorDataCount());
  Serial.println(" sensor data items...");

  // Combine device_id with idCode for the full device identifier
  String fullDeviceID = state.deviceID + state.idCode;

  // Publish sensor data via HTTP
  bool publishSuccess = network.publishSensorData(sensorDataManager, fullDeviceID);

  if (publishSuccess)
  {
    Serial.println("[HTTP] Data published successfully - clearing sensor data buffer");
  }
  else
  {
    Serial.println("[HTTP] Failed to publish data - keeping data for next attempt");
  }

  return publishSuccess;
}

// Sleep function with proper WiFi management
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

  Serial.print("[SYSTEM] Preparing to sleep for ");
  Serial.print(sleepDuration);
  Serial.println(" milliseconds");

  // Properly disconnect WiFi and MQTT before sleep
  if (network.isConnected())
  {
    Serial.println("[SYSTEM] Disconnecting MQTT and WiFi before sleep...");
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
  Serial.println("[SYSTEM] Woke up from sleep");
}

void setup()
{
  Serial.begin(115200);
  delay(5000); // Allow time for serial to initialize
  Serial.println("\n\n[SYSTEM] Garden Guardian - Temperature & Humidity Monitor");

  // Record start time for stabilization tracking
  state.deviceStartTime = millis();
  Serial.print("[SYSTEM] Device start time: t=");
  Serial.println(state.deviceStartTime);

  Serial.println("[SENSOR] Initializing sensors...");

  // Initialize DHT sensor
  if (!dhtSensor.begin())
  {
    Serial.println("[SENSOR] ERROR: Failed to connect to DHT sensor!");
    state.sensorError = true;
    state.lastErrorTime = millis();
  }
  else
  {
    Serial.println("[SENSOR] DHT sensor initialized successfully");
  }
  delay(1000);

  // Load and apply device settings BEFORE network initialization
  loadDeviceSettings();

  // Initialize Network
  Serial.println("[SYSTEM] Initializing network connections...");
  setupNetwork();

  // Initialize MQTT if connected to network
  Serial.println("[SYSTEM] Initializing MQTT connection...");
  initializeMQTT();

  Serial.print("[SYSTEM] Setup complete.");
}

void loop()
{
  unsigned long currentMillis = millis();

  // Update system mode based on conditions
  switch (state.currentMode)
  {
  case SystemMode::INITIALIZING:
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
      Serial.println("Entering Configuration Mode, Awaiting Network Configuration...");
      state.currentTime = millis();
    }
    break;

  case SystemMode::NORMAL_OPERATION:

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
    //   Serial.print("[SYSTEM] DHT Sensor stabilized at t=");
    //   Serial.print(currentMillis);
    //   Serial.println(", DHT Sensor readings will begin.");
    // }

    // Check if it's time to read Data from connected Sensors
    if (currentMillis - state.lastReadingTime >= state.sensorRead_interval)
    {
      Serial.print("[SYSTEM] Time to take a sensor reading at t=");
      Serial.println(currentMillis);
      state.lastReadingTime = currentMillis;

      // Read sensor, but discard data if not yet stabilized
      // bool discardReading = !state.deviceStabilized;
      // if (discardReading)
      // {
      //   Serial.println("[SYSTEM] Device not stabilized yet, reading will be discarded");
      // }

      bool readSuccess = readSensorData(false);

      if (DEBUG_MODE)
      {
        // print all sensor data
        sensorDataManager.printAllSensorData();
      }
      state.lastSensorRead = state.currentTime;
    }

    // Debugging, log time to publish details
    if (DEBUG_MODE)
    {
      Serial.print("[DEBUG] Time since last HTTP publish: ");
      Serial.print(currentMillis - state.lastHTTPPublishTime);
      Serial.print(" ms, Interval: ");
      Serial.print(state.httpPublishInterval);
      Serial.println(" ms");

      // add time remaining and is time to publish boolen
      unsigned long timeSinceLastPublish = currentMillis - state.lastHTTPPublishTime;
      unsigned long timeRemaining = (timeSinceLastPublish >= state.httpPublishInterval) ? 0 : (state.httpPublishInterval - timeSinceLastPublish);
      Serial.print("[DEBUG] Time remaining until next HTTP publish: ");
      Serial.print(timeRemaining);
      Serial.println(" ms");
      Serial.print("[DEBUG] Is it time to publish? ");
      Serial.println((timeSinceLastPublish >= state.httpPublishInterval) ? "Yes" : "No");

    }
    // Check if it's time to publish data via HTTP (non-blocking timer)
    if (state.httpPublishEnabled && network.isConnected() &&
        (currentMillis - state.lastHTTPPublishTime >= state.httpPublishInterval))
    {
      Serial.print("[HTTP/MQTT] Time to publish sensor data at t=");
      Serial.println(currentMillis);
      state.lastHTTPPublishTime = currentMillis;

      // Only publish if we have data and device is stabilized
      // if (state.deviceStabilized && sensorData.getSensorDataCount() > 0)
      if (sensorDataManager.getSensorDataCount() > 0)
      {

        // First handle HTTP Publishing
        // publishDataWithHTTP();

        // handle Publishing via MQTT as well
        publishDataWithMQTT();

        // Clear sensor data after publishing
        sensorDataManager.resetSensorData();
      }
      else
      {
        Serial.println("[HTTP] No data to publish or device not stabilized yet");
      }
    }

    // Handle web server requests when connected to WiFi
    // if (network.isConnected())
    // {
    //   network.handleClientRequestsWithSensorData(latestReadings);
    // }

    // Sleep testing
    sleep(currentMillis);

    break;

    // System Mode: Error
    // This mode is entered when a sensor error is detected
  case SystemMode::ERROR:
    break; // System Mode: Configuration
    // This mode is entered when the system is in AP mode or Web Server mode
  case SystemMode::CONFIG_MODE:

    // Check device stabilization status
    // if (!state.deviceStabilized && currentMillis - state.deviceStartTime >= state.SENSOR_STABILIZATION_TIME)
    // {
    //   state.deviceStabilized = true;
    //   Serial.print("[SYSTEM] DHT Sensor stabilized at t=");
    //   Serial.print(currentMillis);
    //   Serial.println(", DHT Sensor readings will begin.");
    // }

    // Check if it's time to read Data from connected Sensors (even in config mode)
    if (currentMillis - state.lastReadingTime >= state.sensorRead_interval)
    {
      Serial.print("[SYSTEM] Time to take a sensor reading at t=");
      Serial.println(currentMillis);
      state.lastReadingTime = currentMillis;

      // Read sensor, but discard data if not yet stabilized
      // bool discardReading = !state.deviceStabilized;
      // if (discardReading)
      // {
      //   Serial.println("[SYSTEM] Device not stabilized yet, reading will be discarded");
      // }

      bool readSuccess = readSensorData(false);
      state.lastSensorRead = state.currentTime;
    }

    // Process DNS first
    // network.processDNSRequests();
    // network.handleClientRequestsWithSensorData(latestReadings);
    break;
  case SystemMode::WAKE_UP:

    // We need to reconnect network after wake-up
    Serial.println("[SYSTEM] Re-initializing network connections after wake-up...");

    // Try reconnection with retry logic first
    bool reconnected = network.reconnectToNetwork(3);

    if (!reconnected)
    {
      Serial.println("[SYSTEM] Reconnection failed, trying full WiFi setup...");
      WiFiCredentials credentials = network.loadWiFiCredentials();
      network.setupWiFi(credentials, state.idCode, state.apAlwaysOn);
    }

    // Reconnect MQTT if WiFi is connected
    if (network.isConnected())
    {
      Serial.println("[SYSTEM] Reconnecting MQTT...");
      mqtt.checkConnection();
      state.currentMode = SystemMode::NORMAL_OPERATION;

      // Update time after reconnection
      state.currentTime = network.getRTCTime();
      Serial.println("[SYSTEM] WiFi reconnected successfully, resuming normal operation");
    }
    else
    {
      Serial.println("[SYSTEM] WiFi reconnection failed, entering configuration mode");
      state.currentMode = SystemMode::CONFIG_MODE;
    }

    break;
  }

  // Add a debug statement every 30 seconds to show the system is still running
  static unsigned long lastHeartbeat = 0;
  if (currentMillis - lastHeartbeat > 30000 && DEBUG_MODE)
  {
    Serial.println();
    Serial.print("[SYSTEM] Heartbeat at t=");
    Serial.print(currentMillis);
    Serial.print(", stabilized=");
    Serial.println(state.deviceStabilized ? "true" : "false");
    lastHeartbeat = currentMillis;

    // Print Current time from RTC
    Serial.print("[SYSTEM] Current time from RTC: ");
    Serial.print("Unix Epoch: ");
    Serial.println(state.currentTime);
    Serial.print("Formatted Time: ");
    struct tm *timeinfo = localtime((time_t *)&state.currentTime);
    Serial.print(asctime(timeinfo));

    // Network Status
    Serial.print("[NETWORK] Connected: ");
    Serial.println(network.isConnected() ? "Yes" : "No");
    Serial.print("[NETWORK] AP Mode: ");
    Serial.println(network.isAPMode() ? "Yes" : "No");

    Serial.println();
  }

  // Short delay to prevent busy-waiting
  delay(100);
}
