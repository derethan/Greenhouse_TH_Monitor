#include "mqttConnection.h"
#include "NetworkConnections.h"

// ===== Private Static Members =====
static WiFiClient wifiClient; // WiFi client for network connection
WiFiClientSecure net = WiFiClientSecure();
MQTTClient mqttClient = MQTTClient(256);
Preferences MqttConnection::preferences;
NetworkConnections netConn; // Create an instance

static bool isMQTTConnected = false; // Track connection status
static const char broker[] = AWS_IOT_ENDPOINT;
String MqttConnection::deviceID = "";

// ===== Initialization Functions =====
void MqttConnection::initializeMQTT(const char *deviceID)
{
    MqttConnection::deviceID = String(deviceID); // Store the deviceID

    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    // Connect to the MQTT broker on the AWS endpoint we defined earlier
    mqttClient.begin(AWS_IOT_ENDPOINT, AWS_IOT_PORT, net);

    // Create a handler for incoming messages
    mqttClient.onMessage(messageHandler);
}

void MqttConnection::connectMQTT()
{
    Serial.print("Connecting to AWS IoT broker: ");
    Serial.println(broker);

    const int maxAttempts = 5;
    const unsigned long maxDuration = 60000;
    int attempts = 0;
    unsigned long startTime = millis();

    while (!mqttClient.connect(deviceID.c_str(), false))
    {
        Serial.print(".");
        delay(100);
        attempts++;

        if (attempts >= maxAttempts || (millis() - startTime) >= maxDuration)
        {
            Serial.println("Failed to connect to AWS IoT Core within the allowed attempts/duration.");
            isMQTTConnected = false;
            return;
        }
    }

    isMQTTConnected = true;
    Serial.println("You're connected to AWS IoT Core!");

    // Subscribe to incoming message topic
    Serial.println("Subscribing to configuration topic: " + String(AWS_IOT_CONFIGURATION_TOPIC) + deviceID);
    mqttClient.subscribe(String(AWS_IOT_CONFIGURATION_TOPIC) + deviceID);
}

// ===== Connection Management =====
void MqttConnection::checkConnection()
{
    if (!mqttClient.connected())
    {
        connectMQTT();
    }
    mqttClient.loop(); // Keep the MQTT connection alive
}

void MqttConnection::disconnect()
{
    if (mqttClient.connected())
    {
        Serial.println("[MQTT] Disconnecting from MQTT broker...");
        mqttClient.disconnect();
        delay(100);
        Serial.println("[MQTT] Disconnected from MQTT broker");
    }
}

bool MqttConnection::isConnected()
{
    return mqttClient.connected();
}

// ===== Message Handling =====
bool MqttConnection::publishMessage(String message)
{
    Serial.println("Publishing message");

    if (mqttClient.publish(AWS_IOT_PUBLISH_TOPIC, message))
    {

        Serial.println("Message sent successfully");
        return true;
    }

    Serial.println("Failed to send message");
    return false;
}

// handles the incoming messages from AWS IoT Core
void MqttConnection::messageHandler(String &topic, String &payload)
{
    Serial.println("received:");
    Serial.println("- topic: " + topic);
    Serial.println("- payload:");
    Serial.println(payload);

    // Process the incoming data as json object, Then conditionally handle based on topic
    if (topic == AWS_IOT_CONFIGURATION_TOPIC + deviceID)
    {
        handleConfigurationTopic(payload);
    }
}

void MqttConnection::handleConfigurationTopic(String &payload)
{
    // Verify we have a valid payload
    if (payload.length() == 0)
    {
        Serial.println("Error: Empty calibration payload");
        return;
    }

    // Parse the JSON payload
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    // Check for parsing errors
    if (error)
    {
        Serial.print("Error parsing JSON: ");
        Serial.println(error.c_str());
        return;
    }

    // Validate required fields
    if (doc["device_id"].isNull() || doc["settings"].isNull())
    {
        errorLog("Invalid configuration: missing required fields", true);
        return;
    }

    // Extract device ID from message
    String messageDeviceId = doc["device_id"].as<String>();

    // Verify the message is intended for this device
    if (messageDeviceId != deviceID)
    {
        errorLog("Configuration message not for this device", true);
        return;
    }

    // Check for collection_interval in settings
    if (!doc["settings"]["collection_interval"].isNull())
    {
        // Extract the collection interval in ms
        unsigned long interval = doc["settings"]["collection_interval"].as<unsigned long>();

        // Update the collection interval
        state.sensorRead_interval = interval;
        debug("Collection interval update requested: " + String(interval / 1000) + " seconds", true);

        // Send acknowledgment
        String ackMessage = "{\"device_id\":\"" + deviceID +
                            "\",\"status\":\"success\"," +
                            "\"config\":{\"collection_interval\":" + String(interval) + "}}";

        mqttClient.publish(AWS_IOT_CONFIG_ACK_TOPIC, ackMessage);

        // Save to NVS as a backup
        preferences.begin("config", false); //
        preferences.putString("cInterval", String(interval));
        preferences.end();
    }

    else if (!doc["settings"]["publish_interval"].isNull())
    {
        // Extract the collection interval in seconds
        unsigned long interval = doc["settings"]["publish_interval"].as<unsigned long>();

        // Update the collection interval
        state.httpPublishInterval = interval;
        debug("Collection interval update requested: " + String(interval) + " seconds", true);

        // Send acknowledgment
        String ackMessage = "{\"device_id\":\"" + deviceID +
                            "\",\"status\":\"success\"," +
                            "\"config\":{\"publish_interval\":" + String(interval) + "}}";

        mqttClient.publish(AWS_IOT_CONFIG_ACK_TOPIC, ackMessage);

        // Save to NVS as a backup
        preferences.begin("config", false); //
        preferences.putString("pInterval", String(interval));
        preferences.end();
    }

    // Check for Wifi SSID and Password in settings
    else if (!doc["settings"]["wifi_settings"].isNull())
    {
        // Extract the WiFi settings
        JsonObject wifiSettings = doc["settings"]["wifi_settings"];

        // Check for required fields
        if (wifiSettings["ssid"].isNull() || wifiSettings["password"].isNull())
        {
            debug("Invalid WiFi settings: missing required fields", true);
            return;
        }

        // Extract the SSID and password
        String ssid = wifiSettings["ssid"].as<String>();
        String password = wifiSettings["password"].as<String>();

        // Save Wifi Credentials
        netConn.saveWiFiCredentials(ssid, password); // Call the method

        // Send acknowledgment
        String ackMessage = "{\"device_id\":\"" + deviceID +
                            "\",\"status\":\"success\"," +
                            "\"config\":{\"wifi_settings\":{\"ssid\":\"" + ssid + "\"}}}";

        mqttClient.publish(AWS_IOT_CONFIG_ACK_TOPIC, ackMessage);

        debug("WiFi settings updated: " + ssid, true);

        // Restart the device to apply the new settings
        delay(1000);
        ESP.restart();
    }
}

String MqttConnection::getTimestamp()
{
    unsigned long now = NetworkConnections::getTime() * 1000;
    unsigned long seconds = now / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;

    String timestamp = String(hours) + ":" +
                       String(minutes % 60) + ":" +
                       String(seconds % 60) + "." +
                       String(now % 1000);
    return timestamp;
}

void MqttConnection::debug(const String &message, bool publishToAWS)
{

    unsigned long now = NetworkConnections::getTime() * 1000;
    // Create JSON structure
    String jsonMessage = "{";
    jsonMessage += "\"device_id\":\"" + deviceID + "\",";
    jsonMessage += "\"timestamp\":\"" + String(now) + "\",";
    jsonMessage += "\"message\":\"" + message + "\"";
    jsonMessage += "}";

    // Always print to Serial
    Serial.println(jsonMessage);

    // Publish to AWS if requested and connected
    if (publishToAWS && mqttClient.connected())
    {
        // Publish the message to the topic with device ID
        mqttClient.publish(AWS_IOT_DEBUG_TOPIC + deviceID, jsonMessage);
    }
}

void MqttConnection::errorLog(const String &message, bool publishToAWS)
{
    unsigned long now = NetworkConnections::getTime() * 1000;
    // Create JSON structure
    String jsonMessage = "{";
    jsonMessage += "\"device_id\":\"" + deviceID + "\",";
    jsonMessage += "\"timestamp\":\"" + String(now) + "\",";
    jsonMessage += "\"error\":\"" + message + "\"";
    jsonMessage += "}";

    // Always print to Serial
    Serial.println(jsonMessage);

    // Publish to AWS if requested and connected
    if (publishToAWS && mqttClient.connected())
    {
        // Publish the error message to the error topic with device ID
        mqttClient.publish(AWS_IOT_ERROR_TOPIC + deviceID, jsonMessage);
    }
}