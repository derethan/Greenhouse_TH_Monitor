/**
 * @file dataProvider.cpp
 * @brief Sensor data management and JSON conversion implementation
 *
 * Manages a collection of sensor data, validates data integrity, and provides
 * JSON serialization conforming to the company standard payload envelope.
 * Includes size limiting to prevent memory overflow on embedded systems.
 */

#include "dataProvider.h"
#include "base/sysLogs.h"

SensorDataManager::SensorDataManager(String cat) : category(cat) {}

SensorDataManager::~SensorDataManager()
{
    resetSensorData();
}

/**
 * @brief Add a sensor data entry to the collection after validation.
 */
void SensorDataManager::addSensorData(const sensorData &data)
{
    if (validateSensorData(data))
    {
        sensorDataList.push_back(data);
        limitDataListSize();
    }
    else
    {
        SysLogs::logWarning("Invalid sensor data not added");
    }
}

const std::vector<sensorData> &SensorDataManager::getAllSensorData() const
{
    return sensorDataList;
}

size_t SensorDataManager::getSensorDataCount() const
{
    return sensorDataList.size();
}

bool SensorDataManager::findSensorById(const String &sensorId, sensorData &result) const
{
    for (const auto &data : sensorDataList)
    {
        if (data.sensorID == sensorId)
        {
            result = data;
            return true;
        }
    }
    return false;
}

void SensorDataManager::limitDataListSize(size_t maxSize)
{
    if (sensorDataList.size() > maxSize)
    {
        sensorDataList.erase(sensorDataList.begin(),
                             sensorDataList.begin() + (sensorDataList.size() - maxSize));
    }
}

/**
 * @brief Validate sensor data structure for integrity.
 */
bool SensorDataManager::validateSensorData(const sensorData &data)
{
    if (data.sensorID.isEmpty())
        return false;
    if (data.sensorType.isEmpty())
        return false;
    // Allow empty values array only when status indicates a fault
    if (data.status == 200 && data.values.empty())
        return false;
    return true;
}

/**
 * @brief Build the full company-spec payload envelope as a JSON string.
 *
 * Format:
 * {
 *   "id": "<deviceID>",
 *   "batch_id": <n>,
 *   "timestamp": <epoch>,
 *   "wake_count": <n>,
 *   "uptime_s": <n>,
 *   "firmware_version": "<ver>",
 *   "location": { "lat": <f>, "lon": <f> },
 *   "status": "ok",
 *   "sensor_data": [ { "sensor_id", "sensor_type", "status", "unit", "timestamp", "values" }, ... ]
 * }
 */
String SensorDataManager::buildFullPayload(const String &deviceID,
                                           uint32_t batchId,
                                           uint32_t wakeCount,
                                           unsigned long timestampEpoch,
                                           unsigned long uptimeSeconds,
                                           float lat,
                                           float lon,
                                           const String &firmwareVersion) const
{
    JsonDocument doc;

    doc["id"]               = removeNullCharacters(deviceID);
    doc["batch_id"]         = batchId;
    doc["timestamp"]        = timestampEpoch;
    doc["wake_count"]       = wakeCount;
    doc["uptime_s"]         = uptimeSeconds;
    doc["firmware_version"] = firmwareVersion;

    JsonObject location = doc["location"].to<JsonObject>();
    location["lat"] = lat;
    location["lon"] = lon;

    doc["status"] = "ok";

    JsonArray sensorArray = doc["sensor_data"].to<JsonArray>();

    for (const auto &data : sensorDataList)
    {
        JsonObject entry = sensorArray.add<JsonObject>();
        entry["sensor_id"]   = removeNullCharacters(data.sensorID);
        entry["sensor_type"] = removeNullCharacters(data.sensorType);
        entry["status"]      = data.status;
        entry["unit"]        = removeNullCharacters(data.unit);
        entry["timestamp"]   = data.timestamp;

        JsonArray vals = entry["values"].to<JsonArray>();
        for (float v : data.values)
        {
            vals.add(v);
        }
    }

    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}

/**
 * @brief Remove null characters from a string to prevent JSON injection issues.
 */
String SensorDataManager::removeNullCharacters(const String &input) const
{
    String output;
    for (char c : input)
    {
        if (c != '\0')
            output += c;
    }
    return output;
}

void SensorDataManager::resetSensorData()
{
    sensorDataList.clear();
}

/**
 * @brief Print all sensor data to the debug log.
 */
void SensorDataManager::printAllSensorData()
{
    for (const auto &data : sensorDataList)
    {
        SysLogs::print("Sensor ID: ");   SysLogs::println(data.sensorID);
        SysLogs::print("Sensor Type: "); SysLogs::println(data.sensorType);
        SysLogs::print("Status: ");      SysLogs::println(String(data.status));
        SysLogs::print("Unit: ");        SysLogs::println(data.unit);
        SysLogs::print("Timestamp: ");   SysLogs::println(String(data.timestamp));
        SysLogs::print("Values: ");
        for (float v : data.values)
        {
            SysLogs::print(String(v));
            SysLogs::print(" ");
        }
        SysLogs::println();
        SysLogs::println("----------------------------");
    }
}
