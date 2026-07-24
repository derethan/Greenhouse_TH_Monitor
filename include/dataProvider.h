#ifndef DATAPROVIDER_H
#define DATAPROVIDER_H

#include <Arduino.h>
#include "ArduinoJson.h"
#include <vector>
#include "Config.h"

struct sensorData
{
    String sensorID;        // Unique identifier for the sensor (e.g. "DHT-A9C5ECF3")
    String sensorType;      // Human-readable sensor type (e.g. "Device Temperature")
    int status;             // 200 = OK, 503 = sensor fault
    String unit;            // Unit of measurement (e.g. "C", "%")
    unsigned long timestamp;// Unix epoch of the reading
    std::vector<float> values; // Measured values
};

// Class to encapsulate sensor data management
class SensorDataManager
{
private:
    String category;
    std::vector<sensorData> sensorDataList;

    String removeNullCharacters(const String &input) const;
    bool validateSensorData(const sensorData &data);

public:
    SensorDataManager(String cat);
    ~SensorDataManager();

    void addSensorData(const sensorData &data);
    void resetSensorData();

    // Build the full company-spec envelope JSON payload
    String buildFullPayload(const String &deviceID,
                            uint32_t batchId,
                            uint32_t wakeCount,
                            unsigned long timestampEpoch,
                            unsigned long uptimeSeconds,
                            float lat,
                            float lon,
                            const String &firmwareVersion) const;

    void printAllSensorData();

    const std::vector<sensorData> &getAllSensorData() const;
    size_t getSensorDataCount() const;
    bool findSensorById(const String &sensorId, sensorData &result) const;
    void limitDataListSize(size_t maxSize = 100);

    static SensorDataManager &getInstance(String cat = "greenhouse")
    {
        static SensorDataManager instance(cat);
        return instance;
    }
};

#endif // DATAPROVIDER_H