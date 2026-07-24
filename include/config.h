#ifndef CONFIG_H
#define CONFIG_H

// Debug mode - now a runtime variable instead of compile-time constant
// Declared in state.h and defined in main.cpp
extern bool DEBUG_MODE;

// Serial Configuration Mode Settings
#define SERIAL_ACCESS_PASSWORD "menu" // Default command to enter SERIAL_MODE
#define SERIAL_TIMEOUT 300000 // 5 minutes timeout for serial mode (milliseconds)

// Device Information
#define DEVICE_ID "GH_THM-"
#define IDCODE "A9C5ECF3" // Unique device identifier
#define FIRMWARE_VERSION "1.0.0"

// Default GPS location (placeholder — update per deployment site)
#define DEFAULT_LAT 0.0
#define DEFAULT_LON 0.0

// Publishing transport flags
#define USE_HTTP true
#define USE_MQTT true

// pin definitions
#define DHTPIN 22     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302)

#endif // CONFIG_H