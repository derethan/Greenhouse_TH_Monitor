#ifndef CONFIG_H
#define CONFIG_H

// Debug mode - now a runtime variable instead of compile-time constant
// Declared in state.h and defined in main.cpp
extern bool DEBUG_MODE;

// Serial Configuration Mode Settings
#define SERIAL_ACCESS_PASSWORD "adminConfig" // Default password to enter SERIAL_MODE
#define SERIAL_TIMEOUT 300000 // 5 minutes timeout for serial mode (milliseconds)

// Device Information
#define DEVICE_ID "GH_THM-"
#define IDCODE "A9C5ECF3" // Unique device identifier

// pin definitions
#define DHTPIN 22     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302)

#endif // CONFIG_H