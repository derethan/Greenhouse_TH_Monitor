#include "base/sysLogs.h"
#include "state.h"

void SysLogs::printSectionHeader(const String &header)
{
    Serial.println();
    Serial.println("===== " + header + " =====");
}

void SysLogs::logError(const String &message)
{
    state.lastErrorMessage = message;
    state.lastErrorTime = state.currentTime;
    if (DEBUG_MODE)
    {
        Serial.print("[ERROR]: ");
        Serial.println(message);
    }
}

void SysLogs::logWarning(const String &message)
{
    if (DEBUG_MODE)
    {
        Serial.print("[WARNING]: ");
        Serial.println(message);
    }
}

void SysLogs::logInfo(const String &messageType, const String &message)
{
    if (DEBUG_MODE)
    {
        Serial.print("[ ");
        Serial.print(messageType);
        Serial.print("]: ");
        Serial.println(message);
    }
}