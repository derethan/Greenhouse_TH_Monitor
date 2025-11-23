#ifndef BASE_SYSLOGS_H
#define BASE_SYSLOGS_H

#include <Arduino.h>

/*****************************************
 * System Logs Interface
 *****************************************/

namespace SysLogs
{

    // System logging functions
    void logError(const String &message);
    void logWarning(const String &message);
    void logInfo(const String &messageType, const String &message);

    // Formatting helper
    void printSectionHeader(const String &header);
}

#endif // BASE_SYSLOGS_H