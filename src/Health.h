#ifndef HEALTH_H
#define HEALTH_H

#include <ArduinoJson.h>

// Device health: why the last restart happened, what the previous run looked
// like, crash reports, WiFi stability, and self-healing when the device loses
// WiFi or runs low on memory.
namespace health {

// Call first thing in setup().
void begin();

// Call from loop(). `paused` suspends self-healing restarts (e.g. during OTA).
void loop(bool paused);

// Records why we restart deliberately, then restarts.
[[noreturn]] void restart(const char *cause);

void toJson(JsonObject o);

} // namespace health

#endif // HEALTH_H
