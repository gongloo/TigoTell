#include "Health.h"

#include "TaskLog.h"

#include <Preferences.h>
#include <WiFi.h>
#include <esp_core_dump.h>
#include <esp_system.h>

namespace health {
namespace {

// Self-healing thresholds.
constexpr uint32_t kWifiReconnectAfterMs = 60UL * 1000;
constexpr uint32_t kWifiReconnectEveryMs = 60UL * 1000;
constexpr uint32_t kWifiRestartAfterMs = 10UL * 60 * 1000;
constexpr uint32_t kLowMemoryBlock = 10 * 1024; // largest free block
constexpr uint32_t kLowMemoryRestartAfterMs = 60UL * 1000;

// Survives panics, watchdog resets and software restarts (not power loss).
struct RtcRun {
  uint32_t magic;
  uint32_t uptimeS;
  uint32_t heapMin;
  uint32_t maxBlockMin;
  uint32_t wifiDisconnects;
  char restartCause[40];
  uint32_t check;
};
constexpr uint32_t kRtcMagic = 0x7160A11E;
RTC_NOINIT_ATTR RtcRun rtcRun;

uint32_t rtcCheck(const RtcRun &r) {
  return r.magic ^ r.uptimeS ^ (r.heapMin * 31) ^ (r.maxBlockMin * 17) ^
         (r.wifiDisconnects * 7) ^ 0x5A5A5A5A;
}

// Snapshot taken at boot.
esp_reset_reason_t resetReason;
bool havePrevious = false;
RtcRun previous;
String crashSummary; // most recent crash, persisted in NVS
uint32_t crashCount = 0;

// Runtime state.
volatile uint32_t wifiDisconnects = 0;
volatile uint8_t lastDisconnectReason = 0;
uint32_t wifiConnectedSinceMs = 0;
uint32_t wifiOfflineSinceMs = 0;
uint32_t lastReconnectMs = 0;
uint32_t lowMemorySinceMs = 0;
uint32_t maxBlockMin = UINT32_MAX;

const char *resetReasonText(esp_reset_reason_t r) {
  switch (r) {
  case ESP_RST_UNKNOWN: return "Unknown";
  case ESP_RST_POWERON: return "Power on";
  case ESP_RST_EXT: return "External reset pin";
  case ESP_RST_SW: return "Software restart";
  case ESP_RST_PANIC: return "Crash (exception or abort)";
  case ESP_RST_INT_WDT: return "Interrupt watchdog (hang)";
  case ESP_RST_TASK_WDT: return "Task watchdog (hang)";
  case ESP_RST_WDT: return "Watchdog (hang)";
  case ESP_RST_DEEPSLEEP: return "Wake from deep sleep";
  case ESP_RST_BROWNOUT: return "Brownout (supply voltage dropped)";
  case ESP_RST_SDIO: return "SDIO reset";
  case ESP_RST_USB: return "USB reset";
  case ESP_RST_JTAG: return "JTAG reset";
  case ESP_RST_EFUSE: return "eFuse error";
  case ESP_RST_PWR_GLITCH: return "Power glitch";
  case ESP_RST_CPU_LOCKUP: return "CPU lockup";
  default: return "Unknown";
  }
}

void readCrashReport() {
  Preferences prefs;
  if (prefs.begin("health", false)) {
    crashCount = prefs.getUInt("crashes", 0);
    crashSummary = prefs.getString("crash", "");
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH && CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
    if (esp_core_dump_image_check() == ESP_OK) {
      esp_core_dump_summary_t *s = new (std::nothrow) esp_core_dump_summary_t;
      if (s && esp_core_dump_get_summary(s) == ESP_OK) {
        char buf[300];
        int n = snprintf(buf, sizeof(buf),
                         "task %.16s, PC 0x%08lx, cause %lu, addr 0x%08lx, "
                         "after %lus uptime, backtrace",
                         s->exc_task, (unsigned long)s->exc_pc,
                         (unsigned long)s->ex_info.exc_cause,
                         (unsigned long)s->ex_info.exc_vaddr,
                         (unsigned long)(havePrevious ? previous.uptimeS : 0));
        for (uint32_t i = 0; i < s->exc_bt_info.depth && i < 16 &&
                             n < (int)sizeof(buf) - 12;
             i++) {
          n += snprintf(buf + n, sizeof(buf) - n, " 0x%08lx",
                        (unsigned long)s->exc_bt_info.bt[i]);
        }
        if (s->exc_bt_info.corrupted && n < (int)sizeof(buf) - 12)
          snprintf(buf + n, sizeof(buf) - n, " |corrupted");
        crashSummary = buf;
        crashCount++;
        prefs.putString("crash", crashSummary);
        prefs.putUInt("crashes", crashCount);
      }
      delete s;
      // Report each crash once; the summary stays in NVS.
      esp_core_dump_image_erase();
    }
#endif
    prefs.end();
  }
}

void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    wifiDisconnects++;
    lastDisconnectReason = info.wifi_sta_disconnected.reason;
  }
}

} // namespace

void begin() {
  resetReason = esp_reset_reason();
  bool rtcValid =
      rtcRun.magic == kRtcMagic && rtcRun.check == rtcCheck(rtcRun) &&
      resetReason != ESP_RST_POWERON && resetReason != ESP_RST_BROWNOUT;
  if (rtcValid) {
    previous = rtcRun;
    previous.restartCause[sizeof(previous.restartCause) - 1] = '\0';
    havePrevious = true;
  }
  memset(&rtcRun, 0, sizeof(rtcRun));
  rtcRun.magic = kRtcMagic;
  rtcRun.check = rtcCheck(rtcRun);

  readCrashReport();
  WiFi.onEvent(onWifiEvent);
}

void restart(const char *cause) {
  strncpy(rtcRun.restartCause, cause, sizeof(rtcRun.restartCause) - 1);
  rtcRun.check = rtcCheck(rtcRun);
  taskLog.printf("Restarting: %s", cause);
  delay(200);
  ESP.restart();
  while (true) {
  }
}

void loop(bool paused) {
  uint32_t now = millis();

  static uint32_t lastBeat = 0;
  if (now - lastBeat >= 1000) {
    lastBeat = now;
    uint32_t block = ESP.getMaxAllocHeap();
    maxBlockMin = std::min(maxBlockMin, block);
    rtcRun.uptimeS = now / 1000;
    rtcRun.heapMin = ESP.getMinFreeHeap();
    rtcRun.maxBlockMin = maxBlockMin;
    rtcRun.wifiDisconnects = wifiDisconnects;
    rtcRun.check = rtcCheck(rtcRun);

    // Memory: a fragmented heap starves the web server; restart cleanly
    // instead of limping on.
    if (block < kLowMemoryBlock) {
      if (lowMemorySinceMs == 0)
        lowMemorySinceMs = now;
      else if (!paused && now - lowMemorySinceMs > kLowMemoryRestartAfterMs)
        restart("Low memory");
    } else {
      lowMemorySinceMs = 0;
    }
  }

  // WiFi: nudge a reconnect, and restart if WiFi stays gone (after a restart,
  // the boot-time connect retries the saved network). Never while someone
  // uses the setup portal.
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiOfflineSinceMs != 0 || wifiConnectedSinceMs == 0) {
      wifiConnectedSinceMs = now;
      if (wifiOfflineSinceMs != 0)
        taskLog.printf("WiFi back after %lus",
                       (unsigned long)((now - wifiOfflineSinceMs) / 1000));
    }
    wifiOfflineSinceMs = 0;
    return;
  }
  if (wifiOfflineSinceMs == 0)
    wifiOfflineSinceMs = now == 0 ? 1 : now;
  bool portalInUse =
      (WiFi.getMode() & WIFI_MODE_AP) && WiFi.softAPgetStationNum() > 0;
  if (paused || portalInUse)
    return;
  uint32_t offline = now - wifiOfflineSinceMs;
  if (offline > kWifiRestartAfterMs)
    restart("WiFi lost for 10 minutes");
  if (offline > kWifiReconnectAfterMs &&
      now - lastReconnectMs > kWifiReconnectEveryMs) {
    lastReconnectMs = now;
    WiFi.reconnect();
  }
}

void toJson(JsonObject o) {
  o["uptime_s"] = millis() / 1000;
  o["reset_reason"] = resetReasonText(resetReason);
  if (havePrevious) {
    JsonObject p = o["previous_run"].to<JsonObject>();
    p["uptime_s"] = previous.uptimeS;
    p["heap_min"] = previous.heapMin;
    if (previous.maxBlockMin != UINT32_MAX)
      p["heap_max_block_min"] = previous.maxBlockMin;
    p["wifi_disconnects"] = previous.wifiDisconnects;
    if (previous.restartCause[0])
      p["restart_cause"] = previous.restartCause;
  }
  if (crashCount > 0) {
    o["crash_count"] = crashCount;
    o["last_crash"] = crashSummary;
  }
  o["heap_free"] = ESP.getFreeHeap();
  o["heap_min"] = ESP.getMinFreeHeap();
  o["heap_max_block"] = ESP.getMaxAllocHeap();
  if (maxBlockMin != UINT32_MAX)
    o["heap_max_block_min"] = maxBlockMin;
  JsonObject w = o["wifi"].to<JsonObject>();
  w["connected"] = WiFi.status() == WL_CONNECTED;
  if (WiFi.status() == WL_CONNECTED) {
    w["rssi"] = WiFi.RSSI();
    w["connected_s"] = (millis() - wifiConnectedSinceMs) / 1000;
  }
  w["disconnects"] = wifiDisconnects;
  if (lastDisconnectReason)
    w["last_disconnect_reason"] = lastDisconnectReason;
}

} // namespace health
