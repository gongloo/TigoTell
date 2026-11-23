#include "Health.h"

#include "TaskLog.h"

#include <Preferences.h>
#include <WebSerial.h>
#include <WiFi.h>
#include <atomic>
#include <esp_core_dump.h>
#include <esp_system.h>
#include <esp_timer.h>

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
  uint32_t h = r.magic ^ r.uptimeS ^ (r.heapMin * 31) ^ (r.maxBlockMin * 17) ^
               (r.wifiDisconnects * 7) ^ 0x5A5A5A5A;
  for (unsigned char c : r.restartCause)
    h = (h * 33) ^ c;
  return h;
}

// Snapshot taken at boot.
esp_reset_reason_t resetReason;
bool havePrevious = false;
RtcRun previous;
String crashSummary; // most recent crash, persisted in NVS
uint32_t crashCount = 0;

// Runtime state.
std::atomic<uint32_t> wifiDisconnects{0};
std::atomic<uint8_t> lastDisconnectReason{0};
std::atomic<uint64_t> wifiConnectedSinceMs{0};
uint64_t wifiOfflineSinceMs = 0;
uint64_t lastReconnectMs = 0;
uint64_t lowMemorySinceMs = 0;
std::atomic<uint32_t> maxBlockMin{UINT32_MAX};

uint64_t nowMs() { return static_cast<uint64_t>(esp_timer_get_time() / 1000); }

uint64_t elapsedMs(uint64_t now, uint64_t since) {
  return now >= since ? now - since : 0;
}

uint32_t updateMaxBlockMin(uint32_t block) {
  uint32_t minBlock =
      std::min(maxBlockMin.load(std::memory_order_relaxed), block);
  maxBlockMin.store(minBlock, std::memory_order_relaxed);
  return minBlock;
}

void snapshotRtc(uint64_t now, uint32_t blockMin) {
  rtcRun.uptimeS = static_cast<uint32_t>(now / 1000);
  rtcRun.heapMin = ESP.getMinFreeHeap();
  rtcRun.maxBlockMin = blockMin;
  rtcRun.wifiDisconnects = wifiDisconnects.load(std::memory_order_relaxed);
  rtcRun.check = rtcCheck(rtcRun);
}

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
    wifiDisconnects.fetch_add(1, std::memory_order_relaxed);
    lastDisconnectReason.store(info.wifi_sta_disconnected.reason,
                               std::memory_order_relaxed);
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
  rtcRun.restartCause[sizeof(rtcRun.restartCause) - 1] = '\0';
  snapshotRtc(nowMs(), updateMaxBlockMin(ESP.getMaxAllocHeap()));
  Serial.printf("Restarting: %s\n", cause);
  taskLog.printf("Restarting: %s", cause);
  taskLog.flush();
  delay(1);
  WebSerial.loop(); // >100 us: _print_buffer -> _buffer
  delay(110);
  WebSerial.loop(); // >100 ms: _buffer -> _ws->binaryAll()
  delay(90);        // allow async_tcp to transmit
  ESP.restart();
  while (true) {
  }
}

void loop(bool paused) {
  uint64_t now = nowMs();

  static uint64_t lastBeat = 0;
  if (elapsedMs(now, lastBeat) >= 1000) {
    lastBeat = now;
    uint32_t block = ESP.getMaxAllocHeap();
    snapshotRtc(now, updateMaxBlockMin(block));

    // Memory: a fragmented heap starves the web server; restart cleanly
    // instead of limping on.
    if (block < kLowMemoryBlock) {
      if (lowMemorySinceMs == 0)
        lowMemorySinceMs = now == 0 ? 1 : now;
      else if (!paused &&
               elapsedMs(now, lowMemorySinceMs) > kLowMemoryRestartAfterMs)
        restart("Low memory");
    } else {
      lowMemorySinceMs = 0;
    }
  }

  // WiFi: nudge a reconnect, and restart if WiFi stays gone (after a restart,
  // the boot-time connect retries the saved network). Never while paused or
  // while SoftAP mode is active.
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiOfflineSinceMs != 0 ||
        wifiConnectedSinceMs.load(std::memory_order_relaxed) == 0) {
      wifiConnectedSinceMs.store(now == 0 ? 1 : now,
                                 std::memory_order_relaxed);
      if (wifiOfflineSinceMs != 0)
        taskLog.printf(
            "WiFi back after %lus",
            (unsigned long)(elapsedMs(now, wifiOfflineSinceMs) / 1000));
    }
    wifiOfflineSinceMs = 0;
    return;
  }
  wifiConnectedSinceMs.store(0, std::memory_order_relaxed);
  bool portalInUse = (WiFi.getMode() & WIFI_MODE_AP) != 0;
  if (paused || portalInUse) {
    wifiOfflineSinceMs = 0;
    return;
  }
  if (wifiOfflineSinceMs == 0)
    wifiOfflineSinceMs = now == 0 ? 1 : now;
  uint64_t offline = elapsedMs(now, wifiOfflineSinceMs);
  if (offline > kWifiRestartAfterMs)
    restart("WiFi lost for 10 minutes");
  if (offline > kWifiReconnectAfterMs &&
      elapsedMs(now, lastReconnectMs) > kWifiReconnectEveryMs) {
    lastReconnectMs = now;
    WiFi.reconnect();
  }
}

void toJson(JsonObject o) {
  uint64_t now = nowMs();
  o["uptime_s"] = now / 1000;
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
  uint32_t minBlock = maxBlockMin.load(std::memory_order_relaxed);
  if (minBlock != UINT32_MAX)
    o["heap_max_block_min"] = minBlock;
  JsonObject w = o["wifi"].to<JsonObject>();
  bool connected = WiFi.status() == WL_CONNECTED;
  w["connected"] = connected;
  if (connected) {
    w["rssi"] = WiFi.RSSI();
    uint64_t connSince = wifiConnectedSinceMs.load(std::memory_order_relaxed);
    w["connected_s"] =
        (connSince ? elapsedMs(now, connSince) : 0) / 1000;
  }
  w["disconnects"] = wifiDisconnects.load(std::memory_order_relaxed);
  uint8_t disconnectReason =
      lastDisconnectReason.load(std::memory_order_relaxed);
  if (disconnectReason)
    w["last_disconnect_reason"] = disconnectReason;
}

} // namespace health
