#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ElegantOTA.h>
#include <LittleFS.h>
#include <NetWizard.h>
#include <WebSerial.h>

#include <limits>
#include <map>
#include <memory>
#include <mutex>

#include "PowerStatsAccumulator.h"
#include "TigoProtocol.h"
#include "Version.h"
#include <GithubReleaseOTA.h>
#include <Preferences.h>
#include <esp_wifi.h>

// Increase to help prevent overflow (CRC errors result).
constexpr size_t kRxBufferSize = 1024;

AsyncUDP udp;
TigoParser parser;

struct Config {
  String hostname = "TigoTell";
  int txPin = 17;
  int rxPin = 18;
  int enPin = 21;
  String influxHost = "";
  uint16_t influxPort = 0;
  int pushStatsInterval = 60;
  String measurementName = "tigotell_v0";
  int minSecondsNodeUpdates = 10;
  int minSecondsNodeTableUpdates = 60;
  String otaUser = "...";
  String otaPass = "...";
};

Config appConfig;
Preferences preferences;
String fsVersion = "unknown";

void loadSettings() {
  preferences.begin("tigotell", true);
  appConfig.hostname = preferences.getString("hostname", appConfig.hostname);
  appConfig.txPin = preferences.getInt("txPin", appConfig.txPin);
  appConfig.rxPin = preferences.getInt("rxPin", appConfig.rxPin);
  appConfig.enPin = preferences.getInt("enPin", appConfig.enPin);
  appConfig.influxHost =
      preferences.getString("influxHost", appConfig.influxHost);
  appConfig.influxPort =
      preferences.getUShort("influxPort", appConfig.influxPort);
  appConfig.pushStatsInterval =
      preferences.getInt("pushStats", appConfig.pushStatsInterval);
  appConfig.measurementName =
      preferences.getString("measName", appConfig.measurementName);
  appConfig.minSecondsNodeUpdates =
      preferences.getInt("nodeUpd", appConfig.minSecondsNodeUpdates);
  appConfig.minSecondsNodeTableUpdates =
      preferences.getInt("tableUpd", appConfig.minSecondsNodeTableUpdates);
  appConfig.otaUser = preferences.getString("otaUser", appConfig.otaUser);
  appConfig.otaPass = preferences.getString("otaPass", appConfig.otaPass);
  preferences.end();
}

void saveSettings() {
  preferences.begin("tigotell", false);
  preferences.putString("hostname", appConfig.hostname);
  preferences.putInt("txPin", appConfig.txPin);
  preferences.putInt("rxPin", appConfig.rxPin);
  preferences.putInt("enPin", appConfig.enPin);
  preferences.putString("influxHost", appConfig.influxHost);
  preferences.putUShort("influxPort", appConfig.influxPort);
  preferences.putInt("pushStats", appConfig.pushStatsInterval);
  preferences.putString("measName", appConfig.measurementName);
  preferences.putInt("nodeUpd", appConfig.minSecondsNodeUpdates);
  preferences.putInt("tableUpd", appConfig.minSecondsNodeTableUpdates);
  preferences.putString("otaUser", appConfig.otaUser);
  preferences.putString("otaPass", appConfig.otaPass);
  preferences.end();
}

void sendUdpPayload(const std::string &payload) {
  if (payload.empty() || WiFi.status() != WL_CONNECTED ||
      appConfig.influxHost.length() == 0 || appConfig.influxPort == 0) {
    return;
  }
  udp.writeTo(reinterpret_cast<const uint8_t *>(payload.c_str()),
              payload.length(), appConfig.influxHost.c_str(),
              appConfig.influxPort);
}

volatile bool isUpdatingOTA = false;

void onOTAStart() {
  isUpdatingOTA = true;
  WebSerial.println("OTA Update Started. Pausing reads from Tigo...");
}

void onOTAEnd(bool success) {
  isUpdatingOTA = false;
  if (success) {
    WebSerial.println("OTA Update Successful!");
  } else {
    WebSerial.println("OTA Update Failed!");
  }
}

void runOtaUpdate(void *parameter) {
  onOTAStart();
  GithubReleaseOTA ota("gongloo", "TigoTell");
  GithubRelease release = ota.getLatestRelease();
  if (release.tag_name != NULL) {
    // Normalize versions for comparison
    String latest = String(release.tag_name);
    if (latest.startsWith("v"))
      latest = latest.substring(1);
    String fwCurrent = String(VERSION);

    bool fwNeedsUpdate = (fwCurrent != latest);
    bool fsNeedsUpdate = (fsVersion != latest);

    WebSerial.printf("Versions - Current FW: %s, FS: %s | Latest: %s\n",
                     fwCurrent.c_str(), fsVersion.c_str(), latest.c_str());

    if (fwNeedsUpdate || fsNeedsUpdate) {
      // 1. Unmount filesystem for safety
      WebSerial.printf("Starting selective GitHub Update to v%s...",
                       latest.c_str());
      delay(100);
      yield();

      // 2. Flash Firmware if out of date (Priority)
      if (fwNeedsUpdate) {
        if (ota.flashFirmware(release, "firmware.bin") == OTA_SUCCESS) {
          WebSerial.println("Firmware updated successfully.");
        } else {
          WebSerial.println("Firmware update FAILED or asset missing.");
        }
        delay(500);
        yield();
      }

      // 3. Flash FileSystem if out of date
      if (fsNeedsUpdate) {
        LittleFS.end();
        if (ota.flashSpiffs(release, "littlefs.bin") == OTA_SUCCESS) {
          WebSerial.println("FileSystem updated successfully.");
        } else {
          WebSerial.println("FileSystem update FAILED or asset missing.");
        }
        delay(500);
        yield();
      }

      ota.freeRelease(release);
      onOTAEnd(true);
      WebSerial.println("Update sequence complete. Rebooting...");
      delay(3000);
      ESP.restart();
    } else {
      WebSerial.println("Everything is already up to date.");
      ota.freeRelease(release);
      onOTAEnd(true);
    }
  } else {
    WebSerial.println("Failed to fetch latest release for update.");
    onOTAEnd(false);
  }
  vTaskDelete(NULL);
}

AsyncWebServer server(80);
NetWizard netWizard(&server);

unsigned long lastStatsTime = 0;

// Store the latest power frame and timestamp for each node
struct PowerNodeData {
  std::shared_ptr<TigoPowerFrame> frame;
  uint64_t last_updated;
};

std::map<uint16_t, PowerNodeData> nodePowerData;

struct NodeInfo {
  uint16_t address = 0;
  uint16_t pv_node_id = 0;
  std::string barcode = "";
  uint32_t barcode_fragment = 0;
  uint64_t last_seen = 0;
};

// Persistent registry of all discovered nodes
std::map<uint16_t, NodeInfo> persistentNodeTable;

// Store the last update time for each node
std::map<uint16_t, unsigned long> lastUpdateTimes;
// Store the last update time for each announced address
std::map<uint16_t, unsigned long> lastUpdateTimesAnnounce;

std::mutex dataMutex;

std::map<uint16_t, PowerStatsAccumulator> nodeAccumulators;
bool isElegantOtaStarted = false;

void setup() {
  loadSettings();

  Serial.begin(115200);
  pinMode(appConfig.enPin, OUTPUT);
  digitalWrite(appConfig.enPin, LOW); // Set to RX mode for sniffing
  Serial1.setRxBufferSize(kRxBufferSize);
  Serial1.begin(38400, SERIAL_8N1, appConfig.rxPin, appConfig.txPin);

  Serial.println("Starting Tigo Sniffer...");

  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
  } else {
    // Read filesystem version
    if (LittleFS.exists("/version")) {
      File f = LittleFS.open("/version", "r");
      if (f) {
        fsVersion = f.readString();
        fsVersion.trim();
        f.close();
        Serial.printf("FileSystem Version: %s\n", fsVersion.c_str());
      }
    }
  }

  // Setup NetWizard (WiFi Manager)
  netWizard.setHostname(appConfig.hostname.c_str());
  netWizard.autoConnect("TigoTell", "");

  // mDNS
  if (!MDNS.begin(appConfig.hostname.c_str())) {
    Serial.println("Error setting up MDNS responder!");
  }
  Serial.printf("mDNS responder started at %s.local\n",
                appConfig.hostname.c_str());

  // Setup WebSerial
  WebSerial.begin(&server);

  // JSON Dump endpoint
  server.on("/json", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();

    {
      std::lock_guard<std::mutex> lock(dataMutex);

      // Parser Statistics
      const auto &stats = parser.getStatistics();
      JsonObject jsonStats = root["stats"].to<JsonObject>();
      jsonStats["total_frames"] = stats.totalPhysicalFrames;
      jsonStats["crc_errors"] = stats.crcErrors;
      jsonStats["power_count"] = stats.powerDataCount;
      jsonStats["announce_count"] = stats.announceCount;
      jsonStats["nodetable_count"] = stats.nodeTableCount;
      jsonStats["management_count"] = stats.managementCount;

      // Platform/Software Info
      root["version"] = VERSION;
      root["fs_version"] = fsVersion;
      root["build_timestamp"] = BUILD_TIMESTAMP;
      root["uptime_ms"] = esp_timer_get_time() / 1000;

      // Power Data (Latest Metrics)
      JsonArray powerNodes = root["power"].to<JsonArray>();
      for (auto const &[key, val] : nodePowerData) {
        JsonObject node = powerNodes.add<JsonObject>();
        auto frame = val.frame;
        node["address"] = frame->address;
        node["pv_node_id"] = frame->pv_node_id;
        node["voltage_in"] = frame->voltage_in;
        node["voltage_out"] = frame->voltage_out;
        node["current_in"] = frame->current_in;
        node["temperature"] = frame->temperature;
        node["duty_cycle"] = frame->duty_cycle;
        node["rssi"] = frame->rssi;
        node["last_updated"] = val.last_updated;
      }

      // Discovery Data (Persistent Map)
      JsonArray discoverNodes = root["nodes"].to<JsonArray>();
      for (auto const &[key, val] : persistentNodeTable) {
        JsonObject node = discoverNodes.add<JsonObject>();
        node["address"] = val.address;
        node["pv_node_id"] = val.pv_node_id;
        if (!val.barcode.empty()) {
          node["barcode"] = val.barcode.c_str();
        }
        if (val.barcode_fragment != 0) {
          node["barcode_fragment"] = val.barcode_fragment;
        }
        if (val.last_seen != 0) {
          node["last_seen_ms"] = val.last_seen;
        }
      }
    }

    AsyncResponseStream *response =
        request->beginResponseStream("application/json");
    serializeJson(doc, *response);
    request->send(response);
  });

  // Config GET endpoint
  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["hostname"] = appConfig.hostname;
    doc["txPin"] = appConfig.txPin;
    doc["rxPin"] = appConfig.rxPin;
    doc["enPin"] = appConfig.enPin;
    doc["influxHost"] = appConfig.influxHost;
    doc["influxPort"] = appConfig.influxPort;
    doc["pushStatsInterval"] = appConfig.pushStatsInterval;
    doc["measurementName"] = appConfig.measurementName;
    doc["minSecondsNodeUpdates"] = appConfig.minSecondsNodeUpdates;
    doc["minSecondsNodeTableUpdates"] = appConfig.minSecondsNodeTableUpdates;

    AsyncResponseStream *response =
        request->beginResponseStream("application/json");
    serializeJson(doc, *response);
    request->send(response);
  });

  // Config POST endpoint
  server.on(
      "/api/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
         size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (!error) {
          if (doc["hostname"].is<const char *>())
            appConfig.hostname = doc["hostname"].as<String>();
          if (doc["txPin"].is<int>())
            appConfig.txPin = doc["txPin"];
          if (doc["rxPin"].is<int>())
            appConfig.rxPin = doc["rxPin"];
          if (doc["enPin"].is<int>())
            appConfig.enPin = doc["enPin"];
          if (doc["influxHost"].is<const char *>())
            appConfig.influxHost = doc["influxHost"].as<String>();
          if (doc["influxPort"].is<int>())
            appConfig.influxPort = doc["influxPort"];
          if (doc["pushStatsInterval"].is<int>())
            appConfig.pushStatsInterval = doc["pushStatsInterval"];
          if (doc["measurementName"].is<const char *>())
            appConfig.measurementName = doc["measurementName"].as<String>();
          if (doc["minSecondsNodeUpdates"].is<int>())
            appConfig.minSecondsNodeUpdates = doc["minSecondsNodeUpdates"];
          if (doc["minSecondsNodeTableUpdates"].is<int>())
            appConfig.minSecondsNodeTableUpdates =
                doc["minSecondsNodeTableUpdates"];
          if (doc["otaUser"].is<const char *>()) {
            String val = doc["otaUser"].as<String>();
            if (val.length() > 0)
              appConfig.otaUser = val;
          }
          if (doc["otaPass"].is<const char *>()) {
            String val = doc["otaPass"].as<String>();
            if (val.length() > 0)
              appConfig.otaPass = val;
          }

          saveSettings();
          request->send(200, "text/plain", "OK");
        } else {
          request->send(400, "text/plain", "Invalid JSON");
        }
      });

  // Reboot endpoint
  server.on("/api/reboot", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Rebooting...");
    // Let the response finish before rebooting
    request->onDisconnect([]() {
      delay(500);
      ESP.restart();
    });
  });

  // Serve static files from LittleFS
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // GitHub OTA Endpoints
  server.on("/api/update/check", HTTP_GET, [](AsyncWebServerRequest *request) {
    // We do the check in-request, which might be slow.
    // GitHub API can take a second or two.
    GithubReleaseOTA ota("gongloo", "TigoTell");

    // Optional: Set insecure if certificates are a pain, but let's try defaults
    // first ota.setInsecure();

    GithubRelease release = ota.getLatestRelease();
    JsonDocument doc;

    if (release.tag_name != NULL) {
      // Consolidate update status
      String latest = String(release.tag_name);
      if (latest.startsWith("v"))
        latest = latest.substring(1);
      doc["latest_version"] = latest;
      String current = String(VERSION);

      WebSerial.printf(
          "[UpdateCheck] FW Current: %s, FS Current: %s, Github Latest: %s\n",
          current.c_str(), fsVersion.c_str(), latest.c_str());

      doc["update_available"] = (latest != current || latest != fsVersion);
    } else {
      doc["error"] = "Failed to fetch release info";
    }
    ota.freeRelease(release);

    AsyncResponseStream *response =
        request->beginResponseStream("application/json");
    serializeJson(doc, *response);
    request->send(response);
  });

  server.on("/api/update/execute", HTTP_POST,
            [](AsyncWebServerRequest *request) {
              request->send(200, "text/plain", "Update started...");
              // Run update in background task to avoid blocking the server
              // request
              xTaskCreate(runOtaUpdate, "ota_task", 16384, NULL, 1, NULL);
            });

  // Debug Dump endpoint
  server.on("/version", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response = "Version: " + String(VERSION) + "\n";
    response += "Build Timestamp: " + String(BUILD_TIMESTAMP) + "\n";
    request->send(200, "text/plain", response);
  });

  // Setup ElegantOTA
  if (appConfig.otaUser.length() > 0 && appConfig.otaUser != "..." &&
      appConfig.otaPass.length() > 0 && appConfig.otaPass != "...") {
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onEnd(onOTAEnd);
    ElegantOTA.begin(&server, appConfig.otaUser.c_str(),
                     appConfig.otaPass.c_str());
    isElegantOtaStarted = true;
    Serial.println("ElegantOTA started with authentication.");
  } else {
    isElegantOtaStarted = false;
    Serial.println("ElegantOTA disabled (no credentials).");
  }
  server.begin();
}

void loop() {
  netWizard.loop();
  if (isElegantOtaStarted) {
    ElegantOTA.loop();
  }

  if (!isUpdatingOTA) {
    // Actively enforce WiFi sleep mode. Often, internal TCP/IP reconnections
    // or NetWizard loops will implicitly re-enable 802.11 power saving.
    static unsigned long lastWifiCheck = 0;
    if (millis() - lastWifiCheck > 5000) {
      lastWifiCheck = millis();
      if (WiFi.status() == WL_CONNECTED) {
        esp_wifi_set_ps(WIFI_PS_NONE);       // The deep IDF-level disable
        WiFi.setTxPower(WIFI_POWER_19_5dBm); // Restore to max for stability
      }
    }

    // Read from Serial1 and feed the parser in batches
    size_t available = Serial1.available();
    if (available > 0) {
      uint8_t serialBuf[128];
      size_t bytesToRead = std::min(available, sizeof(serialBuf));
      size_t count = Serial1.read(
          serialBuf,
          bytesToRead); // Direct FIFO extraction, skips Stream::timedRead()
      if (count > 0) {
        std::lock_guard<std::mutex> lock(dataMutex);
        parser.pushBytes(serialBuf, count);
      }
    }
  }

  // Process parsed frames
  while (parser.hasFrames()) {
    std::unique_ptr<TigoFrame> frame = parser.popFrame();

    switch (frame->getType()) {
    case TigoFrameType::PowerData: {
      TigoPowerFrame *powerFrame = static_cast<TigoPowerFrame *>(frame.get());
      if (powerFrame) {
        unsigned long currentTime = millis();

        // Add frame to accumulator
        nodeAccumulators[powerFrame->pv_node_id].add(powerFrame);

        bool shouldSend = false;
        if (lastUpdateTimes.find(powerFrame->pv_node_id) ==
            lastUpdateTimes.end()) {
          shouldSend = true;
        } else {
          if (currentTime - lastUpdateTimes[powerFrame->pv_node_id] >
              static_cast<unsigned long>(appConfig.minSecondsNodeUpdates) *
                  1000) {
            shouldSend = true;
          }
        }

        if (shouldSend && WiFi.status() == WL_CONNECTED) {
          if (nodeAccumulators[powerFrame->pv_node_id].hasSamples()) {
            std::string lineProtocol =
                nodeAccumulators[powerFrame->pv_node_id].toInfluxLineProtocol(
                    appConfig.measurementName.c_str(), powerFrame->address,
                    powerFrame->pv_node_id);

            // Send over UDP
            sendUdpPayload(lineProtocol);

            // Debug output
            WebSerial.print("Sent: ");
            WebSerial.println(lineProtocol.c_str());
          }

          lastUpdateTimes[powerFrame->pv_node_id] = currentTime;
          nodeAccumulators[powerFrame->pv_node_id].reset();
        }

        // Store the latest power frame for JSON endpoint
        {
          std::lock_guard<std::mutex> lock(dataMutex);
          nodePowerData[powerFrame->pv_node_id] = {
              std::shared_ptr<TigoPowerFrame>(
                  static_cast<TigoPowerFrame *>(frame.release())),
              static_cast<uint64_t>(esp_timer_get_time() / 1000)};
        }
      }
      break;
    }
    case TigoFrameType::Announce: {
      TigoAnnounceFrame *announceFrame =
          static_cast<TigoAnnounceFrame *>(frame.get());
      if (announceFrame) {
        unsigned long currentTime = millis();
        bool shouldSend = false;
        if (lastUpdateTimesAnnounce.find(announceFrame->pv_node_id) ==
            lastUpdateTimesAnnounce.end()) {
          shouldSend = true;
        } else {
          if (currentTime - lastUpdateTimesAnnounce[announceFrame->pv_node_id] >
              static_cast<unsigned long>(appConfig.minSecondsNodeUpdates) *
                  1000) {
            shouldSend = true;
          }
        }

        if (shouldSend && WiFi.status() == WL_CONNECTED) {
          std::string lineProtocol = announceFrame->toInfluxLineProtocol(
              appConfig.measurementName.c_str());

          // Send over UDP
          sendUdpPayload(lineProtocol);

          // Debug output
          WebSerial.print("Sent Announce: ");
          WebSerial.println(lineProtocol.c_str());

          lastUpdateTimesAnnounce[announceFrame->pv_node_id] = currentTime;
        }

        // Update persistent node table
        {
          std::lock_guard<std::mutex> lock(dataMutex);
          NodeInfo &info = persistentNodeTable[announceFrame->pv_node_id];
          info.address = announceFrame->address;
          info.pv_node_id = announceFrame->pv_node_id;
          info.barcode_fragment = announceFrame->barcode_fragment;
          info.last_seen = esp_timer_get_time() / 1000;
        }
      }
      break;
    }
    case TigoFrameType::NodeTable: {
      TigoNodeTableFrame *nodeTableFrame =
          static_cast<TigoNodeTableFrame *>(frame.get());
      if (nodeTableFrame) {
        unsigned long currentTime = millis();

        if (WiFi.status() == WL_CONNECTED) {
          std::string lineProtocol = nodeTableFrame->toInfluxLineProtocol(
              appConfig.measurementName.c_str());

          // Send over UDP
          sendUdpPayload(lineProtocol);

          // Debug output
          WebSerial.print("Sent NodeTable: ");
          WebSerial.println(lineProtocol.c_str());
        }

        // Update persistent node table with all records from this frame
        {
          std::lock_guard<std::mutex> lock(dataMutex);
          for (const auto &entry : nodeTableFrame->nodes) {
            NodeInfo &info = persistentNodeTable[entry.pv_node_id];
            info.pv_node_id = entry.pv_node_id;
            info.barcode = entry.barcode;
          }
        }
      }
      break;
    }
    }
  }

  // Send Statistics periodically
  if (millis() - lastStatsTime >
          static_cast<unsigned long>(appConfig.pushStatsInterval) * 1000 &&
      WiFi.status() == WL_CONNECTED) {
    lastStatsTime = millis();
    std::string statsLine = parser.getStatisticsInfluxLineProtocol(
        appConfig.measurementName.c_str());
    statsLine +=
        ",uptime=" + std::to_string(esp_timer_get_time() / 1000000) + "i";

    sendUdpPayload(statsLine);

    WebSerial.print("Sent Stats: ");
    WebSerial.println(statsLine.c_str());
  }
}
