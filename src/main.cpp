#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ElegantOTA.h>
#include <LittleFS.h>
#include <NetWizard.h>
#include <WebSerial.h>
#include <WiFiUdp.h>

#include <map>
#include <memory>
#include <mutex>

#include "TigoProtocol.h"
#include "Version.h"
#include "config.h"

// Increase to help prevent overflow (CRC errors result).
constexpr size_t kRxBufferSize = 1024;

WiFiUDP udp;
TigoParser parser;
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

void setup() {
  Serial.begin(115200);
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW); // Set to RX mode for sniffing
  Serial1.setRxBufferSize(kRxBufferSize);
  Serial1.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN);

  // Setup Sniffing Pin (if required by hardware, e.g. RS485 RE/DE)
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW); // RX Mode

  Serial.println("Starting Tigo Sniffer...");

  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
  }

  // Setup NetWizard (WiFi Manager)
#ifdef HOSTNAME
  netWizard.setHostname(HOSTNAME);
#endif
  netWizard.autoConnect("TigoTell", "");

  // mDNS
#ifdef HOSTNAME
  if (!MDNS.begin(HOSTNAME)) {
    Serial.println("Error setting up MDNS responder!");
  }
  Serial.printf("mDNS responder started at %s.local\n", HOSTNAME);
#endif

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

  // Serve static files from LittleFS
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // Debug Dump endpoint
  server.on("/version", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response = "Version: " + String(VERSION) + "\n";
    response += "Build Timestamp: " + String(BUILD_TIMESTAMP) + "\n";
    request->send(200, "text/plain", response);
  });

  // Setup ElegantOTA
  ElegantOTA.begin(&server, OTA_USER, OTA_PASS);
  server.begin();
}

void loop() {
  netWizard.loop();
  ElegantOTA.loop();

  // Read from Serial1 and feed the parser in batches
  size_t available = Serial1.available();
  if (available > 0) {
    uint8_t serialBuf[128];
    size_t bytesToRead = std::min(available, sizeof(serialBuf));
    size_t count = Serial1.readBytes(serialBuf, bytesToRead);
    if (count > 0) {
      std::lock_guard<std::mutex> lock(dataMutex);
      parser.pushBytes(serialBuf, count);
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
        bool shouldSend = false;
        if (lastUpdateTimes.find(powerFrame->pv_node_id) ==
            lastUpdateTimes.end()) {
          shouldSend = true;
        } else {
          if (currentTime - lastUpdateTimes[powerFrame->pv_node_id] >
              MIN_SECONDS_BETWEEN_NODE_UPDATES * 1000) {
            shouldSend = true;
          }
        }

        if (shouldSend && WiFi.status() == WL_CONNECTED) {
          std::string lineProtocol =
              powerFrame->toInfluxLineProtocol(MEASUREMENT_NAME);

          // Send over UDP
          udp.beginPacket(kInfluxHost, kInfluxPort);
          udp.print(lineProtocol.c_str());
          udp.endPacket();

          // Debug output
          WebSerial.print("Sent: ");
          WebSerial.println(lineProtocol.c_str());

          lastUpdateTimes[powerFrame->pv_node_id] = currentTime;
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
              MIN_SECONDS_BETWEEN_NODE_UPDATES * 1000) {
            shouldSend = true;
          }
        }

        if (shouldSend && WiFi.status() == WL_CONNECTED) {
          std::string lineProtocol =
              announceFrame->toInfluxLineProtocol(MEASUREMENT_NAME);

          // Send over UDP
          udp.beginPacket(kInfluxHost, kInfluxPort);
          udp.print(lineProtocol.c_str());
          udp.endPacket();

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
          std::string lineProtocol =
              nodeTableFrame->toInfluxLineProtocol(MEASUREMENT_NAME);

          // Send over UDP
          udp.beginPacket(kInfluxHost, kInfluxPort);
          udp.print(lineProtocol.c_str());
          udp.endPacket();

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
  if (millis() - lastStatsTime > PUSH_STATS_INTERVAL_IN_S * 1000 &&
      WiFi.status() == WL_CONNECTED) {
    lastStatsTime = millis();
    std::string statsLine =
        parser.getStatisticsInfluxLineProtocol(MEASUREMENT_NAME);

    udp.beginPacket(kInfluxHost, kInfluxPort);
    udp.print(statsLine.c_str());
    udp.endPacket();

    WebSerial.print("Sent Stats: ");
    WebSerial.println(statsLine.c_str());
  }

  // Small yield to keep watchdog happy
  delay(1);
}
