#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ElegantOTA.h>
#include <LittleFS.h>
#include <NetWizard.h>
#include <WebSerial.h>

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <new>
#include <string>

#include "Health.h"
#include "RepoInfo.h"
#include "MqttPublisher.h"
#include "TaskLog.h"
#include "PowerStatsAccumulator.h"
#include "TigoMqtt.h"
#include "TigoProtocol.h"
#include "Version.h"
#include <GithubReleaseOTA.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <nvs.h>

// Increase to help prevent overflow (CRC errors result).
constexpr size_t kRxBufferSize = 1024;

AsyncUDP udp;
TigoParser parser;
MqttPublisher mqtt;
TigoMqttPublisher tigoMqtt;

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
  String mqttHost = "";
  uint16_t mqttPort = 1883;
  String mqttUser = "";
  String mqttPass = "";
  String mqttTopic = "tigotell";
  int mqttInterval = 30;
  bool mqttDiscovery = true;
};

Config appConfig;
std::mutex dataMutex;
String fsVersion = "unknown";

void loadSettings() {
  Preferences prefs;
  prefs.begin("tigotell", true);
  appConfig.hostname = prefs.getString("hostname", appConfig.hostname);
  appConfig.txPin = prefs.getInt("txPin", appConfig.txPin);
  appConfig.rxPin = prefs.getInt("rxPin", appConfig.rxPin);
  appConfig.enPin = prefs.getInt("enPin", appConfig.enPin);
  appConfig.influxHost = prefs.getString("influxHost", appConfig.influxHost);
  appConfig.influxPort = prefs.getUShort("influxPort", appConfig.influxPort);
  appConfig.pushStatsInterval =
      prefs.getInt("pushStats", appConfig.pushStatsInterval);
  appConfig.measurementName =
      prefs.getString("measName", appConfig.measurementName);
  appConfig.minSecondsNodeUpdates =
      prefs.getInt("nodeUpd", appConfig.minSecondsNodeUpdates);
  appConfig.minSecondsNodeTableUpdates =
      prefs.getInt("tableUpd", appConfig.minSecondsNodeTableUpdates);
  appConfig.otaUser = prefs.getString("otaUser", appConfig.otaUser);
  appConfig.otaPass = prefs.getString("otaPass", appConfig.otaPass);
  appConfig.mqttHost = prefs.getString("mqttHost", appConfig.mqttHost);
  appConfig.mqttPort = prefs.getUShort("mqttPort", appConfig.mqttPort);
  appConfig.mqttUser = prefs.getString("mqttUser", appConfig.mqttUser);
  appConfig.mqttPass = prefs.getString("mqttPass", appConfig.mqttPass);
  appConfig.mqttTopic = prefs.getString("mqttTopic", appConfig.mqttTopic);
  appConfig.mqttInterval = prefs.getInt("mqttIntv", appConfig.mqttInterval);
  appConfig.mqttDiscovery =
      prefs.getBool("mqttHaDisc", appConfig.mqttDiscovery);
  prefs.end();
}

void saveSettings(const Config &cfg) {
  Preferences prefs;
  prefs.begin("tigotell", false);
  prefs.putString("hostname", cfg.hostname);
  prefs.putInt("txPin", cfg.txPin);
  prefs.putInt("rxPin", cfg.rxPin);
  prefs.putInt("enPin", cfg.enPin);
  prefs.putString("influxHost", cfg.influxHost);
  prefs.putUShort("influxPort", cfg.influxPort);
  prefs.putInt("pushStats", cfg.pushStatsInterval);
  prefs.putString("measName", cfg.measurementName);
  prefs.putInt("nodeUpd", cfg.minSecondsNodeUpdates);
  prefs.putInt("tableUpd", cfg.minSecondsNodeTableUpdates);
  prefs.putString("otaUser", cfg.otaUser);
  prefs.putString("otaPass", cfg.otaPass);
  prefs.putString("mqttHost", cfg.mqttHost);
  prefs.putUShort("mqttPort", cfg.mqttPort);
  prefs.putString("mqttUser", cfg.mqttUser);
  prefs.putString("mqttPass", cfg.mqttPass);
  prefs.putString("mqttTopic", cfg.mqttTopic);
  prefs.putInt("mqttIntv", cfg.mqttInterval);
  prefs.putBool("mqttHaDisc", cfg.mqttDiscovery);
  prefs.end();
}

void sendUdpPayload(const std::string &payload) {
  if (payload.empty() || WiFi.status() != WL_CONNECTED)
    return;
  String host;
  uint16_t port;
  {
    std::lock_guard<std::mutex> lock(dataMutex);
    host = appConfig.influxHost;
    port = appConfig.influxPort;
  }
  if (host.length() == 0 || port == 0)
    return;
  udp.writeTo(reinterpret_cast<const uint8_t *>(payload.c_str()),
              payload.length(), host.c_str(), port);
}

void applyMqttSettings(const Config &cfg) {
  MqttPublisher::Settings s;
  s.host = cfg.mqttHost;
  s.port = cfg.mqttPort;
  s.user = cfg.mqttUser;
  s.pass = cfg.mqttPass;
  s.baseTopic = cfg.mqttTopic;
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char suffix[8];
  snprintf(suffix, sizeof(suffix), "-%02x%02x%02x", mac[3], mac[4], mac[5]);
  s.clientId = cfg.hostname + suffix;
  mqtt.configure(s);
}

volatile bool isUpdatingOTA = false;

void onOTAStart() {
  isUpdatingOTA = true;
  taskLog.println("OTA Update Started. Pausing reads from Tigo...");
}

void onOTAEnd(bool success) {
  isUpdatingOTA = false;
  if (success) {
    taskLog.println("OTA Update Successful!");
  } else {
    taskLog.println("OTA Update Failed!");
  }
}

void runOtaUpdate(void *parameter) {
  onOTAStart();
  GithubReleaseOTA ota(TIGOTELL_REPO_OWNER, TIGOTELL_REPO_NAME);
  GithubRelease release = ota.getLatestRelease();
  if (release.tag_name != NULL) {
    // Normalize versions for comparison
    String latest = String(release.tag_name);
    if (latest.startsWith("v"))
      latest = latest.substring(1);
    String fwCurrent = String(VERSION);

    bool fwNeedsUpdate = (fwCurrent != latest);
    bool fsNeedsUpdate = (fsVersion != latest);

    taskLog.printf("Versions - Current FW: %s, FS: %s | Latest: %s\n",
                   fwCurrent.c_str(), fsVersion.c_str(), latest.c_str());

    if (fwNeedsUpdate || fsNeedsUpdate) {
      // 1. Unmount filesystem for safety
      taskLog.printf("Starting selective GitHub Update to v%s...",
                     latest.c_str());
      delay(100);
      yield();

      // 2. Flash Firmware if out of date (Priority)
      if (fwNeedsUpdate) {
        if (ota.flashFirmware(release, "firmware.bin") == OTA_SUCCESS) {
          taskLog.println("Firmware updated successfully.");
        } else {
          taskLog.println("Firmware update FAILED or asset missing.");
        }
        delay(500);
        yield();
      }

      // 3. Flash FileSystem if out of date
      if (fsNeedsUpdate) {
        LittleFS.end();
        if (ota.flashSpiffs(release, "littlefs.bin") == OTA_SUCCESS) {
          taskLog.println("FileSystem updated successfully.");
        } else {
          taskLog.println("FileSystem update FAILED or asset missing.");
        }
        delay(500);
        yield();
      }

      ota.freeRelease(release);
      onOTAEnd(true);
      taskLog.println("Update sequence complete. Rebooting...");
      delay(3000);
      ESP.restart();
    } else {
      taskLog.println("Everything is already up to date.");
      ota.freeRelease(release);
      onOTAEnd(true);
    }
  } else {
    taskLog.println("Failed to fetch latest release for update.");
    onOTAEnd(false);
  }
  vTaskDelete(NULL);
}

AsyncWebServer server(80);
NetWizard netWizard(&server);

unsigned long lastStatsTime = 0;
unsigned long lastMqttTime = 0;

// Nodes without a power frame for this long are not published (matches the
// dashboard's stale threshold).
constexpr uint64_t kMqttStaleMs = 60000;

// Store the latest power frame and timestamp for each node
struct PowerNodeData {
  std::shared_ptr<TigoPowerFrame> frame;
  uint64_t last_updated;
};

std::map<uint16_t, PowerNodeData> nodePowerData;

struct NodeInfo {
  uint16_t address = 0;
  uint16_t pv_node_id = 0;
  std::string barcode = "";      // Tigo format, e.g. "4-EF82C4J"
  std::string long_address = ""; // 16 hex digits from the node table
  uint32_t barcode_fragment = 0;
  uint64_t last_seen = 0;
};

// Persistent registry of all discovered nodes
std::map<uint16_t, NodeInfo> persistentNodeTable;

// The CCA sends its node table (module ID -> long address) rarely, mostly
// while modules are being added. Keep the mapping in NVS so it survives
// restarts. Format: version byte, then per node 2 bytes ID + 8 bytes address.
constexpr size_t kNodeMapMaxNodes = 200;

void saveNodeMap(const std::vector<uint8_t> &blob) {
  Preferences prefs; // own handle: loop() and web handlers may overlap
  if (!prefs.begin("nodemap", false))
    return;
  bool ok = prefs.putBytes("v1", blob.data(), blob.size()) == blob.size();
  prefs.end();
  taskLog.printf("Node map %s: %u modules", ok ? "saved" : "NOT saved",
                 (unsigned)((blob.size() - 1) / 10));
}

void loadNodeMap() {
  Preferences prefs;
  if (!prefs.begin("nodemap", true))
    return;
  size_t len = prefs.getBytesLength("v1");
  std::vector<uint8_t> blob(len);
  if (len > 0)
    prefs.getBytes("v1", blob.data(), len);
  prefs.end();
  if (len < 1 || blob[0] != 1)
    return;
  for (size_t pos = 1; pos + 10 <= len; pos += 10) {
    uint16_t id = (blob[pos] << 8) | blob[pos + 1];
    char hex[17];
    for (int i = 0; i < 8; i++)
      snprintf(hex + i * 2, 3, "%02X", blob[pos + 2 + i]);
    NodeInfo &info = persistentNodeTable[id];
    info.pv_node_id = id;
    info.long_address = hex;
    info.barcode = tigoBarcode(&blob[pos + 2]);
  }
}

// Builds the blob from the registry; call with dataMutex held.
std::vector<uint8_t> nodeMapBlob() {
  std::vector<uint8_t> blob{1};
  for (auto const &[id, info] : persistentNodeTable) {
    if (info.long_address.size() != 16)
      continue;
    if ((blob.size() - 1) / 10 >= kNodeMapMaxNodes)
      break;
    blob.push_back(id >> 8);
    blob.push_back(id & 0xFF);
    uint64_t addr = std::strtoull(info.long_address.c_str(), nullptr, 16);
    for (int shift = 56; shift >= 0; shift -= 8)
      blob.push_back((addr >> shift) & 0xFF);
  }
  return blob;
}

// Store the last update time for each node
std::map<uint16_t, unsigned long> lastUpdateTimes;
// Store the last update time for each announced address
std::map<uint16_t, unsigned long> lastUpdateTimesAnnounce;

std::map<uint16_t, PowerStatsAccumulator> nodeAccumulators;
bool isElegantOtaStarted = false;

// Layout for the designer page. Stored in NVS (not LittleFS), because a
// filesystem update overwrites LittleFS completely. NVS is small (20 KB) and
// shared with WiFi, PHY calibration and settings, so the layout may only use
// what is left after keeping a reserve for those.
constexpr size_t kLayoutMaxBytes = 8192;
constexpr size_t kNvsReserveEntries = 64; // ~2 KB kept free for others
const char *kDefaultLayout = "{\"v\":1,\"el\":[]}";

// NVS entries a blob occupies: 32-byte data entries, a header per chunk (one
// chunk per 4 KB page) and an index entry.
size_t blobEntries(size_t len) {
  return len == 0 ? 0 : (len + 31) / 32 + (len + 3999) / 4000 + 2;
}

size_t storedLayoutLength() {
  Preferences prefs;
  size_t len = 0;
  if (prefs.begin("tigolayout", true)) {
    len = prefs.getBytesLength("layout");
    prefs.end();
  }
  return len;
}

// Largest layout that can be stored right now.
size_t layoutCapacity() {
  nvs_stats_t stats;
  if (nvs_get_stats(NULL, &stats) != ESP_OK)
    return 0;
  long entries = static_cast<long>(stats.available_entries) +
                 static_cast<long>(blobEntries(storedLayoutLength())) -
                 static_cast<long>(kNvsReserveEntries);
  // Overhead of a blob that spans up to three pages.
  long bytes = (entries - 5) * 32;
  if (bytes <= 0)
    return 0;
  return std::min(static_cast<size_t>(bytes), kLayoutMaxBytes);
}

String loadLayout() {
  Preferences prefs;
  String layout;
  if (prefs.begin("tigolayout", true)) {
    size_t len = prefs.getBytesLength("layout");
    if (len > 0) {
      std::unique_ptr<char[]> buf(new (std::nothrow) char[len + 1]);
      if (buf && prefs.getBytes("layout", buf.get(), len) == len) {
        buf[len] = '\0';
        layout = buf.get();
      }
    }
    prefs.end();
  }
  return layout.length() > 0 ? layout : String(kDefaultLayout);
}

// Remove first: NVS would otherwise need room for old and new blob at once.
// Back up any existing layout in RAM first so a failed write can restore it.
// Only called after layoutCapacity() confirmed the new blob fits.
bool saveLayout(const uint8_t *data, size_t len) {
  Preferences prefs;
  if (!prefs.begin("tigolayout", false))
    return false;
  size_t oldLen = prefs.getBytesLength("layout");
  std::unique_ptr<uint8_t[]> backup;
  if (oldLen > 0 && oldLen <= kLayoutMaxBytes) {
    backup.reset(new (std::nothrow) uint8_t[oldLen]);
    if (!backup || prefs.getBytes("layout", backup.get(), oldLen) != oldLen) {
      prefs.end();
      return false;
    }
  }
  prefs.remove("layout");
  bool ok = prefs.putBytes("layout", data, len) == len;
  if (!ok && backup) {
    bool restored = prefs.putBytes("layout", backup.get(), oldLen) == oldLen;
    taskLog.printf("Layout: save failed; backup restore %s (%u bytes)\n",
                   restored ? "succeeded" : "FAILED",
                   static_cast<unsigned>(oldLen));
  }
  prefs.end();
  return ok;
}

void publishMqtt() {
  MqttPublishSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(dataMutex);
    snap.cfg.discovery = appConfig.mqttDiscovery;
    snap.cfg.topic = appConfig.mqttTopic.c_str();
    snap.cfg.interval = appConfig.mqttInterval;
    snap.cfg.hostname = appConfig.hostname.c_str();
    snap.nowMs = static_cast<uint64_t>(esp_timer_get_time() / 1000);
    snap.nodes.reserve(nodePowerData.size());
    for (auto const &[key, val] : nodePowerData) {
      uint64_t ageMs = snap.nowMs >= val.last_updated
                           ? snap.nowMs - val.last_updated
                           : 0;
      if (ageMs > kMqttStaleMs)
        continue;
      const auto &frame = val.frame;
      auto info = persistentNodeTable.find(frame->pv_node_id);
      std::string barcode =
          info != persistentNodeTable.end() ? info->second.barcode : "";
      snap.nodes.push_back({frame->pv_node_id, frame->address,
                            std::move(barcode), frame->voltage_in,
                            frame->voltage_out, frame->current_in,
                            frame->temperature, frame->duty_cycle,
                            frame->rssi, ageMs / 1000});
    }
    snap.knownNodes = persistentNodeTable.size();
    snap.stats = parser.getStatistics();
    const auto &barcodes = parser.getGatewayBarcodes();
    for (auto const &[id, gw] : parser.getGateways()) {
      if (!gw.seen && gw.polls == 0)
        continue;
      auto bc = barcodes.find(id);
      std::string barcode = bc != barcodes.end() ? bc->second : "";
      snap.taps.push_back({id, gw, std::move(barcode)});
    }
  }
  snap.cfg.swVersion = VERSION;
  snap.cfg.repoUrl = TIGOTELL_REPO_URL;
  snap.cfg.localIp = WiFi.localIP().toString().c_str();

  tigoMqtt.publish(snap, mqtt.sink());
}

struct RequestBodyUpload {
  std::string body;
  int errorStatus = 0; // 0 = ok, else 400 / 413
  size_t rejectedBytes = 0;
};

// Accumulate bounded raw request body chunks onto request->_tempObject.
void accumulateRequestBody(AsyncWebServerRequest *request, uint8_t *data,
                           size_t len, size_t index, size_t total,
                           size_t maxBytes) {
  if (index == 0 && !request->_tempObject) {
    auto *upload = new (std::nothrow) RequestBodyUpload{};
    if (!upload)
      return;
    if (total > maxBytes) {
      upload->errorStatus = 413;
      upload->rejectedBytes = total;
    } else if (total > 0) {
      upload->body.reserve(total);
    }
    request->_tempObject = upload;
    request->onDisconnect([request]() {
      delete static_cast<RequestBodyUpload *>(request->_tempObject);
      request->_tempObject = nullptr;
    });
  }

  auto *upload = static_cast<RequestBodyUpload *>(request->_tempObject);
  if (!upload || upload->errorStatus != 0)
    return;

  if (index != upload->body.size() || len > maxBytes - upload->body.size() ||
      (total > 0 && (len > total || index > total - len))) {
    if (len > maxBytes - upload->body.size()) {
      upload->errorStatus = 413;
      upload->rejectedBytes =
          total > 0 ? total
                    : upload->body.size() + std::min(len, maxBytes + 1);
    } else {
      upload->errorStatus = 400;
    }
    upload->body.clear();
    return;
  }
  if (len > 0)
    upload->body.append(reinterpret_cast<const char *>(data), len);
}

// Take ownership of the accumulated upload and validate completeness.
// Returns nullptr after sending an error response (invoking onTooLarge on 413).
template <typename OnTooLarge>
std::unique_ptr<RequestBodyUpload>
takeUpload(AsyncWebServerRequest *request, OnTooLarge &&onTooLarge) {
  std::unique_ptr<RequestBodyUpload> upload(
      static_cast<RequestBodyUpload *>(request->_tempObject));
  request->_tempObject = nullptr;

  if (!upload) {
    bool nonJson = request->multipart() ||
                   request->contentType().startsWith(
                       "application/x-www-form-urlencoded") ||
                   request->params() > 0;
    if (nonJson) {
      request->send(400, "text/plain", "Expected application/json body");
    } else if (request->contentLength() == 0) {
      request->send(400, "text/plain", "Empty body");
    } else {
      request->send(503, "text/plain", "Out of memory");
    }
    return nullptr;
  }
  if (upload->errorStatus == 413) {
    onTooLarge(upload->rejectedBytes);
    return nullptr;
  }
  if (upload->errorStatus != 0) {
    request->send(400, "text/plain", "Invalid body size");
    return nullptr;
  }
  if (upload->body.empty()) {
    request->send(400, "text/plain", "Empty body");
    return nullptr;
  }
  if (request->contentLength() > 0 &&
      upload->body.size() != request->contentLength()) {
    request->send(400, "text/plain", "Incomplete body");
    return nullptr;
  }
  return upload;
}

void setup() {
  health::begin();
  loadSettings();
  loadNodeMap();

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

  // MQTT (runs in its own task)
  {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char id[24];
    snprintf(id, sizeof(id), "tigotell_%02x%02x%02x", mac[3], mac[4], mac[5]);
    tigoMqtt.setDeviceId(id);
  }
  mqtt.begin();
  applyMqttSettings(appConfig);

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

      JsonObject jsonSys = root["system"].to<JsonObject>();
      health::toJson(jsonSys);
      nvs_stats_t nvsStats = {};
      if (nvs_get_stats(NULL, &nvsStats) == ESP_OK)
        jsonSys["nvs_free_entries"] = nvsStats.available_entries;

      JsonObject jsonMqtt = root["mqtt"].to<JsonObject>();
      jsonMqtt["enabled"] = mqtt.isEnabled();
      jsonMqtt["connected"] = mqtt.isConnected();
      jsonMqtt["published"] = mqtt.publishedCount();

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
        if (frame->gateway_id)
          node["gateway"] = tapHexId(frame->gateway_id).c_str();
        node["last_updated"] = val.last_updated;
      }

      // TAPs (gateways) as seen on the bus
      JsonArray taps = root["taps"].to<JsonArray>();
      uint64_t nowMs = static_cast<uint64_t>(esp_timer_get_time() / 1000);
      const auto &barcodes = parser.getGatewayBarcodes();
      for (auto const &[id, gw] : parser.getGateways()) {
        if (gw.seen || gw.polls > 0) {
          auto bc = barcodes.find(id);
          tapToJson(taps.add<JsonObject>(), gw, nowMs,
                    bc != barcodes.end() ? &bc->second : nullptr);
        }
      }

      // Discovery Data (Persistent Map)
      JsonArray discoverNodes = root["nodes"].to<JsonArray>();
      for (auto const &[key, val] : persistentNodeTable) {
        JsonObject node = discoverNodes.add<JsonObject>();
        node["address"] = val.address;
        node["pv_node_id"] = val.pv_node_id;
        if (!val.long_address.empty())
          node["long_address"] = val.long_address.c_str();
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
    Config cfg;
    {
      std::lock_guard<std::mutex> lock(dataMutex);
      cfg = appConfig;
    }

    JsonDocument doc;
    doc["hostname"] = cfg.hostname;
    doc["txPin"] = cfg.txPin;
    doc["rxPin"] = cfg.rxPin;
    doc["enPin"] = cfg.enPin;
    doc["influxHost"] = cfg.influxHost;
    doc["influxPort"] = cfg.influxPort;
    doc["pushStatsInterval"] = cfg.pushStatsInterval;
    doc["measurementName"] = cfg.measurementName;
    doc["minSecondsNodeUpdates"] = cfg.minSecondsNodeUpdates;
    doc["minSecondsNodeTableUpdates"] = cfg.minSecondsNodeTableUpdates;
    doc["mqttHost"] = cfg.mqttHost;
    doc["mqttPort"] = cfg.mqttPort;
    doc["mqttUser"] = cfg.mqttUser;
    doc["mqttTopic"] = cfg.mqttTopic;
    doc["mqttInterval"] = cfg.mqttInterval;
    doc["mqttDiscovery"] = cfg.mqttDiscovery;
    doc["otaEnabled"] = isElegantOtaStarted;

    AsyncResponseStream *response =
        request->beginResponseStream("application/json");
    serializeJson(doc, *response);
    request->send(response);
  });

  // Config POST endpoint
  constexpr size_t kConfigMaxBytes = 4096;
  server.on(
      "/api/config", HTTP_POST,
      [](AsyncWebServerRequest *request) {
        auto upload = takeUpload(request, [request](size_t) {
          request->send(413, "text/plain", "Payload too large");
        });
        if (!upload)
          return;

        JsonDocument doc;
        DeserializationError error =
            deserializeJson(doc, upload->body.data(), upload->body.size());
        upload.reset();

        if (error) {
          if (error == DeserializationError::NoMemory) {
            request->send(503, "text/plain", "Out of memory");
          } else {
            request->send(400, "text/plain", "Invalid JSON");
          }
          return;
        }
        if (!doc.is<JsonObject>()) {
          request->send(400, "text/plain", "Expected JSON object");
          return;
        }

        Config cfg;
        {
          std::lock_guard<std::mutex> lock(dataMutex);
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
          if (doc["influxPort"].is<int>()) {
            int port = doc["influxPort"];
            if (port >= 0 && port <= 65535)
              appConfig.influxPort = static_cast<uint16_t>(port);
          }
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
          if (doc["mqttHost"].is<const char *>())
            appConfig.mqttHost = doc["mqttHost"].as<String>();
          if (doc["mqttPort"].is<int>()) {
            int port = doc["mqttPort"];
            if (port >= 0 && port <= 65535)
              appConfig.mqttPort = static_cast<uint16_t>(port);
          }
          if (doc["mqttUser"].is<const char *>())
            appConfig.mqttUser = doc["mqttUser"].as<String>();
          if (doc["mqttPass"].is<const char *>()) {
            String val = doc["mqttPass"].as<String>();
            if (val.length() > 0)
              appConfig.mqttPass = val;
          }
          if (doc["mqttTopic"].is<const char *>()) {
            String val = doc["mqttTopic"].as<String>();
            val.trim();
            while (val.endsWith("/"))
              val.remove(val.length() - 1);
            if (val.length() > 0)
              appConfig.mqttTopic = val;
          }
          if (doc["mqttInterval"].is<int>())
            appConfig.mqttInterval =
                std::clamp(doc["mqttInterval"].as<int>(), 1, 3600);
          if (doc["mqttDiscovery"].is<bool>())
            appConfig.mqttDiscovery = doc["mqttDiscovery"];
          cfg = appConfig;
        }

        saveSettings(cfg);
        applyMqttSettings(cfg);
        request->send(200, "text/plain", "OK");
      },
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
         size_t index, size_t total) {
        accumulateRequestBody(request, data, len, index, total,
                              kConfigMaxBytes);
      });

  // Layout endpoints (designer page)
  server.on("/api/layout", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response =
        request->beginResponse(200, "application/json", loadLayout());
    response->addHeader("X-Layout-Max", String(layoutCapacity()));
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
  });

  server.on(
      "/api/layout", HTTP_POST,
      [](AsyncWebServerRequest *request) {
        auto sendTooLarge = [request](size_t bytes, size_t capacity) {
          taskLog.printf("Layout: %u bytes rejected, room for %u bytes\n",
                         static_cast<unsigned>(bytes),
                         static_cast<unsigned>(capacity));
          AsyncWebServerResponse *response = request->beginResponse(
              413, "text/plain",
              "Layout too large: the device has room for " + String(capacity) +
                  " bytes");
          response->addHeader("X-Layout-Max", String(capacity));
          request->send(response);
        };
        auto upload = takeUpload(request, [&sendTooLarge](size_t bytes) {
          sendTooLarge(bytes, layoutCapacity());
        });
        if (!upload)
          return;

        size_t capacity = layoutCapacity();
        if (upload->body.size() > capacity) {
          sendTooLarge(upload->body.size(), capacity);
          return;
        }

        {
          JsonDocument doc;
          DeserializationError err =
              deserializeJson(doc, upload->body.data(), upload->body.size());
          if (err == DeserializationError::NoMemory) {
            request->send(503, "text/plain", "Out of memory");
            return;
          }
          if (err || !doc.is<JsonObject>() || !doc["el"].is<JsonArray>()) {
            request->send(400, "text/plain", "Invalid layout JSON");
            return;
          }
        }

        size_t len = upload->body.size();
        unsigned long started = millis();
        bool ok = saveLayout(
            reinterpret_cast<const uint8_t *>(upload->body.data()), len);
        upload.reset();
        nvs_stats_t stats = {};
        nvs_get_stats(NULL, &stats);
        taskLog.printf("Layout: %s %u bytes in %lu ms, NVS entries left %u, "
                       "free heap %u\n",
                       ok ? "saved" : "FAILED to save",
                       static_cast<unsigned>(len), millis() - started,
                       static_cast<unsigned>(stats.available_entries),
                       ESP.getFreeHeap());
        if (!ok) {
          request->send(507, "text/plain",
                        "The device could not store the layout");
          return;
        }
        request->send(200, "text/plain", "OK");
      },
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
         size_t index, size_t total) {
        accumulateRequestBody(request, data, len, index, total,
                              kLayoutMaxBytes);
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
    GithubReleaseOTA ota(TIGOTELL_REPO_OWNER, TIGOTELL_REPO_NAME);

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

      taskLog.printf(
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
    // Explain instead of answering "Not found".
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(
          503, "text/html",
          "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\">"
          "<meta name=\"viewport\" content=\"width=device-width, "
          "initial-scale=1.0\"><title>Firmware update disabled</title>"
          "<link rel=\"stylesheet\" href=\"/style.css\"></head><body>"
          "<div class=\"container\"><header><h1>Firmware update disabled"
          "</h1></header><p>The firmware upload page needs a username and "
          "password.</p><p>Set them under <a href=\"/config.html#ota\">"
          "Settings &rsaquo; OTA Authentication</a>, save, then reboot the "
          "device.</p><p><a href=\"/\">Back to dashboard</a></p></div>"
          "</body></html>");
    });
  }
  server.begin();

  // Restart (instead of hanging silently) if loop() ever stalls for 5 s.
  enableLoopWDT();
}

void loop() {
  taskLog.flush();
  health::loop(isUpdatingOTA);
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
        parser.setTime(static_cast<uint64_t>(esp_timer_get_time() / 1000));
        parser.pushBytes(serialBuf, count);
      }
    }
  }

  // Process parsed frames
  bool hasFrameCfg = false;
  int minNodeUpdates = 0;
  String measName;
  while (parser.hasFrames()) {
    if (!hasFrameCfg) {
      std::lock_guard<std::mutex> lock(dataMutex);
      minNodeUpdates = appConfig.minSecondsNodeUpdates;
      measName = appConfig.measurementName;
      hasFrameCfg = true;
    }
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
              static_cast<unsigned long>(minNodeUpdates) * 1000) {
            shouldSend = true;
          }
        }

        if (shouldSend && WiFi.status() == WL_CONNECTED) {
          if (nodeAccumulators[powerFrame->pv_node_id].hasSamples()) {
            std::string lineProtocol =
                nodeAccumulators[powerFrame->pv_node_id].toInfluxLineProtocol(
                    measName.c_str(), powerFrame->address,
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
              static_cast<unsigned long>(minNodeUpdates) * 1000) {
            shouldSend = true;
          }
        }

        if (shouldSend && WiFi.status() == WL_CONNECTED) {
          std::string lineProtocol =
              announceFrame->toInfluxLineProtocol(measName.c_str());

          // Send over UDP
          sendUdpPayload(lineProtocol);

          // Debug output
          WebSerial.print("Sent Announce: ");
          WebSerial.println(lineProtocol.c_str());

          lastUpdateTimesAnnounce[announceFrame->pv_node_id] = currentTime;
        }

        // Update persistent node table
        bool changed = false;
        std::vector<uint8_t> blob;
        {
          std::lock_guard<std::mutex> lock(dataMutex);
          NodeInfo &info = persistentNodeTable[announceFrame->pv_node_id];
          info.address = announceFrame->address;
          info.pv_node_id = announceFrame->pv_node_id;
          info.barcode_fragment = announceFrame->barcode_fragment;
          info.last_seen = esp_timer_get_time() / 1000;
          if (!announceFrame->long_address.empty() &&
              info.long_address != announceFrame->long_address) {
            std::string bc = tigoBarcodeFromHex(announceFrame->long_address);
            if (!bc.empty()) {
              info.long_address = announceFrame->long_address;
              info.barcode = bc;
              changed = true;
            }
          }
          if (changed)
            blob = nodeMapBlob();
        }
        if (changed)
          saveNodeMap(blob);
      }
      break;
    }
    case TigoFrameType::NodeTable: {
      TigoNodeTableFrame *nodeTableFrame =
          static_cast<TigoNodeTableFrame *>(frame.get());
      if (nodeTableFrame) {
        if (WiFi.status() == WL_CONNECTED) {
          std::string lineProtocol =
              nodeTableFrame->toInfluxLineProtocol(measName.c_str());

          // Send over UDP
          sendUdpPayload(lineProtocol);

          // Debug output
          WebSerial.print("Sent NodeTable: ");
          WebSerial.println(lineProtocol.c_str());
        }

        // Update persistent node table with all records from this frame
        bool changed = false;
        std::vector<uint8_t> blob;
        {
          std::lock_guard<std::mutex> lock(dataMutex);
          for (const auto &entry : nodeTableFrame->nodes) {
            NodeInfo &info = persistentNodeTable[entry.pv_node_id];
            info.pv_node_id = entry.pv_node_id;
            if (info.long_address != entry.barcode) {
              info.long_address = entry.barcode;
              info.barcode = tigoBarcodeFromHex(entry.barcode);
              changed = true;
            }
          }
          if (changed)
            blob = nodeMapBlob();
        }
        if (changed)
          saveNodeMap(blob);
      }
      break;
    }
    }
  }

  int pushStatsInterval;
  int mqttInterval;
  {
    std::lock_guard<std::mutex> lock(dataMutex);
    pushStatsInterval = appConfig.pushStatsInterval;
    mqttInterval = appConfig.mqttInterval;
  }

  // Send Statistics periodically
  if (millis() - lastStatsTime >
          static_cast<unsigned long>(pushStatsInterval) * 1000 &&
      WiFi.status() == WL_CONNECTED) {
    lastStatsTime = millis();
    String measName;
    {
      std::lock_guard<std::mutex> lock(dataMutex);
      measName = appConfig.measurementName;
    }
    std::string statsLine =
        parser.getStatisticsInfluxLineProtocol(measName.c_str());
    statsLine +=
        ",uptime=" + std::to_string(esp_timer_get_time() / 1000000) + "i";

    sendUdpPayload(statsLine);

    WebSerial.print("Sent Stats: ");
    WebSerial.println(statsLine.c_str());
  }

  // Publish data points to MQTT periodically
  if (mqtt.isConnected() &&
      millis() - lastMqttTime >
          static_cast<unsigned long>(mqttInterval) * 1000) {
    lastMqttTime = millis();
    publishMqtt();
  }
}
