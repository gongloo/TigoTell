#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ElegantOTA.h>
#include <LittleFS.h>
#include <NetWizard.h>
#include <WebSerial.h>

#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <mutex>

#include "Health.h"
#include "RepoInfo.h"
#include "MqttPublisher.h"
#include "TaskLog.h"
#include "PowerStatsAccumulator.h"
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
  appConfig.mqttHost = preferences.getString("mqttHost", appConfig.mqttHost);
  appConfig.mqttPort = preferences.getUShort("mqttPort", appConfig.mqttPort);
  appConfig.mqttUser = preferences.getString("mqttUser", appConfig.mqttUser);
  appConfig.mqttPass = preferences.getString("mqttPass", appConfig.mqttPass);
  appConfig.mqttTopic = preferences.getString("mqttTopic", appConfig.mqttTopic);
  appConfig.mqttInterval =
      preferences.getInt("mqttIntv", appConfig.mqttInterval);
  appConfig.mqttDiscovery =
      preferences.getBool("mqttHaDisc", appConfig.mqttDiscovery);
  preferences.end();
}

void saveSettings(const Config &cfg) {
  preferences.begin("tigotell", false);
  preferences.putString("hostname", cfg.hostname);
  preferences.putInt("txPin", cfg.txPin);
  preferences.putInt("rxPin", cfg.rxPin);
  preferences.putInt("enPin", cfg.enPin);
  preferences.putString("influxHost", cfg.influxHost);
  preferences.putUShort("influxPort", cfg.influxPort);
  preferences.putInt("pushStats", cfg.pushStatsInterval);
  preferences.putString("measName", cfg.measurementName);
  preferences.putInt("nodeUpd", cfg.minSecondsNodeUpdates);
  preferences.putInt("tableUpd", cfg.minSecondsNodeTableUpdates);
  preferences.putString("otaUser", cfg.otaUser);
  preferences.putString("otaPass", cfg.otaPass);
  preferences.putString("mqttHost", cfg.mqttHost);
  preferences.putUShort("mqttPort", cfg.mqttPort);
  preferences.putString("mqttUser", cfg.mqttUser);
  preferences.putString("mqttPass", cfg.mqttPass);
  preferences.putString("mqttTopic", cfg.mqttTopic);
  preferences.putInt("mqttIntv", cfg.mqttInterval);
  preferences.putBool("mqttHaDisc", cfg.mqttDiscovery);
  preferences.end();
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

// Home Assistant discovery state.
String haDeviceId; // "tigotell_<last 3 MAC bytes>", set in setup()
uint32_t lastDiscoveryGen = 0;
bool discoveredDevice = false;
// Discovered nodes; value = whether the barcode was known at the time, so the
// config is re-sent once the barcode shows up.
std::map<uint16_t, bool> discoveredNodes;
std::map<uint16_t, bool> discoveredTaps; // value: barcode was known

// A TAP that sent nothing for this long counts as offline.
constexpr uint64_t kTapOfflineMs = 60000;
// Modules heard through a TAP within this window count as connected to it.
constexpr uint64_t kTapModuleWindowMs = 300000;

String tapHexId(uint16_t id) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%04X", id);
  return buf;
}

// "ok", "degraded" (answers fewer than 90 % of recent polls) or "offline".
const char *tapState(const TigoGatewayStatus &gw, uint64_t now) {
  uint64_t age = now >= gw.lastSeenMs ? now - gw.lastSeenMs : 0;
  if (!gw.seen || age > kTapOfflineMs)
    return "offline";
  int pct = gw.recentAnsweredPercent();
  return pct >= 0 && pct < 90 ? "degraded" : "ok";
}

void tapToJson(JsonObject o, const TigoGatewayStatus &gw, uint64_t now,
               const std::string *barcode = nullptr) {
  o["id"] = tapHexId(gw.id);
  o["state"] = tapState(gw, now);
  if (gw.seen)
    o["last_seen_s"] = (now >= gw.lastSeenMs ? now - gw.lastSeenMs : 0) / 1000;
  o["polls"] = gw.polls;
  o["answered"] = gw.answered;
  o["missed"] = gw.missed;
  int pct = gw.recentAnsweredPercent();
  if (pct >= 0)
    o["recent_answered_pct"] = pct;
  if (gw.rxBuffersUsed >= 0) {
    o["rx_buffers_used"] = gw.rxBuffersUsed;
    o["rx_buffers_max"] = gw.rxBuffersMax;
  }
  if (gw.txBuffersFree >= 0) {
    o["tx_buffers_free"] = gw.txBuffersFree;
    if (gw.txBuffersMin >= 0)
      o["tx_buffers_min"] = gw.txBuffersMin;
  }
  if (gw.slotCounter >= 0)
    o["slot_counter"] = gw.slotCounter;
  o["packets"] = gw.packets;
  o["power_reports"] = gw.powerReports;
  int modules = 0;
  long rssiSum = 0;
  int rssiMin = 999;
  for (auto const &[node, seen] : gw.nodes) {
    uint64_t age = now >= seen.lastMs ? now - seen.lastMs : 0;
    if (age > kTapModuleWindowMs)
      continue;
    modules++;
    rssiSum += seen.rssi;
    rssiMin = std::min(rssiMin, seen.rssi);
  }
  o["modules"] = modules;
  if (modules > 0) {
    o["rssi_avg"] = static_cast<int>(rssiSum / modules);
    o["rssi_min"] = rssiMin;
  }
  if (barcode && !barcode->empty())
    o["barcode"] = barcode->c_str();
  if (!gw.version.empty())
    o["version"] = gw.version.c_str();
  if (gw.radioChannel >= 0)
    o["radio_channel"] = gw.radioChannel;
  if (gw.panId >= 0)
    o["pan_id"] = tapHexId(gw.panId);
}


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

void addHaSensor(JsonObject cmps, const String &uidBase, const char *key,
                 const char *name, const char *deviceClass, const char *unit,
                 const char *stateClass, bool diagnostic) {
  String uid = uidBase + "_" + key;
  JsonObject c = cmps[uid].to<JsonObject>();
  c["p"] = "sensor";
  c["name"] = name;
  c["uniq_id"] = uid;
  c["val_tpl"] = String("{{ value_json.") + key + " }}";
  if (deviceClass)
    c["dev_cla"] = deviceClass;
  if (unit)
    c["unit_of_meas"] = unit;
  if (stateClass)
    c["stat_cla"] = stateClass;
  if (diagnostic)
    c["ent_cat"] = "diagnostic";
}

struct MqttConfigSnapshot {
  bool discovery;
  String topic;
  int interval;
  String hostname;
};

// Fills the parts shared by all discovery messages and sends it retained.
bool sendHaDiscovery(JsonDocument &doc, const String &objectId,
                     const String &stateSubTopic,
                     const MqttConfigSnapshot &cfg) {
  JsonObject origin = doc["o"].to<JsonObject>();
  origin["name"] = "TigoTell";
  origin["sw"] = VERSION;
  origin["url"] = TIGOTELL_REPO_URL;
  doc["stat_t"] = cfg.topic + "/" + stateSubTopic;
  doc["avty_t"] = cfg.topic + "/status";
  // Values without update for this long show as unavailable in HA.
  doc["exp_aft"] = 3 * cfg.interval + 60;

  String payload;
  serializeJson(doc, payload);
  return mqtt.enqueueAbsolute("homeassistant/device/" + objectId + "/config",
                              payload, true);
}

bool publishHaSystemDiscovery(const MqttConfigSnapshot &cfg) {
  JsonDocument doc;
  JsonObject dev = doc["dev"].to<JsonObject>();
  dev["ids"] = haDeviceId;
  dev["name"] = cfg.hostname;
  dev["mdl"] = "TigoTell";
  dev["sw"] = VERSION;
  dev["cu"] = "http://" + WiFi.localIP().toString() + "/";

  JsonObject cmps = doc["cmps"].to<JsonObject>();
  addHaSensor(cmps, haDeviceId, "total_power", "Total power", "power", "W",
              "measurement", false);
  addHaSensor(cmps, haDeviceId, "active_nodes", "Active modules", nullptr,
              nullptr, "measurement", false);
  addHaSensor(cmps, haDeviceId, "known_nodes", "Known modules", nullptr,
              nullptr, "measurement", true);
  addHaSensor(cmps, haDeviceId, "crc_errors", "CRC errors", nullptr, nullptr,
              "total_increasing", true);
  addHaSensor(cmps, haDeviceId, "total_frames", "Frames", nullptr, nullptr,
              "total_increasing", true);
  addHaSensor(cmps, haDeviceId, "uptime_s", "Uptime", "duration", "s",
              nullptr, true);

  return sendHaDiscovery(doc, haDeviceId, "stats", cfg);
}

bool publishHaNodeDiscovery(uint16_t nodeId, const std::string &barcode,
                            const MqttConfigSnapshot &cfg) {
  String uidBase = haDeviceId + "_node_" + String(nodeId);
  JsonDocument doc;
  JsonObject dev = doc["dev"].to<JsonObject>();
  dev["ids"] = uidBase;
  dev["name"] = "Tigo Module " + String(nodeId);
  dev["mf"] = "Tigo";
  dev["mdl"] = "PV module optimizer";
  if (!barcode.empty())
    dev["sn"] = barcode.c_str();
  dev["via_device"] = haDeviceId;

  JsonObject cmps = doc["cmps"].to<JsonObject>();
  addHaSensor(cmps, uidBase, "power", "Power", "power", "W", "measurement",
              false);
  addHaSensor(cmps, uidBase, "voltage_in", "Voltage in", "voltage", "V",
              "measurement", false);
  addHaSensor(cmps, uidBase, "voltage_out", "Voltage out", "voltage", "V",
              "measurement", false);
  addHaSensor(cmps, uidBase, "current_in", "Current", "current", "A",
              "measurement", false);
  addHaSensor(cmps, uidBase, "temperature", "Temperature", "temperature",
              "°C", "measurement", false);
  addHaSensor(cmps, uidBase, "duty_cycle", "Duty cycle", nullptr, nullptr,
              "measurement", true);
  addHaSensor(cmps, uidBase, "rssi", "RSSI", "signal_strength", "dBm",
              "measurement", true);

  return sendHaDiscovery(doc, uidBase, "node/" + String(nodeId), cfg);
}

bool publishHaTapDiscovery(uint16_t id, const std::string &barcode,
                           const MqttConfigSnapshot &cfg) {
  String hex = tapHexId(id);
  String uidBase = haDeviceId + "_tap_" + hex;
  JsonDocument doc;
  JsonObject dev = doc["dev"].to<JsonObject>();
  dev["ids"] = uidBase;
  dev["name"] = "Tigo TAP " + hex;
  dev["mf"] = "Tigo";
  dev["mdl"] = "TAP (Access Point)";
  if (!barcode.empty())
    dev["sn"] = barcode.c_str();
  dev["via_device"] = haDeviceId;

  JsonObject cmps = doc["cmps"].to<JsonObject>();
  JsonObject online = cmps[uidBase + "_online"].to<JsonObject>();
  online["p"] = "binary_sensor";
  online["name"] = "Connection";
  online["uniq_id"] = uidBase + "_online";
  online["dev_cla"] = "connectivity";
  online["val_tpl"] = "{{ 'OFF' if value_json.state == 'offline' else 'ON' }}";
  addHaSensor(cmps, uidBase, "recent_answered_pct", "Polls answered", nullptr,
              "%", "measurement", false);
  addHaSensor(cmps, uidBase, "modules", "Modules", nullptr, nullptr,
              "measurement", false);
  addHaSensor(cmps, uidBase, "rssi_avg", "Average module signal", nullptr,
              nullptr, "measurement", true);
  addHaSensor(cmps, uidBase, "rx_buffers_used", "Receive buffers used",
              nullptr, nullptr, "measurement", true);
  addHaSensor(cmps, uidBase, "tx_buffers_free", "Transmit buffers free",
              nullptr, nullptr, "measurement", true);
  addHaSensor(cmps, uidBase, "missed", "Missed polls", nullptr, nullptr,
              "total_increasing", true);

  return sendHaDiscovery(doc, uidBase, "tap/" + hex, cfg);
}

void publishMqtt() {
  struct NodeSnapshot {
    uint16_t pv_node_id;
    uint16_t address;
    std::string barcode;
    float voltage_in;
    float voltage_out;
    float current_in;
    float temperature;
    uint8_t duty_cycle;
    int rssi;
    uint64_t age_s;
  };
  struct TapSnapshot {
    uint16_t id;
    TigoGatewayStatus gw;
    std::string barcode;
  };

  MqttConfigSnapshot cfg;
  uint64_t now;
  std::vector<NodeSnapshot> nodes;
  size_t knownNodes;
  TigoParser::Statistics stats;
  std::vector<TapSnapshot> taps;

  {
    std::lock_guard<std::mutex> lock(dataMutex);
    cfg = {appConfig.mqttDiscovery, appConfig.mqttTopic,
           appConfig.mqttInterval, appConfig.hostname};
    now = static_cast<uint64_t>(esp_timer_get_time() / 1000);
    nodes.reserve(nodePowerData.size());
    for (auto const &[key, val] : nodePowerData) {
      uint64_t ageMs = now >= val.last_updated ? now - val.last_updated : 0;
      if (ageMs > kMqttStaleMs)
        continue;
      const auto &frame = val.frame;
      auto info = persistentNodeTable.find(frame->pv_node_id);
      std::string barcode =
          info != persistentNodeTable.end() ? info->second.barcode : "";
      nodes.push_back({frame->pv_node_id, frame->address, std::move(barcode),
                       frame->voltage_in, frame->voltage_out,
                       frame->current_in, frame->temperature,
                       frame->duty_cycle, frame->rssi, ageMs / 1000});
    }
    knownNodes = persistentNodeTable.size();
    stats = parser.getStatistics();
    const auto &barcodes = parser.getGatewayBarcodes();
    for (auto const &[id, gw] : parser.getGateways()) {
      if (!gw.seen && gw.polls == 0)
        continue;
      auto bc = barcodes.find(id);
      std::string barcode = bc != barcodes.end() ? bc->second : "";
      taps.push_back({id, gw, std::move(barcode)});
    }
  }

  if (cfg.discovery) {
    uint32_t gen = mqtt.discoveryGeneration();
    if (gen != lastDiscoveryGen) {
      lastDiscoveryGen = gen;
      discoveredDevice = false;
      discoveredNodes.clear();
      discoveredTaps.clear();
    }
    if (!discoveredDevice && publishHaSystemDiscovery(cfg))
      discoveredDevice = true;
  }

  float totalPower = 0;
  for (const auto &n : nodes) {
    float power = n.voltage_out * n.current_in;
    totalPower += power;

    if (cfg.discovery) {
      auto known = discoveredNodes.find(n.pv_node_id);
      if (known == discoveredNodes.end() ||
          (!known->second && !n.barcode.empty())) {
        if (publishHaNodeDiscovery(n.pv_node_id, n.barcode, cfg))
          discoveredNodes[n.pv_node_id] = !n.barcode.empty();
      }
    }

    JsonDocument doc;
    doc["pv_node_id"] = n.pv_node_id;
    doc["address"] = n.address;
    if (!n.barcode.empty())
      doc["barcode"] = n.barcode.c_str();
    doc["power"] = power;
    doc["voltage_in"] = n.voltage_in;
    doc["voltage_out"] = n.voltage_out;
    doc["current_in"] = n.current_in;
    doc["temperature"] = n.temperature;
    doc["duty_cycle"] = n.duty_cycle;
    doc["rssi"] = n.rssi;
    doc["age_s"] = n.age_s;

    String payload;
    serializeJson(doc, payload);
    mqtt.enqueue("node/" + String(n.pv_node_id), payload);
  }

  JsonDocument doc;
  doc["total_power"] = totalPower;
  doc["active_nodes"] = nodes.size();
  doc["known_nodes"] = knownNodes;
  doc["total_frames"] = stats.totalPhysicalFrames;
  doc["crc_errors"] = stats.crcErrors;
  doc["power_count"] = stats.powerDataCount;
  doc["announce_count"] = stats.announceCount;
  doc["nodetable_count"] = stats.nodeTableCount;
  doc["uptime_s"] = now / 1000;
  doc["version"] = VERSION;

  String payload;
  serializeJson(doc, payload);
  mqtt.enqueue("stats", payload);

  for (const auto &tap : taps) {
    if (cfg.discovery) {
      bool hasBarcode = !tap.barcode.empty();
      auto known = discoveredTaps.find(tap.id);
      if (known == discoveredTaps.end() || (!known->second && hasBarcode)) {
        if (publishHaTapDiscovery(tap.id, tap.barcode, cfg))
          discoveredTaps[tap.id] = hasBarcode;
      }
    }
    JsonDocument tapDoc;
    tapToJson(tapDoc.to<JsonObject>(), tap.gw, now, &tap.barcode);
    String tapPayload;
    serializeJson(tapDoc, tapPayload);
    mqtt.enqueue("tap/" + tapHexId(tap.id), tapPayload);
  }
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
    haDeviceId = id;
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
          node["gateway"] = tapHexId(frame->gateway_id);
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
  server.on(
      "/api/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
         size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (!error) {
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
            if (doc["mqttHost"].is<const char *>())
              appConfig.mqttHost = doc["mqttHost"].as<String>();
            if (doc["mqttPort"].is<int>())
              appConfig.mqttPort = doc["mqttPort"];
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
              appConfig.mqttInterval = max(1, doc["mqttInterval"].as<int>());
            if (doc["mqttDiscovery"].is<bool>())
              appConfig.mqttDiscovery = doc["mqttDiscovery"];
            cfg = appConfig;
          }

          saveSettings(cfg);
          applyMqttSettings(cfg);
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
