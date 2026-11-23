#ifndef TIGO_MQTT_H
#define TIGO_MQTT_H

#include <ArduinoJson.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "MqttCore.h"
#include "TigoProtocol.h"

// A TAP that sent nothing for this long counts as offline.
constexpr uint64_t kTapOfflineMs = 60000;
// Modules heard through a TAP within this window count as connected to it.
constexpr uint64_t kTapModuleWindowMs = 300000;

std::string tapHexId(uint16_t id);
// "ok", "degraded" (answers fewer than 90 % of recent polls) or "offline".
const char *tapState(const TigoGatewayStatus &gw, uint64_t now);
void tapToJson(JsonObject o, const TigoGatewayStatus &gw, uint64_t now,
               const std::string *barcode = nullptr);

struct MqttConfigSnapshot {
  bool discovery = true;
  std::string topic;
  int interval = 30;
  std::string hostname;
  std::string swVersion;
  std::string repoUrl;
  std::string localIp;
};

struct MqttNodeSnapshot {
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

struct MqttTapSnapshot {
  uint16_t id;
  TigoGatewayStatus gw;
  std::string barcode;
};

struct MqttPublishSnapshot {
  MqttConfigSnapshot cfg;
  uint64_t nowMs = 0;
  std::vector<MqttNodeSnapshot> nodes;
  size_t knownNodes = 0;
  TigoParser::Statistics stats;
  std::vector<MqttTapSnapshot> taps;
};

// Formats Home Assistant MQTT discovery payloads and telemetry snapshots and
// sends them through an IMqttSink.
class TigoMqttPublisher {
public:
  void setDeviceId(const std::string &id) { haDeviceId = id; }
  const std::string &deviceId() const { return haDeviceId; }

  void publish(const MqttPublishSnapshot &snap, IMqttSink &sink);

private:
  bool publishSystemDiscovery(const MqttConfigSnapshot &cfg, IMqttSink &sink);
  bool publishNodeDiscovery(uint16_t nodeId, const std::string &barcode,
                            const MqttConfigSnapshot &cfg, IMqttSink &sink);
  bool publishTapDiscovery(uint16_t id, const std::string &barcode,
                           const MqttConfigSnapshot &cfg, IMqttSink &sink);

  std::string haDeviceId;
  uint32_t lastDiscoveryGen = 0;
  bool discoveredDevice = false;
  // Discovered nodes/TAPs; value = whether the barcode was known at the time,
  // so the config is re-sent once the barcode shows up.
  std::map<uint16_t, bool> discoveredNodes;
  std::map<uint16_t, bool> discoveredTaps;
};

#endif // TIGO_MQTT_H
