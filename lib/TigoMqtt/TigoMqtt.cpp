#include "TigoMqtt.h"

#include <algorithm>
#include <cstdio>

namespace {

void addHaSensor(JsonObject cmps, const std::string &uidBase, const char *key,
                 const char *name, const char *deviceClass, const char *unit,
                 const char *stateClass, bool diagnostic) {
  std::string uid = uidBase + "_" + key;
  JsonObject c = cmps[uid].to<JsonObject>();
  c["p"] = "sensor";
  c["name"] = name;
  c["uniq_id"] = uid;
  c["val_tpl"] = std::string("{{ value_json.") + key + " }}";
  if (deviceClass)
    c["dev_cla"] = deviceClass;
  if (unit)
    c["unit_of_meas"] = unit;
  if (stateClass)
    c["stat_cla"] = stateClass;
  if (diagnostic)
    c["ent_cat"] = "diagnostic";
}

// Fills the parts shared by all discovery messages and sends it retained.
bool sendHaDiscovery(JsonDocument &doc, const std::string &objectId,
                     const std::string &stateSubTopic,
                     const MqttConfigSnapshot &cfg, IMqttSink &sink) {
  JsonObject origin = doc["o"].to<JsonObject>();
  origin["name"] = "TigoTell";
  origin["sw"] = cfg.swVersion;
  origin["url"] = cfg.repoUrl;
  doc["stat_t"] = cfg.topic + "/" + stateSubTopic;
  doc["avty_t"] = cfg.topic + "/status";
  // Values without update for this long show as unavailable in HA.
  doc["exp_aft"] = 3 * cfg.interval + 60;

  std::string payload;
  serializeJson(doc, payload);
  return sink.enqueueAbsolute("homeassistant/device/" + objectId + "/config",
                              payload, true);
}

} // namespace

std::string tapHexId(uint16_t id) {
  char buf[8];
  std::snprintf(buf, sizeof(buf), "%04X", id);
  return buf;
}

const char *tapState(const TigoGatewayStatus &gw, uint64_t now) {
  uint64_t age = now >= gw.lastSeenMs ? now - gw.lastSeenMs : 0;
  if (!gw.seen || age > kTapOfflineMs)
    return "offline";
  int pct = gw.recentAnsweredPercent();
  return pct >= 0 && pct < 90 ? "degraded" : "ok";
}

void tapToJson(JsonObject o, const TigoGatewayStatus &gw, uint64_t now,
               const std::string *barcode) {
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
    o["pan_id"] = tapHexId(static_cast<uint16_t>(gw.panId));
}

bool TigoMqttPublisher::publishSystemDiscovery(const MqttConfigSnapshot &cfg,
                                               IMqttSink &sink) {
  JsonDocument doc;
  JsonObject dev = doc["dev"].to<JsonObject>();
  dev["ids"] = haDeviceId;
  dev["name"] = cfg.hostname;
  dev["mdl"] = "TigoTell";
  dev["sw"] = cfg.swVersion;
  dev["cu"] = "http://" + cfg.localIp + "/";

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

  return sendHaDiscovery(doc, haDeviceId, "stats", cfg, sink);
}

bool TigoMqttPublisher::publishNodeDiscovery(uint16_t nodeId,
                                             const std::string &barcode,
                                             const MqttConfigSnapshot &cfg,
                                             IMqttSink &sink) {
  std::string nodeStr = std::to_string(nodeId);
  std::string uidBase = haDeviceId + "_node_" + nodeStr;
  JsonDocument doc;
  JsonObject dev = doc["dev"].to<JsonObject>();
  dev["ids"] = uidBase;
  dev["name"] = "Tigo Module " + nodeStr;
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
  addHaSensor(cmps, uidBase, "rssi", "RSSI", nullptr, nullptr,
              "measurement", true);

  return sendHaDiscovery(doc, uidBase, "node/" + nodeStr, cfg, sink);
}

bool TigoMqttPublisher::publishTapDiscovery(uint16_t id,
                                            const std::string &barcode,
                                            const MqttConfigSnapshot &cfg,
                                            IMqttSink &sink) {
  std::string hex = tapHexId(id);
  std::string uidBase = haDeviceId + "_tap_" + hex;
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

  return sendHaDiscovery(doc, uidBase, "tap/" + hex, cfg, sink);
}

void TigoMqttPublisher::publish(const MqttPublishSnapshot &snap,
                                IMqttSink &sink) {
  if (!snap.cfg.discovery) {
    lastDiscoveryGen = 0;
    discoveredDevice = false;
    discoveredNodes.clear();
    discoveredTaps.clear();
  } else {
    uint32_t gen = sink.discoveryGeneration();
    if (gen != lastDiscoveryGen) {
      lastDiscoveryGen = gen;
      discoveredDevice = false;
      discoveredNodes.clear();
      discoveredTaps.clear();
    }
    if (!discoveredDevice && publishSystemDiscovery(snap.cfg, sink))
      discoveredDevice = true;
    for (const auto &n : snap.nodes) {
      auto known = discoveredNodes.find(n.pv_node_id);
      if (known == discoveredNodes.end() ||
          (!known->second && !n.barcode.empty())) {
        if (publishNodeDiscovery(n.pv_node_id, n.barcode, snap.cfg, sink))
          discoveredNodes[n.pv_node_id] = !n.barcode.empty();
      }
    }
    for (const auto &tap : snap.taps) {
      bool hasBarcode = !tap.barcode.empty();
      auto known = discoveredTaps.find(tap.id);
      if (known == discoveredTaps.end() || (!known->second && hasBarcode)) {
        if (publishTapDiscovery(tap.id, tap.barcode, snap.cfg, sink))
          discoveredTaps[tap.id] = hasBarcode;
      }
    }
  }

  float totalPower = 0;
  for (const auto &n : snap.nodes) {
    float power = n.voltage_out * n.current_in;
    totalPower += power;

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

    std::string payload;
    serializeJson(doc, payload);
    sink.enqueue("node/" + std::to_string(n.pv_node_id), payload);
  }

  JsonDocument doc;
  doc["total_power"] = totalPower;
  doc["active_nodes"] = snap.nodes.size();
  doc["known_nodes"] = snap.knownNodes;
  doc["total_frames"] = snap.stats.totalPhysicalFrames;
  doc["crc_errors"] = snap.stats.crcErrors;
  doc["power_count"] = snap.stats.powerDataCount;
  doc["announce_count"] = snap.stats.announceCount;
  doc["nodetable_count"] = snap.stats.nodeTableCount;
  doc["uptime_s"] = snap.nowMs / 1000;
  doc["version"] = snap.cfg.swVersion;

  std::string payload;
  serializeJson(doc, payload);
  sink.enqueue("stats", payload);

  for (const auto &tap : snap.taps) {
    JsonDocument tapDoc;
    tapToJson(tapDoc.to<JsonObject>(), tap.gw, snap.nowMs, &tap.barcode);
    std::string tapPayload;
    serializeJson(tapDoc, tapPayload);
    sink.enqueue("tap/" + tapHexId(tap.id), tapPayload);
  }
}
