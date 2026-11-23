#include <ArduinoJson.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "MqttCore.h"
#include "TigoMqtt.h"

namespace {

struct RecordedMessage {
  std::string topic;
  std::string payload;
  bool retain = false;
};

class RecordingMqttSink : public IMqttSink {
public:
  bool enqueue(const std::string &subTopic,
               const std::string &payload) override {
    if (!acceptMessages)
      return false;
    messages.push_back({baseTopic + "/" + subTopic, payload, false});
    return true;
  }

  bool enqueueAbsolute(const std::string &topic, const std::string &payload,
                       bool retain) override {
    if (!acceptMessages)
      return false;
    messages.push_back({topic, payload, retain});
    return true;
  }

  uint32_t discoveryGeneration() const override { return gen; }

  const RecordedMessage *findByTopic(const std::string &topic) const {
    for (const auto &m : messages) {
      if (m.topic == topic)
        return &m;
    }
    return nullptr;
  }

  std::vector<RecordedMessage>
  findAllWithPrefix(const std::string &prefix) const {
    std::vector<RecordedMessage> out;
    for (const auto &m : messages) {
      if (m.topic.rfind(prefix, 0) == 0)
        out.push_back(m);
    }
    return out;
  }

  std::string baseTopic = "tigotell";
  uint32_t gen = 1;
  bool acceptMessages = true;
  std::vector<RecordedMessage> messages;
};

class MinimalFakeMqttClient : public IMqttClient {
public:
  void setServer(const char *, uint16_t) override {}

  bool connect(const char *, const char *, const char *, const char *, uint8_t,
               bool, const char *) override {
    isConnected = true;
    return true;
  }

  bool connected() override { return isConnected; }

  void disconnect() override { isConnected = false; }

  bool publish(const char *topic, const char *payload, bool retained) override {
    published.push_back({topic ? topic : "", payload ? payload : "", retained});
    return true;
  }

  bool subscribe(const char *) override { return true; }

  bool loop() override { return isConnected; }

  int state() override { return 0; }

  bool isConnected = false;
  std::vector<RecordedMessage> published;
};

MqttConfigSnapshot makeConfig(const std::string &topic = "tigotell",
                              bool discovery = true) {
  MqttConfigSnapshot cfg;
  cfg.discovery = discovery;
  cfg.topic = topic;
  cfg.interval = 30;
  cfg.hostname = "TigoTell-Lab";
  cfg.swVersion = "1.2.3";
  cfg.repoUrl = "https://github.com/example/TigoTell";
  cfg.localIp = "192.168.1.50";
  return cfg;
}

MqttNodeSnapshot makeNode(uint16_t id, uint16_t addr,
                          const std::string &barcode) {
  MqttNodeSnapshot n{};
  n.pv_node_id = id;
  n.address = addr;
  n.barcode = barcode;
  n.voltage_in = 38.5f;
  n.voltage_out = 36.0f;
  n.current_in = 5.5f;
  n.temperature = 41.2f;
  n.duty_cycle = 92;
  n.rssi = 180;
  n.age_s = 4;
  return n;
}

TigoGatewayStatus makeActiveGateway(uint16_t id, uint64_t lastSeenMs) {
  TigoGatewayStatus gw;
  gw.id = id;
  gw.lastSeenMs = lastSeenMs;
  gw.seen = true;
  gw.polls = 100;
  gw.answered = 95;
  gw.missed = 5;
  gw.recentBits = 0x3FFULL; // 10/10 answered
  gw.recentCount = 10;
  gw.rxBuffersUsed = 2;
  gw.rxBuffersMax = 16;
  gw.txBuffersFree = 12;
  gw.txBuffersMin = 8;
  gw.slotCounter = 77;
  gw.packets = 420;
  gw.powerReports = 390;
  gw.radioChannel = 19;
  gw.panId = 0x4A5B;
  gw.version = "v4.1.0";
  return gw;
}

} // namespace

TEST(TigoMqttTest, TapHelpersAndJsonSerialization) {
  EXPECT_EQ(tapHexId(0x0001), "0001");
  EXPECT_EQ(tapHexId(0x1A2B), "1A2B");
  EXPECT_EQ(tapHexId(0x0000), "0000");
  EXPECT_EQ(tapHexId(0xFFFF), "FFFF");

  // tapState edge cases.
  TigoGatewayStatus gw;
  gw.id = 0x0001;
  EXPECT_STREQ(tapState(gw, 1000), "offline"); // !gw.seen

  gw.seen = true;
  gw.lastSeenMs = 100000;
  EXPECT_STREQ(tapState(gw, 100000), "ok"); // recentCount == 0
  EXPECT_STREQ(tapState(gw, 100000 + kTapOfflineMs), "ok");
  EXPECT_STREQ(tapState(gw, 100000 + kTapOfflineMs + 1), "offline");

  // Underflow guard (now < lastSeenMs).
  EXPECT_STREQ(tapState(gw, 50000), "ok");

  // Degraded (< 90%) vs ok (>= 90%): 8/9 = 88% -> degraded, 9/10 = 90% -> ok.
  gw.recentCount = 9;
  gw.recentBits = 0b011111111ULL; // 8/9 = 88%
  EXPECT_STREQ(tapState(gw, 100500), "degraded");
  gw.recentCount = 10;
  gw.recentBits = 0b0111111111ULL; // 9/10 = 90%
  EXPECT_STREQ(tapState(gw, 100500), "ok");

  // Full tapToJson serialization with 64-bit timestamps past 49.7 days.
  constexpr uint64_t kPast49Days = 0x100000000ULL + 5000000ULL;
  TigoGatewayStatus fullGw = makeActiveGateway(0x1A2B, kPast49Days - 12345);
  fullGw.nodes[0x0010] = {kPast49Days - 1000, 140};
  fullGw.nodes[0x0011] = {kPast49Days - 2000, 180};
  // Stale node outside kTapModuleWindowMs (300000 ms) must be excluded.
  fullGw.nodes[0x0012] = {kPast49Days - kTapModuleWindowMs - 1, 250};

  std::string barcode = "4-TAP123A";
  JsonDocument doc;
  tapToJson(doc.to<JsonObject>(), fullGw, kPast49Days, &barcode);

  EXPECT_EQ(doc["id"].as<std::string>(), "1A2B");
  EXPECT_EQ(doc["state"].as<std::string>(), "ok");
  EXPECT_EQ(doc["last_seen_s"].as<uint64_t>(), 12ULL);
  EXPECT_EQ(doc["polls"].as<uint32_t>(), 100u);
  EXPECT_EQ(doc["answered"].as<uint32_t>(), 95u);
  EXPECT_EQ(doc["missed"].as<uint32_t>(), 5u);
  EXPECT_EQ(doc["recent_answered_pct"].as<int>(), 100);
  EXPECT_EQ(doc["rx_buffers_used"].as<int>(), 2);
  EXPECT_EQ(doc["rx_buffers_max"].as<int>(), 16);
  EXPECT_EQ(doc["tx_buffers_free"].as<int>(), 12);
  EXPECT_EQ(doc["tx_buffers_min"].as<int>(), 8);
  EXPECT_EQ(doc["slot_counter"].as<int>(), 77);
  EXPECT_EQ(doc["packets"].as<uint32_t>(), 420u);
  EXPECT_EQ(doc["power_reports"].as<uint32_t>(), 390u);
  EXPECT_EQ(doc["modules"].as<int>(), 2);
  EXPECT_EQ(doc["rssi_avg"].as<int>(), 160);
  EXPECT_EQ(doc["rssi_min"].as<int>(), 140);
  EXPECT_EQ(doc["barcode"].as<std::string>(), "4-TAP123A");
  EXPECT_EQ(doc["version"].as<std::string>(), "v4.1.0");
  EXPECT_EQ(doc["radio_channel"].as<int>(), 19);
  EXPECT_EQ(doc["pan_id"].as<std::string>(), "4A5B");

  // Underflow guard when now < lastSeenMs and now < node.lastMs.
  TigoGatewayStatus underflowGw = makeActiveGateway(0x1A2B, kPast49Days);
  underflowGw.nodes[0x0010] = {kPast49Days, 150};
  JsonDocument underflowDoc;
  tapToJson(underflowDoc.to<JsonObject>(), underflowGw, kPast49Days - 20000,
            nullptr);
  EXPECT_EQ(underflowDoc["last_seen_s"].as<uint64_t>(), 0ULL);
  EXPECT_EQ(underflowDoc["state"].as<std::string>(), "ok");
  EXPECT_TRUE(underflowDoc["barcode"].isNull());
  EXPECT_EQ(underflowDoc["modules"].as<int>(), 1);

  // Unseen minimal gateway omits optional fields.
  TigoGatewayStatus emptyGw;
  emptyGw.id = 0x0007;
  JsonDocument minDoc;
  tapToJson(minDoc.to<JsonObject>(), emptyGw, 1000, nullptr);
  EXPECT_EQ(minDoc["id"].as<std::string>(), "0007");
  EXPECT_EQ(minDoc["state"].as<std::string>(), "offline");
  EXPECT_TRUE(minDoc["last_seen_s"].isNull());
  EXPECT_TRUE(minDoc["recent_answered_pct"].isNull());
  EXPECT_TRUE(minDoc["rx_buffers_used"].isNull());
  EXPECT_TRUE(minDoc["tx_buffers_free"].isNull());
  EXPECT_TRUE(minDoc["tx_buffers_min"].isNull());
  EXPECT_TRUE(minDoc["slot_counter"].isNull());
  EXPECT_EQ(minDoc["modules"].as<int>(), 0);
  EXPECT_TRUE(minDoc["rssi_avg"].isNull());
  EXPECT_TRUE(minDoc["rssi_min"].isNull());
  EXPECT_TRUE(minDoc["barcode"].isNull());
  EXPECT_TRUE(minDoc["version"].isNull());
  EXPECT_TRUE(minDoc["radio_channel"].isNull());
  EXPECT_TRUE(minDoc["pan_id"].isNull());

  // Gateway with only expired modules (> kTapModuleWindowMs) and negative
  // txBuffersMin omits rssi_avg/rssi_min and tx_buffers_min.
  TigoGatewayStatus expiredModulesGw = makeActiveGateway(0x0008, 500000);
  expiredModulesGw.txBuffersMin = -1;
  expiredModulesGw.nodes[0x0020] = {500000 - kTapModuleWindowMs - 1, 120};
  JsonDocument expiredDoc;
  tapToJson(expiredDoc.to<JsonObject>(), expiredModulesGw, 500000, nullptr);
  EXPECT_EQ(expiredDoc["modules"].as<int>(), 0);
  EXPECT_TRUE(expiredDoc["rssi_avg"].isNull());
  EXPECT_TRUE(expiredDoc["rssi_min"].isNull());
  EXPECT_EQ(expiredDoc["tx_buffers_free"].as<int>(), 12);
  EXPECT_TRUE(expiredDoc["tx_buffers_min"].isNull());
}

TEST(TigoMqttTest, SystemNodeAndTapDiscoverySchemas) {
  TigoMqttPublisher pub;
  pub.setDeviceId("tigotell_aabbcc");
  EXPECT_EQ(pub.deviceId(), "tigotell_aabbcc");

  RecordingMqttSink sink;
  sink.baseTopic = "solar/tigo";
  sink.gen = 1;

  MqttPublishSnapshot snap;
  snap.cfg = makeConfig("solar/tigo", true);
  snap.nowMs = 50000;
  snap.nodes.push_back(makeNode(42, 0x0042, "4-MOD042A"));
  snap.nodes.push_back(makeNode(43, 0x0043, "")); // empty barcode -> no sn
  snap.taps.push_back(
      {0x0001, makeActiveGateway(0x0001, 49000), "4-TAP001B"});
  snap.taps.push_back({0x0002, makeActiveGateway(0x0002, 49000), ""});

  pub.publish(snap, sink);

  // 1. System discovery.
  const RecordedMessage *sysMsg =
      sink.findByTopic("homeassistant/device/tigotell_aabbcc/config");
  ASSERT_NE(sysMsg, nullptr);
  EXPECT_TRUE(sysMsg->retain);
  EXPECT_LT(sysMsg->payload.size(), 4096u);

  JsonDocument sysDoc;
  ASSERT_EQ(deserializeJson(sysDoc, sysMsg->payload), DeserializationError::Ok);
  EXPECT_EQ(sysDoc["o"]["name"].as<std::string>(), "TigoTell");
  EXPECT_EQ(sysDoc["o"]["sw"].as<std::string>(), "1.2.3");
  EXPECT_EQ(sysDoc["o"]["url"].as<std::string>(),
            "https://github.com/example/TigoTell");
  EXPECT_EQ(sysDoc["stat_t"].as<std::string>(), "solar/tigo/stats");
  EXPECT_EQ(sysDoc["avty_t"].as<std::string>(), "solar/tigo/status");
  EXPECT_EQ(sysDoc["exp_aft"].as<int>(), 3 * 30 + 60);
  EXPECT_EQ(sysDoc["dev"]["ids"].as<std::string>(), "tigotell_aabbcc");
  EXPECT_EQ(sysDoc["dev"]["name"].as<std::string>(), "TigoTell-Lab");
  EXPECT_EQ(sysDoc["dev"]["mdl"].as<std::string>(), "TigoTell");
  EXPECT_EQ(sysDoc["dev"]["sw"].as<std::string>(), "1.2.3");
  EXPECT_EQ(sysDoc["dev"]["cu"].as<std::string>(), "http://192.168.1.50/");

  JsonObject sysCmps = sysDoc["cmps"].as<JsonObject>();
  ASSERT_EQ(sysCmps.size(), 6u);

  EXPECT_EQ(sysCmps["tigotell_aabbcc_total_power"]["dev_cla"].as<std::string>(),
            "power");
  EXPECT_EQ(
      sysCmps["tigotell_aabbcc_total_power"]["unit_of_meas"].as<std::string>(),
      "W");
  EXPECT_EQ(
      sysCmps["tigotell_aabbcc_total_power"]["stat_cla"].as<std::string>(),
      "measurement");
  EXPECT_TRUE(sysCmps["tigotell_aabbcc_total_power"]["ent_cat"].isNull());

  EXPECT_EQ(
      sysCmps["tigotell_aabbcc_active_nodes"]["stat_cla"].as<std::string>(),
      "measurement");
  EXPECT_TRUE(sysCmps["tigotell_aabbcc_active_nodes"]["ent_cat"].isNull());

  EXPECT_EQ(sysCmps["tigotell_aabbcc_known_nodes"]["ent_cat"].as<std::string>(),
            "diagnostic");
  EXPECT_EQ(
      sysCmps["tigotell_aabbcc_crc_errors"]["stat_cla"].as<std::string>(),
      "total_increasing");
  EXPECT_EQ(sysCmps["tigotell_aabbcc_crc_errors"]["ent_cat"].as<std::string>(),
            "diagnostic");
  EXPECT_EQ(
      sysCmps["tigotell_aabbcc_total_frames"]["stat_cla"].as<std::string>(),
      "total_increasing");
  EXPECT_EQ(sysCmps["tigotell_aabbcc_uptime_s"]["dev_cla"].as<std::string>(),
            "duration");
  EXPECT_EQ(
      sysCmps["tigotell_aabbcc_uptime_s"]["unit_of_meas"].as<std::string>(),
      "s");
  EXPECT_EQ(sysCmps["tigotell_aabbcc_uptime_s"]["ent_cat"].as<std::string>(),
            "diagnostic");

  // 2. Module discovery (with barcode and without barcode).
  const RecordedMessage *node42Msg =
      sink.findByTopic("homeassistant/device/tigotell_aabbcc_node_42/config");
  ASSERT_NE(node42Msg, nullptr);
  EXPECT_TRUE(node42Msg->retain);
  EXPECT_LT(node42Msg->payload.size(), 4096u);

  JsonDocument node42Doc;
  ASSERT_EQ(deserializeJson(node42Doc, node42Msg->payload),
            DeserializationError::Ok);
  EXPECT_EQ(node42Doc["stat_t"].as<std::string>(), "solar/tigo/node/42");
  EXPECT_EQ(node42Doc["dev"]["via_device"].as<std::string>(), "tigotell_aabbcc");
  EXPECT_EQ(node42Doc["dev"]["sn"].as<std::string>(), "4-MOD042A");
  EXPECT_EQ(node42Doc["dev"]["mf"].as<std::string>(), "Tigo");

  JsonObject n42Cmps = node42Doc["cmps"].as<JsonObject>();
  ASSERT_EQ(n42Cmps.size(), 7u);
  EXPECT_EQ(n42Cmps["tigotell_aabbcc_node_42_power"]["unit_of_meas"]
                .as<std::string>(),
            "W");
  EXPECT_EQ(n42Cmps["tigotell_aabbcc_node_42_voltage_in"]["dev_cla"]
                .as<std::string>(),
            "voltage");
  EXPECT_EQ(n42Cmps["tigotell_aabbcc_node_42_voltage_out"]["unit_of_meas"]
                .as<std::string>(),
            "V");
  EXPECT_EQ(n42Cmps["tigotell_aabbcc_node_42_current_in"]["unit_of_meas"]
                .as<std::string>(),
            "A");
  EXPECT_EQ(n42Cmps["tigotell_aabbcc_node_42_temperature"]["unit_of_meas"]
                .as<std::string>(),
            "°C");
  EXPECT_EQ(n42Cmps["tigotell_aabbcc_node_42_duty_cycle"]["ent_cat"]
                .as<std::string>(),
            "diagnostic");

  // RSSI must NOT have dev_cla or unit_of_meas (raw 0..255).
  JsonObject rssiCmp = n42Cmps["tigotell_aabbcc_node_42_rssi"];
  EXPECT_EQ(rssiCmp["p"].as<std::string>(), "sensor");
  EXPECT_TRUE(rssiCmp["dev_cla"].isNull());
  EXPECT_TRUE(rssiCmp["unit_of_meas"].isNull());
  EXPECT_EQ(rssiCmp["stat_cla"].as<std::string>(), "measurement");
  EXPECT_EQ(rssiCmp["ent_cat"].as<std::string>(), "diagnostic");

  const RecordedMessage *node43Msg =
      sink.findByTopic("homeassistant/device/tigotell_aabbcc_node_43/config");
  ASSERT_NE(node43Msg, nullptr);
  JsonDocument node43Doc;
  ASSERT_EQ(deserializeJson(node43Doc, node43Msg->payload),
            DeserializationError::Ok);
  EXPECT_TRUE(node43Doc["dev"]["sn"].isNull());

  // 3. TAP discovery.
  const RecordedMessage *tap1Msg = sink.findByTopic(
      "homeassistant/device/tigotell_aabbcc_tap_0001/config");
  ASSERT_NE(tap1Msg, nullptr);
  EXPECT_TRUE(tap1Msg->retain);
  EXPECT_LT(tap1Msg->payload.size(), 4096u);

  JsonDocument tap1Doc;
  ASSERT_EQ(deserializeJson(tap1Doc, tap1Msg->payload),
            DeserializationError::Ok);
  EXPECT_EQ(tap1Doc["stat_t"].as<std::string>(), "solar/tigo/tap/0001");
  EXPECT_EQ(tap1Doc["dev"]["via_device"].as<std::string>(), "tigotell_aabbcc");
  EXPECT_EQ(tap1Doc["dev"]["sn"].as<std::string>(), "4-TAP001B");

  JsonObject tap1Cmps = tap1Doc["cmps"].as<JsonObject>();
  ASSERT_EQ(tap1Cmps.size(), 7u);
  JsonObject onlineCmp = tap1Cmps["tigotell_aabbcc_tap_0001_online"];
  EXPECT_EQ(onlineCmp["p"].as<std::string>(), "binary_sensor");
  EXPECT_EQ(onlineCmp["dev_cla"].as<std::string>(), "connectivity");
  EXPECT_EQ(onlineCmp["val_tpl"].as<std::string>(),
            "{{ 'OFF' if value_json.state == 'offline' else 'ON' }}");

  EXPECT_EQ(tap1Cmps["tigotell_aabbcc_tap_0001_recent_answered_pct"]
                    ["unit_of_meas"]
                        .as<std::string>(),
            "%");
  EXPECT_FALSE(tap1Cmps["tigotell_aabbcc_tap_0001_modules"].isNull());
  EXPECT_FALSE(tap1Cmps["tigotell_aabbcc_tap_0001_rssi_avg"].isNull());
  EXPECT_FALSE(tap1Cmps["tigotell_aabbcc_tap_0001_rx_buffers_used"].isNull());
  EXPECT_FALSE(tap1Cmps["tigotell_aabbcc_tap_0001_tx_buffers_free"].isNull());
  EXPECT_EQ(
      tap1Cmps["tigotell_aabbcc_tap_0001_missed"]["stat_cla"].as<std::string>(),
      "total_increasing");

  const RecordedMessage *tap2Msg = sink.findByTopic(
      "homeassistant/device/tigotell_aabbcc_tap_0002/config");
  ASSERT_NE(tap2Msg, nullptr);
  JsonDocument tap2Doc;
  ASSERT_EQ(deserializeJson(tap2Doc, tap2Msg->payload),
            DeserializationError::Ok);
  EXPECT_TRUE(tap2Doc["dev"]["sn"].isNull());
}

TEST(TigoMqttTest, TelemetryPayloadsPublishModulesStatsAndTaps) {
  TigoMqttPublisher pub;
  pub.setDeviceId("tigotell_aabbcc");

  RecordingMqttSink sink;
  sink.baseTopic = "solar/tigo";

  MqttPublishSnapshot snap;
  snap.cfg = makeConfig("solar/tigo", false); // telemetry only
  snap.nowMs = 123456;
  snap.knownNodes = 10;
  snap.stats.totalPhysicalFrames = 5000;
  snap.stats.crcErrors = 12;
  snap.stats.powerDataCount = 4000;
  snap.stats.announceCount = 30;
  snap.stats.nodeTableCount = 5;

  MqttNodeSnapshot n1 = makeNode(10, 0x0010, "4-ABC123");
  n1.voltage_out = 40.0f;
  n1.current_in = 5.0f; // power = 200.0
  MqttNodeSnapshot n2 = makeNode(11, 0x0011, "");
  n2.voltage_out = 30.0f;
  n2.current_in = 4.0f; // power = 120.0
  snap.nodes = {n1, n2};

  TigoGatewayStatus unseenUnpolled;
  unseenUnpolled.id = 0x0008;
  unseenUnpolled.seen = false;
  unseenUnpolled.polls = 0;

  TigoGatewayStatus polledOffline;
  polledOffline.id = 0x0009;
  polledOffline.seen = false;
  polledOffline.polls = 14;
  polledOffline.missed = 14;

  snap.taps.push_back({0x0001, makeActiveGateway(0x0001, 120000), "4-TAP1"});
  snap.taps.push_back({0x0008, unseenUnpolled, "4-SKIP"});
  snap.taps.push_back({0x0009, polledOffline, ""});

  pub.publish(snap, sink);

  // Node 10 payload.
  const RecordedMessage *n1Msg = sink.findByTopic("solar/tigo/node/10");
  ASSERT_NE(n1Msg, nullptr);
  EXPECT_FALSE(n1Msg->retain);
  JsonDocument n1Doc;
  ASSERT_EQ(deserializeJson(n1Doc, n1Msg->payload), DeserializationError::Ok);
  EXPECT_EQ(n1Doc["pv_node_id"].as<uint16_t>(), 10);
  EXPECT_EQ(n1Doc["address"].as<uint16_t>(), 0x0010);
  EXPECT_EQ(n1Doc["barcode"].as<std::string>(), "4-ABC123");
  EXPECT_FLOAT_EQ(n1Doc["power"].as<float>(), 200.0f);
  EXPECT_FLOAT_EQ(n1Doc["voltage_in"].as<float>(), 38.5f);
  EXPECT_FLOAT_EQ(n1Doc["voltage_out"].as<float>(), 40.0f);
  EXPECT_FLOAT_EQ(n1Doc["current_in"].as<float>(), 5.0f);
  EXPECT_FLOAT_EQ(n1Doc["temperature"].as<float>(), 41.2f);
  EXPECT_EQ(n1Doc["duty_cycle"].as<uint8_t>(), 92);
  EXPECT_EQ(n1Doc["rssi"].as<int>(), 180);
  EXPECT_EQ(n1Doc["age_s"].as<uint64_t>(), 4ULL);

  // Node 11 omits empty barcode.
  const RecordedMessage *n2Msg = sink.findByTopic("solar/tigo/node/11");
  ASSERT_NE(n2Msg, nullptr);
  JsonDocument n2Doc;
  ASSERT_EQ(deserializeJson(n2Doc, n2Msg->payload), DeserializationError::Ok);
  EXPECT_TRUE(n2Doc["barcode"].isNull());
  EXPECT_FLOAT_EQ(n2Doc["power"].as<float>(), 120.0f);

  // Stats payload.
  const RecordedMessage *statsMsg = sink.findByTopic("solar/tigo/stats");
  ASSERT_NE(statsMsg, nullptr);
  EXPECT_FALSE(statsMsg->retain);
  JsonDocument statsDoc;
  ASSERT_EQ(deserializeJson(statsDoc, statsMsg->payload),
            DeserializationError::Ok);
  EXPECT_FLOAT_EQ(statsDoc["total_power"].as<float>(), 320.0f);
  EXPECT_EQ(statsDoc["active_nodes"].as<size_t>(), 2u);
  EXPECT_EQ(statsDoc["known_nodes"].as<size_t>(), 10u);
  EXPECT_EQ(statsDoc["total_frames"].as<uint64_t>(), 5000ULL);
  EXPECT_EQ(statsDoc["crc_errors"].as<uint64_t>(), 12ULL);
  EXPECT_EQ(statsDoc["power_count"].as<uint64_t>(), 4000ULL);
  EXPECT_EQ(statsDoc["announce_count"].as<uint64_t>(), 30ULL);
  EXPECT_EQ(statsDoc["nodetable_count"].as<uint64_t>(), 5ULL);
  EXPECT_EQ(statsDoc["uptime_s"].as<uint64_t>(), 123ULL);
  EXPECT_EQ(statsDoc["version"].as<std::string>(), "1.2.3");

  // TAP payloads.
  const RecordedMessage *tap1Msg = sink.findByTopic("solar/tigo/tap/0001");
  ASSERT_NE(tap1Msg, nullptr);
  EXPECT_FALSE(tap1Msg->retain);
  JsonDocument tap1Doc;
  ASSERT_EQ(deserializeJson(tap1Doc, tap1Msg->payload),
            DeserializationError::Ok);
  EXPECT_EQ(tap1Doc["state"].as<std::string>(), "ok");
  EXPECT_EQ(tap1Doc["barcode"].as<std::string>(), "4-TAP1");

  const RecordedMessage *tap9Msg = sink.findByTopic("solar/tigo/tap/0009");
  ASSERT_NE(tap9Msg, nullptr);
  JsonDocument tap9Doc;
  ASSERT_EQ(deserializeJson(tap9Doc, tap9Msg->payload),
            DeserializationError::Ok);
  EXPECT_EQ(tap9Doc["state"].as<std::string>(), "offline");
  EXPECT_EQ(tap9Doc["polls"].as<uint32_t>(), 14u);

  // Unseen & unpolled TAP (0008) must be skipped for telemetry and discovery.
  EXPECT_EQ(sink.findByTopic("solar/tigo/tap/0008"), nullptr);

  sink.messages.clear();
  snap.cfg.discovery = true;
  pub.publish(snap, sink);
  EXPECT_EQ(sink.findByTopic("homeassistant/device/tigotell_aabbcc_tap_0008/config"),
            nullptr);
  EXPECT_NE(sink.findByTopic("homeassistant/device/tigotell_aabbcc_tap_0001/config"),
            nullptr);
  EXPECT_NE(sink.findByTopic("homeassistant/device/tigotell_aabbcc_tap_0009/config"),
            nullptr);

  // Empty nodes & taps snapshot publishes zero active_nodes and total_power.
  sink.messages.clear();
  MqttPublishSnapshot emptySnap;
  emptySnap.cfg = makeConfig("solar/tigo", false);
  emptySnap.nowMs = 200000;
  pub.publish(emptySnap, sink);
  ASSERT_EQ(sink.messages.size(), 1u);
  EXPECT_EQ(sink.messages[0].topic, "solar/tigo/stats");
  JsonDocument emptyStatsDoc;
  ASSERT_EQ(deserializeJson(emptyStatsDoc, sink.messages[0].payload),
            DeserializationError::Ok);
  EXPECT_FLOAT_EQ(emptyStatsDoc["total_power"].as<float>(), 0.0f);
  EXPECT_EQ(emptyStatsDoc["active_nodes"].as<size_t>(), 0u);
}

TEST(TigoMqttTest,
     QueueSaturationPrioritizesDiscoveryAndRetriesUntilAllPanelsPublished) {
  MqttCore core;
  MinimalFakeMqttClient client;
  MqttCore::Settings settings;
  settings.host = "broker.local";
  settings.port = 1883;
  settings.clientId = "tigotell-sat";
  settings.baseTopic = "tigotell";
  core.configure(settings);
  core.step(client, 1000, true);
  ASSERT_TRUE(core.isConnected());
  client.published.clear();

  TigoMqttPublisher pub;
  pub.setDeviceId("tigotell_aabbcc");

  MqttPublishSnapshot snap;
  snap.cfg = makeConfig("tigotell", true);
  snap.nowMs = 60000;
  for (uint16_t i = 1; i <= 30; ++i) {
    snap.nodes.push_back(
        makeNode(i, i, "4-NODE" + std::to_string(1000 + i)));
  }
  for (uint16_t t = 1; t <= 2; ++t) {
    snap.taps.push_back(
        {t, makeActiveGateway(t, 59000), "4-TAP" + std::to_string(t)});
  }

  // Tick 1: 30 nodes + 2 TAPs + 1 system (~60 KB) exceeds MqttCore's 24 KB
  // queue. Discovery is sent before telemetry, so discovery fills the queue
  // and no telemetry starves discovery.
  pub.publish(snap, core);
  core.step(client, 2000, true);

  size_t tick1DiscoveryCount = 0;
  size_t tick1DiscoveryBytes = 0;
  for (const auto &msg : client.published) {
    if (msg.topic.rfind("homeassistant/device/", 0) == 0) {
      tick1DiscoveryCount++;
      tick1DiscoveryBytes += msg.topic.size() + msg.payload.size();
    }
  }
  // Discovery fills the 24 KB queue up to the size of the next discovery
  // payload (~1.5 KB), so only small tail-end telemetry can fit in the
  // remaining slack and telemetry never starves discovery.
  EXPECT_GT(tick1DiscoveryCount, 0u);
  EXPECT_LT(tick1DiscoveryCount, 33u);
  EXPECT_GE(tick1DiscoveryBytes, MqttCore::kMaxQueueBytes - 2048u);

  // Ticks 2..6: drain remaining discovery configs cleanly with zero drops and
  // zero duplicates.
  for (int tick = 2; tick <= 6; ++tick) {
    snap.nowMs += 30000;
    pub.publish(snap, core);
    core.step(client, 2000 + tick * 100, true);
  }

  std::set<std::string> seenDiscoveryTopics;
  std::vector<std::string> allDiscoveryTopics;
  for (const auto &msg : client.published) {
    if (msg.topic.rfind("homeassistant/device/", 0) == 0) {
      seenDiscoveryTopics.insert(msg.topic);
      allDiscoveryTopics.push_back(msg.topic);
    }
  }

  // 1 system + 30 modules + 2 TAPs = 33 unique discovery configs, published
  // exactly once each.
  EXPECT_EQ(seenDiscoveryTopics.size(), 33u);
  EXPECT_EQ(allDiscoveryTopics.size(), 33u);

  // Subsequent tick now publishes all telemetry without any discovery messages.
  client.published.clear();
  snap.nowMs += 30000;
  pub.publish(snap, core);
  core.step(client, 5000, true);

  size_t finalDiscovery = 0;
  size_t finalTelemetry = 0;
  for (const auto &msg : client.published) {
    if (msg.topic.rfind("homeassistant/device/", 0) == 0) {
      finalDiscovery++;
    } else {
      finalTelemetry++;
    }
  }
  EXPECT_EQ(finalDiscovery, 0u);
  EXPECT_EQ(finalTelemetry, 33u); // 30 nodes + 1 stats + 2 TAPs
}

TEST(TigoMqttTest, LateBarcodeArrivalReannouncesOnce) {
  TigoMqttPublisher pub;
  pub.setDeviceId("tigotell_aabbcc");

  RecordingMqttSink sink;
  sink.gen = 1;

  MqttPublishSnapshot snap;
  snap.cfg = makeConfig("tigotell", true);
  snap.nowMs = 10000;
  snap.nodes.push_back(makeNode(101, 0x0101, ""));
  snap.taps.push_back({0x0002, makeActiveGateway(0x0002, 9500), ""});

  // 1st publish: empty barcodes -> announced without sn.
  pub.publish(snap, sink);
  auto d1 = sink.findAllWithPrefix("homeassistant/device/");
  ASSERT_EQ(d1.size(), 3u);
  for (const auto &m : d1) {
    JsonDocument doc;
    ASSERT_EQ(deserializeJson(doc, m.payload), DeserializationError::Ok);
    EXPECT_TRUE(doc["dev"]["sn"].isNull());
  }

  // 2nd publish: still empty barcodes -> NOT re-announced.
  sink.messages.clear();
  pub.publish(snap, sink);
  EXPECT_TRUE(sink.findAllWithPrefix("homeassistant/device/").empty());

  // 3rd publish: barcodes arrive -> module and TAP re-announced with sn,
  // system NOT re-announced.
  sink.messages.clear();
  snap.nodes[0].barcode = "4-MOD101X";
  snap.taps[0].barcode = "4-TAP002Y";
  pub.publish(snap, sink);
  auto d3 = sink.findAllWithPrefix("homeassistant/device/");
  ASSERT_EQ(d3.size(), 2u);

  const RecordedMessage *modMsg =
      sink.findByTopic("homeassistant/device/tigotell_aabbcc_node_101/config");
  ASSERT_NE(modMsg, nullptr);
  JsonDocument modDoc;
  ASSERT_EQ(deserializeJson(modDoc, modMsg->payload), DeserializationError::Ok);
  EXPECT_EQ(modDoc["dev"]["sn"].as<std::string>(), "4-MOD101X");

  const RecordedMessage *tapMsg = sink.findByTopic(
      "homeassistant/device/tigotell_aabbcc_tap_0002/config");
  ASSERT_NE(tapMsg, nullptr);
  JsonDocument tapDoc;
  ASSERT_EQ(deserializeJson(tapDoc, tapMsg->payload), DeserializationError::Ok);
  EXPECT_EQ(tapDoc["dev"]["sn"].as<std::string>(), "4-TAP002Y");

  // 4th publish: same barcodes -> NOT re-announced.
  sink.messages.clear();
  pub.publish(snap, sink);
  EXPECT_TRUE(sink.findAllWithPrefix("homeassistant/device/").empty());
}

TEST(TigoMqttTest, DiscoveryGenerationChangeAndToggleReannouncesAll) {
  TigoMqttPublisher pub;
  pub.setDeviceId("tigotell_aabbcc");

  RecordingMqttSink sink;
  sink.gen = 1;

  MqttPublishSnapshot snap;
  snap.cfg = makeConfig("tigotell", true);
  snap.nowMs = 10000;
  snap.nodes.push_back(makeNode(1, 0x0001, "4-M1"));
  snap.taps.push_back({0x0001, makeActiveGateway(0x0001, 9500), "4-T1"});

  // Initial publish (gen = 1).
  pub.publish(snap, sink);
  EXPECT_EQ(sink.findAllWithPrefix("homeassistant/device/").size(), 3u);

  // Same gen -> no discovery.
  sink.messages.clear();
  pub.publish(snap, sink);
  EXPECT_TRUE(sink.findAllWithPrefix("homeassistant/device/").empty());

  // Discovery generation bump (reconnect / HA "online") -> re-announces all.
  sink.messages.clear();
  sink.gen = 2;
  pub.publish(snap, sink);
  EXPECT_EQ(sink.findAllWithPrefix("homeassistant/device/").size(), 3u);

  // Disable discovery -> clears internal discovery state; telemetry still sent.
  sink.messages.clear();
  snap.cfg.discovery = false;
  pub.publish(snap, sink);
  EXPECT_TRUE(sink.findAllWithPrefix("homeassistant/device/").empty());
  EXPECT_EQ(sink.messages.size(), 3u); // node/1, stats, tap/0001

  // Re-enable discovery with the SAME generation -> re-announces all.
  sink.messages.clear();
  snap.cfg.discovery = true;
  pub.publish(snap, sink);
  EXPECT_EQ(sink.findAllWithPrefix("homeassistant/device/").size(), 3u);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
