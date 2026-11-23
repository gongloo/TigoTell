#include "MqttPublisher.h"

#include <esp_timer.h>

#include "TaskLog.h"

namespace {

class PubSubMqttClient : public IMqttClient {
public:
  explicit PubSubMqttClient(PubSubClient &c) : client(c) {}

  void setServer(const char *host, uint16_t port) override {
    client.setServer(host, port);
  }

  bool connect(const char *clientId, const char *user, const char *pass,
               const char *willTopic, uint8_t willQos, bool willRetain,
               const char *willMessage) override {
    if (user && user[0] != '\0') {
      return client.connect(clientId, user, pass, willTopic, willQos,
                            willRetain, willMessage);
    }
    return client.connect(clientId, willTopic, willQos, willRetain,
                          willMessage);
  }

  bool connected() override { return client.connected(); }
  void disconnect() override { client.disconnect(); }

  bool publish(const char *topic, const char *payload, bool retained) override {
    return client.publish(topic, payload, retained);
  }

  bool subscribe(const char *topic) override { return client.subscribe(topic); }
  bool loop() override { return client.loop(); }
  int state() override { return client.state(); }

private:
  PubSubClient &client;
};

} // namespace

void MqttPublisher::begin() {
  // Discovery configs for a module are ~2 KB.
  client.setBufferSize(4096);
  client.setCallback([this](char *topic, uint8_t *payload, unsigned int len) {
    core.onMessage(topic, payload, len);
  });
  client.setKeepAlive(30);
  client.setSocketTimeout(5);
  // Core 0 runs the WiFi stack; loop() (and the RS485 reads) run on core 1.
  xTaskCreatePinnedToCore(taskEntry, "mqtt_task", 6144, this, 1, NULL, 0);
}

void MqttPublisher::configure(const Settings &settings) {
  MqttCore::Settings s;
  s.host = settings.host.c_str();
  s.port = settings.port;
  s.user = settings.user.c_str();
  s.pass = settings.pass.c_str();
  s.clientId = settings.clientId.c_str();
  s.baseTopic = settings.baseTopic.c_str();
  core.configure(s);
}

bool MqttPublisher::enqueue(const String &subTopic, const String &payload) {
  return core.enqueue(subTopic.c_str(), payload.c_str());
}

bool MqttPublisher::enqueueAbsolute(const String &topic, const String &payload,
                                    bool retain) {
  return core.enqueueAbsolute(topic.c_str(), payload.c_str(), retain);
}

void MqttPublisher::taskEntry(void *self) {
  static_cast<MqttPublisher *>(self)->run();
}

void MqttPublisher::run() {
  PubSubMqttClient adapter(client);
  for (;;) {
    core.step(adapter, static_cast<uint64_t>(esp_timer_get_time() / 1000),
              WiFi.status() == WL_CONNECTED,
              [](const char *msg) { taskLog.println(msg); });
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
