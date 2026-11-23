#ifndef MQTT_PUBLISHER_H
#define MQTT_PUBLISHER_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "MqttCore.h"

// Publishes MQTT messages from a dedicated FreeRTOS task, so that broker
// connects (which block for seconds if the broker is unreachable) never stall
// the RS485 reads in loop().
class MqttPublisher {
public:
  struct Settings {
    String host;
    uint16_t port = 1883;
    String user;
    String pass;
    String clientId;
    String baseTopic;
  };

  void begin();

  // Thread-safe. Applies new settings and forces a reconnect.
  void configure(const Settings &settings);

  // Thread-safe. Queues payload for "<baseTopic>/<subTopic>". Messages are
  // dropped while disconnected so no stale backlog builds up.
  bool enqueue(const String &subTopic, const String &payload);
  // Same, but with a full topic (e.g. Home Assistant discovery) and retain.
  bool enqueueAbsolute(const String &topic, const String &payload,
                       bool retain);

  IMqttSink &sink() { return core; }

  bool isEnabled() const { return core.isEnabled(); }
  bool isConnected() const { return core.isConnected(); }
  uint32_t publishedCount() const { return core.publishedCount(); }
  // Increments on every (re)connect and whenever Home Assistant comes online,
  // i.e. whenever discovery configs should be (re)published.
  uint32_t discoveryGeneration() const { return core.discoveryGeneration(); }

private:
  static void taskEntry(void *self);
  void run();

  WiFiClient wifiClient;
  PubSubClient client{wifiClient};
  MqttCore core;
};

#endif // MQTT_PUBLISHER_H
