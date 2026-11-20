#ifndef MQTT_PUBLISHER_H
#define MQTT_PUBLISHER_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include <atomic>
#include <deque>
#include <mutex>

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

  bool isEnabled() const { return enabled; }
  bool isConnected() const { return connected; }
  uint32_t publishedCount() const { return published; }
  // Increments on every (re)connect and whenever Home Assistant comes online,
  // i.e. whenever discovery configs should be (re)published.
  uint32_t discoveryGeneration() const { return discoveryGen; }

private:
  struct Message {
    String topic;
    String payload;
    bool retain;
  };
  static constexpr size_t kMaxQueue = 128;
  // Also bound by bytes, so a slow broker cannot eat the heap the web
  // server needs.
  static constexpr size_t kMaxQueueBytes = 24 * 1024;
  static constexpr const char *kHaStatusTopic = "homeassistant/status";
  static constexpr unsigned long kReconnectIntervalMs = 5000;

  static void taskEntry(void *self);
  void run();
  void connect();
  void onMessage(char *topic, uint8_t *payload, unsigned int length);
  bool push(String topic, const String &payload, bool retain);

  WiFiClient wifiClient;
  PubSubClient client{wifiClient};

  std::mutex mutex;              // guards pending, hasPending, queue
  Settings pending;              // written by configure()
  bool hasPending = false;
  Settings active;               // only touched by the MQTT task
  String statusTopic;            // only touched by the MQTT task
  std::deque<Message> queue;
  size_t queuedBytes = 0;        // guarded by mutex

  std::atomic<bool> enabled{false};
  std::atomic<bool> connected{false};
  std::atomic<uint32_t> published{0};
  std::atomic<uint32_t> discoveryGen{0};
  String baseTopic;              // guarded by mutex
};

#endif // MQTT_PUBLISHER_H
