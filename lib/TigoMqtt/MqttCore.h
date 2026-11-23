#ifndef MQTT_CORE_H
#define MQTT_CORE_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>

// Minimal abstraction over an MQTT client library (e.g. PubSubClient) so the
// state machine, queuing, and reconnect logic can be tested natively.
class IMqttClient {
public:
  virtual ~IMqttClient() = default;
  virtual void setServer(const char *host, uint16_t port) = 0;
  // If user is nullptr or empty, connect anonymously without credentials.
  virtual bool connect(const char *clientId, const char *user,
                       const char *pass, const char *willTopic,
                       uint8_t willQos, bool willRetain,
                       const char *willMessage) = 0;
  virtual bool connected() = 0;
  virtual void disconnect() = 0;
  virtual bool publish(const char *topic, const char *payload,
                       bool retained) = 0;
  virtual bool subscribe(const char *topic) = 0;
  virtual bool loop() = 0;
  virtual int state() = 0;
};

// Sink interface used by higher-level telemetry/discovery publishers.
class IMqttSink {
public:
  virtual ~IMqttSink() = default;
  virtual bool enqueue(const std::string &subTopic,
                       const std::string &payload) = 0;
  virtual bool enqueueAbsolute(const std::string &topic,
                               const std::string &payload, bool retain) = 0;
  virtual uint32_t discoveryGeneration() const = 0;
};

// Pure C++17 MQTT connection/reconnect state machine and bounded message queue.
class MqttCore : public IMqttSink {
public:
  struct Settings {
    std::string host;
    uint16_t port = 1883;
    std::string user;
    std::string pass;
    std::string clientId;
    std::string baseTopic;
  };

  static constexpr size_t kMaxQueue = 128;
  // Also bound by bytes, so a slow broker cannot eat the heap the web
  // server needs.
  static constexpr size_t kMaxQueueBytes = 24 * 1024;
  static constexpr uint64_t kReconnectIntervalMs = 5000;
  static constexpr const char *kHaStatusTopic = "homeassistant/status";

  // Thread-safe. Applies new settings and forces a reconnect.
  void configure(const Settings &settings);

  // Thread-safe. Queues payload for "<baseTopic>/<subTopic>". Messages are
  // dropped while disconnected so no stale backlog builds up.
  bool enqueue(const std::string &subTopic,
               const std::string &payload) override;
  // Same, but with a full topic (e.g. Home Assistant discovery) and retain.
  bool enqueueAbsolute(const std::string &topic, const std::string &payload,
                       bool retain) override;

  // Handles incoming subscribed messages (e.g. Home Assistant birth message).
  void onMessage(const char *topic, const uint8_t *payload,
                 unsigned int length);

  // Advances the connection state machine and drains queued messages.
  // Call periodically from the MQTT worker task (or test loop).
  void step(IMqttClient &client, uint64_t nowMs, bool wifiConnected,
            const std::function<void(const char *)> &logFn = nullptr);

  bool isEnabled() const { return enabled.load(); }
  bool isConnected() const { return connected.load(); }
  uint32_t publishedCount() const { return published.load(); }
  // Increments on every (re)connect and whenever Home Assistant comes online,
  // i.e. whenever discovery configs should be (re)published.
  uint32_t discoveryGeneration() const override { return discoveryGen.load(); }

private:
  struct Message {
    std::string topic;
    std::string payload;
    bool retain;
  };

  bool push(std::string topic, const std::string &payload, bool retain);
  void connect(IMqttClient &client,
               const std::function<void(const char *)> &logFn);

  std::mutex mutex;       // guards pending, hasPending, queue, queuedBytes, baseTopic
  Settings pending;       // written by configure()
  bool hasPending = false;
  Settings active;        // only touched by step()/connect()
  std::string statusTopic; // only touched by step()/connect()
  std::deque<Message> queue;
  size_t queuedBytes = 0; // guarded by mutex

  uint64_t lastAttemptMs = 0;
  bool firstAttempt = true;

  std::atomic<bool> enabled{false};
  std::atomic<bool> connected{false};
  std::atomic<uint32_t> published{0};
  std::atomic<uint32_t> discoveryGen{0};
  std::string baseTopic; // guarded by mutex
};

#endif // MQTT_CORE_H
