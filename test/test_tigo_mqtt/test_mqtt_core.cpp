#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "MqttCore.h"

namespace {

struct ConnectCall {
  std::string clientId;
  bool hasUser = false;
  std::string user;
  bool hasPass = false;
  std::string pass;
  std::string willTopic;
  uint8_t willQos = 0;
  bool willRetain = false;
  std::string willMessage;
  bool succeeded = false;
};

struct PublishedMessage {
  std::string topic;
  std::string payload;
  bool retained = false;
};

class FakeMqttClient : public IMqttClient {
public:
  void setServer(const char *host, uint16_t port) override {
    std::lock_guard<std::mutex> lock(mu);
    serverHost = host ? host : "";
    serverPort = port;
  }

  bool connect(const char *clientId, const char *user, const char *pass,
               const char *willTopic, uint8_t willQos, bool willRetain,
               const char *willMessage) override {
    std::function<void()> hook;
    bool ok = false;
    {
      std::lock_guard<std::mutex> lock(mu);
      ConnectCall call;
      call.clientId = clientId ? clientId : "";
      call.hasUser = (user != nullptr);
      call.user = user ? user : "";
      call.hasPass = (pass != nullptr);
      call.pass = pass ? pass : "";
      call.willTopic = willTopic ? willTopic : "";
      call.willQos = willQos;
      call.willRetain = willRetain;
      call.willMessage = willMessage ? willMessage : "";
      ok = connectShouldSucceed;
      call.succeeded = ok;
      connectCalls.push_back(std::move(call));
      isConnected = ok;
      hook = onConnectHook;
    }
    if (hook)
      hook();
    return ok;
  }

  bool connected() override {
    std::lock_guard<std::mutex> lock(mu);
    return isConnected;
  }

  void disconnect() override {
    std::lock_guard<std::mutex> lock(mu);
    isConnected = false;
    disconnectCount++;
  }

  bool publish(const char *topic, const char *payload, bool retained) override {
    std::function<void()> hook;
    bool ok = false;
    {
      std::lock_guard<std::mutex> lock(mu);
      if (failPublishAfter >= 0 &&
          publishedAttemptCount >= static_cast<size_t>(failPublishAfter)) {
        ok = false;
      } else {
        ok = publishShouldSucceed;
      }
      publishedAttemptCount++;
      if (ok) {
        published.push_back(
            {topic ? topic : "", payload ? payload : "", retained});
      }
      hook = onPublishHook;
    }
    if (hook)
      hook();
    return ok;
  }

  bool subscribe(const char *topic) override {
    std::lock_guard<std::mutex> lock(mu);
    subscribedTopics.push_back(topic ? topic : "");
    return true;
  }

  bool loop() override {
    std::lock_guard<std::mutex> lock(mu);
    loopCount++;
    return isConnected;
  }

  int state() override {
    std::lock_guard<std::mutex> lock(mu);
    return isConnected ? 0 : connectFailState;
  }

  void simulateBrokerDrop() {
    std::lock_guard<std::mutex> lock(mu);
    isConnected = false;
  }

  std::mutex mu;
  std::string serverHost;
  uint16_t serverPort = 0;
  bool connectShouldSucceed = true;
  int connectFailState = -2;
  bool publishShouldSucceed = true;
  int failPublishAfter = -1;
  size_t publishedAttemptCount = 0;
  bool isConnected = false;
  size_t disconnectCount = 0;
  size_t loopCount = 0;
  std::vector<ConnectCall> connectCalls;
  std::vector<PublishedMessage> published;
  std::vector<std::string> subscribedTopics;
  std::function<void()> onConnectHook;
  std::function<void()> onPublishHook;
};

MqttCore::Settings makeSettings(const std::string &host = "broker.local",
                                uint16_t port = 1883,
                                const std::string &baseTopic = "tigotell",
                                const std::string &user = "",
                                const std::string &pass = "",
                                const std::string &clientId = "tigotell-1") {
  MqttCore::Settings s;
  s.host = host;
  s.port = port;
  s.user = user;
  s.pass = pass;
  s.clientId = clientId;
  s.baseTopic = baseTopic;
  return s;
}

} // namespace

TEST(MqttCoreTest, DisabledByDefaultAndWhenHostEmptyOrPortZero) {
  MqttCore core;
  FakeMqttClient client;

  EXPECT_FALSE(core.isEnabled());
  EXPECT_FALSE(core.isConnected());
  EXPECT_EQ(core.publishedCount(), 0u);
  EXPECT_EQ(core.discoveryGeneration(), 0u);

  core.step(client, 1000, true);
  EXPECT_TRUE(client.connectCalls.empty());
  EXPECT_FALSE(core.isEnabled());
  EXPECT_FALSE(core.isConnected());

  // Empty host disables MQTT.
  core.configure(makeSettings("", 1883));
  EXPECT_FALSE(core.isEnabled());
  core.step(client, 2000, true);
  EXPECT_TRUE(client.connectCalls.empty());
  EXPECT_FALSE(core.isConnected());

  // Port 0 disables MQTT.
  core.configure(makeSettings("broker.local", 0));
  EXPECT_FALSE(core.isEnabled());
  core.step(client, 3000, true);
  EXPECT_TRUE(client.connectCalls.empty());
  EXPECT_FALSE(core.isConnected());

  // Valid configuration connects.
  core.configure(makeSettings("broker.local", 1883, "tigotell"));
  EXPECT_TRUE(core.isEnabled());
  core.step(client, 4000, true);
  EXPECT_TRUE(core.isConnected());
  ASSERT_EQ(client.connectCalls.size(), 1u);

  // Reconfiguring to empty host disconnects the already-connected client.
  core.configure(makeSettings("", 1883, "tigotell"));
  EXPECT_FALSE(core.isEnabled());
  EXPECT_FALSE(core.isConnected());
  core.step(client, 5000, true);
  EXPECT_EQ(client.disconnectCount, 1u);
  EXPECT_FALSE(client.isConnected);
  EXPECT_FALSE(core.isConnected());
  ASSERT_EQ(client.published.size(), 2u);
  EXPECT_EQ(client.published[1].topic, "tigotell/status");
  EXPECT_EQ(client.published[1].payload, "offline");
  EXPECT_TRUE(client.published[1].retained);

  // Reconnect and then disable via port == 0.
  core.configure(makeSettings("broker.local", 1883, "tigotell"));
  core.step(client, 6000, true);
  EXPECT_TRUE(core.isConnected());
  core.configure(makeSettings("broker.local", 0, "tigotell"));
  EXPECT_FALSE(core.isEnabled());
  core.step(client, 7000, true);
  EXPECT_EQ(client.disconnectCount, 2u);
  EXPECT_FALSE(core.isConnected());
}

TEST(MqttCoreTest, EnqueueRequiresConnectionAndFormatsTopics) {
  MqttCore core;
  FakeMqttClient client;

  EXPECT_FALSE(core.enqueue("stats", "{\"a\":1}"));
  EXPECT_FALSE(
      core.enqueueAbsolute("homeassistant/device/x/config", "{}", true));

  core.configure(makeSettings("broker.local", 1883, "solar/tigo"));
  EXPECT_FALSE(core.enqueue("stats", "{\"a\":1}"));

  core.step(client, 1000, true);
  ASSERT_TRUE(core.isConnected());
  client.published.clear();

  EXPECT_TRUE(core.enqueue("stats", "{\"total_power\":123}"));
  EXPECT_TRUE(core.enqueueAbsolute("homeassistant/device/t1/config",
                                   "{\"name\":\"TigoTell\"}", true));

  core.step(client, 1100, true);
  ASSERT_EQ(client.published.size(), 2u);

  EXPECT_EQ(client.published[0].topic, "solar/tigo/stats");
  EXPECT_EQ(client.published[0].payload, "{\"total_power\":123}");
  EXPECT_FALSE(client.published[0].retained);

  EXPECT_EQ(client.published[1].topic, "homeassistant/device/t1/config");
  EXPECT_EQ(client.published[1].payload, "{\"name\":\"TigoTell\"}");
  EXPECT_TRUE(client.published[1].retained);
  EXPECT_EQ(core.publishedCount(), 2u);
}

TEST(MqttCoreTest,
     ConnectWithCredentialsAndAnonymousPublishesBirthAndSubscribes) {
  MqttCore core;
  FakeMqttClient client;
  std::vector<std::string> logs;
  auto logFn = [&](const char *msg) { logs.emplace_back(msg); };

  // Authenticated connection.
  core.configure(makeSettings("mqtt.example.com", 1884, "home/solar", "alice",
                              "secret", "tigo-client-1"));
  core.step(client, 1000, true, logFn);

  EXPECT_TRUE(core.isConnected());
  EXPECT_EQ(client.serverHost, "mqtt.example.com");
  EXPECT_EQ(client.serverPort, 1884);
  ASSERT_EQ(client.connectCalls.size(), 1u);
  const auto &authCall = client.connectCalls[0];
  EXPECT_EQ(authCall.clientId, "tigo-client-1");
  EXPECT_TRUE(authCall.hasUser);
  EXPECT_EQ(authCall.user, "alice");
  EXPECT_TRUE(authCall.hasPass);
  EXPECT_EQ(authCall.pass, "secret");
  EXPECT_EQ(authCall.willTopic, "home/solar/status");
  EXPECT_EQ(authCall.willQos, 0);
  EXPECT_TRUE(authCall.willRetain);
  EXPECT_EQ(authCall.willMessage, "offline");

  ASSERT_EQ(client.published.size(), 1u);
  EXPECT_EQ(client.published[0].topic, "home/solar/status");
  EXPECT_EQ(client.published[0].payload, "online");
  EXPECT_TRUE(client.published[0].retained);

  ASSERT_EQ(client.subscribedTopics.size(), 1u);
  EXPECT_EQ(client.subscribedTopics[0], MqttCore::kHaStatusTopic);
  EXPECT_EQ(core.discoveryGeneration(), 1u);
  ASSERT_EQ(logs.size(), 1u);
  EXPECT_EQ(logs[0], "MQTT: connected to mqtt.example.com:1884\n");

  // Anonymous connection (empty user passes nullptr for both user and pass).
  core.configure(makeSettings("mqtt2.example.com", 1883, "anon/tigo", "",
                              "ignored_pass", "tigo-client-2"));
  core.step(client, 2000, true, logFn);

  ASSERT_EQ(client.connectCalls.size(), 2u);
  const auto &anonCall = client.connectCalls[1];
  EXPECT_EQ(anonCall.clientId, "tigo-client-2");
  EXPECT_FALSE(anonCall.hasUser);
  EXPECT_FALSE(anonCall.hasPass);
  EXPECT_EQ(anonCall.willTopic, "anon/tigo/status");
  EXPECT_EQ(anonCall.willQos, 0);
  EXPECT_TRUE(anonCall.willRetain);
  EXPECT_EQ(anonCall.willMessage, "offline");
  EXPECT_EQ(core.discoveryGeneration(), 2u);
}

TEST(MqttCoreTest, ReconnectBackoffAndImmediateAfterConfigure) {
  MqttCore core;
  FakeMqttClient client;
  client.connectShouldSucceed = false;
  client.connectFailState = -4;

  std::vector<std::string> logs;
  auto logFn = [&](const char *msg) { logs.emplace_back(msg); };

  core.configure(makeSettings("broker.local", 1883, "tigotell"));
  core.step(client, 1000, true, logFn);

  ASSERT_EQ(client.connectCalls.size(), 1u);
  EXPECT_FALSE(core.isConnected());
  ASSERT_EQ(logs.size(), 1u);
  EXPECT_EQ(logs[0], "MQTT: connect to broker.local:1883 failed (state -4)\n");

  // Earlier timestamp (clock skew) and exact kReconnectIntervalMs boundary do
  // not retry.
  core.step(client, 500, true, logFn);
  core.step(client, 1000 + MqttCore::kReconnectIntervalMs, true, logFn);
  EXPECT_EQ(client.connectCalls.size(), 1u);

  // Past kReconnectIntervalMs (> 5000 ms) retries.
  core.step(client, 1000 + MqttCore::kReconnectIntervalMs + 1, true, logFn);
  EXPECT_EQ(client.connectCalls.size(), 2u);
  EXPECT_EQ(logs.size(), 2u);

  // Calling configure() resets firstAttempt and connects immediately on next
  // step() without waiting kReconnectIntervalMs.
  client.connectShouldSucceed = true;
  core.configure(makeSettings("broker.local", 1883, "tigotell"));
  core.step(client, 1000 + MqttCore::kReconnectIntervalMs + 2, true, logFn);
  EXPECT_EQ(client.connectCalls.size(), 3u);
  EXPECT_TRUE(core.isConnected());
}

TEST(MqttCoreTest, WifiLossAndBrokerDropClearQueueAndMarkDisconnected) {
  MqttCore core;
  FakeMqttClient client;
  std::vector<std::string> logs;
  auto logFn = [&](const char *msg) { logs.emplace_back(msg); };

  core.configure(makeSettings("broker.local", 1883, "tigotell"));
  core.step(client, 1000, true, logFn);
  ASSERT_TRUE(core.isConnected());
  client.published.clear();

  // Queue messages while connected, then lose WiFi before step().
  EXPECT_TRUE(core.enqueue("stats", "stale-wifi-drop"));
  core.step(client, 1100, false, logFn);

  EXPECT_FALSE(core.isConnected());
  EXPECT_FALSE(client.isConnected);
  EXPECT_EQ(client.disconnectCount, 1u);
  EXPECT_TRUE(client.published.empty());
  EXPECT_FALSE(core.enqueue("stats", "while-no-wifi"));

  // Recover WiFi: stale message must stay cleared and never be published.
  core.step(client, 1000 + MqttCore::kReconnectIntervalMs + 100, true, logFn);
  ASSERT_TRUE(core.isConnected());
  for (const auto &msg : client.published) {
    EXPECT_NE(msg.payload, "stale-wifi-drop");
  }
  client.published.clear();
  logs.clear();

  // Enqueue while connected, then simulate unexpected broker drop.
  EXPECT_TRUE(core.enqueue("stats", "stale-broker-drop"));
  client.simulateBrokerDrop();
  core.step(client, 8000, true, logFn);

  EXPECT_FALSE(core.isConnected());
  EXPECT_TRUE(client.published.empty());
  ASSERT_EQ(logs.size(), 1u);
  EXPECT_EQ(logs[0], "MQTT: connection lost.");

  // Reconnect after backoff: stale message was cleared.
  core.step(client, 8000 + MqttCore::kReconnectIntervalMs + 1, true, logFn);
  ASSERT_TRUE(core.isConnected());
  for (const auto &msg : client.published) {
    EXPECT_NE(msg.payload, "stale-broker-drop");
  }
}

TEST(MqttCoreTest,
     ConfigureDisconnectsWithOfflineWillUpdatesBaseTopicAndAvoidsTransientConnected) {
  MqttCore core;
  FakeMqttClient client;

  core.configure(makeSettings("old.broker", 1883, "old/base"));
  core.step(client, 1000, true);
  ASSERT_TRUE(core.isConnected());
  client.published.clear();

  // Enqueue stale message on old base topic.
  EXPECT_TRUE(core.enqueue("stats", "stale-before-reconfig"));

  // Reconfigure while connected: immediately flips isConnected() to false so
  // concurrent enqueue() calls drop instead of queueing on stale settings.
  core.configure(makeSettings("new.broker", 1884, "new/base"));
  EXPECT_FALSE(core.isConnected());
  EXPECT_FALSE(core.enqueue("stats", "between-configure-and-step"));

  core.step(client, 1100, true);
  ASSERT_TRUE(core.isConnected());

  // Should have published "offline" on old status topic, disconnected,
  // connected to new.broker, and published "online" on new status topic.
  EXPECT_EQ(client.disconnectCount, 1u);
  EXPECT_EQ(client.serverHost, "new.broker");
  EXPECT_EQ(client.serverPort, 1884);
  ASSERT_GE(client.connectCalls.size(), 2u);
  EXPECT_EQ(client.connectCalls.back().willTopic, "new/base/status");

  ASSERT_EQ(client.published.size(), 2u);
  EXPECT_EQ(client.published[0].topic, "old/base/status");
  EXPECT_EQ(client.published[0].payload, "offline");
  EXPECT_TRUE(client.published[0].retained);

  EXPECT_EQ(client.published[1].topic, "new/base/status");
  EXPECT_EQ(client.published[1].payload, "online");
  EXPECT_TRUE(client.published[1].retained);

  // Subsequent enqueue uses updated baseTopic.
  client.published.clear();
  EXPECT_TRUE(core.enqueue("stats", "fresh"));
  core.step(client, 1200, true);
  ASSERT_EQ(client.published.size(), 1u);
  EXPECT_EQ(client.published[0].topic, "new/base/stats");
  EXPECT_EQ(client.published[0].payload, "fresh");

  // If configure() arrives during step() while connect() is executing,
  // hasPending is true at the end of step(), so isConnected() MUST NOT
  // transiently become true.
  bool observedConnectedDuringPending = false;
  client.onConnectHook = [&]() {
    core.configure(makeSettings("third.broker", 1885, "third/base"));
    if (core.isConnected())
      observedConnectedDuringPending = true;
  };
  core.configure(makeSettings("second.broker", 1884, "second/base"));
  core.step(client, 1300, true);
  EXPECT_FALSE(observedConnectedDuringPending);
  EXPECT_FALSE(core.isConnected());
  client.onConnectHook = nullptr;

  // Next step applies third.broker and connects cleanly.
  core.step(client, 1400, true);
  EXPECT_TRUE(core.isConnected());
  EXPECT_EQ(client.serverHost, "third.broker");
  EXPECT_EQ(client.serverPort, 1885);
}

TEST(MqttCoreTest, QueueCountAndByteLimitsEnforcedAndFreedOnDrain) {
  MqttCore core;
  FakeMqttClient client;

  core.configure(makeSettings("broker.local", 1883, "t"));
  core.step(client, 1000, true);
  ASSERT_TRUE(core.isConnected());
  client.published.clear();

  // Count limit: kMaxQueue (128) messages accepted, 129th rejected.
  for (size_t i = 0; i < MqttCore::kMaxQueue; ++i) {
    EXPECT_TRUE(core.enqueueAbsolute("a", "", false)) << "index=" << i;
  }
  EXPECT_FALSE(core.enqueueAbsolute("a", "", false));

  core.step(client, 1100, true);
  EXPECT_EQ(client.published.size(), MqttCore::kMaxQueue);
  EXPECT_EQ(core.publishedCount(), MqttCore::kMaxQueue);

  // Byte limit: kMaxQueueBytes (24 * 1024) counting topic.size() +
  // payload.size().
  client.published.clear();
  const std::string topic = "t"; // 1 byte
  const std::string chunk(1023, 'x'); // 1 + 1023 = 1024 bytes per message
  constexpr size_t kMessagesFor24K = MqttCore::kMaxQueueBytes / 1024;
  static_assert(kMessagesFor24K <= MqttCore::kMaxQueue,
                "Must hit byte limit before count limit");
  for (size_t i = 0; i < kMessagesFor24K; ++i) {
    EXPECT_TRUE(core.enqueueAbsolute(topic, chunk, false)) << "chunk=" << i;
  }
  // Queue is at exact byte limit: even a 1-byte message must fail.
  EXPECT_FALSE(core.enqueueAbsolute("x", "", false));

  // Drain frees capacity.
  core.step(client, 1200, true);
  EXPECT_EQ(client.published.size(), kMessagesFor24K);
  EXPECT_TRUE(core.enqueueAbsolute("x", "", false));

  // publishedCount() increments only when client.publish() returns true.
  core.step(client, 1300, true);
  uint32_t beforeFailCount = core.publishedCount();
  client.publishShouldSucceed = false;
  EXPECT_TRUE(core.enqueue("stats", "fail-1"));
  EXPECT_TRUE(core.enqueue("stats", "fail-2"));
  core.step(client, 1400, true);
  EXPECT_EQ(core.publishedCount(), beforeFailCount);

  client.publishShouldSucceed = true;
  EXPECT_TRUE(core.enqueue("stats", "ok-1"));
  core.step(client, 1500, true);
  EXPECT_EQ(core.publishedCount(), beforeFailCount + 1);
}

TEST(MqttCoreTest, OnMessageTriggersDiscoveryGenOnlyOnHomeAssistantOnline) {
  MqttCore core;
  uint32_t gen = core.discoveryGeneration();

  const auto callOnMsg = [&](const char *topic, const char *payload,
                             unsigned int len) {
    core.onMessage(topic, reinterpret_cast<const uint8_t *>(payload), len);
  };

  callOnMsg("homeassistant/status", "offline", 7);
  EXPECT_EQ(core.discoveryGeneration(), gen);

  callOnMsg("homeassistant/status", "online!", 7);
  EXPECT_EQ(core.discoveryGeneration(), gen);

  callOnMsg("homeassistant/status", "onlin", 5);
  EXPECT_EQ(core.discoveryGeneration(), gen);

  callOnMsg("other/topic", "online", 6);
  EXPECT_EQ(core.discoveryGeneration(), gen);

  callOnMsg(nullptr, "online", 6);
  EXPECT_EQ(core.discoveryGeneration(), gen);

  core.onMessage("homeassistant/status", nullptr, 6);
  EXPECT_EQ(core.discoveryGeneration(), gen);

  callOnMsg("homeassistant/status", "online", 6);
  EXPECT_EQ(core.discoveryGeneration(), gen + 1);

  // Non-null-terminated buffer of exact length 6 also matches.
  const uint8_t raw[9] = {'o', 'n', 'l', 'i', 'n', 'e', 'X', 'Y', 'Z'};
  core.onMessage("homeassistant/status", raw, 6);
  EXPECT_EQ(core.discoveryGeneration(), gen + 2);
}

TEST(MqttCoreTest, ConcurrentConfigureEnqueueStepStressTest) {
  MqttCore core;
  FakeMqttClient client;
  core.configure(makeSettings("broker-a.local", 1883, "baseA"));

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> stepIterations{0};

  std::thread worker([&]() {
    uint64_t now = 1000;
    while (!stop.load()) {
      core.step(client, now, true);
      now += 100;
      stepIterations.fetch_add(1);
    }
    // Final drain after writers finish.
    core.step(client, now + 10000, true);
  });

  std::thread enqueuer1([&]() {
    for (int i = 0; i < 400; ++i) {
      core.enqueue("stats", "{\"i\":" + std::to_string(i) + "}");
      core.enqueueAbsolute("homeassistant/device/stress/config", "{}", true);
    }
  });

  std::thread enqueuer2([&]() {
    for (int i = 0; i < 400; ++i) {
      core.enqueue("node/1", "{\"v\":12.3}");
      core.onMessage("homeassistant/status",
                     reinterpret_cast<const uint8_t *>("online"), 6);
    }
  });

  std::thread configurer([&]() {
    for (int i = 0; i < 40; ++i) {
      if (i % 2 == 0) {
        core.configure(makeSettings("broker-a.local", 1883, "baseA"));
      } else {
        core.configure(makeSettings("broker-b.local", 1884, "baseB"));
      }
      std::this_thread::yield();
    }
  });

  enqueuer1.join();
  enqueuer2.join();
  configurer.join();
  stop.store(true);
  worker.join();

  EXPECT_GT(stepIterations.load(), 0u);

  // Reconfigure to a deterministic final state and verify everything remains
  // healthy and uncorrupted.
  core.configure(makeSettings("final.local", 1883, "finalBase"));
  core.step(client, 1000000, true);
  EXPECT_TRUE(core.isConnected());
  EXPECT_TRUE(core.enqueue("stats", "final-ok"));
  core.step(client, 1000100, true);

  std::lock_guard<std::mutex> lock(client.mu);
  ASSERT_FALSE(client.published.empty());
  EXPECT_EQ(client.published.back().topic, "finalBase/stats");
  EXPECT_EQ(client.published.back().payload, "final-ok");

  for (const auto &msg : client.published) {
    bool validPrefix = msg.topic.rfind("baseA/", 0) == 0 ||
                       msg.topic.rfind("baseB/", 0) == 0 ||
                       msg.topic.rfind("finalBase/", 0) == 0 ||
                       msg.topic == "homeassistant/device/stress/config";
    EXPECT_TRUE(validPrefix) << "Unexpected topic: " << msg.topic;
  }
}
