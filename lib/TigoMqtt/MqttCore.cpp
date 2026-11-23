#include "MqttCore.h"

#include <cstdio>
#include <cstring>
#include <utility>

void MqttCore::configure(const Settings &settings) {
  std::lock_guard<std::mutex> lock(mutex);
  pending = settings;
  hasPending = true;
  connected = false;
  enabled = !settings.host.empty() && settings.port != 0;
}

bool MqttCore::push(std::string topic, const std::string &payload,
                    bool retain) {
  size_t bytes = topic.length() + payload.length();
  if (queue.size() >= kMaxQueue || queuedBytes + bytes > kMaxQueueBytes)
    return false;
  queue.push_back({std::move(topic), payload, retain});
  queuedBytes += bytes;
  return true;
}

bool MqttCore::enqueue(const std::string &subTopic,
                       const std::string &payload) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!connected)
    return false;
  return push(baseTopic + "/" + subTopic, payload, false);
}

bool MqttCore::enqueueAbsolute(const std::string &topic,
                               const std::string &payload, bool retain) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!connected)
    return false;
  return push(topic, payload, retain);
}

void MqttCore::onMessage(const char *topic, const uint8_t *payload,
                         unsigned int length) {
  if (!topic || !payload)
    return;
  // Home Assistant announces "online" after a restart; it then needs the
  // discovery configs again.
  if (std::strcmp(topic, kHaStatusTopic) == 0 && length == 6 &&
      std::memcmp(payload, "online", 6) == 0) {
    discoveryGen++;
  }
}

void MqttCore::step(IMqttClient &client, uint64_t nowMs, bool wifiConnected,
                    const std::function<void(const char *)> &logFn) {
  // Take new settings under the lock, but do network I/O outside it: a
  // stalled socket must never block enqueue() in loop().
  bool apply = false;
  Settings next;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (hasPending) {
      next = pending;
      hasPending = false;
      apply = true;
    }
  }
  if (apply) {
    if (client.connected()) {
      client.publish(statusTopic.c_str(), "offline", true);
      client.disconnect();
    }
    active = next;
    connected = false;
    firstAttempt = true;
    {
      std::lock_guard<std::mutex> lock(mutex);
      baseTopic = active.baseTopic;
      queue.clear();
      queuedBytes = 0;
    }
  }

  bool wantConnection = !active.host.empty() && active.port != 0 && wifiConnected;

  if (!wantConnection) {
    if (client.connected())
      client.disconnect();
  } else if (!client.connected() &&
             (firstAttempt ||
              (nowMs >= lastAttemptMs &&
               nowMs - lastAttemptMs > kReconnectIntervalMs))) {
    firstAttempt = false;
    lastAttemptMs = nowMs;
    connect(client, logFn);
  }

  bool isUp = client.connected();
  if (connected && !isUp && logFn)
    logFn("MQTT: connection lost.");
  bool pendingNow = false;
  {
    std::lock_guard<std::mutex> lock(mutex);
    pendingNow = hasPending;
    connected = isUp && !pendingNow;
  }

  if (isUp && !pendingNow) {
    client.loop();
    for (;;) {
      Message msg;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.empty())
          break;
        msg = std::move(queue.front());
        queue.pop_front();
        queuedBytes -= msg.topic.length() + msg.payload.length();
      }
      if (client.publish(msg.topic.c_str(), msg.payload.c_str(), msg.retain))
        published++;
    }
  } else if (!isUp) {
    std::lock_guard<std::mutex> lock(mutex);
    queue.clear();
    queuedBytes = 0;
  }
}

void MqttCore::connect(IMqttClient &client,
                       const std::function<void(const char *)> &logFn) {
  statusTopic = active.baseTopic + "/status";
  // PubSubClient keeps the host pointer, so it must point at `active`.
  client.setServer(active.host.c_str(), active.port);

  bool ok = client.connect(
      active.clientId.c_str(),
      active.user.empty() ? nullptr : active.user.c_str(),
      active.user.empty() ? nullptr : active.pass.c_str(),
      statusTopic.c_str(), 0, true, "offline");

  if (ok) {
    client.publish(statusTopic.c_str(), "online", true);
    client.subscribe(kHaStatusTopic);
    discoveryGen++;
    if (logFn) {
      char buf[192];
      std::snprintf(buf, sizeof(buf), "MQTT: connected to %s:%u\n",
                    active.host.c_str(), active.port);
      logFn(buf);
    }
  } else if (logFn) {
    char buf[192];
    std::snprintf(buf, sizeof(buf),
                  "MQTT: connect to %s:%u failed (state %d)\n",
                  active.host.c_str(), active.port, client.state());
    logFn(buf);
  }
}
