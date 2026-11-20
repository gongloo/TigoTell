#include "MqttPublisher.h"

#include "TaskLog.h"

void MqttPublisher::begin() {
  // Discovery configs for a module are ~2 KB.
  client.setBufferSize(4096);
  client.setCallback([this](char *topic, uint8_t *payload, unsigned int len) {
    onMessage(topic, payload, len);
  });
  client.setKeepAlive(30);
  client.setSocketTimeout(5);
  // Core 0 runs the WiFi stack; loop() (and the RS485 reads) run on core 1.
  xTaskCreatePinnedToCore(taskEntry, "mqtt_task", 6144, this, 1, NULL, 0);
}

void MqttPublisher::configure(const Settings &settings) {
  std::lock_guard<std::mutex> lock(mutex);
  pending = settings;
  hasPending = true;
  baseTopic = settings.baseTopic;
  enabled = settings.host.length() > 0 && settings.port != 0;
}

bool MqttPublisher::push(String topic, const String &payload, bool retain) {
  size_t bytes = topic.length() + payload.length();
  if (queue.size() >= kMaxQueue || queuedBytes + bytes > kMaxQueueBytes)
    return false;
  queue.push_back({std::move(topic), payload, retain});
  queuedBytes += bytes;
  return true;
}

bool MqttPublisher::enqueue(const String &subTopic, const String &payload) {
  if (!connected)
    return false;
  std::lock_guard<std::mutex> lock(mutex);
  return push(baseTopic + "/" + subTopic, payload, false);
}

bool MqttPublisher::enqueueAbsolute(const String &topic, const String &payload,
                                    bool retain) {
  if (!connected)
    return false;
  std::lock_guard<std::mutex> lock(mutex);
  return push(topic, payload, retain);
}

void MqttPublisher::onMessage(char *topic, uint8_t *payload,
                              unsigned int length) {
  // Home Assistant announces "online" after a restart; it then needs the
  // discovery configs again.
  if (strcmp(topic, kHaStatusTopic) == 0 && length == 6 &&
      memcmp(payload, "online", 6) == 0) {
    discoveryGen++;
  }
}

void MqttPublisher::taskEntry(void *self) {
  static_cast<MqttPublisher *>(self)->run();
}

void MqttPublisher::run() {
  unsigned long lastAttempt = 0;
  bool firstAttempt = true;

  for (;;) {
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
    }

    bool wantConnection = active.host.length() > 0 && active.port != 0 &&
                          WiFi.status() == WL_CONNECTED;

    if (!wantConnection) {
      if (client.connected())
        client.disconnect();
    } else if (!client.connected() &&
               (firstAttempt || millis() - lastAttempt > kReconnectIntervalMs)) {
      firstAttempt = false;
      lastAttempt = millis();
      connect();
    }

    bool isUp = client.connected();
    if (connected && !isUp)
      taskLog.println("MQTT: connection lost.");
    connected = isUp;

    if (isUp) {
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
        if (client.publish(msg.topic.c_str(), msg.payload.c_str(),
                           msg.retain))
          published++;
      }
    } else {
      std::lock_guard<std::mutex> lock(mutex);
      queue.clear();
      queuedBytes = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void MqttPublisher::connect() {
  statusTopic = active.baseTopic + "/status";
  // PubSubClient keeps the host pointer, so it must point at `active`.
  client.setServer(active.host.c_str(), active.port);

  bool ok;
  if (active.user.length() > 0) {
    ok = client.connect(active.clientId.c_str(), active.user.c_str(),
                        active.pass.c_str(), statusTopic.c_str(), 0, true,
                        "offline");
  } else {
    ok = client.connect(active.clientId.c_str(), statusTopic.c_str(), 0, true,
                        "offline");
  }

  if (ok) {
    client.publish(statusTopic.c_str(), "online", true);
    client.subscribe(kHaStatusTopic);
    discoveryGen++;
    taskLog.printf("MQTT: connected to %s:%u\n", active.host.c_str(),
                   active.port);
  } else {
    taskLog.printf("MQTT: connect to %s:%u failed (state %d)\n",
                   active.host.c_str(), active.port, client.state());
  }
}
