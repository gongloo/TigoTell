#include "TaskLog.h"

#include <WebSerial.h>

#include <cstdarg>

TaskLog taskLog;

void TaskLog::println(const char *line) {
  String s(line);
  while (s.endsWith("\n") || s.endsWith("\r"))
    s.remove(s.length() - 1);
  std::lock_guard<std::mutex> lock(mutex);
  if (lines.size() >= kMaxLines)
    lines.pop_front();
  lines.push_back(std::move(s));
  hasPending.store(true, std::memory_order_release);
}

void TaskLog::printf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  println(buf);
}

void TaskLog::flush() {
  if (!hasPending.load(std::memory_order_acquire))
    return;
  for (size_t i = 0; i < kMaxLines; ++i) {
    String line;
    bool more = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (lines.empty()) {
        hasPending.store(false, std::memory_order_release);
        return;
      }
      line = std::move(lines.front());
      lines.pop_front();
      more = !lines.empty();
      if (!more)
        hasPending.store(false, std::memory_order_release);
    }
    WebSerial.println(line);
    if (!more)
      return;
  }
}
