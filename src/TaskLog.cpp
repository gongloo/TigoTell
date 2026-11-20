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
  std::deque<String> pending;
  {
    std::lock_guard<std::mutex> lock(mutex);
    pending.swap(lines);
  }
  for (const String &line : pending)
    WebSerial.println(line);
}
