#ifndef TASK_LOG_H
#define TASK_LOG_H

#include <Arduino.h>

#include <deque>
#include <mutex>

// WebSerial is not thread-safe: it appends to fixed buffers without locking,
// so writes from two tasks at once can overrun them and corrupt memory. Code
// running outside loop() logs here instead; loop() forwards the lines to
// WebSerial, which is then only ever used from one task.
class TaskLog {
public:
  void println(const char *line);
  void println(const String &line) { println(line.c_str()); }
  void printf(const char *fmt, ...) __attribute__((format(printf, 2, 3)));

  // Call from loop() only.
  void flush();

private:
  static constexpr size_t kMaxLines = 40;
  std::mutex mutex;
  std::deque<String> lines;
};

extern TaskLog taskLog;

#endif // TASK_LOG_H
