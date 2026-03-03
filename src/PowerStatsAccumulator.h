#ifndef POWER_STATS_ACCUMULATOR_H
#define POWER_STATS_ACCUMULATOR_H

#include <cstdint>
#include <limits>
#include <string>

#include "TigoProtocol.h"

class PowerStatsAccumulator {
public:
  PowerStatsAccumulator();

  void add(const TigoPowerFrame *frame);
  std::string toInfluxLineProtocol(const std::string &measurementName,
                                   uint16_t address, uint16_t pv_node_id) const;
  void reset();
  bool hasSamples() const;

private:
  uint32_t count;

  float vin_sum, vin_min, vin_max;
  float vout_sum, vout_min, vout_max;
  float amp_sum, amp_min, amp_max;
  float watt_sum, watt_min, watt_max;
  float temp_sum, temp_min, temp_max;
  int duty_sum, duty_min, duty_max;
  int rssi_sum, rssi_min, rssi_max;
};

#endif // POWER_STATS_ACCUMULATOR_H
