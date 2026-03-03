#include "PowerStatsAccumulator.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

PowerStatsAccumulator::PowerStatsAccumulator() { reset(); }

void PowerStatsAccumulator::add(const TigoPowerFrame *frame) {
  float w = frame->voltage_out * frame->current_in;

  vin_sum += frame->voltage_in;
  vin_min = std::min(vin_min, frame->voltage_in);
  vin_max = std::max(vin_max, frame->voltage_in);

  vout_sum += frame->voltage_out;
  vout_min = std::min(vout_min, frame->voltage_out);
  vout_max = std::max(vout_max, frame->voltage_out);

  amp_sum += frame->current_in;
  amp_min = std::min(amp_min, frame->current_in);
  amp_max = std::max(amp_max, frame->current_in);

  watt_sum += w;
  watt_min = std::min(watt_min, w);
  watt_max = std::max(watt_max, w);

  temp_sum += frame->temperature;
  temp_min = std::min(temp_min, frame->temperature);
  temp_max = std::max(temp_max, frame->temperature);

  duty_sum += frame->duty_cycle;
  duty_min = std::min(duty_min, (int)frame->duty_cycle);
  duty_max = std::max(duty_max, (int)frame->duty_cycle);

  rssi_sum += frame->rssi;
  rssi_min = std::min(rssi_min, frame->rssi);
  rssi_max = std::max(rssi_max, frame->rssi);

  count++;
}

bool PowerStatsAccumulator::hasSamples() const { return count > 0; }

std::string
PowerStatsAccumulator::toInfluxLineProtocol(const std::string &measurementName,
                                            uint16_t address,
                                            uint16_t pv_node_id) const {
  if (count == 0)
    return "";

  std::stringstream ss;
  ss << measurementName << ",addr=" << std::dec << address;
  ss << ",node_id=" << std::setw(4) << std::setfill('0') << pv_node_id;

  ss << " "; // Space separator

  ss << "vin_avg=" << (vin_sum / count) << ",vin_min=" << vin_min
     << ",vin_max=" << vin_max << ",";
  ss << "vout_avg=" << (vout_sum / count) << ",vout_min=" << vout_min
     << ",vout_max=" << vout_max << ",";
  ss << "amp_avg=" << (amp_sum / count) << ",amp_min=" << amp_min
     << ",amp_max=" << amp_max << ",";
  ss << "watt_avg=" << (watt_sum / count) << ",watt_min=" << watt_min
     << ",watt_max=" << watt_max << ",";
  ss << "temp_avg=" << (temp_sum / count) << ",temp_min=" << temp_min
     << ",temp_max=" << temp_max << ",";
  ss << "rssi_avg=" << (rssi_sum / count) << "i,rssi_min=" << rssi_min
     << "i,rssi_max=" << rssi_max << "i,";
  ss << "duty_avg=" << (duty_sum / count) << "i,duty_min=" << duty_min
     << "i,duty_max=" << duty_max << "i,";
  ss << "samples=" << count << "i";

  return ss.str();
}

void PowerStatsAccumulator::reset() {
  count = 0;
  vin_sum = 0;
  vin_min = std::numeric_limits<float>::max();
  vin_max = std::numeric_limits<float>::lowest();
  vout_sum = 0;
  vout_min = std::numeric_limits<float>::max();
  vout_max = std::numeric_limits<float>::lowest();
  amp_sum = 0;
  amp_min = std::numeric_limits<float>::max();
  amp_max = std::numeric_limits<float>::lowest();
  watt_sum = 0;
  watt_min = std::numeric_limits<float>::max();
  watt_max = std::numeric_limits<float>::lowest();
  temp_sum = 0;
  temp_min = std::numeric_limits<float>::max();
  temp_max = std::numeric_limits<float>::lowest();
  duty_sum = 0;
  duty_min = std::numeric_limits<int>::max();
  duty_max = std::numeric_limits<int>::lowest();
  rssi_sum = 0;
  rssi_min = std::numeric_limits<int>::max();
  rssi_max = std::numeric_limits<int>::lowest();
}
