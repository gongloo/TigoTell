#ifndef TIGO_PROTOCOL_H
#define TIGO_PROTOCOL_H

#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// Enum for Frame Types
enum class TigoFrameType { PowerData, Announce, NodeTable };

// Abstract Base Class for Tigo Frames
class TigoFrame {
public:
  virtual ~TigoFrame() = default;
  virtual TigoFrameType getType() const = 0;
  virtual std::string
  toInfluxLineProtocol(const std::string &measurementName) const = 0;
  virtual std::string toString() const = 0;
};

// Class representing PV Module Data (Type 0x31)
class TigoPowerFrame : public TigoFrame {
public:
  uint16_t address;
  uint16_t pv_node_id;
  float voltage_in;
  float voltage_out;
  uint8_t duty_cycle;
  float current_in;
  float temperature;
  int rssi;

  TigoPowerFrame(uint16_t addr, uint16_t id, float vin, float vout,
                 uint8_t duty, float iin, float temp, int rssi_val);

  TigoFrameType getType() const override { return TigoFrameType::PowerData; }
  std::string
  toInfluxLineProtocol(const std::string &measurementName) const override;
  std::string toString() const override;
};

// Class representing Device Announcement (Type 0x09)
class TigoAnnounceFrame : public TigoFrame {
public:
  uint16_t address;
  uint16_t pv_node_id;
  uint32_t barcode_fragment; // 3 bytes usually

  TigoAnnounceFrame(uint16_t addr, uint16_t id, uint32_t barcode);

  TigoFrameType getType() const override { return TigoFrameType::Announce; }
  std::string
  toInfluxLineProtocol(const std::string &measurementName) const override;
  std::string toString() const override;
};

// Class representing Node Table / Command Response (Type 0x27)
class TigoNodeTableFrame : public TigoFrame {
public:
  struct NodeEntry {
    std::string barcode; // Long Address (8 bytes hex)
    uint16_t pv_node_id;
  };
  std::vector<NodeEntry> nodes;

  TigoNodeTableFrame(const std::vector<NodeEntry> &entries);

  TigoFrameType getType() const override { return TigoFrameType::NodeTable; }
  std::string
  toInfluxLineProtocol(const std::string &measurementName) const override;
  std::string toString() const override;
};

// Class for handling the byte stream and producing frames
class TigoParser {
public:
  struct Statistics {
    uint64_t totalPhysicalFrames = 0;
    uint64_t crcErrors = 0;
    uint64_t powerDataCount = 0;
    uint64_t announceCount = 0;
    uint64_t nodeTableCount = 0;
    uint64_t managementCount = 0;
  };

  TigoParser();

  // Feed a byte into the parser. Returns true if a complete frame was just
  // finished.
  void pushByte(uint8_t byte);

  // Feed multiple bytes into the parser.
  void pushBytes(const uint8_t *data, size_t length);

  // Check if there are parsed frames available
  bool hasFrames() const;

  // Retrieve and remove the next available frame
  std::unique_ptr<TigoFrame> popFrame();

  const Statistics &getStatistics() const { return stats; }
  std::string
  getStatisticsInfluxLineProtocol(const std::string &measurementName) const;

private:
  enum class State { WAIT_START, WAIT_FRAME_TYPE, READING, ESCAPE };

  State state;
  std::vector<uint8_t> buffer;
  Statistics stats;
  bool ignoreFirstCrcError;
  std::deque<std::unique_ptr<TigoFrame>> frameQueue;

  void processFrame(const std::vector<uint8_t> &frame);
  void parseTLVPayload(const std::vector<uint8_t> &payload);
  void parseManagementPayload(const std::vector<uint8_t> &payload);
  bool verifyCRC(const std::vector<uint8_t> &frame);
  uint16_t computeCRC(const uint8_t *data, size_t length);

  // Helper to extract bits across byte boundaries
  uint16_t get12Bits(const uint8_t *ptr, bool highNibbleFirst);
  int16_t getSigned12Bits(const uint8_t *ptr, bool highNibbleFirst);
};

#endif // TIGO_PROTOCOL_H
