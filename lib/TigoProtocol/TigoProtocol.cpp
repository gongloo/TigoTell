#include "TigoProtocol.h"

#include <cstdio>
#include <cstring>

// --- TigoPowerFrame Implementation ---

TigoPowerFrame::TigoPowerFrame(uint16_t addr, uint16_t id, float vin,
                               float vout, uint8_t duty, float iin, float temp,
                               int rssi_val)
    : address(addr), pv_node_id(id), voltage_in(vin), voltage_out(vout),
      duty_cycle(duty), current_in(iin), temperature(temp), rssi(rssi_val) {}

std::string
TigoPowerFrame::toInfluxLineProtocol(const std::string &measurementName) const {
  std::stringstream ss;
  // Measurement and Tags
  ss << measurementName << ",addr=" << std::dec << address;
  ss << ",node_id=" << std::setw(4) << std::setfill('0') << pv_node_id;

  // Fields
  ss << " "; // Space separator
  ss << "vin=" << voltage_in << ",";
  ss << "vout=" << voltage_out << ",";
  ss << "amp=" << current_in << ",";
  ss << "watt=" << (voltage_out * current_in) << ",";
  ss << "temp=" << temperature << ",";
  ss << "rssi=" << rssi << ",";
  ss << "duty=" << (int)duty_cycle;

  return ss.str();
}

std::string TigoPowerFrame::toString() const {
  char buffer[128];
  snprintf(buffer, sizeof(buffer),
           "Addr:%u ID:%04X Vin:%.2f Vout:%.2f I:%.2f T:%.1f", address,
           pv_node_id, voltage_in, voltage_out, current_in, temperature);
  return std::string(buffer);
}

// --- TigoAnnounceFrame Implementation ---

TigoAnnounceFrame::TigoAnnounceFrame(uint16_t addr, uint16_t id,
                                     uint32_t barcode)
    : address(addr), pv_node_id(id), barcode_fragment(barcode) {}

std::string TigoAnnounceFrame::toInfluxLineProtocol(
    const std::string &measurementName) const {
  std::stringstream ss;
  ss << measurementName << "_announce,addr=" << std::dec << address;
  ss << " node_id=" << std::setw(4) << std::setfill('0') << pv_node_id << ",";
  ss << "barcode_frag=" << std::hex << barcode_fragment;
  return ss.str();
}

std::string TigoAnnounceFrame::toString() const {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "Announce Addr:%u ID:%04X BarcodeFrag:%06X",
           address, pv_node_id, barcode_fragment);
  return std::string(buffer);
}

// --- TigoNodeTableFrame Implementation ---

TigoNodeTableFrame::TigoNodeTableFrame(const std::vector<NodeEntry> &entries)
    : nodes(entries) {}

std::string TigoNodeTableFrame::toInfluxLineProtocol(
    const std::string &measurementName) const {
  std::stringstream ss;
  // Since this frame contains multiple entries, we generate multiple lines
  // separated by newline
  bool first = true;
  for (const auto &entry : nodes) {
    if (!first)
      ss << "\n";
    first = false;
    ss << measurementName << "_map,addr=" << std::dec << entry.pv_node_id;
    ss << " barcode=\"" << entry.barcode << "\"";
  }
  return ss.str();
}

std::string TigoNodeTableFrame::toString() const {
  std::stringstream ss;
  ss << "NodeTable (" << nodes.size() << " entries): ";
  for (size_t i = 0; i < nodes.size(); i++) {
    if (i > 0)
      ss << ", ";
    ss << "[" << nodes[i].barcode << "->" << std::dec << nodes[i].pv_node_id
       << "]";
    if (i >= 2) {
      ss << "...";
      break;
    } // Truncate for display
  }
  return ss.str();
}

std::string TigoParser::getStatisticsInfluxLineProtocol(
    const std::string &measurementName) const {
  std::stringstream ss;
  ss << measurementName << "_stats "
     << "total_frames=" << stats.totalPhysicalFrames << "i,"
     << "crc_errors=" << stats.crcErrors << "i,"
     << "power_count=" << stats.powerDataCount << "i,"
     << "announce_count=" << stats.announceCount << "i,"
     << "nodetable_count=" << stats.nodeTableCount << "i,"
     << "management_count=" << stats.managementCount << "i";
  return ss.str();
}

// --- TigoParser Implementation ---

// CRC Table (CRC-16/CCITT 0x8408 reflected)
static const uint16_t CRC_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48,
    0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7, 0x1081, 0x0108,
    0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E, 0x9CC9, 0x8D40, 0xBFDB,
    0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876, 0x2102, 0x308B, 0x0210, 0x1399,
    0x6726, 0x76AF, 0x4434, 0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E,
    0xFAE7, 0xC87C, 0xD9F5, 0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E,
    0x54B5, 0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD,
    0xC974, 0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3, 0x5285,
    0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A, 0xDECD, 0xCF44,
    0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72, 0x6306, 0x728F, 0x4014,
    0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9, 0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5,
    0xA96A, 0xB8E3, 0x8A78, 0x9BF1, 0x7387, 0x620E, 0x5095, 0x411C, 0x35A3,
    0x242A, 0x16B1, 0x0738, 0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862,
    0x9AF9, 0x8B70, 0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E,
    0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1,
    0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E, 0xA50A, 0xB483,
    0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5, 0x2942, 0x38CB, 0x0A50,
    0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD, 0xB58B, 0xA402, 0x9699, 0x8710,
    0xF3AF, 0xE226, 0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7,
    0x6E6E, 0x5CF5, 0x4D7C, 0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1,
    0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72,
    0x3EFB, 0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A, 0xE70E,
    0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1, 0x6B46, 0x7ACF,
    0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9, 0xF78F, 0xE606, 0xD49D,
    0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C,
    0x3DE3, 0x2C6A, 0x1EF1, 0x0F78};

TigoParser::TigoParser() : state(State::WAIT_START), ignoreFirstCrcError(true) {
  buffer.reserve(256);
}
void TigoParser::pushByte(uint8_t byte) {
  // Safety break for very large buffers
  if (buffer.size() > 1024) {
    buffer.clear();
    state = State::WAIT_START;
  }

  switch (state) {
  case State::WAIT_START:
    if (byte == 0x7E) {
      state = State::WAIT_FRAME_TYPE;
    }
    break;

  case State::WAIT_FRAME_TYPE:
    if (byte == 0x07) {
      // Confirmed start of a new frame
      buffer.clear();
      state = State::READING;
    } else {
      // Not a valid start sequence, reset
      state = State::WAIT_START;
      // Re-evaluate the current byte in case it's a start delimiter
      if (byte == 0x7E) {
        state = State::WAIT_FRAME_TYPE;
      }
    }
    break;

  case State::READING:
    if (byte == 0x7E) {
      // Potential end of frame or an escape sequence
      state = State::ESCAPE;
    } else {
      buffer.push_back(byte);
    }
    break;

  case State::ESCAPE:
    if (byte == 0x08) {
      // Confirmed end of frame
      processFrame(buffer);
      state = State::WAIT_START;
    } else {
      // It's an escaped data byte
      uint8_t unescaped;
      bool is_known_escape = true;
      switch (byte) {
      case 0x00:
        unescaped = 0x7E;
        break;
      case 0x01:
        unescaped = 0x24;
        break;
      case 0x02:
        unescaped = 0x23;
        break;
      case 0x03:
        unescaped = 0x25;
        break;
      case 0x04:
        unescaped = 0xA4;
        break;
      case 0x05:
        unescaped = 0xA3;
        break;
      case 0x06:
        unescaped = 0xA5;
        break;
      default:
        is_known_escape = false;
        break;
      }

      if (is_known_escape) {
        buffer.push_back(unescaped);
      } else {
        // Per reference, if unknown, keep both bytes
        buffer.push_back(0x7E);
        buffer.push_back(byte);
      }
      state = State::READING;
    }
    break;
  }
}

void TigoParser::pushBytes(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    pushByte(data[i]);
  }
}

bool TigoParser::hasFrames() const { return !frameQueue.empty(); }

std::unique_ptr<TigoFrame> TigoParser::popFrame() {
  if (frameQueue.empty())
    return nullptr;
  std::unique_ptr<TigoFrame> frame = std::move(frameQueue.front());
  frameQueue.pop_front();
  return frame;
}

uint16_t TigoParser::computeCRC(const uint8_t *data, size_t length) {
  uint16_t crc = 0x8408;
  for (size_t i = 0; i < length; i++) {
    uint8_t index = (crc ^ data[i]) & 0xFF;
    crc = (crc >> 8) ^ CRC_TABLE[index];
  }
  crc = (crc >> 8) | (crc << 8); // Final swap
  return crc;
}

bool TigoParser::verifyCRC(const std::vector<uint8_t> &frame) {
  if (frame.size() < 2)
    return false;
  size_t dataLen = frame.size() - 2;
  uint16_t receivedCRC = (frame[dataLen] << 8) | frame[dataLen + 1];
  uint16_t calculatedCRC = computeCRC(frame.data(), dataLen);
  return receivedCRC == calculatedCRC;
}

void TigoParser::processFrame(const std::vector<uint8_t> &frame) {
  if (!verifyCRC(frame)) {
    if (ignoreFirstCrcError) {
      ignoreFirstCrcError = false;
      return;
    }
    stats.crcErrors++;
    return;
  }
  ignoreFirstCrcError = false;
  stats.totalPhysicalFrames++;

  // Remove CRC
  std::vector<uint8_t> payload(frame.begin(), frame.end() - 2);

  // Basic validation
  if (payload.size() < 4)
    return;

  if (payload[2] == 0x01 && payload[3] == 0x49) {
    parseTLVPayload(payload);
  } else if (payload[2] == 0x0B && (payload[3] == 0x10 || payload[3] == 0x0F)) {
    parseManagementPayload(payload);
  }
}

void TigoParser::parseTLVPayload(const std::vector<uint8_t> &payload) {
  // Header parsing logic based on reference `calculateHeaderLength`
  // Payload indices:
  // 0-1: Unknown
  // 2-3: 01 49
  // 4-5: Status Word (used for length calc)

  if (payload.size() < 6)
    return;

  // Status word is at index 4 and 5.
  // Reference: "lowByte + highByte" from hex string.
  // Hex string index 8 corresponds to byte index 4.
  // Reference logic: statusHex = lowByte + highByte.
  // If payload[4]=0x00, payload[5]=0xEE.
  // status = 0x00EE.
  uint16_t status =
      (payload[4] << 8) |
      payload[5]; // Big Endian interpretation of the pair for bitmasking?
  // Actually reference: lowByte = substring(0,2), highByte = substring(2,4).
  // Then strtol(low + high).
  // If bytes are [0x00, 0xEE]. Low="00", High="EE". String="00EE".
  // Value=0x00EE. So it is (Byte4 << 8) | Byte5.

  int headerLen = 2; // Status word itself
  if ((status & (1 << 0)) == 0)
    headerLen += 1;
  if ((status & (1 << 1)) == 0)
    headerLen += 1;
  if ((status & (1 << 2)) == 0)
    headerLen += 2;
  if ((status & (1 << 3)) == 0)
    headerLen += 2;
  if ((status & (1 << 4)) == 0)
    headerLen += 1;
  headerLen += 1; // Packet # low
  headerLen += 2; // Slot counter

  // The payload data starts after the fixed header (01 49 + Status + Calculated
  // Header) Offset in `payload` vector: Bytes 0-3 are fixed preamble? Reference
  // starts calculating header length from hex index 8 (Byte 4). So the
  // calculated `headerLen` is relative to Byte 4. Start of TLV packets = 4 +
  // headerLen.

  size_t pos = 4 + headerLen;

  while (pos < payload.size()) {
    if (pos + 7 > payload.size())
      break; // Min header size for a packet

    uint8_t type = payload[pos];
    // Length is at pos + 6 (from reference: hex index + 12 -> 6 bytes)
    uint8_t len = payload[pos + 6];

    size_t packetTotalSize =
        len + 7; // 7 bytes of packet header + len bytes of data?
    // Reference: packetLengthInChars = length * 2 + 14.
    // 14 chars = 7 bytes.

    if (pos + packetTotalSize > payload.size())
      break;

    if (type == 0x31) { // Power Frame
      stats.powerDataCount++;
      // Data extraction
      // Packet structure relative to `pos`:
      // 0: Type (0x31)
      // 1: Address Byte 1
      // 2: Address Byte 2
      // 3: Node ID Byte 1
      // 4: Node ID Byte 2
      // 5: Unknown
      // 6: Length
      // 7...: Data

      // Reference `processPowerFrame` uses `packet` string.
      // `packet` starts with Type.
      // Addr: substring(2,6) -> Bytes 1, 2. But reference says NodeID is First.
      // So Bytes 1, 2 are NodeID. Bytes 3, 4 are Short Address.
      uint16_t node_id = (payload[pos + 1] << 8) | payload[pos + 2];
      uint16_t addr = (payload[pos + 3] << 8) | payload[pos + 4];

      // Data starts at pos + 7.
      // Vin: substring(14, 17) of packet hex string.
      // Hex index 14 is Byte 7.
      // 3 hex chars = 1.5 bytes.
      // (Byte7 << 4) | (Byte8 >> 4)
      float vin = get12Bits(&payload[pos + 7], true) * 0.05f;

      // Vout: substring(17, 20).
      // Starts at second nibble of Byte 8.
      // ((Byte8 & 0x0F) << 8) | Byte9
      float vout = get12Bits(&payload[pos + 8], false) * 0.10f;

      // Duty: substring(20, 22) -> Byte 10.
      uint8_t duty = payload[pos + 10];

      // Iin: substring(22, 25) -> Byte 11 (high nibble first)
      float iin = get12Bits(&payload[pos + 11], true) * 0.005f;

      // Temp: substring(25, 28) -> Byte 12 (low nibble first)
      float temp = getSigned12Bits(&payload[pos + 12], false) * 0.1f;

      // RSSI: substring(38, 40) -> Byte 19.
      // Wait, offset 19?
      // 38/2 = 19.
      int rssi = payload[pos + 19];

      frameQueue.push_back(std::unique_ptr<TigoFrame>(
          new TigoPowerFrame(addr, node_id, vin, vout, duty, iin, temp, rssi)));
    } else if (type == 0x09) { // Announce Frame
      stats.announceCount++;
      // Structure (Standard Header like PowerFrame):
      // 0: Type (0x09)
      // 1-2: Node ID
      // 3-4: Short Address
      // 7...: Payload (TopologyReport)

      if (pos + 23 <= payload.size()) {
        uint16_t node_id = (payload[pos + 1] << 8) | payload[pos + 2];
        uint16_t addr = (payload[pos + 3] << 8) | payload[pos + 4];
        uint32_t barcode = (payload[pos + 20] << 16) |
                           (payload[pos + 21] << 8) | payload[pos + 22];

        frameQueue.push_back(std::unique_ptr<TigoFrame>(
            new TigoAnnounceFrame(addr, node_id, barcode)));
      }
    }

    pos += packetTotalSize;
  }
}

void TigoParser::parseManagementPayload(const std::vector<uint8_t> &payload) {
  // Structure for 0B10 / 0B0F frames:
  // 0-1: Unknown
  // 2-3: 0B 10 (Header)
  // 7: Type

  if (payload.size() < 8)
    return;

  uint8_t type = payload[7];

  if (type == 0x27) { // Node Table / Command Response
    stats.nodeTableCount++;
    // Reference `process27frame`
    // Data starts at offset 9 (relative to payload start)
    // Num Entries at offset 11 (2 bytes)
    // List starts at offset 13

    if (payload.size() < 13)
      return;

    uint16_t numEntries = (payload[11] << 8) | payload[12];
    size_t pos = 13;

    std::vector<TigoNodeTableFrame::NodeEntry> entries;

    for (int i = 0; i < numEntries; i++) {
      // Each entry is 20 bytes in hex string -> 10 bytes raw
      // 8 bytes Long Address (Barcode)
      // 2 bytes Short Address
      if (pos + 10 > payload.size())
        break;

      TigoNodeTableFrame::NodeEntry entry;
      std::stringstream ss;
      ss << std::hex << std::uppercase << std::setfill('0');
      for (int j = 0; j < 8; j++)
        ss << std::setw(2) << (int)payload[pos + j];
      entry.barcode = ss.str();

      entry.pv_node_id = (payload[pos + 8] << 8) | payload[pos + 9];
      entries.push_back(entry);

      pos += 10;
    }

    if (!entries.empty()) {
      frameQueue.push_back(
          std::unique_ptr<TigoFrame>(new TigoNodeTableFrame(entries)));
    }
  } else {
    stats.managementCount++;
  }
}

int16_t TigoParser::getSigned12Bits(const uint8_t *ptr, bool highNibbleFirst) {
  uint16_t val = get12Bits(ptr, highNibbleFirst);
  // Sign extension for 12-bit to 16-bit
  if (val & 0x0800) {
    return (int16_t)(val | 0xF000);
  }
  return (int16_t)val;
}

uint16_t TigoParser::get12Bits(const uint8_t *ptr, bool highNibbleFirst) {
  if (highNibbleFirst) {
    // "ABC" -> 0xABC. Ptr points to A.
    // Byte 0: AB, Byte 1: CD
    // Result: (AB << 4) | (CD >> 4) -> ABC
    return (ptr[0] << 4) | (ptr[1] >> 4);
  } else {
    // "DEF" -> 0xDEF. Ptr points to D (which is low nibble of previous byte)
    // Byte 0: CD, Byte 1: EF
    // We want D E F.
    // ((Byte0 & 0x0F) << 8) | Byte1
    return ((ptr[0] & 0x0F) << 8) | ptr[1];
  }
}
