#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "TigoProtocol.h"

namespace {

// Independent bit-by-bit CRC-16/CCITT (reflected poly 0x8408, init 0x8408,
// final byte swap) matching TigoParser's framing expectations.
uint16_t computeTestCRC(const uint8_t *data, size_t length) {
  uint16_t crc = 0x8408;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0x8408;
      } else {
        crc >>= 1;
      }
    }
  }
  return static_cast<uint16_t>((crc >> 8) | (crc << 8));
}

void appendEscapedByte(std::vector<uint8_t> &out, uint8_t b) {
  switch (b) {
  case 0x7E:
    out.push_back(0x7E);
    out.push_back(0x00);
    break;
  case 0x24:
    out.push_back(0x7E);
    out.push_back(0x01);
    break;
  case 0x23:
    out.push_back(0x7E);
    out.push_back(0x02);
    break;
  case 0x25:
    out.push_back(0x7E);
    out.push_back(0x03);
    break;
  case 0xA4:
    out.push_back(0x7E);
    out.push_back(0x04);
    break;
  case 0xA3:
    out.push_back(0x7E);
    out.push_back(0x05);
    break;
  case 0xA5:
    out.push_back(0x7E);
    out.push_back(0x06);
    break;
  default:
    out.push_back(b);
    break;
  }
}

std::vector<uint8_t> buildWireFrame(const std::vector<uint8_t> &payload,
                                    uint16_t crcXorMask = 0x0000) {
  uint16_t crc = computeTestCRC(payload.data(), payload.size()) ^ crcXorMask;
  std::vector<uint8_t> unescaped = payload;
  unescaped.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  unescaped.push_back(static_cast<uint8_t>(crc & 0xFF));

  std::vector<uint8_t> wire;
  wire.reserve(unescaped.size() + 8);
  wire.push_back(0x7E);
  wire.push_back(0x07);
  for (uint8_t b : unescaped) {
    appendEscapedByte(wire, b);
  }
  wire.push_back(0x7E);
  wire.push_back(0x08);
  return wire;
}

// Builds a 0x01 0x49 TLV payload with a given status word and concatenated TLV
// packets. By default status = 0x001F (no optional header fields -> headerLen =
// 5, TLV starts at payload offset 9).
std::vector<uint8_t> buildTLVFramePayload(uint16_t status,
                                          const std::vector<uint8_t> &tlvs) {
  std::vector<uint8_t> payload = {
      0x00, 0x01,                                   // Preamble 0-1
      0x01, 0x49,                                   // TLV frame marker 2-3
      static_cast<uint8_t>((status >> 8) & 0xFF),   // Status high
      static_cast<uint8_t>(status & 0xFF),          // Status low
  };

  size_t extraHeaderBytes = 0;
  if ((status & (1 << 0)) == 0)
    extraHeaderBytes += 1;
  if ((status & (1 << 1)) == 0)
    extraHeaderBytes += 1;
  if ((status & (1 << 2)) == 0)
    extraHeaderBytes += 2;
  if ((status & (1 << 3)) == 0)
    extraHeaderBytes += 2;
  if ((status & (1 << 4)) == 0)
    extraHeaderBytes += 1;
  extraHeaderBytes += 1; // Packet # low
  extraHeaderBytes += 2; // Slot counter

  payload.insert(payload.end(), extraHeaderBytes, 0xAA);
  payload.insert(payload.end(), tlvs.begin(), tlvs.end());
  return payload;
}

// Helper to build a 20-byte PowerData (0x31) TLV packet (7-byte header + 13
// data bytes).
std::vector<uint8_t> buildPowerPacket(uint16_t nodeId, uint16_t shortAddr,
                                      uint16_t rawVin12, uint16_t rawVout12,
                                      uint8_t duty, uint16_t rawIin12,
                                      int16_t rawTemp12, uint8_t rssi) {
  std::vector<uint8_t> pkt(20, 0x00);
  pkt[0] = 0x31;
  pkt[1] = static_cast<uint8_t>((nodeId >> 8) & 0xFF);
  pkt[2] = static_cast<uint8_t>(nodeId & 0xFF);
  pkt[3] = static_cast<uint8_t>((shortAddr >> 8) & 0xFF);
  pkt[4] = static_cast<uint8_t>(shortAddr & 0xFF);
  pkt[5] = 0x00;
  pkt[6] = 13; // len -> total size 7 + 13 = 20 bytes

  uint16_t vin12 = rawVin12 & 0x0FFF;
  uint16_t vout12 = rawVout12 & 0x0FFF;
  pkt[7] = static_cast<uint8_t>((vin12 >> 4) & 0xFF);
  pkt[8] = static_cast<uint8_t>(((vin12 & 0x0F) << 4) | ((vout12 >> 8) & 0x0F));
  pkt[9] = static_cast<uint8_t>(vout12 & 0xFF);
  pkt[10] = duty;

  uint16_t iin12 = rawIin12 & 0x0FFF;
  uint16_t temp12 = static_cast<uint16_t>(rawTemp12) & 0x0FFF;
  pkt[11] = static_cast<uint8_t>((iin12 >> 4) & 0xFF);
  pkt[12] =
      static_cast<uint8_t>(((iin12 & 0x0F) << 4) | ((temp12 >> 8) & 0x0F));
  pkt[13] = static_cast<uint8_t>(temp12 & 0xFF);

  pkt[19] = rssi;
  return pkt;
}

std::vector<uint8_t> buildAnnouncePacket(uint16_t nodeId, uint16_t shortAddr,
                                         uint32_t barcodeFrag24) {
  std::vector<uint8_t> pkt(23, 0x00);
  pkt[0] = 0x09;
  pkt[1] = static_cast<uint8_t>((nodeId >> 8) & 0xFF);
  pkt[2] = static_cast<uint8_t>(nodeId & 0xFF);
  pkt[3] = static_cast<uint8_t>((shortAddr >> 8) & 0xFF);
  pkt[4] = static_cast<uint8_t>(shortAddr & 0xFF);
  pkt[5] = 0x00;
  pkt[6] = 16; // len -> total size 7 + 16 = 23 bytes
  pkt[20] = static_cast<uint8_t>((barcodeFrag24 >> 16) & 0xFF);
  pkt[21] = static_cast<uint8_t>((barcodeFrag24 >> 8) & 0xFF);
  pkt[22] = static_cast<uint8_t>(barcodeFrag24 & 0xFF);
  return pkt;
}

struct NodeTableInput {
  uint8_t barcode[8];
  uint16_t nodeId;
};

std::vector<uint8_t>
buildNodeTablePayload(uint8_t subCommand,
                      const std::vector<NodeTableInput> &entries) {
  std::vector<uint8_t> payload = {
      0x00, 0x01,                                          // 0-1
      0x0B, subCommand,                                    // 2-3 (0x0F or 0x10)
      0x00, 0x00, 0x00,                                    // 4-6
      0x27,                                                // 7: NodeTable type
      0x00, 0x00, 0x00,                                    // 8-10
      static_cast<uint8_t>((entries.size() >> 8) & 0xFF),  // 11: numEntries hi
      static_cast<uint8_t>(entries.size() & 0xFF),         // 12: numEntries lo
  };
  for (const auto &entry : entries) {
    payload.insert(payload.end(), entry.barcode, entry.barcode + 8);
    payload.push_back(static_cast<uint8_t>((entry.nodeId >> 8) & 0xFF));
    payload.push_back(static_cast<uint8_t>(entry.nodeId & 0xFF));
  }
  return payload;
}

} // namespace

TEST(TigoParserTest, InitialStateHasNoFramesAndZeroStats) {
  TigoParser parser;
  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.popFrame(), nullptr);

  const auto &stats = parser.getStatistics();
  EXPECT_EQ(stats.totalPhysicalFrames, 0u);
  EXPECT_EQ(stats.crcErrors, 0u);
  EXPECT_EQ(stats.powerDataCount, 0u);
  EXPECT_EQ(stats.announceCount, 0u);
  EXPECT_EQ(stats.nodeTableCount, 0u);
  EXPECT_EQ(stats.managementCount, 0u);
  EXPECT_EQ(parser.getStatisticsInfluxLineProtocol("tigo"),
            "tigo_stats total_frames=0i,crc_errors=0i,power_count=0i,"
            "announce_count=0i,nodetable_count=0i,management_count=0i");
}

TEST(TigoParserTest, PowerDataDeserializationAndFormatting) {
  TigoParser parser;

  // Vin = 700 * 0.05 = 35.00 V (0x2BC)
  // Vout = 345 * 0.10 = 34.50 V (0x159)
  // Duty = 200
  // Iin = 1600 * 0.005 = 8.00 A (0x640)
  // Temp = 255 * 0.1 = 25.5 C (0x0FF)
  // RSSI = 185
  auto pkt = buildPowerPacket(/*nodeId=*/0x0042, /*shortAddr=*/1234,
                              /*rawVin12=*/700, /*rawVout12=*/345,
                              /*duty=*/200, /*rawIin12=*/1600,
                              /*rawTemp12=*/255, /*rssi=*/185);
  auto payload = buildTLVFramePayload(0x001F, pkt);
  auto wire = buildWireFrame(payload);

  parser.pushBytes(wire.data(), wire.size());

  ASSERT_TRUE(parser.hasFrames());
  auto frame = parser.popFrame();
  ASSERT_NE(frame, nullptr);
  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(frame->getType(), TigoFrameType::PowerData);

  auto *power = dynamic_cast<TigoPowerFrame *>(frame.get());
  ASSERT_NE(power, nullptr);
  EXPECT_EQ(power->address, 1234);
  EXPECT_EQ(power->pv_node_id, 0x0042);
  EXPECT_FLOAT_EQ(power->voltage_in, 35.0f);
  EXPECT_FLOAT_EQ(power->voltage_out, 34.5f);
  EXPECT_EQ(power->duty_cycle, 200);
  EXPECT_FLOAT_EQ(power->current_in, 8.0f);
  EXPECT_FLOAT_EQ(power->temperature, 25.5f);
  EXPECT_EQ(power->rssi, 185);

  EXPECT_EQ(power->toString(),
            "Addr:1234 ID:0042 Vin:35.00 Vout:34.50 I:8.00 T:25.5");
  EXPECT_EQ(power->toInfluxLineProtocol("tigo"),
            "tigo,addr=1234,node_id=0066 "
            "vin=35,vout=34.5,amp=8,watt=276,temp=25.5,rssi=185,duty=200");

  const auto &stats = parser.getStatistics();
  EXPECT_EQ(stats.totalPhysicalFrames, 1u);
  EXPECT_EQ(stats.crcErrors, 0u);
  EXPECT_EQ(stats.powerDataCount, 1u);
}

TEST(TigoParserTest, PowerDataNegativeTemperatureAndOptionalHeaderFields) {
  TigoParser parser;

  // Status = 0x00EE -> bits 0 and 4 are 0 (bits 1,2,3 are 1),
  // exercising optional header length calculation:
  // headerLen = 2 + 1(bit0) + 1(bit4) + 1 + 2 = 7 bytes.
  // Temp = -125 * 0.1 = -12.5 C (12-bit two's complement: 0xF83).
  auto pkt = buildPowerPacket(/*nodeId=*/0x1234, /*shortAddr=*/5,
                              /*rawVin12=*/400, /*rawVout12=*/380,
                              /*duty=*/255, /*rawIin12=*/1000,
                              /*rawTemp12=*/-125, /*rssi=*/210);
  auto payload = buildTLVFramePayload(0x00EE, pkt);
  auto wire = buildWireFrame(payload);

  parser.pushBytes(wire.data(), wire.size());

  ASSERT_TRUE(parser.hasFrames());
  auto frame = parser.popFrame();
  auto *power = dynamic_cast<TigoPowerFrame *>(frame.get());
  ASSERT_NE(power, nullptr);
  EXPECT_FLOAT_EQ(power->temperature, -12.5f);
  EXPECT_EQ(power->toString(),
            "Addr:5 ID:1234 Vin:20.00 Vout:38.00 I:5.00 T:-12.5");
}

TEST(TigoParserTest, PowerDataAllStatusBitsZeroHeaderLength) {
  TigoParser parser;

  // Status = 0x0000 -> all 5 optional header bits are 0 -> headerLen = 12.
  auto pkt = buildPowerPacket(/*nodeId=*/0x0001, /*shortAddr=*/99,
                              /*rawVin12=*/200, /*rawVout12=*/200,
                              /*duty=*/100, /*rawIin12=*/500,
                              /*rawTemp12=*/-1, /*rssi=*/150);
  auto payload = buildTLVFramePayload(0x0000, pkt);
  auto wire = buildWireFrame(payload);

  parser.pushBytes(wire.data(), wire.size());

  ASSERT_TRUE(parser.hasFrames());
  auto frame = parser.popFrame();
  auto *power = dynamic_cast<TigoPowerFrame *>(frame.get());
  ASSERT_NE(power, nullptr);
  EXPECT_EQ(power->address, 99);
  EXPECT_EQ(power->pv_node_id, 1);
  EXPECT_FLOAT_EQ(power->temperature, -0.1f);
}

TEST(TigoParserTest, AnnounceDeserializationAndInfluxProtocol) {
  TigoParser parser;

  auto pkt = buildAnnouncePacket(/*nodeId=*/0x001A, /*shortAddr=*/42,
                                 /*barcodeFrag24=*/0x0ABCDEF);
  auto payload = buildTLVFramePayload(0x001F, pkt);
  auto wire = buildWireFrame(payload);

  parser.pushBytes(wire.data(), wire.size());

  ASSERT_TRUE(parser.hasFrames());
  auto frame = parser.popFrame();
  ASSERT_NE(frame, nullptr);
  EXPECT_EQ(frame->getType(), TigoFrameType::Announce);

  auto *announce = dynamic_cast<TigoAnnounceFrame *>(frame.get());
  ASSERT_NE(announce, nullptr);
  EXPECT_EQ(announce->address, 42);
  EXPECT_EQ(announce->pv_node_id, 0x001A);
  EXPECT_EQ(announce->barcode_fragment, 0xABCDEFu);
  EXPECT_EQ(announce->toString(),
            "Announce Addr:42 ID:001A BarcodeFrag:ABCDEF");
  EXPECT_EQ(announce->toInfluxLineProtocol("tigo"),
            "tigo_announce,addr=42 node_id=0026,barcode_frag=abcdef");
  EXPECT_EQ(parser.getStatistics().announceCount, 1u);
}

TEST(TigoParserTest, NodeTableDeserializationFor0B0FAnd0B10) {
  TigoParser parser;

  std::vector<NodeTableInput> entries = {
      {{0x04, 0xC0, 0x5B, 0x30, 0x00, 0x12, 0x34, 0x56}, 10},
      {{0x04, 0xC0, 0x5B, 0x30, 0x00, 0xAB, 0xCD, 0xEF}, 20},
      {{0x04, 0xC0, 0x5B, 0x30, 0x00, 0x00, 0x00, 0x01}, 30},
      {{0x04, 0xC0, 0x5B, 0x30, 0x00, 0xFF, 0xEE, 0xDD}, 40},
  };

  auto payload0F = buildNodeTablePayload(0x0F, entries);
  auto wire0F = buildWireFrame(payload0F);
  parser.pushBytes(wire0F.data(), wire0F.size());

  ASSERT_TRUE(parser.hasFrames());
  auto frame0F = parser.popFrame();
  ASSERT_NE(frame0F, nullptr);
  EXPECT_EQ(frame0F->getType(), TigoFrameType::NodeTable);

  auto *table = dynamic_cast<TigoNodeTableFrame *>(frame0F.get());
  ASSERT_NE(table, nullptr);
  ASSERT_EQ(table->nodes.size(), 4u);
  EXPECT_EQ(table->nodes[0].barcode, "04C05B3000123456");
  EXPECT_EQ(table->nodes[0].pv_node_id, 10);
  EXPECT_EQ(table->nodes[1].barcode, "04C05B3000ABCDEF");
  EXPECT_EQ(table->nodes[1].pv_node_id, 20);
  EXPECT_EQ(table->nodes[2].barcode, "04C05B3000000001");
  EXPECT_EQ(table->nodes[2].pv_node_id, 30);
  EXPECT_EQ(table->nodes[3].barcode, "04C05B3000FFEEDD");
  EXPECT_EQ(table->nodes[3].pv_node_id, 40);

  EXPECT_EQ(table->toString(),
            "NodeTable (4 entries): [04C05B3000123456->10], "
            "[04C05B3000ABCDEF->20], [04C05B3000000001->30]...");

  EXPECT_EQ(
      table->toInfluxLineProtocol("tigo"),
      "tigo_map,addr=10 barcode=\"04C05B3000123456\"\n"
      "tigo_map,addr=20 barcode=\"04C05B3000ABCDEF\"\n"
      "tigo_map,addr=30 barcode=\"04C05B3000000001\"\n"
      "tigo_map,addr=40 barcode=\"04C05B3000FFEEDD\"");

  // Also test 0x0B 0x10 with 2 entries (no "..." truncation in toString)
  std::vector<NodeTableInput> twoEntries = {entries[0], entries[1]};
  auto payload10 = buildNodeTablePayload(0x10, twoEntries);
  auto wire10 = buildWireFrame(payload10);
  parser.pushBytes(wire10.data(), wire10.size());

  ASSERT_TRUE(parser.hasFrames());
  auto frame10 = parser.popFrame();
  auto *table10 = dynamic_cast<TigoNodeTableFrame *>(frame10.get());
  ASSERT_NE(table10, nullptr);
  EXPECT_EQ(table10->nodes.size(), 2u);
  EXPECT_EQ(table10->toString(),
            "NodeTable (2 entries): [04C05B3000123456->10], "
            "[04C05B3000ABCDEF->20]");

  EXPECT_EQ(parser.getStatistics().nodeTableCount, 2u);
  EXPECT_EQ(parser.getStatistics().totalPhysicalFrames, 2u);
}

TEST(TigoParserTest, OtherManagementFrameIncrementsManagementCount) {
  TigoParser parser;

  std::vector<uint8_t> payload = {0x00, 0x01, 0x0B, 0x10,
                                  0x00, 0x00, 0x00, 0x99};
  auto wire = buildWireFrame(payload);
  parser.pushBytes(wire.data(), wire.size());

  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().totalPhysicalFrames, 1u);
  EXPECT_EQ(parser.getStatistics().managementCount, 1u);
  EXPECT_EQ(parser.getStatistics().nodeTableCount, 0u);
}

TEST(TigoParserTest, ByteStuffingEscapesAllKnownSequences) {
  TigoParser parser;

  // Include all special bytes that get escaped on the wire:
  // 0x7E -> 7E 00
  // 0x24 -> 7E 01
  // 0x23 -> 7E 02
  // 0x25 -> 7E 03
  // 0xA4 -> 7E 04
  // 0xA3 -> 7E 05
  // 0xA5 -> 7E 06
  NodeTableInput entry = {
      {0x7E, 0x24, 0x23, 0x25, 0xA4, 0xA3, 0xA5, 0x00},
      0x7E24,
  };
  auto payload = buildNodeTablePayload(0x10, {entry});
  auto wire = buildWireFrame(payload);

  // Sanity check that the wire stream actually contains 0x7E 0x00..0x06
  auto hasSequence = [&](uint8_t escCode) {
    for (size_t i = 2; i + 1 < wire.size() - 2; ++i) {
      if (wire[i] == 0x7E && wire[i + 1] == escCode) {
        return true;
      }
    }
    return false;
  };
  EXPECT_TRUE(hasSequence(0x00));
  EXPECT_TRUE(hasSequence(0x01));
  EXPECT_TRUE(hasSequence(0x02));
  EXPECT_TRUE(hasSequence(0x03));
  EXPECT_TRUE(hasSequence(0x04));
  EXPECT_TRUE(hasSequence(0x05));
  EXPECT_TRUE(hasSequence(0x06));

  parser.pushBytes(wire.data(), wire.size());

  ASSERT_TRUE(parser.hasFrames());
  auto frame = parser.popFrame();
  auto *table = dynamic_cast<TigoNodeTableFrame *>(frame.get());
  ASSERT_NE(table, nullptr);
  ASSERT_EQ(table->nodes.size(), 1u);
  EXPECT_EQ(table->nodes[0].barcode, "7E242325A4A3A500");
  EXPECT_EQ(table->nodes[0].pv_node_id, 0x7E24);

  // Verify 0x7E followed by 0x07 or 0x08 in payload is escaped as 7E 00 07 /
  // 7E 00 08 and does not prematurely split or terminate the frame.
  NodeTableInput delimiterLikeEntry = {
      {0x7E, 0x07, 0x7E, 0x08, 0x01, 0x02, 0x03, 0x04},
      0x7E08,
  };
  auto delimWire =
      buildWireFrame(buildNodeTablePayload(0x10, {delimiterLikeEntry}));
  parser.pushBytes(delimWire.data(), delimWire.size());
  ASSERT_TRUE(parser.hasFrames());
  auto delimFrame = parser.popFrame();
  auto *delimTable = dynamic_cast<TigoNodeTableFrame *>(delimFrame.get());
  ASSERT_NE(delimTable, nullptr);
  ASSERT_EQ(delimTable->nodes.size(), 1u);
  EXPECT_EQ(delimTable->nodes[0].barcode, "7E077E0801020304");
  EXPECT_EQ(delimTable->nodes[0].pv_node_id, 0x7E08);
}

TEST(TigoParserTest, UnknownEscapeSequencePreservesBothBytes) {
  TigoParser parser;

  // Construct a NodeTable payload whose barcode literally contains 0x7E, 0x55.
  // In the wire stream, emit raw 0x7E 0x55 (unknown escape) instead of
  // 0x7E 0x00 0x55; TigoParser should push both 0x7E and 0x55 into buffer.
  NodeTableInput entry = {
      {0x7E, 0x55, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66},
      0x0007,
  };
  auto payload = buildNodeTablePayload(0x10, {entry});
  uint16_t crc = computeTestCRC(payload.data(), payload.size());
  std::vector<uint8_t> unescaped = payload;
  unescaped.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  unescaped.push_back(static_cast<uint8_t>(crc & 0xFF));

  std::vector<uint8_t> wire = {0x7E, 0x07};
  for (size_t i = 0; i < unescaped.size(); ++i) {
    // Keep 0x7E followed by 0x55 unescaped so parser hits the default branch.
    wire.push_back(unescaped[i]);
  }
  wire.push_back(0x7E);
  wire.push_back(0x08);

  parser.pushBytes(wire.data(), wire.size());

  ASSERT_TRUE(parser.hasFrames());
  auto frame = parser.popFrame();
  auto *table = dynamic_cast<TigoNodeTableFrame *>(frame.get());
  ASSERT_NE(table, nullptr);
  ASSERT_EQ(table->nodes.size(), 1u);
  EXPECT_EQ(table->nodes[0].barcode, "7E55112233445566");
}

TEST(TigoParserTest, CrcValidationIgnoresFirstErrorAndCountsSubsequent) {
  TigoParser parser;

  auto pkt = buildPowerPacket(1, 2, 100, 100, 50, 100, 200, 180);
  auto payload = buildTLVFramePayload(0x001F, pkt);
  auto badWire = buildWireFrame(payload, 0x0001);
  auto goodWire = buildWireFrame(payload, 0x0000);

  // 1st CRC error is ignored (not counted in crcErrors)
  parser.pushBytes(badWire.data(), badWire.size());
  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().crcErrors, 0u);
  EXPECT_EQ(parser.getStatistics().totalPhysicalFrames, 0u);

  // 2nd CRC error IS counted and frame is dropped
  parser.pushBytes(badWire.data(), badWire.size());
  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().crcErrors, 1u);
  EXPECT_EQ(parser.getStatistics().totalPhysicalFrames, 0u);

  // 3rd CRC error increments again
  parser.pushBytes(badWire.data(), badWire.size());
  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().crcErrors, 2u);

  // Valid frame parses cleanly
  parser.pushBytes(goodWire.data(), goodWire.size());
  ASSERT_TRUE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().totalPhysicalFrames, 1u);
  EXPECT_EQ(parser.getStatistics().crcErrors, 2u);
  EXPECT_NE(parser.popFrame(), nullptr);
}

TEST(TigoParserTest, ValidFrameClearsIgnoreFirstCrcErrorFlag) {
  TigoParser parser;

  auto pkt = buildPowerPacket(1, 2, 100, 100, 50, 100, 200, 180);
  auto payload = buildTLVFramePayload(0x001F, pkt);
  auto goodWire = buildWireFrame(payload, 0x0000);
  auto badWire = buildWireFrame(payload, 0xFFFF);

  parser.pushBytes(goodWire.data(), goodWire.size());
  ASSERT_TRUE(parser.hasFrames());
  parser.popFrame();

  // Because a valid frame already arrived, the very first CRC error now counts
  parser.pushBytes(badWire.data(), badWire.size());
  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().crcErrors, 1u);
}

TEST(TigoParserTest, ChunkedByteFeedsGarbageAndMultipleFrames) {
  TigoParser parser;

  auto powerPkt = buildPowerPacket(0x0010, 10, 500, 400, 128, 600, 300, 170);
  auto announcePkt = buildAnnouncePacket(0x0010, 10, 0x123456);
  std::vector<uint8_t> combinedTlvs = powerPkt;
  combinedTlvs.insert(combinedTlvs.end(), announcePkt.begin(), announcePkt.end());
  auto tlvPayload = buildTLVFramePayload(0x001F, combinedTlvs);
  auto wire1 = buildWireFrame(tlvPayload);

  auto tablePayload = buildNodeTablePayload(
      0x10, {{{0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}, 10}});
  auto wire2 = buildWireFrame(tablePayload);

  // Build noisy stream:
  // - leading garbage including stray 0x7E bytes and a false start 0x7E 0x7E 0x07
  // - wire1
  // - inter-frame garbage
  // - wire2
  std::vector<uint8_t> stream = {0x00, 0xFF, 0x7E, 0x99, 0x7E, 0x7E};
  // wire1 already starts with 0x7E 0x07; dropping its initial 0x7E tests the
  // 0x7E 0x7E 0x07 recovery transition in WAIT_FRAME_TYPE!
  stream.insert(stream.end(), wire1.begin() + 1, wire1.end());
  stream.push_back(0x12);
  stream.push_back(0x34);
  stream.push_back(0x7E);
  stream.push_back(0x08);
  stream.insert(stream.end(), wire2.begin(), wire2.end());

  // Feed 1 byte at a time (automatically splits mid-escape and mid-frame)
  for (uint8_t b : stream) {
    parser.pushByte(b);
  }

  ASSERT_TRUE(parser.hasFrames());
  auto f1 = parser.popFrame();
  ASSERT_NE(f1, nullptr);
  EXPECT_EQ(f1->getType(), TigoFrameType::PowerData);

  ASSERT_TRUE(parser.hasFrames());
  auto f2 = parser.popFrame();
  ASSERT_NE(f2, nullptr);
  EXPECT_EQ(f2->getType(), TigoFrameType::Announce);

  ASSERT_TRUE(parser.hasFrames());
  auto f3 = parser.popFrame();
  ASSERT_NE(f3, nullptr);
  EXPECT_EQ(f3->getType(), TigoFrameType::NodeTable);

  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().totalPhysicalFrames, 2u);
  EXPECT_EQ(parser.getStatistics().powerDataCount, 1u);
  EXPECT_EQ(parser.getStatistics().announceCount, 1u);
  EXPECT_EQ(parser.getStatistics().nodeTableCount, 1u);
}

TEST(TigoParserTest, TruncatedAndMalformedPayloadsDoNotCrashOrReadOOB) {
  TigoParser parser;

  // 1. Frame shorter than 2 bytes (empty frame between 7E 07 and 7E 08)
  std::vector<uint8_t> emptyWire = {0x7E, 0x07, 0x7E, 0x08};
  parser.pushBytes(emptyWire.data(), emptyWire.size());
  EXPECT_FALSE(parser.hasFrames());

  // 2. Valid CRC on 3-byte payload (< 4 bytes minimum)
  auto wireShort3 = buildWireFrame({0x01, 0x02, 0x03});
  parser.pushBytes(wireShort3.data(), wireShort3.size());
  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().totalPhysicalFrames, 1u);

  // 3. Valid CRC on 5-byte 0x01 0x49 payload (< 6 bytes minimum for TLV header)
  auto wireShortTlvHeader = buildWireFrame({0x00, 0x01, 0x01, 0x49, 0x00});
  parser.pushBytes(wireShortTlvHeader.data(), wireShortTlvHeader.size());
  EXPECT_FALSE(parser.hasFrames());

  // 4. Valid 0x01 0x49 header with truncated TLV packet header (< 7 bytes)
  auto wireTruncatedTlv =
      buildWireFrame(buildTLVFramePayload(0x001F, {0x31, 0x00, 0x01}));
  parser.pushBytes(wireTruncatedTlv.data(), wireTruncatedTlv.size());
  EXPECT_FALSE(parser.hasFrames());

  // 5. TLV packet claiming length larger than remaining payload (len = 250)
  auto wireBadTlvLen = buildWireFrame(
      buildTLVFramePayload(0x001F, {0x31, 0x00, 0x01, 0x00, 0x02, 0x00, 250}));
  parser.pushBytes(wireBadTlvLen.data(), wireBadTlvLen.size());
  EXPECT_FALSE(parser.hasFrames());

  // 6. Announce packet whose TLV length is smaller than 23 bytes (e.g. len = 5 ->
  // packetTotalSize = 12) so pos + 23 > payload.size()
  std::vector<uint8_t> shortAnnounce(12, 0x00);
  shortAnnounce[0] = 0x09;
  shortAnnounce[6] = 5;
  auto wireShortAnnounce =
      buildWireFrame(buildTLVFramePayload(0x001F, shortAnnounce));
  parser.pushBytes(wireShortAnnounce.data(), wireShortAnnounce.size());
  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().announceCount, 1u);

  // 7. Management frame shorter than 8 bytes
  auto wireShortMgmt =
      buildWireFrame({0x00, 0x01, 0x0B, 0x10, 0x00, 0x00, 0x00});
  parser.pushBytes(wireShortMgmt.data(), wireShortMgmt.size());
  EXPECT_FALSE(parser.hasFrames());

  // 8. NodeTable frame (0x27) shorter than 13 bytes
  auto wireShortNodeTable =
      buildWireFrame({0x00, 0x01, 0x0B, 0x10, 0x00, 0x00, 0x00, 0x27, 0x00, 0x01});
  parser.pushBytes(wireShortNodeTable.data(), wireShortNodeTable.size());
  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().nodeTableCount, 1u);

  // 9. NodeTable claiming numEntries = 5 but only having 1 full entry + 4
  // trailing bytes
  std::vector<uint8_t> truncatedNodeTable = {
      0x00, 0x01, 0x0B, 0x10, 0x00, 0x00, 0x00, 0x27, 0x00, 0x00, 0x00,
      0x00, 0x05, // numEntries = 5
      0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x00, 0x09, // entry 0
      0x12, 0x34, 0x56, 0x78                                      // partial
  };
  auto wirePartialNodeTable = buildWireFrame(truncatedNodeTable);
  parser.pushBytes(wirePartialNodeTable.data(), wirePartialNodeTable.size());
  ASSERT_TRUE(parser.hasFrames());
  auto f = parser.popFrame();
  auto *table = dynamic_cast<TigoNodeTableFrame *>(f.get());
  ASSERT_NE(table, nullptr);
  EXPECT_EQ(table->nodes.size(), 1u);
  EXPECT_EQ(table->nodes[0].barcode, "AABBCCDDEEFF0011");
  EXPECT_EQ(table->nodes[0].pv_node_id, 9);

  // 10. NodeTable with 0 entries produces no frame
  auto wireZeroEntries = buildWireFrame(buildNodeTablePayload(0x10, {}));
  parser.pushBytes(wireZeroEntries.data(), wireZeroEntries.size());
  EXPECT_FALSE(parser.hasFrames());

  // 11. Oversized frame (> 1024 bytes) triggers safety reset without crashing
  std::vector<uint8_t> oversized = {0x7E, 0x07};
  oversized.insert(oversized.end(), 1100, 0x42);
  oversized.push_back(0x7E);
  oversized.push_back(0x08);
  parser.pushBytes(oversized.data(), oversized.size());
  EXPECT_FALSE(parser.hasFrames());

  // Verify parser still works normally after oversized reset
  auto validPower =
      buildWireFrame(buildTLVFramePayload(0x001F, buildPowerPacket(1, 2, 100, 100, 10, 100, 100, 100)));
  parser.pushBytes(validPower.data(), validPower.size());
  EXPECT_TRUE(parser.hasFrames());
  EXPECT_NE(parser.popFrame(), nullptr);
}

TEST(TigoParserTest, StatisticsInfluxLineProtocolMatchesCounts) {
  TigoParser parser;

  auto powerWire = buildWireFrame(buildTLVFramePayload(
      0x001F, buildPowerPacket(1, 2, 100, 100, 50, 100, 200, 180)));
  auto badWire = buildWireFrame(
      buildTLVFramePayload(0x001F, buildPowerPacket(1, 2, 100, 100, 50, 100, 200, 180)),
      0x00FF);
  auto announceWire = buildWireFrame(
      buildTLVFramePayload(0x001F, buildAnnouncePacket(1, 2, 0x112233)));
  auto tableWire = buildWireFrame(buildNodeTablePayload(
      0x10, {{{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}, 2}}));
  auto mgmtWire =
      buildWireFrame({0x00, 0x01, 0x0B, 0x10, 0x00, 0x00, 0x00, 0x42});

  parser.pushBytes(powerWire.data(), powerWire.size());
  parser.pushBytes(badWire.data(), badWire.size());
  parser.pushBytes(announceWire.data(), announceWire.size());
  parser.pushBytes(tableWire.data(), tableWire.size());
  parser.pushBytes(mgmtWire.data(), mgmtWire.size());

  EXPECT_EQ(parser.getStatisticsInfluxLineProtocol("solar"),
            "solar_stats total_frames=4i,crc_errors=1i,power_count=1i,"
            "announce_count=1i,nodetable_count=1i,management_count=1i");
}

TEST(TigoParserTest, PowerDataMinMaxBoundaryValues) {
  TigoParser parser;

  // Minimum boundary values:
  // Vin/Vout/Iin = 0x000 (0.0), Temp = 0x800 (-2048 * 0.1 = -204.8°C),
  // duty = 0, rssi = 0
  auto minPkt = buildPowerPacket(/*nodeId=*/0x0000, /*shortAddr=*/0,
                                 /*rawVin12=*/0x000, /*rawVout12=*/0x000,
                                 /*duty=*/0, /*rawIin12=*/0x000,
                                 /*rawTemp12=*/-2048, /*rssi=*/0);
  // Maximum boundary values:
  // Vin = 0xFFF * 0.05 = 204.75V, Vout = 0xFFF * 0.10 = 409.5V,
  // Iin = 0xFFF * 0.005 = 20.475A, Temp = 0x7FF (+2047 * 0.1 = +204.7°C),
  // duty = 255, rssi = 255
  auto maxPkt = buildPowerPacket(/*nodeId=*/0xFFFF, /*shortAddr=*/65535,
                                 /*rawVin12=*/0xFFF, /*rawVout12=*/0xFFF,
                                 /*duty=*/255, /*rawIin12=*/0xFFF,
                                 /*rawTemp12=*/2047, /*rssi=*/255);

  std::vector<uint8_t> tlvs = minPkt;
  tlvs.insert(tlvs.end(), maxPkt.begin(), maxPkt.end());
  auto wire = buildWireFrame(buildTLVFramePayload(0x001F, tlvs));
  parser.pushBytes(wire.data(), wire.size());

  ASSERT_TRUE(parser.hasFrames());
  auto fMin = parser.popFrame();
  auto *minPower = dynamic_cast<TigoPowerFrame *>(fMin.get());
  ASSERT_NE(minPower, nullptr);
  EXPECT_EQ(minPower->address, 0);
  EXPECT_EQ(minPower->pv_node_id, 0x0000);
  EXPECT_FLOAT_EQ(minPower->voltage_in, 0.0f);
  EXPECT_FLOAT_EQ(minPower->voltage_out, 0.0f);
  EXPECT_EQ(minPower->duty_cycle, 0);
  EXPECT_FLOAT_EQ(minPower->current_in, 0.0f);
  EXPECT_FLOAT_EQ(minPower->temperature, -204.8f);
  EXPECT_EQ(minPower->rssi, 0);

  ASSERT_TRUE(parser.hasFrames());
  auto fMax = parser.popFrame();
  auto *maxPower = dynamic_cast<TigoPowerFrame *>(fMax.get());
  ASSERT_NE(maxPower, nullptr);
  EXPECT_EQ(maxPower->address, 65535);
  EXPECT_EQ(maxPower->pv_node_id, 0xFFFF);
  EXPECT_FLOAT_EQ(maxPower->voltage_in, 0xFFF * 0.05f);
  EXPECT_FLOAT_EQ(maxPower->voltage_out, 0xFFF * 0.10f);
  EXPECT_EQ(maxPower->duty_cycle, 255);
  EXPECT_FLOAT_EQ(maxPower->current_in, 0xFFF * 0.005f);
  EXPECT_FLOAT_EQ(maxPower->temperature, 204.7f);
  EXPECT_EQ(maxPower->rssi, 255);
  EXPECT_FALSE(parser.hasFrames());
}

TEST(TigoParserTest, ValidTlvFollowedByTruncatedSecondTlvInSameFrame) {
  TigoParser parser;

  auto validPkt = buildPowerPacket(/*nodeId=*/0x0077, /*shortAddr=*/88,
                                   /*rawVin12=*/600, /*rawVout12=*/300,
                                   /*duty=*/180, /*rawIin12=*/1200,
                                   /*rawTemp12=*/250, /*rssi=*/190);
  // Case A: valid 0x31 packet followed by truncated packet header (< 7 bytes)
  std::vector<uint8_t> tlvsShortHeader = validPkt;
  tlvsShortHeader.insert(tlvsShortHeader.end(), {0x31, 0x00, 0x01, 0x00});
  auto wireA = buildWireFrame(buildTLVFramePayload(0x001F, tlvsShortHeader));
  parser.pushBytes(wireA.data(), wireA.size());

  ASSERT_TRUE(parser.hasFrames());
  auto fA = parser.popFrame();
  auto *pA = dynamic_cast<TigoPowerFrame *>(fA.get());
  ASSERT_NE(pA, nullptr);
  EXPECT_EQ(pA->address, 88);
  EXPECT_EQ(pA->pv_node_id, 0x0077);
  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().powerDataCount, 1u);

  // Case B: valid 0x31 packet followed by a second 0x31 packet whose length
  // exceeds the remaining payload (claims len=13, only 4 data bytes present)
  std::vector<uint8_t> tlvsShortBody = validPkt;
  tlvsShortBody.insert(
      tlvsShortBody.end(),
      {0x31, 0x00, 0x99, 0x00, 0x99, 0x00, 13, 0x11, 0x22, 0x33, 0x44});
  auto wireB = buildWireFrame(buildTLVFramePayload(0x001F, tlvsShortBody));
  parser.pushBytes(wireB.data(), wireB.size());

  ASSERT_TRUE(parser.hasFrames());
  auto fB = parser.popFrame();
  auto *pB = dynamic_cast<TigoPowerFrame *>(fB.get());
  ASSERT_NE(pB, nullptr);
  EXPECT_EQ(pB->address, 88);
  EXPECT_EQ(pB->pv_node_id, 0x0077);
  EXPECT_FALSE(parser.hasFrames());
  EXPECT_EQ(parser.getStatistics().powerDataCount, 2u);
}

TEST(TigoParserTest, NodeTableStrips0x8000MarkerBitFromNodeId) {
  TigoParser parser;

  NodeTableInput entry = {
      {0x04, 0xC0, 0x5B, 0x40, 0x00, 0x9A, 0x57, 0xA2},
      0x802A,
  };
  auto wire = buildWireFrame(buildNodeTablePayload(0x10, {entry}));
  parser.pushBytes(wire.data(), wire.size());

  ASSERT_TRUE(parser.hasFrames());
  auto frame = parser.popFrame();
  auto *table = dynamic_cast<TigoNodeTableFrame *>(frame.get());
  ASSERT_NE(table, nullptr);
  ASSERT_EQ(table->nodes.size(), 1u);
  EXPECT_EQ(table->nodes[0].barcode, "04C05B40009A57A2");
  EXPECT_EQ(table->nodes[0].pv_node_id, 0x002A);
}

TEST(TigoParserTest, TigoBarcodeFormatsNormalAndAllZeroLowerNibbles) {
  const uint8_t addr1[8] = {0x04, 0xC0, 0x5B, 0x40, 0x00, 0x9A, 0x57, 0xA2};
  EXPECT_EQ(tigoBarcode(addr1), "4-9A57A2L");
  EXPECT_EQ(tigoBarcodeFromHex("04C05B40009A57A2"), "4-9A57A2L");
  EXPECT_EQ(tigoBarcodeFromHex("04c05b40009a57a2"), "4-9A57A2L");
  EXPECT_EQ(tigoBarcodeFromHex("04C05B3000123456"), "3-123456W");

  // All 9 lower nibbles zero: keep the final zero nibble ("0-0L", not "0-L").
  const uint8_t allZeroLower[8] = {0x04, 0xC0, 0x5B, 0x00,
                                   0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(tigoBarcode(allZeroLower), "0-0L");
  EXPECT_EQ(tigoBarcodeFromHex("04C05B0000000000"), "0-0L");

  // Malformed hex inputs return empty string.
  EXPECT_EQ(tigoBarcodeFromHex(""), "");
  EXPECT_EQ(tigoBarcodeFromHex("04C05B"), "");
  EXPECT_EQ(tigoBarcodeFromHex("04C05B40009A57A200"), "");
  EXPECT_EQ(tigoBarcodeFromHex("04C05B40009A57GZ"), "");
}

TEST(TigoParserTest, GatewayVersion000BFiltersNonPrintableAndCapsLength) {
  TigoParser parser;

  // Frame from gateway 1 (0x8001), type 0x000B
  std::vector<uint8_t> payload = {
      0x80, 0x01, 0x00, 0x0B,
      0x00, 0x07, 'T', 'A', 'P', '\r', '\n', 'v', '1', '.', '2',
      0x1B, 0x7F, 0x80, 0xFF, ' ', '\r', '\n', ' ',
  };
  auto wire = buildWireFrame(payload);
  parser.pushBytes(wire.data(), wire.size());

  const auto &gateways = parser.getGateways();
  auto it = gateways.find(1);
  ASSERT_NE(it, gateways.end());
  EXPECT_TRUE(it->second.seen);
  EXPECT_EQ(it->second.version, "TAP  v1.2");

  // Version longer than 120 printable chars is capped at 120 chars.
  std::vector<uint8_t> longPayload = {0x80, 0x01, 0x00, 0x0B};
  longPayload.insert(longPayload.end(), 140, 'A');
  auto longWire = buildWireFrame(longPayload);
  parser.pushBytes(longWire.data(), longWire.size());

  it = parser.getGateways().find(1);
  ASSERT_NE(it, parser.getGateways().end());
  EXPECT_EQ(it->second.version.size(), 120u);
  EXPECT_EQ(it->second.version, std::string(120, 'A'));
}

TEST(TigoParserTest, CommandResponse0B10UpdatesTxBuffersMin) {
  TigoParser parser;

  auto send0B10 = [&](uint8_t txFree) {
    std::vector<uint8_t> payload = {0x80, 0x01, 0x0B, 0x10,
                                    0x00, txFree, 0x00, 0x99};
    auto wire = buildWireFrame(payload);
    parser.pushBytes(wire.data(), wire.size());
  };

  send0B10(10);
  auto it = parser.getGateways().find(1);
  ASSERT_NE(it, parser.getGateways().end());
  EXPECT_EQ(it->second.txBuffersFree, 10);
  EXPECT_EQ(it->second.txBuffersMin, 10);

  send0B10(4);
  EXPECT_EQ(it->second.txBuffersFree, 4);
  EXPECT_EQ(it->second.txBuffersMin, 4);

  send0B10(12);
  EXPECT_EQ(it->second.txBuffersFree, 12);
  EXPECT_EQ(it->second.txBuffersMin, 4);
}

TEST(TigoParserTest, GatewayAndNodeTimestampsPreserve64BitValuesPast49Days) {
  TigoParser parser;

  constexpr uint64_t kPast32BitMs = 0x100000000ULL + 12345ULL;
  constexpr uint64_t kLater64BitMs = 0x200000100ULL;

  auto sendGatewayTlv = [&](uint16_t nodeId) {
    auto pkt = buildPowerPacket(nodeId, 123, 700, 345, 200, 1600, 255, 185);
    auto payload = buildTLVFramePayload(0x001F, pkt);
    payload[0] = 0x80; // fromGateway bit
    payload[1] = 0x01; // gateway ID 1
    auto wire = buildWireFrame(payload);
    parser.pushBytes(wire.data(), wire.size());
    while (parser.hasFrames()) {
      parser.popFrame();
    }
  };

  parser.setTime(kPast32BitMs);
  sendGatewayTlv(0x0042);

  const auto &gateways = parser.getGateways();
  auto gwIt = gateways.find(1);
  ASSERT_NE(gwIt, gateways.end());
  EXPECT_TRUE(gwIt->second.seen);
  EXPECT_EQ(gwIt->second.lastSeenMs, kPast32BitMs);
  auto nodeIt = gwIt->second.nodes.find(0x0042);
  ASSERT_NE(nodeIt, gwIt->second.nodes.end());
  EXPECT_EQ(nodeIt->second.lastMs, kPast32BitMs);

  parser.setTime(kLater64BitMs);
  sendGatewayTlv(0x0042);

  EXPECT_EQ(gwIt->second.lastSeenMs, kLater64BitMs);
  EXPECT_EQ(gwIt->second.nodes.at(0x0042).lastMs, kLater64BitMs);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

