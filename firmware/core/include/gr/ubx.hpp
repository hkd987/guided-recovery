// u-blox UBX protocol: a streaming NAV-PVT parser and builders for the
// configuration the pod sends on every boot (M10 receivers keep no config
// without a backup battery). Hardware-independent so it is unit-tested on
// the desktop.
#pragma once

#include <cstddef>
#include <cstdint>

#include "gr/types.hpp"

namespace gr {
namespace ubx {

constexpr uint8_t kSync1 = 0xB5;
constexpr uint8_t kSync2 = 0x62;
constexpr uint8_t kClassNav = 0x01;
constexpr uint8_t kIdNavPvt = 0x07;
constexpr uint8_t kClassCfg = 0x06;
constexpr uint8_t kIdCfgValset = 0x8A;
constexpr uint8_t kClassAck = 0x05;

// Configuration keys (u-blox M10 interface description).
constexpr uint32_t kCfgUart1Baudrate = 0x40520001;
constexpr uint32_t kCfgUart1InUbx = 0x10730001;
constexpr uint32_t kCfgUart1OutUbx = 0x10740001;
constexpr uint32_t kCfgUart1OutNmea = 0x10740002;
constexpr uint32_t kCfgMsgoutNavPvtUart1 = 0x20910007;
constexpr uint32_t kCfgRateMeas = 0x30210001;
constexpr uint32_t kCfgRateNav = 0x30210002;
constexpr uint32_t kCfgNavspgDynmodel = 0x20110021;
constexpr uint32_t kCfgSignalGpsEna = 0x1031001f;
constexpr uint32_t kCfgSignalGalEna = 0x10310021;
constexpr uint32_t kCfgSignalBdsEna = 0x10310022;
constexpr uint32_t kCfgSignalGloEna = 0x10310025;

constexpr uint8_t kLayerRam = 0x01;
constexpr uint8_t kLayerBbr = 0x02;

struct KeyValue {
  uint32_t key;
  uint32_t value;
};

// Value size in bytes encoded in bits 28..30 of a key id.
size_t keyValueSize(uint32_t key);

// Writes a complete UBX frame (sync, header, payload, checksum). Returns its
// length, or 0 if it does not fit.
size_t buildFrame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len, uint8_t* out, size_t cap);

// UBX-CFG-VALSET with the given key/value pairs.
size_t buildValset(const KeyValue* kv, size_t n, uint8_t layers, uint8_t* out, size_t cap);

// The pod's receiver setup: UART1 at `baud`, UBX only, NAV-PVT every fix,
// 10 Hz, airborne <4 g dynamics, GPS + Galileo (10 Hz is not available with
// all four constellations on the M10).
size_t buildPodConfig(uint32_t baud, uint8_t* out, size_t cap);

class Parser {
 public:
  // Feed one byte. Returns true when a complete, checksum-valid NAV-PVT has
  // just been decoded into fix().
  bool feed(uint8_t b);
  const GpsFix& fix() const { return fix_; }
  uint32_t iTow() const { return iTow_; }
  uint32_t framesOk() const { return ok_; }
  uint32_t framesBad() const { return bad_; }
  // Last ACK/NAK seen for a CFG message (cls, id), for config verification.
  bool lastAck() const { return lastAck_; }
  uint8_t lastAckId() const { return lastAckId_; }
  uint32_t acks() const { return acks_; }

 private:
  void handleFrame();

  enum State { kWait1, kWait2, kClass, kId, kLen1, kLen2, kPayload, kCkA, kCkB };
  State st_ = kWait1;
  uint8_t cls_ = 0, id_ = 0;
  uint16_t len_ = 0, idx_ = 0;
  uint8_t ckA_ = 0, ckB_ = 0;
  uint8_t buf_[100] = {};
  bool overflow_ = false;
  GpsFix fix_;
  uint32_t iTow_ = 0;
  uint32_t ok_ = 0, bad_ = 0;
  bool lastAck_ = false;
  uint8_t lastAckId_ = 0;
  uint32_t acks_ = 0;
};

}  // namespace ubx
}  // namespace gr
