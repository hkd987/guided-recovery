// Backup-kit sensor handling: one or two barometers, one or two GPS
// receivers. A basic pod (one of each) runs through the same code.
//
// BaroVoter
//   Each barometer is checked on its own: readings must keep arriving, stay
//   in a physical range, and not freeze while something else shows the pod
//   moving. Two barometers are cross-checked against each other; when they
//   disagree, GPS decides which one is wrong. A single barometer is checked
//   against GPS too. "GPS" here is the receiver's Doppler vertical speed,
//   added up over a few seconds: far more precise than its altitude.
//   trusted() is false when no barometer is believable, or two disagree and
//   nothing can say which is right. The flight core then refuses to confirm
//   a canopy, and spirals if it is already steering.
//
// GpsSelector
//   Uses receiver 1 while it has a usable fix, fails over to receiver 2 when
//   it goes quiet, and corrects receiver 2's position by the offset between
//   the two measured on the pad (so the home point stays consistent).
#pragma once

#include <cstdint>

#include "gr/types.hpp"

namespace gr {

struct BaroVoterParams {
  float staleTime = 0.5f;        // s without a reading: that barometer is unavailable
  float stuckTime = 1.5f;        // s of an identical reading while something else moves
  float stuckMove = 5.0f;        // m the other barometer or GPS must have moved meanwhile
  float agreeMinM = 10.0f;       // two barometers agree within max(this, agreeFrac * height)
  float agreeFrac = 0.05f;
  float disagreeTime = 1.0f;     // s of disagreement before acting on it
  float gpsWindow = 5.0f;        // s over which height change is compared with GPS
  float gpsTolM = 12.0f;         // m of height-change mismatch that condemns a lone barometer...
  float gpsTolFrac = 0.15f;      // ...or this share of the change (the standard atmosphere is only a model)
  float gpsTolTime = 3.0f;       // s the mismatch must persist (ejection pressure spikes are shorter)
  float gpsRecoverTime = 5.0f;   // s a doubted lone barometer must agree again before it is trusted
  float gpsDecideM = 5.0f;       // m by which GPS must favour one barometer to pick it
};

class BaroVoter {
 public:
  void reset();
  // Call every cycle. fresh*/p*: this cycle's readings (p2 ignored unless
  // present2). calibrate: on the pad, measure barometer 2's offset.
  // checkGps: in a stage where the comparison is fair (descent, canopy).
  // gpsGood/gpsVelD: a good GPS fix this cycle and its vertical speed
  // (m/s, down positive).
  void update(float t, bool fresh1, float p1, bool present2, bool fresh2, float p2, bool calibrate, bool checkGps,
              bool gpsGood, float gpsVelD);

  bool fresh() const { return fresh_; }          // a fused reading arrived this cycle
  float pressure() const { return fused_; }      // Pa, in barometer 1's frame
  bool trusted() const { return trusted_; }
  bool failed(int i) const { return failed_[i] || doubt_[i]; }  // broken, or (alone) out of step with GPS
  bool disagree() const { return disagreeT_ >= p_.disagreeTime; }
  bool present2() const { return present2_; }
  float offset2() const { return off2_; }        // Pa, barometer 2 minus barometer 1 on the pad
  // The barometers' surroundings just changed (an ejection opened the bay):
  // compare with GPS only from here on.
  void restartGpsCompare() { ++gpsSeg_; }

  BaroVoterParams p_;

 private:
  struct Sample {
    float t, h[2], gpsH;   // gpsH: Doppler-integrated height
    bool ok[2], gps;
    uint16_t seg;          // GPS segment: no gap longer than 1 s inside one
  };
  static constexpr int kHist = 16;  // 2 Hz, 8 s
  float residual(int i, float t, bool& valid, float* gpsChange = nullptr) const;

  bool present2_ = false;
  bool fresh_ = false, trusted_ = false;
  float fused_ = 0.0f;
  float last_[2] = {0.0f, 0.0f};       // latest pressure (barometer 2 corrected)
  float lastT_[2] = {-1e9f, -1e9f};    // when it last arrived
  float raw_[2] = {0.0f, 0.0f};        // latest raw value, for stuck detection
  float changeT_[2] = {-1e9f, -1e9f};  // when the raw value last changed
  float hAtChange_[2] = {0.0f, 0.0f};  // the other source's height then
  bool failed_[2] = {false, false};    // stuck, or out-voted with GPS's help: latched until reset
  bool doubt_[2] = {false, false};     // alone and out of step with GPS: until it agrees again
  float gpsOkT_[2] = {0.0f, 0.0f};     // how long it has agreed with GPS since
  bool offInit_ = false;
  float off2_ = 0.0f;
  float rawLatest_[2] = {0.0f, 0.0f}, rawT_[2] = {-1e9f, -2e9f};
  float disagreeT_ = 0.0f;
  float gpsBadT_[2] = {0.0f, 0.0f};
  float prevT_ = -1.0f;
  float padH_ = 0.0f;  // barometer 1 height (ISA) on the pad, for the relative tolerance
  bool padInit_ = false;
  // History for the GPS comparison.
  Sample hist_[kHist] = {};
  int head_ = 0, count_ = 0;
  float histT_ = -1e9f;
  bool gpsSeen_ = false;
  float gpsH_ = 0.0f, gpsT_ = -1e9f;  // Doppler-integrated height, last good fix
  uint16_t gpsSeg_ = 0;
};

class GpsSelector {
 public:
  void reset();
  // calibrate: on the pad, learn receiver 2's offset from receiver 1.
  void update(float t, const GpsFix& g1, bool present2, const GpsFix& g2, bool calibrate);
  const GpsFix& fix() const { return out_; }  // out_.fresh: a usable fix this cycle
  int active() const { return active_; }      // 0 = receiver 1, 1 = receiver 2
  bool present2() const { return present2_; }
  bool good(int i) const { return goodT_[i] > -1e8f; }

  float switchAfter = 0.4f;  // s without a usable fix from the active receiver

 private:
  static bool usable(const GpsFix& g) { return g.fixType >= 3 && g.numSv >= 5 && g.hAcc < 15.0f; }
  GpsFix out_;
  int active_ = 0;
  bool present2_ = false;
  float goodT_[2] = {-1e9f, -1e9f};  // last usable fix
  bool offInit_ = false;
  double offLat_ = 0.0, offLon_ = 0.0;  // receiver 2 minus receiver 1, degrees * 1e7
  GpsFix last_[2];
};

}  // namespace gr
