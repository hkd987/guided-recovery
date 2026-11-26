// The pod's flight software in one object. Hardware-independent: the ESP32
// target, the simulator and the log replayer all drive it the same way.
//
//   FlightCore core(params);
//   every 5 ms:  const Outputs& out = core.update(inputs);
//
// Guardrails enforced here (from the spec):
//   * steering arms only after the canopy is confirmed open;
//   * the only possible target is the home point saved before launch;
//   * GPS loss, leaving the geofence, a jammed winch, a lopsided canopy or a
//     low battery end guidance in a slow spiral; a collapse releases brakes.
#pragma once

#include "gr/actuation.hpp"
#include "gr/estimators.hpp"
#include "gr/guidance.hpp"
#include "gr/params.hpp"
#include "gr/phase_detector.hpp"
#include "gr/records.hpp"
#include "gr/types.hpp"

namespace gr {

struct Status {
  float t = 0.0f;
  Phase phase = Phase::Disarmed;
  FlightStage stage = FlightStage::Disarmed;
  GuideMode mode = GuideMode::None;
  uint16_t faults = kFaultNone;
  bool homeValid = false;
  double homeLat = 0.0, homeLon = 0.0;
  uint16_t homeFixes = 0;
  bool groundRefValid = false;
  float groundPressure = 0.0f;
  float agl = 0.0f;
  float climbRate = 0.0f;
  float accelG = 1.0f;
  float yawRate = 0.0f;
  bool gpsGood = false;
  float lastGoodGpsT = -1e9f;
  uint8_t numSv = 0;
  NavState nav;
  GuidanceCmd guide;
  BrakeCmd brakes;
  float winchMm[2] = {0.0f, 0.0f};
  bool steeringActive = false;
  bool candidate = false;       // launch suspected (PhaseDetector candidate flag)
  float trimNeeded = 0.0f;      // brake asymmetry the canopy needs to fly straight
  float yawRateFiltered = 0.0f;
  float launchTime = -1.0f, apogeeTime = -1.0f, canopyTime = -1.0f;
};

// What must survive a processor restart in flight. The ESP32 target keeps
// it in RTC memory (kept through brownout, watchdog and panic resets).
struct ResumeState {
  uint32_t magic = 0;
  uint8_t stage = 0;          // FlightStage
  uint8_t homeValid = 0;
  uint16_t faults = 0;
  double homeLat = 0.0, homeLon = 0.0;
  float groundPressure = 0.0f;
  int32_t encoder[2] = {0, 0};  // winch positions in counts (encoders restart at 0)
  uint32_t checksum = 0;
};
static_assert(sizeof(ResumeState) == 40, "ResumeState must stay padding-free (it is checksummed bytewise)");
constexpr uint32_t kResumeMagic = 0x47525253;  // "GRRS"
uint32_t resumeChecksum(const ResumeState& r);

class FlightCore {
 public:
  explicit FlightCore(const Params& p = Params{});
  void reset();
  const Outputs& update(const Inputs& in);

  const Status& status() const { return st_; }
  const PhaseDetector& detector() const { return det_; }
  const Params& params() const { return p_; }

  // Log record at Params::logRateHz; returns true when one is ready.
  bool takeLogRecord(LogRecord& out);
  bool takeTelemetry(TelemetryPacket& out);

  // Bench control, only honoured while disarmed (magnet in).
  bool benchSetBrake(int channel, float frac);
  void benchRelease();
  bool benchActive() const { return benchActive_; }
  void clearWinchFaults();

  // Snapshot to keep across a restart (call every cycle; cheap).
  ResumeState resumeState(const int32_t encoder[2]) const;
  // Call once, before the first update(), after a restart that was not a
  // power-on. Returns true if a flight was resumed:
  //   Ascent / Descent -> carry on detecting, with home and ground pressure;
  //   Canopy           -> Neutral: both brakes released, no more steering.
  // Anything else (bench, pad, landed) is an ordinary boot.
  bool resume(const ResumeState& r);

 private:
  void updatePadReferences(const Inputs& in, float dt);
  void enterCanopy();
  void enterSpiral(uint16_t fault);
  void checkSafety(const Inputs& in, float dt);
  NavState buildNav() const;
  void fillLog(const Inputs& in);
  void fillTelemetry(const Inputs& in);

  Params p_;
  PhaseDetector det_;
  AttitudeFilter att_;
  AltitudeFilter alt_;
  WindEstimator wind_;
  Guidance guide_;
  Steering steer_;
  WinchController winch_[2];
  GeoOrigin home_;
  Status st_;
  Outputs out_;

  bool first_ = true;
  uint64_t lastUs_ = 0;
  float t_ = 0.0f;
  float lastBaroT_ = -1.0f;
  LowPass groundP_{10.0f};
  // Home averaging (degrees, double precision).
  double homeLatSum_ = 0.0, homeLonSum_ = 0.0;
  uint32_t homeN_ = 0;
  // Navigation.
  Vec2 pos_, vel_;
  float lastWindSolveT_ = 0.0f;
  LowPass trimSink_{5.0f};
  LowPass descent_{1.0f};
  // Safety timers.
  float collapseT_ = 0.0f;
  float lowBattT_ = 0.0f;
  BrakeCmd spiralBrakes_;
  float landedT_ = -1.0f;
  // Bench.
  bool benchActive_ = false;
  float benchTarget_[2] = {0.0f, 0.0f};
  // Records.
  float logAccum_ = 0.0f, telAccum_ = 0.0f;
  bool logReady_ = false, telReady_ = false;
  LogRecord log_;
  TelemetryPacket tel_;
  uint8_t telSeq_ = 0;
  bool gpsFreshThisFrame_ = false;
  int32_t lastLatE7_ = 0, lastLonE7_ = 0;
};

}  // namespace gr
