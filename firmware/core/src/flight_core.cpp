#include "gr/flight_core.hpp"

#include <cmath>
#include <cstddef>

namespace gr {

const char* phaseName(Phase p) {
  switch (p) {
    case Phase::Disarmed: return "DISARMED";
    case Phase::Pad: return "PAD";
    case Phase::Ascent: return "ASCENT";
    case Phase::Descent: return "DESCENT";
    case Phase::Guided: return "GUIDED";
    case Phase::Spiral: return "SPIRAL";
    case Phase::Neutral: return "NEUTRAL";
    case Phase::Landed: return "LANDED";
  }
  return "?";
}

const char* guideModeName(GuideMode m) {
  switch (m) {
    case GuideMode::None: return "-";
    case GuideMode::Homing: return "HOMING";
    case GuideMode::Loiter: return "LOITER";
    case GuideMode::Final: return "FINAL";
    case GuideMode::Flare: return "FLARE";
  }
  return "?";
}

namespace {
int16_t sat16(float v) {
  if (!std::isfinite(v)) return 0;
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return static_cast<int16_t>(std::lround(v));
}
int8_t sat8(float v) {
  if (!std::isfinite(v)) return 0;
  if (v > 127.0f) return 127;
  if (v < -128.0f) return -128;
  return static_cast<int8_t>(std::lround(v));
}
uint8_t satU8(float v) {
  if (!std::isfinite(v) || v < 0.0f) return 0;
  if (v > 255.0f) return 255;
  return static_cast<uint8_t>(std::lround(v));
}
}  // namespace

FlightCore::FlightCore(const Params& p)
    : p_(p), det_(p.det), guide_(p.guide, p.canopy), steer_(p.steer, p.canopy) {
  reset();
}

void FlightCore::reset() {
  det_ = PhaseDetector(p_.det);
  det_.reset();
  att_.reset();
  alt_ = AltitudeFilter();
  wind_.reset(p_.canopy.airspeed);
  guide_ = Guidance(p_.guide, p_.canopy);
  guide_.reset();
  steer_ = Steering(p_.steer, p_.canopy);
  steer_.reset();
  winch_[0].configure(p_.winch, 0);
  winch_[1].configure(p_.winch, 1);
  home_ = GeoOrigin();
  st_ = Status();
  out_ = Outputs();
  first_ = true;
  lastUs_ = 0;
  t_ = 0.0f;
  lastBaroT_ = -1.0f;
  groundP_ = LowPass(10.0f);
  homeLatSum_ = homeLonSum_ = 0.0;
  homeN_ = 0;
  pos_ = vel_ = Vec2{};
  lastWindSolveT_ = 0.0f;
  trimSink_ = LowPass(5.0f);
  descent_ = LowPass(1.0f);
  collapseT_ = lowBattT_ = 0.0f;
  spiralBrakes_ = BrakeCmd{};
  landedT_ = -1.0f;
  benchActive_ = false;
  benchTarget_[0] = benchTarget_[1] = 0.0f;
  logAccum_ = telAccum_ = 0.0f;
  logReady_ = telReady_ = false;
  telSeq_ = 0;
}

bool FlightCore::benchSetBrake(int ch, float frac) {
  if (st_.phase != Phase::Disarmed || ch < 0 || ch > 1) return false;
  benchActive_ = true;
  benchTarget_[ch] = clampf(frac, 0.0f, 1.0f);
  return true;
}

void FlightCore::benchRelease() {
  benchTarget_[0] = benchTarget_[1] = 0.0f;
}

uint32_t resumeChecksum(const ResumeState& r) {
  // FNV-1a over everything before the checksum field.
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&r);
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < offsetof(ResumeState, checksum); ++i) h = (h ^ p[i]) * 16777619u;
  return h;
}

ResumeState FlightCore::resumeState(const int32_t encoder[2]) const {
  ResumeState r;
  r.magic = kResumeMagic;
  r.stage = static_cast<uint8_t>(st_.stage);
  r.homeValid = st_.homeValid ? 1 : 0;
  r.faults = st_.faults;
  r.homeLat = st_.homeLat;
  r.homeLon = st_.homeLon;
  r.groundPressure = st_.groundPressure;
  r.encoder[0] = encoder[0];
  r.encoder[1] = encoder[1];
  r.checksum = resumeChecksum(r);
  return r;
}

bool FlightCore::resume(const ResumeState& r) {
  if (r.magic != kResumeMagic || r.checksum != resumeChecksum(r)) return false;
  const auto stage = static_cast<FlightStage>(r.stage);
  if (stage != FlightStage::Ascent && stage != FlightStage::Descent && stage != FlightStage::Canopy) return false;
  if (!(r.groundPressure > 1000.0f)) return false;
  reset();
  det_.resumeAt(stage);
  st_.stage = stage;
  st_.faults = static_cast<uint16_t>(r.faults | kFaultReset);
  if (r.homeValid) {
    home_.set(r.homeLat, r.homeLon);
    st_.homeValid = true;
    st_.homeLat = r.homeLat;
    st_.homeLon = r.homeLon;
  }
  groundP_.reset(r.groundPressure);
  st_.groundRefValid = true;
  st_.groundPressure = r.groundPressure;
  // Under a canopy we no longer know enough to steer safely: release both brakes.
  st_.phase = stage == FlightStage::Canopy ? Phase::Neutral
              : stage == FlightStage::Ascent ? Phase::Ascent : Phase::Descent;
  return true;
}

void FlightCore::clearWinchFaults() {
  winch_[0].clearFault();
  winch_[1].clearFault();
  st_.faults &= static_cast<uint16_t>(~(kFaultWinchLeft | kFaultWinchRight));
}

void FlightCore::updatePadReferences(const Inputs& in, float dt) {
  (void)dt;
  // Home: average of good, stationary fixes while armed on the pad. Frozen
  // the moment a launch is suspected, and never changeable in flight.
  const GpsFix& g = in.gps;
  if (g.fresh && !det_.launchCandidate() && st_.stage == FlightStage::Pad) {
    const float speed = std::sqrt(g.velN * g.velN + g.velE * g.velE);
    if (g.fixType >= 3 && g.numSv >= p_.det.homeMinSv && g.hAcc <= p_.det.homeMaxHAcc &&
        speed <= p_.det.homeMaxSpeed) {
      const double lat = g.latE7 * 1e-7, lon = g.lonE7 * 1e-7;
      // Running mean over the most recent ~60 s of fixes.
      if (homeN_ >= 600) {
        homeLatSum_ *= 599.0 / 600.0;
        homeLonSum_ *= 599.0 / 600.0;
        homeN_ = 599;
      }
      homeLatSum_ += lat;
      homeLonSum_ += lon;
      ++homeN_;
      if (homeN_ >= p_.det.homeMinFixes) {
        home_.set(homeLatSum_ / homeN_, homeLonSum_ / homeN_);
        st_.homeValid = true;
        st_.homeLat = home_.lat0;
        st_.homeLon = home_.lon0;
      }
      st_.homeFixes = static_cast<uint16_t>(homeN_ > 65535 ? 65535 : homeN_);
    }
  }
  if (st_.stage == FlightStage::Disarmed) {
    // Disarming on the pad forgets the home point; arming learns it fresh.
    homeLatSum_ = homeLonSum_ = 0.0;
    homeN_ = 0;
    st_.homeValid = false;
    st_.homeFixes = 0;
    home_.valid = false;
  }
}

void FlightCore::enterCanopy() {
  st_.canopyTime = t_;
  att_.reset();  // hanging under the canopy: realign "down" from the accelerometer
  trimSink_.reset(p_.canopy.sinkRate);
  wind_.reset(p_.canopy.airspeed);
  guide_.reset();
  steer_.reset();
  collapseT_ = lowBattT_ = 0.0f;
  if (st_.agl < p_.det.minGuidedAlt) {
    st_.faults |= kFaultLowDeploy;
    st_.phase = Phase::Neutral;
  } else if (!st_.homeValid) {
    enterSpiral(kFaultNoHome);
  } else {
    st_.phase = Phase::Guided;
  }
}

void FlightCore::enterSpiral(uint16_t fault) {
  st_.faults |= fault;
  if (st_.phase == Phase::Spiral || st_.phase == Phase::Neutral || st_.phase == Phase::Landed) return;
  const bool badL = winch_[0].stalled(), badR = winch_[1].stalled();
  const float sb = p_.safety.spiralBrake;
  if (fault & kFaultLopsided) {
    // Hold most of the counter-brake the canopy needs: a slow spiral the way
    // it pulls, instead of a fast one.
    spiralBrakes_ = mixBrakes(0.0f, clampf(0.8f * steer_.trimNeeded(), -0.7f, 0.7f));
  } else if (badL && badR) {
    spiralBrakes_ = BrakeCmd{0.0f, 0.0f};
  } else if (badL || badR) {
    // Turn toward the jammed side if it is stuck pulled in, else use the good side.
    const int bad = badL ? 0 : 1;
    const float stuck = winch_[bad].positionMm() / p_.winch.travelMm;
    const bool towardBad = stuck > 0.2f;
    const int side = towardBad ? bad : 1 - bad;
    spiralBrakes_ = BrakeCmd{0.0f, 0.0f};
    if (!towardBad) {
      if (side == 0) spiralBrakes_.left = sb; else spiralBrakes_.right = sb;
    }
  } else {
    spiralBrakes_ = BrakeCmd{0.0f, sb};
  }
  st_.phase = Phase::Spiral;
}

void FlightCore::checkSafety(const Inputs& in, float dt) {
  if (st_.stage != FlightStage::Canopy) return;
  if (st_.phase == Phase::Neutral || st_.phase == Phase::Landed) return;

  // Canopy collapse: falling far faster than any parafoil should.
  if (descent_.value() > p_.safety.collapseDescent) collapseT_ += dt;
  else collapseT_ = 0.0f;
  if (collapseT_ >= p_.safety.collapseTime) {
    st_.faults |= kFaultCollapse;
    st_.phase = Phase::Neutral;
    return;
  }

  if (winch_[0].stalled()) st_.faults |= kFaultWinchLeft;
  if (winch_[1].stalled()) st_.faults |= kFaultWinchRight;
  if (st_.phase != Phase::Guided) return;

  if (t_ - st_.lastGoodGpsT > p_.safety.gpsTimeout) { enterSpiral(kFaultGpsLost); return; }
  if (pos_.norm() > p_.safety.fenceRadius) { enterSpiral(kFaultFence); return; }
  if (winch_[0].stalled()) { enterSpiral(kFaultWinchLeft); return; }
  if (winch_[1].stalled()) { enterSpiral(kFaultWinchRight); return; }
  if (steer_.lopsided()) { enterSpiral(kFaultLopsided); return; }
  if (in.batteryV < p_.safety.lowBatteryV) lowBattT_ += dt;
  else lowBattT_ = 0.0f;
  if (lowBattT_ >= p_.safety.lowBatteryTime) { enterSpiral(kFaultLowBattery); return; }
}

NavState FlightCore::buildNav() const {
  NavState n;
  n.posValid = st_.homeValid && (t_ - st_.lastGoodGpsT) <= p_.safety.gpsTimeout;
  n.pos = pos_;
  n.velGround = vel_;
  n.groundSpeed = vel_.norm();
  n.courseValid = n.groundSpeed >= p_.guide.minGroundSpeedForCourse;
  n.course = n.courseValid ? bearingOf(vel_) : 0.0f;
  n.agl = st_.agl;
  n.sinkRate = trimSink_.initialized() ? trimSink_.value() : p_.canopy.sinkRate;
  n.windValid = wind_.valid();
  n.wind = wind_.wind();
  n.airspeed = wind_.valid() ? wind_.airspeed() : p_.canopy.airspeed;
  if (n.windValid) {
    const Vec2 air = vel_ - n.wind;
    n.headingValid = air.norm() > 2.0f;
    n.heading = n.headingValid ? bearingOf(air) : 0.0f;
  }
  return n;
}

const Outputs& FlightCore::update(const Inputs& in) {
  float dt = 0.005f;
  if (!first_) dt = static_cast<float>(static_cast<double>(in.tUs - lastUs_) * 1e-6);
  if (!(dt >= 0.0f)) dt = 0.0f;
  if (dt > 0.1f) dt = 0.1f;
  first_ = false;
  lastUs_ = in.tUs;
  t_ += dt;
  st_.t = t_;

  // ---- Inertial
  if (in.imu.valid) {
    // Between launch and canopy the accelerometer sees thrust, drag and
    // tumbling, not gravity: let the gyro carry attitude alone.
    const bool accelIsGravity = st_.stage == FlightStage::Disarmed || st_.stage == FlightStage::Pad ||
                                st_.stage == FlightStage::Canopy || st_.stage == FlightStage::Landed;
    att_.update(in.imu.gyro, in.imu.accel, dt, accelIsGravity);
    st_.accelG = in.imu.accel.norm() / kGravity;
    st_.yawRate = att_.initialized() ? att_.yawRate(in.imu.gyro) : 0.0f;
  }

  // ---- Barometer
  alt_.predict(dt);
  if (in.baroFresh && in.pressurePa > 1000.0f) {
    const float bdt = lastBaroT_ < 0.0f ? 0.02f : t_ - lastBaroT_;
    lastBaroT_ = t_;
    if (!st_.groundRefValid) {
      groundP_.reset(in.pressurePa);
      st_.groundRefValid = true;
    } else if (st_.stage <= FlightStage::Pad && !det_.launchCandidate()) {
      groundP_.update(in.pressurePa, bdt);
    }
    st_.groundPressure = groundP_.value();
    alt_.update(pressureToAltitude(in.pressurePa, st_.groundPressure));
  }
  st_.agl = alt_.altitude();
  st_.climbRate = alt_.climbRate();
  descent_.update(-st_.climbRate, dt);

  // ---- GPS
  gpsFreshThisFrame_ = in.gps.fresh;
  if (in.gps.fresh) {
    const GpsFix& g = in.gps;
    st_.numSv = g.numSv;
    st_.gpsGood = g.fixType >= 3 && g.numSv >= 5 && g.hAcc < 15.0f;
    if (st_.gpsGood) {
      st_.lastGoodGpsT = t_;
      lastLatE7_ = g.latE7;
      lastLonE7_ = g.lonE7;
      vel_ = Vec2{g.velN, g.velE};
      if (home_.valid) pos_ = home_.toLocal(g.latE7 * 1e-7, g.lonE7 * 1e-7) + vel_ * p_.gpsLatency;
      if (st_.stage == FlightStage::Canopy) {
        const float sym = clampf(std::fmin(winch_[0].positionMm(), winch_[1].positionMm()) / p_.winch.travelMm, 0.0f, 1.0f);
        wind_.addSample(vel_, t_, wind_.airspeed() * (1.0f - p_.canopy.brakeSpeedLoss * sym));
      }
    }
  } else if (st_.gpsGood) {
    pos_ += vel_ * dt;  // dead-reckon between 10 Hz fixes
  }
  updatePadReferences(in, dt);

  // ---- Flight events
  DetectorInput di;
  di.t = t_;
  di.armed = in.armed;
  di.accelG = st_.accelG;
  di.baroValid = alt_.initialized();
  di.agl = st_.agl;
  di.climbRate = st_.climbRate;
  di.mainFired = in.mainFiredInput;
  const FlightStage before = det_.stage();
  det_.update(di, dt);
  st_.stage = det_.stage();
  st_.candidate = det_.candidateFlag();
  st_.launchTime = det_.launchTime();
  st_.apogeeTime = det_.apogeeTime();

  if (st_.stage != before) {
    switch (st_.stage) {
      case FlightStage::Disarmed: st_.phase = Phase::Disarmed; break;
      case FlightStage::Pad: st_.phase = Phase::Pad; benchActive_ = false; break;
      case FlightStage::Ascent: st_.phase = Phase::Ascent; break;
      case FlightStage::Descent: st_.phase = Phase::Descent; break;
      case FlightStage::Canopy: enterCanopy(); break;
      case FlightStage::Landed: st_.phase = Phase::Landed; landedT_ = t_; break;
    }
  }

  // ---- Wind (only meaningful under canopy)
  if (st_.stage == FlightStage::Canopy && t_ - lastWindSolveT_ >= 1.0f) {
    lastWindSolveT_ = t_;
    wind_.solve(t_);
  }

  checkSafety(in, dt);

  // ---- Control
  BrakeCmd brakes;
  st_.steeringActive = false;
  st_.mode = GuideMode::None;
  if (st_.phase == Phase::Guided) {
    NavState nav = buildNav();
    GuidanceCmd cmd = guide_.update(nav, dt);
    const float actualDiff = (winch_[1].positionMm() - winch_[0].positionMm()) / p_.winch.travelMm;
    brakes = steer_.update(cmd.turnRate, cmd.symBrake, st_.yawRate, actualDiff, dt);
    st_.nav = nav;
    st_.guide = cmd;
    st_.trimNeeded = steer_.trimNeeded();
    st_.yawRateFiltered = steer_.filteredYawRate();
    st_.mode = cmd.mode;
    st_.steeringActive = true;
    // Learn the straight-flight sink rate (corrected for bank in gentle turns).
    const float r = steer_.filteredYawRate();
    if (cmd.symBrake < 0.05f && std::fabs(r) < 0.2f && t_ - st_.canopyTime > 3.0f) {
      const float bank = std::atan(nav.airspeed * std::fabs(r) / kGravity);
      trimSink_.update(descent_.value() * std::cos(bank), dt);
    }
  } else if (st_.phase == Phase::Spiral) {
    brakes = spiralBrakes_;
    st_.yawRateFiltered = st_.yawRate;
    st_.steeringActive = true;
    st_.nav = buildNav();
  } else if (st_.phase == Phase::Disarmed && benchActive_) {
    brakes = BrakeCmd{benchTarget_[0], benchTarget_[1]};
  }
  st_.brakes = brakes;

  // ---- Winches. Motors are unpowered on the pad and in the rocket; they
  // only run once the canopy is confirmed (or on the bench while disarmed).
  bool motorsOn = false;
  switch (st_.phase) {
    case Phase::Guided:
    case Phase::Spiral:
    case Phase::Neutral:
      motorsOn = true;
      break;
    case Phase::Landed:
      motorsOn = (t_ - landedT_) < 10.0f;  // release the lines, then power down
      break;
    case Phase::Disarmed:
      motorsOn = benchActive_;
      break;
    default:
      motorsOn = false;
  }
  out_.motorsEnabled = motorsOn;
  const float tgt[2] = {brakes.left, brakes.right};
  for (int ch = 0; ch < 2; ++ch) {
    const float duty = winch_[ch].update(tgt[ch], in.encoder[ch], in.batteryV, dt, motorsOn);
    out_.motorDuty[ch] = motorsOn ? duty : 0.0f;
    st_.winchMm[ch] = winch_[ch].positionMm();
    // A jam found on the bench is reported too, and stays latched into
    // flight (a winch that jammed on the bench should not fly) until reset.
    if (winch_[ch].stalled()) st_.faults |= (ch == 0 ? kFaultWinchLeft : kFaultWinchRight);
  }

  // ---- Buzzer / LED
  const float tm = std::fmod(t_, 2.0f);
  switch (st_.phase) {
    case Phase::Pad:
      // Fast chirps until a home point is saved, then one slow chirp.
      out_.buzzer = st_.homeValid ? (tm < 0.08f) : (std::fmod(t_, 0.5f) < 0.05f);
      break;
    case Phase::Landed:
      out_.buzzer = tm < 0.5f;
      break;
    default:
      out_.buzzer = false;
  }
  out_.led = st_.phase == Phase::Disarmed ? (tm < 1.0f) : (st_.steeringActive || tm < 0.1f);

  // ---- Records
  logAccum_ += dt;
  if (logAccum_ >= 1.0f / p_.logRateHz) {
    logAccum_ -= 1.0f / p_.logRateHz;
    if (logAccum_ > 1.0f) logAccum_ = 0.0f;
    fillLog(in);
    logReady_ = true;
  }
  telAccum_ += dt;
  if (telAccum_ >= 1.0f / p_.telemetryRateHz) {
    telAccum_ -= 1.0f / p_.telemetryRateHz;
    if (telAccum_ > 1.0f) telAccum_ = 0.0f;
    fillTelemetry(in);
    telReady_ = true;
  }
  return out_;
}

void FlightCore::fillLog(const Inputs& in) {
  LogRecord r;
  r.tMs = static_cast<uint32_t>(in.tUs / 1000ULL);
  r.phase = static_cast<uint8_t>(st_.phase);
  r.mode = static_cast<uint8_t>(st_.mode);
  r.faults = st_.faults;
  r.flags = static_cast<uint8_t>((st_.homeValid ? kLogHomeValid : 0) | (wind_.valid() ? kLogWindValid : 0) |
                                 (gpsFreshThisFrame_ ? kLogGpsFresh : 0) | (in.armed ? kLogArmed : 0) |
                                 (in.mainFiredInput ? kLogMainFired : 0) |
                                 (det_.launchCandidate() ? kLogLaunchCandidate : 0) |
                                 (st_.steeringActive ? kLogSteering : 0));
  r.accel[0] = sat16(in.imu.accel.x / kGravity * 100.0f);
  r.accel[1] = sat16(in.imu.accel.y / kGravity * 100.0f);
  r.accel[2] = sat16(in.imu.accel.z / kGravity * 100.0f);
  r.gyro[0] = sat16(in.imu.gyro.x * 1000.0f);
  r.gyro[1] = sat16(in.imu.gyro.y * 1000.0f);
  r.gyro[2] = sat16(in.imu.gyro.z * 1000.0f);
  r.pressurePa = in.pressurePa;
  r.agl = st_.agl;
  r.climb = sat16(st_.climbRate * 100.0f);
  r.posN = pos_.x;
  r.posE = pos_.y;
  r.velN = sat16(vel_.x * 100.0f);
  r.velE = sat16(vel_.y * 100.0f);
  r.windN = sat8(wind_.wind().x * 5.0f);
  r.windE = sat8(wind_.wind().y * 5.0f);
  r.airspeed = satU8(wind_.airspeed() * 10.0f);
  r.turnCmd = sat16(st_.guide.turnRate * 1000.0f);
  r.yawRate = sat16(st_.yawRate * 1000.0f);
  r.brakeL = satU8(st_.brakes.left * 200.0f);
  r.brakeR = satU8(st_.brakes.right * 200.0f);
  r.winchL = sat16(st_.winchMm[0] * 10.0f);
  r.winchR = sat16(st_.winchMm[1] * 10.0f);
  r.dutyL = sat8(out_.motorDuty[0] * 100.0f);
  r.dutyR = sat8(out_.motorDuty[1] * 100.0f);
  r.battery = satU8(in.batteryV * 20.0f);
  r.numSv = st_.numSv;
  sealRecord(r);
  log_ = r;
}

void FlightCore::fillTelemetry(const Inputs& in) {
  TelemetryPacket p;
  p.seq = telSeq_++;
  p.phase = static_cast<uint8_t>(st_.phase);
  p.mode = static_cast<uint8_t>(st_.mode);
  p.faults = st_.faults;
  p.latE7 = lastLatE7_;
  p.lonE7 = lastLonE7_;
  p.agl = sat16(st_.agl);
  p.climb = sat16(st_.climbRate * 10.0f);
  const float dist = st_.homeValid ? pos_.norm() : 0.0f;
  p.distHome = static_cast<uint16_t>(clampf(dist, 0.0f, 65535.0f));
  p.battery = satU8(in.batteryV * 20.0f);
  p.numSv = st_.numSv;
  p.flags = static_cast<uint8_t>((st_.homeValid ? kLogHomeValid : 0) | (wind_.valid() ? kLogWindValid : 0) |
                                 (in.armed ? kLogArmed : 0));
  sealRecord(p);
  tel_ = p;
}

bool FlightCore::takeLogRecord(LogRecord& out) {
  if (!logReady_) return false;
  out = log_;
  logReady_ = false;
  return true;
}

bool FlightCore::takeTelemetry(TelemetryPacket& out) {
  if (!telReady_) return false;
  out = tel_;
  telReady_ = false;
  return true;
}

}  // namespace gr
