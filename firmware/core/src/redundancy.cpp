#include "gr/redundancy.hpp"

#include <cmath>

#include "gr/estimators.hpp"

namespace gr {

namespace {
bool plausible(float p) { return std::isfinite(p) && p > 30000.0f && p < 110000.0f; }
float isaHeight(float p) { return pressureToAltitude(p, 101325.0f); }
}  // namespace

// ------------------------------------------------------------------ BaroVoter

void BaroVoter::reset() {
  const BaroVoterParams keep = p_;
  *this = BaroVoter();
  p_ = keep;
}

float BaroVoter::residual(int i, float t, bool& valid, float* gpsChange) const {
  // Height change over the window, barometer minus GPS.
  valid = false;
  if (count_ < 2) return 0.0f;
  const Sample& now = hist_[(head_ + kHist - 1) % kHist];
  if (!now.ok[i] || !now.gps || t - now.t > 1.0f) return 0.0f;
  for (int k = count_ - 1; k >= 1; --k) {
    const Sample& old = hist_[(head_ + kHist - 1 - k) % kHist];
    if (now.t - old.t < p_.gpsWindow) continue;
    if (!old.ok[i] || !old.gps || old.seg != now.seg) return 0.0f;
    valid = true;
    if (gpsChange) *gpsChange = now.gpsH - old.gpsH;
    return (now.h[i] - old.h[i]) - (now.gpsH - old.gpsH);
  }
  return 0.0f;
}

void BaroVoter::update(float t, bool fresh1, float p1, bool present2, bool fresh2, float p2, bool calibrate,
                       bool checkGps, bool gpsGood, float gpsVelD) {
  const float dt = prevT_ < 0.0f ? 0.0f : t - prevT_;
  prevT_ = t;
  present2_ = present2;
  fresh_ = false;
  const bool f[2] = {fresh1 && plausible(p1), present2 && fresh2 && plausible(p2)};

  // Barometer 2's offset: learned on the pad; after a restart in flight,
  // from the first pair of readings. The two needn't arrive in the same
  // cycle: pair each new reading with the other's latest (if recent).
  if (f[0]) rawLatest_[0] = p1, rawT_[0] = t;
  if (f[1]) rawLatest_[1] = p2, rawT_[1] = t;
  if ((f[0] || f[1]) && present2 && std::fabs(rawT_[0] - rawT_[1]) < 0.1f && (calibrate || !offInit_)) {
    const float d = rawLatest_[1] - rawLatest_[0];
    if (!offInit_) { off2_ = d; offInit_ = true; }
    else off2_ += (d - off2_) * 0.01f;
  }
  // Reference height for the relative agreement tolerance: the pad, or the
  // first reading after a restart in flight.
  if (f[0] && (calibrate || !padInit_)) { padH_ = isaHeight(p1); padInit_ = true; }
  const float pin[2] = {p1, p2 - off2_};

  if (gpsGood) {
    // Integrate the Doppler vertical speed; a gap over 1 s starts a new segment.
    if (gpsSeen_ && t - gpsT_ <= 1.0f) gpsH_ -= gpsVelD * (t - gpsT_);
    else ++gpsSeg_;
    gpsSeen_ = true;
    gpsT_ = t;
  }
  const bool gpsRecent = gpsSeen_ && t - gpsT_ < 1.0f;

  auto avail = [&](int i) { return (i == 0 || present2_) && !failed_[i] && t - lastT_[i] <= p_.staleTime; };
  // Height of whatever else can show the pod moving: the other barometer, or GPS.
  auto otherHeight = [&](int i, bool& ok) {
    const int j = 1 - i;
    ok = true;
    if (avail(j)) return isaHeight(last_[j]);
    if (gpsRecent) return gpsH_;
    ok = false;
    return 0.0f;
  };

  for (int i = 0; i < 2; ++i) {
    if (!f[i]) continue;
    const float raw = i == 0 ? p1 : p2;
    if (std::fabs(raw - raw_[i]) > 0.005f || changeT_[i] < -1e8f) {
      raw_[i] = raw;
      changeT_[i] = t;
      bool ok;
      const float h = otherHeight(i, ok);
      hAtChange_[i] = ok ? h : NAN;
    }
    last_[i] = pin[i];
    lastT_[i] = t;
  }

  // Stuck: the same reading for a while, while something else moved.
  for (int i = 0; i < 2; ++i) {
    if (!avail(i) || t - changeT_[i] < p_.stuckTime || !std::isfinite(hAtChange_[i])) continue;
    bool ok;
    const float h = otherHeight(i, ok);
    if (ok && std::fabs(h - hAtChange_[i]) > p_.stuckMove) failed_[i] = true;
  }

  // History at 2 Hz for the GPS comparison.
  if (t - histT_ >= 0.5f) {
    histT_ = t;
    Sample sm;
    sm.t = t;
    for (int i = 0; i < 2; ++i) {
      sm.ok[i] = avail(i);
      sm.h[i] = sm.ok[i] ? isaHeight(last_[i]) : 0.0f;
    }
    sm.gps = gpsGood || (gpsRecent && t - gpsT_ < 0.3f);
    sm.gpsH = gpsH_;
    sm.seg = gpsSeg_;
    hist_[head_] = sm;
    head_ = (head_ + 1) % kHist;
    if (count_ < kHist) ++count_;
  }

  // A lone barometer must agree with GPS height change.
  const int nAvail = (avail(0) ? 1 : 0) + (avail(1) ? 1 : 0);
  for (int i = 0; i < 2; ++i) {
    bool valid = false;
    float dg = 0.0f;
    const float r = checkGps && avail(i) ? residual(i, t, valid, &dg) : 0.0f;
    const float tol = std::fmax(p_.gpsTolM, p_.gpsTolFrac * std::fabs(dg));
    if (valid && std::fabs(r) > tol) gpsBadT_[i] += dt;
    else gpsBadT_[i] = 0.0f;
    if (nAvail == 1 && gpsBadT_[i] >= p_.gpsTolTime) failed_[i] = true;
  }

  // Two barometers must agree with each other; GPS decides who is wrong.
  if (avail(0) && avail(1)) {
    const float h0 = isaHeight(last_[0]), h1 = isaHeight(last_[1]);
    const float tol = std::fmax(p_.agreeMinM, p_.agreeFrac * std::fabs(h0 - padH_));
    if (std::fabs(h0 - h1) > tol) disagreeT_ += dt;
    else disagreeT_ = 0.0f;
    if (disagreeT_ >= p_.disagreeTime && checkGps) {
      bool v0 = false, v1 = false;
      const float r0 = std::fabs(residual(0, t, v0)), r1 = std::fabs(residual(1, t, v1));
      if (v0 && v1) {
        if (r0 > r1 + p_.gpsDecideM) failed_[0] = true;
        else if (r1 > r0 + p_.gpsDecideM) failed_[1] = true;
      }
    }
  } else {
    disagreeT_ = 0.0f;
  }

  const bool a0 = avail(0), a1 = avail(1);
  if (a0 && a1) {
    fused_ = 0.5f * (last_[0] + last_[1]);
    trusted_ = disagreeT_ < p_.disagreeTime;
    fresh_ = f[0] || f[1];
  } else if (a0 || a1) {
    const int k = a0 ? 0 : 1;
    fused_ = last_[k];
    trusted_ = true;
    fresh_ = f[k];
  } else {
    trusted_ = false;
    fresh_ = false;
  }
}

// ---------------------------------------------------------------- GpsSelector

void GpsSelector::reset() {
  const float sa = switchAfter;
  *this = GpsSelector();
  switchAfter = sa;
}

void GpsSelector::update(float t, const GpsFix& g1, bool present2, const GpsFix& g2, bool calibrate) {
  present2_ = present2;
  out_.fresh = false;
  if (g1.fresh) {
    last_[0] = g1;
    if (usable(g1)) goodT_[0] = t;
  }
  if (present2 && g2.fresh) {
    last_[1] = g2;
    if (usable(g2)) goodT_[1] = t;
  }
  // Receiver 2's offset from receiver 1, learned while both see the sky on the pad.
  if (present2 && (calibrate || !offInit_) && (g1.fresh || g2.fresh) && usable(last_[0]) && usable(last_[1]) &&
      std::fabs(goodT_[0] - goodT_[1]) < 0.15f && t - goodT_[0] < 0.15f) {
    const double dLat = static_cast<double>(last_[1].latE7) - last_[0].latE7;
    const double dLon = static_cast<double>(last_[1].lonE7) - last_[0].lonE7;
    if (!offInit_) { offLat_ = dLat; offLon_ = dLon; offInit_ = true; }
    else if (calibrate) { offLat_ += (dLat - offLat_) * 0.02; offLon_ += (dLon - offLon_) * 0.02; }
  }
  // Fail over when the active receiver has had no usable fix for a while.
  if (t - goodT_[active_] > switchAfter) {
    const int other = 1 - active_;
    if ((other == 0 || present2_) && t - goodT_[other] <= switchAfter) active_ = other;
  }
  const GpsFix& src = active_ == 0 ? g1 : g2;
  if (src.fresh && (active_ == 0 || present2_)) {
    out_ = src;
    if (active_ == 1 && offInit_) {
      out_.latE7 -= static_cast<int32_t>(std::llround(offLat_));
      out_.lonE7 -= static_cast<int32_t>(std::llround(offLon_));
    }
  }
}

}  // namespace gr
