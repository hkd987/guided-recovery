// Guidance: decides where to fly once the canopy is confirmed. The only
// possible target is the home point (origin of the local frame).
//
//  Homing  - fly the GPS ground course straight at home, brakes off.
//  Loiter  - near home with height to spare: orbit home to burn it off.
//  Final   - when the straight glide to home uses exactly the remaining
//            height, turn in and fly it, trimming the glide with symmetric
//            brake (and S-turns if still high).
//  Flare   - both brakes at the last few metres.
#pragma once

#include "gr/math.hpp"
#include "gr/params.hpp"
#include "gr/types.hpp"

namespace gr {

struct NavState {
  bool posValid = false;
  Vec2 pos;              // m from home (north, east)
  Vec2 velGround;        // m/s
  float groundSpeed = 0.0f;
  float course = 0.0f;   // rad, ground track bearing
  bool courseValid = false;
  // Heading, derived without a compass: direction of (ground velocity - wind).
  bool headingValid = false;
  float heading = 0.0f;
  float agl = 0.0f;      // m above the pad
  float sinkRate = 3.5f; // m/s, smoothed descent rate
  bool windValid = false;
  Vec2 wind;             // m/s, direction the air is moving toward
  float airspeed = 8.0f; // m/s, trim airspeed (estimated or default)
};

struct GuidanceCmd {
  GuideMode mode = GuideMode::None;
  float turnRate = 0.0f;   // rad/s, positive = clockwise seen from above (right turn)
  float symBrake = 0.0f;   // 0..1
  float courseCmd = 0.0f;  // rad, for logging
  float heightNeeded = 0.0f;  // m, straight-in requirement at nominal final brake
};

// Ground speed achievable flying straight along unit vector u, crabbing into
// the wind. Zero if the crosswind is stronger than the airspeed.
float groundSpeedAlong(const Vec2& u, const Vec2& wind, float airspeed);

class Guidance {
 public:
  Guidance(const GuidanceParams& g, const CanopyParams& c) : g_(g), c_(c) {}
  void reset();
  GuidanceCmd update(const NavState& n, float dt);

  // Height needed to glide straight from pos to home at symmetric brake b.
  float heightNeeded(const NavState& n, float brake) const;
  GuideMode mode() const { return mode_; }

 private:
  // Turn rate that makes good the ground track `track`. Steers on heading with
  // a crab angle when the wind is known (works even when the wind is faster
  // than the canopy), otherwise on GPS course.
  float steerTrack(float track, const NavState& n);
  // Heading that holds ground track `track`; into the wind if that is impossible.
  float crabHeading(float track, const NavState& n, float airspeed) const;
  bool strongWind(const NavState& n) const;
  float finalBrakeFor(const NavState& n, bool& tooHigh, float& excess) const;

  GuidanceParams g_;
  CanopyParams c_;
  GuideMode mode_ = GuideMode::None;
  int orbitDir_ = 1;
  float weaveT_ = 0.0f;
  float holdCourse_ = 0.0f;
  float turnSign_ = 1.0f;  // keeps a near-180-degree turn going one way
  bool holding_ = false;
};

}  // namespace gr
