#include "config.h"

#include <Preferences.h>

#include <cstddef>

PodSettings g_settings;

namespace {
const char* kNs = "grpod";
const uint32_t kVersion = 3;
// v2 is v3 without the trailing mainAltM: load it as a prefix.
const size_t kV2Size = offsetof(PodSettings, mainAltM);

String axisString() {
  String s;
  const char names[3] = {'x', 'y', 'z'};
  for (int i = 0; i < 3; ++i) {
    s += g_settings.imuSign[i] < 0 ? '-' : '+';
    s += names[g_settings.imuAxis[i]];
  }
  return s;
}

bool parseAxis(const String& v) {
  // "+x-y-z" style: three signed axes, each sensor axis used once.
  if (v.length() != 6) return false;
  int8_t axis[3], sign[3];
  bool used[3] = {false, false, false};
  for (int i = 0; i < 3; ++i) {
    char sc = v[2 * i], ac = v[2 * i + 1];
    if (sc != '+' && sc != '-') return false;
    int a = ac == 'x' ? 0 : ac == 'y' ? 1 : ac == 'z' ? 2 : -1;
    if (a < 0 || used[a]) return false;
    used[a] = true;
    axis[i] = static_cast<int8_t>(a);
    sign[i] = sc == '-' ? -1 : 1;
  }
  for (int i = 0; i < 3; ++i) {
    g_settings.imuAxis[i] = axis[i];
    g_settings.imuSign[i] = sign[i];
  }
  return true;
}
}  // namespace

void settingsLoad() {
  Preferences p;
  if (!p.begin(kNs, true)) return;
  const uint32_t ver = p.getUInt("ver", 0);
  const size_t len = p.getBytesLength("blob");
  if (ver == kVersion && len == sizeof(PodSettings)) {
    p.getBytes("blob", &g_settings, sizeof(PodSettings));
  } else if (ver == 2 && len == kV2Size) {
    p.getBytes("blob", &g_settings, kV2Size);  // newer fields keep their defaults
  }
  p.end();
}

bool settingsSave() {
  Preferences p;
  if (!p.begin(kNs, false)) return false;
  p.putUInt("ver", kVersion);
  const bool ok = p.putBytes("blob", &g_settings, sizeof(PodSettings)) == sizeof(PodSettings);
  p.end();
  return ok;
}

void settingsReset() { g_settings = PodSettings(); }

gr::Params settingsToParams() {
  gr::Params p;
  p.canopy.airspeed = g_settings.canopyAirspeed;
  p.canopy.sinkRate = g_settings.canopySink;
  p.canopy.turnRatePerBrake = g_settings.canopyTurnGain;
  p.winch.travelMm = g_settings.winchTravelMm;
  p.winch.countsPerMm = g_settings.winchCountsPerMm;
  p.winch.direction[0] = g_settings.winchDirL;
  p.winch.direction[1] = g_settings.winchDirR;
  p.safety.fenceRadius = g_settings.fenceRadius;
  p.det.mainAlt = g_settings.mainAltM;
  return p;
}

void settingsPrint(Print& out) {
  out.printf("canopy.airspeed  %.2f m/s\n", g_settings.canopyAirspeed);
  out.printf("canopy.sink      %.2f m/s\n", g_settings.canopySink);
  out.printf("canopy.mainalt   %.0f m%s\n", g_settings.mainAltM,
             g_settings.mainAltM > 0.0f ? " (dual deploy)" : " (0: single deploy, parafoil out at apogee)");
  out.printf("canopy.turngain  %.3f rad/s per unit brake\n", g_settings.canopyTurnGain);
  out.printf("winch.travel     %.1f mm\n", g_settings.winchTravelMm);
  out.printf("winch.cpmm       %.3f counts/mm\n", g_settings.winchCountsPerMm);
  out.printf("winch.dirL       %d\n", g_settings.winchDirL);
  out.printf("winch.dirR       %d\n", g_settings.winchDirR);
  out.printf("winch.encL       %d\n", g_settings.encSignL);
  out.printf("winch.encR       %d\n", g_settings.encSignR);
  out.printf("safety.fence     %.0f m\n", g_settings.fenceRadius);
  out.printf("imu.map          %s\n", axisString().c_str());
  out.printf("radio.mhz        %.1f\n", g_settings.radioMHz);
  out.printf("radio.dbm        %d\n", g_settings.radioDbm);
  out.printf("batt.scale       %.4f\n", g_settings.batteryScale);
}

bool settingsSet(const String& name, const String& value, String& err) {
  const float f = value.toFloat();
  const int i = value.toInt();
  auto range = [&](float lo, float hi) {
    if (f < lo || f > hi) { err = "out of range"; return false; }
    return true;
  };
  if (name == "canopy.airspeed") { if (!range(3, 16)) return false; g_settings.canopyAirspeed = f; }
  else if (name == "canopy.sink") { if (!range(1, 9)) return false; g_settings.canopySink = f; }
  else if (name == "canopy.mainalt") { if (!range(0, 3000)) return false; g_settings.mainAltM = f; }
  else if (name == "canopy.turngain") { if (!range(0.2f, 2.0f)) return false; g_settings.canopyTurnGain = f; }
  else if (name == "winch.travel") { if (!range(20, 300)) return false; g_settings.winchTravelMm = f; }
  else if (name == "winch.cpmm") { if (!range(1, 500)) return false; g_settings.winchCountsPerMm = f; }
  else if (name == "winch.dirL") { if (i != 1 && i != -1) { err = "use 1 or -1"; return false; } g_settings.winchDirL = static_cast<int8_t>(i); }
  else if (name == "winch.dirR") { if (i != 1 && i != -1) { err = "use 1 or -1"; return false; } g_settings.winchDirR = static_cast<int8_t>(i); }
  else if (name == "winch.encL") { if (i != 1 && i != -1) { err = "use 1 or -1"; return false; } g_settings.encSignL = static_cast<int8_t>(i); }
  else if (name == "winch.encR") { if (i != 1 && i != -1) { err = "use 1 or -1"; return false; } g_settings.encSignR = static_cast<int8_t>(i); }
  else if (name == "safety.fence") { if (!range(100, 5000)) return false; g_settings.fenceRadius = f; }
  else if (name == "imu.map") { if (!parseAxis(value)) { err = "format like +x-y-z"; return false; } }
  else if (name == "radio.mhz") {
    if (!((f >= 433.0f && f <= 435.0f) || (f >= 863.0f && f <= 870.0f) || (f >= 902.0f && f <= 928.0f))) {
      err = "use your region's ISM band (433, 868 or 915)";
      return false;
    }
    g_settings.radioMHz = f;
  }
  else if (name == "radio.dbm") { if (i < -9 || i > 22) { err = "-9..22"; return false; } g_settings.radioDbm = static_cast<int8_t>(i); }
  else if (name == "batt.scale") { if (!range(0.5f, 1.5f)) return false; g_settings.batteryScale = f; }
  else { err = "unknown setting"; return false; }
  return true;
}
