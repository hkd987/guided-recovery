#include "sensors.h"

#include <Wire.h>

#include "board.h"
#include "config.h"

SensorStatus g_sensors;

namespace {

constexpr float kG = 9.80665f;
constexpr float kDegToRad = 0.017453293f;

bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(addr), static_cast<int>(n)) != static_cast<int>(n)) return false;
  for (size_t i = 0; i < n; ++i) buf[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

int readId(uint8_t addr, uint8_t reg) {
  uint8_t v = 0;
  return readRegs(addr, reg, &v, 1) ? v : -1;
}

int16_t le16(const uint8_t* p) { return static_cast<int16_t>(p[0] | (p[1] << 8)); }

// ---- BMP390 calibration (Bosch floating-point compensation)
struct Bmp390Cal {
  double t1, t2, t3;
  double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
} cal;
double tLin = 0.0;

bool bmp390Init(uint8_t addr) {
  writeReg(addr, 0x7E, 0xB6);  // soft reset
  delay(10);
  uint8_t c[21];
  if (!readRegs(addr, 0x31, c, sizeof c)) return false;
  auto u16 = [&](int i) { return static_cast<uint16_t>(c[i] | (c[i + 1] << 8)); };
  auto s16 = [&](int i) { return static_cast<int16_t>(c[i] | (c[i + 1] << 8)); };
  auto s8 = [&](int i) { return static_cast<int8_t>(c[i]); };
  cal.t1 = u16(0) / 0.00390625;           // / 2^-8
  cal.t2 = u16(2) / 1073741824.0;         // / 2^30
  cal.t3 = s8(4) / 281474976710656.0;     // / 2^48
  cal.p1 = (s16(5) - 16384) / 1048576.0;  // (P1 - 2^14) / 2^20
  cal.p2 = (s16(7) - 16384) / 536870912.0;  // (P2 - 2^14) / 2^29
  cal.p3 = s8(9) / 4294967296.0;          // / 2^32
  cal.p4 = s8(10) / 137438953472.0;       // / 2^37
  cal.p5 = u16(11) / 0.125;               // / 2^-3
  cal.p6 = u16(13) / 64.0;                // / 2^6
  cal.p7 = s8(15) / 256.0;                // / 2^8
  cal.p8 = s8(16) / 32768.0;              // / 2^15
  cal.p9 = s16(17) / 281474976710656.0;   // / 2^48
  cal.p10 = s8(19) / 281474976710656.0;   // / 2^48
  cal.p11 = s8(20) / 36893488147419103232.0;  // / 2^65
  bool ok = writeReg(addr, 0x1C, 0x03);   // OSR: pressure x8, temperature x1
  ok &= writeReg(addr, 0x1D, 0x02);       // ODR 50 Hz
  ok &= writeReg(addr, 0x1F, 0x04);       // IIR coefficient 3
  ok &= writeReg(addr, 0x1B, 0x33);       // pressure + temperature, normal mode
  return ok;
}

double bmp390Temp(uint32_t ut) {
  const double pd1 = static_cast<double>(ut) - cal.t1;
  const double pd2 = pd1 * cal.t2;
  tLin = pd2 + pd1 * pd1 * cal.t3;
  return tLin;
}

double bmp390Press(uint32_t up) {
  const double t = tLin, t2 = t * t, t3 = t2 * t, u = static_cast<double>(up);
  const double out1 = cal.p5 + cal.p6 * t + cal.p7 * t2 + cal.p8 * t3;
  const double out2 = u * (cal.p1 + cal.p2 * t + cal.p3 * t2 + cal.p4 * t3);
  const double u2 = u * u;
  const double out3 = u2 * (cal.p9 + cal.p10 * t) + u2 * u * cal.p11;
  return out1 + out2 + out3;
}

}  // namespace

void sensorsBegin() {
  Wire.begin(board::kI2cSda, board::kI2cScl, board::kI2cHz);
  Wire.setTimeOut(5);
  g_sensors = SensorStatus();

  // BMI088: accelerometer 0x18/0x19 (id 0x1E), gyro 0x68/0x69 (id 0x0F).
  for (uint8_t a : {0x18, 0x19}) {
    readId(a, 0x00);  // first read after power-up may be junk
    if (readId(a, 0x00) == 0x1E) { g_sensors.imuAddr = a; break; }
  }
  for (uint8_t g : {0x68, 0x69}) {
    if (readId(g, 0x00) == 0x0F) { g_sensors.gyroAddr = g; break; }
  }
  if (g_sensors.imuAddr && g_sensors.gyroAddr) {
    const uint8_t a = g_sensors.imuAddr, g = g_sensors.gyroAddr;
    writeReg(a, 0x7E, 0xB6);
    delay(2);
    readId(a, 0x00);
    writeReg(a, 0x7D, 0x04);  // ACC_PWR_CTRL: accelerometer on
    delay(5);
    writeReg(a, 0x7C, 0x00);  // ACC_PWR_CONF: active
    delay(5);
    writeReg(a, 0x41, 0x03);  // +-24 g
    writeReg(a, 0x40, 0xAA);  // normal bandwidth, 400 Hz
    writeReg(g, 0x14, 0xB6);
    delay(30);
    writeReg(g, 0x0F, 0x00);  // +-2000 dps
    writeReg(g, 0x10, 0x03);  // 400 Hz ODR, 47 Hz filter
    g_sensors.imu = ImuType::Bmi088;
  } else {
    // LSM6DSO32 at 0x6A/0x6B, WHO_AM_I 0x6C.
    for (uint8_t a : {0x6A, 0x6B}) {
      if (readId(a, 0x0F) == 0x6C) {
        g_sensors.imuAddr = a;
        writeReg(a, 0x12, 0x01);  // software reset
        delay(10);
        writeReg(a, 0x12, 0x44);  // BDU, auto-increment
        writeReg(a, 0x10, 0x64);  // accel 416 Hz, +-32 g
        writeReg(a, 0x11, 0x6C);  // gyro 416 Hz, 2000 dps
        g_sensors.imu = ImuType::Lsm6dso32;
        break;
      }
    }
  }

  // BMP390 at 0x77/0x76, chip id 0x60.
  for (uint8_t b : {0x77, 0x76}) {
    if (readId(b, 0x00) == 0x60 && bmp390Init(b)) {
      g_sensors.baro = true;
      g_sensors.baroAddr = b;
      break;
    }
  }

  // ADXL375 at 0x53, DEVID 0xE5 (optional).
  if (readId(0x53, 0x00) == 0xE5) {
    writeReg(0x53, 0x2C, 0x0B);  // 200 Hz
    writeReg(0x53, 0x31, 0x0B);
    writeReg(0x53, 0x2D, 0x08);  // measure
    g_sensors.highG = true;
  }
}

bool imuRead(gr::Vec3& accel, gr::Vec3& gyro) {
  uint8_t b[6];
  if (g_sensors.imu == ImuType::Bmi088) {
    if (!readRegs(g_sensors.imuAddr, 0x12, b, 6)) { ++g_sensors.imuErrors; return false; }
    const float as = 24.0f * kG / 32768.0f;
    accel = gr::Vec3{le16(b) * as, le16(b + 2) * as, le16(b + 4) * as};
    if (!readRegs(g_sensors.gyroAddr, 0x02, b, 6)) { ++g_sensors.imuErrors; return false; }
    const float gs = 2000.0f / 32768.0f * kDegToRad;
    gyro = gr::Vec3{le16(b) * gs, le16(b + 2) * gs, le16(b + 4) * gs};
    return true;
  }
  if (g_sensors.imu == ImuType::Lsm6dso32) {
    uint8_t d[12];
    if (!readRegs(g_sensors.imuAddr, 0x22, d, 12)) { ++g_sensors.imuErrors; return false; }
    const float gs = 0.070f * kDegToRad;   // 70 mdps/LSB at 2000 dps
    const float as = 0.000976f * kG;       // 0.976 mg/LSB at 32 g
    gyro = gr::Vec3{le16(d) * gs, le16(d + 2) * gs, le16(d + 4) * gs};
    accel = gr::Vec3{le16(d + 6) * as, le16(d + 8) * as, le16(d + 10) * as};
    return true;
  }
  return false;
}

gr::Vec3 toBody(const gr::Vec3& v) {
  const float s[3] = {v.x, v.y, v.z};
  return gr::Vec3{g_settings.imuSign[0] * s[g_settings.imuAxis[0]], g_settings.imuSign[1] * s[g_settings.imuAxis[1]],
                  g_settings.imuSign[2] * s[g_settings.imuAxis[2]]};
}

bool baroRead(float& pressurePa, float& tempC) {
  if (!g_sensors.baro) return false;
  uint8_t st = 0;
  if (!readRegs(g_sensors.baroAddr, 0x03, &st, 1)) { ++g_sensors.baroErrors; return false; }
  if ((st & 0x60) != 0x60) return false;  // pressure and temperature data ready
  uint8_t d[6];
  if (!readRegs(g_sensors.baroAddr, 0x04, d, 6)) { ++g_sensors.baroErrors; return false; }
  const uint32_t up = d[0] | (d[1] << 8) | (static_cast<uint32_t>(d[2]) << 16);
  const uint32_t ut = d[3] | (d[4] << 8) | (static_cast<uint32_t>(d[5]) << 16);
  tempC = static_cast<float>(bmp390Temp(ut));
  pressurePa = static_cast<float>(bmp390Press(up));
  return pressurePa > 1000.0f && pressurePa < 120000.0f;
}

bool highGRead(gr::Vec3& accel) {
  if (!g_sensors.highG) return false;
  uint8_t b[6];
  if (!readRegs(0x53, 0x32, b, 6)) return false;
  const float s = 0.049f * kG;  // 49 mg/LSB
  accel = gr::Vec3{le16(b) * s, le16(b + 2) * s, le16(b + 4) * s};
  return true;
}

void i2cScan(Print& out) {
  out.print("  I2C devices:");
  int n = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { out.printf(" 0x%02X", a); ++n; }
  }
  out.printf("%s\n", n ? "" : " none");
}
