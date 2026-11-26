#include "flashlog.h"

#include <esp_partition.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace {
const esp_partition_t* part = nullptr;
QueueHandle_t queue = nullptr;
uint32_t capacity = 0;
uint32_t writeIdx = 0;  // next free record slot
uint32_t dropped = 0;
constexpr size_t kRec = sizeof(gr::LogRecord);
// Stage records into 256-byte pages (4 records) for efficient programming.
uint8_t page[256];
size_t pageFill = 0;

bool slotErased(uint32_t idx) {
  uint8_t b[kRec];
  if (esp_partition_read(part, idx * kRec, b, kRec) != ESP_OK) return false;
  for (size_t i = 0; i < kRec; ++i)
    if (b[i] != 0xFF) return false;
  return true;
}

void flushPage() {
  if (pageFill == 0) return;
  const uint32_t start = writeIdx * kRec;
  if (start + pageFill <= part->size) esp_partition_write(part, start, page, pageFill);
  writeIdx += static_cast<uint32_t>(pageFill / kRec);
  pageFill = 0;
}
}  // namespace

bool flashLogBegin() {
  part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, static_cast<esp_partition_subtype_t>(0x40), "grlog");
  if (!part) return false;
  capacity = part->size / kRec;
  queue = xQueueCreate(256, kRec);
  // Binary search for the first erased slot (records are written in order).
  uint32_t lo = 0, hi = capacity;
  while (lo < hi) {
    const uint32_t mid = lo + (hi - lo) / 2;
    if (slotErased(mid)) hi = mid; else lo = mid + 1;
  }
  // Start new writes on a fresh 256-byte page boundary.
  writeIdx = ((lo * kRec + 255) / 256) * 256 / kRec;
  return true;
}

FlashLogInfo flashLogInfo() {
  FlashLogInfo i;
  i.ok = part != nullptr;
  i.capacity = capacity;
  i.used = writeIdx + static_cast<uint32_t>(pageFill / kRec);
  return i;
}

void flashLogPush(const gr::LogRecord& r) {
  if (!queue) return;
  if (xQueueSend(queue, &r, 0) != pdTRUE) ++dropped;
}

void flashLogService() {
  if (!part || !queue) return;
  gr::LogRecord r;
  while (xQueueReceive(queue, &r, 0) == pdTRUE) {
    if (writeIdx + pageFill / kRec >= capacity) { ++dropped; continue; }  // full: download and erase
    memcpy(page + pageFill, &r, kRec);
    pageFill += kRec;
    if (pageFill == sizeof(page)) flushPage();
  }
}

uint32_t flashLogDropped() { return dropped; }

bool flashLogErase(Print& out) {
  if (!part) return false;
  out.printf("Erasing %u KB of log flash (about 15 s)...\n", static_cast<unsigned>(part->size / 1024));
  const size_t block = 64 * 1024;
  for (size_t off = 0; off < part->size; off += block) {
    if (esp_partition_erase_range(part, off, block) != ESP_OK) return false;
    if ((off / block) % 8 == 0) out.print('.');
  }
  out.println(" done");
  writeIdx = 0;
  pageFill = 0;
  dropped = 0;
  return true;
}

void flashLogDumpCsv(Print& out) {
  if (!part) return;
  out.println("t_ms,phase,mode,faults,flags,ax_g,ay_g,az_g,gx,gy,gz,pressure_pa,agl,climb,pos_n,pos_e,vel_n,vel_e,"
              "wind_n,wind_e,airspeed,turn_cmd,yaw_rate,brake_l,brake_r,winch_l_mm,winch_r_mm,duty_l,duty_r,batt_v,"
              "num_sv,valid");
  gr::LogRecord r;
  for (uint32_t i = 0; i < writeIdx; ++i) {
    if (esp_partition_read(part, i * kRec, &r, kRec) != ESP_OK) break;
    if (r.magic == 0xFF) continue;  // padding up to a page boundary
    out.printf("%u,%u,%u,%u,%u,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,"
               "%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%d,%d,%.2f,%u,%d\n",
               static_cast<unsigned>(r.tMs), r.phase, r.mode, r.faults, r.flags, r.accel[0] * 0.01f,
               r.accel[1] * 0.01f, r.accel[2] * 0.01f, r.gyro[0] * 0.001f, r.gyro[1] * 0.001f, r.gyro[2] * 0.001f,
               r.pressurePa, r.agl, r.climb * 0.01f, r.posN, r.posE, r.velN * 0.01f, r.velE * 0.01f,
               r.windN * 0.2f, r.windE * 0.2f, r.airspeed * 0.1f, r.turnCmd * 0.001f, r.yawRate * 0.001f,
               r.brakeL / 200.0f, r.brakeR / 200.0f, r.winchL * 0.1f, r.winchR * 0.1f, r.dutyL, r.dutyR,
               r.battery * 0.05f, r.numSv, gr::recordValid(r) ? 1 : 0);
    if ((i & 63) == 0) delay(1);  // let the USB serial drain
  }
}
