// plat_storage.cpp - Persist blob in NVS: Preferences namespace "moonskim", key "p".
// OWNER: group A. The game owns the content and debounces saves (cfg::PERSIST_DEBOUNCE_S);
// the platform only stores bytes. A size or version mismatch loads the defaults.
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "core/config.h"
#include "core/game.h"
#include "platform/cardputer/plat.h"
#include "platform/cardputer/plat_internal.h"

namespace plat {

namespace {

constexpr const char* NVS_NS = "moonskim";
constexpr const char* NVS_KEY = "p";

Preferences g_prefs;
bool g_open = false;

bool ensureOpen() {
  if (g_open) return true;
  g_open = g_prefs.begin(NVS_NS, false);
  if (!g_open) logLine("[ERR] nvs_open");
  return g_open;
}

// Repair individual fields that would be harmful if a blob of the right size/version were
// corrupted (a bad sign flag would silently freeze the tilt; a NaN neutral would poison it).
void sanitize(kj::Persist& p) {
  const kj::Persist d = kj::defaultPersist();
  if (p.tod >= (uint8_t)kj::Tod::Count) p.tod = d.tod;
  if (p.sens > (uint8_t)kj::Sens::High) p.sens = d.sens;
  if (p.assist > (uint8_t)kj::Assist::Pro) p.assist = d.assist;
  if (p.view > 2) p.view = d.view;
  p.tiltOn = p.tiltOn ? 1 : 0;
  p.invert = p.invert ? 1 : 0;
  p.mute = p.mute ? 1 : 0;
  p.drift = p.drift ? 1 : 0;
  if (p.rollSign != 1 && p.rollSign != -1) p.rollSign = d.rollSign;
  if (p.pitchSign != 1 && p.pitchSign != -1) p.pitchSign = d.pitchSign;
  if (p.bright < kj::cfg::BRIGHT_MIN) p.bright = kj::cfg::BRIGHT_MIN;
  const float n2 = p.nx * p.nx + p.ny * p.ny + p.nz * p.nz;
  if (!isfinite(p.nx) || !isfinite(p.ny) || !isfinite(p.nz) || n2 < 0.5f || n2 > 1.5f) {
    p.nx = d.nx;
    p.ny = d.ny;
    p.nz = d.nz;
  }
}

}  // namespace

void savePersist(const kj::Persist& p) {
  if (!ensureOpen()) return;
  const size_t w = g_prefs.putBytes(NVS_KEY, &p, sizeof(p));
  if (w != sizeof(p)) {
    detail::logf("[ERR] nvs_write wrote=%u want=%u", (unsigned)w, (unsigned)sizeof(p));
  }
}

namespace detail {

bool storageLoad(kj::Persist& out) {
  out = kj::defaultPersist();
  if (!ensureOpen()) return false;
  const size_t len = g_prefs.isKey(NVS_KEY) ? g_prefs.getBytesLength(NVS_KEY) : 0;
  if (len != sizeof(kj::Persist)) {
    logf("[NVS] defaults reason=%s len=%u want=%u", len ? "size" : "empty", (unsigned)len,
         (unsigned)sizeof(kj::Persist));
    return false;
  }
  kj::Persist p = {};
  if (g_prefs.getBytes(NVS_KEY, &p, sizeof(p)) != sizeof(p) || p.version != kj::PERSIST_VERSION) {
    logf("[NVS] defaults reason=version have=%lu want=%lu", (unsigned long)p.version,
         (unsigned long)kj::PERSIST_VERSION);
    return false;
  }
  sanitize(p);
  out = p;
  logf("[NVS] loaded best=%lu vol=%u bri=%u tilt=%u rs=%d ps=%d", (unsigned long)p.best,
       (unsigned)p.volume, (unsigned)p.bright, (unsigned)p.tiltOn, (int)p.rollSign,
       (int)p.pitchSign);
  return true;
}

}  // namespace detail
}  // namespace plat
