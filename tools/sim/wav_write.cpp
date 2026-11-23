// wav_write.cpp - see wav_write.h. OWNER: group E.
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include "wav_write.h"

namespace sim {

namespace {

void le16(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}
void le32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

}  // namespace

bool WavWriter::open(const char* path, int sampleRate) {
  close();
  f_ = fopen(path, "wb");
  if (!f_) return false;
  n_ = 0;
  ok_ = true;
  uint8_t h[HEADER_BYTES] = {};
  h[0] = 'R'; h[1] = 'I'; h[2] = 'F'; h[3] = 'F';
  le32(h + 4, 36);  // patched on close
  h[8] = 'W'; h[9] = 'A'; h[10] = 'V'; h[11] = 'E';
  h[12] = 'f'; h[13] = 'm'; h[14] = 't'; h[15] = ' ';
  le32(h + 16, 16);                         // fmt chunk size
  le16(h + 20, 1);                          // PCM
  le16(h + 22, 1);                          // mono
  le32(h + 24, (uint32_t)sampleRate);       // sample rate
  le32(h + 28, (uint32_t)sampleRate * 2u);  // byte rate
  le16(h + 32, 2);                          // block align
  le16(h + 34, 16);                         // bits per sample
  h[36] = 'd'; h[37] = 'a'; h[38] = 't'; h[39] = 'a';
  le32(h + 40, 0);  // patched on close
  if (fwrite(h, 1, HEADER_BYTES, f_) != HEADER_BYTES) ok_ = false;
  return ok_;
}

bool WavWriter::write(const int16_t* samples, int n) {
  if (!f_ || n <= 0) return f_ != nullptr;
  uint8_t buf[1024];
  int done = 0;
  while (done < n) {
    int chunk = n - done;
    if (chunk > 512) chunk = 512;
    for (int i = 0; i < chunk; ++i) le16(buf + 2 * i, (uint16_t)samples[done + i]);
    if (fwrite(buf, 2, (size_t)chunk, f_) != (size_t)chunk) ok_ = false;
    done += chunk;
  }
  n_ += (uint32_t)n;
  return ok_;
}

bool WavWriter::close() {
  if (!f_) return ok_;
  uint8_t b[4];
  const uint32_t dataBytes = n_ * 2u;
  if (fseek(f_, 4, SEEK_SET) == 0) {
    le32(b, 36u + dataBytes);
    if (fwrite(b, 1, 4, f_) != 4) ok_ = false;
  } else {
    ok_ = false;
  }
  if (fseek(f_, 40, SEEK_SET) == 0) {
    le32(b, dataBytes);
    if (fwrite(b, 1, 4, f_) != 4) ok_ = false;
  } else {
    ok_ = false;
  }
  if (fclose(f_) != 0) ok_ = false;
  f_ = nullptr;
  return ok_;
}

}  // namespace sim
