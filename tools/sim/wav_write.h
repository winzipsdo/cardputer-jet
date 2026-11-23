// wav_write.h - streaming 16-bit mono PCM WAV writer (sim --wav). OWNER: group E.
// The header is written with placeholder sizes on open() and patched on close().
#pragma once
#include <stdint.h>
#include <stdio.h>

namespace sim {

class WavWriter {
 public:
  WavWriter() = default;
  WavWriter(const WavWriter&) = delete;
  WavWriter& operator=(const WavWriter&) = delete;
  ~WavWriter() { close(); }

  bool open(const char* path, int sampleRate);
  bool write(const int16_t* samples, int n);
  bool close();  // patches the RIFF/data sizes; safe to call twice
  bool isOpen() const { return f_ != nullptr; }
  uint32_t samples() const { return n_; }

  static constexpr uint32_t HEADER_BYTES = 44;

 private:
  FILE* f_ = nullptr;
  uint32_t n_ = 0;
  bool ok_ = true;
};

}  // namespace sim
