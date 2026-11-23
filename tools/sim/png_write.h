// png_write.h - minimal PNG encoder (8-bit RGB, zlib with STORED deflate blocks), a matching
// verifier/decoder for our own output, nearest-neighbour scaling, tiny caption font and
// contact sheets. OWNER: group E (simulator only; never compiled for the device).
//
// File layout written by pngEncode():
//   signature | IHDR (w, h, depth 8, colour type 2) | one IDAT | IEND
//   IDAT = zlib header 0x78 0x01 | stored deflate blocks of <= 65,535 B | Adler-32 (big-endian)
//   Every scanline starts with filter byte 0. Chunk CRC-32 is table driven.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>
#include <vector>

namespace sim {

struct Image {
  int w = 0, h = 0;
  std::vector<uint8_t> rgb;  // w*h*3, row-major, R G B
  void resize(int W, int H) {
    w = W;
    h = H;
    rgb.assign((size_t)W * (size_t)H * 3u, 0);
  }
  void set(int x, int y, uint32_t c) {
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    uint8_t* p = &rgb[((size_t)y * (size_t)w + (size_t)x) * 3u];
    p[0] = (uint8_t)(c >> 16);
    p[1] = (uint8_t)(c >> 8);
    p[2] = (uint8_t)c;
  }
};

// zlib-style chaining: crc32(b, nb, crc32(a, na)) == crc32(a||b). Start value 0.
uint32_t crc32(const uint8_t* data, size_t n, uint32_t crc = 0);
// Adler-32, start value 1.
uint32_t adler32(const uint8_t* data, size_t n, uint32_t adler = 1);
// 32-bit FNV-1a (shot hashes, determinism checks).
uint32_t fnv1a(const uint8_t* data, size_t n, uint32_t h = 2166136261u);

// Exact size in bytes of the file pngEncode() produces for a w x h image.
size_t pngExpectedSize(int w, int h);
bool pngEncode(const uint8_t* rgb, int w, int h, std::vector<uint8_t>& out);
bool pngWrite(const char* path, const Image& img);

// Decoder for files written by pngEncode (stored deflate, filter 0, 8-bit RGB only). Checks the
// signature, every chunk CRC, the zlib header, block LEN/NLEN and the Adler-32.
bool pngDecodeStored(const uint8_t* data, size_t n, Image& out, std::string* err);

// dst = src scaled k times (nearest neighbour). k >= 1.
void scaleNearest(const Image& src, int k, Image& dst);

// Caption text in a 3x5 pixel font (A-Z 0-9 _ - . + = : / space; lower case folded), each
// font pixel drawn as scale x scale. Unknown characters draw as a small box.
void drawLabel(Image& img, int x, int y, const char* text, uint32_t rgb, int scale);
int labelWidth(const char* text, int scale);

// Contact sheet: tiles laid out in `cols` columns (all tiles must have the same size), each
// with its label underneath. Returns false if there are no tiles or the sizes differ.
bool makeSheet(const std::vector<Image>& tiles, const std::vector<std::string>& labels, int cols,
               Image& out);

}  // namespace sim
