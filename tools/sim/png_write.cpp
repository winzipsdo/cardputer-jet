// png_write.cpp - see png_write.h. OWNER: group E.
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include "png_write.h"

#include <stdio.h>
#include <string.h>

namespace sim {

namespace {

struct CrcTable {
  uint32_t t[256];
  CrcTable() {
    for (uint32_t n = 0; n < 256; ++n) {
      uint32_t c = n;
      for (int k = 0; k < 8; ++k) c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      t[n] = c;
    }
  }
};

const CrcTable& crcTable() {
  static const CrcTable table;
  return table;
}

const size_t STORED_MAX = 65535;

void put32be(std::vector<uint8_t>& o, uint32_t v) {
  o.push_back((uint8_t)(v >> 24));
  o.push_back((uint8_t)(v >> 16));
  o.push_back((uint8_t)(v >> 8));
  o.push_back((uint8_t)v);
}

uint32_t get32be(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

void putChunk(std::vector<uint8_t>& o, const char type[4], const uint8_t* data, size_t n) {
  put32be(o, (uint32_t)n);
  const size_t typePos = o.size();
  o.insert(o.end(), (const uint8_t*)type, (const uint8_t*)type + 4);
  if (n) o.insert(o.end(), data, data + n);
  put32be(o, crc32(&o[typePos], n + 4));
}

size_t rawSize(int w, int h) { return (size_t)h * (1u + (size_t)w * 3u); }

size_t zlibSize(size_t raw) {
  size_t blocks = raw == 0 ? 1 : (raw + STORED_MAX - 1) / STORED_MAX;
  return 2 + blocks * 5 + raw + 4;
}

// ---- 3x5 caption font: 5 rows of 3 pixels, '1' = ink, top row first.
struct Glyph {
  char c;
  const char* bits;
};
const Glyph GLYPHS[] = {
    {'0', "111101101101111"}, {'1', "010110010010111"}, {'2', "111001111100111"},
    {'3', "111001111001111"}, {'4', "101101111001001"}, {'5', "111100111001111"},
    {'6', "111100111101111"}, {'7', "111001001010010"}, {'8', "111101111101111"},
    {'9', "111101111001111"}, {'A', "010101111101101"}, {'B', "110101110101110"},
    {'C', "011100100100011"}, {'D', "110101101101110"}, {'E', "111100110100111"},
    {'F', "111100110100100"}, {'G', "011100101101011"}, {'H', "101101111101101"},
    {'I', "111010010010111"}, {'J', "001001001101010"}, {'K', "101101110101101"},
    {'L', "100100100100111"}, {'M', "101111111101101"}, {'N', "110101101101101"},
    {'O', "010101101101010"}, {'P', "110101110100100"}, {'Q', "010101101110011"},
    {'R', "110101110101101"}, {'S', "011100010001110"}, {'T', "111010010010010"},
    {'U', "101101101101111"}, {'V', "101101101101010"}, {'W', "101101111111101"},
    {'X', "101101010101101"}, {'Y', "101101010010010"}, {'Z', "111001010100111"},
    {'_', "000000000000111"}, {'-', "000000111000000"}, {'.', "000000000000010"},
    {'+', "000010111010000"}, {'=', "000111000111000"}, {':', "000010000010000"},
    {'/', "001001010100100"}, {' ', "000000000000000"},
};
const char* UNKNOWN_GLYPH = "111101101101111";

const char* glyphBits(char c) {
  if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
  for (const Glyph& g : GLYPHS)
    if (g.c == c) return g.bits;
  return UNKNOWN_GLYPH;
}

}  // namespace

uint32_t crc32(const uint8_t* data, size_t n, uint32_t crc) {
  const CrcTable& tb = crcTable();
  uint32_t c = crc ^ 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) c = tb.t[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

uint32_t adler32(const uint8_t* data, size_t n, uint32_t adler) {
  uint32_t a = adler & 0xFFFFu, b = adler >> 16;
  while (n > 0) {
    size_t chunk = n < 5552 ? n : 5552;  // largest run without 32-bit overflow
    n -= chunk;
    while (chunk--) {
      a += *data++;
      b += a;
    }
    a %= 65521u;
    b %= 65521u;
  }
  return (b << 16) | a;
}

uint32_t fnv1a(const uint8_t* data, size_t n, uint32_t h) {
  for (size_t i = 0; i < n; ++i) {
    h ^= data[i];
    h *= 16777619u;
  }
  return h;
}

size_t pngExpectedSize(int w, int h) {
  if (w <= 0 || h <= 0) return 0;
  return 8 + (12 + 13) + (12 + zlibSize(rawSize(w, h))) + 12;
}

bool pngEncode(const uint8_t* rgb, int w, int h, std::vector<uint8_t>& out) {
  out.clear();
  if (rgb == nullptr || w <= 0 || h <= 0) return false;
  static const uint8_t SIG[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  out.reserve(pngExpectedSize(w, h));
  out.insert(out.end(), SIG, SIG + 8);

  uint8_t ihdr[13];
  ihdr[0] = (uint8_t)((uint32_t)w >> 24); ihdr[1] = (uint8_t)((uint32_t)w >> 16);
  ihdr[2] = (uint8_t)((uint32_t)w >> 8);  ihdr[3] = (uint8_t)w;
  ihdr[4] = (uint8_t)((uint32_t)h >> 24); ihdr[5] = (uint8_t)((uint32_t)h >> 16);
  ihdr[6] = (uint8_t)((uint32_t)h >> 8);  ihdr[7] = (uint8_t)h;
  ihdr[8] = 8;   // bit depth
  ihdr[9] = 2;   // colour type RGB
  ihdr[10] = 0;  // deflate
  ihdr[11] = 0;  // adaptive filtering (we always use filter 0)
  ihdr[12] = 0;  // no interlace
  putChunk(out, "IHDR", ihdr, 13);

  // Raw scanlines: filter byte 0 + w*3 bytes.
  const size_t rowBytes = (size_t)w * 3u;
  std::vector<uint8_t> raw(rawSize(w, h));
  for (int y = 0; y < h; ++y) {
    uint8_t* r = &raw[(size_t)y * (rowBytes + 1)];
    r[0] = 0;
    memcpy(r + 1, rgb + (size_t)y * rowBytes, rowBytes);
  }

  // zlib stream with stored deflate blocks.
  std::vector<uint8_t> z;
  z.reserve(zlibSize(raw.size()));
  z.push_back(0x78);
  z.push_back(0x01);
  size_t pos = 0;
  do {
    size_t len = raw.size() - pos;
    if (len > STORED_MAX) len = STORED_MAX;
    const bool final = pos + len >= raw.size();
    z.push_back(final ? 1 : 0);  // BFINAL, BTYPE = 00 (stored), padded to the byte boundary
    z.push_back((uint8_t)len);
    z.push_back((uint8_t)(len >> 8));
    z.push_back((uint8_t)~len);
    z.push_back((uint8_t)(~len >> 8));
    if (len) z.insert(z.end(), raw.begin() + (ptrdiff_t)pos, raw.begin() + (ptrdiff_t)(pos + len));
    pos += len;
  } while (pos < raw.size());
  put32be(z, adler32(raw.data(), raw.size()));
  putChunk(out, "IDAT", z.data(), z.size());
  putChunk(out, "IEND", nullptr, 0);
  return out.size() == pngExpectedSize(w, h);
}

bool pngWrite(const char* path, const Image& img) {
  std::vector<uint8_t> buf;
  if (!pngEncode(img.rgb.data(), img.w, img.h, buf)) return false;
  FILE* f = fopen(path, "wb");
  if (!f) return false;
  const size_t wr = fwrite(buf.data(), 1, buf.size(), f);
  const int cl = fclose(f);
  return wr == buf.size() && cl == 0;
}

bool pngDecodeStored(const uint8_t* d, size_t n, Image& out, std::string* err) {
  auto fail = [&](const char* m) {
    if (err) *err = m;
    return false;
  };
  static const uint8_t SIG[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  if (n < 8 || memcmp(d, SIG, 8) != 0) return fail("bad signature");
  size_t p = 8;
  int w = 0, h = 0;
  bool haveHdr = false, haveEnd = false;
  std::vector<uint8_t> z;
  while (p + 12 <= n && !haveEnd) {
    const uint32_t len = get32be(d + p);
    if (p + 12 + (size_t)len > n) return fail("truncated chunk");
    const uint8_t* type = d + p + 4;
    const uint8_t* data = d + p + 8;
    if (crc32(type, (size_t)len + 4) != get32be(data + len)) return fail("chunk crc mismatch");
    if (memcmp(type, "IHDR", 4) == 0) {
      if (len != 13) return fail("bad IHDR length");
      w = (int)get32be(data);
      h = (int)get32be(data + 4);
      if (w <= 0 || h <= 0 || data[8] != 8 || data[9] != 2 || data[10] != 0 || data[11] != 0 ||
          data[12] != 0)
        return fail("unsupported IHDR");
      haveHdr = true;
    } else if (memcmp(type, "IDAT", 4) == 0) {
      z.insert(z.end(), data, data + len);
    } else if (memcmp(type, "IEND", 4) == 0) {
      haveEnd = true;
    }
    p += 12 + (size_t)len;
  }
  if (!haveHdr || !haveEnd) return fail("missing IHDR or IEND");
  if (z.size() < 6) return fail("IDAT too short");
  if ((z[0] & 0x0F) != 8 || ((unsigned)z[0] * 256u + z[1]) % 31u != 0 || (z[1] & 0x20))
    return fail("bad zlib header");
  std::vector<uint8_t> raw;
  size_t q = 2;
  bool final = false;
  while (!final) {
    if (q + 5 > z.size()) return fail("truncated deflate block");
    const uint8_t hdr = z[q];
    final = (hdr & 1) != 0;
    if ((hdr >> 1) & 3) return fail("deflate block is not stored");
    const unsigned len = (unsigned)z[q + 1] | ((unsigned)z[q + 2] << 8);
    const unsigned nlen = (unsigned)z[q + 3] | ((unsigned)z[q + 4] << 8);
    if ((len ^ 0xFFFFu) != nlen) return fail("LEN/NLEN mismatch");
    q += 5;
    if (q + len > z.size()) return fail("stored block overruns IDAT");
    raw.insert(raw.end(), z.begin() + (ptrdiff_t)q, z.begin() + (ptrdiff_t)(q + len));
    q += len;
  }
  if (q + 4 > z.size()) return fail("missing adler32");
  if (adler32(raw.data(), raw.size()) != get32be(&z[q])) return fail("adler32 mismatch");
  if (raw.size() != rawSize(w, h)) return fail("raw size mismatch");
  out.resize(w, h);
  const size_t rowBytes = (size_t)w * 3u;
  for (int y = 0; y < h; ++y) {
    const uint8_t* r = &raw[(size_t)y * (rowBytes + 1)];
    if (r[0] != 0) return fail("unsupported filter type");
    memcpy(&out.rgb[(size_t)y * rowBytes], r + 1, rowBytes);
  }
  return true;
}

void scaleNearest(const Image& src, int k, Image& dst) {
  if (k < 1) k = 1;
  dst.resize(src.w * k, src.h * k);
  for (int y = 0; y < dst.h; ++y) {
    const uint8_t* srow = &src.rgb[(size_t)(y / k) * (size_t)src.w * 3u];
    uint8_t* drow = &dst.rgb[(size_t)y * (size_t)dst.w * 3u];
    for (int x = 0; x < dst.w; ++x) {
      const uint8_t* s = srow + (size_t)(x / k) * 3u;
      drow[x * 3 + 0] = s[0];
      drow[x * 3 + 1] = s[1];
      drow[x * 3 + 2] = s[2];
    }
  }
}

int labelWidth(const char* text, int scale) {
  int n = (int)strlen(text);
  return n > 0 ? (n * 4 - 1) * scale : 0;
}

void drawLabel(Image& img, int x, int y, const char* text, uint32_t rgb, int scale) {
  for (int i = 0; text[i]; ++i) {
    const char* bits = glyphBits(text[i]);
    for (int gy = 0; gy < 5; ++gy)
      for (int gx = 0; gx < 3; ++gx) {
        if (bits[gy * 3 + gx] != '1') continue;
        for (int sy = 0; sy < scale; ++sy)
          for (int sx = 0; sx < scale; ++sx)
            img.set(x + (i * 4 + gx) * scale + sx, y + gy * scale + sy, rgb);
      }
  }
}

bool makeSheet(const std::vector<Image>& tiles, const std::vector<std::string>& labels, int cols,
               Image& out) {
  if (tiles.empty()) return false;
  const int tw = tiles[0].w, th = tiles[0].h;
  for (const Image& t : tiles)
    if (t.w != tw || t.h != th) return false;
  if (cols < 1) cols = 1;
  const int n = (int)tiles.size();
  if (cols > n) cols = n;
  const int rows = (n + cols - 1) / cols;
  const int gap = 4, labelScale = 2, labelH = 5 * labelScale + 6;
  const int cellW = tw + gap, cellH = th + labelH;
  out.resize(gap + cols * cellW, gap + rows * cellH);
  // dark slate background
  for (size_t i = 0; i < out.rgb.size(); i += 3) {
    out.rgb[i] = 0x18;
    out.rgb[i + 1] = 0x1A;
    out.rgb[i + 2] = 0x22;
  }
  for (int i = 0; i < n; ++i) {
    const int ox = gap + (i % cols) * cellW, oy = gap + (i / cols) * cellH;
    const Image& t = tiles[(size_t)i];
    for (int y = 0; y < th; ++y)
      memcpy(&out.rgb[((size_t)(oy + y) * (size_t)out.w + (size_t)ox) * 3u],
             &t.rgb[(size_t)y * (size_t)tw * 3u], (size_t)tw * 3u);
    if ((size_t)i < labels.size())
      drawLabel(out, ox + 1, oy + th + 3, labels[(size_t)i].c_str(), 0xE8ECF4u, labelScale);
  }
  return true;
}

}  // namespace sim
