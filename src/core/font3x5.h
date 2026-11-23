// font3x5.h - original 3x5 pixel font for the HUD (group D).
// Each glyph is 5 rows of 3 bits, written as a 5-digit OCTAL number: digit i = row i
// (top first), bit 2 = left pixel, bit 0 = right pixel. Row i of glyph g is
//   (g >> (3 * (4 - i))) & 7
// Table covers ASCII 32..95 (upper case). Lower case is folded to upper case by hudText.
// Extra symbols (control codes) are listed in FONT3X5_EXTRA.
#pragma once
#include <stdint.h>

namespace kj {

constexpr uint16_t FONT3X5[64] = {
  000000,  // 32 ' '
  022202,  // 33 '!'
  055000,  // 34 '"'
  057575,  // 35 '#'
  036236,  // 36 '$'
  051245,  // 37 '%'
  025253,  // 38 '&'
  022000,  // 39 '''
  012221,  // 40 '('
  042224,  // 41 ')'
  005250,  // 42 '*'
  002720,  // 43 '+'
  000024,  // 44 ','
  000700,  // 45 '-'
  000002,  // 46 '.'
  011244,  // 47 '/'
  075557,  // 48 '0'
  026227,  // 49 '1'
  061247,  // 50 '2'
  061216,  // 51 '3'
  055711,  // 52 '4'
  074616,  // 53 '5'
  034757,  // 54 '6'
  071222,  // 55 '7'
  075757,  // 56 '8'
  075716,  // 57 '9'
  002020,  // 58 ':'
  002024,  // 59 ';'
  012421,  // 60 '<'
  007070,  // 61 '='
  042124,  // 62 '>'
  061202,  // 63 '?'
  025743,  // 64 '@'
  025755,  // 65 'A'
  065656,  // 66 'B'
  034443,  // 67 'C'
  065556,  // 68 'D'
  074647,  // 69 'E'
  074644,  // 70 'F'
  034553,  // 71 'G'
  055755,  // 72 'H'
  072227,  // 73 'I'
  011152,  // 74 'J'
  056465,  // 75 'K'
  044447,  // 76 'L'
  057755,  // 77 'M'
  065555,  // 78 'N'
  025552,  // 79 'O'
  065644,  // 80 'P'
  025563,  // 81 'Q'
  065655,  // 82 'R'
  074717,  // 83 'S'
  072222,  // 84 'T'
  055557,  // 85 'U'
  055552,  // 86 'V'
  055775,  // 87 'W'
  055255,  // 88 'X'
  055222,  // 89 'Y'
  071247,  // 90 'Z'
  064446,  // 91 '['
  044211,  // 92 '\'
  031113,  // 93 ']'
  025000,  // 94 '^'
  000007,  // 95 '_'
};

// Control-code glyphs usable inside HUD strings.
constexpr char GLYPH_UP = '\x01';     // small solid up triangle
constexpr char GLYPH_DOWN = '\x02';   // small solid down triangle
constexpr char GLYPH_LEFT = '\x03';   // left chevron
constexpr char GLYPH_RIGHT = '\x04';  // right chevron
constexpr char GLYPH_BAR = '\x05';    // full block
constexpr uint16_t FONT3X5_EXTRA[6] = {
  000000,  // 0 (unused)
  002700,  // 1 up triangle
  007200,  // 2 down triangle
  012421,  // 3 left chevron
  042124,  // 4 right chevron
  077777,  // 5 block
};

inline uint16_t fontGlyph(char ch) {
  int c = (unsigned char)ch;
  if (c >= 1 && c <= 5) return FONT3X5_EXTRA[c];
  if (c >= 'a' && c <= 'z') c -= 32;
  if (c == '`') c = '\'';
  if (c == '{') c = '(';
  if (c == '}') c = ')';
  if (c == '|') return 022222;
  if (c == '~') return 000630;
  if (c < 32 || c > 95) return 0;
  return FONT3X5[c - 32];
}

}  // namespace kj
