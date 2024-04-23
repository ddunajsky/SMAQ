#include <stddef.h>
#include <string.h>
#include <time.h>

#if defined(__cplusplus)
extern "C" {
#endif
const char *mg_unlist(size_t no);
const char *mg_unpack(const char *, size_t *, time_t *);
#if defined(__cplusplus)
}
#endif

static const unsigned char v1[] = {
  31, 139,   8,   8, 142,  22, 237, 101,   4,   0, 105, 110, // .......e..in
 100, 101, 120,  46, 104, 116, 109, 108,   0, 173,  86,  81, // dex.html..VQ
  79, 219,  48,  16, 126,  71, 226,  63,  28, 121,  33, 149, // O.0.~G.?.y!.
  72,  82,   2, 149,  38, 209,  68, 218,  10, 108,  60,  76, // HR..&.D..l<L
  48,   1, 147, 246, 104, 146, 131, 120, 115, 156, 204, 118, // 0...h..xs..v
 202, 170, 137, 255, 190,  75, 210, 164,   9,  45,  29,  17, // .....K...-..
 248,  37, 246, 249, 187, 239, 187, 179, 206, 231,  76, 247, // .%........L.
  78,  47, 103,  55,  63, 174, 206,  32,  49, 169,   8, 119, // N/g7?.. 1..w
 119, 166, 229,  23,   4, 147,  15, 129, 133, 210, 170,  44, // w..........,
 200, 226, 242, 155, 162,  97,  16,  37,  76, 105,  52, 129, // .....a.%Li4.
 117, 123, 115, 238, 124, 176,  90, 187, 100,  41,   6, 214, // u{s.|.Z.d)..
 156, 227,  99, 158,  41,  99,  65, 148,  73, 131, 146, 112, // ..c.)cA.I..p
 143,  60,  54,  73,  16, 227, 156,  71, 232,  84, 139,   3, // .<6I...G.T..
 224, 146,  27, 206, 132, 163,  35,  38,  48,  56, 116, 199, // ......#&08t.
  21, 143, 225,  70,  96, 120, 141,  82, 103,  10,  78, 153, // ...F`x.Rg.N.
  78, 238,  50, 166, 226, 169,  87, 217,   9, 176, 187,   3, // N.2...W.....
 203,  49, 221, 115,  28, 152,  93,  95, 131, 224, 242,  23, // .1.s..]_....
  56,  78,   8, 157, 189, 202, 166,  80,   4, 150,  54,  11, // 8N.....P..6.
 129,  58,  65, 164, 120,  18, 133, 247, 141, 197, 141, 180, // .:A.x.......
 174,  20, 189,  38, 181, 187,  44,  94, 132, 228,  76, 211, // ...&..,^..L.
 152, 207,  33,  18,  76, 235, 192,  42, 115,  96,  92, 162, // ..!.L..*s`..
  34, 112, 197, 223, 221, 212, 101, 156, 237,  78, 181, 155, // "p....e..N..
 248, 225,  13, 166,  57,  42, 102,  10, 133, 196, 238, 119, // ....9*f....w
 119, 243, 190, 167,  51, 103, 162,  64,  11, 120,  28,  88, // w...3g.@.x.X
 102, 229, 101, 133, 142,  51, 245, 242,  70, 208,  35, 197, // f.e..3..F.#.
  87, 139, 127,  41,  82,  30, 115, 179,  24, 160, 156,  44, // W..)R.s....,
  93, 222,  34, 123, 197, 148, 225, 145,  64, 248, 202, 140, // ]."{....@...
  65,   5, 190,  59,  25,  16,  65, 158, 250, 147, 183, 168, // A..;..A.....
 207,  46, 253,   1, 106,  81, 230, 191,  69, 236, 227, 183, // ....jQ..E...
 139,   1,  98, 236,  55, 223,  44, 214, 138,  46,  71, 135, // ..b.7.,...G.
  78,  71, 138, 231, 166,  91, 208, 158,   7, 231, 133, 140, // NG...[......
  12, 207,  36, 152,  12, 138,  60, 102,   6, 161,  86, 131, // ..$...<f..V.
  74,  77, 195,  35,  55,   9,  40,  38, 227,  44,   5,  89, // JM.#7.(&.,.Y
 164, 119, 168, 244, 138, 224, 190, 241, 254, 140, 166, 190, // .w..........
  93, 223,  43,  47, 123,   4, 127, 107,  84, 139,  68,  19, // ].+/{..kT.D.
  37, 246, 190, 199, 114, 238, 213,   2, 122, 127, 212, 135, // %...r...z...
 184,  38,  65, 105,  43,   8,  66,  80, 238,  79, 157,  73, // .&Ai+.BP.O.I
 123, 180,  17,  65,  65,  50,   2, 181,  10, 171,  65,  49, // {..AA2....A1
 171, 101, 248, 135,  85,  32,  16,  64, 137, 118,  59, 183, // .e..U .@.v;.
 224, 100, 155, 151, 223, 243, 106,  42, 120, 171, 203,  81, // .d....j*x..Q
 207, 165,  44, 185, 173, 240, 227,  30, 156, 106, 102,   3, // ..,......jf.
  58, 206, 162,  34, 165,  14, 231,  62, 160,  57,  19,  88, // :.."...>.9.X
  78,  63,  45,  46,  98, 187, 119, 153,  71, 148, 212,  31, // N?-.b.w.G...
  51, 171, 123,  33, 241, 117, 211,  30, 194, 217,  36, 249, // 3.{!.u....$.
   2, 161,  63, 152, 176,  60, 130,  23, 200, 142,   6, 147, // ..?..<......
 209,   1, 189, 192, 117, 188, 137, 235, 105, 244, 108, 189, // ....u...i.l.
 185,  84, 233, 246, 108, 175, 208, 152, 235, 156,  46, 228, // .T..l.......
  59,  87, 104,  89,   5,  36, 221, 171,   0,  90,  83,  14, // ;WhY.$...ZS.
 175,  58,  11, 130, 174, 157,   5, 217, 254, 119,  14,  79, // .:.......w.O
 229, 243, 214, 223, 110,  86, 171,  73, 123,  42,  43, 162, // ....nV.I{*+.
 245,  59, 125, 210, 235,  28, 183, 155, 186,   5, 206,  81, // .;}........Q
  45,  96,  66,  86, 122, 225,  98,  13, 246, 100,  60,  30, // -`BVz.b..d<.
  67, 202, 133, 224,  75,  83,  39,  50, 122, 236,  47,  40, // C...KS'2z./(
  17,  69, 190, 246,  51, 177,   3,  40,  29, 107, 197, 117, // .E..3..(.k.u
 112,  29, 236,  58, 102, 234, 213,  61, 174, 237, 137, 229, // p..:f..=....
 227,  91,  53, 197, 234,  23, 228,  31, 117, 143, 150, 170, // .[5.....u...
 146,   8,   0,   0, 0 // ....
};
static const unsigned char v2[] = {
  31, 139,   8,   8,  82, 144, 222, 101,   4,   0, 115, 116, // ....R..e..st
 121, 108, 101, 115,  46,  99, 115, 115,   0, 109, 208, 193, // yles.css.m..
  14, 130,  48,  12,   6, 224, 187, 137, 239, 208, 196, 171, // ..0.........
  51, 104,  52,  49, 243, 228, 163,  20,  54, 160, 201,  88, // 3h41....6..X
 201,  54,  20,  52, 190, 187,  67,   8,  40, 113, 183, 117, // .6.4..C.(q.u
 255, 190, 118,  75,  89, 117, 240,  92, 175,  32, 174, 156, // ..vKYu... ..
 109,  16,  57,  86, 100,  58,   9,  87,  71, 104, 182, 224, // m.9Vd:.WGh..
 209, 122, 225, 181, 163, 252,  50, 132,  42, 116,   5,  89, // .z....2.*t.Y
   9, 201, 184, 175,  81,  41, 178, 197,  80, 120, 173,  87, // ....Q)..Px.W
 187,  44,  42,  72,  86, 187, 200, 142,  55,  90, 113,  39, // .,*HV...7Zq'
  21,  74,   9, 231,  36, 169, 219,  37,   4, 216,   4,  94, // .J..$..%...^
 106, 135,  24,  28,  65, 175, 173, 231,  73,  75, 217,  41, // j...A...IK.)
 237,  36, 236, 235,  22,  60,  27,  82, 176, 201, 178, 236, // .$...<.R....
 242, 125,  40,  28,  42, 106, 188, 132, 211, 212, 108, 118, // .}(.*j....lv
 151,   3, 136, 148,  67, 224, 234,  95, 199, 242, 240, 105, // ....C.._...i
  58,  71,   3, 215, 195,  59, 231, 144, 184, 161, 105, 244, // :G...;....i.
 207,  15, 122, 122, 232, 200,  29, 123, 110, 174, 222,  53, // ..zz...{n..5
  21, 101, 144, 113,  68, 163, 122, 225,  13, 131, 159,  89, // .e.qD.z....Y
  36, 121,   1,   0,   0, 0 // $y...
};

static const struct packed_file {
  const char *name;
  const unsigned char *data;
  size_t size;
  time_t mtime;
} packed_files[] = {
  {"/web_root/index.html.gz", v1, sizeof(v1), 1710036665},
  {"/web_root/styles.css.gz", v2, sizeof(v2), 1710036662},
  {NULL, NULL, 0, 0}
};

static int scmp(const char *a, const char *b) {
  while (*a && (*a == *b)) a++, b++;
  return *(const unsigned char *) a - *(const unsigned char *) b;
}
const char *mg_unlist(size_t no) {
  return packed_files[no].name;
}
const char *mg_unpack(const char *name, size_t *size, time_t *mtime) {
  const struct packed_file *p;
  for (p = packed_files; p->name != NULL; p++) {
    if (scmp(p->name, name) != 0) continue;
    if (size != NULL) *size = p->size - 1;
    if (mtime != NULL) *mtime = p->mtime;
    return (const char *) p->data;
  }
  return NULL;
}
