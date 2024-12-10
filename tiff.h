
#include <stdio.h>
#include <cstdint>
#define TIFF_SHORT 3
#define TIFF_LONG 4
#define TIFF_RATIONAL 5
#define HEADER_BLOCK_SIZE 1024

struct tiff_header {
  /* data */
  long ImageWidth;                 // 256
  long ImageLength;                // 257
  short BitsPerSample;             // 258
  short Compression;               // 259
  short PhotometricInterpretation; // 262
  // long StripOffset;                  //273
  short SamplesPerPixel; // 277
  short RowsPerStrip;    // 278
  // long StripByteCounts;               //279
  long XResolution;     // 282
  long YResolution;     // 283
  short ResolutionUnit; // 296
  long blockSize;
};
struct simple_buffer {
  uint32_t length;
  uint32_t index;
  uint8_t *buffer;
};
bool sd_init();

bool tiff_create(long image_length, long image_width);
bool tiff_write_line(uint8_t *buffer, uint32_t length);
bool tiff_close();