#include "tiff.h"
#include "SdFat.h"
#define SD_CONFIG SdioConfig(FIFO_SDIO)

SdExFat sd;
ExFile file;

void putu8(uint8_t in, struct simple_buffer *buffer) {
  buffer->buffer[buffer->index] = in;
  buffer->index++;
  if (buffer->index == HEADER_BLOCK_SIZE) {
    file.write(buffer->buffer, HEADER_BLOCK_SIZE);
    buffer->index = 0;
  }
}
void putu16(u_int16_t in, struct simple_buffer *buffer) {
  putu8(in >> 8, buffer);
  putu8(in, buffer);
}
void putlong(u_int32_t in, struct simple_buffer *buffer) {
  putu8(in >> 24, buffer);
  putu8(in >> 16, buffer);
  putu8(in >> 8, buffer);
  putu8(in, buffer);
}

void tiff_write_header(u_int8_t *rawbuffer, struct tiff_header header) {
  struct simple_buffer header_buffer;
  header_buffer.index = 0;

  uint32_t start_image = 8 * header.ImageLength + 256;
  uint32_t startsector = start_image / HEADER_BLOCK_SIZE;
  if (start_image % HEADER_BLOCK_SIZE > 0) {
    startsector++;
  }
  start_image = startsector * HEADER_BLOCK_SIZE;

  header_buffer.length = HEADER_BLOCK_SIZE;
  header_buffer.buffer = rawbuffer;
  putu8('M', &header_buffer);
  putu8('M', &header_buffer);
  putu16(42, &header_buffer);
  // start of idf location
  putlong(8, &header_buffer);
  // start of idf
  putu16(12, &header_buffer);
  // start of fields
  putu16(256, &header_buffer);
  putu16(TIFF_LONG, &header_buffer);
  putlong(1, &header_buffer);
  putlong(header.ImageWidth, &header_buffer);
  putu16(257, &header_buffer);
  putu16(TIFF_LONG, &header_buffer);
  putlong(1, &header_buffer);
  putlong(header.ImageLength, &header_buffer);
  // bits per pixel on adress 200
  putu16(258, &header_buffer);
  putu16(TIFF_SHORT, &header_buffer);
  putlong(3, &header_buffer);
  putlong(200, &header_buffer); // bits per sample
  putu16(259, &header_buffer);
  putu16(TIFF_SHORT, &header_buffer);
  putlong(1, &header_buffer);
  putu16(header.Compression, &header_buffer);
  putu16(0, &header_buffer);
  putu16(262, &header_buffer);
  putu16(TIFF_SHORT, &header_buffer);
  putlong(1, &header_buffer);
  putu16(header.PhotometricInterpretation, &header_buffer);
  putu16(0, &header_buffer);
  // ofssets
  putu16(273, &header_buffer);
  putu16(TIFF_LONG, &header_buffer);
  putlong(header.ImageLength, &header_buffer);
  putlong(256, &header_buffer); // StipOffsets
  putu16(277, &header_buffer);
  putu16(TIFF_SHORT, &header_buffer);
  putlong(1, &header_buffer);
  putu16(header.SamplesPerPixel, &header_buffer);
  putu16(0, &header_buffer);
  putu16(278, &header_buffer);
  putu16(TIFF_SHORT, &header_buffer);
  putlong(1, &header_buffer);
  putu16(header.RowsPerStrip, &header_buffer);
  putu16(0, &header_buffer);
  putu16(279, &header_buffer);
  putu16(TIFF_LONG, &header_buffer);
  putlong(header.ImageLength, &header_buffer);
  putlong(256 + header.ImageLength * 4, &header_buffer); // StripByteCounts

  putu16(282, &header_buffer);
  putu16(TIFF_RATIONAL, &header_buffer);
  putlong(1, &header_buffer);
  putlong(220, &header_buffer);
  putu16(283, &header_buffer);
  putu16(TIFF_RATIONAL, &header_buffer);
  putlong(1, &header_buffer);
  putlong(228, &header_buffer);
  putu16(296, &header_buffer);
  putu16(TIFF_SHORT, &header_buffer);
  putlong(1, &header_buffer);
  putu16(header.ResolutionUnit, &header_buffer);
  putu16(0, &header_buffer);
  putlong(0, &header_buffer);

  header_buffer.index = 200;
  putu16(header.BitsPerSample, &header_buffer);
  putu16(header.BitsPerSample, &header_buffer);
  putu16(header.BitsPerSample, &header_buffer);

  header_buffer.index = 220;
  putlong(header.XResolution, &header_buffer);
  putlong(1, &header_buffer);
  putlong(header.YResolution, &header_buffer);
  putlong(1, &header_buffer);

  header_buffer.index = 256;
  for (long i = 0; i < header.ImageLength; i++) {
    putlong(start_image + i * header.blockSize, &header_buffer);
  }
  for (long i = 0; i < header.ImageLength; i++) {
    putlong(6 * header.ImageWidth, &header_buffer);
  }
  // write zeros to fill up sector to write the buffer to storage
  while (header_buffer.index) {
    putu8(0, &header_buffer);
  }
}
bool sd_init() {
  if (!sd.begin(SD_CONFIG)) {
    printf("SDCARD connection error\n");
    return false;
  }
  return true;
}
bool tiff_write_line(u_int8_t *buffer, u_int32_t length) {
  if (file.write(buffer, length) == length) {
    return true;
  } else {
    return false;
  };
}
bool tiff_close() { return file.close(); }
bool tiff_create(long image_length, long image_width) {
  uint16_t imgnum = 0;
  char filename[] = "image00.tiff";
  while (sd.exists(filename)) {
    imgnum++;
    filename[5] = imgnum / 10 + '0';
    filename[6] = imgnum % 10 + '0';
  }
  if (file.open(filename, O_RDWR | O_CREAT | O_TRUNC)) {
    printf("Opnening image file succes\n");
  } else {
    return false;
  }

  struct tiff_header tiff_file;
  tiff_file.ImageWidth = image_width;
  tiff_file.ImageLength = image_length;
  tiff_file.BitsPerSample = 16;
  tiff_file.Compression = 1;
  tiff_file.PhotometricInterpretation = 2; // 34892;//2;
  tiff_file.RowsPerStrip = 1;
  tiff_file.SamplesPerPixel = 3;
  tiff_file.XResolution = 300;
  tiff_file.YResolution = 300;
  tiff_file.ResolutionUnit = 2;
  tiff_file.blockSize = image_width * 6;

  uint32_t start_image = 8 * tiff_file.ImageLength + 256;
  uint32_t startsector = start_image / HEADER_BLOCK_SIZE;
  if (start_image % HEADER_BLOCK_SIZE > 0) {
    startsector++;
  }
  start_image = startsector * HEADER_BLOCK_SIZE;

  uint32_t file_size =
      tiff_file.blockSize * tiff_file.ImageLength + start_image;
  file.preAllocate(file_size);
  uint32_t currentsector = file.firstSector();
  sd.card()->writeStart(currentsector, file_size / 512);
  uint8_t *header_buffer = (uint8_t *)malloc(HEADER_BLOCK_SIZE);
  if (header_buffer) {
    tiff_write_header(header_buffer, tiff_file);
    free(header_buffer);
    return true;
  } else {
    return false;
  }
}