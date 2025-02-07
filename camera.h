#include <cstdint>
#define COMMAND_CAPTURE 0
#define COMMAND_FOCUS 1
#define COMMAND_PREVIEW 2
#define COMMAND_ABORT 3
#define COMMAND_EXPOSE 4
#define COMMAND_IDLE 5
#define COMMAND_MOVE 6
#define MSG_HEADER_LEN 4
#define HISTOGRAM_LENGTH 3 * 2 * 128
#define MSG_PREVIEW 1;
#define MSG_FOCUS 2;
#define MSG_HISTOGRAM 3;
#define MSG_TEXT 4;
#define MSG_JPG 5

struct __attribute__((packed)) web_command {
  uint8_t command;     // command
  uint16_t exp_time;   // integration time fraction of second
  uint8_t resolution;  // resolution setting
  uint8_t gain;        // uniform gain
  uint32_t lines;      // amount of lines to capture
  uint16_t steps_line; // amount of steps per line
  uint16_t crop_start; // crop for preview start
  uint16_t crop_end;   // crop for preview end
  uint8_t preview_avg; // amount of pixels to avarage for preview
};
struct web_data{
  uint32_t length;
  uint8_t* buffer;
};
void camera_task();

void add_line_to_preview();
