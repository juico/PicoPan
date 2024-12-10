
#define COMMAND_CAPTURE 0
#define COMMAND_FOCUS 1
#define COMMAND_PREVIEW 2
#define COMMAND_ABORT 3
#define COMMAND_EXPOSE 4
#define COMMAND_IDLE 5
struct __attribute__((packed)) web_command {
  u_int8_t command;     // command
  u_int16_t exp_time;   // integration time in micro seconds
  u_int8_t resolution;  // resolution setting
  u_int8_t gain;        // uniform gain
  u_int32_t lines;      // amount of lines to capture
  u_int16_t steps_line; // amount of steps per line
  u_int16_t crop_start; // crop for preview start
  u_int16_t crop_end;   // crop for preview end
  u_int8_t preview_avg; // amount of pixels to avarage for preview
};
void camera_task();