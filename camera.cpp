
#include <stdio.h>

#include "ak8419.h"
#include "camera.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

#include "tiff.h"
#include <cstdlib>
#include <cstring>
#define MSG_HEADER_LEN 4
#define HISTOGRAM_LENGTH 3 * 2 * 128
#define MSG_PREVIEW 1;
#define MSG_FOCUS 2;
#define MSG_HISTOGRAM 3;
#define MSG_TEXT 4;

uint32_t lines_remaining = 0;
uint8_t gain_red = 0, gain_green = 0, gain_blue = 0;
extern int buffer_num;
extern bool data_ready;
extern uint8_t *pixel_buffers[2];
extern bool write_ready;
extern queue_t commandqueue;
extern queue_t dataqueue;
uint32_t avg_sum_dark_red, avg_sum_dark_green, avg_sum_dark_blue;
uint32_t avg_sum_wb_red, avg_sum_wb_green, avg_sum_wb_blue;
uint32_t steps_per_line;
uint slice_num, channel;
uint64_t step_time_start;
int8_t camera_state = -1;

uint8_t msg_preview[MSG_HEADER_LEN + 3 * 5400 / 8] = {};
uint8_t msg_focus[MSG_HEADER_LEN + 8 * 4] = {};
uint8_t msg_histogram[MSG_HEADER_LEN + HISTOGRAM_LENGTH] = {};

void auto_offset() {
  set_offset(128, 128, 128);
  set_gain(0, 0, 0);
  // data_ready=false;
  avg_sum_dark_red = 0;
  avg_sum_dark_green = 0;
  avg_sum_dark_blue = 0;
  avg_sum_wb_red = 0;
  avg_sum_wb_green = 0;
  avg_sum_wb_blue = 0;
  ccd_start_capture();

  // Wait for data to be ready
  for (int i = 0; i < 4; ++i) {
    while (!data_ready) {
      sleep_ms(10);
    }
    printf("line %d ready\n", i);
    data_ready = false;
  }

  ccd_stop_capture();
  printf("\n value at 128: %u\n",
         (pixel_buffers[buffer_num ^ 1][128 * 6] << 8) +
             pixel_buffers[buffer_num ^ 1][128 * 6 + 1]);
  for (int j = 127; j < 256; j++) {
    // avg_sum0= avg_sum0 + ((uint16_t*) (pixel_buffers[buffer_num^1]))[j*3];
    avg_sum_dark_red = avg_sum_dark_red +
                       (pixel_buffers[buffer_num ^ 1][j * 6] << 8) +
                       pixel_buffers[buffer_num ^ 1][j * 6 + 1];

    avg_sum_dark_green = avg_sum_dark_green +
                         (pixel_buffers[buffer_num ^ 1][j * 6 + 2] << 8) +
                         pixel_buffers[buffer_num ^ 1][j * 6 + 3];
    avg_sum_dark_blue = avg_sum_dark_blue +
                        (pixel_buffers[buffer_num ^ 1][j * 6 + 4] << 8) +
                        pixel_buffers[buffer_num ^ 1][j * 6 + 5];
  }
  avg_sum_dark_red = avg_sum_dark_red >> 7;
  avg_sum_dark_green = avg_sum_dark_green >> 7;
  avg_sum_dark_blue = avg_sum_dark_blue >> 7;
  printf("avg R: %u, G: %u,B: %u \n", (avg_sum_dark_red), (avg_sum_dark_green),
         (avg_sum_dark_blue));

  for (int j = 511; j < 4608; j++) {
    avg_sum_wb_red = avg_sum_wb_red +
                     (pixel_buffers[buffer_num ^ 1][j * 6] << 8) +
                     pixel_buffers[buffer_num ^ 1][j * 6 + 1];
    avg_sum_wb_green = avg_sum_wb_green +
                       (pixel_buffers[buffer_num ^ 1][j * 6 + 2] << 8) +
                       pixel_buffers[buffer_num ^ 1][j * 6 + 3];
    avg_sum_wb_blue = avg_sum_wb_blue +
                      (pixel_buffers[buffer_num ^ 1][j * 6 + 4] << 8) +
                      pixel_buffers[buffer_num ^ 1][j * 6 + 5];
  }
  printf("avg R: %u, G: %u,B: %u \n", avg_sum_wb_red >> 12,
         avg_sum_wb_green >> 12, avg_sum_wb_blue >> 12);

  // check if red channel is brightest?
  double gain_green_correction =
      ((double)(avg_sum_wb_red >> 12) - avg_sum_dark_red) /
      ((double)(avg_sum_wb_green >> 12) - avg_sum_dark_green);
  double gain_blue_correction =
      ((double)(avg_sum_wb_red >> 12) - avg_sum_dark_red) /
      ((double)(avg_sum_wb_blue >> 12) - avg_sum_dark_blue);
  gain_green = 9 * (-122008 + 121875 * gain_green_correction) /
               (15251 + 12500 * gain_green_correction);
  gain_blue = 9 * (-122008 + 121875 * gain_blue_correction) /
              (15251 + 12500 * gain_blue_correction);
  gain_red = 0;

  printf("gain setting R: %u, G: %u,B: %u \n", gain_red, gain_green, gain_blue);
  set_gain(gain_red, gain_green, gain_blue);
  set_offset(128 - (avg_sum_dark_red) / 56, 128 - (avg_sum_dark_green) / 56,
             128 - (avg_sum_dark_blue) / 56);
}
void auto_wb() {}
void run_stepper(int frequency) {
  gpio_put(ENABLE_PIN, 0);

  gpio_put(DIR_PIN, 1);
  pwm_set_wrap(slice_num, 1000000 / (frequency));
  pwm_set_chan_level(slice_num, channel, 500000 / (frequency));
  pwm_set_enabled(slice_num, true);
  step_time_start = time_us_64();
}
void return_stepper(int frequency) {
  gpio_put(DIR_PIN, 0);
#define return_freq 250
  uint64_t step_time_end = time_us_64();

  pwm_set_wrap(slice_num, 1000000 / (return_freq * steps_per_line));
  pwm_set_chan_level(slice_num, channel,
                     500000 / (return_freq * steps_per_line));

  sleep_us((step_time_end - step_time_start) * frequency /
           (return_freq * steps_per_line));
  pwm_set_enabled(slice_num, false);
  gpio_put(ENABLE_PIN, 1);
}
void camera_task() {
  sleep_ms(1000);
  printf("Second core started\n");
  // task that controlls camera and takes commands from other core
  // if(sd_init()){
  //     printf("SD card mounted");
  // }
  ccd_init();
  uint8_t *preview_buffer = msg_preview + MSG_HEADER_LEN;
  uint16_t *msg_preview_type = (uint16_t *)msg_preview;
  uint16_t *msg_preview_len = (uint16_t *)(msg_preview + 2);
  *msg_preview_type = MSG_PREVIEW;
  *msg_preview_len = sizeof(msg_preview);

  float *focus_data = (float *)(msg_focus + MSG_HEADER_LEN);
  uint16_t *msg_focus_type = (uint16_t *)msg_focus;
  uint16_t *msg_focus_len = (uint16_t *)(msg_focus + 2);
  *msg_focus_type = MSG_FOCUS;
  *msg_focus_len = sizeof(msg_focus);

  uint16_t *histogram_data = (uint16_t *)(msg_histogram + MSG_HEADER_LEN);
  uint16_t *msg_histogram_type = (uint16_t *)msg_histogram;
  uint16_t *msg_histogram_len = (uint16_t *)(msg_histogram + 2);
  *msg_histogram_type = MSG_HISTOGRAM;
  *msg_histogram_len = sizeof(msg_histogram);

  uint16_t *pixelbuffer;
  struct web_command command;
  int bin_step = 0;
  int pixels_per_line = 5400;
  int real_pixels = 4864;
  int start_pixels = 272;

  uint32_t ratio = 8 * 51 * 16;
  uint32_t steps_total = 3*150 * ratio / 18;
  uint32_t lines_total = 4000;
  steps_per_line = steps_total / lines_total;

  gpio_init(DIR_PIN);
  gpio_set_dir(DIR_PIN, GPIO_OUT);
  gpio_put(DIR_PIN, 1);
  gpio_init(ENABLE_PIN);
  gpio_set_dir(ENABLE_PIN, GPIO_OUT);
  gpio_put(ENABLE_PIN, 1);
  gpio_set_function(STEP_PIN, GPIO_FUNC_PWM);
  slice_num = pwm_gpio_to_slice_num(STEP_PIN);
  channel = pwm_gpio_to_channel(STEP_PIN);

  pwm_set_clkdiv(slice_num, CLOCK_SPEED); // PWM clock should now be running at 1MHz
  // pwm_set_wrap(slice_num, 1000000/(steps_per_line*));  // Set period of 1024
  // cycles (0 to 1023 inclusive) pwm_set_chan_level(slice_num,)
  // pwm_set_enabled(slice_num, true);

  // auto_offset();
  //  pixel_buffers[0] = (uint8_t*)malloc(pixels_per_line*6);
  //  pixel_buffers[1] = (uint8_t*)malloc(pixels_per_line*6);
  //  uint16_t *pixel_buffers_16 = (uint16_t*) pixel_buffers[buffer_num];
  //   for(int i=0;i<pixels_per_line;i++){
  //       pixel_buffers_16[i*3] = i*200;
  //       pixel_buffers_16[i*3+1] = (i+100)*200;
  //       pixel_buffers_16[i*3+2] = (i+200)*200;
  //   }
  while (1) {
    switch (camera_state) {

    case COMMAND_PREVIEW:
      if (lines_remaining > 0) {
        if (data_ready) {
          printf("Lines remaining:%d\n", lines_remaining);
          // only send out 1/8 of the lines and 1/8 of the pixels per line
          // if (++bin_step == 8)
          //{
          for (int i = 0; i < pixels_per_line / 8; i++) {
            // only extract the MSB and copy it to the preview buffer
            preview_buffer[3 * i] = pixel_buffers[buffer_num ^ 1][(i * 8) * 6];
            preview_buffer[3 * i + 1] =
                pixel_buffers[buffer_num ^ 1][(i * 8) * 6 + 2];
            preview_buffer[3 * i + 2] =
                pixel_buffers[buffer_num ^ 1][(i * 8) * 6 + 4];
          }
          printf("messagelen: %d,type:%d\n", *msg_preview_len,
                 *msg_preview_type);
          *msg_preview_type = MSG_PREVIEW;
          *msg_preview_len = sizeof(msg_preview);
          struct web_data preview_data;
          preview_data.length= sizeof(msg_preview);
          preview_data.buffer=msg_preview;
          queue_add_blocking(&dataqueue,&preview_data);
          printf("preview pushed on queue");
          // multicore_fifo_push_blocking((uint32_t)msg_preview);
          bin_step = 0;
          // printf("sending line to other core");
          //}
          // sleep_ms(30);
          write_ready = true;
          data_ready = false;
          // lines_remaining=lines_remaining-8;
          lines_remaining--;
          lines_remaining--;
          lines_remaining--;
          lines_remaining--;
          lines_remaining--;
          lines_remaining--;
          lines_remaining--;
          lines_remaining--;
        }
      } else {
        camera_state = COMMAND_IDLE;
        lines_remaining = 0;
        ccd_stop_capture();
        return_stepper(steps_per_line * command.exp_time * 8);
        // free(pixel_buffers[0]);
        // free(pixel_buffers[1]);
        printf("Klaar met preview");
        // free(preview_buffer);
      }
      break;
    case COMMAND_CAPTURE:
      if (lines_remaining > 0) {
        if (data_ready) {
          tiff_write_line(pixel_buffers[buffer_num ^ 1] + (start_pixels * 6),
                          6 * real_pixels);
          data_ready = false;
          write_ready = true;
          lines_remaining--;
          // sleep_ms(30); 
        }
      } else {
        tiff_close();
        camera_state = COMMAND_IDLE;
        lines_remaining = 0;
        ccd_stop_capture();
        return_stepper(steps_per_line * command.exp_time);
        printf("klaar met schrijven");

        // free(pixel_buffers[0]);
        // free(pixel_buffers[1]);
        // free(preview_buffer);
      }
      break;

    case COMMAND_FOCUS:
      if (data_ready) {
        pixelbuffer = ((uint16_t *)pixel_buffers[buffer_num ^ 1]);
        int32_t diff;
        int seglen = 625;
        int segnum = 8;
        uint64_t focus = 0;
        for (int j = 0; j < segnum; j++) {
          focus = 0;
          for (int i = j * seglen; i < (seglen * (j + 1) - 2); i++) {
            diff = (int16_t)(__builtin_bswap16(pixelbuffer[i * 3]) >> 1) -
                   (int16_t)(__builtin_bswap16(pixelbuffer[(i + 2) * 3]) >> 1);
            focus += (diff * diff);
            diff = (int16_t)(__builtin_bswap16(pixelbuffer[i * 3 + 1]) >> 1) -
                   (int16_t)(__builtin_bswap16(pixelbuffer[(i + 2) * 3 + 1]) >> 1);
            focus += (diff * diff);
            diff = (int16_t)(__builtin_bswap16(pixelbuffer[i * 3 + 2]) >> 1) -
                   (int16_t)(__builtin_bswap16(pixelbuffer[(i + 2) * 3 + 2]) >> 1);
            focus += (diff * diff);
          }
          focus_data[j] = (float)focus;
        }
        struct web_data web_focus;
        web_focus.buffer=msg_focus;
        web_focus.length=sizeof(msg_focus);
        queue_add_blocking(&dataqueue,&web_focus);

        // multicore_fifo_push_blocking((uint32_t)msg_focus);

        // sleep_ms(30);
        write_ready = true;
        data_ready = false;
      }
      break;
    case COMMAND_EXPOSE:
      if (data_ready) {
        pixelbuffer = ((uint16_t *)pixel_buffers[buffer_num ^ 1]);

        for (int i = 0; i < HISTOGRAM_LENGTH / 2; i++) {
          histogram_data[i] = 0;
        }
        for (int i = 0; i < pixels_per_line; i++) {
          histogram_data[(pixel_buffers[buffer_num ^ 1][i * 6] >> 1)]++;
          histogram_data[(pixel_buffers[buffer_num ^ 1][i * 6 + 2] >> 1) +
                         128]++;
          histogram_data[(pixel_buffers[buffer_num ^ 1][i * 6 + 4] >> 1) +
                         256]++;
        }
        struct web_data web_historgram;
        web_historgram.length=sizeof(msg_histogram);
        web_historgram.buffer=msg_histogram;
                  queue_add_blocking(&dataqueue,&web_historgram);

        // multicore_fifo_push_blocking((uint32_t)msg_histogram);
        write_ready = true;

        data_ready = false;
      }
      break;

    default:
      break;
    }
    // if command is recieved from other core
    if (!queue_is_empty(&commandqueue)) {
      // memcpy(&command, (void *)multicore_fifo_pop_blocking(), 16);
                queue_remove_blocking(&commandqueue,&command);

      lines_remaining = command.lines;
      set_exposure_time(command.exp_time);
      // set_gain(command.gain);
      switch (command.command) {
      case COMMAND_ABORT:
        // if(camera_state==COMMAND_CAPTURE)
        // {
        //     tiff_close();
        //     camera_state = COMMAND_IDLE;
        // }
        camera_state = COMMAND_IDLE;
        lines_remaining = 0;
        ccd_stop_capture();

        break;
      case COMMAND_CAPTURE:
        // set_exposure_time(command.exp_time);
        // set_gain(command.gain);

        tiff_create(command.lines, real_pixels);
        // auto_offset();
        run_stepper(steps_per_line * command.exp_time);

        ccd_start_capture();

        camera_state = COMMAND_CAPTURE;
        // data_ready=true;//for debugginh

        break;
      case COMMAND_PREVIEW:

        // data_ready=true;//for debugginh
        camera_state = COMMAND_PREVIEW;
        // auto_offset();
        //  set_gain(command.gain,command.gain,command.gain);
        run_stepper(8 * steps_per_line * command.exp_time);

        ccd_start_capture();
        printf("starting preview.");
        break;
      case COMMAND_EXPOSE:
        camera_state = COMMAND_EXPOSE;
        ccd_start_capture();

        break;
      case COMMAND_FOCUS:
        camera_state = COMMAND_FOCUS;
        ccd_start_capture();

        break;
      default:
        break;
      }
    }
  }
}