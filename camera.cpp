
#include <stdio.h>

#include "ak8419.h"
#include "camera.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "stepper.h"
#include "tiff.h"
#include <cstdlib>
#include <cstring>
#include "jpeg.h"
#define STEPPER_ACCEL 200000.0
#define STEPPER_MOVE_SPEED 40000.0
#define JPEG_QUALITY 90
#define FOCUS_SEGMENTS 8
int lines_remaining = 0;
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

uint8_t preview_buffer[16 * (3 * PIXELS_PREVIEW)] = {};

uint8_t msg_focus[MSG_HEADER_LEN + 8 * 4] = {};
uint8_t msg_histogram[MSG_HEADER_LEN + HISTOGRAM_LENGTH] = {};
int preview_frame = 0;
void auto_offset()
{
  set_offset(128, 128, 128);
  set_gain(0, 0, 0);
  // data_ready=false;
  avg_sum_dark_red = 0;
  avg_sum_dark_green = 0;
  avg_sum_dark_blue = 0;
  ccd_start_capture();

  // Wait for data to be ready
  for (int i = 0; i < 8; ++i)
  {
    while (!data_ready)
    {
      sleep_ms(1);
    }
    printf("line %d ready\n", i);
    data_ready = false;
    write_ready = true;
  }

  ccd_stop_capture();
  int dark_pixel_start = 336;
  int dark_pixel_end = 376;
  int dark_pixel_num = dark_pixel_end - dark_pixel_start;
  for (int j = dark_pixel_start; j < dark_pixel_end; j++)
  {
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
  avg_sum_dark_red = avg_sum_dark_red / dark_pixel_num;
  avg_sum_dark_green = avg_sum_dark_green / dark_pixel_num;
  avg_sum_dark_blue = avg_sum_dark_blue / dark_pixel_num;

  set_offset(128 - (avg_sum_dark_red) / 56, 128 - (avg_sum_dark_green) / 56,
             128 - (avg_sum_dark_blue) / 56);
}
void auto_wb() {}
void run_stepper(int frequency)
{
}
void return_stepper(int frequency)
{
}
void camera_task()
{
  sleep_ms(1000);
  printf("Second core started\n");
  // task that controlls camera and takes commands from other core
  // if(sd_init()){
  //     printf("SD card mounted");
  // }
  ccd_init();
  stepper_init();
  init_gamma_table();
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
  int pixels_per_line = 5400;
  //int real_pixels = 7168;
  // int real_pixels = 4864;//7424;
  //int start_pixels = 272;
double stepper_speed;
  // uint32_t ratio = 8 * 51 * 16;
  // uint32_t steps_total = 3 * 150 * ratio / 18;
  // uint32_t lines_total = 4000;
  // steps_per_line = steps_total / lines_total;

  while (1)
  {
    switch (camera_state)
    {

    case COMMAND_PREVIEW:
      if (lines_remaining > 0)
      {
        if (data_ready)
        {
          add_line_to_preview();
          if (preview_frame % 8 == 0)
          {
            uint8_t *preview_block = preview_buffer + ((preview_frame - 8) * 3 * PIXELS_PREVIEW);
            // printf("writing block\n");
            int extra_lines = 0;
            if (preview_frame >= 16)
            {
              preview_frame = 0;
            }
            for (int x = 0; x < PIXELS_PREVIEW; x += 8)
            {
              process_block(preview_block, PIXELS_PREVIEW, x);
              if (data_ready)
              {
                add_line_to_preview();
                extra_lines++;
              }
            }
            // printf("extra lines captured%d\n",extra_lines);
          }
        }
      }
      else
      {
        camera_state = COMMAND_IDLE;
        lines_remaining = 0;
        ccd_stop_capture();
        jo_write_jpg_end();
        move_to(0, STEPPER_MOVE_SPEED, STEPPER_ACCEL);
        printf("Klaar met preview");
      }
      break;
    case COMMAND_CAPTURE:
      if (lines_remaining > 0)
      {
        if (data_ready)
        {
          tiff_write_line(pixel_buffers[buffer_num ^ 1] + (CCD_PIXEL_DARK_START * 6),
                          6 * CCD_PIXEL_CAPTURE_NUM);
          data_ready = false;
          write_ready = true;
          lines_remaining--;
          // sleep_ms(30); 
        }
      }
      else
      {

        camera_state = COMMAND_IDLE;
        lines_remaining = 0;
        tiff_close();
        ccd_stop_capture();
        move_to(0, STEPPER_MOVE_SPEED, STEPPER_ACCEL);
        printf("klaar met schrijven");
      }
      break;

    case COMMAND_FOCUS:
      if (data_ready)
      {
        pixelbuffer = ((uint16_t *)pixel_buffers[buffer_num ^ 1]);
        int32_t diff;
        int seglen = CCD_PIXEL_PREVIEW_NUM/FOCUS_SEGMENTS;
        uint64_t focus = 0;
        for (int j = 0; j < FOCUS_SEGMENTS; j++)
        {
          focus = 0;
          for (int i = j * seglen+CCD_PIXEL_LIGHT_START; i < (seglen * (j + 1) - 2); i++)
          {
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
        web_focus.buffer = msg_focus;
        web_focus.length = sizeof(msg_focus);
        queue_add_blocking(&dataqueue, &web_focus);

        // multicore_fifo_push_blocking((uint32_t)msg_focus);

        // sleep_ms(30);
        write_ready = true;
        data_ready = false;
      }
      break;
    case COMMAND_EXPOSE:
      if (data_ready)
      {
        pixelbuffer = ((uint16_t *)pixel_buffers[buffer_num ^ 1]);

        for (int i = 0; i < HISTOGRAM_LENGTH / 2; i++)
        {
          histogram_data[i] = 0;
        }
        for (int i = 0; i < pixels_per_line; i++)
        {
          histogram_data[(pixel_buffers[buffer_num ^ 1][i * 6] >> 1)]++;
          histogram_data[(pixel_buffers[buffer_num ^ 1][i * 6 + 2] >> 1) +
                         128]++;
          histogram_data[(pixel_buffers[buffer_num ^ 1][i * 6 + 4] >> 1) +
                         256]++;
        }
        struct web_data web_historgram;
        web_historgram.length = sizeof(msg_histogram);
        web_historgram.buffer = msg_histogram;
        queue_add_blocking(&dataqueue, &web_historgram);

        // multicore_fifo_push_blocking((uint32_t)msg_histogram);
        write_ready = true;

        data_ready = false;
      }
      break;

    default:
      break;
    }
    // if command is recieved from other core
    if (!queue_is_empty(&commandqueue))
    {
      // memcpy(&command, (void *)multicore_fifo_pop_blocking(), 16);
      queue_remove_blocking(&commandqueue, &command);

      // set_gain(command.gain);
      switch (command.command)
      {
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
        lines_remaining = command.lines;
        set_exposure_time(command.exp_time);
        // auto_offset();
        set_gain(command.gain, command.gain, command.gain);
        if (tiff_create(command.lines, CCD_PIXEL_CAPTURE_NUM))
        {
          stepper_speed = (((double)command.steps_line) / 100.0 * ((double)command.exp_time));

          move_to(-accel_steps(stepper_speed, STEPPER_ACCEL), STEPPER_MOVE_SPEED, STEPPER_ACCEL);
          while (get_stepper_state() != STOP)
          {
            tight_loop_contents();
          }

          move_to((((int)command.steps_line) *((int) command.lines))/100+accel_steps(stepper_speed, STEPPER_ACCEL), stepper_speed, STEPPER_ACCEL);
          while (get_stepper_state() != CONSTANT)
          {
            tight_loop_contents();
          }
          ccd_start_capture();

          camera_state = COMMAND_CAPTURE;
        }
        else
        {
          printf("Error creating tiff file aborting");
          camera_state = COMMAND_IDLE;
        }

        // auto_offset();
        // run_stepper(steps_per_line * command.exp_time);

        // data_ready=true;//for debugginh

        break;
      case COMMAND_PREVIEW:
        preview_frame = 0;
        lines_remaining = command.lines / 8;
        set_exposure_time(command.exp_time);
        // auto_offset();
        set_gain(command.gain, command.gain, command.gain);
        camera_state = COMMAND_PREVIEW;
        jo_write_jpg(PIXELS_PREVIEW, command.lines / 8, JPEG_QUALITY);
        stepper_speed = 8.0 * (((double)command.steps_line / 100.0 * (double)command.exp_time));

        move_to(-accel_steps(stepper_speed, STEPPER_ACCEL), stepper_speed, STEPPER_ACCEL);
        while (get_stepper_state() != STOP)
        {
          tight_loop_contents();
        }

        move_to((((int)command.steps_line) *(int) command.lines)/100+accel_steps(stepper_speed, STEPPER_ACCEL), stepper_speed, STEPPER_ACCEL);
        while (get_stepper_state() != CONSTANT)
        {
          tight_loop_contents();
        }
        ccd_start_capture();
        printf("starting preview.");
        break;
      case COMMAND_EXPOSE:
        lines_remaining = command.lines;
        set_exposure_time(command.exp_time);
        // auto_offset();
        set_gain(command.gain, command.gain, command.gain);
        camera_state = COMMAND_EXPOSE;
        ccd_start_capture();

        break;
      case COMMAND_FOCUS:
        lines_remaining = command.lines;
        set_exposure_time(command.exp_time);

        set_gain(command.gain, command.gain, command.gain);
        camera_state = COMMAND_FOCUS;
        ccd_start_capture();
        // auto_offset();
        //  Throw away the first 8 lines
        for (int i = 0; i < 8; ++i)
        {
          while (!data_ready)
          {
            sleep_ms(1);
          }
          data_ready = false;
          write_ready = true;
        }
        break;
      case COMMAND_MOVE:
      printf("moving\n");
        move_to((((int)command.steps_line) *(int) command.lines)/100  , STEPPER_MOVE_SPEED, STEPPER_ACCEL);
        camera_state = COMMAND_IDLE;
        break;
      default:
        break;
      }
    }
  }
}

void add_line_to_preview()
{
  if (preview_frame >= 16)
  {
    preview_frame = 0;
  }
  // shift lines per color to match in preview
  // int preview_frame_r = ((preview_frame) % 16) * PIXELS_PREVIEW;
  // int preview_frame_g = ((preview_frame + 2) % 16) * PIXELS_PREVIEW;
  // int preview_frame_b = ((preview_frame + 4) % 16) * PIXELS_PREVIEW;
  int preview_frame_r = ((preview_frame+2) % 16) * PIXELS_PREVIEW;
  int preview_frame_g = ((preview_frame + 1) % 16) * PIXELS_PREVIEW;
  int preview_frame_b = ((preview_frame ) % 16) * PIXELS_PREVIEW;
  int read_buffer = buffer_num ^ 1;
  int j=CCD_PIXEL_LIGHT_START/8;
  for (int i = 0; i < PIXELS_PREVIEW; i++)
  {
    // only extract the MSB and copy it to the preview buffer
    preview_buffer[(preview_frame_r + i) * 3] = pixel_buffers[read_buffer][(j*8)*6];
    preview_buffer[(preview_frame_g + i) * 3 + 1] = pixel_buffers[read_buffer][(j*8)*6+ 2];
    preview_buffer[(preview_frame_b + i) * 3 + 2] = pixel_buffers[read_buffer][(j*8)*6 + 4];
    j++;//Increase by 6 * 8 bytes 
  }
  preview_frame++;

  write_ready = true;
  data_ready = false;
  lines_remaining--;
  // printf("Lines remaing:%d\n",lines_remaining);
}
