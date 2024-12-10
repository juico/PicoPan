

#include "ak8419.h"
#include "ak8419.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "step.pio.h"
#include <iostream>
#include <stdio.h>

uint offset_trigger;
uint offset_sync;
uint offset_data_in;
uint pixel_dma_chan;
PIO pio_ak8419 = pio0;

uint sm_data_in = 1;
uint sm_trigger = 2;
uint sm_sync = 3;

uint16_t int_time = 100;
// uint32_t ratio = 8 * 51 * 16;
// uint32_t steps_total = 150 * ratio / 18;
// uint32_t lines_total = 4000;
// uint32_t steps_per_line = steps_total / lines_total;
uint8_t reg_table[87] = {
    0b00000000, 0b00000010, 0b00000000, 0b00000001,
    0b00000000, 0b00000000, 0b10000000, 0b10000000, // 00h-07h
    0b10000000, 0b00000000, 0b00000000, 0b00000000,
    0b01101000, 0b00000000, 0b01001111, 0b00000000, // 08h-0Fh
    0b01001111, 0b00000000, 0b10101111, 0b00000100,
    0b00000100, 0b10101111, 0b00000100, 0b10101111, // 10h-17h
    0b10101111, 0b00000100, 0b11110100, 0b10101111,
    0b11111111, 0b11111111, 0b10101111, 0b00000100, // 18h-1Fh
    0b00000100, 0b10101111, 0b00000000, 0b00101000,
    0b00000000, 0b00111111, 0b00101100, 0b00100100, // 20h-27h
    0b00111111, 0b00110000, 0b00110000, 0b00111111,
    0b00110000, 0b00111111, 0b00010011, 0b00010000, // 28h-2Fh
    0b00011000, 0b00000000, 0b00000010, 0b00000000,
    0b00111111, 0b00000000, 0b00000010, 0b00000000, // 30h-37h
    0b00000000, 0b00000000, 0b00000010, 0b00000000,
    0b00000000, 0b00000000, 0b00000000, 0b00000000, // 38h-3Fh
    0b00000000, 0b00000010, 0b00000000, 0b00011110,
    0b00001010, 0b00011110, 0b00000000, 0b01010010, // 40h-47h
    0b00000001, 0b00000000, 0b00000000, 0b00000000,
    0b00000000, 0b00000000, 0b00000000, 0b00000000, // 48h-4Fh
    0b00000000, 0b00000000, 0b00000000, 0b00000000,
    0b00000000, 0b00000000, 0b00000000 // 50h-56h
};

uint8_t pixel_buffer_1[CCD_BYTES] = {};
uint8_t pixel_buffer_2[CCD_BYTES] = {};
uint8_t *pixel_buffers[2] = {pixel_buffer_1, pixel_buffer_2};

int buffer_num = 0;
bool data_ready = false;
bool registercheck = false;
bool write_ready = true;
u_int32_t lines_aqcuired = 0;
u_int32_t lines_written = 0;

struct ak8419_config *ccd_config = (ak8419_config *)reg_table;

void write_register(uint8_t adress, uint8_t data)
{
  gpio_init(SDATA_PIN);
  gpio_init(SDCLK_PIN);
  // start serial communication by setting SDENB low
  gpio_put(SDENDB_PIN, 0);
  gpio_set_dir(SDATA_PIN, GPIO_OUT);
  gpio_set_dir(SDCLK_PIN, GPIO_OUT);
  // first set output and then cycle clock
  // set first bit low for read
  gpio_put(SDATA_PIN, 0);
  sleep_us(5);
  gpio_put(SDCLK_PIN, 1);
  sleep_us(5);
  gpio_put(SDCLK_PIN, 0);
  sleep_us(10);

  for (int i = 0; i < 7; i++)
  {
    gpio_put(SDATA_PIN, (adress << i) & 64);
    sleep_us(5);
    gpio_put(SDCLK_PIN, 1);
    sleep_us(5);

    gpio_put(SDCLK_PIN, 0);
    sleep_us(10);
  }
  for (int i = 0; i < 8; i++)
  {
    gpio_put(SDATA_PIN, (data << i) & 128);
    sleep_us(5);
    gpio_put(SDCLK_PIN, 1);
    sleep_us(5);
    gpio_put(SDCLK_PIN, 0);
    sleep_us(10);
  }
  gpio_put(SDENDB_PIN, 1);
  gpio_deinit(SDATA_PIN);
  gpio_deinit(SDCLK_PIN);
}
uint8_t read_register(uint8_t adress)
{
  gpio_init(SDATA_PIN);
  gpio_init(SDCLK_PIN);
  // start serial communication by setting SDENB low
  uint8_t register_value = 0;
  gpio_put(SDENDB_PIN, 0);
  gpio_set_dir(SDATA_PIN, GPIO_OUT);
  gpio_set_dir(SDCLK_PIN, GPIO_OUT);
  // first set output and then cycle clock
  // set first bit high for read
  gpio_put(SDATA_PIN, 1);
  sleep_us(5);
  gpio_put(SDCLK_PIN, 1);
  sleep_us(5);
  gpio_put(SDCLK_PIN, 0);
  sleep_us(10);

  for (int i = 0; i < 7; i++)
  {
    gpio_put(SDATA_PIN, (adress << i) & 64);
    sleep_us(5);
    gpio_put(SDCLK_PIN, 1);
    sleep_us(5);

    gpio_put(SDCLK_PIN, 0);
    sleep_us(10);
  }
  gpio_set_dir(SDATA_PIN, GPIO_IN);
  gpio_pull_up(SDATA_PIN);
  for (int i = 0; i < 8; i++)
  {

    sleep_us(5);
    register_value = register_value << 1 | gpio_get(SDATA_PIN);
    gpio_put(SDCLK_PIN, 1);
    sleep_us(5);
    gpio_put(SDCLK_PIN, 0);
    sleep_us(10);
  }
  gpio_put(SDENDB_PIN, 1);
  gpio_deinit(SDATA_PIN);
  gpio_deinit(SDCLK_PIN);
  return register_value;
}
bool check_registers()
{
  for (uint8_t i = 0; i < 87; i++)
  {
    // printf("reg %x : %x,[%x]\n", i, read_register(i), reg_table[i]);
    if (read_register(i) != reg_table[i])
    {
      return false;
    }
    sleep_us(100);
  }
  return true;
}
void write_registers()
{

  for (uint8_t i = 0; i < 87; i++)
  {
    write_register(i, reg_table[i]);
    sleep_us(100);
  }
}
void toggle_reset()
{
  gpio_put(RESETB_PIN, 0);
  sleep_ms(1); // Maybe this needs to be longer?
  gpio_put(RESETB_PIN, 1);
}
void data_in_finish()
{
  dma_hw->ints0 = 1u << pixel_dma_chan;
  lines_aqcuired++;
  // if (write_ready)
  // {
  // Only if the previous pixel buffer is written to the SD card we should
  // switch buffer.
  buffer_num = buffer_num ^ write_ready;
  write_ready = false;
  // send move command to stepper motor pio
  // pio_sm_put(pio0, sm_step, steps_per_line - 1);
  // pio_sm_put(pio0, sm_step, clock_speed / (int_time * steps_per_line * 2));
  // }
            //buffer_num = buffer_num ^ 1;
  pio_sm_exec(pio_ak8419,sm_data_in,pio_encode_jmp(offset_data_in+2));
  pio_sm_clear_fifos(pio_ak8419, sm_data_in);
    pio_sm_exec(pio_ak8419,sm_data_in,pio_encode_push(false,false));

  pio_sm_clear_fifos(pio_ak8419, sm_data_in);


  data_ready = true;
  dma_channel_set_write_addr(pixel_dma_chan, pixel_buffers[buffer_num], true);
}
bool ccd_start_capture()
{
  gpio_put(RESETB_PIN, 1);

  // configure the adc to send sync patterns in order to sync with PIO(sm_sync)
  ccd_config->sync_pattern = true;
  ccd_config->tg_enable = false;
  ccd_config->ADCK0_pin_out = 0b101;

  write_registers();
  if (!check_registers())
  {
    // return false;
    printf("register schrijven niet gelukt\n");
  }

  // Reinitilize the pio programs
  trigger_program_init(pio_ak8419, sm_trigger, offset_trigger, TRIGB_PIN);
  sync_program_init(pio_ak8419, sm_sync, offset_sync);
  data_in_program_init(pio_ak8419, sm_data_in, offset_data_in, D0_PIN);

  pio_sm_set_enabled(pio_ak8419, sm_sync, true);

  dma_channel_config c = dma_channel_get_default_config(pixel_dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_bswap(
      &c,
      true); // Makes sure that the bytes are in the right order due to endianes
  // Pace the DMA tranfer by the data_in PIO data ready signal
  channel_config_set_dreq(&c, pio_get_dreq(pio_ak8419, sm_data_in, false));
  dma_channel_set_irq0_enabled(pixel_dma_chan, true);
  irq_set_exclusive_handler(
      DMA_IRQ_0,
      data_in_finish); // Strarts interrupt that switches buffer location and
                       // restarts DMA when transfer is done
  irq_set_enabled(DMA_IRQ_0, true);
  dma_channel_configure(pixel_dma_chan,               // Channel to be configured
                        &c,                           // The configuration we just created
                        pixel_buffers[buffer_num],    // The initial write address
                        &pio_ak8419->rxf[sm_data_in], // Adress of data_in FIFO
                        CCD_PIXELS / 2,               // Every 32 bits contain 2 pixels
                        true                          // Start immediately.
  );
  ccd_config->sync_pattern = false;
  ccd_config->tg_enable = true;
  write_register(0x01, reg_table[0x01]); // disable sync output
  write_register(0x0C, reg_table[0x0C]); // Enable timing generator output

  pio_sm_clear_fifos(pio_ak8419, sm_data_in);
  pio_sm_set_enabled(pio_ak8419, sm_data_in, true);
            pio_sm_put(pio_ak8419, sm_data_in, CCD_PIXELS - 1+128*3);
  pio_sm_set_enabled(pio_ak8419, sm_trigger, true);
  pio_sm_put(pio_ak8419, sm_trigger, CLOCK_SPEED / int_time);

  return true;
}
bool ccd_stop_capture()
{
  pio_sm_set_enabled(pio_ak8419, sm_data_in, false);
  pio_sm_clear_fifos(pio_ak8419, sm_data_in);

  pio_sm_set_enabled(pio_ak8419, sm_trigger, false);
  pio_sm_clear_fifos(pio_ak8419, sm_trigger);

  pio_sm_set_enabled(pio_ak8419, sm_sync, false);
  pio_sm_clear_fifos(pio_ak8419, sm_sync);

  dma_channel_cleanup(pixel_dma_chan);

  gpio_put(RESETB_PIN, 0);
  return true;
}
bool ccd_init()
{
  // Init pins and set correct direction
  gpio_init(SDENDB_PIN);
  gpio_init(RESETB_PIN);
  gpio_init(OSCI_PIN);
  // gpio_set_drive_strength(OSCI_PIN,GPIO_DRIVE_STRENGTH_4MA);
  gpio_set_dir(SDENDB_PIN, GPIO_OUT);
  gpio_set_dir(RESETB_PIN, GPIO_OUT);
  gpio_set_dir(OSCI_PIN, GPIO_OUT);

  gpio_put(SDENDB_PIN, 1);
  gpio_put(RESETB_PIN, 0);

  // note sure if needed?
  for (int i = 0; i < 8; i++)
  {
    gpio_pull_down(D0_PIN + i);
  }
  // configure pwm channel for ADC clock

  gpio_set_function(OSCI_PIN, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(OSCI_PIN);
  uint channel = pwm_gpio_to_channel(OSCI_PIN);
  pwm_set_wrap(slice_num, 29);
  pwm_set_chan_level(slice_num, channel, 15);
  pwm_set_enabled(slice_num, true);

  // toggle reset after startup
  sleep_us(2000);
  gpio_put(RESETB_PIN, 1);

  sleep_ms(100);
  // Try writing and reading some registers to make sure the ADC is connected
  write_registers();
  if (!check_registers())
  {
    // return false;
    printf("register schrijven niet gelukt");
  }
  else
  {
    printf("register schrijven gelukt");
  }
  // turn off the ADC/CCD to save power
  gpio_put(RESETB_PIN, 0);

  // configure the PIO to read out ADC/CCD
  offset_trigger = pio_add_program(pio_ak8419, &trigger_program);
  offset_sync = pio_add_program(pio_ak8419, &sync_program);
  offset_data_in = pio_add_program(pio_ak8419, &data_in_program);

  // Setup for DMA from data_in FIFO a ping pong pixel_buffers
  pixel_dma_chan = dma_claim_unused_channel(true);

  // This defines how many times per second the TRIGB pulse is send and thus
  // defines the integration time
  // set_exposure_time(100);
  // while (lines_written < lines_total)
  // {
  //     if (data_ready)
  //     {
  //         data_ready = false;
  //         file.write(pixel_buffers[buffer_num ^ 1], CCD_BYTES);
  //         write_ready = true;
  //         lines_written++;
  //     }
  // }

  // stepper_return();

  // // turn on LED to show that the acquisition is done
  // gpio_put(LED_PIN, 1);
  // printf("acquired:%d, written:%d\n", lines_aqcuired, lines_written);
  return true;
}

void set_exposure_time(uint16_t exp_time) { int_time = exp_time; }
void set_gain(uint8_t gain0, uint8_t gain1, uint8_t gain2)
{
  ccd_config->gain_ccd0 = gain0;
  ccd_config->gain_ccd1 = gain1;
  ccd_config->gain_ccd2 = gain2;
}
void set_offset(uint8_t offset0, uint8_t offset1, uint8_t offset2)
{
  ccd_config->offset_ccd0 = offset0;
  ccd_config->offset_ccd1 = offset1;
  ccd_config->offset_ccd2 = offset2;
}
void print_config()
{
  printf("clock_mode = %d\n", ccd_config->clock_mode);
  printf("num_channel = %d\n", ccd_config->num_channel);
  printf("clock_freq = %d\n", ccd_config->clock_freq);
  printf("power_down = %d\n", ccd_config->power_down);
  printf("sync_pattern = %d\n", ccd_config->sync_pattern);
  printf("p_buffer = %d\n", ccd_config->p_buffer);
  printf("sh_buffer = %d\n", ccd_config->sh_buffer);
  printf("d_buffer = %d\n", ccd_config->d_buffer);
  printf("offset_ccd0 = %d\n", ccd_config->offset_ccd0);
  printf("offset_ccd1 = %d\n", ccd_config->offset_ccd1);
  printf("offset_ccd2 = %d\n", ccd_config->offset_ccd2);
  printf("gain_ccd0 = %d\n", ccd_config->gain_ccd0);
  printf("gain_ccd1 = %d\n", ccd_config->gain_ccd1);
  printf("gain_ccd2 = %d\n", ccd_config->gain_ccd2);
  printf("tg_enable = %d\n", ccd_config->tg_enable);
  printf("ADCK0_pin_out = %d\n", ccd_config->ADCK0_pin_out);
  printf("PSH_pol_half = %d\n", ccd_config->PSH_pol_half);
  printf("PSH_mode_D = %d\n", ccd_config->PSH_mode_D);
  printf("PSH_mode_V = %d\n", ccd_config->PSH_mode_V);
  printf("P0_mode_V = %d\n", ccd_config->P0_mode_V);
  printf("P0_mode_D = %d\n", ccd_config->P0_mode_D);
  printf("P1_mode_V = %d\n", ccd_config->P1_mode_V);
  printf("P1_mode_D = %d\n", ccd_config->P1_mode_D);
  printf("P0_pol_stop = %d\n", ccd_config->P0_pol_stop);
  printf("P0N_pol_stop = %d\n", ccd_config->P0N_pol_stop);
  printf("P1_pol_stop = %d\n", ccd_config->P1_pol_stop);
  printf("P1N_pol_stop = %d\n", ccd_config->P1N_pol_stop);
  printf("PRS_pol_stop = %d\n", ccd_config->PRS_pol_stop);
  printf("PCL_pol_stop = %d\n", ccd_config->PCL_pol_stop);
  printf("PSH_pol_stop = %d\n", ccd_config->PSH_pol_stop);
  printf("P0_delay = %d\n", ccd_config->P0_delay);
  printf("P0N_delay = %d\n", ccd_config->P0N_delay);
  printf("P1_delay = %d\n", ccd_config->P1_delay);
  printf("P1N_delay = %d\n", ccd_config->P1N_delay);
  printf("PRS_delay = %d\n", ccd_config->PRS_delay);
  printf("PCL_delay = %d\n", ccd_config->PCL_delay);
  printf("PSH_delay = %d\n", ccd_config->PSH_delay);
  printf("P0_rise_D_1 = %d\n", ccd_config->P0_rise_D_1);
  printf("P0_rise_D_2 = %d\n", ccd_config->P0_rise_D_2);
  printf("P0_fall_D_1 = %d\n", ccd_config->P0_fall_D_1);
  printf("P0_fall_D_2 = %d\n", ccd_config->P0_fall_D_2);
  printf("P0_rise_D_3 = %d\n", ccd_config->P0_rise_D_3);
  printf("P0_rise_D_4 = %d\n", ccd_config->P0_rise_D_4);
  printf("P0_fall_D_3 = %d\n", ccd_config->P0_fall_D_3);
  printf("P0_fall_D_4 = %d\n", ccd_config->P0_fall_D_4);
  printf("P0_rise_V_1 = %d\n", ccd_config->P0_rise_V_1);
  printf("P0_rise_V_2 = %d\n", ccd_config->P0_rise_V_2);
  printf("P0_fall_V_1 = %d\n", ccd_config->P0_fall_V_1);
  printf("P0_fall_V_2 = %d\n", ccd_config->P0_fall_V_2);
  printf("P0_rise_V_3 = %d\n", ccd_config->P0_rise_V_3);
  printf("P0_rise_V_4 = %d\n", ccd_config->P0_rise_V_4);
  printf("P0_fall_V_3 = %d\n", ccd_config->P0_fall_V_3);
  printf("P0_fall_V_4 = %d\n", ccd_config->P0_fall_V_4);
  printf("P1_rise_D_1 = %d\n", ccd_config->P1_rise_D_1);
  printf("P1_rise_D_2 = %d\n", ccd_config->P1_rise_D_2);
  printf("P1_fall_D_1 = %d\n", ccd_config->P1_fall_D_1);
  printf("P1_fall_D_2 = %d\n", ccd_config->P1_fall_D_2);
  printf("P1_rise_D_3 = %d\n", ccd_config->P1_rise_D_3);
  printf("P1_rise_D_4 = %d\n", ccd_config->P1_rise_D_4);
  printf("P1_fall_D_3 = %d\n", ccd_config->P1_fall_D_3);
  printf("P1_fall_D_4 = %d\n", ccd_config->P1_fall_D_4);
  printf("P1_rise_V_1 = %d\n", ccd_config->P1_rise_V_1);
  printf("P1_rise_V_2 = %d\n", ccd_config->P1_rise_V_2);
  printf("P1_fall_V_1 = %d\n", ccd_config->P1_fall_V_1);
  printf("P1_fall_V_2 = %d\n", ccd_config->P1_fall_V_2);
  printf("P1_rise_V_3 = %d\n", ccd_config->P1_rise_V_3);
  printf("P1_rise_V_4 = %d\n", ccd_config->P1_rise_V_4);
  printf("P1_fall_V_3 = %d\n", ccd_config->P1_fall_V_3);
  printf("P1_fall_V_4 = %d\n", ccd_config->P1_fall_V_4);
  printf("PRS_rise_D = %d\n", ccd_config->PRS_rise_D);
  printf("PRS_fall_D = %d\n", ccd_config->PRS_fall_D);
  printf("PRS_rise_V = %d\n", ccd_config->PRS_rise_V);
  printf("PRS_fall_V = %d\n", ccd_config->PRS_fall_V);
  printf("PCL_rise_D = %d\n", ccd_config->PCL_rise_D);
  printf("PCL_fall_D = %d\n", ccd_config->PCL_fall_D);
  printf("PCL_rise_V = %d\n", ccd_config->PCL_rise_V);
  printf("PCL_fall_V = %d\n", ccd_config->PCL_fall_V);
  printf("PSH_rise_D = %d\n", ccd_config->PSH_rise_D);
  printf("PSH_fall_D = %d\n", ccd_config->PSH_fall_D);
  printf("PSH_rise_V = %d\n", ccd_config->PSH_rise_V);
  printf("PSH_fall_V = %d\n", ccd_config->PSH_fall_V);
  printf("SHR_rise = %d\n", ccd_config->SHR_rise);
  printf("SHR_fall = %d\n", ccd_config->SHR_fall);
  printf("SHD_rise = %d\n", ccd_config->SHD_rise);
  printf("SHD_fall = %d\n", ccd_config->SHD_fall);
  printf("SH0_first_rise = %d\n", ccd_config->SH0_first_rise);
  printf("V_period_enable = %d\n", ccd_config->V_period_enable);
  printf("pixel_rate_start_stop_pos_2_enable = %d\n",
         ccd_config->pixel_rate_start_stop_pos_2_enable);
  printf("pixel_rate_start_stop_pos_1_enable = %d\n",
         ccd_config->pixel_rate_start_stop_pos_1_enable);
  printf("SH3_enable = %d\n", ccd_config->SH3_enable);
  printf("SH2_enable = %d\n", ccd_config->SH2_enable);
  printf("SH1_enable = %d\n", ccd_config->SH1_enable);
  printf("SH0_enable = %d\n", ccd_config->SH0_enable);
  printf("SHDM_enable = %d\n", ccd_config->SHDM_enable);
  printf("SH1_first_rise = %d\n", ccd_config->SH1_first_rise);
  printf("SH2_first_rise = %d\n", ccd_config->SH2_first_rise);
  printf("SH3_rise = %d\n", ccd_config->SH3_rise);
  printf("SH3_fall = %d\n", ccd_config->SH3_fall);
  printf("SHDM_first_rise = %d\n", ccd_config->SHDM_first_rise);
  printf("SHX_first_fall = %d\n", ccd_config->SHX_first_fall);
  printf("SHX_second_rise = %d\n", ccd_config->SHX_second_rise);
  printf("SHX_second_fall = %d\n", ccd_config->SHX_second_fall);
  printf("single_SH_single = %d\n", ccd_config->single_SH_single);
  printf("pixel_rate_clock_start_1 = %d\n",
         ccd_config->pixel_rate_clock_start_1);
  printf("pixel_rate_clock_end_1 = %d\n", ccd_config->pixel_rate_clock_end_1);
  printf("pixel_rate_clock_start_2 = %d\n",
         ccd_config->pixel_rate_clock_start_2);
  printf("pixel_rate_clock_end_2 = %d\n", ccd_config->pixel_rate_clock_end_2);
  printf("V_period_start = %d\n", ccd_config->V_period_start);
  printf("V_period_end = %d\n", ccd_config->V_period_end);
  printf("SH_pulse_polarity = %d\n", ccd_config->SH_pulse_polarity);
  printf("SH3_phase_adjust = %d\n", ccd_config->SH3_phase_adjust);
  printf("SH3_phase = %d\n", ccd_config->SH3_phase);
}

// int main()
// {
//     set_sys_clock_khz(clock_speed / 1000, true);
//     stdio_init_all();
//     // reg_table[0x06]=0b01110000;
//     reg_table[0x01] = 0b01000010; // sync mode output
//     reg_table[0x0C] = 0b00101000; // TG output disable with debug at d0-d3

//     sleep_ms(1000);
//     // SD card startup
//     //

//     return 0;
// }

// void stepper_init()
// {
//     gpio_init(DIR_PIN);
//     gpio_set_dir(DIR_PIN, GPIO_OUT);
//     gpio_put(DIR_PIN, 1);
//     uint offset_step = pio_add_program(pio_ak8419, &step_program);
//     step_program_init(pio_ak8419, sm_step, offset_step, STEP_PIN);
//     pio_sm_put(pio_ak8419, sm_step, steps_per_line - 1);
//     pio_sm_put(pio_ak8419, sm_step, clock_speed / (int_time * steps_per_line
//     * 2)); pio_sm_set_enabled(pio_ak8419, sm_step, true);
// }
// void stepper_return(){
//     gpio_put(DIR_PIN, 0);
//     pio_sm_put(pio_ak8419, sm_step, steps_per_line * lines_total - 1);
//     pio_sm_put(pio_ak8419, sm_step, clock_speed / RETURN_SPEED);
// }
