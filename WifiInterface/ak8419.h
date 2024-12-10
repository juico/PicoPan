
#include <stdio.h>

using namespace std;

#define D0_PIN 1
#define SDENDB_PIN 27
#define OSCI_PIN 0
#define RESETB_PIN 28
#define TRIGB_PIN 26
#define SDCLK_PIN D0_PIN + 7
#define SDATA_PIN D0_PIN + 6

#define STEP_PIN 18
#define DIR_PIN 19
#define ENABLE_PIN 22
#define RETURN_SPEED 5000

#define LED_PIN 25

#define CCD_PIXELS 24000//16384
#define CCD_BYTES CCD_PIXELS * 2
#define CLOCK_SPEED 150000000
struct __attribute__((__packed__)) ak8419_config {
  bool power_down : 1;
  uint8_t clock_freq : 2;
  uint8_t num_channel : 3;
  uint8_t : 1;
  bool clock_mode : 1;

  uint8_t : 6;
  bool sync_pattern : 1;
  bool : 1;

  uint8_t : 8;

  uint8_t d_buffer : 2;
  uint8_t sh_buffer : 2;
  uint8_t p_buffer : 2;
  uint8_t cds_buffer : 2;

  uint16_t : 16;

  uint8_t offset_ccd0 : 8;

  uint8_t offset_ccd1 : 8;

  uint8_t offset_ccd2 : 8;

  uint8_t gain_ccd0 : 6;
  uint8_t : 2;

  uint8_t gain_ccd1 : 6;
  uint8_t : 2;

  uint8_t gain_ccd2 : 6;
  uint8_t : 2;

  bool PSH_mode_V : 1;
  bool PSH_mode_D : 1;
  bool PSH_pol_half : 1;
  uint8_t ADCK0_pin_out : 3;
  bool tg_enable : 1;
  uint8_t : 1;

  bool P1_mode_D : 1;
  bool P1_mode_V : 1;
  bool P0_mode_D : 1;
  bool P0_mode_V : 1;
  uint8_t : 4;

  bool PSH_pol_stop : 1;
  bool PCL_pol_stop : 1;
  bool PRS_pol_stop : 1;
  bool P1N_pol_stop : 1;
  bool P1_pol_stop : 1;
  bool P0N_pol_stop : 1;
  bool P0_pol_stop : 1;
  uint8_t : 1;

  uint8_t : 8;

  uint8_t P1N_delay : 2;
  uint8_t P1_delay : 2;
  uint8_t P0N_delay : 2;
  uint8_t P0_delay : 2;

  uint8_t PSH_delay : 2;
  uint8_t PCL_delay : 2;
  uint8_t PRS_delay : 2;
  uint8_t : 2;

  uint8_t P0_rise_D_2 : 4;
  uint8_t P0_rise_D_1 : 4;

  uint8_t P0_fall_D_2 : 4;
  uint8_t P0_fall_D_1 : 4;

  uint8_t P0_rise_D_4 : 4;
  uint8_t P0_rise_D_3 : 4;

  uint8_t P0_fall_D_4 : 4;
  uint8_t P0_fall_D_3 : 4;

  uint8_t P0_rise_V_2 : 4;
  uint8_t P0_rise_V_1 : 4;

  uint8_t P0_fall_V_2 : 4;
  uint8_t P0_fall_V_1 : 4;

  uint8_t P0_rise_V_4 : 4;
  uint8_t P0_rise_V_3 : 4;

  uint8_t P0_fall_V_4 : 4;
  uint8_t P0_fall_V_3 : 4;

  uint8_t P1_rise_D_2 : 4;
  uint8_t P1_rise_D_1 : 4;

  uint8_t P1_fall_D_2 : 4;
  uint8_t P1_fall_D_1 : 4;

  uint8_t P1_rise_D_4 : 4;
  uint8_t P1_rise_D_3 : 4;

  uint8_t P1_fall_D_4 : 4;
  uint8_t P1_fall_D_3 : 4;

  uint8_t P1_rise_V_2 : 4;
  uint8_t P1_rise_V_1 : 4;

  uint8_t P1_fall_V_2 : 4;
  uint8_t P1_fall_V_1 : 4;

  uint8_t P1_rise_V_4 : 4;
  uint8_t P1_rise_V_3 : 4;

  uint8_t P1_fall_V_4 : 4;
  uint8_t P1_fall_V_3 : 4;

  uint8_t PRS_rise_D : 6;
  bool PRS_repeat : 1;
  uint8_t : 1;

  uint8_t PRS_fall_D : 6;
  uint8_t : 2;

  uint8_t PRS_rise_V : 6;
  uint8_t : 2;

  uint8_t PRS_fall_V : 6;
  uint8_t : 2;

  uint8_t PCL_rise_D : 6;
  uint8_t : 2;

  uint8_t PCL_fall_D : 6;
  uint8_t : 2;

  uint8_t PCL_rise_V : 6;
  uint8_t : 2;

  uint8_t PCL_fall_V : 6;
  uint8_t : 2;

  uint8_t PSH_rise_D : 6;
  uint8_t : 2;

  uint8_t PSH_fall_D : 6;
  uint8_t : 2;

  uint8_t PSH_rise_V : 6;
  uint8_t : 2;

  uint8_t PSH_fall_V : 6;
  uint8_t : 2;

  uint8_t SHR_rise : 6;
  uint8_t : 2;

  uint8_t SHR_fall : 6;
  uint8_t : 2;

  uint8_t SHD_fall : 4;
  uint8_t SHD_rise : 4;

  uint16_t SH0_first_rise : 16;

  uint8_t : 8;

  bool SHDM_enable : 1;
  bool SH0_enable : 1;
  bool SH1_enable : 1;
  bool SH2_enable : 1;
  bool SH3_enable : 1;
  bool pixel_rate_start_stop_pos_1_enable : 1;
  bool pixel_rate_start_stop_pos_2_enable : 1;
  bool V_period_enable : 1;

  uint16_t SH1_first_rise : 16;

  uint16_t : 16;

  uint16_t SH2_first_rise : 16;

  uint16_t : 16;

  uint16_t SH3_rise : 16;

  uint16_t SH3_fall : 16;

  uint8_t SHDM_first_rise : 8;

  uint8_t : 8;

  uint8_t SHX_first_fall : 8;

  uint8_t SHX_second_rise : 8;

  uint8_t SHX_second_fall : 8;

  bool single_SH_single : 1;
  uint8_t : 7;

  uint8_t pixel_rate_clock_start_1 : 8;

  uint8_t pixel_rate_clock_end_1 : 8;

  uint16_t pixel_rate_clock_start_2 : 16;

  uint16_t pixel_rate_clock_end_2 : 16;

  uint16_t V_period_start : 16;

  uint16_t V_period_end : 16;

  uint8_t SH3_phase : 6;
  bool SH3_phase_adjust : 1;
  bool SH_pulse_polarity : 1;

  uint32_t : 32;
};

bool ccd_init();
bool ccd_start_capture();
bool ccd_stop_capture();
void stepper_return();
void stepper_init();
void set_exposure_time(uint16_t exp_time);
void set_gain(uint8_t gain0, uint8_t gain1, uint8_t gain2);
void set_offset(uint8_t offset0, uint8_t offset1, uint8_t offset2);
void tiff_create();
void print_config();
void tiff_write_header(u_int8_t *rawbuffer, struct tiff_header header);
