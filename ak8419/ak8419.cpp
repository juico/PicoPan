
#include <iostream>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "SdFat.h"
#include "ak8419.h"
#include "ak8419.pio.h"
#include "step.pio.h"

#define SD_CONFIG SdioConfig(FIFO_SDIO)


SdExFat sd;
ExFile file;



uint32_t clock_speed = 150000000;
uint32_t int_time = 20;
uint32_t ratio = 8 * 51 * 16;
uint32_t steps_total = 150 * ratio / 18;
uint32_t lines_total = 4000;
uint32_t steps_per_line = steps_total / lines_total;

PIO pio_ak8419 = pio0;
uint sm_step = 0;
uint sm_data_in = 1;
uint sm_trigger = 2;
uint sm_sync = 3;

uint8_t pixel_buffer_1[CCD_BYTES] = {};
uint8_t pixel_buffer_2[CCD_BYTES] = {};
uint8_t *pixel_buffers[2] = {pixel_buffer_1, pixel_buffer_2};

int buffer_count = 0;
int chan;
bool data_ready = false;
bool registercheck = false;
bool write_ready = true;
uint32_t lines_aqcuired = 0;
uint32_t lines_written = 0;

uint8_t reg_table[87] = {
    0b00000000, 0b00000010, 0b00000000, 0b00000001, 0b00000000, 0b00000000, 0b10000000, 0b10000000, // 00h-07h
    0b10000000, 0b00000000, 0b00000000, 0b00000000, 0b01101000, 0b00000000, 0b01001111, 0b00000000, // 08h-0Fh
    0b01001111, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, // 10h-17h
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, // 18h-1Fh
    0b00000000, 0b00000000, 0b00000000, 0b00101000, 0b00000000, 0b00111111, 0b00101100, 0b00100100, // 20h-27h
    0b00111111, 0b00110000, 0b00110000, 0b00111111, 0b00110000, 0b00111111, 0b00010011, 0b00010000, // 28h-2Fh
    0b00011000, 0b00000000, 0b00000010, 0b00000000, 0b00111111, 0b00000000, 0b00000010, 0b00000000, // 30h-37h
    0b00000000, 0b00000000, 0b00000010, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, // 38h-3Fh
    0b00000000, 0b00000010, 0b00000000, 0b00011110, 0b00001010, 0b00011110, 0b00000000, 0b01010010, // 40h-47h
    0b00000001, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, // 48h-4Fh
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000              // 50h-56h
};


void putu8(uint8_t in, struct simple_buffer *buffer)
{
    buffer->buffer[buffer->index] = in;
    buffer->index++;
    if (buffer->index == HEADER_BLOCK_SIZE)
    {
        file.write(buffer->buffer, HEADER_BLOCK_SIZE);
        buffer->index = 0;
    }
}
void putu16(u_int16_t in, struct simple_buffer *buffer)
{
    putu8(in >> 8, buffer);
    putu8(in, buffer);
}
void putlong(u_int32_t in, struct simple_buffer *buffer)
{
    putu8(in >> 24, buffer);
    putu8(in >> 16, buffer);
    putu8(in >> 8, buffer);
    putu8(in, buffer);
}

void tiff_write_header(u_int8_t *rawbuffer, struct tiff_header header)
{
    struct simple_buffer header_buffer;
    header_buffer.index = 0;

    uint32_t start_image = 8 * header.ImageLength + 256;
    uint32_t startsector = start_image / HEADER_BLOCK_SIZE;
    if (start_image % HEADER_BLOCK_SIZE > 0)
    {
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
    for (long i = 0; i < header.ImageLength; i++)
    {
        putlong(start_image + i * header.blockSize, &header_buffer);
    }
    for (long i = 0; i < header.ImageLength; i++)
    {
        putlong(6 * header.ImageWidth, &header_buffer);
    }
    // write zeros to fill up sector to write the buffer to storage
    while (header_buffer.index)
    {
        putu8(0, &header_buffer);
    }
}

void toggle_reset()
{
    gpio_put(RESETB_PIN, 0);
    sleep_ms(1); // Maybe this needs to be longer?
    gpio_put(RESETB_PIN, 1);
}
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
        printf("reg %d : %d,[%d]\n", i, read_register(i), reg_table[i]);
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

void data_in_finish()
{
    dma_hw->ints0 = 1u << chan;
    lines_aqcuired++;
    if (write_ready)
    {
        // Only if the previous pixel buffer is written to the SD card we should switch buffer.
        buffer_count = buffer_count ^ 1;
        write_ready = false;
        // send move command to stepper motor pio
        pio_sm_put(pio0, sm_step, steps_per_line - 1);
        pio_sm_put(pio0, sm_step, clock_speed / (int_time * steps_per_line * 2));
    }

    pio_sm_clear_fifos(pio0, 1);
    data_ready = true;
    dma_channel_set_write_addr(chan, pixel_buffers[buffer_count], true);
}
int main()
{
    set_sys_clock_khz(clock_speed / 1000, true);
    stdio_init_all();
    // reg_table[0x06]=0b01110000;
    reg_table[0x01] = 0b01000010; // sync mode output
    reg_table[0x0C] = 0b00101000; // TG output disable with debug at d0-d3
    
    sleep_ms(1000);
    // SD card startup
    //

    return 0;
}
bool sd_init()
{
    if (!sd.begin(SD_CONFIG))
    {
        printf("SDCARD connection error\n");
        return false;
    }
    return true;
}
void tiff_create()
{
    uint16_t imgnum = 0;
    char filename[] = "image00.tiff";
    while (sd.exists(filename))
    {
        imgnum++;
        filename[5] = imgnum / 10 + '0';
        filename[6] = imgnum % 10 + '0';
    }
    if (file.open(filename, O_RDWR | O_CREAT | O_TRUNC))
    {
        printf("Opnening image file succes\n");
    }

    struct tiff_header tiff_file;
    tiff_file.ImageWidth = 5400;
    tiff_file.ImageLength = lines_total;
    tiff_file.BitsPerSample = 16;
    tiff_file.Compression = 1;
    tiff_file.PhotometricInterpretation = 2;
    tiff_file.RowsPerStrip = 1;
    tiff_file.SamplesPerPixel = 3;
    tiff_file.XResolution = 300;
    tiff_file.YResolution = 300;
    tiff_file.ResolutionUnit = 2;
    tiff_file.blockSize = CCD_BYTES;

    uint32_t start_image = 8 * tiff_file.ImageLength + 256;
    uint32_t startsector = start_image / HEADER_BLOCK_SIZE;
    if (start_image % HEADER_BLOCK_SIZE > 0)
    {
        startsector++;
    }
    start_image = startsector * HEADER_BLOCK_SIZE;

    uint32_t file_size = CCD_BYTES * lines_total + start_image;
    file.preAllocate(file_size);
    uint32_t currentsector = file.firstSector();
    sd.card()->writeStart(currentsector, file_size / 512);

    write_header(pixel_buffers[0], tiff_file);
}
void set_exposure_time(uint16_t exp_time)
{

    pio_sm_put(pio_ak8419, sm_trigger, clock_speed / exp_time);
}
bool ccd_init()
{
    // Init pins and set correct direction
    gpio_init(SDENDB_PIN);
    gpio_init(RESETB_PIN);
    gpio_set_dir(SDENDB_PIN, GPIO_OUT);
    gpio_set_dir(RESETB_PIN, GPIO_OUT);
    gpio_put(SDENDB_PIN, 1);
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
    // note sure if needed?
    for (int i = 0; i < 8; i++)
    {
        gpio_pull_down(D0_PIN + i);
    }
    // configure pwm channel for CCD clock
    gpio_set_dir(OSCI_PIN, GPIO_OUT);
    gpio_set_function(OSCI_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(OSCI_PIN);
    uint channel = pwm_gpio_to_channel(OSCI_PIN);
    pwm_set_wrap(slice_num, 29);
    pwm_set_chan_level(slice_num, channel, 15);
    pwm_set_enabled(slice_num, true);

    // toggle reset after startup
    gpio_put(RESETB_PIN, 0);
    sleep_us(2000);
    gpio_put(RESETB_PIN, 1);

    sleep_ms(100);
    write_registers();
    if (!check_registers())
    {
        return false;
    }
    // start the PIO to read out CCD
    uint offset_trigger = pio_add_program(pio_ak8419, &trigger_program);
    printf("Trigger program loaded at:%d\n", offset_trigger);
    trigger_program_init(pio_ak8419, sm_trigger, offset_trigger, TRIGB_PIN);

    uint offset_sync = pio_add_program(pio_ak8419, &sync_program);
    sync_program_init(pio_ak8419, sm_sync, offset_sync);
    pio_sm_set_enabled(pio_ak8419, sm_sync, true);

    uint offset_data_in = pio_add_program(pio_ak8419, &data_in_program);
    printf("Loaded data in program at%d\n", offset_data_in);
    data_in_program_init(pio_ak8419, sm_data_in, offset_data_in, D0_PIN);

    // Setup for DMA from data_in FIFO a ping pong pixel_buffers
    chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_bswap(&c, true); // Makes sure that the bytes are in the right order due to endianes
    // Pace the DMA tranfer by the data_in PIO data ready signal
    channel_config_set_dreq(&c, pio_get_dreq(pio_ak8419, sm_data_in, false));
    dma_channel_configure(
        chan,                         // Channel to be configured
        &c,                           // The configuration we just created
        pixel_buffers[buffer_count],  // The initial write address
        &pio_ak8419->rxf[sm_data_in], // Adress of data_in FIFO
        CCD_PIXELS / 2,               // Every 32 bits contain 2 pixels
        true                          // Start immediately.
    );

    dma_channel_set_irq0_enabled(chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, data_in_finish); // Strarts interrupt that switches buffer location and restarts DMA when transfer is done
    irq_set_enabled(DMA_IRQ_0, true);

    write_register(0x01, 0b00000010); // disable sync output
    write_register(0x0C, 0b01101000); // Enable timing generator output

    pio_sm_set_enabled(pio_ak8419, sm_data_in, true);
    pio_sm_put(pio_ak8419, sm_data_in, CCD_PIXELS - 1);
    pio_sm_set_enabled(pio_ak8419, sm_trigger, true);

    // This defines how many times per second the TRIGB pulse is send and thus defines the integration time
    set_exposure_time(100);
    while (lines_written < lines_total)
    {
        if (data_ready)
        {
            data_ready = false;
            file.write(pixel_buffers[buffer_count ^ 1], CCD_BYTES);
            write_ready = true;
            lines_written++;
        }
    }
    // turn off the CCD
    gpio_put(RESETB_PIN, 0);
    stepper_return();

    // turn on LED to show that the acquisition is done
    gpio_put(LED_PIN, 1);
    file.sync();
    printf("acquired:%d, written:%d\n", lines_aqcuired, lines_written);
    return true;
}

void stepper_init()
{
    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 1);
    uint offset_step = pio_add_program(pio_ak8419, &step_program);
    step_program_init(pio_ak8419, sm_step, offset_step, STEP_PIN);
    pio_sm_put(pio_ak8419, sm_step, steps_per_line - 1);
    pio_sm_put(pio_ak8419, sm_step, clock_speed / (int_time * steps_per_line * 2));
    pio_sm_set_enabled(pio_ak8419, sm_step, true);
}
void stepper_return(){
    gpio_put(DIR_PIN, 0);
    pio_sm_put(pio_ak8419, sm_step, steps_per_line * lines_total - 1);
    pio_sm_put(pio_ak8419, sm_step, clock_speed / RETURN_SPEED);
}