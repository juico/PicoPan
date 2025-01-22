bool jo_write_jpg( int width, int height, int quality);
bool process_block_line(uint8_t *image_data, int width);
bool process_block(uint8_t *image_data, int width,int x);

bool jo_write_jpg_end();
void init_gamma_table();