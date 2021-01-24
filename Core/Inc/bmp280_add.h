int8_t BMP280_init();
int8_t BMP280_read(double *temp, int32_t *temp32);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(char * api_name, int8_t rslt);


