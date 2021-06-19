/*
 * bme280_control.h
 *
 *  Created on: Jun 12, 2021
 *      Author: Void
 */

#ifndef INC_BME280_CONTROL_H_
#define INC_BME280_CONTROL_H_

int8_t bme280_init_indoor(void);
int8_t bme280_read(void);
void print_sensor_data(struct bme280_data *comp_data);
void user_delay_ms(uint32_t period, void *intf_ptr);
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

#endif /* INC_BME280_CONTROL_H_ */
