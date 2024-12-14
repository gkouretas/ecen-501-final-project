/*
 * vl53l0x_driver.h
 *
 *  Created on: Dec 4, 2024
 *      Author: thegr
 */

#ifndef INC_VL53L0X_DRIVER_H_
#define INC_VL53L0X_DRIVER_H_

#include "main.h"
#include <stdbool.h>

typedef struct VL53l0X_Interface VL53l0X_Interface_t;

VL53l0X_Interface_t *vl53l0x_init(I2C_HandleTypeDef *i2c_handle, uint16_t i2c_addr, GPIO_TypeDef *gpio_port, uint16_t gpio_pin);
void vl53l0x_set_isr_flag(void);
HAL_StatusTypeDef vl53l0x_prepare_sample(VL53l0X_Interface_t *interface);
HAL_StatusTypeDef vl53l0x_read_range_single(VL53l0X_Interface_t *interface, uint16_t *range, bool prepare_sample);

#endif /* INC_VL53L0X_DRIVER_H_ */
