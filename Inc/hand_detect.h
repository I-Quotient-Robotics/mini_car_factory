#ifndef HAND_DETECT_H_
#define HAND_DETECT_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "common_func.h"

#include "stm32f1xx_hal.h"

#include "vl53l0x_api.h"

#include "main.h"

#define HAND_DETECT_THRES 80.0f // 80mm

I2C_HandleTypeDef* i2c_ptr;
VL53L0X_Dev_t vl53l0x_dev;

uint8_t hand_detect_flag;

void HandDetectSetup(I2C_HandleTypeDef* I2cPtr);
uint8_t HandDetectCheck();

#ifdef __cplusplus
}
#endif

#endif /* HAND_DETECT_H_ */
