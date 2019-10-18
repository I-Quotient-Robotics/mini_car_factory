#include "servo_control.h"

#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal.h"

#include "main.h"
#include "common_func.h"

extern UART_HandleTypeDef huart2;
extern IWDG_HandleTypeDef hiwdg;

void ServoSetSpeed(int16_t speed) {
  uint8_t data[9];
  uint8_t direction;
  int16_t temp_speed;

  if(speed < 0) {
    temp_speed = -speed;
    direction = 1;
  } else {
    temp_speed = speed;
    direction = 0;
  }

  if(temp_speed > 1023) temp_speed = 1023;
  temp_speed |= (direction << 10);

  data[0] = SERVO_MSG_HEADER_1;
  data[1] = SERVO_MSG_HEADER_2;
  data[2] = SERVO_INDEX;
  data[3] = 5;
  data[4] = SERVO_MSG_WRITE;
  data[5] = SERVO_MSG_SPEED;
  memcpy(data+6, &temp_speed, 2);
  data[8] = ~CheckSum(data+2, 6);

  HAL_UART_Transmit(&huart2, data, 9, TX_TIMEOUT);
}

void ServoUpdatePosition() {
  uint8_t data[9];

  data[0] = SERVO_MSG_HEADER_1;
  data[1] = SERVO_MSG_HEADER_2;
  data[2] = SERVO_INDEX;
  data[3] = 4;
  data[4] = SERVO_MSG_READ;
  data[5] = SERVO_MSG_CURRENT_POSITION;
  data[6] = 2;
  data[7] = ~CheckSum(data+2, 6);

  HAL_UART_Transmit_DMA(&huart2, data, 8);
  HAL_Delay(1);

  HAL_UART_Receive_DMA(&huart2, data, 8);
  HAL_Delay(1);
  printf("Servo %d: ", SERVO_INDEX);
  for(int i=0; i<8; i++) {
    printf("%02X, ", data[i]);
  }
  printf("\r\n");
}

void ServoSetup() {
  uint8_t data[15];

  /* set servo clockwise and counter clockwise limit to 0 */
  data[0] = SERVO_MSG_HEADER_1;
  data[1] = SERVO_MSG_HEADER_2;
  data[2] = SERVO_INDEX;
  data[3] = 7;
  data[4] = SERVO_MSG_WRITE;
  data[5] = SERVO_MSG_CLOCKWISE_LIMIT;
  data[6] = 0x00;
  data[7] = 0x00;
  data[8] = 0x00;
  data[9] = 0x00;
  data[10] = ~CheckSum(data+2, 6);

  HAL_UART_Transmit(&huart2, data, 11, TX_TIMEOUT);
  HAL_Delay(1);

  /* set servo min/max voltage to 6.0v-13.0v */
  data[0] = SERVO_MSG_HEADER_1;
  data[1] = SERVO_MSG_HEADER_2;
  data[2] = SERVO_INDEX;
  data[3] = 5;
  data[4] = SERVO_MSG_WRITE;
  data[5] = SERVO_MSG_SET_VOLTAGE;
  data[6] = 0x3C;
  data[7] = 0x82;
  data[8] = ~CheckSum(data+2, 6);
  HAL_UART_Transmit(&huart2, data, 9, TX_TIMEOUT);
  HAL_Delay(1);

  /* set servo force to 512(0x200) */
  data[0] = SERVO_MSG_HEADER_1;
  data[1] = SERVO_MSG_HEADER_2;
  data[2] = SERVO_INDEX;
  data[3] = 5;
  data[4] = SERVO_MSG_WRITE;
  data[5] = SERVO_MSG_SET_FORCE;
  data[6] = 0xff;
  data[7] = 0x03;
  data[8] = ~CheckSum(data+2, 6);
  HAL_UART_Transmit(&huart2, data, 9, TX_TIMEOUT);
  HAL_Delay(1);

  /* set servo force to enable */
  data[0] = SERVO_MSG_HEADER_1;
  data[1] = SERVO_MSG_HEADER_2;
  data[2] = SERVO_INDEX;
  data[3] = 4;
  data[4] = SERVO_MSG_WRITE;
  data[5] = SERVO_MSG_ENABLE_FORCE;
  data[6] = 0x01;
  data[7] = ~CheckSum(data+2, 5);
  HAL_UART_Transmit(&huart2, data, 8, TX_TIMEOUT);
  HAL_Delay(1);
}

void ServoCheck() {
//  ServoSetSpeed(500);
//  HAL_Delay(2000);
//  ServoSetSpeed(-500);
//  HAL_Delay(2000);

  if(HAL_GPIO_ReadPin(IR_1_GPIO_Port, IR_1_Pin)==GPIO_PIN_SET &&
     HAL_GPIO_ReadPin(IR_2_GPIO_Port, IR_2_Pin)== GPIO_PIN_SET) {
    get_caught = 0;
  } else {
    get_caught = 1;
  }
}

void Transform(uint8_t type) {
  uint32_t timeout = 5000;

  switch(type) {
    case 0:
      while(timeout>0) {
        transform_state = kTrasnforming;
        servo_limit_B = HAL_GPIO_ReadPin(LIMIT_B_GPIO_Port, LIMIT_B_Pin);
        if(servo_limit_B == GPIO_PIN_SET) {
          printf("Transfrom Open\r\n");
          break;
        }

        ServoSetSpeed(-500);
        HAL_Delay(10);
        timeout -= 10;
      }
      ServoSetSpeed(0);
      servo_position = -70;
      transform_state = kTransformOpen;
      break;
    case 1:
      while(timeout>0) {
        transform_state = kTrasnforming;
        servo_limit_A = HAL_GPIO_ReadPin(LIMIT_A_GPIO_Port, LIMIT_A_Pin);
        servo_ir_1 = HAL_GPIO_ReadPin(IR_1_GPIO_Port, IR_1_Pin);
        servo_ir_2 = HAL_GPIO_ReadPin(IR_2_GPIO_Port, IR_2_Pin);
        // HAL_IWDG_Refresh(&hiwdg);
        if(servo_limit_A==GPIO_PIN_SET || servo_ir_1==GPIO_PIN_RESET || servo_ir_2==GPIO_PIN_RESET) {
          break;
        }

        ServoSetSpeed(500);
        HAL_Delay(10);
        timeout -= 10;

        printf("Transform sensor: trigger_B %d, ir_1 %d, ir_2 %d\r\n", servo_limit_A, servo_ir_1, servo_ir_2);
      }
      ServoSetSpeed(0);
      servo_position = 0;
      transform_state = kTransformClose;
      break;
  }
}
