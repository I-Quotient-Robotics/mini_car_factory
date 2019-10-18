/*
 * comm.c
 *
 *  Created on: Sep 19, 2019
 *      Author: flyma
 */

#include "comm.h"

#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal.h"

#include "common_func.h"

#include "bms.h"
#include "hand_detect.h"
#include "led_control.h"
#include "servo_control.h"
#include "robot_state.h"
#include "motor_control.h"

extern UART_HandleTypeDef huart1;

void SendBMSStatus() {
  uint8_t data[35];
  uint16_t temp_16;
  int16_t temp_sign;

  data[0] = COMM_HEADER_1;
  data[1] = COMM_HEADER_2;
  data[2] = 22;
  data[3] = COMM_RESPONSE;
  data[4] = COMM_DATA_BATTERY_STATUS;
  data[5] = bms_rsoc;

  temp_16 = (uint16_t)(bms_voltage*100.0f);
  memcpy(data+6, &temp_16, 2);

  temp_sign = (int16_t)(bms_current*100.0f);
  memcpy(data+8, &temp_sign, 2);

  temp_16 = (uint16_t)(bms_residual_capacity/10.0f);
  memcpy(data+10, &temp_16, 2);

  temp_16 = (uint16_t)(bms_design_capacity/10.0f);
  memcpy(data+12, &temp_16, 2);

  memcpy(data+14, &bms_cycle, 2);

  temp_sign = (int16_t)(bms_temperature*10.0f);
  memcpy(data+16, &temp_sign, 2);

  temp_16 = (uint16_t)(bms_cell_1_voltage*100.0f);
  memcpy(data+18, &temp_16, 2);

  temp_16 = (uint16_t)(bms_cell_2_voltage*100.0f);
  memcpy(data+20, &temp_16, 2);

  temp_16 = (uint16_t)(bms_cell_3_voltage*100.0f);
  memcpy(data+22, &temp_16, 2);

  data[24] = bms_charging;

  data[25] = CheckSum(data+2, 22);
  data[26] = COMM_END;

  HAL_UART_Transmit(&huart1, data, 27, TX_TIMEOUT);
//  HAL_UART_Transmit_DMA(&huart1, data, 27);
}

void SendStatus() {
  uint8_t data[35];
  int16_t temp = 0;

  data[0] = COMM_HEADER_1;
  data[1] = COMM_HEADER_2;
  data[2] = 26;
  data[3] = COMM_RESPONSE;
  data[4] = COMM_DATA_STATUS;

  /* motor speed and position */
  data[5] = motor_enable;
  for(uint8_t i=0; i<4; i++) {
    temp = (int16_t)(motor_state[i+1].speed*1000.0f);
    memcpy(data+6+i*2, &temp, 2);
  }
  for(uint8_t i=0; i<4; i++) {
    temp = (int16_t)(motor_state[i+1].position*10000.0f);
    memcpy(data+14+i*2, &temp, 2);
  }

  /* ptz position */
  data[22] = ptz_enable;
  // ptz_position = M_PI / 4.0f;
  temp = (int16_t)((ptz_state.position)*10000.0);
  memcpy(data+23, &temp, 2);

  /* sensor info */
  data[25] = servo_position;
  data[26] = transform_state;
  data[27] = get_caught;
  data[28] = hand_detect_flag;

  data[29] = CheckSum(data+3, 25);
  data[30] = COMM_END;

  HAL_UART_Transmit(&huart1, data, 31, TX_TIMEOUT);
//  HAL_UART_Transmit_DMA(&huart1, data, 31);
}

void SendLEDState() {
  uint16_t temp;

  comm_tx_buffer[0] = COMM_HEADER_1;
  comm_tx_buffer[1] = COMM_HEADER_2;
  comm_tx_buffer[2] = 5;
  comm_tx_buffer[3] = COMM_RESPONSE;
  comm_tx_buffer[4] = COMM_DATA_SET_LED;
  comm_tx_buffer[5] = led_control.state;

  temp = LEDGetBytes();
  memcpy(comm_tx_buffer+6, &temp, 2);
  comm_tx_buffer[8] = CheckSum(comm_tx_buffer+3, 5);
  comm_tx_buffer[9] = COMM_END;

  HAL_UART_Transmit(&huart1, comm_tx_buffer, 10, TX_TIMEOUT);
}

void CommResponse(uint8_t command, uint8_t status) {
  comm_tx_buffer[0] = COMM_HEADER_1;
  comm_tx_buffer[1] = COMM_HEADER_2;
  comm_tx_buffer[2] = 3;
  comm_tx_buffer[3] = COMM_RESPONSE;
  comm_tx_buffer[4] = command;
  comm_tx_buffer[5] = status;
  comm_tx_buffer[6] = CheckSum(comm_tx_buffer+3, 3);
  comm_tx_buffer[7] = COMM_END;

  HAL_UART_Transmit(&huart1, comm_tx_buffer, 8, TX_TIMEOUT);
//  HAL_UART_Transmit_DMA(&huart1, comm_tx_buffer, 8);
}

uint8_t CommCheckPacket() {
  uint8_t data_length = 0;
  if((comm_rx_buffer[0]==0x55) & (comm_rx_buffer[1]==0xAA)) {
    data_length = comm_rx_buffer[2];
    if(comm_rx_buffer[4+data_length]==0x7D) {
      if(comm_rx_buffer[4+data_length-1] == CheckSum(comm_rx_buffer+3, data_length)) {
        return 1;
      }
    }
  }

  return 0;
}

void DataParse() {
  uint16_t utemp_16 = 0;
  int16_t temp_16 = 0;

  /* Parse */
  switch(comm_data_buffer[4]) {
    case COMM_DATA_SET_SPEED:
      /* set Motor cmd */
      motor_enable_cmd = comm_data_buffer[5];
      for(uint8_t i=0; i<4; i++) {
        memcpy(&temp_16, comm_data_buffer+6+i*2, 2);
        motor_state[i+1].speed_cmd = temp_16 / 1000.0f;
      }

      /* response ok */
      CommResponse(COMM_DATA_SET_SPEED, COMM_RESPONSE_OK);

      printf("Command SET_MOTOR: %d, %7f, %7f, %7f, %7f\r\n",
             motor_enable_cmd, motor_state[1].speed_cmd, motor_state[2].speed_cmd, motor_state[3].speed_cmd, motor_state[4].speed_cmd);
      break;
    case COMM_DATA_SET_PTZ:
      ptz_enable = comm_data_buffer[5];

      memcpy(&temp_16, comm_data_buffer+6, 2);
      ptz_state.position_cmd = temp_16 / 10000.0f;
      CommResponse(COMM_DATA_SET_PTZ, COMM_RESPONSE_OK);
      printf("Command SET_PTZ: %7f\r\n", ptz_state.position_cmd);
      break;
    case COMM_DATA_STATUS:
      SendStatus();
      printf("Command DATA_STATUS: OK\r\n" );
      break;
    case COMM_DATA_BATTERY_STATUS:
      SendBMSStatus();
      printf("Command DATA_BATTERY_STATUS: OK\r\n" );
      break;
    case COMM_DATA_SET_LED:
      led_control.command = comm_data_buffer[5];
      memcpy(&utemp_16, comm_data_buffer+6, 2);
      for(uint8_t i=0; i<10; i++) {
        led_state[i].command = (utemp_16>>i)&0x01;
      }
      led_command_timestamp = HAL_GetTick();
      SendLEDState();
      printf("Command SET_LED: manual %d, led %d %d %d %d %d %d %d %d %d %d\r\n",
             led_control.command, led_state[0].command, led_state[1].command, led_state[2].command,
             led_state[3].command, led_state[4].command, led_state[5].command, led_state[6].command,
             led_state[7].command, led_state[8].command, led_state[9].command);
      break;
    case COMM_DATA_TRANSFORM:
      CommResponse(COMM_DATA_TRANSFORM, COMM_RESPONSE_ERROR);
      printf("Command DATA_TRANSFORM: ERROR\r\n" );
      break;
  }
}

void CommParse() {
  /* Packet check */
  if(CommCheckPacket() == 0) {
    printf("comm: invaild packet\r\n");
    CommResponse(COMM_DATA_UNKNOWN, COMM_RESPONSE_ERROR);
    get_request = 0;
    return;
  }

  get_request = 1;
}

void CommSetup() {
  get_request = 0;
  HAL_UART_Receive_DMA(&huart1, comm_rx_buffer, 31);
}

void CommCheck() {
  if(get_request == 1) {
    memcpy(comm_data_buffer, comm_rx_buffer, 31);
    DataParse();
  }
  get_request = 0;
//  SendMotorStatus();
//  HAL_Delay(1);
//  SendBMSStatus();
}

