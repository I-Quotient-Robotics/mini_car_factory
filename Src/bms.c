#include "bms.h"

#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal.h"

#include "common_func.h"

extern UART_HandleTypeDef huart4;

uint16_t BMSCheckSum(uint8_t *data, uint8_t length) {
  uint16_t check_sum = 0;

  for(uint8_t i=0; i<length; i++) {
    check_sum += data[i];
  }

  check_sum = ~check_sum + 1;
  return check_sum;
}

void BMSVoltagesDecode() {
  bms_cell_1_voltage = (bms_data_buffer[4]<<8|bms_data_buffer[5])/1000.0;
  bms_cell_2_voltage = (bms_data_buffer[6]<<8|bms_data_buffer[7])/1000.0;
  bms_cell_3_voltage = (bms_data_buffer[8]<<8|bms_data_buffer[9])/1000.0;
}

void BMSDecode() {
  bms_voltage = (bms_data_buffer[4]<<8|bms_data_buffer[5])/100.0;
  if(((bms_data_buffer[6] & 0b10000000) >> 7) == 1) {
    bms_current = ((bms_data_buffer[6]<<8|bms_data_buffer[7])-65535.0)/100.0;
  } else {
    bms_current = (bms_data_buffer[6]<<8|bms_data_buffer[7])/100.0;
  }

  bms_residual_capacity = (bms_data_buffer[8]<<8|bms_data_buffer[9])*10;
  bms_design_capacity = (bms_data_buffer[10]<<8|bms_data_buffer[11])*10;
  bms_cycle = (bms_data_buffer[12]<<8|bms_data_buffer[13]);
  bms_balance_state = (bms_data_buffer[16]<<8|bms_data_buffer[17]|bms_data_buffer[18]<<8|bms_data_buffer[19]);
  bms_protection_state = (bms_data_buffer[20]<<8|bms_data_buffer[21]);

  bms_rsoc = bms_data_buffer[23];
  bms_fet_state = bms_data_buffer[24];

  bms_cell_number = bms_data_buffer[25];
  bms_temperature = ((bms_data_buffer[27]<<8|bms_data_buffer[28])-2731.0)/10.0;
}

void BMSCheckStatus() {
  // uint8_t data[7];
  uint16_t check_sum;

  static uint8_t stage2_count = 0;

  switch(bms_check_stage) {
    case kBMSCheckStage1:
      /* Get BMS status */
      bms_trans_buffer[0] = BMS_MSG_HEADER;
      bms_trans_buffer[1] = BMS_MSG_READ;
      bms_trans_buffer[2] = BMS_MSG_STATUS;
      bms_trans_buffer[3] = 0x00;

      check_sum = BMSCheckSum(bms_trans_buffer+2, 2);
      bms_trans_buffer[4] = check_sum >> 8;
      bms_trans_buffer[5] = check_sum;
      bms_trans_buffer[6] = BMS_MSG_END;

      // HAL_UART_Transmit(&huart4, data, 7, TX_TIMEOUT);
      HAL_UART_Transmit_DMA(&huart4, bms_trans_buffer, 7);
      // HAL_UART_Receive(&huart4, bms_data_buffer, 32, 100);
      HAL_UART_Receive_DMA(&huart4, bms_data_buffer, 32);
      bms_data_received = 0;
      bms_check_stage = kBMSCheckStage2;
      stage2_count = 0;
      break;
    case kBMSCheckStage2:
      if(bms_data_received == 1) {
        BMSDecode();
        bms_check_stage = kBMSCheckStage3;
      } else {
        if(stage2_count > 4) {
          bms_check_stage = kBMSCheckStage1;
        }
        stage2_count += 1;
        // printf("no bms data 1\r\n");
      }
      break;
    case kBMSCheckStage3:
      /* Get each cell voltage */
      bms_trans_buffer[0] = BMS_MSG_HEADER;
      bms_trans_buffer[1] = BMS_MSG_READ;
      bms_trans_buffer[2] = BMS_MSG_VOLTAGES;
      bms_trans_buffer[3] = 0x00;

      check_sum = BMSCheckSum(bms_trans_buffer+2, 2);
      bms_trans_buffer[4] = check_sum >> 8;
      bms_trans_buffer[5] = check_sum;
      bms_trans_buffer[6] = BMS_MSG_END;

      // HAL_UART_Transmit(&huart4, data, 7, TX_TIMEOUT);
      HAL_UART_Transmit_DMA(&huart4, bms_trans_buffer, 7);
      // HAL_UART_Receive(&huart4, bms_data_buffer, 13, 100);
      HAL_UART_Receive_DMA(&huart4, bms_data_buffer, 13);
      bms_data_received = 0;
      bms_check_stage = kBMSCheckStage4;
      break;
    case kBMSCheckStage4:
      if(bms_data_received == 1) {
        BMSVoltagesDecode();
        bms_charging = HAL_GPIO_ReadPin(CHARGING_STATE_GPIO_Port, CHARGING_STATE_Pin);
        bms_check_stage = kBMSCheckStage1;

//        printf("BMS-1: voltage %.3fV, current %.3fA, rsoc %d%%, temp %.3fC\r\n", bms_voltage, bms_current, bms_rsoc, bms_temperature);
//        printf("BMS-2: cell_number %d, design_capacity %ldmAh, residual_capacity %ldmAh\r\n", bms_cell_number, bms_design_capacity, bms_residual_capacity);
//        printf("BMS-3: cell_1_voltage %.3fV, cell_2_voltage %.3fV, cell_3_voltage %.3fV\r\n", bms_cell_1_voltage, bms_cell_2_voltage, bms_cell_3_voltage);
      } else {
        // printf("no bms data 2\r\n");
      }
      break;
  }
}

void BMSCheckStatusAll() {
  uint8_t data[7];
  uint16_t check_sum;

  /* Get BMS status */
  data[0] = BMS_MSG_HEADER;
  data[1] = BMS_MSG_READ;
  data[2] = BMS_MSG_STATUS;
  data[3] = 0x00;

  check_sum = BMSCheckSum(data+2, 2);
  data[4] = check_sum >> 8;
  data[5] = check_sum;
  data[6] = BMS_MSG_END;

  HAL_UART_Transmit(&huart4, data, 7, TX_TIMEOUT);
  HAL_UART_Receive(&huart4, bms_data_buffer, 32, 100);
  BMSDecode();

  HAL_Delay(1);

  /* Get each cell voltage */
  data[0] = BMS_MSG_HEADER;
  data[1] = BMS_MSG_READ;
  data[2] = BMS_MSG_VOLTAGES;
  data[3] = 0x00;

  check_sum = BMSCheckSum(data+2, 2);
  data[4] = check_sum >> 8;
  data[5] = check_sum;
  data[6] = BMS_MSG_END;

  HAL_UART_Transmit(&huart4, data, 7, TX_TIMEOUT);
  HAL_UART_Receive(&huart4, bms_data_buffer, 13, 100);
  BMSVoltagesDecode();

  bms_charging = HAL_GPIO_ReadPin(CHARGING_STATE_GPIO_Port, CHARGING_STATE_Pin);
  bms_check_stage = kBMSCheckStage1;

  HAL_Delay(1);

  printf("BMS-1: voltage %.3fV, current %.3fA, rsoc %d%%, temp %.3fC\r\n", bms_voltage, bms_current, bms_rsoc, bms_temperature);
  printf("BMS-2: cell_number %d, design_capacity %ldmAh, residual_capacity %ldmAh\r\n", bms_cell_number, bms_design_capacity, bms_residual_capacity);
  printf("BMS-3: cell_1_voltage %.3fV, cell_2_voltage %.3fV, cell_3_voltage %.3fV\r\n", bms_cell_1_voltage, bms_cell_2_voltage, bms_cell_3_voltage);
}

void BMSCallback() {
  bms_data_received = 1;
  // printf("BMS callback\r\n");
}

void BMSSetup() {
  bms_check_stage = kBMSCheckStage1;
}

void BMSCheck() {
  BMSCheckStatus();
}
