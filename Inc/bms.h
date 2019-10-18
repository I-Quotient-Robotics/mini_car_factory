#ifndef BMS_H_
#define BMS_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

#include <math.h>
#include <stdint.h>

#define BMS_MSG_HEADER     0xDD
#define BMS_MSG_END        0x77

#define BMS_MSG_READ      0xA5
#define BMS_MSG_WRITE     0x5A

#define BMS_MSG_STATUS          0x03
#define BMS_MSG_VOLTAGES        0x04
#define BMS_MSG_HARDWARE_INFO   0x05

#define BMS_MSG_RESPONSE_OK     0x00
#define BMS_MSG_RESPONSE_ERROR  0x80

#define TX_TIMEOUT 10
#define RX_TIMEOUT 10

enum BMSCheckStage {
  kBMSCheckStage1 = 0x01,   // send bms data request
  kBMSCheckStage2,          // get bms data
  kBMSCheckStage3,          // send cells voltage data request
  kBMSCheckStage4,          // get cells voltage data
};
enum BMSCheckStage bms_check_stage;

uint8_t bms_data_received;

uint8_t bms_trans_buffer[30];
uint8_t bms_data_buffer[35];

/* bms status */
uint8_t bms_charging;

float bms_voltage;
float bms_current;

uint16_t bms_cycle;
uint32_t bms_design_capacity;
uint32_t bms_residual_capacity;

float bms_cell_1_voltage;
float bms_cell_2_voltage;
float bms_cell_3_voltage;

uint8_t bms_rsoc;
float bms_temperature;

uint32_t bms_balance_state;
uint8_t bms_fet_state;
uint16_t bms_protection_state;

char bms_production_date[10];
uint8_t bms_cell_number;
/* bms status */

uint16_t BMSCheckSum(uint8_t *data, uint8_t length);

void BMSCallback();
void BMSSetup();
void BMSCheck();

uint8_t BMSCharging();

void BMSCheckStatusAll();

#ifdef __cplusplus
}
#endif

#endif /* BMS_H_ */
