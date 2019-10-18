#ifndef COMM_H_
#define COMM_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <math.h>
#include <stdint.h>

#define COMM_HEADER_1     0x55
#define COMM_HEADER_2     0xAA
#define COMM_END          0x7D

#define COMM_REQUEST          0x01
#define COMM_RESPONSE         0x02

#define COMM_DATA_UNKNOWN             0x00
#define COMM_DATA_SET_SPEED           0x01
#define COMM_DATA_SET_PTZ             0x02
#define COMM_DATA_STATUS              0x03
#define COMM_DATA_BATTERY_STATUS      0x04
#define COMM_DATA_SET_LED             0x05
#define COMM_DATA_TRANSFORM           0x06

#define COMM_RESPONSE_OK          0x01
#define COMM_RESPONSE_ERROR       0x00

#define TX_TIMEOUT 10
#define RX_TIMEOUT 10

uint8_t get_request;

uint8_t comm_tx_buffer[50];
uint8_t comm_rx_buffer[50];
uint8_t comm_data_buffer[50];

void CommParse();
void CommCheck();
void CommSetup();

#ifdef __cplusplus
}
#endif

#endif /* COMM_H_ */
