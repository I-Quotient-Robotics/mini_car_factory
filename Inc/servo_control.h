#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <math.h>
#include <stdint.h>

#define SERVO_INDEX 0x01

#define SERVO_MSG_HEADER_1     0xFF
#define SERVO_MSG_HEADER_2     0xFF

#define SERVO_MSG_PING            0x01
#define SERVO_MSG_READ            0x02
#define SERVO_MSG_WRITE           0x03
#define SERVO_MSG_REG_WRITE       0x04
#define SERVO_MSG_ACTION          0x05
#define SERVO_MSG_RESET           0x06
#define SERVO_MSG_SYNC_WRITE      0x83

#define SERVO_MSG_CLOCKWISE_LIMIT     0x06
#define SERVO_MSG_C_CLOCKWISE_LIMIT   0x08
#define SERVO_MSG_SET_VOLTAGE         0x0C
#define SERVO_MSG_SET_FORCE           0x0E
#define SERVO_MSG_ENABLE_FORCE        0x18
#define SERVO_MSG_SPEED               0x20
#define SERVO_MSG_CURRENT_POSITION    0x24

#define TX_TIMEOUT 10
#define RX_TIMEOUT 10

enum TransformState {
  kTransformClose = 0x00,
  kTransformOpen,
  kTrasnforming,
  kTransformError,
  kTransformRelease
};

enum TransformState transform_state;

uint16_t servo_angle;
int8_t servo_position;

uint8_t servo_data_buffer[30];

uint8_t get_caught;
uint8_t transform_state;

uint8_t servo_limit_A;
uint8_t servo_limit_B;

uint8_t servo_ir_1;
uint8_t servo_ir_2;

void ServoSetup();
void ServoCheck();
void ServoSetSpeed(int16_t speed);

void Transform(uint8_t type);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_CONTROL_H_ */
