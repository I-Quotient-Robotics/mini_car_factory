#ifndef MOTOR_CONTRO_H_
#define MOTOR_CONTRO_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <math.h>
#include <stdint.h>

#define MOTOR_ALL           0x00
#define MOTOR_FL            0x01
#define MOTOR_FR            0x02
#define MOTOR_BR            0x03
#define MOTOR_BL            0x04
#define MOTOR_PTZ           0x05

#define MOTOR_CMD_HEADER        0x3E

#define MOTOR_CMD_ENCODER_ZERO      0x19
#define MOTOR_CMD_GET_PID			0x30
#define MOTOR_CMD_SET_PID_ROM		0x32
#define MOTOR_CMD_OFF               0x80
#define MOTOR_CMD_STOP              0x81
#define MOTOR_CMD_RESUME            0x88

#define MOTOR_CMD_ENCODER               0x90

#define MOTOR_CMD_FORCE_LOOP            0xA1
#define MOTOR_CMD_SPEED_LOOP            0xA2
#define MOTOR_CMD_POSITION_LOOP1        0xA3
#define MOTOR_CMD_POSITION_LOOP2        0xA4
#define MOTOR_CMD_POSITION_LOOP3        0xA5
#define MOTOR_CMD_POSITION_LOOP4        0xA6

#define TX_TIMEOUT 10
#define RX_TIMEOUT 10

#define MOTOR_ENCODER_MAX 16383.0f

#define MOTOR_ENCODER_OFFSET 5800    // degrees * 100

#define PTZ_SPEED 9000.0f

#define PTZ_MIN_RAD -3.2724924f
#define PTZ_MAX_RAD 0.9162979f

enum GyemsCommandType {
  kGyemsCommandGetState1 = 0x9A,
  kGyemsCommandGetState2 = 0x9C,
};

struct MotorState {
  uint8_t state;
  uint8_t enable;

  uint32_t tick;

  float speed;
  float current;
  uint16_t encoder;
  float position;
  float temperature;

  float speed_cmd;
  float position_cmd;
};

struct MotorPID {
	uint8_t pos_p;
	uint8_t pos_i;
	uint8_t speed_p;
	uint8_t speed_i;
	uint8_t effort_p;
	uint8_t effort_i;
};

struct MotorPID MotorGetPID(uint8_t index);
void MotorSetPID(uint8_t index, struct MotorPID motor_pid);
void MotorCheck();
void MotorSetup();
void MotorStopAll();
void MotorShake();

void MotorSetPTZOffset();
void SetPTZPositionRad(float pos_rad);

void DisableMotor(uint8_t index);
void ResumeMotor(uint8_t index);
void SetMotorSpeed(uint8_t index, int32_t speed);

void SetMotorZero(uint8_t index);
void SetPTZZero();

int16_t ptz_angle_offset;
int16_t ptz_encoder_offset;

// motor index start from 1, so we need 5+1 array
struct MotorState ptz_state;
struct MotorState motor_state[5];

uint8_t motor_enable;
uint8_t motor_enable_cmd;
uint8_t ptz_enable;
uint8_t ptz_enable_cmd;

// float motor_cmd[4];
// float motor_speed[4];
// float motor_position[4];
// uint32_t motor_tick[5];
// uint16_t motor_encoder[5];

// float ptz_cmd;
// float ptz_position;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTRO_H_ */
