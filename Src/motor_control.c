#include "motor_control.h"

#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal.h"

#include "robot_state.h"
#include "common_func.h"

extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef hiwdg;

void SerializeInt32(uint8_t *buf, int32_t val) {
  uint32_t uval = val;
  buf[0] = uval;
  buf[1] = uval >> 8;
  buf[2] = uval >> 16;
  buf[3] = uval >> 24;
}

uint16_t ParseUInt16(uint8_t *buf) {
  uint16_t value = 0;
  value = (buf[1] << 8) | buf[0];
  return value;
}

int16_t ParseInt16(uint8_t *buf) {
  int16_t value = 0;
  memcpy(&value, buf, 2);
  // value = (buf[1] << 8) | buf[0];
  return value;
}

float MotorGetCurrent(int16_t value) {
  return (value - (-2048.0f)) * (33.0f - (-33.0f)) / (2048.0f - (-2048.0f)) + (-33.0f);
}

void ResumeMotor(uint8_t index) {
  uint8_t data[5] = {0};

  if(index > 0x00) {
    data[0] = MOTOR_CMD_HEADER;
    data[1] = MOTOR_CMD_RESUME;
    data[2] = index;
    data[3] = 0x00;
    data[4] = CheckSum(data, 4);
    // HAL_UART_Transmit(&huart3, data, 5, TX_TIMEOUT);
    HAL_UART_Transmit_DMA(&huart3, data, 5);
    HAL_UART_Receive_DMA(&huart3, data, 5);
    HAL_Delay(2);
  } else {
    /* Unknown Motor Index */
  }
}

void DisableMotor(uint8_t index) {
  uint8_t data[5] = {0};

  if(index > 0) {
    data[0] = MOTOR_CMD_HEADER;
    data[1] = MOTOR_CMD_OFF;
    data[2] = index;
    data[3] = 0x00;
    data[4] = CheckSum(data, 4);
    // HAL_UART_Transmit(&huart3, data, 5, TX_TIMEOUT);
    HAL_UART_Transmit_DMA(&huart3, data, 5);
    HAL_UART_Receive_DMA(&huart3, data, 5);
    HAL_Delay(2);
  } else {
    /* Unknown Motor Index */
  }

  HAL_Delay(1);
  HAL_UART_Receive(&huart3, data, 5, RX_TIMEOUT);
}

void StopMotor(uint8_t index) {
  uint8_t data[5] = {0};

  if(index > 0) {
    data[0] = MOTOR_CMD_HEADER;
    data[1] = MOTOR_CMD_STOP;
    data[2] = index;
    data[3] = 0x00;
    data[4] = CheckSum(data, 4);
    // HAL_UART_Transmit(&huart3, data, 5, TX_TIMEOUT);
    HAL_UART_Transmit_DMA(&huart3, data, 5);
    HAL_UART_Receive_DMA(&huart3, data, 5);
    HAL_Delay(2);
  } else {
    /* Unknown Motor Index */
  }

  HAL_Delay(1);
  HAL_UART_Receive(&huart3, data, 5, RX_TIMEOUT);
}

void SetMotorZero(uint8_t index) {
  uint8_t data[5] = {0};

  if(index > 0) {
    data[0] = MOTOR_CMD_HEADER;
    data[1] = MOTOR_CMD_ENCODER_ZERO;
    data[2] = index;
    data[3] = 0x00;
    data[4] = CheckSum(data, 4);
    HAL_UART_Transmit_DMA(&huart3, data, 5);
    HAL_UART_Receive_DMA(&huart3, data, 5);
    HAL_Delay(2);
  } else {
    /* Unknown Motor Index */
  }
}

struct MotorPID MotorGetPID(uint8_t index) {
  uint8_t data[15] = {0};
  struct MotorPID motor_pid;

  if(index > 0) {
	data[0] = MOTOR_CMD_HEADER;
	data[1] = MOTOR_CMD_GET_PID;
	data[2] = index;
	data[3] = 0x00;
	data[4] = CheckSum(data, 4);
	HAL_UART_Transmit_DMA(&huart3, data, 5);
	HAL_UART_Receive_DMA(&huart3, data, 12);
	HAL_Delay(2);
	motor_pid.pos_p = data[5];
	motor_pid.pos_i = data[6];
	motor_pid.speed_p = data[7];
	motor_pid.speed_i = data[8];
	motor_pid.effort_p = data[9];
	motor_pid.effort_i = data[10];

	printf("Motor %d PID: pos %d %d, speed %d %d, effort %d %d\r\n", motor_pid.pos_p, motor_pid.pos_i, motor_pid.speed_p, motor_pid.speed_i, motor_pid.effort_p, motor_pid.effort_i);
  } else {
	/* Unknown Motor Index */
	printf("Unknown motor index");
  }

  return motor_pid;
}

void MotorSetPID(uint8_t index, struct MotorPID motor_pid) {
  uint8_t data[15] = {0};

  if(index > 0) {
	data[0] = MOTOR_CMD_HEADER;
	data[1] = MOTOR_CMD_SET_PID_ROM;
	data[2] = index;
	data[3] = 0x00;
	data[4] = CheckSum(data, 4);
	data[5] = motor_pid.pos_p;
	data[6] = motor_pid.pos_i;
	data[7] = motor_pid.speed_p;
	data[8] = motor_pid.speed_i;
	data[9] = motor_pid.effort_p;
	data[10] = motor_pid.effort_i;
	data[11] = CheckSum(data+5, 6);
	HAL_UART_Transmit_DMA(&huart3, data, 12);
	HAL_UART_Receive_DMA(&huart3, data, 12);
	printf("Motor %d set PID: pos %d %d, speed %d %d, effort %d %d\r\n", motor_pid.pos_p, motor_pid.pos_i, motor_pid.speed_p, motor_pid.speed_i, motor_pid.effort_p, motor_pid.effort_i);
  } else {
	/* Unknown Motor Index */
	printf("Unknown motor index");
  }
}

void SetMotorSpeedRad(uint8_t index, float speed) {
  int32_t speed_i = (int32_t)(speed / M_PI / 2.0 * 360.0 * 100.0);

  SetMotorSpeed(index, speed_i);
}

void SetMotorSpeed(uint8_t index, int32_t speed) {
  uint8_t data[20] = {0};
  uint8_t rxdata[20] = {0};

  // printf("motor_index %d\r\n", index);

  data[0] = MOTOR_CMD_HEADER;
  data[1] = MOTOR_CMD_SPEED_LOOP;
  data[2] = index;
  data[3] = 0x04;
  data[4] = CheckSum(data, 4);
  SerializeInt32(data+5, speed);
  data[9] = CheckSum(data+5, 4);
  HAL_UART_Transmit(&huart3, data, 10, TX_TIMEOUT);
//  HAL_UART_Transmit_DMA(&huart3, data, 10);

  HAL_UART_Receive_DMA(&huart3, rxdata, 13);
  HAL_Delay(2);

  if(rxdata[2] != index) {
    return;
  }

  /* detect suspiciously large readings, possibly from encoder roll over */
  float encoder_delta = (float)ParseUInt16(rxdata+10) - (float)motor_state[index].encoder;
  if(encoder_delta > 10000.0f) {
    encoder_delta = encoder_delta - MOTOR_ENCODER_MAX;
  } else if (encoder_delta < -10000.0f) {
    encoder_delta = encoder_delta + MOTOR_ENCODER_MAX;
  }

  motor_state[index].temperature = rxdata[5];
  motor_state[index].current = MotorGetCurrent(ParseInt16(rxdata+6));
  // motor_state[index].speed = (float)ParseInt16(rxdata+8) / 180.0f * M_PI;
  motor_state[index].speed = encoder_delta / (float)(HAL_GetTick()-motor_state[index].tick) * 1000.0f / MOTOR_ENCODER_MAX * M_PI * 2.0f;
  motor_state[index].encoder = ParseUInt16(rxdata+10);
  motor_state[index].position = (float)motor_state[index].encoder / 16383.0f * M_PI * 2.0f;
  motor_state[index].tick = HAL_GetTick();
}

void SetPTZPosition(int64_t pose_i) {
  uint8_t data[20] = {0};
  uint8_t rxdata[20] = {0};

  data[0] = MOTOR_CMD_HEADER;
  data[1] = MOTOR_CMD_POSITION_LOOP2;
  data[2] = MOTOR_PTZ;
  data[3] = 0x0C;
  data[4] = CheckSum(data, 4);
  memcpy(data+5, &pose_i, 8);

  uint32_t temp = PTZ_SPEED;
  memcpy(data+13, &temp, 4);
  data[17] = CheckSum(data+5, 12);
  HAL_UART_Transmit(&huart3, data, 18, TX_TIMEOUT);
  // HAL_Delay(1);
  // HAL_UART_Transmit_DMA(&huart3, data, 10);
  HAL_UART_Receive_DMA(&huart3, rxdata, 13);
  HAL_Delay(1);

  if(rxdata[2] != MOTOR_PTZ) {
    return;
  }

  ptz_state.encoder = ParseUInt16(rxdata+10);
  ptz_state.position = (float)ptz_state.encoder / 16383.0f * 2.0f * M_PI;
  ptz_state.position = MapFloat(ptz_state.position, M_PI*2.0f, M_PI/3.0f*2.0f, -3.2724924f, 0.9162979f);
  // printf("PTZ: offset %d, encoder %d, position %f\r\n", ptz_encoder_offset, ptz_state.encoder, ptz_state.position);
}

void SetPTZPositionRad(float pos_rad) {
  int64_t pose_i;

  pos_rad = MapFloat(pos_rad, 3.2724924f, -0.9162979f, 0, -4.1887903f);
  pose_i = (int64_t)(pos_rad / M_PI / 2.0 * 360.0 * 100.0);
  SetPTZPosition(pose_i);
}

void UpdateMotorPosition(uint8_t index) {
  uint8_t data[20] = {0};
  uint8_t rxdata[20] = {0};

  data[0] = MOTOR_CMD_HEADER;
  data[1] = kGyemsCommandGetState2;
  data[2] = index;
  data[3] = 0x00;
  data[4] = CheckSum(data, 4);

  HAL_UART_Transmit(&huart3, data, 5, TX_TIMEOUT);
//  HAL_UART_Transmit_DMA(&huart3, data, 10);

  HAL_UART_Receive_DMA(&huart3, rxdata, 13);
  HAL_Delay(1);

  /* detect suspiciously large readings, possibly from encoder roll over */
  float encoder_delta = (float)ParseUInt16(rxdata+10) - (float)motor_state[index].encoder;
  if(encoder_delta > 10000.0f) {
    encoder_delta = encoder_delta - MOTOR_ENCODER_MAX;
  } else if (encoder_delta < -10000.0f) {
    encoder_delta = encoder_delta + MOTOR_ENCODER_MAX;
  }

  motor_state[index].temperature = rxdata[5];
  motor_state[index].current = MotorGetCurrent(ParseInt16(rxdata+6));
  // motor_state[index].speed = (float)ParseInt16(rxdata+8) / 180.0f * M_PI;
  motor_state[index].speed = encoder_delta / (float)(HAL_GetTick()-motor_state[index].tick) * 1000.0f / MOTOR_ENCODER_MAX * M_PI * 2.0f;
  motor_state[index].encoder = ParseUInt16(rxdata+10);
  motor_state[index].position = (float)motor_state[index].encoder / 16383.0f * M_PI * 2.0f;
  motor_state[index].tick = HAL_GetTick();
}

void UpdatePTZPosition() {
  uint8_t data[20] = {0};
  uint8_t rxdata[20] = {0};

  data[0] = MOTOR_CMD_HEADER;
  data[1] = kGyemsCommandGetState2;
  data[2] = MOTOR_PTZ;
  data[3] = 0x00;
  data[4] = CheckSum(data, 4);

  HAL_UART_Transmit(&huart3, data, 5, TX_TIMEOUT);
//  HAL_UART_Transmit_DMA(&huart3, data, 10);

  HAL_UART_Receive_DMA(&huart3, rxdata, 13);
  HAL_Delay(1);

  ptz_state.encoder = ParseUInt16(rxdata+10);
  ptz_state.position = (float)ptz_state.encoder / 16383.0f * 2.0f * M_PI;
  ptz_state.position = MapFloat(ptz_state.position, M_PI*2.0f, M_PI/3.0f*2.0f, -3.2724924f, 0.9162979f);
  // printf("PTZ: offset %d, encoder %d, position %f\r\n", ptz_encoder_offset, ptz_state.encoder, ptz_state.position);
}

void MotorCheck() {
  if(motor_enable != motor_enable_cmd) {
    if(motor_enable_cmd == 0) {
      DisableMotor(MOTOR_FL);
      DisableMotor(MOTOR_FR);
      DisableMotor(MOTOR_BL);
      DisableMotor(MOTOR_BR);
    }
    motor_enable = motor_enable_cmd;
  } else {
    if(motor_enable == 1) {
      SetMotorSpeedRad(MOTOR_FL, motor_state[MOTOR_FL].speed_cmd);
      SetMotorSpeedRad(MOTOR_FR, motor_state[MOTOR_FR].speed_cmd);
      SetMotorSpeedRad(MOTOR_BR, motor_state[MOTOR_BR].speed_cmd);
      SetMotorSpeedRad(MOTOR_BL, motor_state[MOTOR_BL].speed_cmd);
    } else {
      UpdateMotorPosition(MOTOR_FL);
      UpdateMotorPosition(MOTOR_FR);
      UpdateMotorPosition(MOTOR_BR);
      UpdateMotorPosition(MOTOR_BL);
    }
  }

  if(ptz_enable != ptz_enable_cmd) {
    if(ptz_enable_cmd == 0) {
      DisableMotor(MOTOR_PTZ);
    }
    ptz_enable = ptz_enable_cmd;
  } else {
    if(ptz_enable == 1) {
      SetPTZPositionRad(ptz_state.position_cmd);
    } else {
      UpdatePTZPosition();
    }
  }

  // printf("Speed: %7f, %7f, %7f, %7f\r\n", motor_state[1].speed, motor_state[2].speed, motor_state[3].speed, motor_state[4].speed);
  // printf("Encoder: %05d, %05d, %05d, %05d\r\n", motor_state[1].encoder, motor_state[2].encoder, motor_state[3].encoder, motor_state[4].encoder);
  // printf("Position: %7f, %7f, %7f, %7f\r\n", motor_state[1].position, motor_state[2].position, motor_state[3].position, motor_state[4].position);
  // printf("PTZ position: %7f\r\n", ptz_state.position);
}

void MotorStopAll() {
  // StopMotor(MOTOR_FL);
  // StopMotor(MOTOR_FR);
  // StopMotor(MOTOR_BL);
  // StopMotor(MOTOR_BR);

  SetMotorSpeed(MOTOR_FL, 0);
  SetMotorSpeed(MOTOR_FR, 0);
  SetMotorSpeed(MOTOR_BL, 0);
  SetMotorSpeed(MOTOR_BR, 0);
}

void MotorSetPTZOffset() {
  UpdatePTZPosition();
  ptz_encoder_offset = 7200 - ptz_state.encoder;
  ptz_angle_offset = 360.0f * ((float)ptz_encoder_offset/16384.0f);
}

void SetPTZZero() {
  SetMotorZero(MOTOR_PTZ);
}

void MotorShake() {
  SetMotorSpeed(MOTOR_FL, 12000);
  SetMotorSpeed(MOTOR_FR, 12000);
  SetMotorSpeed(MOTOR_BR, 12000);
  SetMotorSpeed(MOTOR_BL, 12000);
  HAL_Delay(1000);
  SetMotorSpeed(MOTOR_FL, -12000);
  SetMotorSpeed(MOTOR_FR, -12000);
  SetMotorSpeed(MOTOR_BR, -12000);
  SetMotorSpeed(MOTOR_BL, -12000);
  HAL_Delay(2000);
  SetMotorSpeed(MOTOR_FL, 12000);
  SetMotorSpeed(MOTOR_FR, 12000);
  SetMotorSpeed(MOTOR_BR, 12000);
  SetMotorSpeed(MOTOR_BL, 12000);
  HAL_Delay(1000);
  MotorStopAll();
}

void MotorSetup() {
  motor_enable = 1;
  motor_enable_cmd = 1;

  ptz_enable = 1;
  ptz_enable_cmd = 1;
  ptz_enable_cmd = 1;
  ptz_encoder_offset = 0;

  DisableMotor(MOTOR_FL);
  DisableMotor(MOTOR_FR);
  DisableMotor(MOTOR_BR);
  DisableMotor(MOTOR_BL);
}
