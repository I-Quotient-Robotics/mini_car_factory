#ifndef ROBOT_STATE_H_
#define ROBOT_STATE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"

enum MiniCarMotorID {
  kMiniCarMotorFL = 0x01,
  kMiniCarMotorFR = 0x02,
  kMiniCarMotorBR = 0x03,
  kMiniCarMotorBL = 0x04,
  kMiniCarMotorPTZ = 0x05,
};

int8_t wake_up_call;

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_STATE_H_ */
