#ifndef LED_CONTROL_H_
#define LED_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "common_func.h"

#define LED_HB_FAST_PERIOD            400
#define LED_HB_NORMAL_PERIOD          1000
#define LED_HB_STANDBY_PERIOD         3000

#define LED_HB_FAST                   200
#define LED_HB_NORMAL                 200
#define LED_HB_STANDBY                200

#define LED_SEQUENCE_THRES            100
#define LED_SEQUENCE_HOLD             3000

uint16_t LEDGetBytes();

void LEDSet(uint8_t index);
void LEDUnset(uint8_t index);
void LEDToggle(uint8_t index);

void LEDSetAll();
void LEDUnsetAll();
void LEDToggleAll();

void LEDStartupSEQ(uint8_t number);
void LEDStandbySEQ();

void LEDSetup();
void LEDCheck(int8_t mode);
void LEDHartbeatCheck(uint8_t mode);

enum LEDMode {
  kLEDManual = 0x01,
  kLEDBattery,
  kLEDNormal,
  kLEDCharging,
  kLEDLowBattery,
  kLEDWarning
};
enum LEDMode led_mode, last_led_mode;

enum HeartBeatLEDMode {
  kHBStandBy = 0x01,
  kHBNormal,
  kHBWarning,
  kHBLowBattery
};
enum HeartBeatLEDMode heart_beat_led_mode;

struct UINT8Interface {
  uint8_t state;
  uint8_t command;
};

struct LEDState {
  uint8_t state;
  uint8_t command;
};

struct LEDState led_state[10];
struct UINT8Interface led_control;

uint32_t led_command_timestamp;

#ifdef __cplusplus
}
#endif

#endif /* LED_CONTROL_H_ */
