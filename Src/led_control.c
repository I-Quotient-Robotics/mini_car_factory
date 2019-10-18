#include "led_control.h"
#include "stm32f1xx_hal.h"

#include "main.h"

#include "bms.h"

extern IWDG_HandleTypeDef hiwdg;

uint16_t LEDGetBytes() {
  uint16_t temp;

  for(uint8_t i=0; i<10; i++) {
    temp += (2^i) * led_state[i].command;
  }

  return temp;
}

void LEDSet(uint8_t index) {
  switch (index) {
    case 0:
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
      break;
    case 1:
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      break;
    case 2:
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
      break;
    case 3:
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
      break;
    case 4:
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
      break;
    case 5:
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
      break;
    case 6:
      HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
      break;
    case 7:
      HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
      break;
    case 8:
      HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
      break;
    case 9:
      HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, GPIO_PIN_SET);
      break;
    case 10:
      HAL_GPIO_WritePin(TLED_GPIO_Port, TLED_Pin, GPIO_PIN_SET);
  }
}

void LEDToggle(uint8_t index) {
  switch (index) {
    case 0:
      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
      break;
    case 1:
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      break;
    case 2:
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      break;
    case 3:
      HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
      break;
    case 4:
      HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
      break;
    case 5:
      HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
      break;
    case 6:
      HAL_GPIO_TogglePin(LED6_GPIO_Port, LED6_Pin);
      break;
    case 7:
      HAL_GPIO_TogglePin(LED7_GPIO_Port, LED7_Pin);
      break;
    case 8:
      HAL_GPIO_TogglePin(LED8_GPIO_Port, LED8_Pin);
      break;
    case 9:
      HAL_GPIO_TogglePin(LED9_GPIO_Port, LED9_Pin);
      break;
    case 10:
      HAL_GPIO_TogglePin(TLED_GPIO_Port, TLED_Pin);
  }
}

void LEDUnset(uint8_t index) {
  switch (index) {
    case 0:
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
      break;
    case 1:
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      break;
    case 2:
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      break;
    case 3:
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
      break;
    case 4:
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
      break;
    case 5:
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
      break;
    case 6:
      HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
      break;
    case 7:
      HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
      break;
    case 8:
      HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
      break;
    case 9:
      HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, GPIO_PIN_RESET);
      break;
    case 10:
      HAL_GPIO_WritePin(TLED_GPIO_Port, TLED_Pin, GPIO_PIN_RESET);
  }
}

void LEDSetAll() {
  for(uint8_t i=0; i<10; i++) {
    LEDSet(i);
  }
}

void LEDUnsetAll() {
  for(uint8_t i=0; i<10; i++) {
    LEDUnset(i);
  }
}

void LEDToggleAll() {
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
  HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
  HAL_GPIO_TogglePin(LED6_GPIO_Port, LED6_Pin);
  HAL_GPIO_TogglePin(LED7_GPIO_Port, LED7_Pin);
  HAL_GPIO_TogglePin(LED8_GPIO_Port, LED8_Pin);
  HAL_GPIO_TogglePin(LED9_GPIO_Port, LED9_Pin);
}

void LEDStartupSEQ(uint8_t number) {
  // uint32_t seq_start_tick = HAL_GetTick();
  if(number > 10) number = 10;

  LEDUnsetAll();
  HAL_Delay(100);

  for(uint8_t i=0; i<number; i++) {
    LEDSet(i);
    HAL_Delay(LED_SEQUENCE_THRES);
  }

  HAL_Delay(LED_SEQUENCE_HOLD);
//  while(HAL_GetTick()-seq_start_tick<3000) {
//    // HAL_IWDG_Refresh(&hiwdg);
//    HAL_Delay(100);
//  }
  LEDUnsetAll();
}

void LEDStandbySEQ() {

}

void LEDHartbeatCheck(uint8_t mode) {
  uint16_t up_time = LED_HB_NORMAL;
  uint16_t period = LED_HB_NORMAL_PERIOD;

  uint8_t value;
  switch(mode) {
    case 0:
      up_time = LED_HB_NORMAL;
      period = LED_HB_NORMAL_PERIOD;
      break;
    case 1:
      up_time = LED_HB_FAST;
      period = LED_HB_FAST_PERIOD;
      break;
    case 2:
      up_time = LED_HB_STANDBY;
      period = LED_HB_STANDBY_PERIOD;
  }

  if((HAL_GetTick() % period) < up_time) {
    LEDSet(10);
  } else {
    LEDUnset(10);
  }
}

void LEDCheck(int8_t mode) {

  /* debug set */
  // bms_rsoc = 4;
  // bms_charging = 1;
  static uint8_t led_up = 0;
  static uint8_t led_total = 0;
  static uint32_t last_tick = 0;

  /* set to normal */
  led_mode = kLEDNormal;

  /* show battery */
  if(mode==1 || last_led_mode==kLEDBattery) {
    led_mode = kLEDBattery;
  }

  /* low battery */
  if(bms_rsoc < 10.0f) {
    led_mode = kLEDLowBattery;
  }

  /* charging */
  if(bms_charging == 1) {
    led_mode = kLEDCharging;
  }

  /* warning */


  /* manual */
  if((HAL_GetTick()-led_command_timestamp) < 5000.0f) {
    led_mode = kLEDManual;
  }

  /* force set */
  // led_mode = kLEDLowBattery;

  switch(led_mode) {
    case kLEDBattery:
      if(last_led_mode != kLEDBattery) {
        led_up = 0;

        // get led count by battery percent
        led_total = bms_rsoc / 10;
        // led_total will be 10 if bms_rsoc=100, so set it to 9 manually
        led_total = led_total>9 ? 9 : led_total;
        last_tick = HAL_GetTick();
        last_led_mode = kLEDBattery;
      } else {
        if(led_up <= led_total) {
          if(HAL_GetTick()-last_tick > LED_SEQUENCE_THRES) {
            LEDSet(led_up);
            led_up += 1;
            last_tick = HAL_GetTick();
          }
          last_led_mode = kLEDBattery;
        } else {
          if(HAL_GetTick()-last_tick > LED_SEQUENCE_HOLD) {
            LEDUnsetAll();
            last_led_mode = kLEDNormal;
          } else {
            last_led_mode = kLEDBattery;
          }
        }
      }
      break;
    case kLEDManual:
      for(uint8_t i=0; i<10; i++) {
        if(led_state[i].command == 1) {
          LEDSet(i);
        } else {
          LEDUnset(i);
        }
      }
      last_led_mode = kLEDManual;
      break;
    case kLEDNormal:
      LEDUnsetAll();
      last_led_mode = kLEDNormal;
      break;
    case kLEDCharging:
      for(uint8_t i=0; i<(bms_rsoc/10); i++) {
        LEDSet(i);
      }

      if(bms_rsoc > 0.01 && bms_rsoc<99.0) {
        /* blink when current>0.01A */
        if((HAL_GetTick() % 500) < 250) {
          LEDSet(bms_rsoc/10);
        } else {
          LEDUnset(bms_rsoc/10);
        }
      } else {
        /* has no current */
        LEDSet(bms_rsoc/10-1);
      }
      last_led_mode = kLEDCharging;
      break;
    case kLEDLowBattery:
      for(uint8_t i=1; i<10; i++) {
        LEDUnset(i);
      }
      if((HAL_GetTick() % 2000) < 1000) {
        LEDSet(0);
      } else {
        LEDUnset(0);
      }
      last_led_mode = kLEDLowBattery;
      break;
    case kLEDWarning:
      last_led_mode = kLEDWarning;
      break;
  }
}

void LEDSetup() {
  led_command_timestamp = 0;
  LEDUnsetAll();
}
