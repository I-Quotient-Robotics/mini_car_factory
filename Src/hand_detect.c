#include "hand_detect.h"

void HandDetectSetup(I2C_HandleTypeDef* I2cPtr) {
  hand_detect_flag = 0;

  vl53l0x_dev.I2cPtr=I2cPtr;
  vl53l0x_dev.I2cDevAddr=0x52;

  VL53L0X_Error Status=VL53L0X_ERROR_NONE;

  static uint32_t refSpadCount = 0;
  static uint8_t isApertureSpads = 0;
  static uint8_t VhvSettings = 0;
  static uint8_t PhaseCal = 0;

//  int8_t result = HAL_I2C_IsDeviceReady(vl53l0x_dev.I2cPtr, vl53l0x_dev.I2cDevAddr, 2, 2);
//  printf("VL53L0X online: %d\r\n", result);

  /* set sensor to standby mode and restart */
  //HAL_GPIO_WritePin(VL530LX_XSHUT_GPIO_Port, VL530LX_XSHUT_Pin, GPIO_PIN_RESET);
  //HAL_Delay(100);
  HAL_GPIO_WritePin(VL530LX_XSHUT_GPIO_Port, VL530LX_XSHUT_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

//  result = HAL_I2C_IsDeviceReady(vl53l0x_dev.I2cPtr, 0x12, 2, 2);
//  printf("VL53L0X online: %d\r\n", result);

  if (Status == VL53L0X_ERROR_NONE)
  {
      Status=VL53L0X_SetDeviceAddress( &vl53l0x_dev, 0x52 );
      vl53l0x_dev.I2cDevAddr=0x52;
  }

  if (Status == VL53L0X_ERROR_NONE)
  {
      Status=VL53L0X_DataInit( &vl53l0x_dev );
  }

  printf("error %d\r\n", Status);

  if (Status == VL53L0X_ERROR_NONE)
  {
      Status=VL53L0X_StaticInit( &vl53l0x_dev );
  }

  printf("error %d\r\n", Status);

  if (Status == VL53L0X_ERROR_NONE)
  {
      Status = VL53L0X_PerformRefSpadManagement( &vl53l0x_dev, &refSpadCount, &isApertureSpads);
  }

  printf("error %d\r\n", Status);
  if (Status == VL53L0X_ERROR_NONE)
  {
      Status = VL53L0X_PerformRefCalibration( &vl53l0x_dev, &VhvSettings, &PhaseCal);
  }
  printf("error %d\r\n", Status);
  if (Status == VL53L0X_ERROR_NONE)
  {
      Status=VL53L0X_SetReferenceSpads( &vl53l0x_dev, refSpadCount, isApertureSpads);
  }
  printf("error %d\r\n", Status);
  if (Status == VL53L0X_ERROR_NONE)
  {
      Status=VL53L0X_SetRefCalibration( &vl53l0x_dev, VhvSettings, PhaseCal);
  }
  printf("error %d\r\n", Status);
  if (Status == VL53L0X_ERROR_NONE)
  {
      Status=VL53L0X_SetDeviceMode( &vl53l0x_dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  }
  printf("error %d\r\n", Status);
  if (Status == VL53L0X_ERROR_NONE)
  {
//      Status = VL53L0X_SetLimitCheckValue( &vl53l0x_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536) );
  }
  printf("error %d\r\n", Status);
  if (Status == VL53L0X_ERROR_NONE)
  {
//      Status = VL53L0X_SetLimitCheckValue( &vl53l0x_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32*65536) );
  }
  printf("error %d\r\n", Status);
  if (Status == VL53L0X_ERROR_NONE)
  {
//      Status =VL53L0X_SetMeasurementTimingBudgetMicroSeconds( &vl53l0x_dev, 20000 );
  }
  printf("error %d\r\n", Status);
  if (Status == VL53L0X_ERROR_NONE)
  {
      Status=VL53L0X_StartMeasurement( &vl53l0x_dev );
  }

  HAL_Delay(100);
  printf("error %d\r\n", Status);
}

void initSensor( I2C_HandleTypeDef* I2cPtr )
{

}

uint8_t HandDetectCheck() {
  static uint8_t counter=0;
//  char device_error[35],
  char range_error[35];

  VL53L0X_RangingMeasurementData_t data;
//  VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev, &data);
//  HAL_Delay(500);
  VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &data);
  VL53L0X_GetRangeStatusString(data.RangeStatus, range_error);

//  VL53L0X_DeviceModes mode;
//  VL53L0X_GetDeviceMode (&vl53l0x_dev, &mode);

//  VL53L0X_DeviceError error;
//  VL53L0X_GetDeviceErrorStatus(&vl53l0x_dev, &error);
//  VL53L0X_GetDeviceErrorString(error, device_error);

//  printf("Hand Detect: value %d, range_error %s\r\n", data.RangeMilliMeter, range_error);
//  printf("Hand Detect: value %d, mode %d, range_error %s, device_error %s\r\n", data.RangeMilliMeter, mode, range_error, device_error);

  if((data.RangeMilliMeter<HAND_DETECT_THRES) && (data.RangeMilliMeter>0)) {
    counter += 1;
  } else {
    counter = 0;
  }

  if(counter > 2) {
    counter = 0;
    hand_detect_flag = 1;
  } else {
    hand_detect_flag = 0;
  }
  printf("Hand Detect: value %d, hand_detect %d\r\n", data.RangeMilliMeter, hand_detect_flag);
//  int8_t result = HAL_I2C_IsDeviceReady(vl53l0x_dev.I2cPtr, vl53l0x_dev.I2cDevAddr, 2, 2);
//  printf("VL53L0X online: %d\r\n", result);

  return hand_detect_flag;
}
