#include "common_func.h"

#include "stdio.h"

uint8_t CheckSum(uint8_t *buf, uint32_t length) {
  uint8_t check_sum = 0;
  for(uint32_t i=0; i<length; i++) {
    check_sum += buf[i];
  }
  return check_sum;
}

void PrintHex(char header[], uint8_t *data, uint32_t length) {
  printf(header);
  for(uint32_t i=0; i<length; i++) {
    printf("%02X, ", data[i]);
  }
  printf("\r\n");
}

float MapFloat(float value, float c_min, float c_max, float r_min, float r_max) {
  return ((value - c_min) / (c_max - c_min)) * (r_max - r_min) + r_min;
}
