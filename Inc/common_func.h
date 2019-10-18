#ifndef COMMON_FUNC_H_
#define COMMON_FUNC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <math.h>
#include <stdint.h>

uint8_t CheckSum(uint8_t *buf, uint32_t length);
void PrintHex(char header[], uint8_t *data, uint32_t length);
float MapFloat(float value, float c_min, float c_max, float r_min, float r_max);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_FUNC_H_ */
