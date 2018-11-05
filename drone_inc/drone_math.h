/*
 * drone_math.h
 *
 *  Created on: 2018. 10. 12.
 *      Author: JuYeong
 */

#ifndef DRONE_MATH_H_
#define DRONE_MATH_H_

#include "stm32f1xx_hal.h"
#include "struct_type.h"
#include <math.h>

int32_t abs(int32_t x);
int16_t _atan2(int32_t y, int32_t x);
float InvSqrt (float x);
void rotate_vector(float_data* vector, float_data* gyro_delta);

#endif /* DRONE_MATH_H_ */
