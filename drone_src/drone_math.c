/*
 * drone_math.c
 *
 *  Created on: 2018. 10. 12.
 *      Author: JuYeong
 */

#include "drone_math.h"

int32_t abs(int32_t x)
{
  if(x<0)
    return -x;
  else
    return x;
}

int16_t _atan2(int32_t y, int32_t x)
{
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) ){
     a = 573 * z / (1.0f + 0.28f * z * z);
   if (x<0) {
     if (y<0) a -= 1800;
     else a += 1800;
   }
  } else {
   a = 900 - 573 * z / (z * z + 0.28f);
   if (y<0) a -= 1800;
  }
  return a;
}

float InvSqrt (float x)
{
  union{
    int32_t i;
    float  f;
  } conv;
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

void rotate_vector(float_data* vector, float_data* gyro_delta)
{
  float_data v_tmp = *vector;
  vector->xyz.x += gyro_delta->xyz.x * v_tmp.xyz.z + gyro_delta->xyz.z * v_tmp.xyz.y;
  vector->xyz.y += gyro_delta->xyz.y * v_tmp.xyz.z + gyro_delta->xyz.z * v_tmp.xyz.x;
  vector->xyz.z -= gyro_delta->xyz.x * v_tmp.xyz.x + gyro_delta->xyz.y * v_tmp.xyz.y;
}

