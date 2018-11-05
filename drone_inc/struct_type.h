/*
 * struct_type.h
 *
 *  Created on: 2018. 10. 10.
 *      Author: JuYeong
 */

#ifndef STRUCT_TYPE_H_
#define STRUCT_TYPE_H_
#include "stm32f1xx_hal.h"

enum{
  ROLL,PITCH,YAW,ALTITUDE
};
typedef struct _Raw_data
{
  uint8_t data[8];
}Raw_data;

typedef union _XYZ_INT16
{
    struct{
      int16_t x;
      int16_t y;
      int16_t z;
    }xyz;
    struct{
        int16_t data[3];
    }array;
}int16_data;

typedef union _XYZ_INT32
{
    struct{
      int32_t x;
      int32_t y;
      int32_t z;
    }xyz;
    struct{
        int32_t data[3];
    }array;
}int32_data;

typedef union _XYZ_INT64
{
    struct{
      int64_t x;
      int64_t y;
      int64_t z;
    }xyz;
    struct{
        int64_t data[3];
    }array;
}int64_data;

typedef union _XYZ_FLOAT
{
    struct{
      float x;
      float y;
      float z;
    }xyz;
    struct{
        float data[3];
    }array;
}float_data;



#endif /* STRUCT_TYPE_H_ */
