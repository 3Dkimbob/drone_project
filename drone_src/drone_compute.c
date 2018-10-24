/*
 * drone_compute.c
 *
 *  Created on: 2018. 10. 12.
 *      Author: JuYeong
 */

#include "drone_compute.h"

extern volatile uint32_t time;

/* 지자기 센서 데이터를 받아 센서티비티로 데이터를 보정하는 함수
 * 센서티비티는 미리 함수를 콜하여 받아와야한다.
 * */
void compute_mag_revise(const int16_data* mag16,const int16_data* sensitivity,int32_data* mag32)
{
  for(int i=0; i<3; i++){
    mag32->array.data[i] = mag16->array.data[i]*((float)(sensitivity->array.data[i]-128)*0.5/128+1) ;
  }
}
/* 자이로 raw데이터를 첫번째 변수로 받아 두번째 변수에 변환해서 넣어주는 함수
 * 단위는 degree/s
 * */
void compute_trans_gyro(const int16_data* gyro_raw,int32_data* gyro)
{
  for(int i=0;i<3;i++){
    gyro->array.data[i] = gyro_raw->array.data[i]/MPU_GYRO_CONFIG_FS_DIV;
  }
}
/* 가속도 raw데이터를 첫번째 변수로 받아 두번째 변수에 변환해서넣어주는 함수
 * 단위는 중력가속도(g)
 * */
void compute_trans_acc(const int16_data* acc_raw,int32_data* acc)
{
  for(int i=0;i<3;i++){
    acc->array.data[i] = acc_raw->array.data[i]/MPU_ACC_CONFIG_FS_DIV;
  }
}


void compute_attitude(float_data* rpy)
{
  int16_data acc_raw,gyro_raw,mag_raw;
  int32_data baro_raw;
  static int32_data accsmooth={0};
  static uint16_t TIME;



}




























