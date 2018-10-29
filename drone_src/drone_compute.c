/*
 * drone_compute.c
 *
 *  Created on: 2018. 10. 12.
 *      Author: JuYeong
 */

#include "drone_compute.h"

extern volatile uint32_t time;

/* ���ڱ� ���� �����͸� �޾� ����Ƽ��Ƽ�� �����͸� �����ϴ� �Լ�
 * ����Ƽ��Ƽ�� �̸� �Լ��� ���Ͽ� �޾ƿ;��Ѵ�.
 * */
void compute_mag_revise(const int16_data* mag16,const int16_data* sensitivity,int32_data* mag32)
{
  for(int i=0; i<3; i++){
    mag32->array.data[i] = mag16->array.data[i]*((float)(sensitivity->array.data[i]-128)*0.5/128+1) ;
  }
}
/* ���̷� raw�����͸� ù��° ������ �޾� �ι�° ������ ��ȯ�ؼ� �־��ִ� �Լ�
 * ������ degree/s
 * */
void compute_trans_gyro(const int16_data* gyro_raw,int32_data* gyro)
{
  for(int i=0;i<3;i++){
    gyro->array.data[i] = gyro_raw->array.data[i]/MPU_GYRO_CONFIG_FS_DIV;
  }
}
/* ���ӵ� raw�����͸� ù��° ������ �޾� �ι�° ������ ��ȯ�ؼ��־��ִ� �Լ�
 * ������ �߷°��ӵ�(g)
 * */
void compute_trans_acc(const int16_data* acc_raw,int32_data* acc)
{
  for(int i=0;i<3;i++){
    acc->array.data[i] = acc_raw->array.data[i]/MPU_ACC_CONFIG_FS_DIV;
  }
}


void compute_attitude(float_data* rpy)
{
  int16_data acc,gyro,mag;
  static int16_data accLPF32={0};
  int32_data baro_raw;
  float_data deltaGyroAngle;
  float scale;
  static int32_data accsmooth={0};
  static uint16_t previousT;
  uint16_t currentT=time;

  scale = (currentT - previousT) * GYRO_SCALE; // GYRO_SCALE unit: radian/microsecond
  previousT = currentT;

  // Initialization
  for (int i = 0; i < 3; i++) {
    deltaGyroAngle[i] = gyro.array.data[i] * scale; // radian

    accLPF32.array.data[i]   -= accLPF32.array.data[i]>>ACC_LPF_FACTOR;
    accLPF32.array.data[i]   += acc[i];
    accsmooth.array.data[i]   = accLPF32.array.data[i]>>ACC_LPF_FACTOR;

    accMag += (int32_t)imu.accSmooth[axis]*imu.accSmooth[axis] ;
  }
  rotateV(&EstG.V,deltaGyroAngle);


    accMag = accMag*100/((int32_t)ACC_1G*ACC_1G);
    validAcc = 72 < (uint16_t)accMag && (uint16_t)accMag < 133;
    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    for (axis = 0; axis < 3; axis++) {
      if ( validAcc )
        EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + imu.accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
       EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float
    }

    if ((int16_t)EstG32.A[2] > ACCZ_25deg)
      f.SMALL_ANGLES_25 = 1;
    else
      f.SMALL_ANGLES_25 = 0;

    // Attitude of the estimated vector
    int32_t sqGX_sqGZ = sq(EstG32.V.X) + sq(EstG32.V.Z);
    invG = InvSqrt(sqGX_sqGZ + sq(EstG32.V.Y));
    att.angle[ROLL]  = _atan2(EstG32.V.X , EstG32.V.Z);
    att.angle[PITCH] = _atan2(EstG32.V.Y , InvSqrt(sqGX_sqGZ)*sqGX_sqGZ);

}




























