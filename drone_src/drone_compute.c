/*
 * drone_compute.c
 *
 *  Created on: 2018. 10. 12.
 *      Author: JuYeong
 */

#include "drone_compute.h"

extern volatile uint32_t time;
extern volatile uint32_t time1;
extern int16_data acc_calibrate;
extern UART_HandleTypeDef huart3;

uint8_t str[200];
uint8_t temp;
static float_data rpy={0};

#ifdef regacy
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

static float_data EstG={0};
static int32_data EstG32={0};

void compute_attitude(float_data* rpy,int16_data* acc,int16_data* gyro,int16_data* mag)
{
  static int32_data accLPF32={0};
  float_data deltaGyroAngle;
  int32_t accMag = 0;
  float scale;
  static int32_data accsmooth={0};
  uint32_t currentT=time;

  scale = (float)currentT * TIME_SCALE_US * GYRO_SCALE; // GYRO_SCALE unit: radian/microsecond
  time = 0;

  // Initialization
  for (int i = 0; i < 3; i++) {
    deltaGyroAngle.array.data[i] = gyro->array.data[i] * scale; // radian

    accLPF32.array.data[i]   -= accLPF32.array.data[i]>>ACC_LPF_FACTOR;
    accLPF32.array.data[i]   += acc->array.data[i];
    accsmooth.array.data[i]   = accLPF32.array.data[i]>>ACC_LPF_FACTOR;
  }
  rotate_vector(&EstG,&deltaGyroAngle);

  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro

  for (int i = 0; i < 3; i++) {
    EstG.array.data[i] = EstG.array.data[i] *RPY_COMF_FILTER_FACTOR + accsmooth.array.data[i] * (1-RPY_COMF_FILTER_FACTOR);
    EstG32.array.data[i] = EstG.array.data[i];
  }
//  for(int i=0; i<3; i++){
//    sprintf((char*)str,"EstG-%d : %f\n",i+1,EstG.array.data[i]);
//    HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),5);
//  }
  int32_t sqGX_sqGZ = EstG32.xyz.x*EstG32.xyz.x + EstG32.xyz.z*EstG32.xyz.z;
//  float sqGX_sqGZ_f = sqrt(EstG.xyz.x*EstG.xyz.x + EstG.xyz.z*EstG.xyz.z);
  rpy->array.data[ROLL]  = _atan2(EstG32.xyz.x , EstG32.xyz.z);
  rpy->array.data[PITCH] = _atan2(EstG32.xyz.y , InvSqrt(sqGX_sqGZ)*sqGX_sqGZ);
//  rpy->array.data[ROLL]  = atan2(EstG.xyz.x , EstG.xyz.z)*180/PI;
//  rpy->array.data[PITCH] = atan2(EstG.xyz.y , sqGX_sqGZ_f)*180/PI;

}

void compute_IMU(float_data* rpy) {
  static int16_data acc,gyro,mag;
  static int16_t gyroADCprevious[3] = {0,0,0};
  int16_t gyroADCp[3];
  int16_t gyroADCinter[3];
  static uint32_t timeInterleave = 0;

  mpu_getacc(&acc);
  compute_attitude(rpy,&acc,&gyro,&mag);
  mpu_getgyro(&gyro);

  for (int i = 0; i < 3; i++)
    gyroADCp[i] =  gyro.array.data[i];
//  timeInterleave = time*10;
//  while((uint16_t)(time*10-timeInterleave)<650)

  mpu_getgyro(&gyro);
  for (int i = 0; i < 3; i++) {
  gyroADCinter[i] =  gyro.array.data[i]+gyroADCp[i];
  // empirical, we take a weighted value of the current and the previous values
  gyro.array.data[i] = (gyroADCinter[i]+gyroADCprevious[i])/3;
  gyroADCprevious[i] = gyroADCinter[i]>>1;
  }
}

void droneRPY_print()
{
  uint32_t dt=time1/100;
  float_data rpy={0};
  static uint32_t count=0;
  static toggle = 0;


  compute_IMU(&rpy);
  count++;
  if(dt >= 10){
//      sprintf((char*)str,"ROLL : %f\n",rpy.array.data[ROLL]);
//      HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),5);
//      sprintf((char*)str,"PITCH : %f\n",rpy.array.data[PITCH]);
//      HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),5);

//        sprintf((char*)str,"%f,%f\n",rpy.array.data[ROLL],rpy.array.data[PITCH]);
//        HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),5);
    sprintf((char*)str,"%f,%f\n",rpy.xyz.x,rpy.xyz.y);
    HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),5);
//    temp = i2c_read_byte(BMP280_I2C_ADDRESS, BMP280_CHIP_ID_REG);
//    sprintf((char*)str,"LOOPTIME : %lu us  BMP280_ID : %d\n",dt,temp);
//    HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),5);
    time1=0;
    count=0;
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, toggle);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, toggle);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, toggle);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, toggle);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, !toggle);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, !toggle);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, !toggle);
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, !toggle);

    toggle = !toggle;
  }
}
#else
void compute_rpy(float_data* RPY)
{
  float dt=time*TIME_SCALE_S;
  int16_data acc={0},gyro={0},mag={0};
  float_data delta_gyro,degree_acc;
  time = 0;
  mpu_getacc(&acc);
  mpu_getgyro(&gyro);

  delta_gyro.array.data[ROLL]  = gyro.array.data[ROLL]*dt*GYRO_SCALE;
  delta_gyro.array.data[PITCH] = gyro.array.data[PITCH]*dt*GYRO_SCALE;
  delta_gyro.array.data[YAW]   = gyro.array.data[YAW]*dt*GYRO_SCALE;

  float XZ_plane = sqrt(acc.array.data[ROLL]*acc.array.data[ROLL]+acc.array.data[YAW]*acc.array.data[YAW]);
  float YZ_plane = sqrt(acc.array.data[PITCH]*acc.array.data[PITCH]+acc.array.data[YAW]*acc.array.data[YAW]);
  degree_acc.array.data[ROLL]  = atan2(acc.array.data[ROLL],YZ_plane)*ACC_SCALE;
  degree_acc.array.data[PITCH] = atan2(acc.array.data[PITCH],XZ_plane)*ACC_SCALE;
  degree_acc.array.data[YAW]   = 0;

//  for(int i=0; i<3; i++){
//    sprintf((char*)str,"acc-%d : %d\n",i+1,acc.array.data[i]);
//    HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),5);
//  }
//  for(int i=0; i<3; i++){
//    sprintf((char*)str,"degree-%d : %f\n",i+1,degree_acc.array.data[i]);
//    HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),5);
//  }
//  for(int i=0; i<3; i++){
//    sprintf((char*)str,"acc_calibrate-%d : %d\n",i+1,acc_calibrate.array.data[i]);
//    HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),5);
//  }
//  HAL_Delay(500);

//  RPY->array.data[ROLL]  = (RPY->array.data[ROLL]  + delta_gyro.array.data[ROLL]) *RPY_CMPF_FACTOR + degree_acc.array.data[ROLL] *(1-RPY_CMPF_FACTOR);
//  RPY->array.data[PITCH] = (RPY->array.data[PITCH] + delta_gyro.array.data[PITCH])*RPY_CMPF_FACTOR + degree_acc.array.data[PITCH]*(1-RPY_CMPF_FACTOR);
//  RPY->array.data[YAW]   = (RPY->array.data[YAW]   + delta_gyro.array.data[YAW])  *RPY_CMPF_FACTOR + degree_acc.array.data[YAW]  *(1-RPY_CMPF_FACTOR);

  for(int i=0; i<3; i++)
    RPY->array.data[i] = (RPY->array.data[i] + delta_gyro.array.data[i])*RPY_CMPF_FACTOR + degree_acc.array.data[i]*(1-RPY_CMPF_FACTOR);
}

void droneRPY_print()
{
  uint32_t dt=time1*10;

  compute_rpy(&rpy);
  if(dt > 900000){
    for(int i=0; i<3; i++){
      sprintf((char*)str,"rpy-%d : %f\n",i+1,rpy.array.data[i]);
      HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),5);
    }
    temp = i2c_read_byte(BMP280_I2C_ADDRESS, BMP280_CHIP_ID_REG);
    sprintf((char*)str,"LOOPTIME : %lu us  BMP280_ID : %d\n",dt,temp);
    HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),5);
    time1=0;
  }
}
#endif

















