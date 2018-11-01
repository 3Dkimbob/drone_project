/*
 * drone_compute.h
 *
 *  Created on: 2018. 10. 12.
 *      Author: JuYeong
 */

#ifndef DRONE_COMPUTE_H
#define DRONE_COMPUTE_H

#include "stm32f1xx_hal.h"
#include "struct_type.h"
#include "mpu9250.h"
#include "drone_math.h"
#include <math.h>

#define regacy

#ifdef regacy
#define ACC_LPF_FACTOR    4
#define GYR_CMPF_FACTOR 580
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define GYRO_SCALE            (4 / MPU_GYRO_CONFIG_FS_DIV * PI / 180.0 / 1000000.0)
#define ACCZ_25deg            (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)

void compute_mag_revise(const int16_data* mag16,const int16_data* sensitivity,int32_data* mag32);
void compute_trans_gyro(const int16_data* gyro_raw,int32_data* gyro);
void compute_trans_acc(const int16_data* acc_raw,int32_data* acc);
void compute_attitude(float_data* rpy,int16_data* acc,int16_data* gyro,int16_data* mag);
void compute_IMU (float_data* rpy);
void droneRPY_print();

#else

#define ACC_SCALE         (180.0/PI)
#define GYRO_SCALE        (180.0/PI)/MPU_GYRO_CONFIG_FS_DIV
#define RPY_CMPF_FACTOR   0.96f
#define TIME_SCALE_MS     0.01
#define TIME_SCALE_S      0.00001
#define TIME_SCALE_US     10.0

void compute_rpy(float_data* RPY);
void droneRPY_print();

#endif




#endif
