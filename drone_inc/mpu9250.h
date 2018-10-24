/*
 * mpu9250.h
 *
 *  Created on: 2018. 9. 28.
 *      Author: JuYeong
 */

#ifndef MPU9250_H_
#define MPU9250_H_
#include "stm32f1xx_hal.h"
#include "struct_type.h"
#include "I2Cmaster.h"
#include <string.h>

#define PI 3.1415926535
#define Gravity 9.8
#define RAD_TO_DEGREE 180/PI

#define MPU9250_I2C_ADDRESS             0X68
#define MPU9250_WHO_AM_I                0x75
#define MPU9250_ID                      0x71

#define MPU9250_X_ACC_REG               0x3B
#define MPU9250_X_GYRO_REG              0x43
#define MPU9250_TEMP_LOW_REG            0x42

#define MPU9250_POWER_SET_REG           0x6B
#define MPU9250_POWER_SET_REG_RESET     0x80
#define MPU9250_POWER_SET_CLKSET        0x01
#define MPU9250_INT_PIN_CFG_REG         0x37
#define MPU9250_INT_PIN_CFG_BYPASS_EN   0x02
#define MPU9250_USER_CTRL_REG           0x6A
#define MPU9250_USER_CTRL_I2CDISABLE    0x00

#define MPU9250_GYRO_CONFIG_REG       0x1B
#define MPU9250_GYRO_CONFIG_FS_250    (0x00<<3)
#define MPU9250_GYRO_CONFIG_FS_500    (0x01<<3)
#define MPU9250_GYRO_CONFIG_FS_1000   (0x02<<3)
#define MPU9250_GYRO_CONFIG_FS_2000   (0x03<<3)
#define MPU9250_GYRO_CONFIG_FS_X      MPU9250_GYRO_CONFIG_FS_2000

#if MPU9250_GYRO_CONFIG_FS_X==MPU9250_GYRO_CONFIG_FS_250
#define MPU_GYRO_CONFIG_FS_DIV 131
#elif MPU9250_GYRO_CONFIG_FS_X==MPU9250_GYRO_CONFIG_FS_500
#define MPU_GYRO_CONFIG_FS_DIV 65.5
#elif MPU9250_GYRO_CONFIG_FS_X==MPU9250_GYRO_CONFIG_FS_1000
#define MPU_GYRO_CONFIG_FS_DIV 32.8
#elif MPU9250_GYRO_CONFIG_FS_X==MPU9250_GYRO_CONFIG_FS_2000
#define MPU_GYRO_CONFIG_FS_DIV 16.4
#endif

#define MPU9250_ACC_CONFIG_REG      0x1C
#define MPU9250_ACC_CONFIG_FS_2     (0x00<<3)
#define MPU9250_ACC_CONFIG_FS_4     (0x01<<3)
#define MPU9250_ACC_CONFIG_FS_8     (0x02<<3)
#define MPU9250_ACC_CONFIG_FS_16    (0x03<<3)
#define MPU9250_ACC_CONFIG_FS_X     MPU9250_ACC_CONFIG_FS_8

#if MPU9250_ACC_CONFIG_FS_X==MPU9250_ACC_CONFIG_FS_2
#define MPU_ACC_CONFIG_FS_DIV 16384
#elif MPU9250_ACC_CONFIG_FS_X==MPU9250_ACC_CONFIG_FS_4
#define MPU_ACC_CONFIG_FS_DIV 8192
#elif MPU9250_ACC_CONFIG_FS_X==MPU9250_ACC_CONFIG_FS_8
#define MPU_ACC_CONFIG_FS_DIV 2048
#elif MPU9250_ACC_CONFIG_FS_X==MPU9250_ACC_CONFIG_FS_16
#define MPU_ACC_CONFIG_FS_DIV 2048
#endif

#define ACC_1G                (MPU_ACC_CONFIG_FS_DIV>>MPU9250_ACCRAW_DIV)

#define MPU9250_CONFIG_REG          0x1A                    //    /*    ACC   */    /*      GYRO      */
#define MPU9250_CONFIG_DLPF_CFG     MPU9250_CONFIG_DLPF_0   //    Bandwith  Delay   Bandwidth Delay Fs
#define MPU9250_CONFIG_DLPF_0       0                       //      260      0      256   0.98  8
#define MPU9250_CONFIG_DLPF_1       1                       //      184     2.0     188    1.9  1
#define MPU9250_CONFIG_DLPF_2       2                       //       94     3.0      98    2.8  1
#define MPU9250_CONFIG_DLPF_3       3                       //       44     4.9      42    4.8  1
#define MPU9250_CONFIG_DLPF_4       4                       //       21     8.5      20    8.3  1
#define MPU9250_CONFIG_DLPF_5       5                       //       10    13.8      10   13.4  1
#define MPU9250_CONFIG_DLPF_6       6                       //        5    19.0       5   18.6  1
#define MPU9250_CONFIG_DLPF_7       7                       //    RESERVED  RESERVED  RESERVED  RESERVED  8

#define MPU9250_SMPLRT_DIV_REG        0x19
#define MPU9250_SMPLRT_DIV_X          0x00

#define MPU9250_ACCRAW_DIV          3
#define MPU9250_GYRORAW_DIV         2

#define ERROR_GYRO                    100
#define ERROR_ACC                     100

#define PI 3.1415926535
#define Gravity 9.8
#define RAD_TO_DEGREE 180/PI


#define AK8963_I2C_ADDRESS              0x0C

#define AK8963_WHO_AM_I                 0x00
#define AK8963_ID                       0x48
#define AK8963_X_MAG_REG                0x03
#define AK8963_STATUS_REG               0x02
#define AK8963_CNTL1_REG                0x0A
#define AK8963_CNTL1_FUSEROM_14bit      0x0F

#define AK8963_ASAX_REG           0x10
#define AK8963_ASAY_REG           0x11
#define AK8963_ASAZ_REG           0x12

/* BMP280센서 상세정보
 * 기압 측정범위 300 ~ 1100hPa
 * 상대적정확도 -0.12~+0.12hPa  -1m~+1m
 * 절대적정확도 -1hPa ~ 1hPa
 * 구동온도 범위 -40도 ~ 80도
 *
 * */

#define BMP280_I2C_ADDRESS                    (0x76)
#define BMP280_CHIP_ID_NUM                    (0x58)

#define BMP280_CHIP_ID_REG                    (0xD0)  /* Chip ID Register */
#define BMP280_RST_REG                        (0xE0)  /* Softreset Register */
#define BMP280_STAT_REG                       (0xF3)  /* Status Register */

#define BMP280_OVERSAMP_SKIPPED               (0x00)
#define BMP280_OVERSAMP_1X                    (0x01)
#define BMP280_OVERSAMP_2X                    (0x02)
#define BMP280_OVERSAMP_4X                    (0x03)
#define BMP280_OVERSAMP_8X                    (0x04)
#define BMP280_OVERSAMP_16X                   (0x05)

#define BMP280_CTRL_MEAS_REG                  (0xF4)  /* Ctrl Measure Register */
#define BMP280_CTRL_OSRS_T                    BMP280_OVERSAMP_1X  //  0~7
#define BMP280_CTRL_OSRS_T_MASK               (5)
#define BMP280_CTRL_OSRS_P                    BMP280_OVERSAMP_8X  //  0~7
#define BMP280_CTRL_OSRS_P_MASK               (2)
#define BMP280_CTRL_MODE                      (0x01)  //  0~3
#define BMP280_CTRL_MODE_MASK                 (0)
#define BMP280_CTRL_MEAS_CTRL                 (BMP280_CTRL_OSRS_T<<BMP280_CTRL_OSRS_T_MASK) | \
                                              (BMP280_CTRL_OSRS_P<<BMP280_CTRL_OSRS_P_MASK) | \
                                              (BMP280_CTRL_MODE<<BMP280_CTRL_MODE_MASK)

#define BMP280_CONFIG_REG                     (0xF5)  /* Configuration Register */

#define BMP280_PRESSURE_MSB_REG               (0xF7)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG               (0xF8)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG              (0xF9)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG            (0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG            (0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG           (0xFC)  /* Temperature XLSB Reg */


#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_FRAME_SIZE               (6)


#define T_INIT_MAX                       (20)
// 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX           (37)
// 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX             (10)
// 10/16 = 0.625 ms

void mpu_getacc(int16_data* data);
void mpu_getgyro(int16_data* data);
void mpu_getmag(int16_data* data);
void mpu_set_mag_init(int16_data* data);
void mpu_set_init();
void mpu_set_baro_init();
void mpu_get_sensor_data(int16_data* acc, int16_data* gyro, int16_data* mag, int32_data* baro);
uint8_t mpu_test();


#define CALIBRATE_NUM     500
void mpu_calibrate(int16_data* acc, int16_data* gyro);
void mpu_get_mpucalibrate();

#endif /* MPU9250_H_ */
























