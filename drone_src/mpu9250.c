/*
 * mpu9250.c
 *
 *  Created on: 2018. 9. 28.
 *      Author: JuYeong
 */
#include "mpu9250.h"
/*  외부소스  전역변수    */
extern UART_HandleTypeDef huart3;
/*****************/

static int16_data acc_calibrate={0};
static int16_data gyro_calibrate={0};
//static int16_data mag_calibrate={0};
static int16_t complete_calibrate=0;


void mpu_getacc(int16_data* data)
{
  Raw_data raw={0};
  i2c_read_bytes(MPU9250_I2C_ADDRESS,MPU9250_X_ACC_REG,&raw,6);
  data->array.data[ROLL]  = (raw.data[0]<<8 | raw.data[1]) >> MPU9250_ACCRAW_DIV;
  data->array.data[PITCH] = (raw.data[2]<<8 | raw.data[3]) >> MPU9250_ACCRAW_DIV;
  data->array.data[YAW]   = (raw.data[4]<<8 | raw.data[5]) >> MPU9250_ACCRAW_DIV;
  if(complete_calibrate){
    for(int i=0;i<3;i++){
      data->array.data[i] = data->array.data[i] - acc_calibrate.array.data[i];
    }
  }
}
void mpu_getgyro(int16_data* data)
{
  Raw_data raw={0};
  i2c_read_bytes(MPU9250_I2C_ADDRESS,MPU9250_X_GYRO_REG,&raw,6);
  data->array.data[ROLL]  = -((raw.data[2]<<8 | raw.data[3]) >> MPU9250_GYRORAW_DIV);
  data->array.data[PITCH] =  (raw.data[0]<<8 | raw.data[1])  >> MPU9250_GYRORAW_DIV;
  data->array.data[YAW]   =  (raw.data[4]<<8 | raw.data[5])  >> MPU9250_GYRORAW_DIV;
  if(complete_calibrate){
    for(int i=0;i<3;i++){
      data->array.data[i] = data->array.data[i] - gyro_calibrate.array.data[i];
    }
  }
}
void mpu_getmag(int16_data* data)
{
  Raw_data raw={0};
  if((i2c_read_byte(AK8963_I2C_ADDRESS,AK8963_STATUS_REG)&0x01)){
    i2c_read_bytes(AK8963_I2C_ADDRESS,AK8963_X_MAG_REG,&raw,7);
    for(int i=0;i<3;i++){
      data->array.data[i] = (raw.data[i*2+1]<<8) | (raw.data[i*2]);
    }
  }
}
void mpu_getbaro(int32_data* data)
{
  Raw_data raw={0};
  i2c_read_bytes(BMP280_I2C_ADDRESS,BMP280_TEMPERATURE_MSB_REG,&raw,6);
  data->array.data[0] = (raw.data[0]<<16) | (raw.data[1]<<8) | (raw.data[2]);   //온도
  data->array.data[1] = (raw.data[3]<<16) | (raw.data[4]<<8) | (raw.data[5]);   //압력
}
void mpu_get_sensor_data(int16_data* acc, int16_data* gyro, int16_data* mag, int32_data* baro)
{
  mpu_getacc(acc);
  mpu_getgyro(gyro);
  mpu_getmag(mag);
  mpu_getbaro(baro);
}
void mpu_set_mag_init(int16_data* data)
{
  Raw_data raw={0};
  i2c_write_byte(AK8963_I2C_ADDRESS, AK8963_CNTL1_REG, 0x0F); HAL_Delay(10);

  i2c_read_bytes(AK8963_I2C_ADDRESS,AK8963_ASAX_REG,&raw,3);
  for(int i=0;i<3;i++)
    data->array.data[i]=(int16_t)raw.data[i];

  i2c_write_byte(AK8963_I2C_ADDRESS, AK8963_CNTL1_REG, 0x00); HAL_Delay(10);
  i2c_write_byte(AK8963_I2C_ADDRESS, AK8963_CNTL1_REG, 0x16); HAL_Delay(10);
}
void mpu_set_init()
{
  i2c_write_byte(MPU9250_I2C_ADDRESS, MPU9250_POWER_SET_REG,    MPU9250_POWER_SET_REG_RESET);
  HAL_Delay(10);
  i2c_write_byte(MPU9250_I2C_ADDRESS, MPU9250_POWER_SET_REG,    MPU9250_POWER_SET_CLKSET);
  HAL_Delay(10);
  i2c_write_byte(MPU9250_I2C_ADDRESS, MPU9250_ACC_CONFIG_REG,   MPU9250_ACC_CONFIG_FS_X);
  HAL_Delay(10);
  i2c_write_byte(MPU9250_I2C_ADDRESS, MPU9250_ACC_CONFIG_REG,   MPU9250_GYRO_CONFIG_FS_X);
  HAL_Delay(10);
  i2c_write_byte(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG,  MPU9250_INT_PIN_CFG_BYPASS_EN);
  HAL_Delay(10);
  i2c_write_byte(MPU9250_I2C_ADDRESS, MPU9250_USER_CTRL_REG,    MPU9250_USER_CTRL_I2CDISABLE);
  HAL_Delay(10);
}
void mpu_set_baro_init()
{
  i2c_write_byte(BMP280_I2C_ADDRESS, BMP280_CTRL_MEAS_REG, 1);
}

uint8_t mpu_test()    //return값 0 : 정상      1: AK8963,BMP280문제     3: MPU9250,BMP280문제    4: BMP280문제
{                     //                  5: MPU9250,AK8963문제   6: AK8963문제     8: MPU9250문제    0xFF:모두문제
  uint8_t temp=0;
  uint8_t str[100];
  uint8_t check=0;
  uint8_t sensor[3]={0};
  uint8_t perspect=0;
  for(int i=0 ;i<0x7F ; i++){
    for(int j=0; j<0xFF;j++){
      temp = i2c_read_byte(i, j);
      if(temp == MPU9250_ID && i == MPU9250_I2C_ADDRESS && j == MPU9250_WHO_AM_I){
        sprintf((char*)str,"FIND%#X address - %#X reg : %#X\n",i,j,temp);
        HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),10);
        check++;
        sensor[0]=1;
      }
      else if(temp == AK8963_ID && i == AK8963_I2C_ADDRESS && j == AK8963_WHO_AM_I){
        sprintf((char*)str,"FIND%#X address - %#X reg : %#X\n",i,j,temp);
        HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),10);
        check++;
        sensor[1]=3;
      }
      else if(temp == BMP280_CHIP_ID_NUM && i == BMP280_I2C_ADDRESS && j == BMP280_CHIP_ID_REG){
        sprintf((char*)str,"FIND%#X address - %#X reg : %#X\n",i,j,temp);
        HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),10);
        check++;
        sensor[2]=5;
      }
    }
  }
  if(check==3){
    sprintf((char*)str,"ALL SENSOR GREEN");
    HAL_UART_Transmit(&huart3, str, (uint16_t)strlen((char*)str),10);
    return 0;
  }
  else{
    perspect = sensor[0]+sensor[1]+sensor[2];
    if(perspect == 0)
      return 0xFF;
    return perspect;
  }
}

/* mpu의 데이터를 칼리브레이션 하기전 호출하여 데이터를 모으는 함수
 * 호출하면 500번의 데이터를 수집하고 그 평균을 초기값으로 사용한다
 * 초기값은 현재 이페이지 소스의 전역변수에 들어간다.
 * */
void mpu_get_mpucalibrate()
{
  static int32_data acc={0};
  static int32_data gyro={0};
  //static int32_data mag;

  for(int i=0;i<CALIBRATE_NUM;i++){
    mpu_getacc(&acc_calibrate);
    mpu_getgyro(&gyro_calibrate);
    //mpu_getmag(&mag_calibrate);

    for(int j=0;j<3;j++){
      acc.array.data[j]   += acc_calibrate.array.data[j];
      gyro.array.data[j]  += gyro_calibrate.array.data[j];
     // mag.array.data[j]   += mag_calibrate.array.data[j];
    }
    HAL_Delay(1);
  }
  for(int j=0;j<3;j++){
    acc_calibrate.array.data[j]   = acc.array.data[j]/CALIBRATE_NUM;
    gyro_calibrate.array.data[j]  = gyro.array.data[j]/CALIBRATE_NUM;
    //mag_calibrate.array.data[j]   = mag.array.data[j]/CALIBRATE_NUM;
  }
  acc_calibrate.xyz.z -= ACC_1G;
  complete_calibrate = 1;
}
/* compute_get_mpucalibrate()를 먼저 호출하여야한다.
 * 전역변수로 선언된 칼리브레이트 기준값을 이용하여 읽어들이는 값을 보정한다.
 * */












