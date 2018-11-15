/*
 * filter.h
 *
 *  Created on: 2018. 11. 11.
 *      Author: JuYeong
 */


extern volatile float twoKp;          // 2 * proportional gain (Kp)
extern volatile float twoKi;          // 2 * integral gain (Ki)
extern volatile float beta;           // algorithm gain
extern volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);


void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);


