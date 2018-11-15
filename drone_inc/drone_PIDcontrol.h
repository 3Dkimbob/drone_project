/*
 * drone_PIDcontrol.h
 *
 *  Created on: 2018. 11. 2.
 *      Author: JuYeong
 */

#ifndef DRONE_PIDCONTROL_H_
#define DRONE_PIDCONTROL_H_
#include "struct_type.h"

float_data PID_control(float_data* rpy,int32_data* target_angle);
float_data PID_dual(float_data* rpy,float_data* output);


#endif /* DRONE_PIDCONTROL_H_ */
