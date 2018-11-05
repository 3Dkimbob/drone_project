/*
 * drone_PIDcontrol.c
 *
 *  Created on: 2018. 11. 2.
 *      Author: JuYeong
 */


#include "drone_PIDcontrol.h"


static float_data P_constant={0};
static float_data I_constant={0};
static float_data D_constant={0};





void PID_control(float_data* rpy,int32_data* target_angle,float_data* output)
{
  static float_data error_angle;
  static float_data P_output;
  static float_data I_output;
  static float_data D_output;

  for(int i=0;i<3;i++){
    error_angle.array.data[i] = target_angle->array.data[i] - rpy->array.data[i];

    P_output.array.data[i] = P_constant.array.data[i]*error_angle.array.data[i];
  }
}
void PID_dual(float_data* rpy,float_data* output)
{

}


























