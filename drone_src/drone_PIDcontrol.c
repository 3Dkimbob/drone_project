/*
 * drone_PIDcontrol.c
 *
 *  Created on: 2018. 11. 2.
 *      Author: JuYeong
 */


#include "drone_PIDcontrol.h"


static float_data PID_constant[3];





float_data PID_control(float_data* rpy,int32_data* target_angle)
{

  static float error_angle;
  static float p_error;
  static float_data iterm;
  float_data output;
  float pterm,dterm;

  for(int i=0;i<3;i++){
    error_angle = target_angle->array.data[i] - rpy->array.data[i];

    pterm = PID_constant[i].array.data[P]*error_angle;

    iterm.array.data[i] += error_angle*PID_constant[i].array.data[I];

    dterm = PID_constant[i].array.data[D]*(p_error-error_angle);
    p_error = error_angle;
    output.array.data[i] = pterm + iterm + dterm;
  }
  return output;
}
float_data PID_dual(float_data* rpy,float_data* target_angle,float_data* gyro)
{

}


























