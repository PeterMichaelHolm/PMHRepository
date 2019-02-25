

#ifndef PID_H
  #define PID_H
  #include "globdata.h"

#define max_I_Term  2.0

   void pid_Init( float kp, float ki, float kd, float abtast);
   float pid_Controller(float processvalue, float setpoint);
   void pid_Reset_Integr(void); 




#endif
