#include "globdata.h"
#include "main.h" 
//#include <__cross_studio_io.h>

void set_motor_drehzahl(float drehzahl,float steps,float,int array_nr);



void set_motor_drehzahl(float drehzahl,float steps,float min_drehzahl,int array_nr ){
 float f;
 float e;
 float g;
 float min;
  steper_motor.drehzahl =drehzahl;

  e = 1/72000000.0;
  f = (float) 500/60; //umd pro sec
  if ( f < 1.0) {
      f*= 2000.0;
      steper_motor.minimal_zeit[array_nr]=f/e;
   }
   else {
      f = 1/f; //Zeit für eine umdrehung in sec
      f /= 2000.0;
      steper_motor.minimal_zeit[array_nr]=f/e;
   }

  f = (float) drehzahl/60; //umd pro sec
  if ( f < 1.0) {
      f*= 2000.0;
      steper_motor.min_zeit[array_nr]=f/e;
   }
   else {
      f = 1/f; //Zeit für eine umdrehung in sec
      f /= 2000.0;
      steper_motor.min_zeit[array_nr]=f/e;
   }

 
  min = min_drehzahl/60.0;
  if ( min < 1.0) {
      min = 1/min;
      min /= 2000;
      min /= e;
  }
  else {
    min = 1/min; // Zeit für eine Umdrehung in sec
    min/=2000.0;
    min /=e;
    }
  
  steper_motor.max_zeit[array_nr] = min;
  e= (float ) min;
  e -= steper_motor.min_zeit[array_nr];
  g=e/steps;
  steper_motor.delta_timer[array_nr] = g;
  steper_motor.ist_timer = (float)min;

  rampe.minimal_zeit = 500.0/60.0;
  rampe.minimal_zeit =1/rampe.minimal_zeit;
  rampe.minimal_zeit /= 2000.0;
  rampe.minimal_zeit *=72000000.0;
 
  
//  set_motor_timer(); 
}
//
//float get_max_zeit(void) {
// //return (float )steper_motor.max_zeit;
//}
//float get_min_zeit(void) {
//return 1.0;
// //return steper_motor.min_zeit;
// }
//void set_motor_zeit(float val) {
//steper_motor.ist_timer= val;
//}
//void set_motor_state(int val) {
//  steper_motor.state = val;
//  }

