
#include "PIDRegler.h"
#include "globdata.h"
//#include "adc.h"
#include "main.h" 
#include <string.h>
#include <stdio.h>
//#include <__cross_studio_io.h>
int toggle;

TIM_HandleTypeDef    Tim4Handle;
extern int SDADC_GetVal(void) ;
extern float get_adc_val(void); 
extern void set_up_motor_rampe(int array_nr) ;
extern void print_f_lf0( char * text); 
void pid_state(void) ;
void set_pid_timer(void); 
void TIM4_Handler(void); 

float error;
extern float max_druck;

void pid_Init( float kp, float ki, float kd, float abtast)  {
int tim_per;
  
 
  set_pid_timer(); 
  sollwert=0.0;
  processvalue =0.0;
}

void set_pid_start(void) {

    
    if(HAL_TIM_Base_Start_IT(&Tim4Handle) != HAL_OK){
#ifdef debug_print
   debug_printf("Error pid_start \n");
#endif
    /* Starting Error */
    //Error_Handler();
  }


}

void set_pid_timer(void) {
 int k;

   
  /* Set TIMx instance */
  Tim4Handle.Instance = TIMx4;
 
  Tim4Handle.Init.Period =1000 -1;
  Tim4Handle.Init.Prescaler = 72;  
  Tim4Handle.Init.ClockDivision = 0;
  Tim4Handle.Init.CounterMode = TIM_COUNTERMODE_DOWN;
   /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  __HAL_RCC_TIM4_CLK_ENABLE();
  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set Interrupt Group Priority */ 
  HAL_NVIC_SetPriority(TIM4_IRQn, 4, 0);
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  TIM_Base_SetConfig(Tim4Handle.Instance, &Tim4Handle.Init); 
  
  /* Initialize the TIM state*/
  Tim4Handle.State= HAL_TIM_STATE_READY;

}


void TIM4_Handler(void)  {
  HAL_TIM_IRQHandler(&Tim4Handle);
#ifdef STM_EVAL
  BSP_LED_Toggle(LED2);
#endif
  pid_state(); 
  timer_m++;

}
void pid_state(void){
   
            //ist_druck =get_adc_val();
             //ist_druck *=  rampe1.m;
             //ist_druck +=  rampe1.b;
//             if (ist_druck >= max_druck) {
//                  main_state = 11;
//                  pid_freigabe =0;
//                  }

             if (pid_freigabe ==1 ) {
                 if (ist_druck >= max_druck) {
                  main_state = 11;
                  store_offset((int) ist_druck); 
                  pid_freigabe =0;
                  }

                  sollwert += (rampe1.delta_druck[rampe1.array_nr]*10);
                  motor_zeit++;
                  if ( ist_druck >= rampe1.end_druck_rauf[rampe1.array_nr]) {
#ifdef debug_print
                  debug_printf(" Rampen wechsel\n");
#endif
                    rampe1.array_nr ++;
                    if ( rampe1.array_nr > rampe1.max_array) {
                        main_state = 11;
                        pid_freigabe =0;
                      }
                    }

              }
 

}
