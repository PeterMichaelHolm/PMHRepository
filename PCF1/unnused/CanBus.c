//#include <__cross_studio_io.h>
//#include "CANopen.h"
extern void debug_inf(char * text);

int init_can(void);

int init_can(void){
int error = -1;
CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
CO_ReturnError_t err;
uint16_t timer1msPrevious;
 debug_inf("canopen init");

 while(reset != CO_RESET_APP){
/* CANopen communication reset - initialize CANopen objects *******************/
   /* disable CAN and CAN interrupts */
/* initialize CANopen */
     err = CO_init(0/* CAN module address */, 10/* NodeID */, 1000 /* bit rate */);
     if(err != CO_ERROR_NO){
      debug_inf("error in CO_init");

        while(1);
            /* CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err); */
        }
      else {
      reset = CO_RESET_APP;
      error =1;
       debug_inf(" ok ");
      }
  }
 return err;
}
