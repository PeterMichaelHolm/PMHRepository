//#include <__cross_studio_io.h>
#include "CanBusData.h" 
#include "globdata.h"

extern int can_rx_flag;
extern int can_status;
extern int timer3;
extern int main_error;

extern void CAN_Config(void);
#define SDO_RX_ADR                0x580     // Senden
#define SDO_TX_ADR                0x600     // Empfangen

#define SDO_Wait                  0
#define SDO_Start                 1
#define SDO_Transimtt_process     2
#define SDO_Reseive_process       3
#define SDO_SendACK               4
#define SDO_GetACK                5
#define SDO_GetBlock_len          6
#define SDO_GetFirstBlock         7
#define SDO_GetNextBlock          8
#define SDO_SetBlockACK           9
#define SDO_Error                 10

#define sdo_state_write_1b        0x2f
#define sdo_state_write_2b        0x2b
#define sdo_state_write_4b        0x23
#define sdo_state_write_g4b1      0x20
#define sdo_state_write_g4b2      0x21
#define sdo_state_confirm         0x60
#define sdo_state_toggle1         0x20
#define sdo_state_toggle2         0x30
#define sdo_state_read_1          0x40  // anzahl unbestimmt
#define sdo_state_read_2          0x41  // anzahl stehen in byte 5 .. 8
#define sdo_state_read_4b         0x42  // 4 byte in byte 5 .. 8
#define sdo_state_read_1b         0x4f  // 1 byte in byte 5
#define sdo_state_read_2b         0x4b  // 2 byte in byte 5,6
#define sdo_state_read_e4b        0x43


void init_sdo(void) {
int i;
   can_time_out =0;
   can_time_out_freigabe=0; 
   my_sdo.state =SDO_Wait;
   my_sdo.art =0;
   my_sdo.time_out =0;
   for (i=0; i<8; i++) {my_sdo.data[i]=0; }
}
//typedef struct  {
//  int NodeID;
//  unsigned char object_parameter;
//  unsigned int object_index;
//  unsigned char obect_sub_index;
//  unsigned int var;
//  unsigned char data[8]
//  unsigned char state;
//  unsigned char art;
//  int time_out;
//}sdo ;

int sdo_reseive(int node, int object, char sub_object) {
int i;
int erg =0;
int ende;
 my_sdo.NodeID = node;
   my_sdo.data[0] = sdo_state_read_1  ; 
   my_sdo.data[1] = object&0x00ff; 
   my_sdo.data[2] = object>>8;
   my_sdo.data[3] = object=sub_object;

   CanHandle.pTxMsg->StdId = my_sdo.NodeID | SDO_TX_ADR;
   CanHandle.pTxMsg->ExtId = 0x01;
   CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
   CanHandle.pTxMsg->IDE = CAN_ID_STD;
   CanHandle.pTxMsg->DLC = 4;
   for(i=0; i< 4; i++) {
      CanHandle.pTxMsg->Data[i] = my_sdo.data[i];
    }
     can_rx_flag =0;
     can_time_out_freigabe =1;
   /*##-3- Start the Transmission process ###############################*/
   if (HAL_CAN_Transmit(&CanHandle, 10) != HAL_OK)
      {
       /* Transmition Error */
//#ifdef debug_print
       printf("can error reseive1");
//#endif
        } 
    ende =0;
    can_time_out_freigabe=1;
    while (ende ==0) {
        if (can_time_out > 200 ) { ende =1; }
        if (can_rx_flag !=0 ){ ende =1; }

    }
    can_time_out_freigabe=0;
    if ( can_rx_flag !=0 ) {
        if (CanHandle.pRxMsg->StdId == (my_sdo.NodeID | SDO_RX_ADR)) { // ok richtige adesse
            switch (CanHandle.pRxMsg->Data[0]) {
              case sdo_state_read_e4b:
              case 0x80:
                    erg  = CanHandle.pRxMsg->Data[7]; erg <<=8;
                    erg |= CanHandle.pRxMsg->Data[6]; erg <<=8;
                    erg |= CanHandle.pRxMsg->Data[5]; erg <<=8;
                    erg |= CanHandle.pRxMsg->Data[4];

            break;
            case sdo_state_read_1b:
                    erg  = CanHandle.pRxMsg->Data[4]; 
            break;
            case sdo_state_read_2b:
                    erg  = CanHandle.pRxMsg->Data[5]; erg <<=8;
                    erg |= CanHandle.pRxMsg->Data[4];
            break;
            case sdo_state_read_4b:
            break;
            default: // mehr als 4 byte
            break;
          }// switch
        }// stdId
      }
      else {main_error =5; 
            can_time_out_freigabe =0 ;
            main_state = 100;
  //       }
    }

  return erg;
}
int sdo_send (int node, int object, char sub_object,int data,int len) {
//printf("can 133\n");
	int i;
int erg =0;
int ende;
 my_sdo.NodeID = node;
 if (len <=4 ) {
    switch (len) {
        case 1: my_sdo.data[0] =sdo_state_write_1b; 
                my_sdo.data[4] = data&0xff;
        break;
        case 2: my_sdo.data[0] =sdo_state_write_2b;
                my_sdo.data[4] = data& 0xff;
                my_sdo.data[5] = data >>8;
        break;
        case 4: my_sdo.data[0] =sdo_state_write_4b;
                my_sdo.data[4] = data&0xff;
                my_sdo.data[5] = data >>=8;
                my_sdo.data[6] = data >>=8;
                my_sdo.data[7] = data >>8;

        break;
    }
   my_sdo.data[1] = object&0x00ff; 
   my_sdo.data[2] = object>>8;
   my_sdo.data[3] = object=sub_object;

   CanHandle.pTxMsg->StdId = my_sdo.NodeID | SDO_TX_ADR;
   CanHandle.pTxMsg->ExtId = 0x01;
   CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
   CanHandle.pTxMsg->IDE = CAN_ID_STD;
   CanHandle.pTxMsg->DLC = 4+len;
   for(i=0; i< (4+len); i++) {
      CanHandle.pTxMsg->Data[i] = my_sdo.data[i]; 
      }
   my_sdo.art =1; // warten das was zurück kommt
   can_rx_flag =0;
   /*##-3- Start the Transmission process ###############################*/
   if (HAL_CAN_Transmit(&CanHandle, 10) != HAL_OK)
      {
       /* Transmition Error */
//#ifdef debug_print
       printf("can error transmitt ");
//#endif
//       CAN_Config();
//       debug_printf("can config ");
          //Error_Handler();
        } 
//!!!!!!!!!!!!!!!! Timeout
     ende =0;
    can_time_out_freigabe=1;
    while (ende ==0) {
        if (can_time_out > 200 ) { ende =1; }

        if (can_rx_flag !=0 ){ ende =1; }

    }
    can_time_out_freigabe=0;
//    while (can_rx_flag ==0) {
//    }
    if ( can_rx_flag !=0 ) {  
        if (CanHandle.pRxMsg->StdId == (my_sdo.NodeID | SDO_RX_ADR)) { // ok richtige adesse
            switch (CanHandle.pRxMsg->Data[0]) {
                case sdo_state_confirm:
                    erg =1;
                break;
              }
          }
        }
    else {main_error =5; 
            can_time_out_freigabe =0 ;
            main_state = 100;
      }
  }// ende von len < 5
return erg;
 }


int reset_can(void) {
int erg =0;
int ende;
int i;
   CanHandle.pTxMsg->StdId = 0x000; 
   CanHandle.pTxMsg->ExtId = 0x01;
   CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
   CanHandle.pTxMsg->IDE = CAN_ID_STD;
   CanHandle.pTxMsg->DLC = 2;
   CanHandle.pTxMsg->Data[0] =0x81; 
   CanHandle.pTxMsg->Data[1] =0x01; 
   my_sdo.art =1; // warten das was zurück kommt
   can_rx_flag =0;
   can_status=0;
   timer3 =0;
   /*##-3- Start the Transmission process ###############################*/
   if ( i = HAL_CAN_Transmit(&CanHandle, 10) != HAL_OK) {
	   printf("can error in reset can"); }
   ende =0; 
   while ( ende ==0){
      if (timer3 > 2000) { ende =1; }
      if ( (can_status & 0x0008) == 0x0008) { ende =2; }
    } 
   if( can_status !=0) {
//#ifdef debug_print
  printf(" Timer3 = %d\n",timer3);
//#endif
      can_status =0;
      ende =0; 
      timer3 =0;
      while ( ende ==0){
          if (timer3 > 2000) { ende =1; }
          if ( (can_status & 0x0008) == 0x0008) { ende =2; }
        }
      if ( ende ==2) {  
//#ifdef debug_print
  printf(" Timer3 = %d\n",timer3);
//#endif
          CanHandle.pTxMsg->Data[0] =0x01; 
          CanHandle.pTxMsg->Data[1] =0x01; 
          can_status=0;
          timer3 =0;
   /*##-3- Start the Transmission process ###############################*/
          if (HAL_CAN_Transmit(&CanHandle, 10) != HAL_OK)  {
//#ifdef debug_print
  printf("can error in reset can ");
//#endif
            } 
          ende =0; 
          while ( ende ==0){
              if (timer3 > 3000) { ende =1; }
              if ( (can_status & 0x000f) == 0x0000f) { ende =2; }
            } 
//#ifdef debug_print
  printf(" Timer3 = %d\n",timer3);
//#endif
          if( ende  =2) {
              erg =1;
            }
        }
    }
 return erg;
 }

