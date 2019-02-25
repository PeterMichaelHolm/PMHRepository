/**
  ******************************************************************************
  * @file    PCF1/src/main.c
  * @author  Elektronik
  * @version v0
  * @date    2019-02-24
  * @brief
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "CanBusSDO.h"
#include "globdata.h"
#include "stm32373c_eval.h"
//#include "eeprom.h"
#include "Anzeige.h"
#include "eep.h" 
#include "eep_adr.h" 
#include "Motor1.h"
#include "globdata.h"
#include "at45db161.h"
#include "bit.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//extern uint8_t  BSP_LCD_Init(void);
//extern void     BSP_LCD_Clear(uint16_t Color);
//extern void     BSP_LCD_ClearStringLine(uint16_t Line);
//extern void     BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *pText);
//extern void     BSP_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *pText, Line_ModeTypdef Mode);
//extern void     BSP_LCD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii);
extern void I2C1_Init(void);
extern uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
extern uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);
extern void CANx_RX_IRQHandler(void);
extern void AdcInit(void);
extern float get_adc_val(void); 
extern void print_f_lf0( char * text); 
extern void befehl(void); 
extern void reset_in_0_buff( void) ;
extern int find_offset(void);

extern  uint32_t SPI2_Read(char val);
extern  uint32_t SPIx_Read(void);
extern void SPIx_Write(uint8_t Value);
extern void SPI2_Write(uint8_t Value);
extern void SPIx_Init(void);
extern int set_not_stop(void); 
//

//#define  fwVersion 100000 // 0.1.0

#define   m_start                         0
#define   m_start_motor_power_on          1
#define   m_motor_entlage_rechtslauf      2
#define   m_motor_stop                    3
#define   m_motor_entlage_linkslauf       4
#define   m_motor_startpos                5
#define   m_wait_start                    6
#define   m_motor_regeln                  7
#define   m_links_lauf                    8
#define   m_rechts_lauf                   9
#define   m_motor_delay2                  12
#define   m_motor_delay                   10
#define   m_motor_fertig                  11
#define   m_motor_stoppen_v1              20
#define   m_motor_stoppen_v2              21
#define   m_motor_stoppen_v3              22
#define   m_motor_stoppen_r1              23
#define   m_motor_stoppen_r2              24
#define   m_motor_stoppen_r3              25
#define t_warte_rauf_runter2              30
#define   m_wait_conn                     40
#define   m_error                         100



#define  e_motor_power_faild              1
#define  e_motor_entlage_links            2
#define  e_motor_entlage_rechts           3
#define  e_motor_startpos                 4
#define  e_can_time_out                   5
#define  e_abbruch                        6
#define  e_motor_steht                    7
#define  e_fuellstand_k                   8
#define  e_fuellstand_g                   9






#define taste_rauf                       0x01
#define taste_runter                     0x02
#define taste_enter                      0x04 
#define  e_can_config_error               6

#define debug_info                         1
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define KEY_PRESSED     0x00
#define KEY_NOT_PRESSED 0x01


// Pumpenvariablen, Oszillierung
extern int Oszi_OT_Multi;
extern int Oszi_UT_Multi;
extern int Oszi_UT_Temp;
extern int Oszi_OT_Temp;
extern int val;
extern int val1;

extern int V1;



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef    TimHandle;
uint32_t uwPrescalerValue = 0;
int can_rx_first;
uint8_t ubKeyNumber = 0x0;
CAN_HandleTypeDef    CanHandle;
UART_HandleTypeDef UartHandle;
SPI_HandleTypeDef SpiHandle1;
SPI_HandleTypeDef SpiHandle2;
SPI_HandleTypeDef SpiHandle;
ADC_HandleTypeDef    AdcHandle;
/* Buffer used for transmission */
uint8_t aTxBuffer[3] ;
uint8_t aRxBuffer[3];
/* Callback error called */
uint8_t aCallbackError = 0;

int can_rx_flag =0;
int soll_zeit;
int motor_timer;
extern char ser_in_flag0;
extern int store_offset(int val);

extern void motor_pin_init(void); //stm32373c.eval not used?
extern void spi_pin_init(void); //stm32373c.eval
extern void uart2Init(void); // Uart2.c

extern void pid_Init( float kp, float ki, float kd, float abtast)  ;
extern void set_pid_start(void);
extern void test_anzeige(void);
extern double Mittelwert_(double Eingabe[]);

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void CAN_Config(void);
//static void LED_Display(uint8_t LedStatus);
static void uart_init(void);

void set_timer3(void);
void init_spi(void) ;
int init_can(void);
void init_motor2(void);
void set_up_motor_rampe(int array_nr) ;
void debug_inf(char * text);
void wait_ms(int msec); 
void main_init(void);
void timer3_int(void);
float get_float(char art, char * text,int start_wert, int end_wert) ;
void setup_m_gesch(void);
void get_taste(void);
void init_adc(void);
float get_oel_stand(void); 
int get_timer_val(int art);



/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */




int position_alt;
int main_error =0;
int motor_power_status;
int motor_status;
int motor_node_id = 0x01;
int motor_position ;
int motor_position2 ;
int motor_geschw;
int motor_soll_tackt;
int motor_ist_tackt;
int motor_freigabe=0;
int can_status;
int i;
int last_NodeId;
int taste,tasten;
int taste_alt;
int tasten_timer1,tasten_timer2;
double Abtast_rate =1000.0;
int prg_nr;
float max_druck;
float f;
float max =0.0;
float min = 300.0;
float oel_stand;
double Druckwerte [101];
int Druckzaehler=0;
double x1,x2,y10,y2,m,b,dd;
int anzeige_timer;
char c;
int Schleppfehler;
int SpeedMultiplier;
double mess_druck;
/* Main Program ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

int main(void)

{
  int l;int SpeedMesser=0;
//  printf(" \n Starte Programm %f \n",y10);
  HAL_Init();

  SystemClock_Config();
  connect=0;
  main_init();
for (i=0; i < 99; ++i){Druckwerte[i]=0;}
printf("\n %d %d %d %d \n",rampe1.Oszi, rampe1.Oszi_UT_Toleranz, rampe1.Oszi_OT_Toleranz, rampe1.Oszi_Schrittweite);
wait_ms(50);
printf("\n %d %d %d %d \n",rampe1.rampe_start[0], rampe1.rampe_stop[0], rampe1.rampe_start_r[0], rampe1.rampe_stop_r[0]);
wait_ms(50);
  main_state =t_warte_rauf_runter2; //30
  tasten_ende =0;
  taste=0;
  tasten =0;
  taste_alt =0;
  motor_zeit=0;

  pid_freigabe =0;
  pid_Init( 0.0,0.0 , 0.0,100.0);
  set_pid_start();
// APo: Dieser Befehl überschreibt die rampe1-Werte
  i= find_offset(); //EEprom

  AdcInit();
  y10 = get_adc_val();
  printf("\n Druck %f \n",y10);

 get_oel_stand();
 oel_stand=get_oel_stand();

  max_ad_val =0.0;
  min_ad_val = 10.0; 
  //i= set_motor_entlage_rechts();
  anzeige_send_int((int)oel_stand);
  printf("\n Oil level %f \n",oel_stand);
/*APo
  while( (can_rx_first!=2) && (timer3 < 450)) {
      if ( ser_in_flag0 >0) {  befehl(); }
  
  }
 */
  #ifdef WLAN
      HAL_GPIO_WritePin(GPIOF, rs_485_dir_pin,GPIO_PIN_SET);
  #endif
  wait_ms(50);
 if( oel_stand < rampe1.min_fuell_st) {
      //main_error = e_fuellstand_k;
      //main_state = m_error;
 }
 if( oel_stand > rampe1.max_fuell_st ) {
      //main_error = e_fuellstand_g;
      //main_state = m_error;
 }
 // Motor Drehrichtung hard codiert / nur für China !!
 rampe1.Drehrichtung = 1;

  main_state =t_warte_rauf_runter2;
  for (i=1; i<=30;++i){printf("a");}printf("\n");

  anzeige_send_int(8);
  wait_ms1(200);
  // Initialisierung gegen Endanschlag
  i= set_motor_entlage_rechts();
  anzeige_send_int(9);
  wait_ms1(200);
  //
  for (i=1; i<=30;++i){printf("b");}printf("\n");

  if( i == -1 ) {
     main_error = e_motor_power_faild;
     main_state = m_error;
  }

rampe1.entlagen_rechts=entlage_rechts;

printf(" Nullpunkt (OT) bei %d \n",my_position);

//if( my_position >= 0 ) { my_position += rampe1.offset_step; }
//   else {my_position -= rampe1.offset_step; }

if (rampe1.Drehrichtung == 1) {my_position += rampe1.offset_step;}
if (rampe1.Drehrichtung == 0) {my_position -= rampe1.offset_step;}
for (i=1; i<=30;++i){printf("x");}printf("\n");

printf("\n Nullpunkt (OT) mit Offset bei %d \n",my_position);

position_alt = my_position;

anzeige_send_int((int)max_druck);
printf(" maxdruck %f\n",max_druck);

if (rampe1.Drehrichtung == 1) {val = rampe1.max_steps+ my_position;}
else {val = my_position - rampe1.max_steps;}

printf("\n my_position %d val %d \n", my_position, val);

val1 = my_position;
Oszi_OT_Multi=1;
Oszi_UT_Multi=1;
Oszi_UT_Temp = val;
Oszi_OT_Temp = my_position;
//SpeedMultiplier = 1.0;
SpeedMultiplier= rampe1.geschwind[rampe1.array_nr];
//rampe1.StallTol=100;
  /* Main Loop ---------------------------------------------------------*/


//i= set_motor_entlage_rechts();


  while (1)
  {
	  // China4 Notreparatur
	  //rampe1.m = 67.6211243;
	  //rampe1.b = -7.02230072;

	  //printf("main_state %s:",main_state);
	  ++SpeedMesser;
	  if ( ser_in_flag0 >0) {   befehl();     } //Wlan-Befehl




    switch (main_state) {
//----------------------------------------------------------------------------------------------------------
      case t_warte_rauf_runter2:  //  30
    	  //printf("t_warte_rauf_runter2 30\n");
    	  rs232_druck = 0.0;
    	  printf("Max Druck 1 %f \n", max_druck);
           max_druck= get_float(1,"Max Druck = ",(int)rampe1.maximal_druck, rampe1.maximal_druck); 
           if ( rs232_druck > 0.0) {  max_druck= rs232_druck; }
           if ( rs232_druck > rampe1.maximal_druck) { max_druck = rampe1.maximal_druck;} 
           printf("Max Druck 2 %f \n", max_druck);
           l=0;
            while( l < rampe1.max_array){
                 if ( ist_druck <  rampe1.end_druck_rauf[l]) {
                    rampe1.array_nr =l;
                    l = rampe1.max_array;
                    }
                  l ++;
                  }

            main_state = m_start;		// Main State gesetzt

// APo: Befehl überschreibt Rampen-Werte
            i= find_offset();


          //printf("\n +++t_warte_rauf_runter2 beendet +++\n");
      break;
//----------------------------------------------------------------------------------------------------------
      case m_links_lauf:  //  8  DRUCKAUFBAU
    	  //rampe1.m = 67.6211243;
    	  //rampe1.b = -7.02230072;

    	  printf("\n xxxxxxxxxxxxxxx m_links_lauf 8 max Druck %f", max_druck);
    	  rampe1.max_array = 8;
    	  int l = 0;
    	  while( l < rampe1.max_array){
              if ( ist_druck <  rampe1.end_druck_rauf[l]) {
                  rampe1.array_nr =l;
                  printf("%d", l);
                  l = rampe1.max_array;
                }
              l ++;
          }


          l = sdo_reseive(1,0x6064,0);    // aktuelle Position
           wait_ms(2);
           //printf("Position: %d Zielwert %d my_position %d \n",l,V1, my_position);
           // Abbruchbedingung: Ziel erreicht

           int tempV=0;
           tempV = (int) (SpeedMultiplier * rampe1.geschwind[rampe1.array_nr]);


           //printf("\n xxx %f, %f, %f xxx \n",SpeedMultiplier,  rampe1.geschwind[rampe1.array_nr], rampe1.StallTol);
           //printf("\nx %f %d %i x\n",rampe1.StallTol,rampe1.StallTol,rampe1.StallTol);
           //printf("drehrichtung %d", rampe1.Drehrichtung);

           if (rampe1.Drehrichtung == 1) {
			  if (l >= (V1-5)){
				  printf("\n Ziel erreicht %d \n", SpeedMesser);
 				  SpeedMesser = 0;
				  pid_freigabe=0;
				  //################################################################
				  main_state = m_rechts_lauf;
		          i=setup_motorrueckfahrt(0);
		              if ( i== 1) {
		                    main_state = m_rechts_lauf;
		                    anzeige_timer=0;
		               }
		              else {
		                //main_error = e_motor_power_faild;
		                //main_state = m_error;
		            }
		          //#################################################################
				  //main_state=m_motor_delay;
				  //set_motor_power_off();
			  }
          }
          if (rampe1.Drehrichtung == 0) {
				  if (l <= (V1+5)){
					  //printf("\n Ziel erreicht \n");
					  pid_freigabe=0;
					  //################################################################
					  main_state = m_rechts_lauf;
			          i=setup_motorrueckfahrt(0);
			              if ( i== 1) {
			                    main_state = m_rechts_lauf;
			                    anzeige_timer=0;
			               }
			              else {
			                main_error = e_motor_power_faild;
			                main_state = m_error;
			            }
			          //#################################################################


					  //main_state=m_motor_delay;
					  //set_motor_power_off();
				  }
          }

 	  timer3 =0;
             ist_druck =get_adc_val();
             mess_druck = ist_druck;
             ist_druck *=  rampe1.m;
             ist_druck +=  rampe1.b;

             Druckwerte[Druckzaehler] = ist_druck;
 	  	  	 ++ Druckzaehler;
 	  	  	 if (Druckzaehler > 9) Druckzaehler = 0;

             //printf(" Ist Druck %f Mittelwert %f \n ", ist_druck, Mittelwert_(Druckwerte));
             printf("Druckwerte: Ist-Druck: %f m: %f b: %f mess_druck: %f \n", ist_druck, rampe1.m, rampe1.b, mess_druck);
             //ist_druck = Mittelwert_(Druckwerte);


             if( ist_druck <0) { ist_druck =0;}

             oel_stand=get_oel_stand();
             if( oel_stand < rampe1.min_fuell_st ) {
                  //set_motor_power_off();
                  //main_error = e_fuellstand_k;
                  //main_state = m_motor_stoppen_r1;
              }
             if( oel_stand > rampe1.max_fuell_st ) {
                  //set_motor_power_off();
                  //main_error = e_fuellstand_g;
                  //main_state = m_motor_stoppen_r1;
                }
             while(timer3 < 10) {
                  if (tasten == taste_enter ){
                	//i= sdo_send (1,0x605a, 0,2,2);
                    //wait_ms(50);
                    //set_motor_power_off();

                    setup_motorrueckfahrt(1);

                    //main_error = e_abbruch;
                    main_state = m_motor_stoppen_v1;
                    timer3 = 21;

                  }
              }
             printf(" Ist Druck %f max_Druck %f \n ", ist_druck, max_druck);
             if (ist_druck >= max_druck) {
                 setup_motorrueckfahrt(1);

                 //main_error = e_abbruch;
                 main_state = m_motor_stoppen_v1;
                 timer3 = 21;
             }
              anzeige_timer++;
              if( anzeige_timer>5 ) {
                   anzeige_timer =0;
                                      
                   if ((tasten == taste_rauf ) || ( tasten == taste_runter)) {
                      oel_stand=get_oel_stand();
                      anzeige_send_int((int)oel_stand);
                   }
                   else {
                      anzeige_send_int((int)ist_druck );
                  }
              }
              /*i =  sdo_reseive(1,0x2039,1);
              wait_ms(2);
              i +=  sdo_reseive(1,0x2039,2);
              wait_ms(2);
              i +=  sdo_reseive(1,0x2039,3);
              wait_ms(2);
              i +=  sdo_reseive(1,0x2039,4);*/
              //printf("Strom: %d V1: %d, Oszi_UT_Multi %d  my_position %d \n", i,V1, Oszi_UT_Multi,my_position);




              // Prüfe auf Schleppfehler
              // Abbruchbedingung: Schleppfehler
              i =  sdo_reseive(1,0x6041,0);

              wait_ms(2);
              int j =  sdo_reseive(1,0x1003,0);
              //printf("\nFehler: %d %d %d %d\n",i,j,l,V1);

              Schleppfehler = 0;
              if((i & 8192) == 8192) {
                  printf("Schleppfehler!\n");
            	  Schleppfehler = 1;
                  //set_motor_power_off();
                  //main_error=e_motor_steht;
                  //main_state = m_motor_stoppen_v1;

              }
              //printf("\ny %f : %f \n", rampe1.array_nr, rampe1.geschwind[rampe1.array_nr]);
              if ( main_error == 0 ) {
                  //if ((motor_timer > soll_zeit) || (Schleppfehler == 1)){
            	  if (Schleppfehler == 1){
					  Schleppfehler = 0;
                      pid_freigabe=0;
                      // Geschwindigkeit runternehmen
                      SpeedMultiplier -= 10;
                      	 if (SpeedMultiplier <= 50) SpeedMultiplier = 50;
                      	 printf("Speed %d",SpeedMultiplier);
                      	set_motor_power_off();
                      	 i=setup_motorhinfahrt(SpeedMultiplier);

   		             	//
                      //main_state=m_motor_delay;
                      //set_motor_power_off();


                  }

               }


               //printf("++ Linkslauf Ende %d, %d, %d, %d, %d \n", motor_timer, soll_zeit, main_state, l, main_error);
break;
//----------------------------------------------------------------------------------------------------------
case m_rechts_lauf:  //  9 Ruecklauf
    	  //printf("xxxxxxxxxxxxxxx m_rechts_lauf 9");

          l = sdo_reseive(1,0x6064,0);    // aktuelle Position
            wait_ms(2);
            //printf("Position: %d Zielwert %d my_position %d \n",l,V1, my_position);


            if (rampe1.Drehrichtung == 1){
            	if (l <= (V1+5)){
            //printf("\n Ziel OT erreicht \n");

            		//##########################################################################
            		main_state = m_links_lauf;

            		SpeedMultiplier= rampe1.geschwind[rampe1.array_nr];
            		i=setup_motorhinfahrt(SpeedMultiplier);
  		              if ( i== 1) {
  		                    main_state = m_links_lauf;
  		                    anzeige_timer=0;
  		               }
  		              else {
  		                main_error = e_motor_power_faild;
  		                main_state = m_error;
  		            }
  		              //##########################################################################
            		//main_state = m_motor_delay2;
            		//set_motor_power_off();
           	   }
           }
           if (rampe1.Drehrichtung == 0) {
           	if (l >= (V1-5)){
           	 //##########################################################################
           	            		main_state = m_links_lauf;
           	            		SpeedMultiplier= rampe1.geschwind[rampe1.array_nr];
           	  		             i=setup_motorhinfahrt(SpeedMultiplier);
           	  		              if ( i== 1) {
           	  		                    main_state = m_links_lauf;
           	  		                    anzeige_timer=0;
           	  		               }
           	  		              else {
           	  		                main_error = e_motor_power_faild;
           	  		                main_state = m_error;
           	  		            }
           	//##########################################################################
           	            		//main_state = m_motor_delay2;
        	  //main_state = m_motor_delay2;
        	  //set_motor_power_off();

           	}
           }




    	  timer3 =0;
              if (tasten == taste_enter ){
                  
                  main_error = e_abbruch;
                  main_state = m_motor_stoppen_v2;
                  anzeige_send_fehler(main_error);
             }
             ist_druck =get_adc_val();
             ist_druck *=  rampe1.m;
             ist_druck +=  rampe1.b;
             if( ist_druck <0) { ist_druck =0;}

             oel_stand=get_oel_stand();
             if( oel_stand < rampe1.min_fuell_st) {
                  //set_motor_power_off();
                  //main_error = e_fuellstand_k;
                  //main_state = m_motor_stoppen_r1;
              }
            if( oel_stand > rampe1.max_fuell_st ) {
                  //set_motor_power_off();
                  //main_error = e_fuellstand_g;
                  //main_state = m_motor_stoppen_r1;
                }
                anzeige_timer++;
               if( anzeige_timer>5 ) {
                   anzeige_timer =0;
                                      
                   if ((tasten == taste_rauf ) || ( tasten == taste_runter)) {
                      oel_stand=get_oel_stand();
                      anzeige_send_int((int)oel_stand);
                   }
                   else {
                      anzeige_send_int((int)ist_druck );
                  }
                }
              while(timer3 < 10) {
                 
              
              }
              
              l = sdo_reseive(1,0x6064,0);    // akuelle Position
              //printf(" POS: %d, %d, Ziel: %d \n", l, my_position,V1);
//              if ( l != position_alt) { position_alt = l;}
//              else {main_state=m_motor_delay;
//                      set_motor_power_off();
//                      main_error=e_motor_steht;
//                      main_state = m_motor_stoppen_r1;
//                  }
              if ( main_error ==0 ) { 
                  if (rampe1.Drehrichtung ==1){

					  if ( l <=  my_position) {
						  /*main_state = m_motor_delay2;
						  //main_state =m_start_motor_power_on;
						  set_motor_power_off();*/

                    }
                  }
                  if (rampe1.Drehrichtung ==0){

					  if ( l >=  my_position) {
						  /*main_state = m_motor_delay2;
						  //main_state =m_start_motor_power_on;
						  set_motor_power_off();*/

                    }
                  }

                }
              //printf("xxxxxxxxxxxxxxx Ende POS %d, %d \n", l, my_position);
      break;
      //----------------------------------------------------------------------------------------------------------
case m_motor_stoppen_v2:  //  21
    	  printf("xxxxxxxxxxxxxxx m_motor_stoppen_v2\n");

    	  /*
    	  l =  sdo_reseive(1,0x1001,0); // fehler auslesen
            if( l > 3000) {
            	//printf("zzzzzzzzz 444\n");
            	set_motor_power_off();
                //printf("power off\n");
                main_error=e_motor_steht;
                main_state = m_motor_stoppen_r1;

             }
             */
              wait_ms(2);
                 l = sdo_reseive(1,0x6064,0);    // aktuelle Position
                  wait_ms(2);
                if ( l <=  my_position +5) {
                      main_state =t_warte_rauf_runter2;
                      set_motor_power_off();
                       anzeige_send_int((int)max_druck);

                    }
      break;
 //----------------------------------------------------------------------------------------------------------
      case m_error: //  100
    	  printf("xxxxxxxxx m_error 100\n");
    	  anzeige_send_fehler(main_error);
          //set_motor_power_off();
          while(1) {
            if ( ser_in_flag0 >0) {

                befehl();
                }
          }
      break;
//----------------------------------------------------------------------------------------------------------
      case m_start:  // 0
     	  //printf("xxxxxxxxxxxxxxx 0\n");
     	  main_error=0;
                 // START Motor
     	  //printf("\n m_start 0 setup_motorhinfahrt \n");
     	 SpeedMultiplier= rampe1.geschwind[rampe1.array_nr];
     	  i = setup_motorhinfahrt(SpeedMultiplier);
     	  	  	// **************************************************
                 soll_zeit = get_timer_val(0);
                 if( i !=1) {
                     main_error = e_motor_power_faild;
                     main_state = m_error;
                   }
                 else {


                       set_pid_start();
                       pid_freigabe=1;

                       main_state = m_links_lauf;
                       anzeige_timer=0;

                   }


            // main_state =  m_start_motor_power_on;
       break;
 //----------------------------------------------------------------------------------------------------------
      case m_motor_stoppen_v1:  //  20
    	  printf("xxxxxxxxx m_motor_stoppen_v1\n");
    	  anzeige_send_fehler(main_error);
            wait_ms(200);
            i=setup_motorrueckfahrt(2);
            //i= set_not_stop();
              if ( i== 1) {
            	  main_state =m_motor_stoppen_v2;
               }
              else {
            	main_error = e_motor_power_faild;
                main_state = m_error;
            }
      break;
      //----------------------------------------------------------------------------------------------------------













   /*


      case m_wait_conn:  // 40
    	  //printf("xxxxxxxxxxxxxxx 40\n");
    	  ist_druck =get_adc_val();
                  ist_druck *=  rampe1.m;
                  ist_druck +=  rampe1.b;
                  if( ist_druck <0) { ist_druck =0;}
                   anzeige_send_int((int)ist_druck );
                if ( tasten == taste_enter) {
                    while (tasten == taste_enter) {}
                    timer3 =0;
                    while(timer3 < 50) { }
                    main_state = m_start;
                 }

      break;
      case m_start:  // 0
    	  //printf("xxxxxxxxxxxxxxx 0\n");
    	  main_error=0;
                // START Motor
    	  //printf("\n m_start 0 setup_motorhinfahrt \n");
    	  i = setup_motorhinfahrt();
    	  	  	// **************************************************
                soll_zeit = get_timer_val(0);
                if( i !=1) {
                    main_error = e_motor_power_faild;
                    main_state = m_error;
                  }
                else {


                      set_pid_start();
                      pid_freigabe=1;

                      main_state = m_links_lauf;
                      anzeige_timer=0;

                  }


           // main_state =  m_start_motor_power_on;
      break;

      case m_start_motor_power_on:  //  1
    	  //printf("xxxxxxxxxxxxxxx 1\n");
    	  //printf("\n m_start_motor_power_on 1 setup_motorhinfahrt \n");
    	  i = setup_motorhinfahrt();
              soll_zeit = get_timer_val(0);
              if( i ==1){
                 main_state = m_links_lauf;
                 pid_freigabe=1;
                 anzeige_timer=0;
                }
              else {
                  main_error = e_motor_power_faild;
                  main_state = m_error;
                    }


      break;




      case m_motor_delay:  //  10
    	  printf("\n m_motor_delay 10\n");
    	  //store_offset((int) ist_druck);
              oel_stand = get_oel_stand() ;
              //wait_ms(rampe1.delay[rampe1.array_nr]/10);
              main_state = m_rechts_lauf;
              ist_druck =get_adc_val();
              ist_druck *=  rampe1.m;
              ist_druck +=  rampe1.b;
              if( ist_druck <0) { ist_druck =0;}
              store_offset((int) ist_druck);
              l=0;
//              while( l < rampe1.max_array){
//                  if ( ist_druck <  rampe1.end_druck_rauf[l]) {
//                      rampe1.array_nr =l;
//                      l = rampe1.max_array;
//                    }
//                  l ++;
//                }
//              for (i=0;i< rampe1.max_array; i++){
//                  if ( ist_druck <  rampe1.end_druck_rauf[i]) {
//                    rampe1.array_nr =i;
//                    }
//                }

              printf("\n m_motor_delay 10 setup_motorrueckfahrt \n");
              i=setup_motorrueckfahrt();
              if ( i== 1) {
                    main_state = m_rechts_lauf;
                    anzeige_timer=0;
               }
              else {
                main_error = e_motor_power_faild;
                main_state = m_error;
            }

      break;




      case m_motor_delay2:  //  12
    	  printf("xxxxxxxxx m_motor_delay2 12\n");
    	  wait_ms(rampe1.delay[rampe1.array_nr]/10);
                  l=0;
                  while( l < rampe1.max_array){
                      if ( ist_druck <  rampe1.end_druck_rauf[l]) {
                          rampe1.array_nr =l;
                          l = rampe1.max_array;
                        }
                      l ++;
                  }
                  main_state =m_start_motor_power_on;
                  set_motor_power_off();

      break;


      case m_motor_entlage_rechtslauf:  //  2
    	  printf("xxxxxxxxxxxxxxx m_motor_entlage_rechtslauf 2\n");
    	  if (set_motor_entlage_rechts() ==1 ) {
               main_state = m_motor_entlage_linkslauf;
            }
            else {
              main_error = e_motor_entlage_rechts;
              main_state = m_error;
            }
      break;


      case m_motor_stop:  //  3
      break;


      case m_motor_entlage_linkslauf:  //  4
    	  printf("xxxxxxxxx m_motor_entlage_linkslauf 4\n");
    	  if (set_motor_entlage_links() == 1) {
              main_state = m_motor_startpos;
            }
            else {
              main_error = e_motor_entlage_links;
              main_state = m_error;
            }
      break;


      case m_motor_startpos:  //  5
    	  printf("xxxxxxxxx m_motor_startpos 5\n");
    	  if( set_motor_startpos() == 1) {
              main_state=m_wait_start;
            }
            else {
              main_error = e_motor_startpos;
              main_state = m_error;
            }
      break;


      case m_wait_start:  //  6
      break;
 

      case  m_motor_fertig :  //  11
    	  printf("xxxxxxxxx m_motor_fertig 11\n");
    	  i= sdo_send (1,0x605a, 0,1,4);
             wait_ms(2);
            set_motor_power_off();
             
             main_state = m_motor_stoppen_v1;
             main_error = e_abbruch;
             //anzeige_send_int((int)max_druck);
           // main_state = m_wait_start;
      break;


      case m_motor_regeln:  //  7
      break;








      case m_motor_stoppen_v3:  //  22
      break;


      case m_motor_stoppen_r1:  //  23
    	  printf("xxxxxxxxxxxxxxx m_motor_stoppen_r1 23\n");
    	  anzeige_send_fehler(main_error);
            main_state =m_motor_stoppen_r2;
            
      break;


      case m_motor_stoppen_r2:  //  24
    	  printf("xxxxxxxxx m_motor_stoppen_r2 24\n");
    	  if ((tasten == taste_rauf ) || ( tasten == taste_runter)) {
              main_state =m_motor_stoppen_v1;
          }
      break;


      case m_motor_stoppen_r3:  //  25
      break;



*/
    }
 
  }
}

int get_timer_val(int art){
int i,j;
double s1,s2,sg,a,t,t2,v,tg,t3;
 int erg =0;
 i = rampe1.array_nr;
 if( art ==0) {
    a = rampe1.rampe_start[i];
    a /= 60.0;
    v= rampe1.geschwind[i];
    v /= 60.0;
    t= v/a;
    s1= a/2;
    s1 *= t;
    s1 *= t;  // s = a/2 * t+t
    a = rampe1.rampe_stop[i];
    a/= 60.0;
    t2 = v/a;
    s2 = a/2;
    s2 *=t2;
    s2 *=t2;
    sg = s1 + s2;
    sg *= 2000;// Steps / umdr
    tg = t+t2;
    j =  rampe1.max_steps-sg-rampe1.offset_step; // noch zu fahrende steps
    v *=2000; // steps / sec
    t3 = (float) j / v; // Zeit in sec
    tg += t3;
    tg *= 100; // in timer_tics
    erg = (int)tg;
   }
 else {
     a = rampe1.rampe_start_r[i];
    a /= 60.0;
    v= rampe1.geschwindr[i];
    v /= 60.0;
    t= v/a;
    s1= a/2;
    s1 *= t;
    s1 *= t;  // s = a/2 * t+t
    a = rampe1.rampe_stop_r[i];
    a /= 60.0 ;
    t2 = v/a;
    s2 = a/2;
    s2 *=t2;
    s2 *=t2;
    sg = s1 + s2;
    sg *= 2000;// Steps / umdr
    tg = t+t2;
    j =  my_position+rampe1.max_steps-sg; // noch zu fahrende steps
    v *=2000; // steps / sec
    t3 = (float) j / v; // Zeit in sec
    tg += t3;
    tg *= 100; // in timer_tics
    erg = (int) tg;
  }
 motor_timer =0;
 erg += 30;
 return erg;
 }

float get_oel_stand(void) {
 float f,f1;
 __IO uint16_t uhADCxConvertedValue = 0;
  if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler();
  }
  /*##-4- Wait for the end of conversion #####################################*/
  /*  For simplicity reasons, this example is just waiting till the end of the
      conversion, but application may perform other tasks while conversion
      operation is ongoing. */
  if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK)
  {
    /* End Of Conversion flag not set on time */
    Error_Handler();
  }
  else
  {
    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel  ########################*/
    uhADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
  }
  adc_oel=uhADCxConvertedValue;
  f = (float ) adc_oel;
  f -= (float) rampe1.fuell_min;
  f1 = (float )rampe1.fuell_max -(float) rampe1.fuell_min;
  f = f/f1;
  f *= 100.0;
  if ( f >100.0) { f = 100.0;}
  if ( f< 0.0) { f = 0.0;}
 return f;
 }

void init_adc(void) {
ADC_ChannelConfTypeDef sConfig;
__HAL_RCC_ADC1_CONFIG(RCC_ADC1PCLK2_DIV6);
  AdcHandle.Instance          = ADCx;
  AdcHandle.Init.ScanConvMode          = DISABLE;
  AdcHandle.Init.ContinuousConvMode    = DISABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfDiscConversion   = 0;
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion       = 1;
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* Initiliazation Error */
    Error_Handler();
  }
  /*##-2- Configure ADC regular channel ######################################*/
  sConfig.Channel      = ADCx_CHANNEL;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
}

float get_float(char art, char * text,int start_wert, int end_wert)
{

 int wert;
 int wartezeit;
// char ende =0;
 wartezeit = 50;
 wert = start_wert;
 blink_timer =0;
 adc_timer=0;
 while( tasten_ende ==0) {
    if ( ser_in_flag0 >0) {  befehl(); }
    switch (anzeige_state) {
        case 1: // Anzeige von soll oder ist druck
                if ( tasten ==0) {wartezeit = 40; 
                    if( blink_timer > 500) { anzeige_state =0; }
                }
                else { blink_timer =0; }
                if ( tasten ==taste_enter) { tasten_ende =1; }
                if ( tasten == taste_rauf ) { // +
                    wert ++;
                    if ( wert > rampe1.maximal_druck ) { wert = rampe1.maximal_druck  ;}
                        anzeige_send_int(wert);
                        wait_ms(wartezeit);
                        if(wartezeit > 5) { wartezeit -=2; }       
 
                  }
     
                if ( tasten == taste_runter) { // 
                    if ( wert > rampe1.maximal_druck) { wert =  rampe1.maximal_druck -1;}

                    wert --;    
                if ( wert < 0 ) { wert = 0 ;}
                      anzeige_send_int(wert);
                      wait_ms(wartezeit);
                      if(wartezeit > 5) { wartezeit -=2; }       
                }
            
       break;


       case 0:set_blink();
              anzeige_state =2;
             adc_timer =0;
       break;


       case 2:
             if ( adc_timer >25 ) { 
                ist_druck =get_adc_val();
                ist_druck *=  rampe1.m;
                ist_druck +=  rampe1.b;
                if( ist_druck <0) { ist_druck =0;}
                anzeige_send_int(ist_druck);
                adc_timer=0;
               }
             if( tasten != 0 ) {
                anzeige_state=1;
                //tasten_ende=1;
                reset_blink();
                anzeige_send_int(wert);
                blink_timer=0;
                anzeige_state =1;

             }
       break;
      }
    }
 tasten_ende=0;
 reset_blink();
 return (float) wert;
  }


void  init_spi(void)
{

  SpiHandle1.Instance               = SPI1;
  SpiHandle1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  SpiHandle1.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle1.Init.CRCPolynomial     = 7;
  SpiHandle1.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle1.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle1.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  SpiHandle1.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
  SpiHandle1.Init.Mode = SPI_MODE_MASTER;
    if(HAL_SPI_Init(&SpiHandle1) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

  SpiHandle2.Instance               = SPI2;
  SpiHandle2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  SpiHandle2.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle2.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle2.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle2.Init.CRCPolynomial     = 7;
  SpiHandle2.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle2.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle2.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle2.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  SpiHandle2.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
  SpiHandle2.Init.Mode = SPI_MODE_MASTER;
    if(HAL_SPI_Init(&SpiHandle1) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }
  __HAL_SPI_ENABLE(&SpiHandle2);
  __HAL_SPI_ENABLE(&SpiHandle1);
  
}
void main_init(void) {
//short vers;
int ee_status;
float f;

 uart_init();
 uart2Init();
 spi_pin_init();
init_eep();
   

  /*##-1- Configure the CAN peripheral #######################################*/
  CAN_Config();
  printf("CAN Config ok\n");
  can_rx_first =0;
 
  /*##-2- Start the Reception process and enable reception interrupt #########*/
  if (HAL_CAN_Receive_IT(&CanHandle, CAN_FIFO0) != HAL_OK)
  {
  printf("Can receive error\n");
    /* Reception Error */
    Error_Handler();
  }

  I2C1_Init();
  anzeige_init();
  anzeige_send_int(0);
  set_timer3();
  can_status=0;


  init_eep();
  get_struct((char *)&rampe1, 8, sizeof(rampe1) );
  if ( rampe1.version == 0xffffffff)
  {
    y10 = get_adc_val();
    y10 =0.158;
    y2 = 4.5;
    x2 = 300;
    m = x2 /(y2-y10);
    b= 300 -(m *4.5) ;
////////  b = -y10*m;
  x1 = 4.4;
  dd= m*x1;
  dd+=b;
//  debug_printf(" Druck bei %f int %f \n",x1,dd);  
  x1 = 4.5;
  dd= m*x1;
  dd+=b;
//  debug_printf(" Druck bei %f int %f \n",x1,dd);  
rampe1.b = -4.82523;
rampe1.m = 68.985;
rampe1.maximal_druck= 250.0;
store_struct((char *) &rampe1, 8, sizeof(rampe1));
  }
  if( rampe1.version != '9') {
      init_motor2();
      rampe1.version = '9';
      rampe1.fuell_min= 0x0758;
      rampe1.fuell_max = 0x08bf;
 
      strcpy(rampe1.ssid,"Hermes");
      strcpy(rampe1.pass,"?Gulpl@Hermes17a");
      rampe1.ip_adress[0]= 192;
      rampe1.ip_adress[1]= 168;
      rampe1.ip_adress[2]= 178;
      rampe1.ip_adress[3]= 100;
      strcpy(rampe1.kennung,"anfang");
      rampe1.fuell_min= 25;
      rampe1.fuell_max= 75;
      rampe1.max_steps=136000;
      rampe1.offset_step = 1500;
      rampe1.startstrom=800; 
      rampe1.b = -4.82523;
      rampe1.m = 68.985;
      rampe1.max_fuell_st= 90.0;
      rampe1.min_fuell_st=10.0;
      rampe1.maximal_druck= 250.0;
      store_struct((char *) &rampe1, 8, sizeof(rampe1));
      
   }
  else {
      setup_m_gesch(); 
    }






init_adc();
 AdcInit();

    f = get_adc_val(); 
    f = get_adc_val(); 
    f *= 100.0;
    ee_status = rampe1.end_druck_rauf[rampe1.max_array];
    anzeige_send_int((int)ee_status);

 



}
void init_motor2(void){
int i,j;
double f,g;
  rampe1.array_nr =0;
  rampe1.max_array =0; 
  rampe1.richtung =1;
  for ( i=0; i<8; i++) {  
    
      rampe1.end_druck_rauf[i] =0;   
      rampe1.delay[i] =0;
      rampe1.geschwind[i]=0;
      rampe1.delta_druck[i]=0;
      rampe1.delay[i] =0;   
      rampe1.rampe_stop[i]=400;
      rampe1.rampe_stop_r[i]=400;
      rampe1.rampe_start[i]=250;
      rampe1.rampe_start_r[i]=400;
      rampe1.strom[i] =  1000;
      rampe1.strom_r[i] =  2500;
    }
  

  rampe1.geschwind[0]= 125;
  rampe1.start_druck_rauf[0]=0;
  rampe1.end_druck_rauf[0]    =100; // Druck in MPa
  rampe1.delay[0] =3000;
  rampe1.geschwindr[0]=800.0;// min ^-1 
  
  rampe1.geschwind[1]= 97;
  rampe1.start_druck_rauf[1]= 100;
  rampe1.end_druck_rauf[1]    =180; // Druck in MPa
  rampe1.delay[1] =4000;
  rampe1.geschwindr[1]=750.0;// min^-1 

  rampe1.geschwind[2]= 83;
  rampe1.start_druck_rauf[2]= 180;
  rampe1.end_druck_rauf[2]    =250; // Druck in MPa
  rampe1.delay[2] =5000;
  rampe1.geschwindr[2]=700;// min^-1 
  rampe1.max_steps= 2000*58; // 2000 steps pro Umdrehung 60 Umdehungen pro Hub
  j=0;
   // 100 Hübe bis maximalen Druck 2000 pulse pro Umdrehung mit 60 umdehungen bis ende der Stange
  for( i=0; i<8; i++)  {   if(rampe1.geschwind[i] > 0) { j++; } } 
  rampe1.max_array =j-1;
  for (i=0; i<= rampe1.max_array; i++) {
     set_up_motor_rampe(i); 
     f = rampe1.end_druck_rauf[i]- rampe1.start_druck_rauf[i];    // Druck in Bar
     f /= 0.000025; // Druckänderung pro step
     f /= 2000;   // Steps pro umdrehung
     //g = rampe1.soll_zeit_rauf[i];
     g /=1000 ;  // Zeit in sec
     f = f/g;     // Geschwindigkeit
     f *= 60;       // Umdrehunhen pro min
     //rampe1.geschwind[i]= f;
  }
  

  rampe1.array_nr =0;

  strcpy(rampe1.pass,"Wurz12");
//strcpy(rampe1.pass_word,"Wurz12");

  akt_geschw = rampe1.geschwind[i];
 }
  
void setup_m_gesch(void) {
int i,j;
double f,g;
  rampe1.array_nr =0;
  rampe1.max_array =0; 
  rampe1.richtung =1;
 j=0;
   // 100 Hübe bis maximalen Druck 2000 pulse pro Umdrehung mit 60 umdehungen bis ende der Stange
  for( i=0; i<8; i++)  {   if(rampe1.geschwind[i] > 0) { j++; } } 
  rampe1.max_array =j-1;
  for (i=0; i<= rampe1.max_array; i++) {
     set_up_motor_rampe(i); 
     f = rampe1.end_druck_rauf[i]- rampe1.start_druck_rauf[i];    // Druck in Bar
     f /= 0.000025; // Druckänderung pro step
     f /= 2000;   // Steps pro umdrehung
    // g = rampe1.soll_zeit_rauf[i];
     g /=1000 ;  // Zeit in sec
     f = f/g;     // Geschwindigkeit
     f *= 60;       // Umdrehunhen pro min
     //rampe1.geschwind[i]= f;
 
  }
  rampe1.array_nr =0;
 
  akt_geschw =rampe1.geschwind[0];
}


void set_up_motor_rampe(int array_nr) {

 
  rampe1.delta_druck[array_nr]=rampe1.end_druck_rauf[array_nr] - rampe1.start_druck_rauf[array_nr] ;
 // rampe1.delta_druck[array_nr] /=rampe1.soll_zeit_rauf[array_nr];
//  rampe.zeit_hin =rampe.soll_zeit_rauf[array_nr];
//  rampe.startwert_hin =rampe.start_druck_rauf[array_nr] ;
//  rampe.endwert_hin =rampe.end_druck_rauf[array_nr];
  
  

}



void debug_inf(char * text){
#ifdef debug_info
  printf(text);
#ifdef debug_print
  debug_printf(text);

#endif
#endif
}

void get_taste(void) {
int j =0;

j = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8);// & bit8;
 if ( j ==0) { tasten |= taste_enter ; }
 else        { tasten &= ~taste_enter;}

 j = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9);// & bit9;
 if ( j ==0) { tasten |= taste_runter ; }
 else        { tasten &= ~taste_runter;}

 j = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4); // & bit4;
 if ( j ==0) { tasten |= taste_rauf ; }
 else        { tasten &= ~taste_rauf;} 

}

void timer3_int(void) {
 blink_timer++;
 motor_timer++;
 adc_timer++;
 get_taste();
//  if (BSP_PB_GetState(BUTTON_KEY) == KEY_PRESSED) {
//      if ( tasten_timer1 < 200) {    tasten_timer1++; }
//      if (tasten_timer1 > 100) {   
//            tasten |= 1;
//            if (tasten_timer2 > 50 ) {   tasten |= 4;}
//        }
//  
//      if (tasten_timer1 > 150) { tasten |= 2; }
//  }
//  else {
//     
//      tasten &= ~0x0003;
//      tasten_timer1 =0;
//  }
// if (BSP_PB_GetState(BUTTON_TAMPER) == KEY_PRESSED) {
//      if (tasten_timer2 < 200) { tasten_timer2 ++; }
//      if (tasten_timer2 > 100) {   
//            tasten |= 4; 
//      if (tasten_timer1 > 50 ) {   tasten |= 1; }       
//        }
// 
//      if (tasten_timer2 > 150) {  tasten |= 8; }
//
//  }
//  else { 
//     
//      tasten &= ~0x000c;
//      tasten_timer2 =0;
//  }
  if ( can_time_out_freigabe ==1 ) {
      can_time_out++;
 
        
    }
  else {can_time_out =0;
  }
  if (  rs_232_time_out_freigabe ==1) {
      rs_232_time_out ++;
      if ( rs_232_time_out > 500 ) {
            rs_232_time_out_freigabe =0;
            reset_in_0_buff();
       }
   }
   else {rs_232_time_out =0;
    }
}
void wait_ms(int msec) {
  timer3 =0;
  while(timer3 < msec) {
       if ( ser_in_flag0 >0) {   befehl();     }
  }
 }

void set_timer3(void) {
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2 
      => TIM3CLK = HCLK = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) (SystemCoreClock / 10000) - 1;
  /* Set TIMx instance */
  TimHandle.Instance = TIMx;
  /* Initialize TIM3 peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
     */
  TimHandle.Init.Period = 100 - 1;
  TimHandle.Init.Prescaler = uwPrescalerValue;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
   __HAL_RCC_TIM3_CLK_ENABLE();
  TIM_Base_SetConfig(TimHandle.Instance, &TimHandle.Init); 

  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
 { 
#ifdef debug_print 
 debug_printf("error tim3");
 #endif
    /* Initialization Error */
    Error_Handler();
  }
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  HAL_NVIC_SetPriority(TIMx_IRQn, 4, 0);
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIMx_IRQn);

  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}




void can_test(void) {
 while ( 1) {
    while (BSP_PB_GetState(BUTTON_KEY) == KEY_PRESSED)
    {
      if (ubKeyNumber == 0x4)
      {
        ubKeyNumber = 0x00;
      }
      else
      {
#ifdef STM_EVAL
        LED_Display(++ubKeyNumber);
#endif        
        /* Set the data to be tranmitted */

//#ifdef debug_print
  printf("Send Data ");
//#endif
        CanHandle.pTxMsg->Data[0] = 0x00;
        CanHandle.pTxMsg->Data[1] = 0xAD;
        
        /*##-3- Start the Transmission process ###############################*/
        if (HAL_CAN_Transmit(&CanHandle, 10) != HAL_OK)
        {
          /* Transmition Error */
//#ifdef debug_print
  printf("can error ");
//#endif
            CAN_Config();
//#ifdef debug_print
  printf("can config ");
//#endif
          //Error_Handler();
        }
       
        HAL_Delay(10);
        
        while (BSP_PB_GetState(BUTTON_KEY) != KEY_NOT_PRESSED)
        {
        }
      }
    }
  }
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = RCC_PLL_MUL9 (9)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
//#ifdef debug_print
printf(" Error");
//#endif
  while (1)
  {
  }
}

/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
void CAN_Config(void)
{
 int i;
  CAN_FilterConfTypeDef  sFilterConfig;
  static CanTxMsgTypeDef        TxMessage;
  static CanRxMsgTypeDef        RxMessage;

  /*##-1- Configure the CAN peripheral #######################################*/
  CanHandle.Instance = CANx;
  CanHandle.pTxMsg = &TxMessage;
  CanHandle.pRxMsg = &RxMessage;

  CanHandle.Init.TTCM = DISABLE;
  CanHandle.Init.ABOM = DISABLE;
  CanHandle.Init.AWUM = DISABLE;
  CanHandle.Init.NART = DISABLE;
  CanHandle.Init.RFLM = DISABLE;
  CanHandle.Init.TXFP = DISABLE;
  CanHandle.Init.Mode = CAN_MODE_NORMAL;
  CanHandle.Init.SJW = CAN_SJW_1TQ;
  CanHandle.Init.BS1 = CAN_BS1_4TQ;
  CanHandle.Init.BS2 = CAN_BS2_4TQ;
  CanHandle.Init.Prescaler = 4;
//  CanHandle.Init.SJW = CAN_SJW_1TQ;
//  CanHandle.Init.BS1 = CAN_BS1_5TQ;
//  CanHandle.Init.BS2 = CAN_BS2_3TQ;
//  CanHandle.Init.Prescaler = 2;
  i = HAL_CAN_Init(&CanHandle);
  if (i != HAL_OK)
  { 
//#ifdef debug_print
  printf("init can error %d \n", i);
//#endif
    /* Initiliazation Error */
    Error_Handler();
  }
  printf("kein init error\n");
  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
  {
//#ifdef debug_print
  printf("init filter error\n");
//#endif
    /* Filter configuration Error */
    Error_Handler();
  }
  printf("KEIN init filter error\n");

  /*##-3- Configure Transmission process #####################################*/
  CanHandle.pTxMsg->StdId = 0x321;
  CanHandle.pTxMsg->ExtId = 0x01;
  CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
  CanHandle.pTxMsg->IDE = CAN_ID_STD;
  CanHandle.pTxMsg->DLC = 2;
  HAL_NVIC_SetPriority((IRQn_Type)(CAN_RX0_IRQn), 0x0F, 0);
  HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);

// HAL_NVIC_SetPriority((IRQn_Type)(CAN_RX1_IRQn), 0x0F, 0);
// HAL_NVIC_EnableIRQ(CAN_RX1_IRQn);


}

/**
  * @brief  Transmission  complete callback in non blocking mode
  * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle)
{

if (CanHandle->pRxMsg->StdId == ( 0x701 )) {
      can_rx_first=1;
  }
// printf("Empfang\n");
  if (CanHandle->pRxMsg->StdId == ( 0x580 | motor_node_id)) {
// printf( "ID = %x \n",CanHandle->pRxMsg->StdId );
  }

 
//  if(CanHandle->pRxMsg->DLC ==1) { debug_printf(" %x ",a);}
//  if(CanHandle->pRxMsg->DLC ==2) { debug_printf(" %x %x",a,b);}
//  if(CanHandle->pRxMsg->DLC ==3) { debug_printf(" %x %x %x",a,b,c);}
//  if(CanHandle->pRxMsg->DLC ==4) { debug_printf(" %x %x %x %x",a,b,c,d);}
//  if(CanHandle->pRxMsg->DLC ==8) { debug_printf(" %x %x %x %x %x %x %x %x",a,b,c,d,e,f,g,h);}
  //debug_printf(" Empfang\n");
  can_rx_flag =1; 
 last_NodeId = CanHandle->pRxMsg->StdId;
 if (CanHandle->pRxMsg->StdId == ( 0x180 | motor_node_id)) {
      motor_status  = CanHandle->pRxMsg->Data[7]; motor_status <<=8;
      motor_status |= CanHandle->pRxMsg->Data[6]; motor_status <<=8;
      motor_status |= CanHandle->pRxMsg->Data[5]; motor_status <<=8;
      motor_status |= CanHandle->pRxMsg->Data[4];
      motor_power_status = motor_status & 0x03ff;
      can_status|=0x0001;
      //can_rx_flag =0;
    }
 if (CanHandle->pRxMsg->StdId == ( 0x280 | motor_node_id)) {
      motor_position  = CanHandle->pRxMsg->Data[3]; motor_position <<=8;
      motor_position |= CanHandle->pRxMsg->Data[2]; motor_position <<=8;
      motor_position |= CanHandle->pRxMsg->Data[1]; motor_position <<=8;
      motor_position |= CanHandle->pRxMsg->Data[0];
      can_status|=0x0002;
      //can_rx_flag =0;
      
    }
if (CanHandle->pRxMsg->StdId == ( 0x380 | motor_node_id)) {
      motor_geschw  = CanHandle->pRxMsg->Data[7]; motor_geschw <<=8;
      motor_geschw |= CanHandle->pRxMsg->Data[6]; motor_geschw <<=8;
      motor_geschw |= CanHandle->pRxMsg->Data[5]; motor_geschw <<=8;
      motor_geschw |= CanHandle->pRxMsg->Data[4];
      //can_rx_flag =0;
      can_status|=0x0004;  
    }
if (CanHandle->pRxMsg->StdId == ( 0x700 | motor_node_id)) {
      can_status|=0x0008;
   }
if (CanHandle->pRxMsg->StdId == ( 0x480 | motor_node_id)) {
      can_status|=0x0010;
   }


  /* Receive */
  if (HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0) != HAL_OK)
  {
//#ifdef debug_print
  printf("receiver error interrupt\n");
//#endif
    /* Reception Error */
   // Error_Handler();
  }
}

 void uart_init(void) {
  UartHandle.Instance        = USARTx;
  HAL_UART_DeInit(&UartHandle); 
 
  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  HAL_UART_MspInit(&UartHandle);
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

 }

 



 int _write(int file, char *ptr, int len) {
   /* Implement your write code here, this is used by puts and printf for example */
   for (int i = 0; i < len; i++)
     ITM_SendChar((*ptr++));
   return len;
 }
double Mittelwert_ (double Eingabe[]){
	double tWert = 0;
	int i = 0;
	while (Eingabe[i] != 0)
	{
		tWert += Eingabe[i];
		++i;
	}
	if (i > 0){

		return tWert / i;
		}
	else{

		return 0;
		}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

