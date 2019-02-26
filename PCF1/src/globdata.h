#ifndef GLOBDADA_H
#define GLOBDATA_H
#include "struktur.h"
#define WLAN        1
// #define RS485      1
extern unsigned int fwVersion;
extern unsigned int adc_array[16];
extern int adc_val;
extern int timer2;
extern int motor_step_anzahl;
extern int motor_step_zeit;
extern int drehzahl;
extern int micro_step;
extern void wait_ms( int val);

extern int adc_val_array[16];
extern int adc_state;
extern float ist_druck;
extern int Motortemperatur;
extern int connect;
//typedef struct  {
extern float lastProcessValue;
extern float summError;
extern float maxSummError;
extern float maxError;
extern float KP;
extern float KI;
extern float KD;
    // Abtastrate in Hz
extern float abtastRate; 
extern float regler_erg;
extern float regel_abweichung_max;
extern float regel_abweichung_min;
//   }pidData_t;
//extern pidData_t pid;

extern float sollwert;
extern float regler_val;
extern int regler_flag;
extern float processvalue;
extern float betrag;
extern char dummy[256];
extern char dummy2[100];
extern int ausgabe_timer;
extern int timer3;
extern int main_state;
extern int tx_restart0;
extern char ser_in_flag0;


//extern ramp rampe;
extern ramp1 rampe1;
extern int pid_freigabe;
extern int motor_zeit;
extern int can_time_out;
extern int can_time_out_freigabe;
extern int rs_232_time_out;
extern int rs_232_time_out_freigabe;
extern int akt_geschw;


extern char flash_buffer[256];
extern char life_time_buffer[256];
extern int timer1_counter;
extern int in_anzeige_block;
extern int life_time_block;
extern int in_life_time_buffer;
extern int life_time_block;
extern int liter_offset;
extern int timer1_counter;
extern short adc_oel;
extern int my_position;

extern int entlage_rechts,entlage_links;
extern int anzeige_state;
extern int blink_timer; 
extern int tasten_ende;
extern int timer_m;
extern int adc_timer;
extern float max_ad_val;
extern float min_ad_val;
extern float rs232_druck; 
#endif
