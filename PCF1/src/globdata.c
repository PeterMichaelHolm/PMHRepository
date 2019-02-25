#include "struktur.h"


unsigned int adc_array[16];
int adc_val;
int timer2;

float sollwert;
float regler_val;
float processvalue;
float ist_druck;
int regler_flag=0;
int adc_val_array[16];
int adc_state;
char dummy[256];
char dummy2[100];
int ausgabe_timer;
float rs232_druck;
//typedef struct  {
    float lastProcessValue;
    float summError;
    float maxSummError;
    float maxError;
    float KP;
    float KI;
    float KD;
    // Abtastrate in Hz
    float abtastRate; 
    float regler_erg;
    float betrag;
    float regel_abweichung_max;
    float regel_abweichung_min;
//   }pidData_t;
//pidData_t pid;

int timer3;
int main_state =0;
int tx_restart0;
char ser_in_flag0;
//ramp rampe;
ramp1 rampe1;
int my_position;
short adc_oel;
int connect;
int pid_freigabe;
int motor_zeit;
int can_time_out;
int can_time_out_freigabe;
int rs_232_time_out;
int rs_232_time_out_freigabe;
int akt_geschw;
//moto steper_motor;
char flash_buffer[256];
char life_time_buffer[256];
int in_life_time_buffer;
int life_time_block;

int timer1_counter;
int in_anzeige_block;
int liter_offset;
int entlage_rechts,entlage_links;
int anzeige_state =0;
int blink_timer;
int tasten_ende;
int adc_timer;
int timer_m;
float max_ad_val;
float min_ad_val; 