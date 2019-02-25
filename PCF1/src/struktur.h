#ifndef STRUKTUR_H
#define STRUKTUR_H

typedef struct {
  int version;
  int array_nr;
  int max_array; 
  int richtung;
  //int soll_zeit_rauf[9];// in  msec
  float start_druck_rauf[9];
  float end_druck_rauf[9];
  float geschwind[9];
  float delta_druck[9];
  int strom[9];
  int strom_r[9];
  int delay[9];           // in msec
  int rampe_start[9];
  int rampe_stop[9];
  int rampe_start_r[9];
  int rampe_stop_r[9];
  int mikro_step_vor[9];
  int mikro_step_r[9];
  int max_steps;
  float geschwindr[9];
  unsigned char pass_word[20];
  float offset_sensor;
  float max_druck_sensor;
  float ist_druck_sensor;
  float m;
  float b;
  short fuell_min;
  short fuell_max;
  int entlagen_rechts;
  int entlagen_links;
  char ssid[40];
  char pass[40];
  char ip_adress[4];
  char kennung[16];
  int offset_step;
  int startstrom;
  float max_fuell_st;
  float min_fuell_st;
  float maximal_druck;
 // Kolbenoszillation
  int Oszi;
  int Oszi_UT_Toleranz;
  int Oszi_OT_Toleranz;
  int Oszi_Schrittweite;
  int Drehrichtung;
  int StallTol;
  int SS1;
  int SR1;
  int SS2;
  int SR2;
  int SS3;
  int SR3;
  int SSLimit;
  //########################


 } ramp1;


#endif
