/**
*    @file     befehl.c
*    @brief    Auswertung und Verarbeitung der Befehle über WLAN
*
*    @date     2018-11-
*    @version  0.1.0
*    überarbeitet APo 24.02.2019xx 1122
*/

/**
* Definition eines Befehls:
* Kodierung UTF8!!!!!
* Ein Befehl wird als Block gekapselt der mit einem ESC Zeichen beginnt und dem END Zeichen endet
*  Es gibt immer einen Block als Antwort 
*  Ein weiterer Block darf erst gesendet werden , wenn eine Antwort auf den voherigen erfolg ist 
*  Sollte in dem Befehlsblock ein ESC oder END Zeichen vorkommen wird diesem ein ESC Zeichen vorangestellt.
*  Gibt es mehr als einen Parametet werden diese durch ein ';' Zeichen getrennt.
*  Bei Float werten werden die Zeichen '.' oder ',' als Trennung des nach Komma teils verwendet   
*
*  ping "p"              Parameter: Test der Komunikation
*                        Rückgabe ApOKZ 
*
*  set_zeit "z"          Parameter:
*                           1: Speichern =1 auslesen =0 abgeschlossen mit ;
*                           2: Array Nr. '1' .. '8' Ascii Rampen Nr. Abgeschlossen mit ;
*                           3: Zeit in msec
*                         Rückgabe AzOKZ 
*                         Beispiel "Az1;2;510000Z Speichert diese Zeit in Array Nr.2
*
*  set_end_druck "d"     Parameter:
*                           1: Speichern =1 auslesen =0 abgeschlossen mit ;
*                           2: Array Nr. '1' .. '8' Ascii Rampen Nr. Abgeschlossen mit ;
*                           3: Druck als Float im Mpa
*                         Rückgabe AzOKZ 
*                         Beispiel "Ad1;2;250.3Z oder auch
*                         Beispiel "Ad1;2;250,3Z  
*                         Setzt denn Solldruck Der zweiten Rampe im ersten Prozess auf 250.3 Mpa
*
*  set_prg_nr "n"         Vorgesehen aber noch nicht realisiert
*                         Parameter :
*                           1: Prozess Nr. als int '1' .. '8'
*                         Rückgabe AzOKZ 
*                         Beispiel "AP2Z"
*                         Prozess mit dieser Nr.  wird als default eingestellt
*
*  start "s"              Parameter Keine
*                         Rückgabe AsOKZ 
*                         Beispiel "AsZ"  
*                         Ein Prozess wird gestartet
*
*  Stop "S"               Parameter Keine
*                         Rückgabe ASOKZ 
*                         Beispiel "ASZ"  
*
*  Für die folgenden Befehle muß ein Passwort gesendet werden
*  Somit können Änderungen nur vom Fachpersonal durchgeführt werden
*
*  passwort "a"           Parameter 
*                             1:  Passwort in UTF8
*                         Rückgabe AaOKZ 
*                         Beispiel "Aa********Z"  
*                         Die Ausführung des nächsten Befehls wird freigegeben
*
*  set_max_druck_geber "M"  Parameter
*                               1: Maximaler Druck des Sensor's in MPa als Float
*                           Rückgabe AMOKZ
*                           Beispiel "AM300.0Z"
*
*  set_max_druck "m"      Zum einstellen der Steigung der Geradengleichung
*                         Parameter 
*                                1: Aktueller Druck als Float
*                         Rückgabe AmOKZ
*                         Beispiel "Am257.9Z
*
*  set_offset "o"         Zum einstellen des offsets wenn denn einer vorhanden ist
*                         Wird bei drucklosen System eingestellt.
*                             Parameter Keine
*                         Rückgabe AoOKZ
*                         Beispiel "AoZ
*
*/

#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "globdata.h" 
#include "stm32373c_eval.h"
#include "at45db161.h"


#define  fwVersion 100000 // 0.1.0

#define ESC                     'A'
#define END                     'Z'

#define ping                    'p' 
#define set_zeit                'z'
#define set_end_druck           'd'
#define set_rueck_geschw        'r'
#define set_max_step            'b'
#define set_prg_nr              'n'
#define start                   's'
#define stop                    'S'
#define set_beschl_rauf         'c'
#define set_beschl_runter       'e'
#define set_beschl_rauf_r       'D'
#define set_beschl_runter_r     'E'
#define get_druck               'f'
#define passwort                'a'
#define set_max_druck_geber     'M'
#define set_max_druck           'm'
#define set_offset              'o'
#define print_druck             'P'
#define print_rampe_vor         'B'
#define print_rampe_zurueck     'C'
#define set_strom               'g'
#define set_strom_r             'j'
#define set_max_fuell           'I'
#define set_delay               'k'
#define set_steigung            't'
#define set_entlage1            'v'
#define set_fuell_min           'w'
#define set_fuell_max           'x'
#define set_netzwerk            'y'
#define set_start_strom         'h'
#define set_rampe_offset        'i'
#define set_conn                'n'  // not used
#define get_block_zahl          'F'
#define set_block               'G'
#define get_end_druck           'H'
#define del_speicher            'K'
#define max_druck_eingabe       'l'
#define print_fwversion         'V'

// Ergaenzung Kolbenoszillierung
#define set_Oszi           		'Y'
#define set_Oszi_UT_Toleranz	'X'
#define set_Oszi_OT_Toleranz	'U'
#define set_Oszi_Schrittweite	'W'
#define set_Drehrichtung		'T'
#define set_StallTol			'R'

#define AllgParameter			'J'


//#define set_SS1					'J'
#define set_SR1					'L'
#define set_SS2					'N'
#define set_SR2					'O'
#define set_SS3					'Q'
#define set_SR3					'u'
#define set_SSLimit				'q'


extern void setup_m_gesch(void); // main.c
extern void print_f0( char * text) ; // Uart2.c
extern void print_f_lf0( char * text); //main.c
extern void reset_in_0_buff( ); // Uart2.c
extern float get_adc_val(void); //Adc.c
extern int set_motor_entlage_rechts(void); // motor1.c
extern float get_oel_stand(void); // main.c
extern int set_motor_power_off(void) ;  //motor1.c
extern int sdo_send (int node, int object, char sub_object,int data,int len); // CanBusSDO.c
extern int sdo_receive(int node, int object, char sub_object); // CanBusSDO.c
extern int find_offset(void); //at45db161.c
extern void anzeige_send_int(int val); // anzeige.c
extern int get_in_buff_len(void); // std?
extern unsigned char get_char(void); // std
extern void send_slip0( char * text, int len); // Uart2.c
extern int auto_setup(void); //keine
extern int get_eep_int16(int block,int adr); // keine
extern void wait_ms(int msec); // main.c

int toString(char []);
void send_ACK(char c, char art) ;
int get_ser_int( char * buff, int* address) ;
float get_ser_float( char * buff, int * adress);
void send_float_val(char befehl, float val);
void  send_int_val(char befehl,int val);


extern int main_error ;
extern float max_druck;
extern float oel_stand;
extern int soll_zeit;
extern int motor_timer;
extern int tasten;


int password=1;


/****************************************************************************************************************************************
 * Main Switch
 * arbeitet die Befehle ab, die über WLAN eingehen
 */

void befehl(void) {
char b;        // Befehlcode Buchstabe
char c;        //
char d;        //
char e;        // Unterbefehlscode Ziffer (optional, 1 = setzen, 0 = lesen)
char str[40];  // Parameter
int i,k,j;
int l;
float f,f2,f1;


if (ser_in_flag0 !=0 ) {                                                 // Befehlsdaten empfangen
    ser_in_flag0 --;                                                     // erstes Zeichen = Befehlscode
    b = get_char();
    printf("Befehl= %c  \n",b);

        switch (b) { // Hauptschleife *************************************************************************

      	case AllgParameter: // 'J'

        		printf("\n allg Parameter!\n");
        		int il;

        		i = get_in_buff_len(); // i = Länge des empfangenen Strings
                il = i;
        		k =0; j =0; l =0;

                 while(( i> 0 ) && ( k<255)) { // String einlesen get_char() UART2.c, i wird runtergezählt, max. Länge 255
                	 str[k++] = get_char();
                     i--;
                     }

                 str[k++] = ';';
                 str[k] =0;  // ??? Das ergibt keinen Sinn
                 char Parametersatz[10];
                 char Application_[251];
            	 int ii = 0;int j = 0;

                 Parametersatz[0] = str[2];
                 Parametersatz[1] = str[3];
                 Parametersatz[2] = str[4];
                 int Param;
                 Param = toString(Parametersatz);
                 printf ("\n Param: %i \n", Param);
                 switch(Param)
                 {

                 case 101: // Befehl 1
                	 f = get_ser_float(&str[i],&l);

                 case 201: // Application
                	 for (ii=5; ii <= il; ++ii)
                	 {
                		 Application_[j] = str[ii];
                		 ++j;
                	 }
                	 strcpy(rampe1.Application,Application_);
                	 store_struct((char *) &rampe1, 8, sizeof(rampe1));
                 case 202: // Memo1
                	 for (ii=5; ii <= il; ++ii)
                	 {
                		 Application_[j] = str[ii];
                		 ++j;
                	 }
                	 strcpy(rampe1.Memo1,Application_);
                	 store_struct((char *) &rampe1, 8, sizeof(rampe1));
                 case 203: // Memo2
                	 for (ii=5; ii <= il; ++ii)
                	 {
                		 Application_[j] = str[ii];
                		 ++j;
                	 }
                	 strcpy(rampe1.Memo2,Application_);
                	 store_struct((char *) &rampe1, 8, sizeof(rampe1));

                 break;
                 }



            break;
        	case max_druck_eingabe: // "ll1"

        		printf("\n max Druck!");

        		i = get_in_buff_len(); // i = Länge des empfangenen Strings
                 k =0; j =0; l =0;

                 while(( i> 0 ) && ( k<255)) { // String einlesen get_char() UART2.c, i wird runtergezählt, max. Länge 255
                	 str[k++] = get_char();
                     i--;
                     }

                 str[k++] = ';';
                 str[k] =0;  // ??? Das ergibt keinen Sinn
                 printf("Das ist der String: %s", str);
                 i =0; k=0;
                 c = str[i++]; // c = str[1]

                 if (( c > 0x2f) && ( c < 0x39)) { // 2f 39 = "/0123456789"
                     e =c;			// e = str[1]
                     c = str[i++]; // c = str[2] i = 2
                     if ( c == ';') {
                         if( e == '1' ) {
                             f = get_ser_float(&str[i],&l);
                             rampe1.maximal_druck= f;
                             store_struct((char *) &rampe1, 8, sizeof(rampe1));
                             send_ACK(max_druck_eingabe, 1);
                             }
                         else {
                             send_float_val(max_druck_eingabe,rampe1.maximal_druck);
                             }
                         }
                     else {send_ACK(max_druck_eingabe, 0);
                         }
                     }
                 else {
                	 send_ACK(max_druck_eingabe, 0);
                     }
                 break;

            case get_block_zahl:  // "F"
                 i= find_offset();
                 i -= 1024;
                 send_int_val(get_block_zahl,i);
                 break;

            case del_speicher:  //  "K"
            	anzeige_send_int((int)600.0);
                 del_bereich(1024,1024);
                 anzeige_send_int((int)600.0);
                 break;

            case print_fwversion:  //  "V"
                 strcpy(dummy,"AV;");
                 sprintf(dummy2,"%d",fwVersion);
                 strcat(dummy,dummy2);
                 strcat(dummy,";");
                 strcat(dummy,"Z");
                 print_f_lf0(dummy);
                 break;

            case set_block:  //  G
                 i = get_in_buff_len();
                 k =0; j =0; l =0;

                 while(( i> 0 ) && ( k<255)) {
                     str[k++] = get_char();i--; }
                 i =0;k=0;
                  c = str[i++];
                  if ( c == ';') {
                    j=1;
                    i = get_ser_int(&str[j],&l);
                    i += 1024;
                    life_time_block=i;
                    in_life_time_buffer=0;
                    eep_get_block( i,life_time_buffer);
                    send_ACK(set_block, 1);

                  }
                  else {  send_ACK(set_block, 0);
                  }
              break;

              case get_end_druck:  //  H unvollständig?
                    i =0; 
                    j =0;
                    strcpy(dummy,"AH;");
                    print_f0(dummy); 

                    while( i ==0) {
                      l=0;
                      l  = life_time_buffer[j++];    l<<=8;
                      l |= life_time_buffer[j++];
                      if ( l != 0x0000ffff) {
                        sprintf(dummy,"%d;");
                        print_f0(dummy); 
                       }
                      else {i=1;
                      if ( j > 0x7f ) { i =1; }
                      }
                    }
                   strcpy(dummy,"Z");
                   print_f_lf0(dummy); 
              break;

              case set_max_fuell:  //  "I" Füllstand max und min setzen und lesen
                   if (password==1 ) {
                       //  password =0;
                       i = get_in_buff_len();
                       k =0; j =0; l =0;

                       while(( i> 0 ) && ( k<40)) {
                	       str[k++] = get_char();
                	       i--;
                           }
                       str[k] = ';';
                       c = str[j++];

                       if (( c > 0x2f) && ( c < 0x39)) {
                           e = c;
                           c = str[j++];
                           if (c == ';') {
                               if (e =='1') {                                    // Füllstand max und min setzen
                                   f = get_ser_float(&str[j],&l);
                                   rampe1.max_fuell_st= f;
                                   j =l+j;
                                   f = get_ser_float(&str[j],&l);
                                   rampe1.min_fuell_st= f;
                                   store_struct((char *) &rampe1, 8, sizeof(rampe1));
                                   //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                                   send_ACK(set_max_fuell, 1);
                                   }
                               else {                                            // Füllstand max und min lesen
                                   strcpy(dummy,"AH;");
                                   sprintf(dummy2,"%04.1f;",rampe1.max_fuell_st);
                                   strcat(dummy,dummy2);
                                   sprintf(dummy2,"%04.1f;Z",rampe1.min_fuell_st);
                                   strcat(dummy,dummy2);
                                   print_f_lf0(dummy);
                                   }
                               }
                           else {
                    	       send_ACK(set_max_fuell, 0);
                               }
                           }
                       else {
                	       send_ACK(set_max_fuell, 0);
                           }
                       }
                   else {
                       send_ACK(set_max_fuell, 0);  // kein Password
                       }
                   break;
            
            
              case set_conn:  //  n
                    //main_state=40;
                    //debug_printf("Connect\n");
                   // tasten_ende=1;
                   // connect =1;
                   break;

              case set_start_strom:  //  h
              i = get_in_buff_len();
              k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;
              c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c;   
                  c = str[j++];
                  if( c == ';') {
                     
                     if( e =='1') {
                            
                        rampe1.startstrom = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_start_strom, 1); 
                           
                       }
                     else {send_int_val(set_start_strom, rampe1.startstrom);}
                   }

                  else {  send_ACK(set_start_strom, 0); }
               }
            else {  send_ACK(set_start_strom, 0); }


          break;
             case set_rampe_offset:  //  i
              i = get_in_buff_len();
              k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;
              c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c;   
                  c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                       rampe1.offset_step = get_ser_int(&str[j],&l);
                       store_struct((char *) &rampe1, 8, sizeof(rampe1));
                       send_ACK(set_rampe_offset, 1); 
                      }
                      else {send_int_val(set_rampe_offset, rampe1.offset_step);}
                    }

                   else {  send_ACK(set_rampe_offset, 0); }
               }
              else {  send_ACK(set_rampe_offset, 0); }
          break;
 
          case set_netzwerk:  //  y
              i = get_in_buff_len();
              k =0; j =0; l =0;
              while(( i> 0 ) && ( k<255)) {dummy[k++] = get_char();i--; }
              i =0;k=0;
              c = dummy[i++];
              if (( c > 0x2f) && ( c < 0x39)) {
                 if ( c == '1') {
                    c = dummy[i++];
                    if( c == ';') {
                        c = dummy[i++];
                        while( c != ';') {    str[k++]= c;    c = dummy[i++]; }
                        str[k] =0;
                        strcpy(rampe1.ssid,dummy);
                        k =0;
                        c = dummy[i++];
                        while( c != ';')    {str[k++]= c;    c = dummy[i++];    }
                        str[k] =0;
                        strcpy(rampe1.pass,dummy);k =0;
                        rampe1.ip_adress[0] = get_ser_int( &dummy[i], (int*)& l);   i +=l;
                        rampe1.ip_adress[1] = get_ser_int( &dummy[i], (int*)& l);   i +=l;
                        rampe1.ip_adress[2] = get_ser_int( &dummy[i], (int*)& l);   i +=l;
                        rampe1.ip_adress[3] = get_ser_int( &dummy[i], (int*)& l);   i +=l;
                        c = dummy[i++];
                        while( c != ';') {    str[k++]= c;    c = dummy[i++];   }
                        str[k] =0;
                        strcpy(rampe1.kennung,str);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_netzwerk, 1); 
                       }
                    
                  else { send_ACK(set_netzwerk, 0); }
                }
              else {  strcpy(dummy,"A Netzwerk = ;");
                      strcpy(dummy,rampe1.ssid);  strcat(dummy,";");
                      strcat(dummy,rampe1.pass);  strcat(dummy,";");
                      itoa(rampe1.ip_adress[0],str,10);
                      strcat(dummy,str);  strcat(dummy,".");
                      itoa(rampe1.ip_adress[1],str,10);
                      strcat(dummy,str);  strcat(dummy,".");
                      itoa(rampe1.ip_adress[2],str,10);
                      strcat(dummy,str);  strcat(dummy,".");
                      itoa(rampe1.ip_adress[3],str,10);
                      strcat(dummy,str);  strcat(dummy,";");
                      strcat(dummy,rampe1.kennung); 
                      
                      strcat(dummy,";Z"); 
                      print_f_lf0(dummy); 
                }
              }
          break;

            case set_fuell_min:  //  w
                 f = get_oel_stand();
                 rampe1.fuell_min= adc_oel;
                 store_struct((char *) &rampe1, 8, sizeof(rampe1));
                 send_ACK(set_fuell_min, 1);
                 break;

            case set_fuell_max:  //  x
                 f = get_oel_stand();
                 rampe1.fuell_max = adc_oel;
                 store_struct((char *) &rampe1, 8, sizeof(rampe1));
                 send_ACK(set_fuell_max, 1);
                 break;

            case set_entlage1:  //  v
                 i= set_motor_entlage_rechts();
                 if( i ==1 ) { send_ACK(set_entlage1, 1);}
                 else { send_ACK(set_entlage1, 0);}
                 break;
        
            case ping:  //  p
                 send_ACK(ping, 1);
                 break;

            case set_zeit:  //  z
                 i = get_in_buff_len();
                 k =0; j =0; l =0;
                 while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                 str[k++]= ';' ;
                 str[k]=0;
                 c = str[j++];
                 if (( c > 0x2f) && ( c < 0x39)) {
                     e =c;
                     c = str[j++];
                     if ( c == ';') {
                         c = str[j++];
                         if (( c > 0x30) && ( c < 0x39)) {
                             c &= 0x0f;  d =c;
                             c = str[j++];
                             if( c == ';') {
                                 if( e == '1'){
                                     i = get_ser_int(&str[j],&l);
                                     rampe1.geschwind[d-1]= i;
                                     store_struct((char *) &rampe1, 8, sizeof(rampe1));
                                     //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                                     setup_m_gesch();
                                     send_ACK(set_zeit, 1);
                                     }
                                 else { send_int_val(set_zeit,rampe1.geschwind[d-1]); }
                                 }
                             else {send_ACK(set_zeit, 0);}
                             }
                         else {  send_ACK(set_zeit, 0); }
                         }
                     else {  send_ACK(set_zeit, 0); }
                     }
                 else {  send_ACK(set_zeit, 0); }
                 break;

            case set_end_druck:  //  d
                 i = get_in_buff_len();
                 k =0; j =0; l =0;
                 while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                 str[k++]= ';' ;
                 str[k]=0;
                 c = str[j++];
                 if (( c > 0x2f) && ( c < 0x39)) {
                     e = c;
                     c = str[j++];
                     if( c == ';') {
                         c = str[j++];
                         if (( c > 0x30) && ( c < 0x39)) {
                             c &= 0x0f;
                             d =c;
                             if ( e == '1') {
                                 c = str[j++];
                                 if( c == ';') {
                                     f = get_ser_float(&str[j],&l);
                                     rampe1.end_druck_rauf[d-1]= f;
                                     if ( d ==1) { rampe1.start_druck_rauf[0]=0;}
                                         if ( d < 9) { rampe1.start_druck_rauf[d]=f;}
                                         store_struct((char *) &rampe1, 8, sizeof(rampe1));
                                         //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                                         setup_m_gesch();
                                         send_ACK(set_end_druck, 1);
                                         }
                                     else {  send_ACK(set_end_druck, 0); }
                                     }
                                else {send_float_val(set_end_druck,rampe1.end_druck_rauf[d-1]); }
                                }
                            else {  send_ACK(set_end_druck, 0); }
                            }
                        else {  send_ACK(set_end_druck, 0); }
                        }
                    else {  send_ACK(set_end_druck, 0); }
                    break;
 
            case set_rueck_geschw:  //  r
                 i = get_in_buff_len();
                 k =0; j =0; l =0;
                 while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                 str[k]= ';' ;
                 c = str[j++];
                 if (( c > 0x2f) && ( c < 0x39)) {
                     e = c;
                     c = str[j++];
                     if( c == ';') {
                         c = str[j++];
                         if (( c > 0x30) && ( c < 0x39)) {
                             c &= 0x0f;
                             d =c;
                             if( e =='1') {
                                 c = str[j++];
                                 if( c == ';') {
                                     f = get_ser_float(&str[j],&l);
                                     rampe1.geschwindr[d-1]= f;
                                     store_struct((char *) &rampe1, 8, sizeof(rampe1));
                                     send_ACK(set_rueck_geschw, 1);
                                     }
                                 else {  send_ACK(set_rueck_geschw, 0); }
                                 }
                             else {send_float_val(set_rueck_geschw, rampe1.geschwindr[d-1]);}
                             }
                         else {  send_ACK(set_rueck_geschw, 0); }
                         }
                     else {  send_ACK(set_rueck_geschw, 0); }
                     }
                 else {  send_ACK(set_rueck_geschw, 0); }
                 break;

            case set_max_step:  //  b
                 if (password==1 ) {
                     //  password =0;
                     i = get_in_buff_len();
                     k =0; j =0; l =0;
                     while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                     str[k] = ';';
                     c = str[j++];
                     if (( c > 0x2f) && ( c < 0x39)) {
                         e = c;
                         c = str[j++];
                         if( c == ';') {
                             if( e =='1') {
                                 i = get_ser_int(&str[j],&l);
                                 rampe1.max_steps= i;
                                 j =l+j;
                                 i = get_ser_int(&str[j],&l);
                                 rampe1.offset_step= i;
                                 store_struct((char *) &rampe1, 8, sizeof(rampe1));
                                 //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                                 send_ACK(set_max_step, 1);
                                 }
                             else {
                                 strcpy(dummy,"Ab;");
                                 i = rampe1.max_steps;
                                 sprintf(dummy2,"%d",i);
                                 strcat(dummy,dummy2);
                                 strcat(dummy,";");
                                 i = rampe1.offset_step;
                                 sprintf(dummy2,"%d",i);
                                 strcat(dummy,dummy2);
                                 strcat(dummy,";");
                                 sprintf(dummy2,"%04.1f",max_druck);
                                 strcat(dummy,dummy2);
                                 strcat(dummy,";Z");
                                 print_f_lf0(dummy);
                                 }
                            }
                        else { send_ACK(set_max_step, 0); }
                        }
                    else {  send_ACK(set_max_step, 0);  }
                    }
                else {  send_ACK(set_max_step, 0);  }
                break;
 
          case start:  //  s
               i = get_in_buff_len();
               k =0; j =0; l =0;
               while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
               str[k] = ';';
                  c = str[j++];
                  if( c == ';') {
                    rs232_druck = get_ser_float(&str[j],&l);
                    printf("\nrs232_druck %f", rs232_druck);
                     send_ACK(start, 1); 
                     i= find_offset();
                   }
                  
                  else {send_ACK( start, 0); }
              if (( main_state == 30) || (main_state ==40)) {   
                  main_state = 0;
                  tasten_ende =1;
                  }
          break;

          case stop:  //  S
                    if (( main_state== 8)||(main_state ==0) || (main_state==1)) {
                        l = sdo_receive(1,0x6040,0);
                        wait_ms(2);
                        l |=0x0100;
                        i= sdo_send (1,0x6040, 0,l,2); 
                        wait_ms(2);
                        l = sdo_receive(1,0x6040,0);
                        wait_ms(2);
                        set_motor_power_off();
                
                        main_error =6;
                        main_state = 20;
                        }
                    if (( main_state== 9)||(main_state ==10)) {
                        main_error = 6;
                        main_state = 21;
                      }

              send_ACK(stop, 1);
          break;

          case set_beschl_rauf:  //  c
              if (password==1 ) {
                   //password =0;
                  i = get_in_buff_len();
                  k =0; j =0; l =0;
                  while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                  str[k] = ';';
                   c = str[j++];
                   if (( c > 0x2f) && ( c < 0x39)) {
                     e =c;
                          c = str[j++];
                          if ( c == ';') {
                              c = str[j++];
                              if (( c > 0x30) && ( c < 0x39)) {
                                d = c & 0x0f;
                                c=str[j++];
                                if ( e=='1') {
                                i = get_ser_int(&str[j],&l);
                                rampe1.rampe_start[d-1]= i;
                                store_struct((char *) &rampe1, 8, sizeof(rampe1));
                               // set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                                send_ACK(set_beschl_rauf, 1);
                               }
                              else { send_int_val(set_beschl_rauf,rampe1.rampe_start[d-1]);   }
                             }
                           else {send_ACK(set_beschl_rauf, 0); }
                        }
                     else { send_ACK(set_beschl_rauf, 0); }
                   } 
               else { send_ACK(set_beschl_rauf, 1); }
              }
           else { send_ACK(set_beschl_rauf, 0); }
          break;

          case set_beschl_runter:  //  e
              if (password==1 ) {
                  // password =0;
                  i = get_in_buff_len();
                  k =0; j =0; l =0;
                  while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                  str[k++] = ';';
                  str[k]=0;
                   c = str[j++];
                   if (( c > 0x30) && ( c < 0x39)) {
                      e =c;
                      c = str[j++];
                      if ( c == ';') {
                        c = str[j++];
                        if (( c > 0x30) && ( c < 0x39)) {
                          d = c & 0x0f;
                          c = str[j++];
                          if ( e == '1'){
                                i = get_ser_int(&str[j],&l);
                                rampe1.rampe_stop[d-1]= i;
                                store_struct((char *) &rampe1, 8, sizeof(rampe1));
                                //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                                send_ACK(set_beschl_runter, 1);
                               }
                              else {  send_ACK(set_beschl_runter, 0); }
                             }
                           else {send_int_val(set_beschl_runter,rampe1.rampe_stop[d-1]); }
                        }
                     else {send_ACK(set_beschl_runter, 0); }
                   } 
               else { send_ACK(set_beschl_runter, 0); }
              }
           else { send_ACK(set_beschl_runter, 0); }
          break;

          case get_druck:  //  f
            //    send_float_val(get_druck, float val) 
                    strcpy(dummy,"Ablaufzeitwerte= ;");
                    sprintf(dummy2,"%04.1f;",ist_druck);

                    strcat(dummy,dummy2);
                    sprintf(dummy2,"%04.1f;",Motortemperatur);
                    /*
                    strcat(dummy,dummy2);
                    sprintf(dummy2,"%02d;",main_state);
                    strcat(dummy,dummy2);
                    sprintf(dummy2,"%04.1f;",oel_stand);
                    strcat(dummy,dummy2);
                    sprintf(dummy2,"%d;",soll_zeit);
                    strcat(dummy,dummy2);
                    sprintf(dummy2,"%d;",motor_timer);
                    strcat(dummy,dummy2);
                    */
                    strcat(dummy,"Z");
                    print_f_lf0(dummy); 
          break;

          case passwort:  //  a
              //password=0;
              i = get_in_buff_len();
              k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k] =0;
              i = strcmp(rampe1.pass_word, str);
              if ( i==0 ){ password = 1 ;}
              send_ACK(passwort, 1);
          break;

          case set_max_druck_geber:  //  M
               if (password==1 ) {
                   //password =0;
                 i = get_in_buff_len();
                 k =0; j =0; l =0;
                  while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                  str[k++]= ';';
                  str[k] =0;
                  j=2;
                   f = get_ser_float(&str[j],&l);
                   rampe1.max_druck_sensor= f;

 //                   f = get_ser_float(&str[j],&l);
                   f1 =  get_adc_val();
 //f1 = 4.5;
                   f2 = rampe1.offset_sensor;
//f2 = 0.02;
                   f2 = f1 -f2;

                   rampe1.m= f/f2;
                   rampe1.b = f-(rampe1.m*f1);
                   store_struct((char *) &rampe1, 8, sizeof(rampe1));
                   //debug_printf(" m = %f \n",rampe1.m);
                   //debug_printf(" b = %f \n",rampe1.b);
                  store_struct((char *) &rampe1, 8, sizeof(rampe1)); 
                   send_ACK(set_max_druck_geber, 1);
                }
              
              else { send_ACK(set_max_druck_geber, 1); }
          break;

          case set_max_druck:  //  m
               if (password==1 ) {
                   //password =0;
                 i = get_in_buff_len();
                 k =0; j =0; l =0;
                  while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                  str[k++]= ';';
                  str[k] =0;
                   f = get_ser_float(&str[j],&l);
                  // f1 =  get_adc_val();
                   f1 = 4.5;
                   f2 = rampe1.offset_sensor;
                   f2 = f1 -f2;

                   rampe1.m= f/f1;
                   rampe1.b = f-(rampe1.m*f1);
                   

//!!!!!!!!!!!
                   store_struct((char *) &rampe1, 8, sizeof(rampe1)); 
                   //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                   
                  send_ACK(set_max_druck, 1);
                }
              else { send_ACK(set_beschl_rauf, 1); }
          break;

          case set_offset:  //  o
               if (password==1 ) {
                   //password =0;
                   f = get_adc_val();
                  
                   rampe1.offset_sensor=f;
                   store_struct((char *) &rampe1, 8, sizeof(rampe1));
                   //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                    
                  send_ACK(set_offset, 1);
                }
              else { send_ACK(set_offset, 1); }
          break;

          case set_steigung:  //  t
                f = get_adc_val();
                f = 4.5;
                i = get_in_buff_len();
                k =0; j =0; l =0;
                while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                str[k] = ';';
                k =0;
                if ( str[k++] == '1'){
                    if ( str[k++] == ';') {

                     }
                     else { send_ACK(set_steigung, 0); }

                 }
                else {
                }
                   
          break;

          case set_beschl_rauf_r:  //  D
               if (password==1 ) {
                 //  password =0;
                  i = get_in_buff_len();
                  k =0; j =0; l =0;
                  while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                  str[k] = ';';
                   c = str[j++];
                   if (( c > 0x30) && ( c < 0x39)) {
                      e =c;
                      c = str[j++];
                      if ( c == ';') {
                        c = str[j++];
                        if (( c > 0x30) && ( c < 0x39)) {
                          d = c & 0x0f;
                          c = str[j++];
                          if ( e =='1') {
                                i = get_ser_int(&str[j],&l);
                                rampe1.rampe_start_r[d-1]= i;
                                store_struct((char *) &rampe1, 8, sizeof(rampe1));
                                //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                                send_ACK(set_beschl_rauf_r, 1);
                               }
                              else {send_int_val(set_beschl_rauf_r,rampe1.rampe_start_r[d-1]);  }
                             }
                           else {send_ACK(set_beschl_rauf_r, 0); }
                        }
                     else { send_ACK(set_beschl_rauf_r, 0); }
                   } 
               else { send_ACK(set_beschl_rauf_r, 0); }
              }
           else { send_ACK(set_beschl_rauf_r, 0); }

          break;

          case set_beschl_runter_r:  //  E
               if (password==1 ) {
                 //  password =0;
                  i = get_in_buff_len();
                  k =0; j =0; l =0;
                  while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                  str[k] = ';';
                   c = str[j++];
                   if (( c > 0x30) && ( c < 0x39)) {
                     e =c;
                     c = str[j++];
                     if ( c == ';') {
                       c = str[j++];
                       if (( c > 0x30) && ( c < 0x39)) {
                         d = c & 0x0f;
                         c = str[j++];
                         if( e == '1') {
                                i = get_ser_int(&str[j],&l);
                                rampe1.rampe_stop_r[d-1]= i;
                                store_struct((char *) &rampe1, 8, sizeof(rampe1));
                                //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                                send_ACK(set_beschl_runter_r, 1);
                               }
                              else {send_int_val(set_beschl_runter_r,rampe1.rampe_stop_r[d-1]); }
                             }
                           else {send_ACK(set_beschl_runter_r, 0); }
                        }
                     else {send_ACK(set_beschl_runter_r, 0);   }
                   } 
               else { send_ACK(set_beschl_runter_r, 1); }
              }
           else { send_ACK(set_beschl_runter_r, 0); }

          break;

          case set_strom:  //  g
                if (password==1 ) {
                  // password =0;
                  i = get_in_buff_len();
                  k =0; j =0; l =0;
                  while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                  str[k] = ';';
                   c = str[j++];
                   if (( c > 0x2f) && ( c < 0x39)) {
                      e =c;
                      c = str[j++];
                      if ( c == ';') {
                         c = str[j++];
                         if (( c > 0x30) && ( c < 0x39)) {
                           d = c & 0x0f;
                           c= str[j++];
                           if ( e =='1' ) {
                                i = get_ser_int(&str[j],&l);
                                rampe1.strom[d-1]= i;
                                store_struct((char *) &rampe1, 8, sizeof(rampe1));
                                //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                                send_ACK(set_strom, 1);
                               }
                              else { send_int_val(set_strom,rampe1.strom[d-1]); }
                             }
                           else {send_ACK(set_strom, 0); }
                        }
                     else {  send_ACK(set_strom, 1);  }
                   } 
               else { send_ACK(set_strom, 1); }
              }
           else { send_ACK(set_strom, 0); }

          break;

          case set_strom_r:  //  j
                if (password==1 ) {
                   //password =0;
                  i = get_in_buff_len();
                  k =0; j =0; l =0;
                  while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
                  str[k] = ';';
                   c = str[j++];
                   if (( c > 0x2f) && ( c < 0x39)) {
                        e = c;
                        c = str[j++];
                          if ( c == ';') {
                              c = str[j++];
                              if (( c > 0x30) && ( c < 0x39)) {
                                d = c & 0x0f;
                                c= str[j++];
                                if ( e=='1') {
                                  i = get_ser_int(&str[j],&l);
                                  rampe1.strom_r[d-1]= i;
                                  store_struct((char *) &rampe1, 8, sizeof(rampe1));
                                  //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                                  send_ACK(set_strom_r, 1);
                                }
                              else {send_int_val(set_strom,rampe1.strom_r[d-1]);  }
                             }
                           else {send_ACK(set_strom_r, 0); }
                        }
                     else { send_ACK(set_strom_r, 0); }
                   } 
               else { send_ACK(set_strom_r, 1); }
              }
           else { send_ACK(set_strom_r, 0); }

          break;

          case set_delay:  //  k
              i = get_in_buff_len();
              k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k++]= ';' ;
              str[k]=0;
              c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c;   
                  c = str[j++];
                  if( c == ';') {
                      c = str[j++];
                      if (( c > 0x30) && ( c < 0x39)) {
                          c &= 0x0f;
                          d =c;
                          if ( e == '1') {
                              c = str[j++];
                              if( c == ';') {
                                  i = get_ser_int(&str[j],&l); 
                                  rampe1.delay[d-1]= i;
                                  store_struct((char *) &rampe1, 8, sizeof(rampe1));
                                  //set_eep_block(eep_adr_start_prozess1,( short *) &rampe1,sizeof(rampe1));
                                  setup_m_gesch(); 
                                  send_ACK(set_delay, 1);  
                                }
                              else {  send_ACK(set_delay, 0); }
                            }
                          else {send_int_val(set_delay,rampe1.delay[d-1]); }
                       }
                      else {  send_ACK(set_delay, 0); }
                   }
                  else {  send_ACK(set_delay, 0); }
                }
              else {  send_ACK(set_delay, 0); }
 
          break;


          case print_druck:  // P
                i = get_in_buff_len();
                c = get_char(); // Rampe Nr.
                if (( c > 0x30) && ( c < 0x39)) {
                    c &= 0x0f;
                    strcpy(dummy,"A Startdruck = ;");
                    f = rampe1.start_druck_rauf[c-1];
                    printf("%04.1f",f);
                    sprintf(dummy2,"%04.1f",f);
                    strcat(dummy,dummy2);
                    strcat(dummy,";");
                    strcat(dummy," Enddruck = ;");
                    f = rampe1.end_druck_rauf[c-1];
                    sprintf(dummy2,"%04.1f",f);
                    strcat(dummy,dummy2);
                    strcat(dummy,";");
                    strcat(dummy," geschwind = ;");
                    i = rampe1.geschwind[c-1];
                    sprintf(dummy2,"%d",i);
                    strcat(dummy,dummy2);
                    strcat(dummy,";");
                    strcat(dummy,"Z");
                    print_f_lf0(dummy); 

                 }
                else {send_ACK(print_druck, 0); }  
          break;

          case  print_rampe_vor:  //  B
                i = get_in_buff_len();
                c = get_char();
                if (( c > 0x30) && ( c < 0x39)) {
                    c &= 0x0f;
                    strcpy(dummy,"Startrampe = ;");
                    i = rampe1.rampe_start[c-1];
                    sprintf(dummy2,"%d",i);
                    strcat(dummy,dummy2);
                    strcat(dummy,";Stoprampe = ;");
                    i = rampe1.rampe_stop[c-1];
                    sprintf(dummy2,"%d",i);
                    strcat(dummy,dummy2);
                    strcat(dummy,";");
                    strcat(dummy,"Z");
                    print_f_lf0(dummy);

                  }
              else{send_ACK(print_rampe_vor, 0);}
                     
          break;

          case print_rampe_zurueck:  //  C
                i = get_in_buff_len();
                c = get_char();
                if (( c > 0x30) && ( c < 0x39)) {
                    c &= 0x0f;
                    strcpy(dummy,"Startrampe Ruecklauf= ;");
                    i = rampe1.rampe_start_r[c-1];
                    sprintf(dummy2,"%d",i);
                    strcat(dummy,dummy2);
                    strcat(dummy,";Stoprampe Ruecklauf= ;");
                    i = rampe1.rampe_stop_r[c-1];
                    sprintf(dummy2,"%d",i);
                    strcat(dummy,dummy2);
                    strcat(dummy,";Z");
                   
                    print_f_lf0(dummy); 
                }
              else{send_ACK(print_rampe_zurueck, 0);}
          break;
          
 //#############################################################################################
 /* #define set_Oszi           		'Y'
#define set_Oszi_UT_Toleranz	'X'
#define set_Oszi_OT_Toleranz	'U'
#define set_Oszi_Schrittweite	'W'*/
/*

          case set_Oszi:  //  "Y"
              i = get_in_buff_len();
              k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;
              c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c;
                  c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.Oszi = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_Oszi, 1);
                       }
                     else {send_int_val(set_Oszi, rampe1.Oszi);}
                   }
                  else {  send_ACK(set_Oszi, 0); }
               }
            else {  send_ACK(set_Oszi, 0); }
          break;

          case set_Oszi_UT_Toleranz:  //  "X"
              i = get_in_buff_len();
              k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;
              c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c;
                  c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.Oszi_UT_Toleranz = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_Oszi_UT_Toleranz, 1);
                       }
                     else {send_int_val(set_Oszi_UT_Toleranz, rampe1.Oszi_UT_Toleranz);}
                   }
                  else {  send_ACK(set_Oszi_UT_Toleranz, 0); }
               }
            else {  send_ACK(set_Oszi_UT_Toleranz, 0); }
          break;


          case set_Oszi_OT_Toleranz:  //  "U"
              i = get_in_buff_len();
              k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;
              c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c;
                  c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.Oszi_OT_Toleranz = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_Oszi_OT_Toleranz, 1);
                       }
                     else {send_int_val(set_Oszi_OT_Toleranz, rampe1.Oszi_OT_Toleranz);}
                   }
                  else {  send_ACK(set_Oszi_OT_Toleranz, 0); }
               }
            else {  send_ACK(set_Oszi_OT_Toleranz, 0); }
          break;


          case set_Oszi_Schrittweite:  //  "W"
              i = get_in_buff_len();
              k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;
              c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c;
                  c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.Oszi_Schrittweite = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_Oszi_Schrittweite, 1);
                       }
                     else {send_int_val(set_Oszi_Schrittweite, rampe1.Oszi_Schrittweite);}
                   }
                  else {  send_ACK(set_Oszi_Schrittweite, 0); }
               }
            else {  send_ACK(set_Oszi_Schrittweite, 0); }
          break;

          case set_Drehrichtung:  //  "T"
              i = get_in_buff_len();
              k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;
              c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c;
                  c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.Drehrichtung = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_Drehrichtung, 1);
                       }
                     else {send_int_val(set_Drehrichtung, rampe1.Drehrichtung);}
                   }
                  else {  send_ACK(set_Drehrichtung, 0); }
               }
            else {  send_ACK(set_Drehrichtung, 0); }
          break;

          case set_StallTol:  //  "R"
              i = get_in_buff_len();
              k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;
              c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c;
                  c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.StallTol = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_StallTol, 1);
                       }
                     else {send_int_val(set_StallTol, rampe1.StallTol);}
                   }
                  else {  send_ACK(set_StallTol, 0); }
               }
            else {  send_ACK(set_StallTol, 0); }
          break;

/*
#define set_SS1					'J'
#define set_SR1					'L'
#define set_SS2					'N'
#define set_SR2					'O'
#define set_SS3					'Q'
#define set_SR3					'R'
#define set_SSLimit				'q'
*/
          /*
          case set_SS1:  //  "J"
              i = get_in_buff_len();k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c; c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.SS1 = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_SS1, 1);
                       }
                     else {send_int_val(set_SS1, rampe1.SS1);}
                   }
                  else {  send_ACK(set_SS1, 0); }
               }
            else {  send_ACK(set_SS1, 0); }
          break;
          */
          case set_SR1:  //  "J"
              i = get_in_buff_len();k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c; c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.SR1 = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_SR1, 1);
                       }
                     else {send_int_val(set_SR1, rampe1.SR1);}
                   }
                  else {  send_ACK(set_SR1, 0); }
               }
            else {  send_ACK(set_SR1, 0); }
          break;
          case set_SS2:  //  "J"
              i = get_in_buff_len();k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c; c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.SS2 = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_SS2, 1);
                       }
                     else {send_int_val(set_SS2, rampe1.SS2);}
                   }
                  else {  send_ACK(set_SS2, 0); }
               }
            else {  send_ACK(set_SS2, 0); }
          break;
          case set_SR2:  //  "J"
              i = get_in_buff_len();k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c; c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.SR2 = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_SR2, 1);
                       }
                     else {send_int_val(set_SR2, rampe1.SR2);}
                   }
                  else {  send_ACK(set_SR2, 0); }
               }
            else {  send_ACK(set_SR2, 0); }
          break;
          case set_SS3:  //  "J"
              i = get_in_buff_len();k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c; c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.SS3 = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_SS3, 1);
                       }
                     else {send_int_val(set_SS3, rampe1.SS3);}
                   }
                  else {  send_ACK(set_SS3, 0); }
               }
            else {  send_ACK(set_SS3, 0); }
          break;
          case set_SR3:  //  "J"
              i = get_in_buff_len();k =0; j =0; l =0;
              while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
              str[k]= ';' ;c = str[j++];
              if (( c > 0x2f) && ( c < 0x39)) {
                  e = c; c = str[j++];
                  if( c == ';') {
                     if( e =='1') {
                        rampe1.SR3 = get_ser_int(&str[j],&l);
                        store_struct((char *) &rampe1, 8, sizeof(rampe1));
                        send_ACK(set_SR3, 1);
                       }
                     else {send_int_val(set_SR3, rampe1.SR3);}
                   }
                  else {  send_ACK(set_SR3, 0); }
               }
            else {  send_ACK(set_SR3, 0); }
          break;
          case set_SSLimit:  //  "J"
               i = get_in_buff_len();k =0; j =0; l =0;
               while(( i> 0 ) && ( k<40)) {str[k++] = get_char();i--; }
               str[k]= ';' ;c = str[j++];
               if (( c > 0x2f) && ( c < 0x39)) {
                   e = c; c = str[j++];
                   if( c == ';') {
                      if( e =='1') {
                         rampe1.SSLimit = get_ser_int(&str[j],&l);
                         store_struct((char *) &rampe1, 8, sizeof(rampe1));
                         send_ACK(set_SSLimit, 1);
                        }
                      else {send_int_val(set_SSLimit, rampe1.SSLimit);}
                    }
                   else {  send_ACK(set_SSLimit, 0); }
                }
             else {  send_ACK(set_SSLimit, 0); }
           break;


          default:
                send_ACK(b,0);
          break;

      }
    reset_in_0_buff( ); 
    }
 
}

/*
 *  Funktions
 */
void send_ACK(char c, char art) {
 char str[10];
  str[0]=ESC;
  str[1]= c;
  str[2]= ';';
  if ( art ==1) { str[3]= 'O'; str[4]= 'K';}
  else          { str[3]= 'K'; str[4]= 'O';}
  str[5]=';';
  str[6]= END;
  str[7]=0;
  print_f_lf0(str);
}
int get_ser_int( char * buff, int* adress) {
  char c;
  char str[40];
  int k;
  int ende;
  int i;
  int j;
  ende =0;
  k =0;
  j =0;
  while(ende ==0) {
      c = *buff++;
      if (( c > 0x2f ) && ( c < 0x3a)) {
         str[k++] = c;
         j++;
       
       }
      if ( c == ';' ) { ende =1; j++; }

   }
   ( * adress ) = j;
  str[k] =0;
  i = atoi(str);
 return i;
}

float get_ser_float( char * buff, int * adress) {
  
  char str[40];
  int ii = 0;
  for (ii = 0;ii<40;++ii){str[ii]="";}
  str[0]="";
  char c;
  int ende ;
  float f;
  int k,i;
  ende =0;
  k =0;
  while ( ende ==0) {
      
          c = * buff++;
          if (( c > 0x2f ) && ( c < 0x3a)) {
             str[k++] = c;
             
            }
          if ( c == '.' ) { str[k++] = '.';  }
          if ( c == ',' ) { str[k++] = '.';  }
          if ( c == ';' ) { ende =1; k++;}
              
      }
  ( * adress ) = k;
 
  str[k] =0;
  c=0;
  for ( i =0; i<k; i++) {
      if( str[i] == '.' ) { c=1; }
   }
   if( c ==0) { 
     str[k++] = '.';
     str[k++] = '0';
     str[k] =0;
   }
      f = atof(str); 
 
  return f;
}


void  send_int_val(char befehl,int val){
char str[20];
char str1[20];
unsigned int val1;
  val1 = val;
  //val &= 0x0000ffff;
  sprintf(str1,"%d",val);
  str[0]= ESC;
  str[1] = befehl;
  str[2]=';';
  str[3] = 0;
  strcat(str,str1);
  strcat(str,";Z");
  print_f_lf0(str);
 
}



void send_float_val(char befehl, float val) {
char str[20];
char str1[20];
  sprintf(str1,"%04.1f",val);
  str[0]= ESC;
  str[1] = befehl;
  str[2] = ';';
  str[3] = 0;
  strcat(str,str1);
  strcat(str,";Z");
  print_f_lf0(str); 
  
}


int toString(char a[]) {
  int c, sign, offset, n;

  if (a[0] == '-') {  // Handle negative integers
    sign = -1;
  }

  if (sign == -1) {  // Set starting position to convert
    offset = 1;
  }
  else {
    offset = 0;
  }
  offset = 0;
  n = 0;

  for (c = offset; a[c] != '\0'; c++) {
    n = n * 10 + a[c] - '0';
  }

  if (sign == -1) {
    n = -n;
  }

  return n;
}


