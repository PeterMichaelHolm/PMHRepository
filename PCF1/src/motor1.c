//#include <__cross_studio_io.h>
#include "CanBusSDO.h"
#include "globdata.h"
#include "main.h"

#define teiler      6

extern int timer3;
extern void wait_ms(int msec);
extern void debug_inf(char * text);

volatile int Oszi_OT_Multi;
volatile int Oszi_UT_Multi;
volatile int Oszi_UT_Temp;
volatile int Oszi_OT_Temp;
volatile int val;
volatile int val1;
volatile int V1;

int probefahrt(int zeit); // not used anymore
// int auto_setup(void); // not used
int set_motor_power_on(void);
int set_motor_power_off(void);
int set_motor_entlage_links(void);
int set_motor_entlage_rechts(void);
int set_motor_stop(void);
int set_motor_gesch(int val );
int set_rampe_rauf(void);
int set_rampe_runter(void);
int set_rampe_rauf_r(void);
int set_rampe_runter_r(void);
int set_position_r(int Position);

int set_motor_startpos(void);  
int set_mode(unsigned char val); 
int set_position(void) ;
int set_strom_ru(void); 
int set_strom(void) ;
int wert1;
int wert2;
int wert3;
int wert4;
int wert5;

void wait_ms1(int msec) {
  //timer_m=0;
  //while(timer_m < msec) {
 // }
  wait_ms(msec);
 }
int probefahrt(int zeit) {
int i,k,l;
int erg =0;
  init_sdo();
  
  set_motor_power_on();
  wait_ms(2);

 
  sdo_send (1,0x6083, 0,10,4); 
  wait_ms1(2);
  i = sdo_reseive(1,0x6083,0);
  wait_ms1(2);
 
   i = sdo_reseive(1,0x3202,0);
   
 //i |= 0x00000001;
   i &= 0x0000000e;
  sdo_send (1,0x3202, 0,i,4); 
  i = sdo_reseive(1,0x3202,0);
   

  i = sdo_reseive(1,0x2031,0);
   
   i= 3500;
  sdo_send (1,0x2031, 0,i,4); 
  i = sdo_reseive(1,0x2031,0);
   

  sdo_send (1,0x6060, 0,0x02,1); 
  wait_ms(4);
  i= sdo_reseive(1,0x6061,0);
  if ((i & 0xff) == 0x02) {
      
      sdo_send (1,0x6042,0,900,2);
   
      wait_ms1(10);
      sdo_send (1,0x6040,0,0x0006,2);
      wait_ms1(10);
      i= sdo_reseive(1,0x6041,0);
      i &= 0x03ff;
      
     
      //if (i == 0x0221) {
        //  debug_printf(" Fahrt mit vorgwählter Geschwindigkeit \n");
          wait_ms1(10);
          sdo_send (1,0x6040,0,0x0007,2);
          wait_ms1(10);
          i= sdo_reseive(1,0x6041,0);
          i&= 0x03ff;
         
            sdo_send (1,0x6040,0,0x000f,2);
            wait_ms1(2000);

           
            i =10;
            while(i <910) {
                wait_ms1(100);
                i +=10;
                
                sdo_send (1,0x6042,0,i,2);
            }
            while(i >20) {
                wait_ms1(100);
                i -=10;
                sdo_send (1,0x6042,0,i,2);
               
            }
           
            erg =1;
            sdo_send (1,0x6040,0,0x0000,2);
          }
     // }
   //}
 return erg;
}

int set_motor_power_on(void) {
 int erg = 0;
 int i;
 int j;
 init_sdo();
  i = sdo_send (1,0x6060, 0,0x01,1);
  wait_ms1(2);
  if ( i==1) {  
      i= sdo_reseive(1,0x6041,0);
      wait_ms1(2);
      i= sdo_send (1,0x6040, 0,0x0080,2); 
      if ( i==1) {
         wait_ms1(2);
         i= sdo_send (1,0x2031, 0,0x834,4);
         if ( i==1) {
            i = sdo_send (1,0x6040, 0,0x0006,2); 
            wait_ms1(2);
            if ( i==1) {
              i= sdo_reseive(1,0x6041,0);
              i &= 0x000f;
              if ( i == 0x001) { erg = 1;}
            }
          }
       
      }
    }
 if ( erg ==1) {
    i = sdo_reseive(1,0x3202,0);
    wait_ms1(2);
   // debug_printf("object 3202= %x \n",i);
    i |= 0x00000001;
    i &= 0xfffffffe;
    sdo_send (1,0x3202, 0,i,4); // Close loop off
    wait_ms1(2);
 }
 return erg;
 }

int set_motor_power_off(void) {
int i;
 int erg = 0;
 //i = sdo_reseive(1,0x6041,0);
 //wait_ms1(2);
 i = sdo_send (1,0x6040, 0,0,2);

  if( i==1) { erg =1; }
  wait_ms1(2);
 return erg;
 }


int set_motor_entlage_links(void) {
int erg = -1;
int j,k,i,z,l;
int ende;
for (i=1; i<=30;++i){printf("x");}printf("\n");

printf("111\n");
if (set_motor_power_on() ==1) {
//#ifdef debug_print
  printf("222\n");
//#endif
    i = sdo_reseive(1,0x3202,0);
    wait_ms1(2);
   // debug_printf("object 3202= %x \n",i);
    i |= 0x00000001;
    i &= 0xfffffffe;
    sdo_send (1,0x3202, 0,i,4); // Close loop
    wait_ms1(2);
    j = sdo_reseive(1,0x6065,0); // Strom auslesen
    wait_ms1(2);
    i = sdo_send (1,0x6065, 0,50,4);
    wait_ms1(2);
    j = sdo_reseive(1,0x6065,0); // Strom auslesen
    wait_ms1(2);
    j = sdo_reseive(1,0x2031,0); // Strom auslesen
    j /= teiler;
    wait_ms1(2);
    j = rampe1.startstrom;
    i = sdo_send (1,0x2031, 0,j,4); i <<=1; // Strom 210 mA
    wait_ms1(2);
    k = sdo_reseive(1,0x2031,0);
    wait_ms1(2);
    l= sdo_send (1,0x607e, 0,0x00,1);
    wait_ms1(2);

    if ( k==j) { j =1;
    printf("333\n");
    	j|= sdo_send (1,0x6060, 0,0x02,1);     j <<=1;
        wait_ms1(2);
        j|= sdo_send (1,0x6042, 0,100,2);      j <<=1;
        wait_ms1(2);
        j|= sdo_send (1,0x6040, 0,0x0006,2);     j <<=1;
        wait_ms1(2);
        j|= sdo_send (1,0x6040, 0,0x0007,2);     j <<=1;
        wait_ms1(2);
        j|= sdo_send (1,0x6040, 0,0x000f,2);     j <<=1;
        wait_ms1(2);
        ende =0;
        z=0;
         
         l =  sdo_reseive(1,0x6063,0);
          wait_ms1(10);
        while (ende ==0) {
           k =  sdo_reseive(1,0x6063,0); // fehler auslesen
//           debug_printf("Position = %x  \n",k);
           wait_ms1(2);
            z =  sdo_reseive(1,0x1001,0); // fehler auslesen
//           debug_printf("fehler = %x  \n",z);
           if(k == l ) { ende =1; }
           else {l = k;}
           wait_ms1(20);
           z++;
           if(z > 3000) { j=0; ende =1; }
        }
        printf("444\n");

        j|= sdo_send (1,0x6040, 0,0x0000,2);
        wait_ms1(2);
        if( j ==   0x3f) { erg =1; }
        l = sdo_reseive(1,0x6064,0);
        entlage_links= l;
        if ( entlage_links > 0 ) { entlage_links -= 1000;}
        else { entlage_links +=1000;}

        printf("Position = %d \n",l);

        wait_ms1(2);

        sdo_send (1,0x607c, 0,l,4);
        wait_ms1(2);

        l = sdo_reseive(1,0x6063,0);
//  debug_printf("\n Null-Position = %d \n",l);
        my_position=l;
        


    }  
  }
 
 return erg;
 }

int set_motor_entlage_rechts(void) {
// Referenzierungsfahrt

int erg = -1;
int j,k,i,z,l,u;
int ende;
int time_x = 2;
 if (set_motor_power_on() ==1) {

	 i = sdo_reseive(1,0x3202,0);
    wait_ms1(time_x);
   // debug_printf("object 3202= %x \n",i);
    i |= 1;

    sdo_send (1,0x3202, 0,i,4); // Closed loop

    wait_ms1(time_x);

    j = sdo_reseive(1,0x6065,0); // Following Error Window , maximaler Schleppfehler, Default 100h (256 dez)
    wait_ms1(time_x);
    i = sdo_send (1,0x6065, 0,256,4);
    wait_ms1(time_x);
    j = sdo_reseive(1,0x6065,0);
    wait_ms1(time_x);

    j = sdo_reseive(1,0x2031,0); // Maximum Current
    j /= teiler;
    wait_ms1(time_x);
    j = rampe1.startstrom;
    i = sdo_send (1,0x2031, 0,j,4); i <<=1; // Strom 210 mA







    wait_ms1(20);



    k = sdo_reseive(1,0x2031,0);
    wait_ms1(time_x);

    //printf("\n\n Drehrichtung xxx %d", Drehrichtung);
  if(rampe1.Drehrichtung == 0){l= sdo_send (1,0x607e, 0,0x00,1);wait_ms1(2);printf("Drehrichtung HP1\n");}
  if(rampe1.Drehrichtung == 1){l= sdo_send (1,0x607e, 0,0xc0,1);wait_ms1(2);printf("Drehrichtung HP2\n");}


  wait_ms1(time_x);

    if ( k==j) { j =1;
        // Auto-Setup CanOpen , Seite 25
    	j|= sdo_send (1,0x6060, 0,0x02,1);     j <<=1; // 6060h Modes of Operation, 02: Velocity Mode
        wait_ms1(time_x);

        j|= sdo_send (1,0x6042, 0,100,2);      j <<=1; // 6042 Target Velocity
        wait_ms1(time_x);

        j|= sdo_send (1,0x6040, 0,0x0006,2);     j <<=1; // 6040 006 = 110 = Enable Voltage, QuickStop = Aus
        wait_ms1(time_x);

        j|= sdo_send (1,0x6040, 0,0x0007,2);     j <<=1; // 6040 007 = 111 Switched On = 1
        wait_ms1(time_x);

        j|= sdo_send (1,0x6040, 0,0x000f,2);     j <<=1; // 6040 000f = 1111 Enable Operation
        wait_ms1(time_x);

        ende =0;
        z=0;
         
        l =  sdo_reseive(1,0x6063,0);	// 6063 Position Actual internal Value = Drehgeberposition
        wait_ms1(50);

        // Schleife so lange bis sich Position nicht mehr ändert
        while (ende ==0) {

        	k =  sdo_reseive(1,0x6063,0); // POsition auslesen
           wait_ms1(time_x);

           z =  sdo_reseive(1,0x1001,0); // fehler auslesen, 1001: Error-Register


           if(k == l ) { ende =1; }
           else {l = k;}
           wait_ms1(10);
           z++;
           if(z > 3000) { j=0; ende =1; } // Fraglich ob das Sinn ergibt
        }



        j|= sdo_send (1,0x6040, 0,0x0000,2);	// PSM PowerStateMAchine STOP
        wait_ms1(2);

        if( j ==   0x3f) { erg =1; } // Bedeutung unklar

        l = sdo_reseive(1,0x6064,0);	// Position auslesen, Nullpunkt gefunden
        entlage_rechts= l;
        my_position=l;
        wait_ms1(2);

        //sdo_send (1,0x607c, 0,l,4);	//607c: Home-Offset = 0
        wait_ms1(2);
        //sdo_reseive(1,0x607c,0);
        wait_ms1(2);



        k= sdo_send (1,0x607e, 0,0x00,1);  // 607e: Polarity, Drehrichtung
        wait_ms1(2);

        //sdo_send (1,0x320b, 4,-1,4);
        wait_ms1(2);
        //l = sdo_reseive(1,0x6064,0);
        wait_ms1(2);
        //my_position=l;
        printf("\n Null-Position = %d \n",l);



           
    }  
  }
  return erg;
}






int set_motor_stop(void) {
 int erg = -1;

 return erg;
 }
int setup_motorrueckfahrt(Position) {
int erg =-1;
int j,k,l,xx;
//printf("\n__setup_motorrueckfahrt START");

if ( set_motor_power_on() == 1 ) {
      j  = set_rampe_rauf_r();
      j <<=1;   // 0x6083

      j |= set_rampe_runter_r();
      j <<=1;

      j |= set_strom_ru();
      j <<=1;

      j |= set_motor_gesch(rampe1.geschwindr[rampe1.array_nr] );
      j <<=1;

      j |= set_position_r(Position);
      j <<=1;

      j = sdo_send(1,0x6040, 0, 0x003f,2);
      //if( j == 0x003f) { erg =1; }


      if( j == 1) { erg =1; }


      sdo_send (1,0x3202, 0,1,4); // Closed loop
      wait_ms1(2);
  }
//printf("__setup_motorrueckfahrt  ENDE \n");
return erg;
}

int set_not_stop(void) {
int erg =-1;
int j,k,l;
      wait_ms(2); 
      j  = set_rampe_rauf();                                      j <<=1;   // 0x6083
      j |= set_rampe_runter();                                    j <<=1;   // 0x6082
      j |= set_strom_ru();                                        j <<=1;
      j |= set_motor_gesch(rampe1.geschwindr[rampe1.array_nr] );  j <<=1;
      j |= set_position_r(0);                                      j <<=1;
      j |= sdo_send(1,0x6040, 0, 0x003f,2);
      if( j == 0x003f) { erg =1; }
 
  return erg;

}
int setup_motorhinfahrt(SpeedMultiplier){
int erg =-1;
int j,k,l;
  if ( set_motor_power_on() == 1 ) {



	  sdo_send (1,0x3202, 0,1,4); // Closed loop
	  wait_ms1(2);

	  //sdo_send (1,0x60f2, 0,2,2); // Zielposition relativ zur aktuellen Position
	  wait_ms1(2);

	  wait_ms(2);
      j  = set_rampe_rauf();                                      j <<=1;   // 0x6083
      j |= set_rampe_runter();                                    j <<=1;   // 0x6084
      j |= set_strom_vor();   									  j <<=1;	// 0x2031 Strom Druckaufbau

      //int tempV=0;
      //tempV = (int) (SpeedMultiplier * rampe1.geschwind[rampe1.array_nr]);

      //j |= set_motor_gesch(SpeedMultiplier );   j <<=1;   // 0x6081 und 6085
      j |= set_motor_gesch(rampe1.geschwind[rampe1.array_nr] );   j <<=1;
      //printf("\nx %f : %f \n", rampe1.array_nr, rampe1.geschwind[rampe1.array_nr]);
      j |= set_position();                                        j <<=1;   // 0x607a
      j |= set_mode(1);                                           j <<=1;   // 0x6060 1 = ProfilePositionMode
      l = sdo_reseive(1,0x6040,0);
      j |= sdo_send(1,0x6040, 0, 0x003f,2);									// PPM Bit 5: Fahrauftrag sofort und Bit 6: Zielposition rel.
      if ( j== 0x007f) { erg =1; }

      //sdo_send (1,0x3202, 0,1,4); // Closed Loop
      wait_ms1(2);

      sdo_send (1,0x6065, 0,rampe1.StallTol,4); // Following Error Window / Maximaler Schleppfehler
      wait_ms1(2);

      sdo_send (1,0x6066, 0,100,2); // Following Error Window / Maximaler Schleppfehler
      wait_ms1(2);



      sdo_send (1,0x3700, 0,0xffff,2); // Following Error Option Code / Closed Loop => Sofort Stop bei Schleppfehler
	  wait_ms1(2);

   }
 return  erg;

 }

 int setup_moto_offset(void){
int erg =-1;
int j,k,l;
  if ( set_motor_power_on() == 1 ) {
      wait_ms(2); 
      j  = set_rampe_rauf();                                      j <<=1;   // 0x6083
      j |= set_rampe_runter();                                    j <<=1;   // 0x6082
      j |= set_strom_ru();                                        j <<=1;
      j |= set_motor_gesch(rampe1.geschwind[rampe1.array_nr] );   j <<=1;   // 0x6081
      j |= set_position();                                        j <<=1;   // 0x607a
      j |= set_mode(1);                                           j <<=1;     // 0x6060
      l = sdo_reseive(1,0x6040,0);
      j |= sdo_send(1,0x6040, 0, 0x003f,2);
      if ( j== 0x007f) { erg =1; }
   }
 return  erg;

 }
int set_motor_gesch(int val ) {
 int erg = -1;
 int j;
     sdo_send (1,0x6081, 0,val,4); // Profile Velocity
     wait_ms1(2);
    j = sdo_reseive(1,0x6081,0);
     wait_ms1(2);
    if ( j == val) { erg =1; }
    /* Sinn?
    sdo_send (1,0x6085, 0,200,4); // Bremsbeschleunigung
     wait_ms1(2);
    j = sdo_reseive(1,0x6085,0);
    wait_ms1(2);
	*/


    //    wert4=erg;
  return erg;
}
int set_mode(unsigned char val) {
 int erg = -1;
 int j;


    sdo_send (1,0x6060, 0,val,1);
      wait_ms1(2);
    j = sdo_reseive(1,0x6060,0);
     wait_ms1(2);
    if ( j == val) { erg =1; }
  return erg;
}

int set_strom_vor(void) {
 int erg = -1;
 int j;
 int val;
 val = rampe1.strom[ rampe1.array_nr];
    sdo_send (1,0x2031, 0,val,4);
      wait_ms1(2);
    j = sdo_reseive(1,0x2031,0);
     wait_ms1(2);
    if ( j == val) { erg =1; }
  return erg;
}

int set_strom_ru(void) {
 int erg = -1;
 int j;
 int val;
 val = rampe1.strom_r[ rampe1.array_nr];
    sdo_send (1,0x2031, 0,val,4);
      wait_ms1(2);
    j = sdo_reseive(1,0x2031,0);
     wait_ms1(2);
    if ( j == val) { erg =1; }
//    wert3=erg;
  return erg;
}


int set_rampe_rauf(void) {
int erg =-1;
int j;
int val= rampe1.rampe_start[rampe1.array_nr];
    sdo_send (1,0x6083, 0,val,4); 
     wait_ms1(2);
    j = sdo_reseive(1,0x6083,0);
     wait_ms1(2);
    if ( j == val) { erg =1; }
//    wert1=erg;
  return erg;
}
int set_rampe_runter(void) {
int erg =-1;
int j;
int val =rampe1.rampe_stop[rampe1.array_nr];
    sdo_send (1,0x6084, 0,val,4); 
     wait_ms1(2);
    j = sdo_reseive(1,0x6084,0);
     wait_ms1(2);
    if ( j == val) { erg =1; }
//    wert2=erg;

  return erg;
}
int set_rampe_rauf_r(void) {
int erg =-1;
int j;
int val= rampe1.rampe_start_r[rampe1.array_nr];
    sdo_send (1,0x6083, 0,val,4); 
     wait_ms1(2);
    j = sdo_reseive(1,0x6083,0);
     wait_ms1(2);
    if ( j == val) { erg =1; }
  return erg;
}
int set_rampe_runter_r(void) {
int erg =-1;
int j;
int val =rampe1.rampe_stop_r[rampe1.array_nr];
    sdo_send (1,0x6084, 0,val,4); 
     wait_ms1(2);
    j = sdo_reseive(1,0x6084,0);
     wait_ms1(2);
    if ( j == val) { erg =1; }
  return erg;
}

int set_position(void) {
int erg =-1;
int j;
int k;
 k=1;

if(rampe1.Drehrichtung == 1 && rampe1.Oszi == 1){

	if (val <= (Oszi_UT_Temp - rampe1.Oszi_UT_Toleranz)) {Oszi_UT_Multi = -1;}
	if (val >= (Oszi_UT_Temp )) {Oszi_UT_Multi = +1;}
	val -= Oszi_UT_Multi * rampe1.Oszi_Schrittweite;
}
if(rampe1.Drehrichtung == 0 && rampe1.Oszi == 1){
	if (val >= (Oszi_UT_Temp + rampe1.Oszi_UT_Toleranz)) {Oszi_UT_Multi = -1;}
	if (val <= (Oszi_UT_Temp )) {Oszi_UT_Multi = +1;}
	val += Oszi_UT_Multi * rampe1.Oszi_Schrittweite;
}

V1 = val;
  sdo_send (1,0x607a, 0,val,4);
     wait_ms1(2);
    j = sdo_reseive(1,0x607a,0);
     wait_ms1(2);
    if ( j == val) { erg =1; }
    wert3=erg;
  return erg;
}





int set_position_r(Position) {

int erg =-1; int l;
int j;
int k;
float Offset=0;

if(rampe1.Drehrichtung == 1 && rampe1.Oszi == 1){
	if (val1 >= (Oszi_OT_Temp + rampe1.Oszi_OT_Toleranz)) {Oszi_OT_Multi = -1;}
	if (val1 <= (Oszi_OT_Temp )) {Oszi_OT_Multi = 1;}
	val1 = val1 + Oszi_OT_Multi * rampe1.Oszi_Schrittweite;
}
if(rampe1.Drehrichtung == 0 && rampe1.Oszi == 1){
	if (val1 <= (Oszi_OT_Temp - rampe1.Oszi_OT_Toleranz)) {Oszi_OT_Multi = -1;}
	if (val1 >= (Oszi_OT_Temp )) {Oszi_OT_Multi = 1;}
	val1 -= Oszi_OT_Multi * rampe1.Oszi_Schrittweite;
}
V1=val1;
if (Position == 1){
//fahre ein bischen vor bei stop
	l = sdo_reseive(1,0x6064,0);    // aktuelle Position
	if (rampe1.Drehrichtung == 1){Offset = 100;} else {Offset = -100;}
	wait_ms(2);
	  sdo_send (1,0x607a, 0,val1 + Offset,4);
	     wait_ms1(2);
	    j = sdo_reseive(1,0x607a,0);
	     wait_ms1(2);
	    if ( j == val1) { erg =1; }
	    wert5=erg;



  return erg;
}
else if (Position == 2){
// fahre zurück auf 0
  sdo_send (1,0x607a, 0,my_position,4);
     wait_ms1(2);
    j = sdo_reseive(1,0x607a,0);
     wait_ms1(2);
    if ( j == val1) { erg =1; }
    wert5=erg;
  return erg;
}
else {
// fahre zurück bei Oszi
	sdo_send (1,0x607a, 0,val1,4);
	     wait_ms1(2);
	    j = sdo_reseive(1,0x607a,0);
	     wait_ms1(2);
	    if ( j == val1) { erg =1; }
	    wert5=erg;
	    return erg;
	}
}
int set_motor_startpos(void) {
int erg = -1;

 return erg;
 }
