#include "main.h"
//#include <__cross_studio_io.h>
#include "globdata.h" 
#include "stm32373c_eval.h"
#include "stm32f3xx_hal_i2c.h"
//const unsigned char zeichen[] = { 0xf6, 0xc0, 0x6e, 0xea, 0xd8, 0xba, 0x9e, 0xe0, 0xfe, 0xf8, 0xfc, 0x9e,
//                                  0x36, 0xce, 0x3e, 0x3c };

const unsigned char zeichen2[]= { 0x6f, 0x28, 0x76, 0x7c, 0x39, 0x5d, 0x1f, 0x68, 0x7f, 0x79, 0x7b, 0x1f,
                                  0x47, 0x3e, 0x57, 0x53}; 
extern I2C_HandleTypeDef heval_I2c1;

void anzeige_init(void); 
void anzeige_set_valu(int val); 
//unsigned char transform(unsigned char val);
//void test_anzeige(void);

void anzeige_init(void) {
HAL_StatusTypeDef hs;
unsigned char data[8];
data[0]=0x5d;
hs =HAL_I2C_Master_Transmit( &heval_I2c1 , 0x7c , &data[0], 1 , 10000);
data[0]= 0x00;
hs =HAL_I2C_Master_Transmit( &heval_I2c1 , 0x7c , &data[0], 1 , 10000);
data[0]= 0x60;
hs =HAL_I2C_Master_Transmit( &heval_I2c1 , 0x7c , &data[0], 1 , 10000);
data[0]= 0x78;
hs =HAL_I2C_Master_Transmit( &heval_I2c1 , 0x7c , &data[0], 1 , 10000);
data[0]= 0x70;
hs =HAL_I2C_Master_Transmit( &heval_I2c1 , 0x7c , &data[0], 1 , 10000);
}
void anzeige_set_valu(int val) {
HAL_StatusTypeDef hs;
unsigned char data[8];
int i;
i = val;
data[0]=0x60;
data[1]= val & 0x000000ff;    val >>=8;
data[2]= val & 0x000000ff;    val >>=8;
data[3]= val & 0x000000ff;
hs =HAL_I2C_Master_Transmit( &heval_I2c1 , 0x7c , &data[0], 4 , 10000);
}

void anzeige_send_int(int val) {
//	printf("Anzeige: %d \n",val);
int i,j;
char c,d,e;
int erg;
  erg =0;
  i= val;
  if( i > 300) { i = 400; }
  if ( i > 99 ) {
      j = i/100;
      c= zeichen2[j];
      i -= (j * 100);
    }
  else { c =0 ;}

  if ( i > 9 ) {
      j = i/10;
      d= zeichen2[j];
      i -= (j * 10);
    }
  else { d =0; 
      if (c > 0) {d= zeichen2[0];   }
    }
  e= zeichen2[i];
//  c = transform(c);
//  d = transform(d);
//  e = transform(e);
  erg  = c;    erg<<=8;
  erg |= d;    erg<<=8;
  erg |= e;

  anzeige_set_valu(erg);
  //printf("Oelstand: %d\n",erg);
  //anzeige_set_valu(8);
}
//void test_anzeige(void) {
//char c,d,e;
//int erg;
//int k;
  
//  e = 0x10;//zeichen2[1];
//  c =0x10;//zeichen2[2];
//  d =0x10;//zeichen2[3];
//  erg  = e;    erg<<=8;
//  erg |= c;    erg<<=8;
//  erg |= d;
//  anzeige_set_valu(erg);
//  k =1;
//  while(1) {}
   

//}
//unsigned char transform(unsigned char val) {
//char erg =0;
// if ( val & 0x80) { erg |= 0x01; }
// if ( val & 0x40) { erg |= 0x02; }
// if ( val & 0x20) { erg |= 0x04; }
// if ( val & 0x10) { erg |= 0x08; }
// if ( val & 0x08) { erg |= 0x10; }
// if ( val & 0x04) { erg |= 0x20; }
// if ( val & 0x02) { erg |= 0x40; }
//  return erg;

//}
void anzeige_send_fehler(int val) {
	printf("Anzeige: %d \n",val);
int i,j;
//char c;
char d,e,f;
int erg;
  erg =0;
  i= val;
  if( val == 6) // "---"
  {
      erg  = 0x10;  erg <<=8;
      erg |= 0x10;  erg <<=8;
      erg |= 0x10;
      anzeige_set_valu(erg); 
  }
  else {
    d= zeichen2[15]; // "F"

    if ( i > 9 ) {
       j = i/10;
        e= zeichen2[j];
        i -= (j * 10);
      }
    else { e = 0; }
      f=zeichen2[i];
      erg = d;  erg <<=8;
      erg |= e; erg <<=8;
      erg |=f;
      anzeige_set_valu(erg); 
    }
}

void set_blink(void){
HAL_StatusTypeDef hs;
unsigned char data[8];
data[0]=0x73;
hs =HAL_I2C_Master_Transmit( &heval_I2c1 , 0x7c , &data[0], 1 , 10000);

 }
void reset_blink(void){
HAL_StatusTypeDef hs;
unsigned char data[8];
data[0]=0x70;
hs =HAL_I2C_Master_Transmit( &heval_I2c1 , 0x7c , &data[0], 1 , 10000);

}
