

#include <string.h>
#include "stm32373c_eval.h"
#include "stm32f3xx_hal_spi.h"
#include "stm32f373xc.h"
#include "globdata.h"



#define eep_get_id              0x9f
#define eep_prog_buffer1	0x83
#define eep_prog_buffer2	0x86
#define eep_write_buffer1	0x84
#define eep_write_buffer2	0x87
#define eep_status_read		0xd7
#define eep_block_erase         0x50
#define eep_main_page_read	0xd2
#define eep_prog_del_buff1      0x88
#define eep_prog_del_buff2      0x89
#define eep_sektor_erase        0x7c

#define _start_page             1024
#define _end_page               2047


char adr_buffer[10];

 
extern void SPI2_Write(char Value);
extern  char SPI2_Read(char val);

char eep_get_status(void );
void eep_get_block( int adr,char * block);
void eep_set_block( int adr,char * block);
void init_eep(void);

void eep_set_bef(int anzahl,char * block);
void eep_get_val(int anzahl,char * block);
int get_eep_int(int block,int adr);
//int set_eep_int(int block,int adr,int val);
int setup_eep( void);
int del_sektor(int adress) ;
int del_block(int adress); 
void EEP_CS(int art);

char eep_write_byte(char val) ;




void EEP_CS(int art){
  if ( art ==0) {   Spi1_Cs_LOW() ;   }
      
  else {            Spi1_Cs_HIGH();     }
}
void init_eep(void)
{
 char c;
 
 Spi1_Sclk_HIGH();  
 

 EEP_CS(1); 
}

int get_eep_id(void ) {
int erg;
char c;
 erg =0;
 c = eep_write_byte(eep_get_id);
 erg = eep_write_byte(0x55);
 erg <<=8;
 erg |= eep_write_byte(0x55);
 erg <<=8;
 erg |= eep_write_byte(0x55);
 erg <<=8;
 erg |= eep_write_byte(0x55);
 EEP_CS(1);	
 return erg;
}

char eep_get_status(void )
{
 char c;
 
    EEP_CS(0);
    c = eep_write_byte(eep_status_read);
    
    c = eep_write_byte(0xaa);	// dummy write  
    EEP_CS(1);	
 return c;
}

char eep_write_byte(char val)
{
 char c;
   c =0; 
   EEP_CS(0);
   if ( val & 0x80) { Spi1_Mosi_HIGH(); }
   else             { Spi1_Mosi_LOW();  }
   Spi1_Sclk_LOW();
   c<<=1;
   c |= HAL_GPIO_ReadPin(GPIOA,Spi1_Miso_pin);
   Spi1_Sclk_HIGH(); 

   if ( val & 0x40) { Spi1_Mosi_HIGH(); }
   else             { Spi1_Mosi_LOW();  }
   Spi1_Sclk_LOW();
   c<<=1;
   c |= HAL_GPIO_ReadPin(GPIOA,Spi1_Miso_pin);
   Spi1_Sclk_HIGH(); 

   if ( val & 0x20) { Spi1_Mosi_HIGH(); }
   else             { Spi1_Mosi_LOW();  }
   Spi1_Sclk_LOW();
   c<<=1;
   c |= HAL_GPIO_ReadPin(GPIOA,Spi1_Miso_pin);
   Spi1_Sclk_HIGH(); 

   if ( val & 0x10) { Spi1_Mosi_HIGH(); }
   else             { Spi1_Mosi_LOW();  }
   Spi1_Sclk_LOW();
   c<<=1;
   c |= HAL_GPIO_ReadPin(GPIOA,Spi1_Miso_pin);
   Spi1_Sclk_HIGH();
    
   if ( val & 0x08) { Spi1_Mosi_HIGH(); }
   else             { Spi1_Mosi_LOW();  }
   Spi1_Sclk_LOW();
   c<<=1;
   c |= HAL_GPIO_ReadPin(GPIOA,Spi1_Miso_pin);
   Spi1_Sclk_HIGH(); 

   if ( val & 0x04) { Spi1_Mosi_HIGH(); }
   else             { Spi1_Mosi_LOW();  }
   Spi1_Sclk_LOW();
   c<<=1;
   c |= HAL_GPIO_ReadPin(GPIOA,Spi1_Miso_pin);
   Spi1_Sclk_HIGH();
    
   if ( val & 0x02) { Spi1_Mosi_HIGH(); }
   else             { Spi1_Mosi_LOW();  }
   Spi1_Sclk_LOW();
   c<<=1;
   c |= HAL_GPIO_ReadPin(GPIOA,Spi1_Miso_pin);
   Spi1_Sclk_HIGH(); 
   
   if ( val & 0x01) { Spi1_Mosi_HIGH(); }
   else             { Spi1_Mosi_LOW();  }
   Spi1_Sclk_LOW();
   c<<=1;
   c |= HAL_GPIO_ReadPin(GPIOA,Spi1_Miso_pin);
   Spi1_Sclk_HIGH(); 
  


  //c = SPI2_Read( val);
 
 return c;
}	




void eep_set_bef(int anzahl,char * block)
{
 int a;
 char d;
   EEP_CS( 0) ;  	// und bleibt es auch
   a=0;
   while ( a < anzahl)
      {    d =  eep_write_byte(block[a++]);      }  
}	

void eep_get_val(int anzahl,char * block)
{
 int a;
 EEP_CS(0) ;  	// und bleibt es auch
 a=0;
 while ( a < anzahl)
   {     block[a++]= eep_write_byte(0);   }  

}	


void eep_get_block( int adr,char * block)
{

int k,j;
char erg;	
  erg = eep_get_status( );
  while ( (erg & 0x80) ==0 )  {erg = eep_get_status( ); } 
  k = adr<<3;
  j =0;
  adr_buffer[j++]=eep_main_page_read;
  adr_buffer[j++]= (char ) (k>>8);
  adr_buffer[j++]= (char )  k;
  adr_buffer[j++]= 0x00;
  adr_buffer[j++]= 0x00;
  adr_buffer[j++]= 0x00;
  adr_buffer[j++]= 0x00; 
  adr_buffer[j++]= 0x00;   
  eep_set_bef(j,adr_buffer);

  eep_get_val(256, block);
  EEP_CS( 1);	

}

void eep_set_block( int adr,char * block)
{ int k,j;
  char erg ;
  erg = eep_get_status( );
  while ( (erg & 0x80) ==0 )  {erg = eep_get_status( ); }  
  k = adr<<2;
  j =0;
  adr_buffer[j++]=eep_write_buffer1;
  adr_buffer[j++]= 0;
  adr_buffer[j++]= 0;
  adr_buffer[j++]= 0;
  eep_set_bef(j,adr_buffer);
 
  eep_set_bef(256,block);
  EEP_CS(1);
  erg = eep_get_status( );
  while ( (erg & 0x80) ==0 ) {erg = eep_get_status( ); } 
   k = adr <<3 ; 
    
  j=0;
  adr_buffer[j++]= eep_prog_buffer1;    
  adr_buffer[j++]= (char ) (k>>8);
  adr_buffer[j++]= (char ) k;
  adr_buffer[j++]= 0;
  eep_set_bef(j,adr_buffer);  
  EEP_CS(1);
  erg = eep_get_status( );
   while ( (erg & 0x80) ==0 ) {erg = eep_get_status( ); } 
  EEP_CS(1);
}	

int del_sektoren(int ab, int anzahl){
int erg =0;
int i,j;
  j = ab;
  for ( i= 0; i< anzahl; i++) {  
       del_sektor(j);
       j++;
   }      

}

// löschen eines Sektors im Flash = 128 pages = 64 KB 
int del_sektor(int adress) {
int erg =0; 
int k,j;

  k =0;
  erg = eep_get_status( );
  while  ( (erg & 0x80) ==0 )  {
      erg =eep_get_status( ); 
      k++;
    }
      j =0; 
       k = adress;
       k&= 0x003f;
       adr_buffer[j++] =  eep_sektor_erase ;
       adr_buffer[j++]= (char ) (k);
       adr_buffer[j++]= 0;
       adr_buffer[j++]= 0;
       eep_set_bef(j,adr_buffer);  
       EEP_CS(1);
       erg = eep_get_status( );
       k =0;
       while  ( (erg & 0x80) ==0 ){
      erg = eep_get_status( ); 
      k++;
      }
 
 
return erg;
}
int del_bereich(int ab, int anzahl){
int erg =0;
int i,j;
  j = ab;
  for ( i= 0; i< anzahl; i++) {  
       del_block(j);
       j+=8;
   }     
return erg;
}
int del_block(int adress) {
int erg =0; 
int k,j;

  k =0;
  erg = eep_get_status( );
  while  ( (erg & 0x80) ==0 )  {
      erg =eep_get_status( ); 
      k++;
    
    }
      j =0; 
       k = adress<<1;
       k&= 0xfff8;
       adr_buffer[j++] = eep_block_erase;
       adr_buffer[j++]= (char ) (k>>8);
       adr_buffer[j++]= (char ) k;
       adr_buffer[j++]= 0;
       eep_set_bef(j,adr_buffer);  
       EEP_CS(1);
       erg = eep_get_status( );
       k =0;
       while  ( (erg & 0x80) ==0 ){
      erg = eep_get_status( ); 
      k++;
      }
 
 
return erg;
}
// Speichern eines ram Bereiches , egal wie groß dieser ist
// der speicherbereich muß aber vorher mit int del_bereich(int ab, int anzahl)
// gelöscht werden
int store_struct(char * struktur, int adress, int len){
 int erg =0;
 int i=0;
 int j=0;
 int k;
 int ad;
 ad = adress;
 while ( erg < len) {
      k = len -erg;
      if (k  > 256)   { j = 256;}
      else            { j = len-erg;}
      memcpy(flash_buffer,(struktur+i),j);
      eep_set_block(adress++,flash_buffer);
      erg += j;
      i   += j; 
  }

 return erg;
}  

int get_struct(char * struktur, int adresse, int len){
int erg =0;
int i=0;
int j=0;
int k=0;
   
 while ( erg < len) { 
     k = len-erg;   
      eep_get_block( adresse++,flash_buffer);
      if ( k > 256) { j= 256;}      
      else          { j = len-erg; }
      memcpy((struktur+erg),flash_buffer,j);
      erg += j;
  }    

       
return erg;
}
int Store_del_Block(int adress,char * data ) {
int erg =0; 
int k,j;

  k =0;
  erg = eep_get_status( );
  while  ( (erg & 0x80) ==0 )   {
      erg =eep_get_status( ); 
      k++;
   }
    k = adress<<3;
    j =0;
    adr_buffer[j++]=eep_write_buffer1;
    adr_buffer[j++]= 0;
    adr_buffer[j++]= 0;
    adr_buffer[j++]= 0;
    eep_set_bef(j,adr_buffer);
    eep_set_bef(256,data);
    EEP_CS(1);
    erg = eep_get_status( );
    k =0;
    while  ( (erg & 0x80) ==0 ) {
      erg = eep_get_status( ); 
      k++;
      }
 
       k = adress <<3 ; 
       j=0;
       adr_buffer[j++]= eep_prog_del_buff1;    
       adr_buffer[j++]= (char ) (k>>8);
       adr_buffer[j++]= (char ) k;
       adr_buffer[j++]= 0;
       eep_set_bef(j,adr_buffer);  
       EEP_CS(1);
    erg = eep_get_status( );
    k =0;
    while  ( (erg & 0x80) ==0 ) {
      erg = eep_get_status( ); 
      k++;
      }

return erg;
 }




int test_eep(void)
{
int i;
int j;
 j =1;


 for ( i=0; i< 256;i++)  { flash_buffer[i]=i; }
 eep_set_block( 0,(char *) &flash_buffer[0]);
 eep_get_block( 0,(char * )&flash_buffer[0]);
for ( i=0; i< 256;i++)
  { if ( flash_buffer[i]!= (i& 0xff))
      {
        j=-1;
      }
      
  }	
 
return j;	
} 


void eep_get_block2( int adr,char * block)
{

int k,j;
char erg;	
  erg = eep_get_status( );
  while ( (erg & 0x80) ==0 )  {erg = eep_get_status( ); } 
  k = adr<<2;
  j =0;
  adr_buffer[j++]=eep_main_page_read;
  adr_buffer[j++]= (char ) (k>>8);
  adr_buffer[j++]= (char )  k;
  adr_buffer[j++]= 0x00;
  adr_buffer[j++]= 0x00;
  adr_buffer[j++]= 0x00;
  adr_buffer[j++]= 0x00; 
  adr_buffer[j++]= 0x00;   
  eep_set_bef(j,adr_buffer);

  eep_get_val(32, block);
  EEP_CS( 1);	

}

int get_eep_int16(int block,int adr) { // rückgabewert int16  
 int erg;
  eep_get_block( block,(char * )&flash_buffer[0]);
  erg = flash_buffer[adr];
  erg <<=8;
  erg &= 0x0000ff00;
  erg |= (flash_buffer[adr+1] & 0x000000ff);
 return erg;
}

int set_eep_int16(int block,int adr,int val) {
  eep_get_block( block,(char * )&flash_buffer[0]);
  flash_buffer[adr] = val >>8;
  flash_buffer[adr+1] = val;
  eep_set_block( block,(char *) &flash_buffer[0]);
return 1;
}



int get_eep_int32(int block,int adr) { // rückgabewert int32  
 int erg;
  eep_get_block2( block,(char * )&flash_buffer[0]);
  erg = flash_buffer[adr];
  erg <<=8;
  erg &= 0x0000ff00;
  erg |= (flash_buffer[adr+1] & 0x000000ff);
  erg <<=8;
  erg &= 0x00ffff00;
  erg |= (flash_buffer[adr+2] & 0x000000ff);
  erg <<=8;
  erg &= 0xffffff00;
  erg |= (flash_buffer[adr+3] & 0x000000ff);
 return erg;
}

int set_eep_int32(int block,int adr,int val) {
  eep_get_block( block,(char * )&flash_buffer[0]);
  flash_buffer[adr]   = val >>24;
  flash_buffer[adr+1] = val >>16;
  flash_buffer[adr+2] = val >>8;
  flash_buffer[adr+3] = val;
  eep_set_block( block,(char *) &flash_buffer[0]);
return 1;
}


int find_offset(void){
int i=0;
int j=0;
int k=0;
int ende =0;
  i = _start_page;
  while( ende ==0) {
    j = get_eep_int16(i,k);
    if ( j != 0x0000ffff) {
        i++;
        if( i > _end_page) {ende =1; in_life_time_buffer=0; life_time_block = 1024; }
    }
    else {
      life_time_block=i;
      in_life_time_buffer=0;
      ende =1;
      }
    
  }

 return life_time_block;
}

int store_offset(int val) {
 int i;
 i= in_anzeige_block;
 set_eep_int16(life_time_block,in_anzeige_block,val);
 in_anzeige_block++;
 in_anzeige_block++;
 in_anzeige_block &= 0x00ff;
 
 
}

int setup_eep( void) {

  
  return 1;
}
        
