#include "globdata.h" 
#include "stm32373c_eval.h"
#include "stm32f3xx_hal_gpio.h"
void AdcInit(void){
  Spi0_Cs_HIGH(); 
  Spi0_Clk_HIGH();
}

void wait_low(void) {
int i;
 for ( i=0; i< 15; i++) { }
}

void wait_high(void) {
 int i;
 for ( i=0; i< 15; i++) { } 
}
int get_adc(void) {
int i,j;
  Spi0_Cs_LOW();
  Spi0_Clk_LOW();
  wait_low(); 
  Spi0_Clk_HIGH();    //1
  wait_high();
  Spi0_Clk_LOW();
  wait_low(); 
  Spi0_Clk_HIGH();    //2
  wait_high();
  Spi0_Clk_LOW();
  wait_low();
  Spi0_Clk_HIGH();    //3
  wait_high();
  j =0;
  for(i=0; i< 16; i++) {
       Spi0_Clk_LOW();
       wait_low();
       j<<=1;
       j |= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
        Spi0_Clk_HIGH();    //n
        wait_high();


    }
    Spi0_Cs_HIGH()
    j &= 0x0000ffff;
 return j;
}

float get_adc_val(void) {
float erg;
int j;
 j =  get_adc(); wait_low();
 j += get_adc(); wait_low();
 j += get_adc(); wait_low();
 j += get_adc(); wait_low();
 j += get_adc(); wait_low();
 j += get_adc(); wait_low();
 j += get_adc(); wait_low();
 j += get_adc(); wait_low();

 j >>=3;
    erg = (float )j;
    erg *= 5.0;
    erg /= 65336;
 if ( erg > max_ad_val ) { max_ad_val = erg;} 
 if ( erg < min_ad_val ) { min_ad_val = erg;}
 return erg; 
}
