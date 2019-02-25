#include "main.h"
//#include <__cross_studio_io.h>
#include "stm32f3xx_hal_usart.h"
#include "globdata.h" 
extern UART_HandleTypeDef UartHandle;
#define TBUF_SIZE         512	    
#define RBUF_SIZE         1024     
#define ESC               'A'
#define END               'Z'
#define TRUE  1
#define FALSE 0
struct buf_st

  {
  unsigned int in;          /* Next In Index */
  unsigned int out;         /* Next Out Index */
  unsigned char state;      /* f?r Slip       */
  unsigned int inh;        /* f?r slip       */
  unsigned char com_art;     /* slip oder kein slip */
  char buf [RBUF_SIZE];     /* Buffer */
  };

extern int eof;

struct buf_st rbuf0 ;
#define SIO_RBUFLEN ((unsigned short)(rbuf0.in - rbuf0.out))
struct buf_st tbuf0 ;
#define SIO_TBUFLEN ((unsigned short)(tbuf0.in - tbuf0.out))




void uart2Init(void) {
tbuf0.in = 0;
tbuf0.out = 0;
tbuf0.state =0;
tbuf0.inh =0;
tx_restart0 = 1;

rbuf0.in = 0;
rbuf0.out = 0;
rbuf0.inh =0;
ser_in_flag0 =0;
__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
}
void rest_buffer(void){
  rbuf0.in = 0;
  rbuf0.out=0;
  ser_in_flag0 =0;
}
void reset_in_0_buff( void) {
    rbuf0.in=0;          /* Next In Index */
    rbuf0.out=0;         /* Next Out Index */
    rbuf0.state=0;      /* f?r Slip       */
    rbuf0.inh=0;        /* f?r slip           */
    ser_in_flag0  =0;

}

int com_putchar0 (  int c) //__swi(8)

{

struct buf_st *p = &tbuf0;
/*------------------------------------------------

If the buffer is full, return an error value.

------------------------------------------------*/
c &= 0x00ff;
if (SIO_TBUFLEN >= TBUF_SIZE) {
    return (-1);

 }

/*------------------------------------------------

Add the data to the transmit buffer.  If the

transmit interrupt is disabled, then enable it.

------------------------------------------------*/



//if (tx_restart0)  {
//    tx_restart0 = 0;
//     __HAL_USART_ENABLE_IT(&UartHandle,USART_IT_TC);
//    UartHandle.Instance->TDR = c;
//
//  }
//
//else  {

    p->buf [p->in & (TBUF_SIZE - 1)] = c;
    p->in++;
  //}
return (0);
}

void print_f0( char * text) {
char c;
char start;
while(tx_restart0 ==0) {}
tbuf0.in =0;
tbuf0.out =0;  
start =  * text ++;
  while ( ( * text) !=0 ) {
    com_putchar0( * text ++);
  }
  tx_restart0 = 0;
#ifdef  RS485
    HAL_GPIO_WritePin(GPIOF, rs_485_dir_pin,GPIO_PIN_SET);
#endif
  __HAL_USART_ENABLE_IT(&UartHandle,USART_IT_TC);
  UartHandle.Instance->TDR = start;
} 



void print_f_lf0( char * text) {
char c;
char start;
 while(tx_restart0 ==0) { }
 tbuf0.in =0;
tbuf0.out =0;  
 start =  * text ++;
  while ( ( * text) !=0 ) {
    com_putchar0( * text ++);
    }	
  com_putchar0(0x0a);
  com_putchar0(0x0d);
  
  tx_restart0 = 0;
#ifdef  RS485
    HAL_GPIO_WritePin(GPIOF, rs_485_dir_pin,GPIO_PIN_SET);
#endif
   __HAL_USART_ENABLE_IT(&UartHandle,USART_IT_TC);
  UartHandle.Instance->TDR = start;

} 
void send_slip0( char * text, int len) {
int i=0;
char c;
  com_putchar0(ESC);
  while (len > 0 ) {
    c= *text++;
    if ( ( c== ESC)  || (c == END)) {
        com_putchar0(ESC);  
    }
    com_putchar0(c);
  }
 com_putchar0(END); 

}

int get_in_buff_len(void) {
int erg;
  erg = rbuf0.in-rbuf0.out;

return erg;
}
unsigned char get_char(void) {
unsigned char c;
c= rbuf0.buf [(rbuf0.out++) & (RBUF_SIZE - 1)];
 return c;
}





void uart_rx_tx_irq(void) {
volatile char dummy;
volatile char cx;
struct buf_st *p;
unsigned int interrupt_identification;
unsigned int int_bit;
unsigned int i,j;
int k;
 j =0;
 if ( __HAL_USART_GET_FLAG(&UartHandle, USART_FLAG_RXNE) == TRUE){
    cx = UartHandle.Instance->RDR;
    j|=0x0001;
    __HAL_USART_CLEAR_FLAG(&UartHandle,USART_FLAG_RXNE); 
    switch (rbuf0.state) {
          case 0: if (cx == ESC ) { rbuf0.state =1; }
          break;
          case 1: if ( cx == END) { 
                      ser_in_flag0 ++; 
                      rbuf0.state =0;
                      }
                  else { if ( cx == ESC ) { rbuf0.state = 2; }
                        else {
                            rbuf0.buf [rbuf0.in & (RBUF_SIZE-1)] = cx;
                            rbuf0.in++;
                            }
                          
          break;            
          case 2:  rbuf0.buf [rbuf0.in & (RBUF_SIZE-1)] = cx;
                   rbuf0.in++;
                   rbuf0.state = 1;
          break;
          }
      }
    }
  if ( __HAL_USART_GET_FLAG(&UartHandle,USART_FLAG_TC)== TRUE){   
      __HAL_USART_CLEAR_FLAG(&UartHandle,USART_FLAG_TC); 
      j|=0x0002;
      if (tbuf0.in != tbuf0.out)      {
          
           cx = tbuf0.buf [tbuf0.out & (TBUF_SIZE-1)];
           UartHandle.Instance->TDR = cx;
           
           tbuf0.out++;
           tx_restart0 = 0;
        }

      else          {
          tx_restart0 = 1;
          tbuf0.in = tbuf0.out =0;
          __HAL_USART_DISABLE_IT(&UartHandle,USART_IT_TC);
#ifdef  RS485
    HAL_GPIO_WritePin(GPIOF, rs_485_dir_pin,GPIO_PIN_RESET);
#endif

          }
  }
 if ( j ==0) {
  k=0;
   __HAL_USART_CLEAR_FLAG(&UartHandle,USART_CLEAR_PEF);
   __HAL_USART_CLEAR_FLAG(&UartHandle,USART_CLEAR_FEF);
   __HAL_USART_CLEAR_FLAG(&UartHandle,USART_CLEAR_NEF);
   __HAL_USART_CLEAR_FLAG(&UartHandle,USART_CLEAR_OREF);
   __HAL_USART_CLEAR_FLAG(&UartHandle,USART_CLEAR_IDLEF);
   __HAL_USART_CLEAR_FLAG(&UartHandle,USART_CLEAR_TCF);
   __HAL_USART_CLEAR_FLAG(&UartHandle,USART_CLEAR_CTSF);


 }

//  USART_ClearITPendingBits(USART2, 0x00121b5f);

}
