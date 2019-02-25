/**



   ----------------------------------------------------------------------
 */






#ifndef HX711_H_
#define HX711_H_

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif



#include "stm32f3xx_hal.h"

/* HX711 Settings */
#define HX711_PortSCK	GPIOB
#define HX711_PinSCK	GPIO_PIN_0
#define HX711_PortDT	GPIOB
#define HX711_PinDT		GPIO_PIN_1


/* Measurement Settings */
/** Channel and Gain for next reading
 *	1: channel A, gain factor 128
 *  2: channel B, gain factor 32
 *  3: channel A, gain factor 64
*/
#define HX711_Gain		1;

/* Functions */
/**
 * @brief  Initializes Ports/Pins for HX711
 * @param  None
 * @retval None
 */
void HX711_Init(void);

/**
 * @brief  Get last measured value
 * @param  None
 * @retval Value 32bit integer
 */
int HX711_Value(int gain);



#endif /* HX711_H_ */
