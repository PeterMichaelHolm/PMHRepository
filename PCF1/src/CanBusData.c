#include "stm32f3xx_hal_conf.h"

typedef struct   {
  int NodeID;
  unsigned char object_parameter;
  unsigned int object_index;
  unsigned char obect_sub_index;
  unsigned int var;
  unsigned char data[8];
  unsigned char state;
  unsigned char art;
  int time_out;
}sdo;
sdo my_sdo;

