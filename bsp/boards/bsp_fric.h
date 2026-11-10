#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#define FRIC_UP 1430
#define FRIC_DOWN 1425 //四号车1410 ，三号车1430
#define FRIC_OFF 1000

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
