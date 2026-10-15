#ifndef TOUCH_H
#define TOUCH_H

#include "stm32h7xx_hal.h"

#define TOUCH_MAX_POINTS 5U
#define TOUCH_OK         0U
#define TOUCH_ERROR      1U

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t size;
} touch_point_t;

uint8_t touch_init(void);
uint8_t touch_scan(touch_point_t *points, uint8_t capacity);
const char *touch_get_product_id(void);

#endif
