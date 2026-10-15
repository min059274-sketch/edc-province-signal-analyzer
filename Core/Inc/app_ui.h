#ifndef APP_UI_H
#define APP_UI_H

#include <stdint.h>
#include "signal_process.h"

void AppUI_Init(uint8_t touch_ok);
void AppUI_HandleTouch(void);
void AppUI_SetCapturing(uint8_t capturing);
void AppUI_Update(const MeasurementResult *result, uint32_t capture_us);

#endif
