#ifndef ACQUISITION_H
#define ACQUISITION_H

#include "stm32h7xx_hal.h"
#include "app_config.h"

HAL_StatusTypeDef Acquisition_Init(void);
HAL_StatusTypeDef Acquisition_Start(void);
uint8_t Acquisition_IsBusy(void);
uint8_t Acquisition_IsReady(void);
void Acquisition_Consume(void);
const uint16_t *Acquisition_GetBuffer(void);
uint32_t Acquisition_GetCaptureTimeUs(void);
void Acquisition_DMA_IRQHandler(void);

#endif
