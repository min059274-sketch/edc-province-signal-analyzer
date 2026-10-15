#include "acquisition.h"

static ADC_HandleTypeDef g_hadc1;
static DMA_HandleTypeDef g_hdma_adc1;
static TIM_HandleTypeDef g_htim2;

__attribute__((section(".adc_dma_buffer"), aligned(32)))
static uint16_t g_adc_buffer[APP_ADC_BUFFER_LENGTH];

static volatile uint8_t g_capture_busy;
static volatile uint8_t g_capture_ready;
static volatile uint32_t g_capture_start_cycles;
static volatile uint32_t g_capture_time_us;

static HAL_StatusTypeDef Acquisition_ClockInit(void)
{
  RCC_PeriphCLKInitTypeDef periph = {0};

  periph.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  periph.PLL2.PLL2M = 4U;
  /* 64 MHz HSI / 4 * 24 / 8 = 48 MHz ADC kernel clock.
   * 36 MHz was marginal for 16-bit, 8.5-cycle sampling at 2 MSa/s and
   * caused every other external trigger to be missed on the target. */
  periph.PLL2.PLL2N = 24U;
  periph.PLL2.PLL2P = 8U;
  periph.PLL2.PLL2Q = 8U;
  periph.PLL2.PLL2R = 8U;
  periph.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  periph.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  periph.PLL2.PLL2FRACN = 0U;
  periph.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;

  return HAL_RCCEx_PeriphCLKConfig(&periph);
}

static HAL_StatusTypeDef Acquisition_DMAInit(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  g_hdma_adc1.Instance = DMA1_Stream0;
  g_hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
  g_hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  g_hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  g_hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  g_hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  g_hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  g_hdma_adc1.Init.Mode = DMA_NORMAL;
  g_hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  g_hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&g_hdma_adc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  __HAL_LINKDMA(&g_hadc1, DMA_Handle, g_hdma_adc1);
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  return HAL_OK;
}

static HAL_StatusTypeDef Acquisition_ADCInit(void)
{
  GPIO_InitTypeDef gpio = {0};
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef channel = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_ADC12_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_0;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &gpio);

  g_hadc1.Instance = ADC1;
  g_hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  g_hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  g_hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  g_hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  g_hadc1.Init.LowPowerAutoWait = DISABLE;
  g_hadc1.Init.ContinuousConvMode = DISABLE;
  g_hadc1.Init.NbrOfConversion = 1U;
  g_hadc1.Init.DiscontinuousConvMode = DISABLE;
  g_hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  g_hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  g_hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  g_hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  g_hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  g_hadc1.Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(&g_hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&g_hadc1, &multimode) != HAL_OK)
  {
    return HAL_ERROR;
  }

  channel.Channel = ADC_CHANNEL_10;
  channel.Rank = ADC_REGULAR_RANK_1;
  /* Keep total conversion time below the 500 ns trigger period. */
  channel.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  channel.SingleDiff = ADC_SINGLE_ENDED;
  channel.OffsetNumber = ADC_OFFSET_NONE;
  channel.Offset = 0U;
  channel.OffsetSignedSaturation = DISABLE;

  if (HAL_ADC_ConfigChannel(&g_hadc1, &channel) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_ADCEx_Calibration_Start(&g_hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

static HAL_StatusTypeDef Acquisition_TimerInit(void)
{
  TIM_MasterConfigTypeDef master = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();
  g_htim2.Instance = TIM2;
  g_htim2.Init.Prescaler = 0U;
  g_htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  /* 200 MHz TIM2 clock / 100 = 2.000 MSa/s. */
  g_htim2.Init.Period = 99U;
  g_htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  g_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&g_htim2) != HAL_OK)
  {
    return HAL_ERROR;
  }

  master.MasterOutputTrigger = TIM_TRGO_UPDATE;
  master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  return HAL_TIMEx_MasterConfigSynchronization(&g_htim2, &master);
}

HAL_StatusTypeDef Acquisition_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  if (Acquisition_ClockInit() != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Acquisition_DMAInit() != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Acquisition_ADCInit() != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Acquisition_TimerInit() != HAL_OK)
  {
    return HAL_ERROR;
  }

  g_capture_busy = 0U;
  g_capture_ready = 0U;
  return HAL_OK;
}

HAL_StatusTypeDef Acquisition_Start(void)
{
  if (g_capture_busy != 0U)
  {
    return HAL_BUSY;
  }

  g_capture_ready = 0U;
  g_capture_busy = 1U;
  __HAL_TIM_SET_COUNTER(&g_htim2, 0U);
  g_capture_start_cycles = DWT->CYCCNT;

  if (HAL_ADC_Start_DMA(&g_hadc1, (uint32_t *)g_adc_buffer, APP_ADC_BUFFER_LENGTH) != HAL_OK)
  {
    g_capture_busy = 0U;
    return HAL_ERROR;
  }
  if (HAL_TIM_Base_Start(&g_htim2) != HAL_OK)
  {
    (void)HAL_ADC_Stop_DMA(&g_hadc1);
    g_capture_busy = 0U;
    return HAL_ERROR;
  }

  return HAL_OK;
}

uint8_t Acquisition_IsBusy(void)
{
  return g_capture_busy;
}

uint8_t Acquisition_IsReady(void)
{
  return g_capture_ready;
}

void Acquisition_Consume(void)
{
  g_capture_ready = 0U;
}

const uint16_t *Acquisition_GetBuffer(void)
{
  return g_adc_buffer;
}

uint32_t Acquisition_GetCaptureTimeUs(void)
{
  return g_capture_time_us;
}

void Acquisition_DMA_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&g_hdma_adc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    uint32_t elapsed = DWT->CYCCNT - g_capture_start_cycles;
    (void)HAL_TIM_Base_Stop(&g_htim2);
    (void)HAL_ADC_Stop_DMA(&g_hadc1);
    g_capture_time_us = elapsed / (SystemCoreClock / 1000000U);
    g_capture_busy = 0U;
    g_capture_ready = 1U;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    (void)HAL_TIM_Base_Stop(&g_htim2);
    g_capture_busy = 0U;
    g_capture_ready = 0U;
  }
}
