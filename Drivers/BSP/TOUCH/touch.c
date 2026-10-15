#include "touch.h"

#include "lcd.h"
#include <string.h>

#define TOUCH_I2C_ADDR        0x14U
#define TOUCH_REG_CTRL        0x8040U
#define TOUCH_REG_PID         0x8140U
#define TOUCH_REG_STATUS      0x814EU
#define TOUCH_REG_POINT1      0x8150U
#define TOUCH_STATUS_READY    0x80U
#define TOUCH_STATUS_COUNT    0x0FU

#define TOUCH_SCL_PORT        GPIOB
#define TOUCH_SCL_PIN         GPIO_PIN_0
#define TOUCH_SDA_PORT        GPIOB
#define TOUCH_SDA_PIN         GPIO_PIN_3
#define TOUCH_INT_PORT        GPIOB
#define TOUCH_INT_PIN         GPIO_PIN_1
#define TOUCH_RST_PORT        GPIOC
#define TOUCH_RST_PIN         GPIO_PIN_5

static char g_touch_pid[5] = {0};

static void touch_delay_us(uint32_t us)
{
  uint32_t start;
  uint32_t ticks;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  start = DWT->CYCCNT;
  ticks = (SystemCoreClock / 1000000U) * us;
  while ((uint32_t)(DWT->CYCCNT - start) < ticks)
  {
  }
}

static void touch_scl(GPIO_PinState state)
{
  HAL_GPIO_WritePin(TOUCH_SCL_PORT, TOUCH_SCL_PIN, state);
}

static void touch_sda(GPIO_PinState state)
{
  HAL_GPIO_WritePin(TOUCH_SDA_PORT, TOUCH_SDA_PIN, state);
}

static GPIO_PinState touch_read_sda(void)
{
  return HAL_GPIO_ReadPin(TOUCH_SDA_PORT, TOUCH_SDA_PIN);
}

static void touch_i2c_start(void)
{
  touch_sda(GPIO_PIN_SET);
  touch_scl(GPIO_PIN_SET);
  touch_delay_us(2U);
  touch_sda(GPIO_PIN_RESET);
  touch_delay_us(2U);
  touch_scl(GPIO_PIN_RESET);
}

static void touch_i2c_stop(void)
{
  touch_sda(GPIO_PIN_RESET);
  touch_delay_us(2U);
  touch_scl(GPIO_PIN_SET);
  touch_delay_us(2U);
  touch_sda(GPIO_PIN_SET);
  touch_delay_us(2U);
}

static void touch_i2c_send_byte(uint8_t value)
{
  uint8_t bit;

  for (bit = 0U; bit < 8U; bit++)
  {
    touch_sda((value & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    touch_delay_us(2U);
    touch_scl(GPIO_PIN_SET);
    touch_delay_us(2U);
    touch_scl(GPIO_PIN_RESET);
    value <<= 1;
  }
  touch_sda(GPIO_PIN_SET);
}

static uint8_t touch_i2c_wait_ack(void)
{
  uint16_t timeout = 0U;

  touch_sda(GPIO_PIN_SET);
  touch_delay_us(2U);
  touch_scl(GPIO_PIN_SET);
  touch_delay_us(2U);
  while (touch_read_sda() == GPIO_PIN_SET)
  {
    if (++timeout > 250U)
    {
      touch_scl(GPIO_PIN_RESET);
      touch_i2c_stop();
      return TOUCH_ERROR;
    }
  }
  touch_scl(GPIO_PIN_RESET);
  touch_delay_us(2U);
  return TOUCH_OK;
}

static void touch_i2c_ack(uint8_t ack)
{
  touch_sda(ack ? GPIO_PIN_RESET : GPIO_PIN_SET);
  touch_delay_us(2U);
  touch_scl(GPIO_PIN_SET);
  touch_delay_us(2U);
  touch_scl(GPIO_PIN_RESET);
  touch_delay_us(2U);
  touch_sda(GPIO_PIN_SET);
}

static uint8_t touch_i2c_recv_byte(uint8_t ack)
{
  uint8_t bit;
  uint8_t value = 0U;

  touch_sda(GPIO_PIN_SET);
  for (bit = 0U; bit < 8U; bit++)
  {
    value <<= 1;
    touch_scl(GPIO_PIN_SET);
    touch_delay_us(2U);
    if (touch_read_sda() == GPIO_PIN_SET)
    {
      value++;
    }
    touch_scl(GPIO_PIN_RESET);
    touch_delay_us(2U);
  }
  touch_i2c_ack(ack);
  return value;
}

static void touch_i2c_gpio_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = TOUCH_SCL_PIN | TOUCH_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);
  touch_i2c_stop();
}

static void touch_hw_reset(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  gpio.Pin = TOUCH_INT_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TOUCH_INT_PORT, &gpio);

  gpio.Pin = TOUCH_RST_PIN;
  HAL_GPIO_Init(TOUCH_RST_PORT, &gpio);

  HAL_GPIO_WritePin(TOUCH_RST_PORT, TOUCH_RST_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOUCH_INT_PORT, TOUCH_INT_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOUCH_INT_PORT, TOUCH_INT_PIN, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(TOUCH_RST_PORT, TOUCH_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(10U);

  gpio.Pin = TOUCH_INT_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_INT_PORT, &gpio);
  HAL_Delay(100U);
}

static uint8_t touch_write_reg(uint16_t reg, const uint8_t *data, uint8_t length)
{
  uint8_t index;

  touch_i2c_start();
  touch_i2c_send_byte((uint8_t)(TOUCH_I2C_ADDR << 1));
  if (touch_i2c_wait_ack() != TOUCH_OK) return TOUCH_ERROR;
  touch_i2c_send_byte((uint8_t)(reg >> 8));
  if (touch_i2c_wait_ack() != TOUCH_OK) return TOUCH_ERROR;
  touch_i2c_send_byte((uint8_t)reg);
  if (touch_i2c_wait_ack() != TOUCH_OK) return TOUCH_ERROR;

  for (index = 0U; index < length; index++)
  {
    touch_i2c_send_byte(data[index]);
    if (touch_i2c_wait_ack() != TOUCH_OK) return TOUCH_ERROR;
  }
  touch_i2c_stop();
  return TOUCH_OK;
}

static uint8_t touch_read_reg(uint16_t reg, uint8_t *data, uint8_t length)
{
  uint8_t index;

  if ((data == NULL) || (length == 0U)) return TOUCH_ERROR;

  touch_i2c_start();
  touch_i2c_send_byte((uint8_t)(TOUCH_I2C_ADDR << 1));
  if (touch_i2c_wait_ack() != TOUCH_OK) return TOUCH_ERROR;
  touch_i2c_send_byte((uint8_t)(reg >> 8));
  if (touch_i2c_wait_ack() != TOUCH_OK) return TOUCH_ERROR;
  touch_i2c_send_byte((uint8_t)reg);
  if (touch_i2c_wait_ack() != TOUCH_OK) return TOUCH_ERROR;

  touch_i2c_start();
  touch_i2c_send_byte((uint8_t)((TOUCH_I2C_ADDR << 1) | 1U));
  if (touch_i2c_wait_ack() != TOUCH_OK) return TOUCH_ERROR;
  for (index = 0U; index < length; index++)
  {
    data[index] = touch_i2c_recv_byte((index + 1U < length) ? 1U : 0U);
  }
  touch_i2c_stop();
  return TOUCH_OK;
}

uint8_t touch_init(void)
{
  uint8_t ctrl;

  touch_hw_reset();
  touch_i2c_gpio_init();
  if (touch_read_reg(TOUCH_REG_PID, (uint8_t *)g_touch_pid, 4U) != TOUCH_OK)
  {
    memcpy(g_touch_pid, "ERR", 4U);
    return TOUCH_ERROR;
  }
  g_touch_pid[4] = '\0';
  if ((strcmp(g_touch_pid, "911") != 0) &&
      (strcmp(g_touch_pid, "9147") != 0) &&
      (strcmp(g_touch_pid, "9271") != 0) &&
      (strcmp(g_touch_pid, "1158") != 0))
  {
    return TOUCH_ERROR;
  }

  ctrl = 0x02U;
  if (touch_write_reg(TOUCH_REG_CTRL, &ctrl, 1U) != TOUCH_OK) return TOUCH_ERROR;
  HAL_Delay(10U);
  ctrl = 0x00U;
  return touch_write_reg(TOUCH_REG_CTRL, &ctrl, 1U);
}

uint8_t touch_scan(touch_point_t *points, uint8_t capacity)
{
  static const uint16_t point_regs[TOUCH_MAX_POINTS] =
  {
    0x8150U, 0x8158U, 0x8160U, 0x8168U, 0x8170U
  };
  uint8_t status;
  uint8_t count;
  uint8_t index;
  uint8_t raw[6];
  uint8_t clear = 0U;

  if ((points == NULL) || (capacity == 0U) || (capacity > TOUCH_MAX_POINTS)) return 0U;
  if (touch_read_reg(TOUCH_REG_STATUS, &status, 1U) != TOUCH_OK) return 0U;
  if ((status & TOUCH_STATUS_READY) == 0U) return 0U;

  count = status & TOUCH_STATUS_COUNT;
  if (count > capacity) count = capacity;
  if (count > TOUCH_MAX_POINTS) count = TOUCH_MAX_POINTS;

  for (index = 0U; index < count; index++)
  {
    uint16_t raw_x;
    uint16_t raw_y;

    if (touch_read_reg(point_regs[index], raw, sizeof(raw)) != TOUCH_OK) break;
    raw_x = (uint16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    raw_y = (uint16_t)(((uint16_t)raw[3] << 8) | raw[2]);

    if (lcddev.dir == 1U)
    {
      points[index].x = raw_x;
      points[index].y = raw_y;
    }
    else
    {
      points[index].x = (raw_y < lcddev.width) ? (uint16_t)(lcddev.width - 1U - raw_y) : 0U;
      points[index].y = raw_x;
    }
    points[index].size = (uint16_t)(((uint16_t)raw[5] << 8) | raw[4]);
  }

  (void)touch_write_reg(TOUCH_REG_STATUS, &clear, 1U);
  return index;
}

const char *touch_get_product_id(void)
{
  return g_touch_pid;
}
