#include "app_ui.h"
#include "lcd.h"
#include "touch.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define UI_BG       0x0841U
#define UI_PANEL    0x18E3U
#define UI_GRID     0x4208U
#define UI_TEXT     LGRAY
#define UI_ACCENT   CYAN
#define UI_WAVE     YELLOW
#define CHART_X0    35U
#define CHART_X1    775U
#define CHART_Y0    120U
#define CHART_Y1    398U
#define CHART_MID_Y 259U
#define SPECTRUM_BASE_Y 390U

static uint8_t g_page;
static uint8_t g_cycles = 1U;
static uint8_t g_touch_was_down;
static MeasurementResult g_last;
static uint8_t g_have_result;
static uint8_t g_drawn_page = 0xFFU;
static uint8_t g_old_wave_valid;
static int16_t g_old_wave_y[741];
static uint8_t g_old_spectrum_count;
static uint16_t g_old_spectrum_x[SIGNAL_MAX_SPECTRUM_PEAKS];
static uint16_t g_old_spectrum_h[SIGNAL_MAX_SPECTRUM_PEAKS];
static uint8_t g_invalid_shown;

static uint32_t rounded_positive(float value)
{
  return value > 0.0f ? (uint32_t)(value + 0.5f) : 0U;
}

static void text_at(uint16_t x, uint16_t y, uint8_t size, uint16_t color, const char *s)
{
  lcd_fill(x, y, 790U, (uint16_t)(y + size + 3U), UI_BG);
  lcd_show_string(x, y, (uint16_t)(790U - x), (uint16_t)(size + 4U), size, (char *)s, color);
}

static void draw_buttons(void)
{
  lcd_fill(0U, 430U, 799U, 479U, UI_PANEL);
  lcd_draw_rectangle(15U, 438U, 170U, 472U, g_page == 0U ? UI_ACCENT : GRAY);
  lcd_draw_rectangle(185U, 438U, 370U, 472U, g_page == 1U ? UI_ACCENT : GRAY);
  lcd_draw_rectangle(610U, 438U, 780U, 472U, UI_ACCENT);
  lcd_show_string(55U, 446U, 100U, 24U, 16U, "WAVE", UI_TEXT);
  lcd_show_string(225U, 446U, 120U, 24U, 16U, "SPECTRUM", UI_TEXT);
  lcd_show_string(628U, 446U, 140U, 24U, 16U, g_cycles == 1U ? "CYCLE: 1" : "CYCLE: 3", UI_TEXT);
}

static void draw_grid(void)
{
  uint16_t x;
  uint16_t y;
  for (x = CHART_X0; x <= CHART_X1; x = (uint16_t)(x + 74U)) lcd_draw_line(x, CHART_Y0, x, CHART_Y1, UI_GRID);
  for (y = CHART_Y0; y <= CHART_Y1; y = (uint16_t)(y + 46U)) lcd_draw_line(CHART_X0, y, CHART_X1, y, UI_GRID);
  lcd_draw_rectangle(CHART_X0, CHART_Y0, CHART_X1, CHART_Y1, GRAY);
}

static void draw_frame(void)
{
  lcd_fill(CHART_X0, CHART_Y0, CHART_X1, CHART_Y1, BLACK);
  lcd_fill(88U, 399U, 136U, 414U, UI_BG);
  draw_grid();
  g_old_wave_valid = 0U;
  g_old_spectrum_count = 0U;
}

static void draw_parameter_panel(const MeasurementResult *r)
{
  char line[64];
  uint32_t value_100;
  uint32_t f_hz;

  /* Repaint the complete parameter panel so removed harmonics cannot remain. */
  lcd_fill(10U, 35U, 790U, 112U, UI_PANEL);
  if (r->valid == 0U)
  {
    lcd_show_string(22U, 45U, 230U, 30U, 24U, "f0  --", LGRAY);
    lcd_show_string(282U, 45U, 230U, 30U, 24U, "Upp  --", LGRAY);
    lcd_show_string(542U, 45U, 230U, 30U, 24U, "Urms  --", LGRAY);
    return;
  }

  f_hz = rounded_positive(r->fundamental_hz);
  (void)snprintf(line, sizeof(line), "f0 %lu.%03lu kHz",
                 (unsigned long)(f_hz / 1000U), (unsigned long)(f_hz % 1000U));
  lcd_show_string(22U, 43U, 240U, 30U, 24U, line, YELLOW);

  value_100 = rounded_positive(r->peak_to_peak_mv * 100.0f);
  (void)snprintf(line, sizeof(line), "Upp %lu.%02lu mVpp",
                 (unsigned long)(value_100 / 100U), (unsigned long)(value_100 % 100U));
  lcd_show_string(282U, 43U, 240U, 30U, 24U, line, CYAN);

  value_100 = rounded_positive(r->rms_mv * 100.0f);
  (void)snprintf(line, sizeof(line), "Urms %lu.%02lu mVrms",
                 (unsigned long)(value_100 / 100U), (unsigned long)(value_100 % 100U));
  lcd_show_string(542U, 43U, 240U, 30U, 24U, line, GREEN);

  for (uint8_t i = 0U; i < r->tone_count; ++i)
  {
    uint16_t x = (uint16_t)(22U + (uint16_t)i * 260U);
    uint32_t harmonic_order = 1U;
    f_hz = rounded_positive(r->tones[i].frequency_hz);
    value_100 = rounded_positive(r->tones[i].amplitude_mv * 100.0f);
    if ((i != 0U) && (r->fundamental_hz > 0.0f))
    {
      harmonic_order = rounded_positive(r->tones[i].frequency_hz / r->fundamental_hz);
      if (harmonic_order < 2U) harmonic_order = 2U;
    }
    (void)snprintf(line, sizeof(line), "H%lu %lu.%03luk  %lu.%02lumVpk",
                   (unsigned long)harmonic_order,
                   (unsigned long)(f_hz / 1000U), (unsigned long)(f_hz % 1000U),
                   (unsigned long)(value_100 / 100U), (unsigned long)(value_100 % 100U));
    lcd_show_string(x, 80U, 250U, 24U, 16U, line, i == 0U ? YELLOW : CYAN);
  }
}

static void draw_wave(const MeasurementResult *r)
{
  uint16_t x;
  int16_t last_y = (int16_t)CHART_MID_Y;
  float half_range = r->peak_to_peak_mv * 0.60f;
  float tone_sin[APP_MAX_TONES];
  float tone_cos[APP_MAX_TONES];
  float step_sin[APP_MAX_TONES];
  float step_cos[APP_MAX_TONES];
  uint8_t tone;
  if (half_range < 25.0f) half_range = 25.0f;
  for (tone = 0U; tone < r->tone_count; tone++)
  {
    uint16_t harmonic_order = (uint16_t)((r->tones[tone].frequency_hz /
                                         r->fundamental_hz) + 0.5f);
    float step;
    if (harmonic_order < 1U) harmonic_order = 1U;
    step = 2.0f * 3.14159265358979323846f * (float)harmonic_order *
           (float)g_cycles / 740.0f;
    tone_sin[tone] = 0.0f;
    tone_cos[tone] = 1.0f;
    step_sin[tone] = sinf(step);
    step_cos[tone] = cosf(step);
  }
  if (g_old_wave_valid != 0U)
  {
    for (x = (uint16_t)(CHART_X0 + 1U); x <= CHART_X1; ++x)
    {
      lcd_draw_line((uint16_t)(x - 1U), (uint16_t)g_old_wave_y[x - 36U], x, (uint16_t)g_old_wave_y[x - 35U], BLACK);
    }
    draw_grid();
  }
  lcd_draw_line(CHART_X0, CHART_MID_Y, CHART_X1, CHART_MID_Y, LGRAY);
  for (x = CHART_X0; x <= CHART_X1; ++x)
  {
    float v = 0.0f;
    for (tone = 0U; tone < r->tone_count; tone++)
    {
      float next_cosine;
      v += r->tones[tone].amplitude_mv * tone_sin[tone];
      next_cosine = (tone_cos[tone] * step_cos[tone]) - (tone_sin[tone] * step_sin[tone]);
      tone_sin[tone] = (tone_sin[tone] * step_cos[tone]) + (tone_cos[tone] * step_sin[tone]);
      tone_cos[tone] = next_cosine;
    }
    int16_t y = (int16_t)((float)CHART_MID_Y - v * 122.0f / half_range);
    if (y < (int16_t)(CHART_Y0 + 2U)) y = (int16_t)(CHART_Y0 + 2U);
    if (y > (int16_t)(CHART_Y1 - 2U)) y = (int16_t)(CHART_Y1 - 2U);
    if (x > CHART_X0) lcd_draw_line((uint16_t)(x - 1U), (uint16_t)last_y, x, (uint16_t)y, UI_WAVE);
    g_old_wave_y[x - CHART_X0] = y;
    last_y = y;
  }
  g_old_wave_valid = 1U;
}

static void draw_spectrum(const MeasurementResult *r)
{
  uint8_t i;
  uint8_t count = r->spectrum_count;
  float max_amp = 1.0f;
  for (i = 0U; i < g_old_spectrum_count; ++i)
  {
    lcd_draw_line(g_old_spectrum_x[i], SPECTRUM_BASE_Y, g_old_spectrum_x[i], (uint16_t)(SPECTRUM_BASE_Y - g_old_spectrum_h[i]), BLACK);
    if (g_old_spectrum_x[i] > 1U) lcd_draw_line((uint16_t)(g_old_spectrum_x[i] - 1U), SPECTRUM_BASE_Y, (uint16_t)(g_old_spectrum_x[i] - 1U), (uint16_t)(SPECTRUM_BASE_Y - g_old_spectrum_h[i]), BLACK);
  }
  if (g_old_spectrum_count != 0U) draw_grid();
  if (count > SIGNAL_MAX_SPECTRUM_PEAKS) count = SIGNAL_MAX_SPECTRUM_PEAKS;
  for (i = 0U; i < count; ++i)
    if (r->spectrum[i].amplitude_mv > max_amp) max_amp = r->spectrum[i].amplitude_mv;
  for (i = 0U; i < count; ++i)
  {
    uint16_t x = (uint16_t)(35.0f + r->spectrum[i].frequency_hz * 740.0f / 500000.0f);
    uint16_t h = (uint16_t)(r->spectrum[i].amplitude_mv * 235.0f / max_amp);
    if (x > CHART_X1) x = CHART_X1;
    lcd_draw_line(x, SPECTRUM_BASE_Y, x, (uint16_t)(SPECTRUM_BASE_Y - h), (i == 0U) ? YELLOW : CYAN);
    if (x > 1U) lcd_draw_line((uint16_t)(x - 1U), SPECTRUM_BASE_Y, (uint16_t)(x - 1U), (uint16_t)(SPECTRUM_BASE_Y - h), (i == 0U) ? YELLOW : CYAN);
    g_old_spectrum_x[i] = x;
    g_old_spectrum_h[i] = h;
  }
  g_old_spectrum_count = count;
  lcd_show_string(93U, 399U, 42U, 14U, 12U, "50k", LGRAY);
}

static void redraw_result(void)
{
  if (g_have_result == 0U) return;
  if (g_drawn_page != g_page)
  {
    draw_frame();
    g_drawn_page = g_page;
  }
  draw_parameter_panel(&g_last);
  if (g_last.valid == 0U)
  {
    draw_frame();
    text_at(240U, 245U, 24U, RED, "NO VALID SIGNAL");
    g_invalid_shown = 1U;
    return;
  }
  if (g_invalid_shown != 0U)
  {
    draw_frame();
    g_invalid_shown = 0U;
  }
  if (g_page == 0U) draw_wave(&g_last); else draw_spectrum(&g_last);
}

void AppUI_Init(uint8_t touch_ok)
{
  (void)touch_ok;
  g_page = 0U;
  g_cycles = 1U;
  g_have_result = 0U;
  g_invalid_shown = 0U;
  g_drawn_page = 0xFFU;
  g_back_color = UI_BG;
  g_point_color = UI_TEXT;
  lcd_clear(UI_BG);
  lcd_show_string(18U, 8U, 360U, 26U, 24U, "PERIODIC SIGNAL ANALYZER", UI_ACCENT);
  draw_frame();
  draw_parameter_panel(&g_last);
  draw_buttons();
  text_at(18U, 410U, 12U, GREEN, "RUNNING");
}

void AppUI_HandleTouch(void)
{
  touch_point_t p[TOUCH_MAX_POINTS];
  uint8_t count = touch_scan(p, TOUCH_MAX_POINTS);
  if ((count > 0U) && (g_touch_was_down == 0U))
  {
    if ((p[0].y >= 430U) && (p[0].x <= 180U)) g_page = 0U;
    else if ((p[0].y >= 430U) && (p[0].x >= 180U) && (p[0].x <= 390U)) g_page = 1U;
    else if ((p[0].y >= 430U) && (p[0].x >= 590U)) g_cycles = (g_cycles == 1U) ? 3U : 1U;
    draw_buttons();
    redraw_result();
  }
  g_touch_was_down = count > 0U;
}

void AppUI_SetCapturing(uint8_t capturing)
{
  (void)capturing;
}

void AppUI_Update(const MeasurementResult *result, uint32_t capture_us)
{
  char line[48];
  memcpy(&g_last, result, sizeof(g_last));
  g_have_result = 1U;
  redraw_result();
  (void)snprintf(line, sizeof(line), "ADC %lu us  DSP %lu us", (unsigned long)capture_us, (unsigned long)result->process_time_us);
  text_at(170U, 410U, 12U, LGRAY, line);
}
