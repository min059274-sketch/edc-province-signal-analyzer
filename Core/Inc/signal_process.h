#ifndef SIGNAL_PROCESS_H
#define SIGNAL_PROCESS_H

#include <stdint.h>
#include "app_config.h"

typedef struct
{
  float frequency_hz;
  float amplitude_mv;
  float phase_rad;
} SignalTone;

#define SIGNAL_MAX_SPECTRUM_PEAKS 12U

typedef struct
{
  float frequency_hz;
  float amplitude_mv;
} SignalSpectrumPeak;

typedef struct
{
  uint8_t valid;
  uint8_t clipped;
  uint8_t tone_count;
  uint8_t spectrum_count;
  float fundamental_hz;
  float peak_to_peak_mv;
  float rms_mv;
  float dc_code;
  float noise_floor_mv;
  uint32_t process_time_us;
  SignalTone tones[APP_MAX_TONES];
  SignalSpectrumPeak spectrum[SIGNAL_MAX_SPECTRUM_PEAKS];
} MeasurementResult;

void SignalProcess_Init(void);
void SignalProcess_SetCalibration(float reference_mv, float midpoint_mv, float input_gain);
uint8_t SignalProcess_Run(const uint16_t *raw, MeasurementResult *result);
float SignalProcess_Evaluate(const MeasurementResult *result, float time_s);

#endif
