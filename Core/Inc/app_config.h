#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_SAMPLE_RATE_HZ          2000000.0f
#define APP_ADC_BUFFER_LENGTH       8192U
#define APP_FFT_LENGTH              4096U
#define APP_FIR_TAPS                81U
#define APP_MAX_TONES               3U
/* Nominal analog-front-end voltage gain at the ADC input. */
#define APP_INPUT_GAIN              8.0f
#define APP_ADC_REFERENCE_MV        3300.0f
#define APP_ADC_MIDPOINT_MV         1650.0f
#define APP_ADC_FULL_SCALE_CODE     4095.0f
#define APP_MEASUREMENT_PERIOD_MS   500U
#define APP_TOUCH_PERIOD_MS         25U

#endif
