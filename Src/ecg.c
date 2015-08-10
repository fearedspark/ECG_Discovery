#include "ecg.h"

static arm_biquad_casd_df1_inst_f32 iir_filter_avg;
static float iir_coeffs_avg[] = {1.0f, 1.0f, 0.0f, 0.99844783544540405f, 0.0f, 1.0f, -1.9930019378662109f, 1.0f, 1.998440146446228f, -0.99844998121261597f};
static float iir_gain_avg = 0.00000115192591961905f; //IIR Biquads Gain
static float iir_states_avg[8];

static arm_biquad_casd_df1_inst_f32 iir_filter_noise;
static float iir_coeffs_noise[] = {1.0f, 1.0f, 0.0f, 0.93953502178192139f, 0.0f, 1.0f, -0.4886741042137146f, 1.0f, 1.9249252080917358f, -0.940143883228302f};
static float iir_gain_noise = 0.00030443458153359246f; //IIR Biquads Gain
static float iir_states_noise[8];

static arm_biquad_casd_df1_inst_f32 iir_filter_peak;
static float iir_coeffs_peak[] = {1.0f, -1.0f, 0.0f, 0.96852105855941772f, 0.0f};
static float iir_gain_peak = 1.0425816296914254f; //IIR Biquads Gain
static float iir_states_peak[16];

float ecg_display_values[350];
float ecg_display_avg[350];
uint16_t current_ecg_position = 0;

uint16_t ecg_refresh_request = 0;
uint8_t bpm_refresh_request = 0;

uint32_t beat_pulse_period;

float threshold;

static uint8_t ecg_bpm_timeout = 0;
static uint8_t samples_counter = 0;

static float peak_max_track[4096];
static int peak_max_current = 0;
static float peak_max;
static float peak_previous;
static uint8_t is_peak;
static uint8_t is_peak_over;
static uint32_t samples_since_last_pulse;
    
void initEcg()
{
    int i;
    is_peak = 0;
    peak_previous = 0.0f;
    current_ecg_position = 0;
    samples_counter = 0;
    peak_max_current = 0;
    for(i = 0; i < 350; i++)
    {
        ecg_display_values[i] = 0.0f;
        ecg_display_avg[i] = 0.0f;
        peak_max_track[i] = 0.0f;
    }
    for(i = 350; i < 4096; i++)
        peak_max_track[i] = 0.0f;
    ecg_refresh_request = 0;
    bpm_refresh_request = 0;
    ecg_bpm_timeout = 0;
    arm_biquad_cascade_df1_init_f32(&iir_filter_noise, 2, iir_coeffs_noise, iir_states_noise);
    arm_biquad_cascade_df1_init_f32(&iir_filter_avg, 2, iir_coeffs_avg, iir_states_avg);
    arm_biquad_cascade_df1_init_f32(&iir_filter_peak, 1, iir_coeffs_peak, iir_states_peak);
    EnableAdc3Int();
    StartAdc3();
    StartTim5();
    StartTim6();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    float measure, average, peak, noise;//, threshold;
    int i;
    measure = HAL_ADC_GetValue(hadc)/65535.0f;
    measure = 1.0f - measure;
    
    arm_biquad_cascade_df1_f32(&iir_filter_noise, &measure, &noise, 1);
    measure = noise * iir_gain_noise;
    arm_biquad_cascade_df1_f32(&iir_filter_avg, &measure, &average, 1);
    average *= iir_gain_avg;
    measure = measure - average + 0.5f;
    arm_biquad_cascade_df1_f32(&iir_filter_peak, &measure, &peak, 1);
    peak *= iir_gain_peak;
    peak_max_track[peak_max_current++] = peak;
    peak_max_current %= 4096;
    peak_max = peak_max_track[0];
    for(i = 0; i < 4096; i++)
        if(peak_max < peak_max_track[i])
            peak_max = peak_max_track[i];
    threshold = peak_max / 2;
    if(is_peak)
    {
        if(!is_peak_over && (peak < peak_previous))
        {
            StopTim6();
            if(Tim6_overflow)
            {
                beat_pulse_period = 0xFFFFFFFF;
                ResetTim6Flag();
            }
            else
                beat_pulse_period = GetTim6();
            ResetTim6();
            StartTim6();
//            beat_pulse_period = samples_since_last_pulse;
//            samples_since_last_pulse = 0;
            bpm_refresh_request = 1;
            is_peak_over = 1;
        }
        else
        {
//            if(samples_since_last_pulse < (uint32_t)0xFFFFFFFF)
//                samples_since_last_pulse++;
            if(is_peak_over && (peak < threshold/2))
                is_peak = is_peak_over = 0;
        }
    }
    if(peak > threshold)
    {
        is_peak = 1;
//        if(samples_since_last_pulse < (uint32_t)0xFFFFFFFF)
//            samples_since_last_pulse++;
    }
    peak_previous = peak;
    if(samples_counter == 0)
    {
        ecg_display_values[current_ecg_position] = measure;
        ecg_display_avg[current_ecg_position] = peak;
        current_ecg_position = (current_ecg_position + 1) % 350;
        ecg_refresh_request++;
    }
    
    samples_counter++;
    samples_counter %= 8;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    ecg_bpm_timeout = 1;
}
