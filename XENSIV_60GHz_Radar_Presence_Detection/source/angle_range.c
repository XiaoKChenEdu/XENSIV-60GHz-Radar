/*
 * angle_range.c
 * Implements range and angle estimation utilities using CMSIS-DSP.
 */

#if 0
#include "angle_range.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "optimization_list.h"

#define RADAR_CARRIER_FREQ_HZ      (60.0e9f)
#define RADAR_BANDWIDTH_HZ         (2000.0e6f)
#define RADAR_CHIRP_PERIOD_S       (6.9e-5f)
#define RADAR_SAMPLE_RATE_HZ       (2352941.0f)
#define RADAR_LIGHT_SPEED_MPS      (299792458.0f)
#define RADAR_RAD_TO_DEG           (57.29577951308232f)
#define RADAR_TWO_PI               (6.283185307179586f)
#define RADAR_ANT_SPACING_X_M      (0.0025f)
#define RADAR_ANT_SPACING_Y_M      (0.0025f)

#ifndef XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS
#define XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS (3)
#endif

#if XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS < 3
#error "angle_range module requires at least three RX antennas"
#endif

static arm_cfft_instance_f32 cfft_instance;
static bool fft_initialised;
static bool stream_enabled;
static angle_range_result_t last_result;
static bool last_valid;

static void copy_first_chirp(const float32_t *frame_data, float32_t (*buffer)[NUM_SAMPLES_PER_CHIRP * 2])
{
    const size_t chirp_span = NUM_SAMPLES_PER_CHIRP * 2U * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS;
    (void)chirp_span; /* Intentional: documented layout */

    taskENTER_CRITICAL();
    for (uint32_t ant = 0U; ant < XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS; ++ant)
    {
        const uint32_t base_idx = ant * NUM_SAMPLES_PER_CHIRP * 2U;
        for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
        {
            buffer[ant][sample * 2U]     = frame_data[base_idx + sample * 2U];
            buffer[ant][sample * 2U + 1U] = frame_data[base_idx + sample * 2U + 1U];
        }
    }
    taskEXIT_CRITICAL();
}

bool angle_range_compute(const float32_t *frame_data, angle_range_result_t *result)
{
    float32_t antenna_data[XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS][NUM_SAMPLES_PER_CHIRP * 2];
    float32_t lambda;
    uint32_t peak_bin = 0U;
    float32_t peak_mag = 0.0f;

    configASSERT(frame_data != NULL);
    configASSERT(result != NULL);

    if (!fft_initialised)
    {
        if (arm_cfft_init_f32(&cfft_instance, NUM_SAMPLES_PER_CHIRP) != ARM_MATH_SUCCESS)
        {
            result->valid = false;
            return false;
        }
        fft_initialised = true;
    }

    copy_first_chirp(frame_data, antenna_data);

    for (uint32_t ant = 0U; ant < XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS; ++ant)
    {
        float32_t mean_real = 0.0f;
        float32_t mean_imag = 0.0f;

        for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
        {
            mean_real += antenna_data[ant][sample * 2U];
            mean_imag += antenna_data[ant][sample * 2U + 1U];
        }

        mean_real /= (float32_t)NUM_SAMPLES_PER_CHIRP;
        mean_imag /= (float32_t)NUM_SAMPLES_PER_CHIRP;

        for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
        {
            antenna_data[ant][sample * 2U]     -= mean_real;
            antenna_data[ant][sample * 2U + 1U] -= mean_imag;
        }

        arm_cfft_f32(&cfft_instance, antenna_data[ant], 0, 1);
    }

    for (uint32_t bin = 1U; bin < (uint32_t)NUM_SAMPLES_PER_CHIRP / 2U; ++bin)
    {
        const float32_t real_part = antenna_data[0][bin * 2U];
        const float32_t imag_part = antenna_data[0][bin * 2U + 1U];
        const float32_t magnitude = (real_part * real_part) + (imag_part * imag_part);

        if (magnitude > peak_mag)
        {
            peak_mag = magnitude;
            peak_bin = bin;
        }
    }

    if ((peak_mag <= 0.0f) || (peak_bin == 0U))
    {
        result->valid = false;
        last_valid = false;
        return false;
    }

    lambda = RADAR_LIGHT_SPEED_MPS / RADAR_CARRIER_FREQ_HZ;

    const float32_t freq_bin_hz = ((float32_t)peak_bin * RADAR_SAMPLE_RATE_HZ) /
                                  (float32_t)NUM_SAMPLES_PER_CHIRP;
    const float32_t range_m = (freq_bin_hz * RADAR_LIGHT_SPEED_MPS * RADAR_CHIRP_PERIOD_S) /
                              (2.0f * RADAR_BANDWIDTH_HZ);

    const float32_t rx1_real = antenna_data[0][peak_bin * 2U];
    const float32_t rx1_imag = antenna_data[0][peak_bin * 2U + 1U];
    const float32_t rx2_real = antenna_data[1][peak_bin * 2U];
    const float32_t rx2_imag = antenna_data[1][peak_bin * 2U + 1U];
    const float32_t rx3_real = antenna_data[2][peak_bin * 2U];
    const float32_t rx3_imag = antenna_data[2][peak_bin * 2U + 1U];

    const float32_t cross_x_real = (rx2_real * rx3_real) + (rx2_imag * rx3_imag);
    const float32_t cross_x_imag = (rx2_imag * rx3_real) - (rx2_real * rx3_imag);
    const float32_t cross_y_real = (rx1_real * rx3_real) + (rx1_imag * rx3_imag);
    const float32_t cross_y_imag = (rx1_imag * rx3_real) - (rx1_real * rx3_imag);

    const float32_t dphi_x = atan2f(cross_x_imag, cross_x_real);
    const float32_t dphi_y = atan2f(cross_y_imag, cross_y_real);

    const float32_t u = (lambda / (RADAR_TWO_PI * RADAR_ANT_SPACING_X_M)) * dphi_x;
    const float32_t v = (lambda / (RADAR_TWO_PI * RADAR_ANT_SPACING_Y_M)) * dphi_y;
    float32_t sin_theta = hypotf(u, v);

    if (sin_theta > 1.0f)
    {
        sin_theta = 1.0f;
    }

    const float32_t theta_rad = asinf(fmaxf(sin_theta, 0.0f));
    const float32_t phi_rad = atan2f(v, u);

    result->range_m = range_m;
    result->elevation_deg = theta_rad * RADAR_RAD_TO_DEG;
    result->azimuth_deg = phi_rad * RADAR_RAD_TO_DEG;
    result->peak_power_db = 10.0f * log10f(peak_mag);
    result->peak_bin = peak_bin;
    result->valid = true;

    last_result = *result;
    last_valid = true;

    return true;
}

void angle_range_print(const angle_range_result_t *result, bool compact)
{
    if ((result == NULL) || (!result->valid))
    {
        printf("angle_range: no dominant peak detected\r\n");
        return;
    }

    if (compact)
    {
        printf("[ANGLE] range=%.2fm elev=%.2fdeg az=%.2fdeg peak=%.1fdB bin=%lu\r\n",
               (double)result->range_m,
               (double)result->elevation_deg,
               (double)result->azimuth_deg,
               (double)result->peak_power_db,
               (unsigned long)result->peak_bin);
    }
    else
    {
        printf("\r\n=== PROCESSED FRAME METRICS ===\r\n");
        printf("Peak bin          : %lu\r\n", (unsigned long)result->peak_bin);
        printf("Range estimate    : %.2f m\r\n", (double)result->range_m);
        printf("Elevation (theta) : %.2f deg\r\n", (double)result->elevation_deg);
        printf("Azimuth (phi)     : %.2f deg\r\n", (double)result->azimuth_deg);
        printf("Peak power        : %.1f dB\r\n", (double)result->peak_power_db);
        printf("================================\r\n\r\n");
    }
}

void angle_range_enable_stream(bool enable)
{
    taskENTER_CRITICAL();
    stream_enabled = enable;
    taskEXIT_CRITICAL();
}

bool angle_range_stream_enabled(void)
{
    bool enabled;

    taskENTER_CRITICAL();
    enabled = stream_enabled;
    taskEXIT_CRITICAL();

    return enabled;
}

bool angle_range_get_last(angle_range_result_t *result)
{
    bool valid;

    configASSERT(result != NULL);

    taskENTER_CRITICAL();
    *result = last_result;
    valid = last_valid;
    taskEXIT_CRITICAL();

    return valid;
}
#endif /* Legacy angle/range helper disabled */

#include "angle_range.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define NUM_SAMPLES_PER_CHIRP   (128U)
#define NUM_RX_ANTENNAS         (3U)
#define NUM_CHIRPS_PER_FRAME    (16U)

#define RADAR_CARRIER_FREQ_HZ   (60.0e9f)
#define RADAR_BANDWIDTH_HZ      (2000.0e6f)
#define RADAR_CHIRP_PERIOD_S    (6.9e-5f)
#define RADAR_SAMPLE_RATE_HZ    (2352941.0f)
#define RADAR_LIGHT_SPEED_MPS   (299792458.0f)
#define RADAR_RAD_TO_DEG        (57.29577951308232f)
#define RADAR_TWO_PI            (6.283185307179586f)
#define RADAR_ANT_SPACING_X_M   (0.0025f)
#define RADAR_ANT_SPACING_Y_M   (0.0025f)

#if NUM_RX_ANTENNAS < 3
#error "AoA computation requires at least three RX antennas"
#endif

static arm_cfft_instance_f32 fft_instance;
static bool fft_ready;
static float32_t antenna_data[NUM_RX_ANTENNAS][NUM_SAMPLES_PER_CHIRP * 2U];

static void build_average_chirp(const float32_t *frame_data)
{
    const uint32_t chirp_stride = NUM_SAMPLES_PER_CHIRP * NUM_RX_ANTENNAS * 2U;
    const uint32_t sample_stride = NUM_RX_ANTENNAS * 2U;

    for (uint32_t ant = 0U; ant < NUM_RX_ANTENNAS; ++ant)
    {
        for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
        {
            antenna_data[ant][sample * 2U] = 0.0f;
            antenna_data[ant][sample * 2U + 1U] = 0.0f;
        }
    }

    for (uint32_t chirp = 0U; chirp < NUM_CHIRPS_PER_FRAME; ++chirp)
    {
        const float32_t *chirp_ptr = frame_data + (chirp * chirp_stride);

        for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
        {
            const float32_t *sample_ptr = chirp_ptr + (sample * sample_stride);

            for (uint32_t ant = 0U; ant < NUM_RX_ANTENNAS; ++ant)
            {
                const uint32_t base = ant * 2U;
                antenna_data[ant][sample * 2U]     += sample_ptr[base];
                antenna_data[ant][sample * 2U + 1U] += sample_ptr[base + 1U];
            }
        }
    }

    const float32_t scale = 1.0f / (float32_t)NUM_CHIRPS_PER_FRAME;

    for (uint32_t ant = 0U; ant < NUM_RX_ANTENNAS; ++ant)
    {
        for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
        {
            antenna_data[ant][sample * 2U]     *= scale;
            antenna_data[ant][sample * 2U + 1U] *= scale;
        }
    }
}

static bool ensure_fft_initialised(void)
{
    if (!fft_ready)
    {
        if (arm_cfft_init_f32(&fft_instance, NUM_SAMPLES_PER_CHIRP) != ARM_MATH_SUCCESS)
        {
            return false;
        }
        fft_ready = true;
    }
    return true;
}

bool angle_range_compute(const float32_t *frame_data, angle_range_result_t *result)
{
    float32_t peak_mag = 0.0f;
    uint32_t peak_bin = 0U;

    if ((frame_data == NULL) || (result == NULL))
    {
        return false;
    }

    if (!ensure_fft_initialised())
    {
        result->valid = false;
        return false;
    }

    build_average_chirp(frame_data);

    for (uint32_t ant = 0U; ant < NUM_RX_ANTENNAS; ++ant)
    {
        float32_t mean_real = 0.0f;
        float32_t mean_imag = 0.0f;

        for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
        {
            mean_real += antenna_data[ant][sample * 2U];
            mean_imag += antenna_data[ant][sample * 2U + 1U];
        }

        mean_real /= (float32_t)NUM_SAMPLES_PER_CHIRP;
        mean_imag /= (float32_t)NUM_SAMPLES_PER_CHIRP;

        for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
        {
            antenna_data[ant][sample * 2U]     -= mean_real;
            antenna_data[ant][sample * 2U + 1U] -= mean_imag;
        }

        arm_cfft_f32(&fft_instance, antenna_data[ant], 0, 1);
    }

    for (uint32_t bin = 1U; bin < NUM_SAMPLES_PER_CHIRP / 2U; ++bin)
    {
        const float32_t real_part = antenna_data[0][bin * 2U];
        const float32_t imag_part = antenna_data[0][bin * 2U + 1U];
        const float32_t magnitude = (real_part * real_part) + (imag_part * imag_part);

        if (magnitude > peak_mag)
        {
            peak_mag = magnitude;
            peak_bin = bin;
        }
    }

    if ((peak_mag <= 0.0f) || (peak_bin == 0U))
    {
        result->valid = false;
        return false;
    }

    const float32_t lambda = RADAR_LIGHT_SPEED_MPS / RADAR_CARRIER_FREQ_HZ;
    const float32_t freq_bin_hz = ((float32_t)peak_bin * RADAR_SAMPLE_RATE_HZ) /
                                  (float32_t)NUM_SAMPLES_PER_CHIRP;
    const float32_t range_m = (freq_bin_hz * RADAR_LIGHT_SPEED_MPS * RADAR_CHIRP_PERIOD_S) /
                              (2.0f * RADAR_BANDWIDTH_HZ);

    const float32_t rx1_real = antenna_data[0][peak_bin * 2U];
    const float32_t rx1_imag = antenna_data[0][peak_bin * 2U + 1U];
    const float32_t rx2_real = antenna_data[1][peak_bin * 2U];
    const float32_t rx2_imag = antenna_data[1][peak_bin * 2U + 1U];
    const float32_t rx3_real = antenna_data[2][peak_bin * 2U];
    const float32_t rx3_imag = antenna_data[2][peak_bin * 2U + 1U];

    const float32_t cross_x_real = (rx2_real * rx3_real) + (rx2_imag * rx3_imag);
    const float32_t cross_x_imag = (rx2_imag * rx3_real) - (rx2_real * rx3_imag);
    const float32_t cross_y_real = (rx1_real * rx3_real) + (rx1_imag * rx3_imag);
    const float32_t cross_y_imag = (rx1_imag * rx3_real) - (rx1_real * rx3_imag);

    const float32_t dphi_x = atan2f(cross_x_imag, cross_x_real);
    const float32_t dphi_y = atan2f(cross_y_imag, cross_y_real);

    const float32_t u = (lambda / (RADAR_TWO_PI * RADAR_ANT_SPACING_X_M)) * dphi_x;
    const float32_t v = (lambda / (RADAR_TWO_PI * RADAR_ANT_SPACING_Y_M)) * dphi_y;
    float32_t sin_theta = hypotf(u, v);

    if (sin_theta > 1.0f)
    {
        sin_theta = 1.0f;
    }

    const float32_t theta_rad = asinf(fmaxf(sin_theta, 0.0f));
    const float32_t phi_rad = atan2f(v, u);

    result->range_m = range_m;
    result->elevation_deg = theta_rad * RADAR_RAD_TO_DEG;
    result->azimuth_deg = phi_rad * RADAR_RAD_TO_DEG;
    result->peak_power_db = 10.0f * log10f(peak_mag);
    result->peak_bin = peak_bin;
    result->valid = true;

    return true;
}

void angle_range_print(const angle_range_result_t *result, bool compact)
{
    if ((result == NULL) || (!result->valid))
    {
        printf("[INFO] No dominant target detected\r\n");
        return;
    }

    if (compact)
    {
        printf("Range %.2fm | Elev %.2fdeg | Az %.2fdeg | Peak %.1fdB\r\n",
               (double)result->range_m,
               (double)result->elevation_deg,
               (double)result->azimuth_deg,
               (double)result->peak_power_db);
    }
    else
    {
        printf("\r\n=== AoA METRICS ===\r\n");
        printf("Peak bin          : %lu\r\n", (unsigned long)result->peak_bin);
        printf("Range estimate    : %.2f m\r\n", (double)result->range_m);
        printf("Elevation (theta) : %.2f deg\r\n", (double)result->elevation_deg);
        printf("Azimuth (phi)     : %.2f deg\r\n", (double)result->azimuth_deg);
        printf("Peak power        : %.1f dB\r\n", (double)result->peak_power_db);
        printf("====================\r\n");
    }
}
