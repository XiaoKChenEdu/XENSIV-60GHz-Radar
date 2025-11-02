/*
 * angle_range.c
 * Implements improved range and angle estimation for human tracking using CMSIS-DSP.
 * 
 * Antenna Configuration:
 * - RX1 (antenna[0]) and RX3 (antenna[2]) used for X-axis (azimuth) tracking
 * - RX2 (antenna[1]) and RX3 (antenna[2]) used for Y-axis (elevation) tracking
 */

#include "angle_range.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/*                             Configuration                                  */
/* ========================================================================== */
#define NUM_SAMPLES_PER_CHIRP   (128U)
#define NUM_RX_ANTENNAS         (3U)
#define NUM_CHIRPS_PER_FRAME    (16U)

/* Radar Parameters (BGT60TR13C) */
#define RADAR_CARRIER_FREQ_HZ   (60.0e9f)         /* Center frequency: 60 GHz */
#define RADAR_BANDWIDTH_HZ      (2000.0e6f)       /* Bandwidth: 2 GHz */
#define RADAR_CHIRP_PERIOD_S    (6.945e-5f)       /* Chirp time: ~69.45 us */
#define RADAR_SAMPLE_RATE_HZ    (2352941.0f)      /* Sample rate: ~2.35 MHz */
#define RADAR_LIGHT_SPEED_MPS   (299792458.0f)    /* Speed of light: ~3e8 m/s */

/* Mathematical Constants */
#define RADAR_RAD_TO_DEG        (57.29577951308232f)  /* 180/pi */
#define RADAR_TWO_PI            (6.283185307179586f)  /* 2*pi */

/* Antenna Spacing (BGT60TR13C typical) */
#define RADAR_ANT_SPACING_X_M   (0.0025f)         /* 2.5mm spacing (X-axis: RX1-RX3) */
#define RADAR_ANT_SPACING_Y_M   (0.0025f)         /* 2.5mm spacing (Y-axis: RX2-RX3) */

/* Human Detection Parameters */
#define MIN_DETECTION_RANGE_M   (0.3f)            /* Minimum detection range: 30cm */
#define MAX_DETECTION_RANGE_M   (5.0f)            /* Maximum detection range: 5m */
#define DETECTION_THRESHOLD_DB  (-30.0f)          /* Minimum peak threshold in dB */

/* Antenna indices for angle computation */
#define RX1_IDX  (0U)  /* RX1: Used for X-axis (azimuth) with RX3 */
#define RX2_IDX  (1U)  /* RX2: Used for Y-axis (elevation) with RX3 */
#define RX3_IDX  (2U)  /* RX3: Reference antenna for both axes */

#if NUM_RX_ANTENNAS < 3
#error "Angle computation requires at least three RX antennas"
#endif

/* ========================================================================== */
/*                          Static Module Data                                */
/* ========================================================================== */
static arm_cfft_instance_f32 fft_instance;
static bool fft_ready = false;
static float32_t range_spectra[NUM_RX_ANTENNAS][NUM_SAMPLES_PER_CHIRP * 2U];
static float32_t chirp_buffer[NUM_SAMPLES_PER_CHIRP * 2U];

/* Hanning window coefficients for better spectral leakage suppression */
static float32_t hanning_window[NUM_SAMPLES_PER_CHIRP];
static bool window_initialized = false;

/* ========================================================================== */
/*                        Private Helper Functions                            */
/* ========================================================================== */

/**
 * @brief Initialize Hanning window for windowing function
 * 
 * Windowing reduces spectral leakage and improves peak detection accuracy
 */
static void init_hanning_window(void)
{
    if (window_initialized)
    {
        return;
    }

    for (uint32_t n = 0U; n < NUM_SAMPLES_PER_CHIRP; ++n)
    {
        hanning_window[n] = 0.5f * (1.0f - cosf((RADAR_TWO_PI * (float32_t)n) / 
                                                 (float32_t)(NUM_SAMPLES_PER_CHIRP - 1U)));
    }
    window_initialized = true;
}

/**
 * @brief Compute coherent range spectra for each antenna across all chirps
 *
 * The incoming frame layout is [chirp][sample][antenna][I/Q]. For every chirp
 * and antenna the routine performs DC removal, applies the configured window,
 * executes a complex FFT, and accumulates the spectra coherently. The final
 * spectra stored in @ref range_spectra are averaged over all chirps so the
 * magnitude reflects the steady-state response per antenna.
 */
static void compute_range_spectra(const float32_t *frame_data)
{
    const uint32_t chirp_stride = NUM_SAMPLES_PER_CHIRP * NUM_RX_ANTENNAS * 2U;
    const uint32_t sample_stride = NUM_RX_ANTENNAS * 2U;

    /* Reset accumulated spectra */
    for (uint32_t ant = 0U; ant < NUM_RX_ANTENNAS; ++ant)
    {
        memset(range_spectra[ant], 0, NUM_SAMPLES_PER_CHIRP * 2U * sizeof(float32_t));
    }

    for (uint32_t chirp = 0U; chirp < NUM_CHIRPS_PER_FRAME; ++chirp)
    {
        const float32_t *chirp_ptr = frame_data + (chirp * chirp_stride);

        for (uint32_t ant = 0U; ant < NUM_RX_ANTENNAS; ++ant)
        {
            float32_t mean_real = 0.0f;
            float32_t mean_imag = 0.0f;

            /* Copy samples for this antenna/chirp into workspace */
            for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
            {
                const float32_t *sample_ptr = chirp_ptr + (sample * sample_stride);
                const uint32_t base = ant * 2U;
                const uint32_t idx = sample * 2U;

                const float32_t real = sample_ptr[base];
                const float32_t imag = sample_ptr[base + 1U];

                chirp_buffer[idx]     = real;
                chirp_buffer[idx + 1U] = imag;

                mean_real += real;
                mean_imag += imag;
            }

            mean_real /= (float32_t)NUM_SAMPLES_PER_CHIRP;
            mean_imag /= (float32_t)NUM_SAMPLES_PER_CHIRP;

            /* Apply DC removal and windowing */
            for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
            {
                const float32_t window = hanning_window[sample];
                const uint32_t idx = sample * 2U;

                chirp_buffer[idx] = (chirp_buffer[idx] - mean_real) * window;
                chirp_buffer[idx + 1U] = (chirp_buffer[idx + 1U] - mean_imag) * window;
            }

            arm_cfft_f32(&fft_instance, chirp_buffer, 0, 1);

            for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
            {
                const uint32_t idx = sample * 2U;
                range_spectra[ant][idx]     += chirp_buffer[idx];
                range_spectra[ant][idx + 1U] += chirp_buffer[idx + 1U];
            }
        }
    }

    const float32_t scale = 1.0f / (float32_t)NUM_CHIRPS_PER_FRAME;

    for (uint32_t ant = 0U; ant < NUM_RX_ANTENNAS; ++ant)
    {
        for (uint32_t sample = 0U; sample < NUM_SAMPLES_PER_CHIRP; ++sample)
        {
            const uint32_t idx = sample * 2U;
            range_spectra[ant][idx]     *= scale;
            range_spectra[ant][idx + 1U] *= scale;
        }
    }
}

/**
 * @brief Ensure FFT is initialized
 * 
 * @return true if FFT is ready, false otherwise
 */
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

/**
 * @brief Convert bin number to range in meters
 * 
 * @param bin FFT bin index
 * @return Range in meters
 */
static inline float32_t bin_to_range_m(uint32_t bin)
{
    const float32_t freq_bin_hz = ((float32_t)bin * RADAR_SAMPLE_RATE_HZ) / 
                                  (float32_t)NUM_SAMPLES_PER_CHIRP;
    return (freq_bin_hz * RADAR_LIGHT_SPEED_MPS * RADAR_CHIRP_PERIOD_S) /
           (2.0f * RADAR_BANDWIDTH_HZ);
}

/**
 * @brief Find peak in range spectrum with range gating
 * 
 * This function searches for the strongest peak within the valid human detection
 * range to avoid false detections from very close or very far objects
 * 
 * @param antenna_idx Which antenna data to use for peak search
 * @param peak_bin Output: bin index of peak
 * @param peak_mag Output: magnitude of peak
 * @return true if valid peak found, false otherwise
 */
static bool find_peak_in_range(uint32_t *peak_bin, float32_t *peak_mag)
{
    *peak_mag = 0.0f;
    *peak_bin = 0U;

    /* Calculate bin range corresponding to detection limits */
    uint32_t min_bin = 1U;  /* Skip DC bin */
    uint32_t max_bin = NUM_SAMPLES_PER_CHIRP / 2U;

    /* Apply range gating for human detection */
    for (uint32_t bin = min_bin; bin < max_bin; ++bin)
    {
        const float32_t range_m = bin_to_range_m(bin);
        
        /* Check if bin is within valid detection range */
        if ((range_m < MIN_DETECTION_RANGE_M) || (range_m > MAX_DETECTION_RANGE_M))
        {
            continue;
        }

        float32_t magnitude = 0.0f;

        for (uint32_t ant = 0U; ant < NUM_RX_ANTENNAS; ++ant)
        {
            const float32_t real_part = range_spectra[ant][bin * 2U];
            const float32_t imag_part = range_spectra[ant][bin * 2U + 1U];
            magnitude += (real_part * real_part) + (imag_part * imag_part);
        }

        magnitude /= (float32_t)NUM_RX_ANTENNAS;

        if (magnitude > *peak_mag)
        {
            *peak_mag = magnitude;
            *peak_bin = bin;
        }
    }

    /* Check if peak exceeds threshold */
    if ((*peak_mag > 0.0f) && (*peak_bin > 0U))
    {
        const float32_t peak_db = 10.0f * log10f(*peak_mag);
        return (peak_db >= DETECTION_THRESHOLD_DB);
    }

    return false;
}

/**
 * @brief Compute phase difference between two antennas for angle estimation
 * 
 * Uses conjugate multiplication to find relative phase between antenna pairs
 * 
 * @param ant1_idx First antenna index
 * @param ant2_idx Second antenna index (reference)
 * @param bin FFT bin to analyze
 * @return Phase difference in radians
 */
static float32_t compute_phase_diff(uint32_t ant1_idx, uint32_t ant2_idx, uint32_t bin)
{
    /* Extract complex values at peak bin */
    const float32_t a1_real = range_spectra[ant1_idx][bin * 2U];
    const float32_t a1_imag = range_spectra[ant1_idx][bin * 2U + 1U];
    const float32_t a2_real = range_spectra[ant2_idx][bin * 2U];
    const float32_t a2_imag = range_spectra[ant2_idx][bin * 2U + 1U];

    /* Conjugate multiplication: a1 * conj(a2) */
    const float32_t cross_real = (a1_real * a2_real) + (a1_imag * a2_imag);
    const float32_t cross_imag = (a1_imag * a2_real) - (a1_real * a2_imag);

    /* Return phase angle */
    return atan2f(cross_imag, cross_real);
}

/* ========================================================================== */
/*                        Public API Functions                                */
/* ========================================================================== */

/**
 * @brief Compute range and angle from radar frame
 * 
 * This improved implementation includes:
 * - Coherent chirp averaging for better SNR
 * - DC removal and windowing for cleaner spectrum
 * - Range gating for human detection
 * - Proper antenna pairing: RX1+RX3 for X-axis, RX2+RX3 for Y-axis
 * - Threshold-based detection
 * 
 * @param frame_data Pointer to frame data buffer
 * @param result Pointer to result structure
 * @return true if target detected, false otherwise
 */
bool angle_range_compute(const float32_t *frame_data, angle_range_result_t *result)
{
    if ((frame_data == NULL) || (result == NULL))
    {
        return false;
    }

    result->valid = false;

    if (!ensure_fft_initialised())
    {
        return false;
    }
    init_hanning_window();

    compute_range_spectra(frame_data);

    uint32_t peak_bin = 0U;
    float32_t peak_mag = 0.0f;

    if (!find_peak_in_range(&peak_bin, &peak_mag))
    {
        return false;
    }

    const float32_t peak_db = 10.0f * log10f(peak_mag + 1.0e-12f);
    if (peak_db < DETECTION_THRESHOLD_DB)
    {
        return false;
    }

    const float32_t range_m = bin_to_range_m(peak_bin);
    const float32_t lambda = RADAR_LIGHT_SPEED_MPS / RADAR_CARRIER_FREQ_HZ;

    const float32_t dphi_x = compute_phase_diff(RX1_IDX, RX3_IDX, peak_bin);
    const float32_t dphi_y = compute_phase_diff(RX2_IDX, RX3_IDX, peak_bin);

    const float32_t u = (lambda / (RADAR_TWO_PI * RADAR_ANT_SPACING_X_M)) * dphi_x;
    const float32_t v = (lambda / (RADAR_TWO_PI * RADAR_ANT_SPACING_Y_M)) * dphi_y;

    float32_t sin_theta = hypotf(u, v);
    if (sin_theta > 1.0f)
    {
        sin_theta = 1.0f;
    }
    const float32_t theta_rad = asinf(sin_theta);
    const float32_t phi_rad = atan2f(v, u);

    result->range_m = range_m;
    result->elevation_deg = theta_rad * RADAR_RAD_TO_DEG;
    result->azimuth_deg = phi_rad * RADAR_RAD_TO_DEG;
    result->peak_power_db = peak_db;
    result->peak_bin = peak_bin;
    result->valid = true;

    return true;
}

/**
 * @brief Print angle and range results
 * 
 * @param result Pointer to result structure
 * @param compact Use compact single-line format if true
 */
void angle_range_print(const angle_range_result_t *result, bool compact)
{
    if ((result == NULL) || (!result->valid))
    {
        printf("[INFO] No target detected in range\r\n");
        return;
    }

    if (compact)
    {
        /* Compact format for continuous monitoring */
        printf("Target: Range=%.2fm | Azimuth=%.1f° | Elevation=%.1f° | Power=%.1fdB\r\n",
               (double)result->range_m,
               (double)result->azimuth_deg,
               (double)result->elevation_deg,
               (double)result->peak_power_db);
    }
    else
    {
        /* Detailed format */
        printf("\r\n====== HUMAN TRACKING RESULT ======\r\n");
        printf("Range (distance)  : %.2f m\r\n", (double)result->range_m);
        printf("Azimuth (X-axis)  : %.1f degrees\r\n", (double)result->azimuth_deg);
        printf("Elevation (Y-axis): %.1f degrees\r\n", (double)result->elevation_deg);
        printf("Signal strength   : %.1f dB\r\n", (double)result->peak_power_db);
        printf("Range bin         : %lu\r\n", (unsigned long)result->peak_bin);
        printf("===================================\r\n");
    }
}

