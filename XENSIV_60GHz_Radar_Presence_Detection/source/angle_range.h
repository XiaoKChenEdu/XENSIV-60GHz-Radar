/*
 * angle_range.h
 * Utility routines for extracting range and angle estimates from raw radar frames.
 */
#ifndef SOURCE_ANGLE_RANGE_H_
#define SOURCE_ANGLE_RANGE_H_

#include <stdbool.h>
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float32_t range_m;
    float32_t elevation_deg;
    float32_t azimuth_deg;
    float32_t peak_power_db;
    uint32_t  peak_bin;
    bool      valid;
} angle_range_result_t;

bool angle_range_compute(const float32_t *frame_data, angle_range_result_t *result);
void angle_range_print(const angle_range_result_t *result, bool compact);

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_ANGLE_RANGE_H_ */
