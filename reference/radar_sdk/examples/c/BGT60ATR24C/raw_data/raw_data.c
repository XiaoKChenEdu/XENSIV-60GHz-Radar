/* ===========================================================================
** Copyright (C) 2021-2022 Infineon Technologies AG
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
** 1. Redistributions of source code must retain the above copyright notice,
**    this list of conditions and the following disclaimer.
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
** 3. Neither the name of the copyright holder nor the names of its
**    contributors may be used to endorse or promote products derived from
**    this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
** INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
** CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
** ===========================================================================
*/

/**
 * @file    raw_data.c
 *
 * @brief   Raw data example.
 *
 * This example illustrates how to fetch time-domain data from an Avian family of FMCW
 * radar sensor like BGT60TR13, BGT60UTR11AIP or BGT60ATR24 using the Radar SDK.
 */

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "ifxBase/Base.h"
#include "ifxBase/internal/Util.h"  // for ifx_util_popcount
#include "ifxFmcw/DeviceFmcw.h"

/*
==============================================================================
   2. LOCAL DEFINITIONS
==============================================================================
*/

#define NUM_FETCHED_FRAMES 10 /**< Number of frames to fetch */

/*
==============================================================================
   6. LOCAL FUNCTIONS
==============================================================================
*/

/**
 * @brief Helper function to process the antenna data, by summing it up.
 *
 * This function is an example showing a possible way
 * of processing antenna signal. The goal in this example is to
 * sum up all chirps into one vector.
 *
 * @param antenna_data data from one antenna containing multiple chirps
 */
static void sum_and_display_antenna_data(const ifx_Matrix_R_t* antenna_data)
{
    ifx_Vector_R_t chirp = {0};
    // Create the sum vector
    ifx_Vector_R_t* sum = ifx_vec_create_r(IFX_MAT_COLS(antenna_data));

    // Iterate through all chirps
    for (uint32_t i = 0; i < IFX_MAT_ROWS(antenna_data); i++)
    {
        // Fetch a chirp from the antenna data matrix
        ifx_mat_get_rowview_r(antenna_data, i, &chirp);
        // add it to the sum vector
        ifx_vec_add_r(&chirp, sum, sum);
    }

    // Divide the sum vector element wise by number of chirps in the antenna data
    ifx_vec_scale_r(sum, 1.0f / IFX_MAT_ROWS(antenna_data), sum);

    for (uint32_t i = 0; i < IFX_VEC_LEN(sum); i++)
    {
        printf("%.4f ", IFX_VEC_AT(sum, i));
    }
    printf("\n\n");
    ifx_vec_destroy_r(sum);
}

//----------------------------------------------------------------------------

/**
 * @brief Helper function which sums, and displays the antenna data, given the deinterleaved frame,
 * as an example for frame processing,
 *
 * Separates different antenna signals, and pass them for further processing.
 * Specificaly, the function processes the deinterleaved frame, by summing up the data
 * for each row, using sum_and_display_antenna_data helper function.
 * Note: This function assumes the sequence is comprised of one chirp nested in a loop, within the frame loop.
 *
 * @param[in] num_chirps The number of chirps.
 * @param[in] num_samples_per_chirp The number of samples per chirp.
 * @param[in] num_rx The number of rx antennas.
 * @param[in] adc_max_value The adc maximum value.
 * @param[in] frame The frame may contain multiple antenna signals,
 *            depending on the device configuration.
 *            Each antenna signal can contain multiple chirps.
 */
static void sum_and_display_frame_data(uint32_t num_chirps, uint32_t num_samples_per_chirp, uint32_t num_rx, ifx_Float_t adc_max_value, ifx_Fmcw_Frame_t* deinterleaved_frame)
{
    ifx_Mda_R_t* data_cube = deinterleaved_frame->cubes[0];
    ifx_Matrix_R_t antenna_data;
    for (uint32_t i = 0; i < IFX_CUBE_ROWS(data_cube); i++)
    {
        ifx_cube_get_row_r(data_cube, i, &antenna_data);
        sum_and_display_antenna_data(&antenna_data);
    }
}

/**
 * @brief Helper function to get the frame dimensions assuming there is only one chirp in the sequence.
 *
 * @param fmcw_Sequence_Element      pointer to first Sequence Element
 * @param num_rx                     pointer to the number of RX antennes.
 * @param num_samples_per_chirp      pointer to the number of samples per chirp
 */
static void get_frame_dimensions(const ifx_Fmcw_Sequence_Element_t* element, uint32_t* num_rx, uint32_t* num_samples_per_chirp)
{
    if (element == NULL)
    {
        *num_samples_per_chirp = 0;
        *num_rx = 0;
        return;
    }

    // At the beginning, it is needed to check if the sequence starts with the frame loop in order to skip it
    if ((element->type == IFX_SEQ_LOOP) && (element->next_element == NULL))
    {
        element = element->loop.sub_sequence;
    }

    while (element != NULL)
    {
        switch (element->type)
        {
            case IFX_SEQ_LOOP:
                element = element->loop.sub_sequence;
                continue;
            case IFX_SEQ_CHIRP:
                *num_samples_per_chirp = element->chirp.num_samples;
                *num_rx = ifx_util_popcount(element->chirp.rx_mask);
                break;
            default:
                break;
        }
        element = element->next_element;
    }
}

/*
==============================================================================
   7. MAIN METHOD
==============================================================================
 */

int main(int argc, char** argv)
{
    ifx_Error_t error = IFX_OK;
    ifx_Fmcw_Raw_Frame_t* frame = NULL;
    ifx_Fmcw_Frame_t* deinterleaved_frame = NULL;
    ifx_Float_t* converted_frame = NULL;
    ifx_Fmcw_Sequence_Element_t* sequence = NULL;
    ifx_Device_Fmcw_t* fmcw_device;

    printf("Radar SDK Version: %s\n", ifx_sdk_get_version_string_full());

    /* Open the device: Connect to the first radar sensor found. */
    fmcw_device = ifx_fmcw_create();
    if ((error = ifx_error_get()) != IFX_OK)
    {
        fprintf(stderr, "Failed to open device: %s\n", ifx_error_to_string(error));
        goto out;
    }

    const char* uuid = ifx_fmcw_get_board_uuid(fmcw_device);
    printf("UUID of board: %s\n", uuid);

    // Load register file, to overwrite the defaults and configure the parameters not exposed in the FMCW API
    // ifx_fmcw_load_register_file(fmcw_device, "config_regs_filename.txt");

    /* A device instance is initialised with the default acquisition
     * sequence for its corresponding radar sensor. This sequence can be
     * simply fetched, analyzed or modified by the user.
     */
    sequence = ifx_fmcw_get_acquisition_sequence(fmcw_device);
    if ((error = ifx_error_get()) != IFX_OK)
    {
        fprintf(stderr, "Failed to get acquisition_sequence:  %s\n", ifx_error_to_string(error));
        goto out;
    }

    /* Print the current device acquisition sequence */
    ifx_fmcw_print_sequence(sequence);

    /* Allocate memory for frame */
    frame = ifx_fmcw_allocate_raw_frame(fmcw_device);
    deinterleaved_frame = ifx_fmcw_allocate_frame(fmcw_device);
    converted_frame = (ifx_Float_t*)calloc(frame->num_samples, sizeof(ifx_Float_t));

    /* Get the frame dimensions for de-interleving, and then processing */
    uint32_t num_rx = 0;
    uint32_t num_samples_per_chirp = 0;
    get_frame_dimensions(sequence, &num_rx, &num_samples_per_chirp);
    if (!num_rx || !num_samples_per_chirp)
    {
        fprintf(stderr, "Failed to determine frame dimensions");
        goto out;
    }
    const uint32_t num_chirps = frame->num_samples / (num_rx * num_samples_per_chirp);
    float adc_max_value = (float)(1 << (ifx_fmcw_get_sensor_information(fmcw_device)->adc_resolution_bits - 1)) - 1;

    /* Fetch NUM_FETCHED_FRAMES number of frames. */
    for (int frame_number = 0; frame_number < NUM_FETCHED_FRAMES; frame_number++)
    {
        /* Get the time-domain data for the next frame. The function will block
         * until the full frame is available and copy the data into the frame
         * handle.
         * This function also creates a frame structure for time domain data
         * acquisition, if not created already. It is the responsibility of
         * the caller to free the returned frame in this scope.
         */
        ifx_fmcw_get_next_raw_frame(fmcw_device, frame);
        if ((error = ifx_error_get()) != IFX_OK)
        {
            fprintf(stderr, "Failed to get next frame: %s\n", ifx_error_to_string(error));
            goto out;
        }

        /* De-interleave frame */
        ifx_fmcw_convert_raw_data_to_float_array(fmcw_device, frame->num_samples, frame->samples, converted_frame);
        ifx_fmcw_view_deinterleaved_frame(fmcw_device, converted_frame, deinterleaved_frame);

        /* Process the frame */
        sum_and_display_frame_data(num_chirps, num_samples_per_chirp, num_rx, adc_max_value, deinterleaved_frame);
    }

    ifx_fmcw_stop_acquisition(fmcw_device);

out:
    /* Close the device after processing all frames. It is valid to pass NULL
     * to destroy functions.
     */
    ifx_fmcw_destroy_raw_frame(frame);
    ifx_fmcw_destroy_frame(deinterleaved_frame);
    free(converted_frame);
    ifx_fmcw_destroy_sequence(sequence);
    ifx_fmcw_destroy(fmcw_device);

    return error == IFX_OK ? EXIT_SUCCESS : EXIT_FAILURE;
}
