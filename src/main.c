#include <inttypes.h>
#include <limits.h>
#include <string.h>

#include "cyhal.h"
#include "cybsp.h"

#include "cy_retarget_io.h"
#include "xensiv_bgt60trxx_mtb.h"

#define XENSIV_BGT60TRXX_CONF_IMPL
#include "presence_radar_settings.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* sensor SPI interface */
#define PIN_XENSIV_BGT60TRXX_SPI_SCLK       CYBSP_SPI_CLK
#define PIN_XENSIV_BGT60TRXX_SPI_MOSI       CYBSP_SPI_MOSI
#define PIN_XENSIV_BGT60TRXX_SPI_MISO       CYBSP_SPI_MISO
#define PIN_XENSIV_BGT60TRXX_SPI_CSN        CYBSP_SPI_CS

/* sensor interrupt output pin */
#define PIN_XENSIV_BGT60TRXX_IRQ            CYBSP_GPIO10
/* sensor HW reset pin */
#define PIN_XENSIV_BGT60TRXX_RSTN           CYBSP_GPIO11
/* enable 1V8 LDO on radar wingboard*/
#define PIN_XENSIV_BGT60TRXX_LDO_EN         CYBSP_GPIO5

#define XENSIV_BGT60TRXX_SPI_FREQUENCY      (25000000UL)

#define NUM_SAMPLES_PER_FRAME               (XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS *\
                                             XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *\
                                             XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP)

/*******************************************************************************
* Global variables
*******************************************************************************/
static cyhal_spi_t cyhal_spi;
static xensiv_bgt60trxx_mtb_t sensor;
static volatile bool data_available = false;
static bool capture_enabled = false;
static bool frame_limit_enabled = false;
static uint32_t frame_limit_total = 0U;
static uint32_t frame_limit_sent = 0U;

/* Allocate enough memory for the radar dara frame. */
static uint16_t samples[NUM_SAMPLES_PER_FRAME];

static bool parse_frame_count_argument(const char *arg, uint32_t *out_value);
static void handle_command(const char *cmd);
static void process_cli(void);

/* Interrupt handler to react on sensor indicating the availability of new data */
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void xensiv_bgt60trxx_mtb_interrupt_handler(void *args, cyhal_gpio_event_t event)
#else
void xensiv_bgt60trxx_mtb_interrupt_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);
    data_available = true;
}

int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals. */
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port. */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    printf("XENSIV BGT60TRxx Example\r\n");

    /* Initialize the SPI interface to BGT60. */
    result = cyhal_spi_init(&cyhal_spi,
                            PIN_XENSIV_BGT60TRXX_SPI_MOSI,
                            PIN_XENSIV_BGT60TRXX_SPI_MISO,
                            PIN_XENSIV_BGT60TRXX_SPI_SCLK,
                            NC,
                            NULL,
                            8,
                            CYHAL_SPI_MODE_00_MSB,
                            false);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Reduce drive strength to improve EMI */
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CY_GPIO_DRIVE_1_8);

    /* Set SPI data rate to communicate with sensor */
    result = cyhal_spi_set_frequency(&cyhal_spi, XENSIV_BGT60TRXX_SPI_FREQUENCY);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Enable the LDO. */
    result = cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_LDO_EN,
                             CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG,
                             true);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Wait LDO stable */
    (void)cyhal_system_delay_ms(5);

    result = xensiv_bgt60trxx_mtb_init(&sensor,
                                       &cyhal_spi,
                                       PIN_XENSIV_BGT60TRXX_SPI_CSN,
                                       PIN_XENSIV_BGT60TRXX_RSTN,
                                       register_list,
                                       XENSIV_BGT60TRXX_CONF_NUM_REGS);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* The sensor will generate an interrupt once the sensor FIFO level is
       NUM_SAMPLES_PER_FRAME */
    result = xensiv_bgt60trxx_mtb_interrupt_init(&sensor,
                                                 NUM_SAMPLES_PER_FRAME,
                                                 PIN_XENSIV_BGT60TRXX_IRQ,
                                                 CYHAL_ISR_PRIORITY_DEFAULT,
                                                 xensiv_bgt60trxx_mtb_interrupt_handler,
                                                 NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Ensure acquisition is idle until commanded via CLI */
    if (xensiv_bgt60trxx_start_frame(&sensor.dev, false) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        CY_ASSERT(0);
    }

    printf("Ready. Type 'start' [frames] or 'stop' followed by Enter.\r\n");

    uint32_t frame_idx = 0;

    for(;;)
    {
        process_cli();

        if (!capture_enabled)
        {
            cyhal_system_delay_ms(10);
            continue;
        }

        /* Wait for the radar device to indicate the availability of the data to fetch. */
        while ((capture_enabled == true) && (data_available == false))
        {
            process_cli();
            cyhal_system_delay_ms(1);
        }

        if (!capture_enabled)
        {
            continue;
        }

        data_available = false;

        if (xensiv_bgt60trxx_get_fifo_data(&sensor.dev, samples,
                                           NUM_SAMPLES_PER_FRAME) == XENSIV_BGT60TRXX_STATUS_OK)
        {
            printf("\r\n========== Frame %" PRIu32 " START ==========%s", frame_idx, "\r\n");
            
            /* Print data organized by chirp, with RX antennas interleaved */
            for (uint32_t chirp = 0; chirp < XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME; ++chirp)
            {
                printf("\r\nChirp %" PRIu32 ":\r\n", chirp + 1U);

                for (uint32_t sample = 0; sample < XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP; ++sample)
                {
                    printf("  Sample %" PRIu32 ": [", sample + 1U);

                    for (uint32_t rx = 0; rx < XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS; ++rx)
                    {
                        uint32_t idx = (chirp * XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS) +
                                      (sample * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS) +
                                      rx;

                        printf("%s%" PRIu16, (rx == 0U) ? "" : ", ", samples[idx]);
                    }
                    printf("]\r\n");
                }
            }
            
            printf("========== Frame %" PRIu32 " END ==========%s", frame_idx, "\r\n");
            frame_idx++;

            if (frame_limit_enabled)
            {
                frame_limit_sent++;

                if (frame_limit_sent >= frame_limit_total)
                {
                    uint32_t completed_frames = frame_limit_total;

                    if (xensiv_bgt60trxx_start_frame(&sensor.dev, false) == XENSIV_BGT60TRXX_STATUS_OK)
                    {
                        capture_enabled = false;
                        data_available = false;
                        frame_limit_enabled = false;
                        frame_limit_total = 0U;
                        frame_limit_sent = 0U;
                        printf("Capture completed (%" PRIu32 " frame%s).\r\n",
                               completed_frames,
                               (completed_frames == 1U) ? "" : "s");
                    }
                    else
                    {
                        printf("Failed to stop capture.\r\n");
                    }
                }
            }
        }
    }
}

static bool parse_frame_count_argument(const char *arg, uint32_t *out_value)
{
    if ((arg == NULL) || (out_value == NULL))
    {
        return false;
    }

    while ((*arg == ' ') || (*arg == '\t'))
    {
        ++arg;
    }

    if (*arg == '\0')
    {
        *out_value = 0U;
        return true;
    }

    uint32_t value = 0U;

    while ((*arg >= '0') && (*arg <= '9'))
    {
        uint32_t digit = (uint32_t)(*arg - '0');

        if (value > ((UINT32_MAX - digit) / 10U))
        {
            return false;
        }

        value = (value * 10U) + digit;
        ++arg;
    }

    while ((*arg == ' ') || (*arg == '\t'))
    {
        ++arg;
    }

    if (*arg != '\0')
    {
        return false;
    }

    *out_value = value;
    return true;
}

static void handle_command(const char *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    while ((*cmd == ' ') || (*cmd == '\t'))
    {
        ++cmd;
    }

    if ((strncmp(cmd, "start", 5) == 0) &&
        ((cmd[5] == '\0') || (cmd[5] == ' ') || (cmd[5] == '\t')))
    {
        uint32_t requested_frames = 0U;

        if (!parse_frame_count_argument(cmd + 5, &requested_frames))
        {
            printf("Invalid frame count.\r\n");
            return;
        }

        if (capture_enabled)
        {
            printf("Capture already running.\r\n");
            return;
        }

        if (xensiv_bgt60trxx_start_frame(&sensor.dev, true) == XENSIV_BGT60TRXX_STATUS_OK)
        {
            capture_enabled = true;
            data_available = false;
            frame_limit_enabled = (requested_frames > 0U);
            frame_limit_total = requested_frames;
            frame_limit_sent = 0U;

            if (frame_limit_enabled)
            {
                printf("Capture started (%" PRIu32 " frame%s).\r\n",
                       requested_frames,
                       (requested_frames == 1U) ? "" : "s");
            }
            else
            {
                printf("Capture started (continuous).\r\n");
            }
        }
        else
        {
            printf("Failed to start capture.\r\n");
        }
    }
    else if ((strncmp(cmd, "stop", 4) == 0) &&
             ((cmd[4] == '\0') || (cmd[4] == ' ') || (cmd[4] == '\t')))
    {
        const char *trailing = cmd + 4;

        while ((*trailing == ' ') || (*trailing == '\t'))
        {
            ++trailing;
        }

        if (*trailing != '\0')
        {
            printf("Unknown command: %s\r\n", cmd);
            return;
        }

        if (!capture_enabled)
        {
            printf("Capture already stopped.\r\n");
            return;
        }

        if (xensiv_bgt60trxx_start_frame(&sensor.dev, false) == XENSIV_BGT60TRXX_STATUS_OK)
        {
            capture_enabled = false;
            data_available = false;
            frame_limit_enabled = false;
            frame_limit_total = 0U;
            frame_limit_sent = 0U;
            printf("Capture stopped.\r\n");
        }
        else
        {
            printf("Failed to stop capture.\r\n");
        }
    }
    else if (*cmd != '\0')
    {
        printf("Unknown command: %s\r\n", cmd);
    }
}

static void process_cli(void)
{
    static char cmd_buffer[32];
    static uint32_t cmd_index = 0;

    while (cyhal_uart_readable(&cy_retarget_io_uart_obj) > 0)
    {
        uint8_t ch = 0;
        if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &ch, 1) != CY_RSLT_SUCCESS)
        {
            break;
        }

        if ((ch == '\r') || (ch == '\n'))
        {
            if (cmd_index > 0)
            {
                cmd_buffer[cmd_index] = '\0';
                handle_command(cmd_buffer);
                cmd_index = 0;
            }
        }
        else
        {
            if (cmd_index < (sizeof(cmd_buffer) - 1))
            {
                cmd_buffer[cmd_index++] = (char)ch;
            }
            else
            {
                /* Command too long; reset buffer */
                cmd_index = 0;
            }
        }
    }
}

