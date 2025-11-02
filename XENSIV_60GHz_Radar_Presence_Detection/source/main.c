#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "xensiv_bgt60trxx.h"
#include "xensiv_bgt60trxx_mtb.h"

#include "radar_device_config.h"
#include "radar_low_framerate_config.h"
#include "resource_map.h"
#include "angle_range.h"

extern cyhal_uart_t cy_retarget_io_uart_obj;

#define RADAR_TASK_STACK_SIZE   (configMINIMAL_STACK_SIZE * 16U)
#define RADAR_TASK_PRIORITY     (tskIDLE_PRIORITY + 2U)

#define COMMAND_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 4U)
#define COMMAND_TASK_PRIORITY   (tskIDLE_PRIORITY + 1U)
#define RADAR_CMD_QUEUE_LENGTH  (8U)

#define RAW_SAMPLES_PER_FRAME   (XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP * \
                                 XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME * \
                                 XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS * 2U)

#define GPIO_INTERRUPT_PRIORITY (7U)

#if defined(XENSIV_BGT60TRXX_CONF_NUM_REGS_MACRO)
#define RADAR_REGISTER_COUNT XENSIV_BGT60TRXX_CONF_NUM_REGS_MACRO
#else
#define RADAR_REGISTER_COUNT XENSIV_BGT60TRXX_CONF_NUM_REGS
#endif

static cyhal_spi_t spi_obj;
static xensiv_bgt60trxx_mtb_t radar_dev;
static TaskHandle_t radar_task_handle;

static uint16_t raw_frame[RAW_SAMPLES_PER_FRAME];
static float32_t frame_buffer[RAW_SAMPLES_PER_FRAME];

typedef enum
{
    RADAR_COMMAND_RUN,
    RADAR_COMMAND_STOP,
    RADAR_COMMAND_RAW_DATA,
    RADAR_COMMAND_RAW_STREAM
} radar_command_type_t;

typedef struct
{
    radar_command_type_t type;
    uint32_t param;
} radar_command_t;

static QueueHandle_t radar_cmd_queue;
static bool radar_running;
static bool raw_streaming;
static bool raw_stream_resume_tracking;
static uint32_t raw_stream_frame_count;

static void radar_task(void *arg);
static void command_task(void *arg);
static cy_rslt_t init_sensor(void);
static bool acquire_frame(void);
static void handle_command(const radar_command_t *cmd);
static size_t read_line(char *buffer, size_t length);
static void clear_frame_notifications(void);
static cy_rslt_t radar_start_acquisition(void);
static cy_rslt_t radar_stop_acquisition(void);
static bool process_raw_data(uint32_t frame_count);
static bool raw_stream_start(void);
static bool raw_stream_stop(void);
static void print_raw_frame_structured(uint32_t frame_number);

#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
static void radar_irq_handler(void *args, cyhal_gpio_event_t event);
#else
static void radar_irq_handler(void *args, cyhal_gpio_irq_event_t event);
#endif

int main(void)
{
    cy_rslt_t result = cybsp_init();

    if (result != CY_RSLT_SUCCESS)
    {
        printf("[ERR] BSP init failed: 0x%08" PRIX32 "\r\n", (uint32_t)result);
        CY_ASSERT(0);
    }

    __enable_irq();

    if (cy_retarget_io_init(CYBSP_DEBUG_UART_TX,
                            CYBSP_DEBUG_UART_RX,
                            CY_RETARGET_IO_BAUDRATE) != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    printf("\r\n========================================\r\n");
    printf("XENSIV 60GHz Radar Human Tracking Demo\r\n");
    printf("========================================\r\n");
    printf("Device: BGT60TR13C\r\n");
    printf("Detection Range: 0.3m to 5.0m\r\n");
    printf("Antenna Config:\r\n");
    printf("  - RX1 + RX3: X-axis (Azimuth)\r\n");
    printf("  - RX2 + RX3: Y-axis (Elevation)\r\n");
    printf("========================================\r\n\r\n");
    fflush(stdout);

    radar_cmd_queue = xQueueCreate(RADAR_CMD_QUEUE_LENGTH, sizeof(radar_command_t));
    if (radar_cmd_queue == NULL)
    {
        printf("[ERR] Failed to create command queue\r\n");
        CY_ASSERT(0);
    }

    if (xTaskCreate(radar_task,
                    "radar",
                    RADAR_TASK_STACK_SIZE,
                    NULL,
                    RADAR_TASK_PRIORITY,
                    &radar_task_handle) != pdPASS)
    {
        printf("[ERR] Failed to create radar task\r\n");
        CY_ASSERT(0);
    }

    if (xTaskCreate(command_task,
                    "cli",
                    COMMAND_TASK_STACK_SIZE,
                    NULL,
                    COMMAND_TASK_PRIORITY,
                    NULL) != pdPASS)
    {
        printf("[ERR] Failed to create command task\r\n");
        CY_ASSERT(0);
    }

    vTaskStartScheduler();
    CY_ASSERT(0);
    return 0;
}

static void radar_task(void *arg)
{
    (void)arg;

    radar_task_handle = xTaskGetCurrentTaskHandle();

    printf("Initializing radar...\r\n");

    if (init_sensor() != CY_RSLT_SUCCESS)
    {
        printf("[ERR] Sensor init failed\r\n");
        CY_ASSERT(0);
    }

    printf("Radar initialized successfully!\r\n");
    printf("System idle. Type 'run' to start tracking.\r\n\r\n");
    fflush(stdout);

    for (;;)
    {
        radar_command_t cmd;
        TickType_t wait_ticks = (radar_running || raw_streaming) ? 0U : portMAX_DELAY;

        if (xQueueReceive(radar_cmd_queue, &cmd, wait_ticks) == pdPASS)
        {
            handle_command(&cmd);
            continue;
        }

        if (!radar_running && !raw_streaming)
        {
            ulTaskNotifyTake(pdTRUE, 0U);
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        TickType_t notify_timeout = raw_streaming ? pdMS_TO_TICKS(1000U) : pdMS_TO_TICKS(100U);

        if (ulTaskNotifyTake(pdTRUE, notify_timeout) == 0U)
        {
            continue;
        }

        if (!acquire_frame())
        {
            printf("[WARN] Failed to read frame\r\n");
            continue;
        }

        if (raw_streaming)
        {
            uint32_t frame_number = ++raw_stream_frame_count;
            print_raw_frame_structured(frame_number);
            continue;
        }

        angle_range_result_t result;

        if (angle_range_compute(frame_buffer, &result))
        {
            angle_range_print(&result, true);
        }
    }
}

static cy_rslt_t init_sensor(void)
{
    if (cyhal_spi_init(&spi_obj,
                       PIN_XENSIV_BGT60TRXX_SPI_MOSI,
                       PIN_XENSIV_BGT60TRXX_SPI_MISO,
                       PIN_XENSIV_BGT60TRXX_SPI_SCLK,
                       NC,
                       NULL,
                       8,
                       CYHAL_SPI_MODE_00_MSB,
                       false) != CY_RSLT_SUCCESS)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI),
                        CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI),
                        CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK),
                        CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK),
                        CY_GPIO_DRIVE_1_8);

    if (cyhal_spi_set_frequency(&spi_obj, 25000000UL) != CY_RSLT_SUCCESS)
    {
        return CY_RSLT_TYPE_ERROR;
    }

#if defined (TARGET_APP_CYSBSYSKIT_DEV_01) || defined (TARGET_APP_KIT_BGT60TR13C_EMBEDD)
    if (cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_LDO_EN,
                        CYHAL_GPIO_DIR_OUTPUT,
                        CYHAL_GPIO_DRIVE_STRONG,
                        true) != CY_RSLT_SUCCESS)
    {
        return CY_RSLT_TYPE_ERROR;
    }
#endif

    cyhal_system_delay_ms(5U);

    cy_rslt_t rslt = xensiv_bgt60trxx_mtb_init(&radar_dev,
                                               &spi_obj,
                                               PIN_XENSIV_BGT60TRXX_SPI_CSN,
                                               PIN_XENSIV_BGT60TRXX_RSTN,
                                               register_list_macro_only,
                                               RADAR_REGISTER_COUNT);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    rslt = xensiv_bgt60trxx_set_fifo_limit(&radar_dev.dev, RAW_SAMPLES_PER_FRAME);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    rslt = xensiv_bgt60trxx_mtb_interrupt_init(&radar_dev,
                                               RAW_SAMPLES_PER_FRAME,
                                               PIN_XENSIV_BGT60TRXX_IRQ,
                                               GPIO_INTERRUPT_PRIORITY,
                                               radar_irq_handler,
                                               radar_task_handle);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    return CY_RSLT_SUCCESS;
}

static bool acquire_frame(void)
{
    if (xensiv_bgt60trxx_get_fifo_data(&radar_dev.dev,
                                       raw_frame,
                                       RAW_SAMPLES_PER_FRAME) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        return false;
    }

    for (uint32_t i = 0U; i < RAW_SAMPLES_PER_FRAME; ++i)
    {
        frame_buffer[i] = ((float32_t)raw_frame[i]) / 4096.0f;
    }

    return true;
}

static void handle_command(const radar_command_t *cmd)
{
    switch (cmd->type)
    {
        case RADAR_COMMAND_RUN:
        {
            if (radar_running)
            {
                printf("[CMD] Radar already running\r\n");
                break;
            }

            cy_rslt_t rslt = radar_start_acquisition();
            if (rslt == CY_RSLT_SUCCESS)
            {
                radar_running = true;
                printf("[CMD] Radar started\r\n");
            }
            else
            {
                printf("[ERR] Radar start failed: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);
            }
            break;
        }

        case RADAR_COMMAND_STOP:
        {
            if (!radar_running)
            {
                printf("[CMD] Radar already stopped\r\n");
                break;
            }

            cy_rslt_t rslt = radar_stop_acquisition();
            if (rslt == CY_RSLT_SUCCESS)
            {
                radar_running = false;
                printf("[CMD] Radar stopped\r\n");
            }
            else
            {
                printf("[ERR] Radar stop failed: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);
            }
            break;
        }

        case RADAR_COMMAND_RAW_DATA:
        {
            if (cmd->param == 0U)
            {
                printf("[ERR] raw_data requires frame count > 0\r\n");
                break;
            }

            (void)process_raw_data(cmd->param);
            break;
        }

        case RADAR_COMMAND_RAW_STREAM:
        {
            if (cmd->param != 0U)
            {
                (void)raw_stream_start();
            }
            else
            {
                (void)raw_stream_stop();
            }
            break;
        }

        default:
            break;
    }
}

static void clear_frame_notifications(void)
{
    while (ulTaskNotifyTake(pdTRUE, 0U) > 0U)
    {
        /* drain */
    }
}

static cy_rslt_t radar_start_acquisition(void)
{
    clear_frame_notifications();
    return xensiv_bgt60trxx_start_frame(&radar_dev.dev, true);
}

static cy_rslt_t radar_stop_acquisition(void)
{
    cy_rslt_t rslt = xensiv_bgt60trxx_start_frame(&radar_dev.dev, false);
    clear_frame_notifications();
    return rslt;
}

static bool process_raw_data(uint32_t frame_count)
{
    if (raw_streaming)
    {
        printf("[ERR] Raw stream active. Stop raw_stream before capturing frames.\r\n");
        return false;
    }

    bool resume = radar_running;
    cy_rslt_t rslt;

    if (resume)
    {
        rslt = radar_stop_acquisition();
        if (rslt != CY_RSLT_SUCCESS)
        {
            printf("[ERR] Failed to pause radar: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);
            radar_running = false;
            resume = false;
        }
        else
        {
            radar_running = false;
            printf("[CMD] Radar paused for raw capture\r\n");
        }
    }

    clear_frame_notifications();

    rslt = xensiv_bgt60trxx_start_frame(&radar_dev.dev, true);
    if (rslt != CY_RSLT_SUCCESS)
    {
        printf("[ERR] Raw capture start failed: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);
        if (resume)
        {
            rslt = radar_start_acquisition();
            if (rslt == CY_RSLT_SUCCESS)
            {
                radar_running = true;
                printf("[CMD] Radar resumed\r\n");
            }
            else
            {
                printf("[ERR] Radar resume failed: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);
            }
        }
        return false;
    }

    printf("[CMD] Collecting %lu frame(s) of raw data\r\n", (unsigned long)frame_count);

    bool success = true;

    for (uint32_t frame_idx = 0U; frame_idx < frame_count; ++frame_idx)
    {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000U)) == 0U)
        {
            printf("[ERR] Raw data capture timeout\r\n");
            success = false;
            break;
        }

        if (!acquire_frame())
        {
            printf("[ERR] Raw data capture failed\r\n");
            success = false;
            break;
        }

        printf("[RAW][%lu]", (unsigned long)frame_idx);
        for (uint32_t i = 0U; i < RAW_SAMPLES_PER_FRAME; ++i)
        {
            printf(" %.4f", frame_buffer[i]);
        }
        printf("\r\n");
    }

    rslt = xensiv_bgt60trxx_start_frame(&radar_dev.dev, false);
    if (rslt != CY_RSLT_SUCCESS)
    {
        printf("[ERR] Raw capture stop failed: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);
        success = false;
    }

    clear_frame_notifications();

    if (resume)
    {
        rslt = radar_start_acquisition();
        if (rslt == CY_RSLT_SUCCESS)
        {
            radar_running = true;
            printf("[CMD] Radar resumed\r\n");
        }
        else
        {
            printf("[ERR] Radar resume failed: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);
            radar_running = false;
            success = false;
        }
    }

    if (success)
    {
        printf("[CMD] Raw capture complete\r\n");
    }

    return success;
}

static bool raw_stream_start(void)
{
    if (raw_streaming)
    {
        printf("[CMD] Raw stream already active\r\n");
        return true;
    }

    raw_stream_resume_tracking = radar_running;

    if (radar_running)
    {
        cy_rslt_t rslt = radar_stop_acquisition();
        if (rslt != CY_RSLT_SUCCESS)
        {
            printf("[ERR] Failed to pause radar: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);
            radar_running = false;
            raw_stream_resume_tracking = false;
            return false;
        }

        radar_running = false;
        printf("[CMD] Radar paused for raw stream\r\n");
    }

    cy_rslt_t rslt = radar_start_acquisition();
    if (rslt != CY_RSLT_SUCCESS)
    {
        printf("[ERR] Raw stream start failed: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);

        if (raw_stream_resume_tracking)
        {
            rslt = radar_start_acquisition();
            if (rslt == CY_RSLT_SUCCESS)
            {
                radar_running = true;
                printf("[CMD] Radar resumed\r\n");
            }
            else
            {
                printf("[ERR] Radar resume failed: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);
                radar_running = false;
            }
        }

        raw_stream_resume_tracking = false;
        return false;
    }

    raw_stream_frame_count = 0U;
    raw_streaming = true;
    printf("[CMD] Raw stream started. Use 'raw_stream stop' to exit.\r\n");

    return true;
}

static bool raw_stream_stop(void)
{
    if (!raw_streaming)
    {
        printf("[CMD] Raw stream not active\r\n");
        return true;
    }

    cy_rslt_t rslt = radar_stop_acquisition();
    if (rslt != CY_RSLT_SUCCESS)
    {
        printf("[ERR] Raw stream stop failed: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);
        return false;
    }

    raw_streaming = false;
    raw_stream_frame_count = 0U;
    printf("[CMD] Raw stream stopped\r\n");

    clear_frame_notifications();

    if (raw_stream_resume_tracking)
    {
        rslt = radar_start_acquisition();
        if (rslt == CY_RSLT_SUCCESS)
        {
            radar_running = true;
            printf("[CMD] Radar resumed\r\n");
        }
        else
        {
            printf("[ERR] Radar resume failed: 0x%08" PRIX32 "\r\n", (uint32_t)rslt);
            radar_running = false;
            raw_stream_resume_tracking = false;
            return false;
        }
    }

    raw_stream_resume_tracking = false;
    return true;
}

static void print_raw_frame_structured(uint32_t frame_number)
{
    const uint32_t chirps_per_frame = XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME;
    const uint32_t samples_per_chirp = XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP;
    const uint32_t rx_antennas = XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS;

    printf("Frame %lu:\r\n", (unsigned long)frame_number);

    for (uint32_t chirp = 0U; chirp < chirps_per_frame; ++chirp)
    {
        printf("  Chirp %lu:\r\n", (unsigned long)(chirp + 1U));

        for (uint32_t rx = 0U; rx < rx_antennas; ++rx)
        {
            printf("    RX%lu:\r\n", (unsigned long)(rx + 1U));

            for (uint32_t sample = 0U; sample < samples_per_chirp; ++sample)
            {
                uint32_t base = (chirp * rx_antennas * samples_per_chirp * 2U) +
                                (rx * samples_per_chirp * 2U) +
                                (sample * 2U);
                int16_t i_val = (int16_t)raw_frame[base];
                int16_t q_val = (int16_t)raw_frame[base + 1U];

                printf("      %3lu: I=%6d Q=%6d\r\n",
                       (unsigned long)(sample + 1U),
                       (int)i_val,
                       (int)q_val);

                if ((sample & 0x0FU) == 0x0FU)
                {
                    vTaskDelay(pdMS_TO_TICKS(1U));
                }
            }

            vTaskDelay(pdMS_TO_TICKS(1U));
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    printf("\r\n");
    fflush(stdout);
}

static size_t read_line(char *buffer, size_t length)
{
    size_t idx = 0U;

    if ((buffer == NULL) || (length == 0U))
    {
        return 0U;
    }

    for (;;)
    {
        uint8_t ch = 0U;
        cy_rslt_t rslt = cyhal_uart_getc(&cy_retarget_io_uart_obj, &ch, 10U);

        if (rslt == CY_RSLT_ERR_CSP_UART_GETC_TIMEOUT)
        {
            vTaskDelay(pdMS_TO_TICKS(1U));
            continue;
        }

        if (rslt != CY_RSLT_SUCCESS)
        {
            vTaskDelay(pdMS_TO_TICKS(5U));
            continue;
        }

        if ((ch == '\r') || (ch == '\n'))
        {
            printf("\r\n");
            break;
        }

        if ((ch == '\b') || (ch == 0x7FU))
        {
            if (idx > 0U)
            {
                --idx;
                printf("\b \b");
            }
            continue;
        }

        if (idx < (length - 1U))
        {
            buffer[idx++] = (char)ch;
            putchar((int)ch);
        }
    }

    buffer[idx] = '\0';
    return idx;
}

static void command_task(void *arg)
{
    (void)arg;

    char line_buffer[64];

    printf("Available commands: run, stop, raw_data <frames>, raw_stream <start|stop>, help\r\n");
    fflush(stdout);

    for (;;)
    {
        printf("> ");
        fflush(stdout);

        if (read_line(line_buffer, sizeof(line_buffer)) == 0U)
        {
            continue;
        }

        char *token = strtok(line_buffer, " \t");
        if (token == NULL)
        {
            continue;
        }

        if (strcmp(token, "run") == 0)
        {
            radar_command_t cmd = { RADAR_COMMAND_RUN, 0U };
            if (xQueueSend(radar_cmd_queue, &cmd, pdMS_TO_TICKS(100U)) != pdPASS)
            {
                printf("[ERR] Failed to queue run command\r\n");
            }
        }
        else if (strcmp(token, "stop") == 0)
        {
            radar_command_t cmd = { RADAR_COMMAND_STOP, 0U };
            if (xQueueSend(radar_cmd_queue, &cmd, pdMS_TO_TICKS(100U)) != pdPASS)
            {
                printf("[ERR] Failed to queue stop command\r\n");
            }
        }
        else if (strcmp(token, "raw_data") == 0)
        {
            char *arg_frames = strtok(NULL, " \t");
            if (arg_frames == NULL)
            {
                printf("Usage: raw_data <frames>\r\n");
                continue;
            }

            char *endptr = NULL;
            unsigned long frames = strtoul(arg_frames, &endptr, 10);
            if ((endptr == arg_frames) || ((endptr != NULL) && (*endptr != '\0')))
            {
                printf("[ERR] Invalid frame count\r\n");
                continue;
            }

            if (frames == 0UL)
            {
                printf("[ERR] Frame count must be greater than zero\r\n");
                continue;
            }

            radar_command_t cmd = { RADAR_COMMAND_RAW_DATA, (uint32_t)frames };
            if (xQueueSend(radar_cmd_queue, &cmd, pdMS_TO_TICKS(100U)) != pdPASS)
            {
                printf("[ERR] Failed to queue raw_data command\r\n");
            }
        }
        else if (strcmp(token, "raw_stream") == 0)
        {
            char *arg_mode = strtok(NULL, " \t");
            if (arg_mode == NULL)
            {
                printf("Usage: raw_stream <start|stop>\r\n");
                continue;
            }

            radar_command_t cmd = { RADAR_COMMAND_RAW_STREAM, 0U };

            if (strcmp(arg_mode, "start") == 0)
            {
                cmd.param = 1U;
            }
            else if (strcmp(arg_mode, "stop") == 0)
            {
                cmd.param = 0U;
            }
            else
            {
                printf("[ERR] raw_stream expects 'start' or 'stop'\r\n");
                continue;
            }

            if (xQueueSend(radar_cmd_queue, &cmd, pdMS_TO_TICKS(100U)) != pdPASS)
            {
                printf("[ERR] Failed to queue raw_stream command\r\n");
            }
        }
        else if (strcmp(token, "help") == 0)
        {
            printf("Commands:\r\n");
            printf("  run                - start automatic tracking\r\n");
            printf("  stop               - halt automatic tracking\r\n");
            printf("  raw_data <frames>  - dump raw samples\r\n");
            printf("  raw_stream <start|stop> - continuously stream structured raw data\r\n");
        }
        else
        {
            printf("[ERR] Unknown command\r\n");
        }
    }
}

#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
static void radar_irq_handler(void *args, cyhal_gpio_event_t event)
#else
static void radar_irq_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    (void)event;

    TaskHandle_t task = (TaskHandle_t)args;
    BaseType_t higher_priority_task_woken = pdFALSE;

    vTaskNotifyGiveFromISR(task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
