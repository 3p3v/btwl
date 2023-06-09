#include "Sim800lESP.hpp"
// #include "Sim800lBB.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include "esp_intr_alloc.h"
#include "soc/soc.h"
#include "hal/uart_ll.h"
#include "esp_log.h"

static char TAG[] = "SIM800l";

int tempUart = 0;

// TODO delete config from constructor
Sim800lESP::Sim800lESP(const char *url)
    : Sim800lBB(url), tx(SIM800L_DEF_UART_TX), rx(SIM800L_DEF_UART_RX), 
    baudRate(SIM800L_DEF_UART_BAUD_RATE), drt(SIM800L_DEF_DRT_PIN),
    rst(SIM800L_DEF_RST_PIN), uart(SIM800L_DEF_UART_NUM) {}

Sim800lESP::Sim800lESP(gpio_num_t tx, gpio_num_t rx, uart_port_t uart, gpio_num_t drt, gpio_num_t rst, const int baudRate, const char *url)
    : Sim800lBB(url), tx(tx), rx(rx), baudRate(baudRate),
    drt(drt), rst(rst), uart(uart) {}

Sim800lESP::~Sim800lESP() {}

/*  UART configuration, includes ESP pattern detection (detects every new line) */
Sim800lError Sim800lESP::init()
{
    /* UART */
    uart_config_t uartConf = {
	    .baud_rate = baudRate,
	    .data_bits = UART_DATA_8_BITS,
	    .parity    = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
	};

    /* Basic UART config */
    ESP_ERROR_CHECK(uart_driver_install(uart, SIM800L_DEF_BUF_SIZE, SIM800L_DEF_BUF_SIZE, SIM800L_DEF_QUEUE_SIZE, &uartQueue, 0));
    ESP_ERROR_CHECK(uart_param_config(uart, &uartConf));
    ESP_ERROR_CHECK(uart_set_pin(uart, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* Pattern detection event config */
    // uart_enable_pattern_det_baud_intr(uart, '\n', 1, SIM800L_PATTERN_INTERVAL, SIM800L_MIN_POST_IDLE, SIM800L_MIN_PRE_IDLE);
    // uart_pattern_queue_reset(uart, SIM800L_DEF_PATTERN_QUEUE_SIZE);

    /* GPIO */
    gpio_config_t gpioConf = {
        .pin_bit_mask = (uint64_t)((1 << drt) | (1 << rst)),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    /* Set DRT and RST pin */
    ESP_ERROR_CHECK(gpio_config(&gpioConf));

    resetForce();

    setStatus(Sim800lUARTInitialised);

    return Sim800lOk;
}

Sim800lError Sim800lESP::deinit()
{
    resetStatus(Sim800lUARTInitialised);
    if(uart_driver_delete(uart) == ESP_OK)
        return Sim800lOk;
    else
        return Sim800lHardwareErr;
}

Sim800lError Sim800lESP::reinit()
{
    if(deinit() == Sim800lOk)
        return init();
    else    
        return Sim800lErr;
}

/* Send data to SIM800l */
Sim800lError Sim800lESP::sendData(const char *data)
{
    /* Get rid of any data left */
    uart_flush_input(uart);
    xQueueReset(uartQueue);

    /* Send data */
    int len = uart_write_bytes(uart, data, strlen(data));

    if (len > 0)
        return Sim800lOk;
    else if (len == 0)
        return Sim800lErr;
    else 
        return Sim800lHardwareErr;
}

/*  Receive data from SIM800l using ESP interrupts.
 *   Detects every new line or timeout.
 */
#define PATTERN_CHR_NUM 1

int Sim800lESP::receiveData()
{
    uart_event_t event;
    size_t buffered_size;
    int startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while(((xTaskGetTickCount() * portTICK_PERIOD_MS) - startTime) < SIM800L_DEF_DOWNLOAD_TIME) {
        if (xQueueReceive(uartQueue, (void *)&event, SIM800L_DEF_DOWNLOAD_TIME / portTICK_PERIOD_MS))
        {
            bzero(receivedData, SIM800L_DEF_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", uart);
            switch (event.type)
            {
            // Event of UART receving data
            case UART_DATA: {
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                int len = uart_read_bytes(uart, receivedData, event.size, portMAX_DELAY);
                printf("%s", receivedData);
                if (len > 0)
                    return len;
                else if (len == 0) 
                    return Sim800lErr;
                else 
                    return Sim800lHardwareErr;
                break;
            }
            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                uart_flush_input(uart);
                xQueueReset(uartQueue);
                break;
            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                uart_flush_input(uart);
                xQueueReset(uartQueue);
                //TODO resize buffer
                return Sim800lBufferFullErr;
                break;
            // Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            // Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            // Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            case UART_DATA_BREAK:
                ESP_LOGI(TAG, "uart data break");
                break;
            case UART_EVENT_MAX:
                ESP_LOGI(TAG, "uart event max");
                break;
            // UART_PATTERN_DET
            case UART_PATTERN_DET:
            {
                // uart_get_buffered_data_len(uart, &buffered_size);
                // int pos = uart_pattern_pop_pos(uart);
                // ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                // if (pos == -1)
                // {
                //     uart_flush_input(uart);
                // }
                // else
                // {
                //     uart_read_bytes(uart, receivedData, pos, 100 / portTICK_PERIOD_MS);
                //     uint8_t pat[PATTERN_CHR_NUM + 1];
                //     memset(pat, 0, sizeof(pat));
                //     uart_read_bytes(uart, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                //     ESP_LOGI(TAG, "read data: %s", receivedData);
                //     ESP_LOGI(TAG, "read pat : %s", pat);
                // }
                break;
            }
            // Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                // break;
            } 
        } else {
            ESP_LOGI(TAG, "timeout");
            return (int)Sim800lTimeoutErr; 
        }
    }

    return (int)Sim800lErr; 
    return receivedLen;
}

void Sim800lESP::simDelay(int ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

Sim800lError Sim800lESP::setDRT(Sim800lPin set) {
    if(gpio_set_level(drt, set) == ESP_OK)
        return Sim800lOk;
    else
        return Sim800lHardwareErr;
}

Sim800lError Sim800lESP::setRST(Sim800lPin set) {
    if(gpio_set_level(rst, set) == ESP_OK)
        return Sim800lOk;
    else
        return Sim800lHardwareErr;
}
