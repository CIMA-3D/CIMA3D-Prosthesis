#include "uart_command_handler.h"

#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_NUM UART_NUM_0
#define UART_BUFFER_SIZE 128
#define CMD_BUFFER_SIZE 256

static uint8_t uart_data[UART_BUFFER_SIZE];
static char cmd_buffer[CMD_BUFFER_SIZE];
static int cmd_pos = 0;

void beginUart(uint32_t baudrate) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = baudrate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
}

void pollUart(void (*parser)(const char* cmd)) {
    size_t len = 0;
    if (uart_get_buffered_data_len(UART_NUM, &len) != ESP_OK || len == 0) return;

    len = uart_read_bytes(UART_NUM, uart_data, len, pdMS_TO_TICKS(10));

    for (int i = 0; i < len; ++i) {
        char c = uart_data[i];
        if (c == '\n' || c == '\r') {
            if (cmd_pos == 0) continue;
            cmd_buffer[cmd_pos] = '\0';
            parser(cmd_buffer);  // call your parser in main.cpp
            cmd_pos = 0;
        } else if (cmd_pos < CMD_BUFFER_SIZE - 1) {
            cmd_buffer[cmd_pos++] = c;
        }
    }
}
