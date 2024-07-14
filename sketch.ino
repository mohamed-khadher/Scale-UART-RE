#include <Arduino.h>
#include "driver/uart.h"

#define UART_NUM UART_NUM_2
#define RX_PIN 16 
#define BUF_SIZE 512
#define BAUD_RATE 125000

static QueueHandle_t uart_queue;
static uint8_t circularBuffer[BUF_SIZE];
static uint64_t timeBuffer[BUF_SIZE];
static int bufferHead = 0;
static int bufferTail = 0;

void uartEventTask(void *pvParameters) {
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    int len;
    
    for(;;) {
        if(xQueueReceive(uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch(event.type) {
                case UART_DATA:
                    uart_get_buffered_data_len(UART_NUM, &buffered_size);
                    len = uart_read_bytes(UART_NUM, dtmp, buffered_size, 100 / portTICK_RATE_MS);
                    for(int i = 0; i < len; i++) {
                        circularBuffer[bufferHead] = dtmp[i];
                        timeBuffer[bufferHead] = esp_timer_get_time();+
                        bufferHead = (bufferHead + 1) % BUF_SIZE;
                        if(bufferHead == bufferTail) {
                            bufferTail = (bufferTail + 1) % BUF_SIZE;
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(100); }
    Serial.println("ESP32 UART Sniffer Ready");

    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 20, &uart_queue, 0);

    xTaskCreatePinnedToCore(uartEventTask, "uart_event_task", 2048, NULL, 12, NULL, 0);
}

void loop() {
    static uint32_t byteCount = 0;
    static uint64_t lastByteTime = 0;

    while(bufferHead != bufferTail) {
        uint8_t receivedByte = circularBuffer[bufferTail];
        uint64_t byteTime = timeBuffer[bufferTail];
        bufferTail = (bufferTail + 1) % BUF_SIZE;
        byteCount++;

        Serial.printf("Byte %u: 0x%02X at %llu us", byteCount, receivedByte, byteTime);
        if (lastByteTime > 0) {
            Serial.printf(" (+" PRIu64 " us)", byteTime - lastByteTime);
        }
        Serial.println();

        lastByteTime = byteTime;
    }

    delay(10);  
}
