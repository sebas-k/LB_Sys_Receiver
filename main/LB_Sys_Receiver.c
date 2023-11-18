#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define Red_LED GPIO_NUM_46 //Red LED, active=>0
#define Green_LED GPIO_NUM_0 //Green LED, active=>0
#define Blue_LED GPIO_NUM_45 //Blue LED, active=>0
#define Yellow_LED GPIO_NUM_48 //Yellow LED, active=>1
#define LED_OUTPUT_PIN_SEL  ((1ULL<<Green_LED) | (1ULL<<Blue_LED) | (1ULL<<Red_LED) | (1ULL<<Yellow_LED))

#define TSOP_1 GPIO_NUM_21 //D10 Arduino Layout
#define GPIO_INTERRUPT_PIN_SEL  ((1ULL<<TSOP_1))

#define ESP_INTR_FLAG_DEFAULT 0

static const char *TAG = "example";
volatile bool reset_time_frame = false;
volatile bool light_barrier_active = false;

uint64_t ms = 0;

typedef struct {
    uint64_t ms_count;
} queue_timer_1ms_t;

static void IRAM_ATTR tsop_isr_handler(void* arg)
{
   reset_time_frame = true;
   light_barrier_active = true;
}

static bool IRAM_ATTR timer_1ms_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;

    ms++;

    if (light_barrier_active == true && reset_time_frame == false){
        light_barrier_active = false;

        QueueHandle_t queue = (QueueHandle_t)user_data;
        queue_timer_1ms_t ele = {
        .ms_count = ms
        };
        xQueueSendFromISR(queue, &ele, &high_task_awoken);
    }

    //insert else/if for the High Task Awoke

    reset_time_frame = false;

    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}


void app_main(void)
{
    //LED initializiation 
    gpio_config_t led_io_conf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = LED_OUTPUT_PIN_SEL,
    .pull_down_en = 0,
    .pull_up_en = 0
    };
    gpio_config(&led_io_conf);

    gpio_set_level (Red_LED, 1);
    gpio_set_level (Green_LED, 1);
    gpio_set_level (Blue_LED, 1);
    gpio_set_level (Yellow_LED, 1);

    //Interrupt initializiation
    gpio_config_t inter_io_conf = {
    .intr_type = GPIO_INTR_NEGEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = GPIO_INTERRUPT_PIN_SEL,
    .pull_down_en = 0,
    .pull_up_en = 1
    };
    gpio_config(&inter_io_conf);

    //install tsop isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for TSOPs gpio pin
    gpio_isr_handler_add(TSOP_1, tsop_isr_handler, (void*) TSOP_1);

    //Create queue for Time Measurments
    queue_timer_1ms_t ele;
    QueueHandle_t queue_timer_1ms = xQueueCreate(10, sizeof(queue_timer_1ms_t));
    if (!queue_timer_1ms) {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }

    // 1) Base Configuration of the timer periphial:
    // 1.1) Create timer handle
    ESP_LOGI(TAG, "Create timer handle");
    gptimer_handle_t gptimer_1ms = NULL;
    
    // 1.2) Config Timer 
    gptimer_config_t timer_config = {
        //.clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .clk_src = GPTIMER_CLK_SRC_XTAL,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };

    // 1.3) Create Timer 
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_1ms));

    // 2) Configuration of the Events 
    // 2.1) Set callback event kind and ISR (Interupt Service Routine)
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_1ms_on_alarm,
    };
    // 2.2) Register the callback
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_1ms, &cbs, queue_timer_1ms));

    // 3) Enable Timer
    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer_1ms));

    // 4) Config the Alarm and start
    // 4.1) Config Alarm
    ESP_LOGI(TAG, "Start timer, alarm and reload");
    gptimer_alarm_config_t alarm_config_1ms = {
        .reload_count = 0,
        .alarm_count = 1000, // period = 1ms
        .flags.auto_reload_on_alarm = true,
    };
    // 4.2) Start and Enable finally
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_1ms, &alarm_config_1ms));
    ESP_ERROR_CHECK(gptimer_start(gptimer_1ms));

    while (1) {
    //for ( int cnt = 1; cnt <= 10; cnt++) {
        if (xQueueReceive(queue_timer_1ms, &ele, portMAX_DELAY)) {
            ESP_LOGI(TAG, "MS since started the Timer %llu", ele.ms_count);
        }   
    }
}
