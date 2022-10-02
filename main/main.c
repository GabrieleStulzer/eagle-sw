#include <stdio.h>

// Free RTOS Libraries
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Internal Drivers
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"

// ESP Utils
#include "esp_log.h"
#include "esp_err.h"

// External Libraries
#include "sdkconfig.h"


// Definitions

#define RUN_LED_GPIO CONFIG_GREEN_LED_GPIO
#define HIGH_V_LED_GPIO CONFIG_RED_LED_GPIO
#define LOW_V_LED_GPIO CONFIG_YELLOW_LED_GPIO
#define BUTTON_GPIO CONFIG_BUTTON_GPIO
#define VOLTAGE_ADC_CHANNEL CONFIG_VOLTAGE_ADC
#define SENSOR_ADC_CHANNEL CONFIG_SENSOR_ADC
#define MIN_V 1.8
#define MAX_V 2.7

#define VIN 3.3 // V power voltage
#define R 10000 //ohm resistance value

// Global Variables

// State Machine states definition
enum States {
    Idle,
    Start,
    Run,
    Warning
};

enum Operation {
    ReadVoltage,
    ReadSensor,
};

typedef struct {
    enum Operation operation;
} queue_element_t;

static const char *TAG = "EagleSW";

static enum States state = Start;
static int raw[2][10];
static int voltage[2][10];
static double p_voltage;

static int sensor_raw[2][10];
static int sensor_voltage[2][10];


static gptimer_handle_t sensor_timer = NULL;
static gptimer_handle_t voltage_timer = NULL;

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool do_calibration;
static queue_element_t ele;

static QueueHandle_t gpio_evt_queue = NULL;
static SemaphoreHandle_t button_semaphore = NULL;

static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);

static bool run_led_state = 0;
static bool min_v_led_state = 0;
static bool max_v_led_state = 0;
// UTILITY FUNCTIONS

void set_led(int gpio_n, int l_state, bool *g_state) {
    ESP_LOGI(TAG, "LED | GPIO: %d - G_STATE: %d - STATE: %d", gpio_n, *g_state, l_state);
    if(l_state != *g_state) {
        if(state != Idle) {
            printf("Led on GPIO: %d, Changed State: %d -> %d\n", gpio_n, *g_state, l_state);
        }
        *g_state = l_state;
        gpio_set_level(gpio_n, l_state);
    }
}

static void configure_led(int gpio_n)
{
    ESP_LOGI(TAG, "Configuration of led to blink on GPIO: %d", gpio_n);
    gpio_reset_pin(gpio_n);
    gpio_set_direction(gpio_n, GPIO_MODE_OUTPUT);
}

static void IRAM_ATTR button_handler(void* arg) {

    xSemaphoreGiveFromISR(button_semaphore, NULL);
}


static bool read_sensor_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;

    queue_element_t ele = {
        .operation = ReadSensor,
    };

    xQueueSendFromISR(queue, &ele, &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

static bool read_voltage_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;

    queue_element_t ele = {
        .operation = ReadVoltage,
    };

    xQueueSendFromISR(queue, &ele, &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

static void read_adc(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t cali_handle, adc_channel_t channel, bool calib, int *raw, int *voltage) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, raw));
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, channel, *raw);
        if (calib) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, *raw, voltage));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Voltage: %d mV", ADC_UNIT_1 + 1, channel, (*voltage));
        }

};

static void configure_timer(gptimer_handle_t *timer, gptimer_config_t *timer_config, gptimer_alarm_config_t *alarm_config, uint64_t count, gptimer_event_callbacks_t *cb, queue_element_t *el) {
    ESP_ERROR_CHECK(gptimer_new_timer(timer_config, timer));

    ESP_ERROR_CHECK(gptimer_set_alarm_action(*timer, alarm_config));

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(*timer, cb, gpio_evt_queue));
    ESP_ERROR_CHECK(gptimer_enable(*timer));
}

// TODO: Add Button Debounce
void button_task(void *arg) {
    for(;;) {
        if(xSemaphoreTake(button_semaphore, portMAX_DELAY) == pdTRUE) {
            ESP_LOGE(TAG, "--- Emergency Button Pressed");
            if(state != Idle) {
                ESP_LOGI(TAG, "Enter Idle State");
                ESP_ERROR_CHECK(gptimer_stop(sensor_timer));
                ESP_ERROR_CHECK(gptimer_stop(voltage_timer));
                state = Idle;
            } else {
                if(state == Idle) {
                    state = Start;
                }
            }
        }


    }
}

void gpio_adc_task(void *arg) {
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &ele, pdMS_TO_TICKS(2000))){
            if (ele.operation == ReadVoltage)
            {
                ESP_LOGI(TAG, "--- Reading Voltage");
                read_adc(adc1_handle, adc1_cali_handle, VOLTAGE_ADC_CHANNEL, do_calibration, &raw[0][0], &voltage[0][0]);
                p_voltage = (double)voltage[0][0] / 1000;

                printf("Time: %lld | SysVoltage: %f V\n", esp_timer_get_time(), p_voltage);

                if(p_voltage < MIN_V || p_voltage > MAX_V) {
                    ESP_ERROR_CHECK(gptimer_stop(sensor_timer));
                    state = Warning;
                }
            }
            if(ele.operation == ReadSensor) {
                // Assume R=20 Lux = 10 | R=50 Lux=100

                ESP_LOGI(TAG,"Reading Sensor");
                read_adc(adc1_handle, adc1_cali_handle, SENSOR_ADC_CHANNEL, do_calibration, &sensor_raw[0][0], &sensor_voltage[0][0]);

                p_voltage = (double)sensor_voltage[0][0] / 1000;

                if(p_voltage > 0) {
                    float RLDR = ((3.3 * 10000.0)/p_voltage) - 10000.0;
                    float RLDRkOhm = RLDR / 1000.0;
                    ESP_LOGW(TAG, "%.6f", RLDRkOhm);
                    if(RLDR < 20000) {
                        printf("SENSOR ERROR: Too Bright");
                    } else if (RLDR > 50000) {
                        printf("SENSOR ERROR: Too Dark");
                    } else {
                        double slope = 1.0 * (100 - 10) / (50 - 20);
                        double lux = 10 + slope * (RLDR / 1000.0 - 20);
                        printf("Time: %lld | Sensor Voltage: %f V - Sensor Value: %.3f\n", esp_timer_get_time(), p_voltage, lux);
                    }
                } else {
                    printf("Error Reading Sensor");
                }

            }
        }

    }

}

static void deinit_timer(gptimer_handle_t timer) {
    ESP_ERROR_CHECK(gptimer_stop(timer));
    ESP_ERROR_CHECK(gptimer_disable(timer));
    ESP_ERROR_CHECK(gptimer_del_timer(timer));
}


void app_main(void)
{
    configure_led(RUN_LED_GPIO);
    configure_led(LOW_V_LED_GPIO);
    configure_led(HIGH_V_LED_GPIO);

    // Configure ISR-Taks communication protocols

    button_semaphore = xSemaphoreCreateBinary();
    gpio_evt_queue = xQueueCreate(10, sizeof(queue_element_t));
    if (!gpio_evt_queue) {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }


    /*---------------------------------------------------------------
            ADC Configuration
    ---------------------------------------------------------------*/

    // Init ADC1
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Configure ADC1
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VOLTAGE_ADC_CHANNEL, &config));

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SENSOR_ADC_CHANNEL, &config));

    // ADC1 Calibration
    do_calibration = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);

    /*---------------------------------------------------------------
            Input GPIO Configuration
    ---------------------------------------------------------------*/


    gpio_config_t input_config = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL<<BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
    };

    gpio_config(&input_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_handler, NULL);


    /*---------------------------------------------------------------
            Timer Configuration
    ---------------------------------------------------------------*/
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };


    gptimer_event_callbacks_t st_cb = {
        .on_alarm = read_sensor_on_alarm_cb, // register user callback
    };
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0, // counter will reload with 0 on alarm event
        .alarm_count = 200000, // period = 1s @resolution 1MHz
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };

    configure_timer(&sensor_timer, &timer_config, &alarm_config, 1000000, &st_cb, &ele);
    

    gptimer_event_callbacks_t vt_cb = {
        .on_alarm = read_voltage_on_alarm_cb, // register user callback
    };
    gptimer_alarm_config_t vt_alarm_config = {
        .reload_count = 0, // counter will reload with 0 on alarm event
        .alarm_count = 350000, // period = 1s @resolution 1MHz
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };

    configure_timer(&voltage_timer, &timer_config, &vt_alarm_config, 2000000, &vt_cb, &ele);


    // Create Tasks

    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(gpio_adc_task, "gpio_adc_task", 2048, NULL, 10, NULL);


    while (1)
    {
        // State Machine
        switch (state)
        {
        case Idle:
            set_led(RUN_LED_GPIO, 0, &run_led_state);
            ESP_LOGD(TAG, "Idle State");
            printf("Time: %lld | Board in waiting state - please press the emergency button\n", esp_timer_get_time());
            set_led(LOW_V_LED_GPIO, 1, &min_v_led_state);
            set_led(HIGH_V_LED_GPIO, 0, &max_v_led_state);
            vTaskDelay(250 / portTICK_PERIOD_MS);
            set_led(LOW_V_LED_GPIO, 0, &min_v_led_state);
            set_led(HIGH_V_LED_GPIO, 1, &max_v_led_state);
            vTaskDelay(250 / portTICK_PERIOD_MS);
            break;

        case Warning:
            ESP_LOGD(TAG, "Warning State");
            set_led(RUN_LED_GPIO, 0, &run_led_state);

            // Stop sensor timer
            ESP_ERROR_CHECK(gptimer_stop(sensor_timer));

            if(p_voltage < MIN_V) {
                printf("VOLTAGE WARNING: Voltage to low\n");
                set_led(LOW_V_LED_GPIO, 1, &min_v_led_state);
                set_led(HIGH_V_LED_GPIO, 0, &max_v_led_state);
            } else if(p_voltage > MAX_V) {
                printf("VOLTAGE WARNING: Voltage to high\n");
                set_led(LOW_V_LED_GPIO, 0, &min_v_led_state);
                set_led(HIGH_V_LED_GPIO, 1, &max_v_led_state);
            }

            if (p_voltage > MIN_V && p_voltage < MAX_V) {
                set_led(HIGH_V_LED_GPIO, 0, &max_v_led_state);
                set_led(LOW_V_LED_GPIO, 0, &min_v_led_state);
                state = Start;
            }

            break;

        case Start:
            // Start timer for sensor reading
            ESP_LOGD(TAG,"Starting Processes");

            set_led(LOW_V_LED_GPIO, 0, &min_v_led_state);
            set_led(HIGH_V_LED_GPIO, 0, &max_v_led_state);
            set_led(RUN_LED_GPIO, 1, &run_led_state);

            ESP_ERROR_CHECK(gptimer_start(sensor_timer));
            ESP_ERROR_CHECK(gptimer_start(voltage_timer));

            state = Run;
            break;
        
        case Run:
            // Run activity executed on subtasks
            break;
        
        default:
            break;
        }

        vTaskDelay(10);
    }

    // Tear Down

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration) {
        adc_calibration_deinit(adc1_cali_handle);
    }

    deinit_timer(sensor_timer);
    deinit_timer(voltage_timer);
}


/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
