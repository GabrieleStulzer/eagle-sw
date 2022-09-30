#include <stdio.h>

// Free RTOS Libraries
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Internal Drivers
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ESP Utils
#include "esp_log.h"
#include "esp_err.h"

// External Libraries
#include "sdkconfig.h"


// Definitions

#define GREEN_LED_GPIO CONFIG_GREEN_LED_GPIO
#define RED_LED_GPIO CONFIG_RED_LED_GPIO

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
    ReadSensor
};

static const char *TAG = "EagleSW";

static enum States state = Start;
static int8_t voltage = 2.0;

static int adc_raw[2][10];
static int adc_voltage[2][10];

static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);

typedef struct {
    enum Operation operation;
} queue_element_t;

// Green Led State
// static uint8_t g_led_state = 0;

// Red Led State
// static uint8_t r_led_state = 0;

// UTILITY FUNCTIONS

void set_led(int gpio_n, int state) {
    gpio_set_level(gpio_n, state);
}

static void configure_led(int gpio_n)
{
    ESP_LOGI(TAG, "Configuration of led to blink on GPIO: %d", gpio_n);
    gpio_reset_pin(gpio_n);
    gpio_set_direction(gpio_n, GPIO_MODE_OUTPUT);
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

static void configure_timer(gptimer_handle_t *timer, gptimer_config_t *timer_config, gptimer_alarm_config_t *alarm_config, uint64_t count, gptimer_event_callbacks_t *cb, QueueHandle_t queue, queue_element_t *el) {
    ESP_ERROR_CHECK(gptimer_new_timer(timer_config, timer));

    ESP_ERROR_CHECK(gptimer_set_alarm_action(*timer, alarm_config));

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(*timer, cb, queue));
    ESP_ERROR_CHECK(gptimer_enable(*timer));

}


void app_main(void)
{
    printf("Init process... \n");

    configure_led(GREEN_LED_GPIO);
    configure_led(RED_LED_GPIO);

    /*---------------------------------------------------------------
            ADC Configuration
    ---------------------------------------------------------------*/

    // Init ADC1
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Configure ADC1
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };

    // Config Channel 6 - GPIO 34 for photoresistor reading
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));

    // Config Channel 7 - GPIO 35 for input voltage reading
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config));

    // ADC1 Calibration
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration1 = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);

    /*---------------------------------------------------------------
            Timer Configuration
    ---------------------------------------------------------------*/
    queue_element_t ele;
    QueueHandle_t queue = xQueueCreate(10, sizeof(queue_element_t));
    if (!queue) {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };


    gptimer_handle_t sensor_timer = NULL;
    gptimer_event_callbacks_t st_cb = {
        .on_alarm = read_sensor_on_alarm_cb, // register user callback
    };
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0, // counter will reload with 0 on alarm event
        .alarm_count = 1000000, // period = 1s @resolution 1MHz
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };

    configure_timer(&sensor_timer, &timer_config, &alarm_config, 1000000, &st_cb, queue, &ele);
    

    gptimer_handle_t voltage_timer = NULL;
    gptimer_event_callbacks_t vt_cb = {
        .on_alarm = read_voltage_on_alarm_cb, // register user callback
    };
    gptimer_alarm_config_t vt_alarm_config = {
        .reload_count = 0, // counter will reload with 0 on alarm event
        .alarm_count = 2000000, // period = 1s @resolution 1MHz
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };

    configure_timer(&voltage_timer, &timer_config, &vt_alarm_config, 2000000, &vt_cb, queue, &ele);


    while (1)
    {
        // State Machine
        switch (state)
        {
        case Idle:
            printf("Idle State\n\n");
            set_led(GREEN_LED_GPIO, 0);
            set_led(RED_LED_GPIO, 1);
            // Stop all timers and processes, check voltage and start led
            if (voltage < 1.3) {
                // GPIO red led on
                set_led(RED_LED_GPIO, 1);
            } else if (voltage > 3.3) {
                // GPIO red gree on
                set_led(RED_LED_GPIO, 1);
            } else {
                set_led(RED_LED_GPIO, 0);
                set_led(GREEN_LED_GPIO, 1);
                state = Start;
            }

            break;

        case Start:
            // Start timer for sensor reading
            printf("Starting Processes\n\n");

            set_led(GREEN_LED_GPIO, 1);

            ESP_ERROR_CHECK(gptimer_start(sensor_timer));
            ESP_ERROR_CHECK(gptimer_start(voltage_timer));

            state = Run;
            break;
        
        case Run:
            // Print Sensor data
            // printf("Running Process\n\n");

            if(xQueueReceive(queue, &ele, pdMS_TO_TICKS(2000))){
                if(ele.operation == ReadSensor) {
                    ESP_LOGI(TAG,"Reading Sensor");
                    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_raw[0][0]));
                    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_6, adc_raw[0][0]);
                    if (do_calibration1) {
                        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &adc_voltage[0][0]));
                        ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC_CHANNEL_6, adc_voltage[0][0]);
                    }
                }
                if (ele.operation == ReadVoltage)
                {
                    ESP_LOGI(TAG, "--- Reading Voltage");
                }
            }
            break;
        
        default:
            break;
        }
    }

    // Tear Down

    // TODO: Deinit timers

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1) {
        adc_calibration_deinit(adc1_cali_handle);
    }
    
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
