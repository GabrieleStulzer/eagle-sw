#include <stdio.h>

enum States {
    Idle,
    Start,
    Run
};

void toggleLed(int gpio_n) {
    printf("Toggling gpio N:", gpio_n);
}

void app_main(void)
{
    static enum States state = Idle;
    int8_t voltage = 0;
    // State Machine application
    switch (state)
    {
    case Idle:
        printf("Idle State");
        // Stop all timers and processes, check voltage and start led
        if (voltage < 1.3) {
            // GPIO red led on
            toggleLed(3);
        } else if (voltage > 3.3) {
            // GPIO red gree on
            toggleLed(4);
        }
        break;

    case Start:
        // Start timer for sensor reading

        state = Run;
        break;
    
    case Run:
        // Print Sensor data
        break;
    
    default:
        break;
    }
}
