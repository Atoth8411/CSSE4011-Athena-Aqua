#include "ultrasonic.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define TRIG_PIN 3 // P0.03
#define ECHO_PIN 4 // P0.04

#define STACK_SIZE 1024
#define PRIORITY 1

K_THREAD_STACK_DEFINE(ultrasonic_stack, STACK_SIZE);
static struct k_thread ultrasonic_thread_data;

const struct device *gpio_dev;
//volatile uint16_t ultrasonic_distance_cm = 0;

void ultrasonic_init(void) {
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        printk("GPIO0 not ready\n");
        return;
    }

    gpio_pin_configure(gpio_dev, TRIG_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio_dev, ECHO_PIN, GPIO_INPUT);
    printk("Ultrasonic GPIOs configured\n");
}

uint16_t measure_distance_cm(const struct device *gpio) {
    uint32_t timeout_us = 30000;
    uint32_t start, end, duration_us;

    gpio_pin_set(gpio, TRIG_PIN, 1);
    k_busy_wait(10);
    gpio_pin_set(gpio, TRIG_PIN, 0);

    uint32_t wait_start = k_cycle_get_32();
    while (gpio_pin_get(gpio, ECHO_PIN) == 0) {
        if (k_cyc_to_us_floor32(k_cycle_get_32() - wait_start) > timeout_us) {
            return 0;
        }
    }

    start = k_cycle_get_32();
    while (gpio_pin_get(gpio, ECHO_PIN) == 1) {
        if (k_cyc_to_us_floor32(k_cycle_get_32() - start) > timeout_us) {
            return 0;
        }
    }

    end = k_cycle_get_32();
    duration_us = k_cyc_to_us_floor32(end - start);

    return (uint16_t)((duration_us * 343) / 20000); // (us × speed) / 2
}

//combine this thread with servo thread 
static void ultrasonic_thread_fn(void *a, void *b, void *c) {

    ultrasonic_init();

    while (1) {
        //ultrasonic_distance_cm = measure_distance_cm(gpio_dev);
        //notify_distance(ultrasonic_distance_cm); //should only track if the tracking is enabled, otherwise just don't
        k_sleep(K_MSEC(500));
    }
}

void ultrasonic_thread_init(void) {
    k_thread_create(&ultrasonic_thread_data, ultrasonic_stack,
                    K_THREAD_STACK_SIZEOF(ultrasonic_stack),
                    ultrasonic_thread_fn, NULL, NULL, NULL, PRIORITY, 0,
                    K_NO_WAIT);
}