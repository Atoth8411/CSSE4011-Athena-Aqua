#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

// Shared distance value (in cm)
extern volatile uint16_t ultrasonic_distance_cm;
extern const struct device *gpio_dev;

// Function to start the ultrasonic thread
void ultrasonic_init(void);
uint16_t measure_distance_cm(const struct device *gpio);

#endif // ULTRASONIC_H