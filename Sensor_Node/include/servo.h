//inclide libaries here
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#ifndef SERVO_H
#define SERVO_H

#pragma once

typedef enum {
    MODE_SWEEP,
    MODE_TRACK
} servo_mode_t;

typedef struct servoState{
    servo_mode_t mode;
    int angle;
    int8_t step;
    int64_t timestamp;
    bool angle_pending;
} servo_info_t;

void servo_init(void);
void servo_start_thread(void);
void servo_set_target_angle_pan(int angle);

extern struct k_mutex PanAngleAccess;
extern volatile servo_info_t servo_info;

#endif // SERVO_H