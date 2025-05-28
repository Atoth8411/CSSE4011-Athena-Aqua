#include "servo.h"
#include "ultrasonic.h"
#include "my_gatt.h"

#define PWM_PERIOD_USEC 20000 // 20ms = 50Hz
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_DEFAULT_ANGLE 90
#define TILT_BOOT_ANGLE 45 // Set once at boot

#define PAN_CHANNEL 0  // P0.13 (PWM_OUT0)
#define TILT_CHANNEL 1 // P0.11 (PWM_OUT1)

#define TIMEOUT_MS 2000
#define SWEEP_ANGLE_UPDATE_RESOLUTION 5

//cannot track under current setup
#define HEURISTIC_DISTANCE_THRESHOLD 100 //(cm)
#define HEURISTIC_MAX_DISTANCE_DIFFERENCE 20 //(cm)

static const struct device *pwm_dev;
static volatile int target_angle_pan = SERVO_DEFAULT_ANGLE;

struct k_mutex PanAngleAccess;
volatile servo_info_t servo_info;

// struct k_mutex PowerStateAccess;
// volatile uint8_t turret_state = 1;

/* === Angle to pulse width === */
static uint32_t angle_to_pulse(int degrees) {
    if (degrees < 0)
        degrees = 0;
    if (degrees > 180)
        degrees = 180;
    return SERVO_MIN_US + ((degrees * (SERVO_MAX_US - SERVO_MIN_US)) / 180);
}

/* === Set pan angle externally (from BLE) === */
void servo_set_target_angle_pan(int angle) {
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;
    target_angle_pan = angle;
    printk("[SERVO] Pan target set to: %d\n", angle);
}

/* === Move a PWM channel to angle === */
static void move_servo_channel(int channel, int angle) {
    uint32_t pulse = angle_to_pulse(angle);
    int err = pwm_set(pwm_dev, channel, PWM_USEC(PWM_PERIOD_USEC),
                      PWM_USEC(pulse), PWM_POLARITY_NORMAL);
    if (err) {
        printk("[SERVO] PWM CH%d set failed: %d\n", channel, err);
    } else {
        //printk("[SERVO] CH%d → angle=%d → pulse=%dus\n", channel, angle, pulse);
    }
}

static void start_sweep(servo_info_t* info) {
    int difference = SERVO_DEFAULT_ANGLE - info->angle;
    if(difference < 0) {
        info->step = -1;
    } else {
        info->step = 1;
    }
}

static void update_sweep(servo_info_t* info) {

    if(info->angle == 180 || info->angle == 0) {
        info->step = -1 * info->step;
    }

    info->angle += info->step;
    move_servo_channel(PAN_CHANNEL, info->angle);

}

/* === Init: set tilt once and center pan === */
void servo_init(void) {
    pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm0));
    if (!device_is_ready(pwm_dev)) {
        printk("[SERVO] PWM0 not ready!\n");
        return;
    }

    move_servo_channel(PAN_CHANNEL, SERVO_DEFAULT_ANGLE);
    move_servo_channel(TILT_CHANNEL, TILT_BOOT_ANGLE); // Set once

    printk("[SERVO] Initialized (Pan: CH0 → P0.13, Tilt: CH1 → P0.11)\n");
}

/* === Pan update loop === */
void servo_thread(void) {
    int last_pan = -1;
    int64_t time_now = 0;
    int16_t sonic_distance = 0;
    // int16_t last_distance = 0;
    // int16_t distance_difference = 0;
    uint8_t count = 0;
    // bool heuristics_on = false;
    // servo_info_t local;
    // servo_info_t received;
    int angleStatus = 1;
    int angle = 90;
    int step = 3;

    //set inital state:
    servo_info.angle = SERVO_DEFAULT_ANGLE;
    servo_info.angle_pending = false;
    servo_info.mode = MODE_SWEEP;
    start_sweep(&servo_info);

    //should it notify with inital value?

    printk("Servo thread started\r\n");
    while (1) {

        k_mutex_lock(&PanAngleAccess,K_FOREVER);
        switch(servo_info.mode) {
            case MODE_TRACK:
                if(servo_info.angle_pending) {
                    //reset pending
                    servo_info.angle_pending = false;

                    //update timestamp
                    //servo_info.timestamp = k_uptime_get();

                    if(servo_info.angle < 0) {
                        //move left
                        angle -= step;

                        if (angle < 0) {
                            angle = 0;
                        } else if (angle > 180) {
                            angle = 180;
                        }
                        
                        move_servo_channel(PAN_CHANNEL, angle);
                        notify_angle(angle);
                    } else if(servo_info.angle > 0) {
                        //move right
                        angle += step;

                        if (angle < 0) {
                            angle = 0;
                        } else if (angle > 180) {
                            angle = 180;
                        }

                        move_servo_channel(PAN_CHANNEL, angle);
                        notify_angle(angle);
                    } else {

                        sonic_distance = measure_distance_cm(gpio_dev);
                        printk("sonic_distance: %d\r\n",sonic_distance);
                        notify_distance(sonic_distance); 
                        notify_angle(angle);
                    }
                } else {
                    // time_now = k_uptime_get();
                    // //check timeout
                    // if(time_now - local.timestamp >= TIMEOUT_MS) {
                    //     local.mode = MODE_SWEEP;
                    //     start_sweep(&local);
                    // }
                    time_now++;
                    if(time_now >= 10) {
                        time_now = 0;
                        servo_info.mode = MODE_SWEEP;
                        start_sweep(&servo_info);
                    }
                }
                k_sleep(K_MSEC(100));
                k_mutex_unlock(&PanAngleAccess); 
                break;

            case(MODE_SWEEP):
                if(servo_info.angle_pending) {
                    servo_info.mode = MODE_TRACK;
                    printk("switching\r\n");
                    continue;
                }

                update_sweep(&servo_info);
                count++;
                
                //this is only for pretty sweep visualization, so can be removed if not needed
                if(count >= SWEEP_ANGLE_UPDATE_RESOLUTION) {
                    count = 0;
                    notify_angle(servo_info.angle);
                    printk("angle %d\r\n",servo_info.angle);
                }

                k_mutex_unlock(&PanAngleAccess); 
                k_sleep(K_MSEC(100));
                break;

        }   
    //     //state machine for the servo (mealy given angle pending as input)
    //     switch(local.mode) {
    //         case(MODE_SWEEP):
    //             if(local.angle_pending) {
    //                 local.mode = MODE_TRACK;
    //                 continue;
    //             }

    //             update_sweep(&local);
    //             count++;
                
    //             //this is only for pretty sweep visualization, so can be removed if not needed
    //             if(count >= SWEEP_ANGLE_UPDATE_RESOLUTION) {
    //                 count = 0;
    //                 notify_angle(local.angle);
    //             }

    //             k_sleep(K_MSEC(100));
    //             break;
    //         case(MODE_TRACK):
    //             if(local.angle_pending) {

    //                 //reset pending
    //                 local.angle_pending = false;

    //                 //reset timer
    //                 local.timestamp = k_uptime_get();

    //                 // set angle to new one
    //                 if (last_pan != local.angle) {
    //                     last_pan = local.angle;

    //                     //move servo
    //                     move_servo_channel(PAN_CHANNEL, last_pan);
    //                     printk("[SERVO] Pan target set to: %d\n", local.angle);

    //                     //get distance at angle
    //                     sonic_distance = measure_distance_cm(gpio_dev);

    //                     //send both updates to all subcribed boards
    //                     notify_angle(last_pan);
    //                     notify_distance(sonic_distance); //need to move this out for close distace heuristics
    //                     last_distance = sonic_distance;

    //                     k_sleep(K_MSEC(10));
    //                 }
    //             } else {
    //                 time_now = k_uptime_get();
    //                 //check timeout
    //                 if(!heuristics_on && (time_now - local.timestamp >= TIMEOUT_MS)) {
    //                     local.mode = MODE_SWEEP;
    //                     start_sweep(&local);
    //                 }
                    
    //                 //get distance for heuristics
    //                 sonic_distance = measure_distance_cm(gpio_dev);

    //                 //get distance difference
    //                 distance_difference = sonic_distance - last_distance;
    //                 if(distance_difference < 0) {
    //                     distance_difference = -distance_difference;
    //                 }

    //                 //apply heuristics
    //                 if(sonic_distance < HEURISTIC_DISTANCE_THRESHOLD && 
    //                         distance_difference <   HEURISTIC_MAX_DISTANCE_DIFFERENCE) {
    //                     heuristics_on = true;

    //                     //send though off position of target
    //                     notify_angle(last_pan);
    //                     notify_distance(sonic_distance); 
    //                 } else {
    //                     heuristics_on = false;
    //                 }

    //                 k_sleep(K_MSEC(100));
    //             }
    //             break;
    //     }

    }
}

/* === Thread management === */
#define SERVO_STACK_SIZE 1024
#define SERVO_THREAD_PRIORITY 10

K_THREAD_STACK_DEFINE(servo_stack_area, SERVO_STACK_SIZE);
static struct k_thread servo_thread_data;

void servo_start_thread(void) {
    servo_init();

    k_thread_create(&servo_thread_data, servo_stack_area,
                    K_THREAD_STACK_SIZEOF(servo_stack_area),
                    (k_thread_entry_t)servo_thread, NULL, NULL, NULL,
                    SERVO_THREAD_PRIORITY, 0, K_NO_WAIT);

    printk("[SERVO] Servo thread started\n");
}