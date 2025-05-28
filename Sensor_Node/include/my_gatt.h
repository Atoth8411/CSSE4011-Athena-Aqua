#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>

#ifndef MY_GATT_H
#define MY_GATT_H

#pragma once

/* Custom Service UUID */
#define BT_UUID_CUSTOM_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_ANGLE_CHAR_VAL \
	BT_UUID_128_ENCODE(0xabcdabcd, 0xabcd, 0xabcd, 0xabcd, 0xabcdabcdef01)

#define BT_UUID_DISTANCE_CHAR_VAL \
	BT_UUID_128_ENCODE(0xdcbaabcd, 0xdcba, 0xdcba, 0xdcba, 0xdcbaabcdef02)

#define BT_UUID_POWER_STATE_VAL \
	BT_UUID_128_ENCODE(0x87654321, 0x4321, 0x8765, 0x4321, 0xabcdef123456)

#define BT_UUID_TRACKING_STATE_VAL \
	BT_UUID_128_ENCODE(0xabcdef01, 0x2345, 0x6789, 0xabcd, 0xef0123456789)

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define MAX_BT_CONNECTIONS CONFIG_BT_MAX_CONN

extern bool angle_notify_enabled;
extern bool distance_notify_enabled;

void bt_ready(int err);
void notify_distance(uint16_t dist);
void notify_angle(int16_t degrees);

#endif // SERVO_H