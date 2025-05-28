#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/printk.h>

#ifndef GATT_CLIENT_H
#define GATT_CLIENT_H

#pragma once

#define BT_UUID_CUSTOM_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_ANGLE_CHAR_VAL \
    BT_UUID_128_ENCODE(0xabcdabcd, 0xabcd, 0xabcd, 0xabcd, 0xabcdabcdef01)
#define BT_UUID_DISTANCE_CHAR_VAL \
    BT_UUID_128_ENCODE(0xdcbaabcd, 0xdcba, 0xdcba, 0xdcba, 0xdcbaabcdef02)
#define BT_UUID_POWER_STATE_VAL \
	BT_UUID_128_ENCODE(0x87654321, 0x4321, 0x8765, 0x4321, 0xabcdef123456)

struct service_info {
    char name[12];
    struct bt_gatt_subscribe_params params;
    uint16_t ccc_handle;
    uint16_t handle;
    bool subscribed;
    bool discovered;
};

extern struct service_info angle_info;
extern struct service_info distance_info;
extern struct service_info power_info;

extern struct k_msgq angleQueue;
extern struct k_msgq distanceQueue;

uint8_t startup_scan(void);
int write_data_with_response(int16_t value, volatile struct service_info* info);

#endif // MY_CLIENT