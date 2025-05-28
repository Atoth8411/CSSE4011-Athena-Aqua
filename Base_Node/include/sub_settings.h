#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/printk.h>

#ifndef SUB_SETTINGS_H
#define SUB_SETTINGS_H

//defines path and subfolders for settings
#define SETTINGS_SUB_PATH "subs"

struct service_info_store {
    uint16_t handle;
    uint16_t ccc_handle;
    bool subscribed;
}__packed;

void settings_load_subs(void);
int save_service_info(volatile struct service_info* info);

extern volatile bool settings_loaded;

#endif // MY_CLIENT