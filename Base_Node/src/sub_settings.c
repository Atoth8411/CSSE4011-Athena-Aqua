#include <zephyr/sys/printk.h>
#include <string.h>
#include <zephyr/settings/settings.h>
#include "gatt_client.h"
#include "sub_settings.h"

//accessed by this once before bluetooth, no mutex necesseary
volatile bool settings_loaded = false;

int save_service_info(volatile struct service_info* info) {

    char key[27] = "subs/";
    //create the key myself
    strncpy(&key[5], (char *)info->name, 22);

    struct service_info_store tmp = {
        .handle = info->handle,
        .ccc_handle = info->ccc_handle,
        .subscribed = info->subscribed //have to fix
    };
    printk("saved to %s\r\n",key);
    return settings_save_one(key, &tmp, sizeof(tmp));
}

//load callback for each variable
static int settings_handler_cb(const char *key, size_t len, settings_read_cb read_cb,
                               void *cb_arg) {
    struct service_info *info;

    if (strcmp(key, "angle") == 0) {
        info = &angle_info;
    } else if (strcmp(key, "distance") == 0) {
        info = &distance_info;
    } else if (strcmp(key, "power") == 0) {
        info = &power_info;
    } else {
        printk("failed to read data\r\n");
        return -ENOENT;
    }

    struct service_info_store tmp;

    ssize_t ret = read_cb(cb_arg, &tmp, sizeof(tmp));
    if (ret <= 0) {
        printk("failed to read data here\r\n");
        return -EIO;
    }

    printk("handle: %d\r\n",tmp.handle);
    info->handle = tmp.handle;
    info->ccc_handle = tmp.ccc_handle;
    //info->subscribed = tmp.subscribed; //leave it false for subcriptions
    info->discovered = true;

    return 0;
}


void settings_load_subs(void) {
    int err;

    err = settings_subsys_init();
    if (err) {
        printk("Settings subsystem init failed (err %d)\n", err);
        return;
    }

    static struct settings_handler subs_handler = {
    .name = SETTINGS_SUB_PATH,
    .h_set = settings_handler_cb,
    };

    err = settings_register(&subs_handler);
    if (err) {
        printk("Settings handler register failed (err %d)\n", err);
        return;
    }

    err = settings_load_subtree(SETTINGS_SUB_PATH);
    if (err) {
        printk("Failed to load subscriptions settings (err %d)\n", err);
        settings_loaded = false;
    } else {
        printk("Subscriptions loaded from settings\n");
        settings_loaded = true;
    }
}