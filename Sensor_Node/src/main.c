/*
 * Copyright (c) 2018 Henrik Brix Andersen <henrik@brixandersen.dk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>
#include "my_gatt.h"
#include "servo.h"
#include "ultrasonic.h"
//#include "ultrasonic.h"

int main(void) {
    printk("Program startup\r\n");

    k_mutex_init(&PanAngleAccess);

    // /* Set a fixed static random address: f6:0b:b0:69:12:C6 */
    // bt_addr_le_t static_addr = {
    //     .type = BT_ADDR_LE_RANDOM,
    //     .a.val = { 0xC6, 0x12, 0x69, 0xb0, 0x0b, 0xF6 } // Little-endian
    // };

    // // int id = bt_id_create(&static_addr, NULL);
    // if (id < 0) {
    //     printk("Failed to create static identity (err %d)\n", id);
    // } else {
    //     printk("Created identity ID %d\n", id);
    // }

    int err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth enable failed (err %d)\n", err);
        return 0;
    }

    //ultrasonic_init();
    servo_start_thread();
    //ultrasonic_thread_init(); //just does not need the extra thread for it, all operations done in servo

    printk("All setup is complete\r\n");
    return 0;
}