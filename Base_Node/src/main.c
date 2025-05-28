#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/data/json.h>
#include <string.h>
#include "gatt_client.h"
#include "sub_settings.h"

struct json_data {
    int16_t angle;
    int16_t distance;
    int tracking_state;
};

static int start_turret(const struct shell* shell, size_t argc, char **argv) {

    if(argc > 1) {
        printk("%s\r\n","too many arguements");
        return 1;
    }

    //send new power state
    write_data_with_response(1,&power_info);
    return 0;
}

static int stop_turret(const struct shell* shell, size_t argc, char **argv) {
    if(argc > 1) {
        printk("%s\r\n","too many arguements");
        return 1;
    }

    //send new power state
    write_data_with_response(0,&power_info);
    return 0;
}

//use the json_data instead to keep distance and angle values
//no variable needed for tracking state given it is determined through not sending distance
static void send_json(struct json_data* data, struct json_obj_descr* descr, char* char_buffer) {
    //encode json
    printk("{\"angle\": %d, \"distance\": %d, \"tracking\": %d}\n",
       data->angle, data->distance, data->tracking_state);
    //json_obj_encode_buf(descr, 3, data, char_buffer, 128);
    //printk("%s\r\n",char_buffer); //use this for now, otherwise try using usb cdc acm
    //send json
}

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
    // Now safe to start scanning
    startup_scan();
}

//use thread, or use main to send received notifications for the angle and distance to dashboard
int main(void) {
    int err;

    //load service settings
    settings_load_subs();
    if(angle_info.handle == 0 || distance_info.handle == 0 || power_info.handle == 0) {
        printk("It is actually a problem %d\r\n", power_info.handle);
        settings_loaded = false;
    }
    settings_loaded = false;

    printk("handle: %d\r\n",angle_info.handle);

    printk("Starting BLE Central Client\n");

    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 1;
    }

    printk("Scanning started\n");
    
    char jsonFull[128];
    struct json_obj_descr descriptor[] = {
    JSON_OBJ_DESCR_PRIM_NAMED(struct json_data,"angle", angle, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM_NAMED(struct json_data,"distance", distance, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM_NAMED(struct json_data,"tracking",tracking_state, JSON_TOK_NUMBER)
    };

    int angleStatus = 0;
    int distanceStatus = 0;
    struct json_data data;

    //main loop //gets the angle_distance data + authorization(implied)+ tracking and sends it to the dashboard
    while(1) {

        //read if any value is available for 
        angleStatus = k_msgq_get(&angleQueue, &data.angle, K_FOREVER); 
        distanceStatus = k_msgq_get(&distanceQueue, &data.distance, K_NO_WAIT);

        //there is a location to track
        if(distanceStatus == 0) {
            data.tracking_state = 1;
        } else {
            data.distance = 0; //sweeping
            data.tracking_state = 0;
        }

        //json normally
        send_json(&data,descriptor,jsonFull);

        //call delay

        k_sleep(K_MSEC(50));

    }
}

SHELL_STATIC_SUBCMD_SET_CREATE(turret_commands,
        SHELL_CMD(start, NULL, "Add beacon nodes to array",start_turret),
        SHELL_CMD(stop, NULL, "Remove Beacon nodes from array",stop_turret),
        SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(Turret, &turret_commands,  "Main command with subcommands for all action",NULL);