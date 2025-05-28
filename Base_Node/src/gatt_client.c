#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/printk.h>
#include <zephyr/data/json.h>
#include <stdbool.h>
#include <string.h>
#include "gatt_client.h"
#include "sub_settings.h"

static struct bt_uuid_128 uuid_service = BT_UUID_INIT_128(BT_UUID_CUSTOM_SERVICE_VAL);
static struct bt_uuid_128 uuid_angle = BT_UUID_INIT_128(BT_UUID_ANGLE_CHAR_VAL);
static struct bt_uuid_128 uuid_distance = BT_UUID_INIT_128(BT_UUID_DISTANCE_CHAR_VAL);
static struct bt_uuid_128 uuid_power = BT_UUID_INIT_128(BT_UUID_POWER_STATE_VAL);

static struct bt_conn *default_conn;
static uint16_t start_handle, end_handle;

static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_discover_params character_params;
static struct bt_gatt_discover_params descriptor_params;
static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params);
uint8_t startup_scan(void);

K_MSGQ_DEFINE(angleQueue, sizeof(int16_t), 10, 8);
K_MSGQ_DEFINE(distanceQueue, sizeof(int16_t), 10, 8); //

//does not need mutex
struct service_info angle_info = {.name = "angle", .subscribed = false, .discovered = false};
struct service_info distance_info = {.name = "distance", .subscribed = false, .discovered = false};
struct service_info power_info = {.name = "power", .subscribed = false, .discovered = false};

//has to account for if connection is still referencable
int write_data_no_ack(int16_t value, volatile struct service_info* info) {

    struct bt_conn *conn = bt_conn_ref(default_conn);
    if (!conn) {
        return -ENOTCONN;
    }

    if(!info->discovered) {
        return -1;
    } 

    //int err = bt_gatt_write_with_response(conn, info->handle, &value, sizeof(value), false);
    // if (err) {
	// 	printk("Failed to write %s (err %d)\n",info->name, err);
	// } else {
	// 	printk("%s value %d written to server\n",info->name, value);
	// }

    bt_conn_unref(conn);

    return 0;
}

//should save to a global variable and a global flag, or a message queue(use this) to get notification values
//then use the message queues to json the shit and any additional porcessing(timestamp, alarm??)
static uint8_t notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length)
{
    if (!data || length != sizeof(int16_t)) {
        printk("Notification stopped or invalid length\n");
        return BT_GATT_ITER_CONTINUE;
    }

    int16_t value;
    memcpy(&value, data, sizeof(value));

    if (params == &angle_info.params) {
        //printk("Angle: %d\n", value);
        k_msgq_put(&angleQueue,&value, K_NO_WAIT); //put received value in message queue to be processed in main

    } else if (params == &distance_info.params) {
        printk("Distance: %d\n", value);
        k_msgq_put(&distanceQueue,&value, K_NO_WAIT);

    }

    return BT_GATT_ITER_CONTINUE;
}

static void generic_sub_start(struct bt_conn *conn, struct service_info *info) {
    if(info->subscribed) {
        return;
    }

	if(info->handle && info->ccc_handle) {
		info->params.ccc_handle = info->ccc_handle;
		info->params.value_handle = info->handle;
        info->params.notify = notify_func;
        info->params.value = BT_GATT_CCC_NOTIFY;
	}

	 	int err = bt_gatt_subscribe(conn, &info->params);
        if (err) {
            printk("Failed to subscribe to %s (err %d)\n",info->name, err);
        } else {
            printk("Subscribed to %s notifications\n",info->name);
            info->subscribed = true;
            if(!settings_loaded) { //if not already saved, save it (for angle and distance only)
                save_service_info(info);
            }
        }
}

static void run_discovery(struct bt_gatt_discover_params* params,uint8_t type, uint16_t start, uint16_t end, struct bt_uuid *uuid) {

	params->uuid = uuid;
    params->func = discover_func; //i can change this
    params->start_handle = start;
    params->end_handle = end;
    params->type = type;

    int dis_err = bt_gatt_discover(default_conn, params);
    if (dis_err) {
        printk("Discovery type %u failed (err %d)\n", type, dis_err);
    }
}

static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("Discovery complete\n");
        memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc;

    if (params->type == BT_GATT_DISCOVER_PRIMARY) {
        start_handle = attr->handle + 1;
        end_handle = ((struct bt_gatt_service_val *)attr->user_data)->end_handle;

        printk("Discovered service: start %u, end %u\n", start_handle, end_handle);

		//related to this
		run_discovery(&character_params,BT_GATT_DISCOVER_CHARACTERISTIC, start_handle, end_handle,NULL);

        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {

        chrc = (struct bt_gatt_chrc *)attr->user_data;

        if (!bt_uuid_cmp(chrc->uuid, &uuid_angle.uuid)) {

            angle_info.handle =  chrc->value_handle;
            angle_info.discovered = true;
            printk("Found angle characteristic at handle %u\n", angle_info.handle);
        } else if (!bt_uuid_cmp(chrc->uuid, &uuid_distance.uuid)) {

            distance_info.handle = chrc->value_handle;
            distance_info.discovered = true;
            printk("Found distance characteristic at handle %u\n", distance_info.handle);
        } else if (!bt_uuid_cmp(chrc->uuid, &uuid_power.uuid)) {

            power_info.handle = chrc->value_handle;
            power_info.discovered = true;
            printk("Found power_state characteristic at handle %u\n", power_info.handle);
            if(!settings_loaded) { //save it for power
                save_service_info(&power_info);
            }
        }

		if (angle_info.handle && distance_info.handle && power_info.handle) {
			run_discovery(&descriptor_params,BT_GATT_DISCOVER_DESCRIPTOR, distance_info.handle + 1, end_handle, NULL);

            return BT_GATT_ITER_STOP;
		}

		return BT_GATT_ITER_CONTINUE;
    }

    if (params->type == BT_GATT_DISCOVER_DESCRIPTOR) {

        if (attr->handle == angle_info.handle + 1) {
            angle_info.ccc_handle = attr->handle;
            printk("Found CCC for angle at handle %u\n", angle_info.ccc_handle);
			generic_sub_start(conn,&angle_info);

        } else if (attr->handle == distance_info.handle + 1) {
            distance_info.ccc_handle = attr->handle;
            printk("Found CCC for distance at handle %u\n", distance_info.ccc_handle);
			generic_sub_start(conn,&distance_info);

        }
        return BT_GATT_ITER_CONTINUE;
    }

    return BT_GATT_ITER_CONTINUE;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
        return;
    }

    printk("Connected\n");
    default_conn = bt_conn_ref(conn);

    //just subcribe if already loaded all subcription settings
    if(settings_loaded) {
        generic_sub_start(default_conn,&angle_info);
        generic_sub_start(default_conn,&distance_info);
        return;
    }

	//run primary discovery if not loaded settings
	run_discovery(&discover_params,BT_GATT_DISCOVER_PRIMARY,BT_ATT_FIRST_ATTRIBUTE_HANDLE,
				  BT_ATT_LAST_ATTRIBUTE_HANDLE,&uuid_service.uuid);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }

    //startup_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static bool eir_found(struct bt_data *data, void *user_data)
{
    bt_addr_le_t *orig_addr = user_data;

    if (data->type == BT_DATA_UUID128_ALL &&
        data->data_len == 16 &&
        !memcmp(data->data, uuid_service.val, 16)) {

        // Copy address before using it
        static bt_addr_le_t target_addr;
        bt_addr_le_copy(&target_addr, orig_addr);

        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&target_addr, addr_str, sizeof(addr_str));
        printk("Connecting to %s\n", addr_str);

        const struct bt_le_conn_param *param = BT_LE_CONN_PARAM_DEFAULT;
        int err = bt_le_scan_stop();
        if (err) {
            printk("Stop LE scan failed (err %d)\n", err);
            return false;
        }

        err = bt_conn_le_create(&target_addr, BT_CONN_LE_CREATE_CONN, param, &default_conn);
        if (err) {
            printk("Create connection failed (err %d)\n", err);
            return false;
        }

        return false; // stop parsing
    }

    return true;
}

// static bool eir_found(struct bt_data *data, void *user_data)
// {
//     bt_addr_le_t *addr = user_data;
//     //int i;

//     if (data->type == BT_DATA_UUID128_ALL &&
//         data->data_len == 16 &&
//         !memcmp(data->data, uuid_service.val, 16)) {

//         struct bt_le_conn_param *param = BT_LE_CONN_PARAM_DEFAULT;
//         int err = bt_le_scan_stop();
//         if (err) {
//             printk("Stop LE scan failed (err %d)\n", err);
//             return false;
//         }

//         err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &default_conn);
//         if (err) {
//             printk("Create connection failed (err %d)\n", err);
//             return false;
//         }

//         return false; // stop parsing
//     }

//     return true;
// }

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
     if (type == BT_GAP_ADV_TYPE_ADV_IND || type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        bt_data_parse(ad, eir_found, (void *)addr);
    }
}

uint8_t startup_scan(void) {
    int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
    if (err) {
        printk("Scanning failed to start (err %d)\n", err);
        return 1;
    }
    return 0;
}