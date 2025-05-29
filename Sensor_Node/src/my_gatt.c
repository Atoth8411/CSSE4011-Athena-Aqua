#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdbool.h>
#include "my_gatt.h"
#include "servo.h"

// Subscription counts per characteristic
static int8_t angle_sub_count = 0;
static int8_t distance_sub_count = 0;
static int8_t power_sub_count = 0;
static int8_t track_sub_count = 0;
static int8_t connection_count = 0;

static struct bt_uuid_128 custom_service_uuid = BT_UUID_INIT_128(BT_UUID_CUSTOM_SERVICE_VAL);
static struct bt_uuid_128 angle_uuid = BT_UUID_INIT_128(BT_UUID_ANGLE_CHAR_VAL);
static struct bt_uuid_128 distance_uuid = BT_UUID_INIT_128(BT_UUID_DISTANCE_CHAR_VAL);
static struct bt_uuid_128 tracking_state_uuid = BT_UUID_INIT_128(BT_UUID_TRACKING_STATE_VAL); 
static struct bt_uuid_128 power_state_uuid = BT_UUID_INIT_128(BT_UUID_POWER_STATE_VAL);//can use from jetson to indicate when to actually read distance

/* Application State */
static int16_t angle = 0;
static int16_t distance = 0;
static uint8_t power_state = 1;
static uint8_t tracking_state = 1;

/* Advertisement data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};


static struct k_work_delayable adv_restart_work;

static void adv_restart(struct k_work *work)
{
    int err = bt_le_adv_stop();
    if (err && err != -EALREADY) {
        printk("Failed to stop advertising: %d\n", err);
        return;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Failed to restart advertising: %d\n", err);
    } else {
        printk("Restarted advertising (via work queue)\n");
    }
}

/* CCC callbacks - track subscription count */
static void angle_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (value == BT_GATT_CCC_NOTIFY) {
        angle_sub_count++;
    } else {
        angle_sub_count--;
        if (angle_sub_count < 0) angle_sub_count = 0;
    }
    printk("Angle notifications %s, subscribers: %d\n",
           value == BT_GATT_CCC_NOTIFY ? "enabled" : "disabled", angle_sub_count);
}

static void distance_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (value == BT_GATT_CCC_NOTIFY) {
        distance_sub_count++;
    } else {
        distance_sub_count--;
        if (distance_sub_count < 0) distance_sub_count = 0;
    }
    printk("Distance notifications %s, subscribers: %d\n",
           value == BT_GATT_CCC_NOTIFY ? "enabled" : "disabled", distance_sub_count);
}

static void power_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (value == BT_GATT_CCC_NOTIFY) {
        power_sub_count++;
    } else {
        power_sub_count--;
        if (power_sub_count < 0) power_sub_count = 0;
    }
    printk("Power notifications %s, subscribers: %d\n",
           value == BT_GATT_CCC_NOTIFY ? "enabled" : "disabled", power_sub_count);
}

static void track_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (value == BT_GATT_CCC_NOTIFY) {
        track_sub_count++;
    } else {
        track_sub_count--;
        if (track_sub_count < 0) track_sub_count = 0;
    }
    printk("Tracking notifications %s, subscribers: %d\n",
           value == BT_GATT_CCC_NOTIFY ? "enabled" : "disabled", track_sub_count);
}

// Work item and associated data
static struct k_work angle_work;

// Worker function
static void angle_work_handler(struct k_work *work) {
    servo_info_t local;
    int err;

    // Safely update shared data
    //k_mutex_lock(&PanAngleAccess, K_FOREVER);
    servo_info.angle = angle;
    servo_info.angle_pending = true;
    local = servo_info;
    //k_mutex_unlock(&PanAngleAccess);

    printk("Angle updated by client (deferred): %d\n", servo_info.angle);
}

// Init function (call once during init or main)
void angle_work_init(void) {
    k_work_init(&angle_work, angle_work_handler);
}

static ssize_t write_angle(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                           const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {

    if (len != sizeof(int16_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&angle, buf, sizeof(angle));

    // Offload work to avoid blocking the GATT stack
    k_work_submit(&angle_work);

    return sizeof(angle);
}

// static ssize_t write_angle(struct bt_conn *conn, const struct bt_gatt_attr *attr,
// 		       const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {

// 	if (len != sizeof(int16_t)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

// 	memcpy(&angle, buf, sizeof(angle));
//     k_mutex_lock(&PanAngleAccess,K_FOREVER);
//     servo_info.angle = angle;
//     servo_info.angle_pending = true;
//     k_mutex_unlock(&PanAngleAccess);
// 	printk("Angle updated by client: %d\n", angle);
//     int err;

    
//     // err = k_msgq_put(&angleData,&local,K_NO_WAIT);

//     // //check if full and remove latest element and resend if so
//     // if(err == -ENOMSG || err == -ENOMEM) {
//     //     servo_info_t discard;
//     //     k_msgq_get(&angleData,&discard,K_NO_WAIT);
//     //     k_msgq_put(&angleData,&local,K_NO_WAIT);
//     // }
//     //can notify like this in this configuration
//     // if (angle_sub_count > 0) {
//     //     bt_gatt_notify(NULL, attr, &angle, sizeof(angle));
//     // }

// 	return sizeof(angle);
// }

//variable may not be used, but implemented in case it is needed
static ssize_t write_power_state(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset,uint8_t flags)
{
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    uint8_t new_val = *((uint8_t *)buf);

    if (new_val != power_state) {
        power_state = new_val;

        turret_state = power_state;

        printk("Power state updated to: %d\n", power_state);

        //it should notifiy given jetson should be subscribed to this
        // if (power_sub_count > 0) {
        //     bt_gatt_notify(NULL, attr, &power_state, sizeof(power_state));
        // }
    }

    return len;
}

static ssize_t write_tracking_state(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t new_val = *((uint8_t *)buf);

    if (new_val != tracking_state) {
        tracking_state = new_val;

        printk("Tracking state updated to: %d\n", tracking_state);

        //same here, tracking state is internal
        // if (track_sub_count > 0) {
        //     bt_gatt_notify(NULL, attr, &tracking_state, sizeof(tracking_state));
        // }
    }

    return len;
}

/* GATT service definition */
BT_GATT_SERVICE_DEFINE(custom_svc,
	BT_GATT_PRIMARY_SERVICE(&custom_service_uuid),

    /* Read-only characteristic with notify: Distance */
	BT_GATT_CHARACTERISTIC(&distance_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ, NULL, NULL, &distance),
	BT_GATT_CCC(distance_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Read-write characteristic with notify: Angle */
	BT_GATT_CHARACTERISTIC(&angle_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL, write_angle, &angle),
	BT_GATT_CCC(angle_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* Power state characteristic */
    BT_GATT_CHARACTERISTIC(&power_state_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        NULL, write_power_state, &power_state),
    BT_GATT_CCC(power_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* Tracking state characteristic */
    BT_GATT_CHARACTERISTIC(&tracking_state_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        NULL, write_tracking_state, &tracking_state),
    BT_GATT_CCC(track_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Notify helpers */
void notify_distance(uint16_t dist) {
    if(dist == 0 || tracking_state == 0) { //return if no valid reading or not tracking currently
        return;
    }
    distance = dist;

    if (distance_sub_count > 0) {
        bt_gatt_notify(NULL, &custom_svc.attrs[2], &distance, sizeof(distance));
    }
}

void notify_angle(int16_t degrees) {
    //angle = degrees;

    if (angle_sub_count > 0) {
        bt_gatt_notify(NULL, &custom_svc.attrs[5], &degrees, sizeof(degrees));
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
    } else {
        connection_count++;
        printk("Connected to %u/%u clients\n",connection_count, MAX_BT_CONNECTIONS);
        if(connection_count < MAX_BT_CONNECTIONS) {
            printk("connection count: %d\r\n",connection_count);
            k_work_schedule(&adv_restart_work, K_NO_WAIT);
        }
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{   
    connection_count--;
    printk("Disconnected %u/%u, (reason %u)\n",connection_count,MAX_BT_CONNECTIONS,reason);

    //restart advertising if new slot was made available, otherwise do not start(connection already handles that case)
    k_work_schedule(&adv_restart_work, K_NO_WAIT);
}

static void setup_advertising(void) {
    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
    } else {
        printk("Advertising started\n");
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    bt_conn_cb_register(&conn_callbacks);

    if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}


    angle_work_init();
    k_work_init_delayable(&adv_restart_work, adv_restart);
    setup_advertising();
}