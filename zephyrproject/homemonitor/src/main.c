/*
* Copyright (c) 2023 Craig Peacock.
* Copyright (c) 2017 ARM Ltd.
* Copyright (c) 2016 Intel Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>

#include <stdio.h>
#include <errno.h>

#include "http_get.h"

/* ============
    Globals
============ */
#define HOMEMONITOR_DEBUG  false

#define SSID  "Naffaa"
#define PSK   "LimePlace!"

#define MINUTES_TO_SECONDS         60
#define DHT_POLLING_DELAY_MINUTES  (30 * MINUTES_TO_SECONDS)

#define THINGSPEAK_URL_MAX_LENGTH  80

#define THINGSPEAK_WRITE_API_KEY   "I4BV5Q70NNDWH0SP"
//#define THINGSPEAK_WRITE_API_KEY   "1LQH82DIQIIJHFRQ"

static K_SEM_DEFINE(wifi_connected, 0, 1);
static K_SEM_DEFINE(ipv4_address_obtained, 0, 1);

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

/* ========================
    Function Prototypes
======================== */
static const char* get_current_timestamp_string(void);
static inline int create_thingspeak_http_get_url(char* buffer, double field1, double field2);

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb);
static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb);
static void handle_ipv4_result(struct net_if *iface);
static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                    uint32_t mgmt_event,
                                    struct net_if *iface);
void wifi_connect(void);
void wifi_status(void);
void wifi_disconnect(void);

int main(void)
{
    const struct device * const dht22 = DEVICE_DT_GET_ONE(aosong_dht);
    if (!device_is_ready(dht22))
    {
        printk("Device %s is not ready\n", dht22->name);
        return 0;
    }

    printk("HomeMonitor Data Collection\n");
    printk("--> Board: %s\n", CONFIG_BOARD);
    printk("--> Polling from device: %s\n", dht22->name);

    /* Attempt to connect to WiFi */
    net_mgmt_init_event_callback(&wifi_cb, wifi_mgmt_event_handler,
                                 NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_init_event_callback(&ipv4_cb, wifi_mgmt_event_handler, NET_EVENT_IPV4_ADDR_ADD);

    net_mgmt_add_event_callback(&wifi_cb);
    net_mgmt_add_event_callback(&ipv4_cb);

    wifi_connect();
    k_sem_take(&wifi_connected, K_FOREVER);
    wifi_status();
    k_sem_take(&ipv4_address_obtained, K_FOREVER);

    /* Poll DHT sensor indefinitely */
    while (true) {
        /* Collect Data from DHT */
        char thingspeakUrl[THINGSPEAK_URL_MAX_LENGTH];
        
        int rc = sensor_sample_fetch(dht22);
        if (rc != 0)
        {
            printk("Sensor fetch failed: %d\n", rc);
            break;
        }

        struct sensor_value temperature;
        struct sensor_value humidity;

        rc = sensor_channel_get(dht22, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
        if (rc == 0) {
            rc = sensor_channel_get(dht22, SENSOR_CHAN_HUMIDITY, &humidity);
        }
        if (rc != 0) {
            printk("Sensor get failed: %d\n", rc);
            break;
        }

        printk("[%s]: %.1f Cel ; %.1f %%RH\n",
                get_current_timestamp_string(),
                sensor_value_to_double(&temperature),
                sensor_value_to_double(&humidity));

        /* Upload data to ThingSpeak */
        int sock;
        char* thingspeakHostname = "api.thingspeak.com";

        rc = create_thingspeak_http_get_url(thingspeakUrl,
                                            sensor_value_to_double(&temperature),
                                            sensor_value_to_double(&humidity));
        if (rc < 0) {
            printk("Thingspeak URL creation failed: %d\n", rc);
            break;
        }
        printk("Thingspeak URL (len = %d)\n  %s\n", rc, thingspeakUrl);

        struct zsock_addrinfo *res;
        nslookup(thingspeakHostname, &res);
        #if (HOMEMONITOR_DEBUG)
        print_addrinfo_results(&res);
        #endif

        sock = connect_socket(&res, 80);

        rc = http_get(sock, thingspeakHostname, thingspeakUrl);
        if (rc < 0) {
            printk("Couldn't HTTP GET to ThingSpeak\n");

            /* Known issue with lower priced ESP32 where WiFi module fails periodically
               (roughly once every few days). Rebooting resolves this issue, and this is
               done is the currently fetched data point will be re-fetched on power cycle
            */
            sys_reboot(SYS_REBOOT_COLD);
        }

        zsock_close(sock);

        k_sleep(K_SECONDS(DHT_POLLING_DELAY_MINUTES));
    }

    wifi_disconnect();

    return 0;
}

/**
 * @brief Create a thingspeak HTTP GET url object
 * 
 * @param buffer - Location to store formatted string
 * @param field1 - Value of Field1
 * @param field2 - Value of Field2
 */
static inline int create_thingspeak_http_get_url(char* buffer, double field1, double field2)
{
    // TODO: Add field 2 support for humidity
    double field1_fahrenheit = (field1 * 1.8) + 32;
    return snprintf(buffer, THINGSPEAK_URL_MAX_LENGTH,
                    "/update.json?api_key=%s&field1=%.1f&field2=%.1f",
                    THINGSPEAK_WRITE_API_KEY, field1_fahrenheit, field2);
}

/**
 * @brief Return a formatted string representing the current time
 * 
 * @return String formatted as "HH:MM:SS.MMM"
 */
static const char* get_current_timestamp_string(void)
{
    static char buf[16]; /* ...HH:MM:SS.MMM */
    uint32_t now = k_uptime_get_32();
    unsigned int ms = now % MSEC_PER_SEC;
    unsigned int s;
    unsigned int min;
    unsigned int h;

    now /= MSEC_PER_SEC;
    s = now % 60U;
    now /= 60U;
    min = now % 60U;
    now /= 60U;
    h = now;

    snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u", h, min, s, ms);
    return buf;
}

/**
 * @brief Callback to determine whether WiFi has connected successfully.
 *        Invoked by network management when a connection is attempted.
 * 
 * @param cb - Callback structure used to register a callback
 *             into the network management event
 */
static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_status *status = (const struct wifi_status *)cb->info;

    if (status->status)
    {
        printk("Connection request failed (%d)\n", status->status);
    }
    else
    {
        printk("Connected\n");
        k_sem_give(&wifi_connected);
    }
}

/**
 * @brief Callback to determine whether WiFi has disconnected successfully.
 *        Invoked by network management when a disconnect is attempted.
 * 
 * @param cb - Callback structure used to register a callback
 *             into the network management event
 */
static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_status *status = (const struct wifi_status *)cb->info;

    if (status->status)
    {
        printk("Disconnection request (%d)\n", status->status);
    }
    else
    {
        printk("Disconnected\n");
        k_sem_take(&wifi_connected, K_NO_WAIT);
    }
}

/**
 * @brief Callback to convert IP address to string form and print result
 * 
 * @param iface - Structure used to handle a network interface on top
 *                of a net_if_dev instance
 */
static void handle_ipv4_result(struct net_if *iface)
{
    int i = 0;

    for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {

        char buf[NET_IPV4_ADDR_LEN];

        if (iface->config.ip.ipv4->unicast[i].ipv4.addr_type != NET_ADDR_DHCP) {
            continue;
        }

        printk("IPv4 address: %s\n",
                net_addr_ntop(AF_INET,
                              &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr,
                              buf, sizeof(buf)));
        printk("Subnet: %s\n",
                net_addr_ntop(AF_INET,
                              &iface->config.ip.ipv4->unicast[i].netmask,
                              buf, sizeof(buf)));
        printk("Router: %s\n",
                net_addr_ntop(AF_INET,
                              &iface->config.ip.ipv4->gw,
                              buf, sizeof(buf)));
        }

        k_sem_give(&ipv4_address_obtained);
}

/**
 * @brief Dispatch function to invoke appropriate callback based on
 *        the network management event specified
 * 
 * @param cb         - Callback structure used to register a callback
 *                     into the network management event
 * @param mgmt_event - Value determining callback to invoke
 * @param iface      - Structure used to handle a network interface on top
 *                     of a net_if_dev instance
 */
static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                    uint32_t mgmt_event,
                                    struct net_if *iface)
{
    switch (mgmt_event)
    {
        case NET_EVENT_WIFI_CONNECT_RESULT:
            handle_wifi_connect_result(cb);
            break;
        case NET_EVENT_WIFI_DISCONNECT_RESULT:
            handle_wifi_disconnect_result(cb);
            break;
        case NET_EVENT_IPV4_ADDR_ADD:
            handle_ipv4_result(iface);
            break;
        default:
            break;
    }
}

/**
 * @brief Functionality to attempt to connect to WiFi
 */
void wifi_connect(void)
{
    struct net_if *iface = net_if_get_default();

    struct wifi_connect_req_params wifi_params = {0};

    wifi_params.ssid = SSID;
    wifi_params.psk = PSK;
    wifi_params.ssid_length = strlen(SSID);
    wifi_params.psk_length = strlen(PSK);
    wifi_params.channel = WIFI_CHANNEL_ANY;
    wifi_params.security = WIFI_SECURITY_TYPE_PSK;
    wifi_params.band = WIFI_FREQ_BAND_2_4_GHZ; 
    wifi_params.mfp = WIFI_MFP_OPTIONAL;

    printk("Connecting to SSID: %s\n", wifi_params.ssid);

    if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
                 &wifi_params, sizeof(struct wifi_connect_req_params)))
    {
        printk("WiFi Connection Request Failed\n");
    }
}

/**
 * @brief Functionality to display WiFi status
 */
void wifi_status(void)
{
    struct net_if *iface = net_if_get_default();
    
    struct wifi_iface_status status = {0};

    if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface,
                 &status, sizeof(struct wifi_iface_status)))
    {
        printk("WiFi Status Request Failed\n");
    }

    #if (HOMEMONITOR_DEBUG)
    printk("\n");

    if (status.state >= WIFI_STATE_ASSOCIATED) {
        printk("SSID: %-32s\n", status.ssid);
        printk("Band: %s\n", wifi_band_txt(status.band));
        printk("Channel: %d\n", status.channel);
        printk("Security: %s\n", wifi_security_txt(status.security));
        printk("RSSI: %d\n", status.rssi);
    }
    #endif
}

/**
 * @brief Functionality to attempt to disconnect from WiFi
 */
void wifi_disconnect(void)
{
    struct net_if *iface = net_if_get_default();

    if (net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0))
    {
        printk("WiFi Disconnection Request Failed\n");
    }
}
