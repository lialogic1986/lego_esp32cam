// prov_ble.c (fixed: STATUS characteristic has READ+NOTIFY with access_cb)
// ESP-IDF v5.5.x + NimBLE
//
// Notes:
// - Uses chunked CFG_RX protocol and CMD=COMMIT/ABORT
// - STATUS_TX now supports READ+NOTIFY to satisfy NimBLE GATT validation
// - ble_store_config_init() is called
//
// Keep UUIDs in sync with your host script.

#include "prov_ble.h"

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_mac.h"

#include "cJSON.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/util/util.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "store/config/ble_store_config.h"

static const char *TAG = "prov_ble";

/* ======== UUIDs ======== */
static const ble_uuid128_t UUID_PROV_SVC =
    BLE_UUID128_INIT(0x9a, 0x7c, 0x10, 0x2b, 0x6f, 0x8b, 0x4d, 0x26, 0x92, 0x6e, 0x7b, 0x14, 0x33, 0xa1, 0x10, 0x01);

static const ble_uuid128_t UUID_INFO_CHR =
    BLE_UUID128_INIT(0x9a, 0x7c, 0x10, 0x2b, 0x6f, 0x8b, 0x4d, 0x26, 0x92, 0x6e, 0x7b, 0x14, 0x33, 0xa1, 0x10, 0x02);

static const ble_uuid128_t UUID_CFG_RX_CHR =
    BLE_UUID128_INIT(0x9a, 0x7c, 0x10, 0x2b, 0x6f, 0x8b, 0x4d, 0x26, 0x92, 0x6e, 0x7b, 0x14, 0x33, 0xa1, 0x10, 0x03);

static const ble_uuid128_t UUID_STATUS_TX_CHR =
    BLE_UUID128_INIT(0x9a, 0x7c, 0x10, 0x2b, 0x6f, 0x8b, 0x4d, 0x26, 0x92, 0x6e, 0x7b, 0x14, 0x33, 0xa1, 0x10, 0x04);

static const ble_uuid128_t UUID_CMD_CHR =
    BLE_UUID128_INIT(0x9a, 0x7c, 0x10, 0x2b, 0x6f, 0x8b, 0x4d, 0x26, 0x92, 0x6e, 0x7b, 0x14, 0x33, 0xa1, 0x10, 0x05);

/* ======== CMD codes ======== */
#define PROV_CMD_COMMIT 0x01
#define PROV_CMD_ABORT 0x02

/* ======== Internal state ======== */
static prov_ble_cfg_t g_cfg;

static uint16_t g_status_val_handle = 0;
static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;

/* config assembly */
static uint8_t *g_buf = NULL;
static uint16_t g_buf_max = 0;
static uint16_t g_total_len = 0;
static uint32_t g_session_id = 0;
static bool g_have_total = false;

/* bitmap per 16 bytes */
static uint8_t *g_blkmap = NULL;
static uint16_t g_blkcnt = 0;
static uint16_t g_blks_received = 0;

/* last status readable */
static char g_last_status[64] = "WAITING";

static uint8_t g_own_addr_type = BLE_OWN_ADDR_PUBLIC;

static void prov_reset_assembly(void)
{
    g_total_len = 0;
    g_session_id = 0;
    g_have_total = false;
    g_blks_received = 0;
    if (g_blkmap && g_blkcnt)
        memset(g_blkmap, 0, g_blkcnt);
}

static bool prov_is_complete(void)
{
    if (!g_have_total || g_total_len == 0)
        return false;
    uint16_t need = (g_total_len + 15) / 16;
    return (g_blks_received >= need);
}

static void prov_mark_block(uint16_t blk)
{
    if (blk >= g_blkcnt)
        return;
    if (g_blkmap[blk] == 0)
    {
        g_blkmap[blk] = 1;
        g_blks_received++;
    }
}

static void status_set_last(const char *s)
{
    snprintf(g_last_status, sizeof(g_last_status), "%s", s);
}

static void status_notify_str(const char *s)
{
    status_set_last(s);

    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE || g_status_val_handle == 0)
        return;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(s, strlen(s));
    if (!om)
        return;

    int rc = ble_gattc_notify_custom(g_conn_handle, g_status_val_handle, om);
    if (rc != 0)
        ESP_LOGW(TAG, "notify rc=%d", rc);
}

/* ======== JSON parse ======== */
static esp_err_t parse_json_cfg(const uint8_t *buf, uint16_t len, prov_session_cfg_t *out)
{
    memset(out, 0, sizeof(*out));

    char *json = (char *)malloc(len + 1);
    if (!json)
        return ESP_ERR_NO_MEM;
    memcpy(json, buf, len);
    json[len] = '\0';

    cJSON *root = cJSON_Parse(json);
    free(json);
    if (!root)
        return ESP_ERR_INVALID_ARG;

    cJSON *wifi = cJSON_GetObjectItem(root, "wifi");
    cJSON *host = cJSON_GetObjectItem(root, "host");
    cJSON *time = cJSON_GetObjectItem(root, "time");
    if (!cJSON_IsObject(wifi) || !cJSON_IsObject(host) || !cJSON_IsObject(time))
    {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *ssid = cJSON_GetObjectItem(wifi, "ssid");
    cJSON *pass = cJSON_GetObjectItem(wifi, "pass");
    cJSON *hip = cJSON_GetObjectItem(host, "ip");
    cJSON *ports = cJSON_GetObjectItem(host, "ports");
    cJSON *ums = cJSON_GetObjectItem(time, "unix_ms");

    if (!cJSON_IsString(ssid) || !cJSON_IsString(pass) || !cJSON_IsString(hip) ||
        !cJSON_IsObject(ports) || (!cJSON_IsNumber(ums) && !cJSON_IsString(ums)))
    {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(out->ssid, sizeof(out->ssid), "%s", ssid->valuestring);
    snprintf(out->pass, sizeof(out->pass), "%s", pass->valuestring);
    snprintf(out->host_ip, sizeof(out->host_ip), "%s", hip->valuestring);

    cJSON *p_video = cJSON_GetObjectItem(ports, "video");
    cJSON *p_tele = cJSON_GetObjectItem(ports, "tele");
    cJSON *p_ws = cJSON_GetObjectItem(ports, "ws");
    if (!cJSON_IsNumber(p_video) || !cJSON_IsNumber(p_tele) || !cJSON_IsNumber(p_ws))
    {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    out->port_video = (uint16_t)p_video->valuedouble;
    out->port_tele = (uint16_t)p_tele->valuedouble;
    out->port_ws = (uint16_t)p_ws->valuedouble;

    out->unix_ms_utc = cJSON_IsNumber(ums) ? (int64_t)ums->valuedouble
                                           : (int64_t)strtoll(ums->valuestring, NULL, 10);

    cJSON_Delete(root);
    return ESP_OK;
}

/* ======== GATT Access ======== */
static int gatt_access_info(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    const char *info = "{\"v\":1,\"need\":[\"wifi\",\"host\",\"time\"]}";
    os_mbuf_append(ctxt->om, info, strlen(info));
    return 0;
}

/* STATUS: READ handler (returns last status string) */
static int gatt_access_status(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    os_mbuf_append(ctxt->om, g_last_status, strlen(g_last_status));
    return 0;
}

static int gatt_access_cfg_rx(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 8)
    {
        status_notify_str("ERROR:CFG_TOO_SHORT");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t hdr[8];
    int rc = ble_hs_mbuf_to_flat(ctxt->om, hdr, sizeof(hdr), NULL);
    if (rc != 0)
        return BLE_ATT_ERR_UNLIKELY;

    uint32_t sid = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
    uint16_t off = (uint16_t)hdr[4] | ((uint16_t)hdr[5] << 8);
    uint16_t total = (uint16_t)hdr[6] | ((uint16_t)hdr[7] << 8);

    if (total == 0 || total > g_buf_max)
    {
        status_notify_str("ERROR:CFG_TOTAL_BAD");
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (off >= total)
    {
        status_notify_str("ERROR:CFG_OFF_BAD");
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (!g_have_total || g_session_id != sid || g_total_len != total)
    {
        prov_reset_assembly();
        g_session_id = sid;
        g_total_len = total;
        g_have_total = true;
        status_notify_str("WAITING");
    }

    uint16_t payload_len = len - 8;
    if ((uint32_t)off + payload_len > g_total_len)
    {
        status_notify_str("ERROR:CFG_OVERFLOW");
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint8_t *flat = (uint8_t *)malloc(len);
    if (!flat)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    rc = ble_hs_mbuf_to_flat(ctxt->om, flat, len, NULL);
    if (rc != 0)
    {
        free(flat);
        return BLE_ATT_ERR_UNLIKELY;
    }

    memcpy(g_buf + off, flat + 8, payload_len);
    free(flat);

    uint16_t b0 = off / 16;
    uint16_t b1 = (off + payload_len - 1) / 16;
    for (uint16_t b = b0; b <= b1; b++)
        prov_mark_block(b);

    if (prov_is_complete())
        status_notify_str("RECEIVED");
    return 0;
}

static int gatt_access_cmd(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    uint8_t cmd = 0;
    if (OS_MBUF_PKTLEN(ctxt->om) < 1)
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, &cmd, 1, NULL);
    if (rc != 0)
        return BLE_ATT_ERR_UNLIKELY;

    if (cmd == PROV_CMD_ABORT)
    {
        prov_reset_assembly();
        status_notify_str("ABORTED");
        return 0;
    }
    if (cmd != PROV_CMD_COMMIT)
    {
        status_notify_str("ERROR:CMD_UNKNOWN");
        return 0;
    }

    if (!prov_is_complete())
    {
        status_notify_str("ERROR:CFG_INCOMPLETE");
        return 0;
    }

    status_notify_str("APPLYING");

    prov_session_cfg_t cfg;
    esp_err_t err = parse_json_cfg(g_buf, g_total_len, &cfg);
    if (err != ESP_OK)
    {
        status_notify_str("ERROR:JSON_BAD");
        return 0;
    }

    err = g_cfg.apply_cb(&cfg);
    if (err != ESP_OK)
    {
        status_notify_str("ERROR:APPLY_FAIL");
        return 0;
    }

    status_notify_str("DONE");
    prov_ble_stop();
    return 0;
}

/* ======== GATT DB ======== */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_PROV_SVC.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &UUID_INFO_CHR.u,
                .access_cb = gatt_access_info,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = &UUID_CFG_RX_CHR.u,
                .access_cb = gatt_access_cfg_rx,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &UUID_STATUS_TX_CHR.u,
                .access_cb = gatt_access_status,
                .val_handle = &g_status_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &UUID_CMD_CHR.u,
                .access_cb = gatt_access_cmd,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {0}},
    },
    {0}};

/* ======== GAP / Advertising ======== */
static void advertise_start(void);

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            g_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected, handle=%d", g_conn_handle);
            status_notify_str("CONNECTED");
        }
        else
        {
            ESP_LOGW(TAG, "Connect failed; status=%d", event->connect.status);
            advertise_start();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        advertise_start();
        return 0;

    default:
        return 0;
    }
}

static void make_adv_name(char *out, size_t out_sz, const char *prefix)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    snprintf(out, out_sz, "%s%02X%02X", prefix ? prefix : "Train-", mac[4], mac[5]);
}

static void advertise_start(void)
{
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* adv data: only flags + name (NO UUIDs) */
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    char name[32];
    make_adv_name(name, sizeof(name), g_cfg.adv_name_prefix);
    ble_svc_gap_device_name_set(name);

    fields.name = (uint8_t *)name;
    fields.name_len = (uint8_t)strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
        return;
    }

    rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "Advertising as %s", name);
        status_set_last("WAITING");
    }
}

static void on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE sync");
    int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    ESP_LOGI(TAG, "ble_hs_id_infer_auto rc=%d own_addr_type=%u", rc, g_own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "id_infer failed rc=%d", rc);
        return;
    }
    advertise_start();
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ======== Public API ======== */
esp_err_t prov_ble_start(const prov_ble_cfg_t *cfg)
{
    if (!cfg || !cfg->apply_cb)
        return ESP_ERR_INVALID_ARG;

    g_cfg = *cfg;
    g_buf_max = (cfg->max_cfg_len == 0) ? 2048 : cfg->max_cfg_len;

    g_buf = (uint8_t *)malloc(g_buf_max);
    if (!g_buf)
        return ESP_ERR_NO_MEM;

    g_blkcnt = (g_buf_max + 15) / 16;
    g_blkmap = (uint8_t *)malloc(g_blkcnt);
    if (!g_blkmap)
    {
        free(g_buf);
        g_buf = NULL;
        return ESP_ERR_NO_MEM;
    }

    prov_reset_assembly();
    status_set_last("WAITING");

    int rc = nimble_port_init();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "nimble_port_init rc=%d", rc);
        return ESP_FAIL;
    }

    ble_hs_cfg.sync_cb = on_sync;

    /* makes host store sane */
    ble_store_config_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svcs);
    ESP_LOGI(TAG, "ble_gatts_count_cfg rc=%d", rc);
    if (rc != 0)
        return ESP_ERR_INVALID_STATE;

    rc = ble_gatts_add_svcs(gatt_svcs);
    ESP_LOGI(TAG, "ble_gatts_add_svcs rc=%d", rc);
    if (rc != 0)
        return ESP_ERR_INVALID_STATE;

    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "prov_ble started (max_cfg_len=%u)", g_buf_max);
    return ESP_OK;
}

void prov_ble_stop(void)
{
    if (g_conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        ble_gap_terminate(g_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    }

    ble_gap_adv_stop();
    nimble_port_stop();

    if (g_blkmap)
    {
        free(g_blkmap);
        g_blkmap = NULL;
    }
    if (g_buf)
    {
        free(g_buf);
        g_buf = NULL;
    }

    g_status_val_handle = 0;
    g_buf_max = 0;
    g_blkcnt = 0;

    ESP_LOGI(TAG, "prov_ble stopped");
}