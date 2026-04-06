#include "wifi_sweep.h"

#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/wifi/webInterface.h"
#include "core/wifi/wifi_common.h"

#include <globals.h>

#include <esp_wifi.h>
#include <nvs_flash.h>

// Simple promiscuous RSSI “scope”:
// - X axis: time (scrolling columns)
// - Y axis: average RSSI during a short sampling window
// - Channel selectable: 1–14 via Prev/Next

namespace {

static volatile int32_t g_rssiSum = 0;
static volatile uint32_t g_rssiCount = 0;

static void IRAM_ATTR wifiSweepPromiscuousCb(void *buf, wifi_promiscuous_pkt_type_t type) {
    (void)type;
    if (buf == nullptr) return;
    const wifi_promiscuous_pkt_t *pkt = static_cast<const wifi_promiscuous_pkt_t *>(buf);
    // rx_ctrl.rssi is int8_t
    int rssi = pkt->rx_ctrl.rssi;
    // Keep it tiny and fast in ISR context
    g_rssiSum += rssi;
    g_rssiCount += 1;
}

static int clampInt(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int avgRssiToHeight(int avgRssi, int graphH) {
    // Map typical Wi-Fi RSSI range (-100..-30 dBm) into 0..graphH
    // weaker -> shorter bar
    const int rssiMin = -100;
    const int rssiMax = -30;
    avgRssi = clampInt(avgRssi, rssiMin, rssiMax);
    long h = map(avgRssi, rssiMin, rssiMax, 0, graphH);
    return clampInt((int)h, 0, graphH);
}

static void stopWebUiIfRunning() {
    if (isWebUIActive || server) {
        stopWebUi();
        // Some existing features also call wifiDisconnect() when stopping WebUI.
        // We keep that behavior to avoid mode conflicts.
        wifiDisconnect();
    }
}

} // namespace

void wifi_sweep() {
    stopWebUiIfRunning();

    // UI layout
    const int screenW = tft.width();
    const int screenH = tft.height();
    const int topH = screenH / 5;           // header area similar to rf_waterfall
    const int graphTop = topH + 2;
    const int graphBottom = screenH - 20;   // leave a small footer
    const int graphH = graphBottom - graphTop;

    // Sampling
    const uint32_t sampleWindowMs = 100; // average RSSI over this window

    // Channel state
    int channel = 1;

    // Clear & banner
    tft.fillScreen(bruceConfig.bgColor);
    drawMainBorderWithTitle("WiFi Sweep");

    // ---- Wi-Fi promiscuous init ----
    ensureWifiPlatform();

    // NVS init pattern copied from sniffer
    nvs_flash_init();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // We use AP mode like existing sniffer/probe sniffer does, to keep driver running
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Promiscuous filter: accept mgmt+data (broad)
    const wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA
    };
    esp_wifi_set_promiscuous_filter(&filter);

    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(nullptr);

    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(wifiSweepPromiscuousCb);

    // Graph background
    tft.fillRect(0, graphTop, screenW, graphH + 1, bruceConfig.bgColor);

    int x = 0;
    uint32_t lastSampleMs = millis();

    // Prime keys
    check(SelPress);
    check(PrevPress);
    check(NextPress);
    check(EscPress);

    while (1) {
        if (check(EscPress)) break;

        // Channel controls
        if (check(NextPress) || check(UpPress)) {
            channel++;
            if (channel > 14) channel = 1;
            ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
        }
        if (check(PrevPress) || check(DownPress)) {
            channel--;
            if (channel < 1) channel = 14;
            ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
        }

        uint32_t now = millis();
        if (now - lastSampleMs >= sampleWindowMs) {
            // Snapshot counters (best-effort, races are fine for a scope)
            int32_t sum = g_rssiSum;
            uint32_t cnt = g_rssiCount;
            g_rssiSum = 0;
            g_rssiCount = 0;

            int avg = (cnt == 0) ? -100 : (int)(sum / (int32_t)cnt);
            int h = avgRssiToHeight(avg, graphH);

            // Scroll: draw one column at x
            // First clear that column
            tft.drawFastVLine(x, graphTop, graphH, bruceConfig.bgColor);

            // Draw bar from bottom up
            int y0 = graphTop + graphH;
            int y1 = y0 - h;
            uint16_t barColor = bruceConfig.priColor;
            tft.drawFastVLine(x, y1, h, barColor);

            // Header refresh (small, minimal redraw)
            tft.fillRect(0, 0, screenW, topH, bruceConfig.bgColor);
            tft.setCursor(3, 3);
            tft.setTextSize(1);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.print("WiFi Sweep  Ch:");
            tft.print(channel);
            tft.print("  avg:");
            tft.print(avg);
            tft.print(" dBm  pkts:");
            tft.print((int)cnt);

            // Footer hint
            tft.fillRect(0, screenH - 18, screenW, 18, bruceConfig.bgColor);
            tft.setCursor(3, screenH - 16);
            tft.setTextSize(1);
            tft.setTextColor(TFT_DARKCYAN, bruceConfig.bgColor);
            tft.print("[PREV/NEXT] Channel  [ESC] Exit");

            x++;
            if (x >= screenW) x = 0;

            lastSampleMs = now;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // ---- Cleanup ----
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(nullptr);
    esp_wifi_stop();

    // IMPORTANT: other modules sometimes call esp_wifi_deinit() to allow clean restart.
    // Do it here too to avoid leaving the driver in a half-initialized state.
    esp_wifi_deinit();

    wifiDisconnect();
    returnToMenu = true;
    vTaskDelay(1 / portTICK_PERIOD_MS);
}
