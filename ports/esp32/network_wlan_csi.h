/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Francesco Pace <francesco.pace@gmail.com>
 * Copyright (c) 2025 MicroPython CSI Module Contributors
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MICROPY_INCLUDED_ESP32_NETWORK_WLAN_CSI_H
#define MICROPY_INCLUDED_ESP32_NETWORK_WLAN_CSI_H

#include "py/obj.h"

#if MICROPY_PY_NETWORK_WLAN_CSI

#include "py/ringbuf.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"

#define CSI_MAX_DATA_LEN (512)

typedef struct {
    uint32_t timestamp_us;
    uint32_t local_timestamp;
    uint16_t len;
    uint16_t sig_len;
    int8_t rssi;
    uint8_t rate;
    int8_t noise_floor;
    uint8_t channel;
    uint8_t ampdu_cnt;
    uint8_t rx_state;
    uint8_t sig_mode : 2;
    uint8_t mcs : 5;
    uint8_t cwb : 1;
    uint8_t smoothing : 1;
    uint8_t not_sounding : 1;
    uint8_t aggregation : 1;
    uint8_t stbc : 2;
    uint8_t fec_coding : 1;
    uint8_t sgi : 1;
    uint8_t secondary_channel : 2;
    uint8_t ant : 2;
    uint8_t _reserved : 1;
    uint8_t agc_gain;
    int8_t fft_gain;
    uint8_t mac[6];
    int8_t data[CSI_MAX_DATA_LEN];
} csi_frame_t;

// ringbuf_t uses uint16_t for the byte size, so keep the Python-visible limit
// within the maximum addressable ringbuffer capacity.
#define CSI_MAX_BUFFER_SIZE ((UINT16_MAX - 1) / sizeof(csi_frame_t))

typedef struct {
    ringbuf_t ringbuffer;
    uint16_t buffer_size;
    volatile uint32_t dropped;
} csi_state_t;

void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info);
void wifi_csi_deinit(void);

extern const mp_obj_fun_builtin_var_t network_wlan_csi_enable_obj;
extern const mp_obj_fun_builtin_fixed_t network_wlan_csi_disable_obj;
extern const mp_obj_fun_builtin_var_t network_wlan_csi_read_obj;
extern const mp_obj_fun_builtin_fixed_t network_wlan_csi_dropped_obj;
extern const mp_obj_fun_builtin_fixed_t network_wlan_csi_available_obj;
extern const mp_obj_fun_builtin_fixed_t network_wlan_csi_force_gain_obj;
extern const mp_obj_fun_builtin_fixed_t network_wlan_csi_gain_lock_supported_obj;

#endif // MICROPY_PY_NETWORK_WLAN_CSI

#endif // MICROPY_INCLUDED_ESP32_NETWORK_WLAN_CSI_H
