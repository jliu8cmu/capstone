#pragma once

#include <stdint.h> 
#include <stddef.h>

#define PKT_VERSION 1
#define PKT_MAX_SAMPLES 6
#define PKT_HEADER_LEN 8
#define PKT_SAMPLE_LEN 12
#define PKT_TRAILER_LEN 4
#define PKT_MAX_LEN 84

#define PKT_FLAG_IMU_ERR (1u << 0)
#define PKT_FLAG_OVERRUN (1u << 1)
#define PKT_FLAG_LOW_BATT (1u<<2)

#define PKT_ERR_BUF_SMALL (-1)
#define PKT_ERR_CRC (-2)
#define PKT_ERR_FORMAT (-3)

typedef struct {
    int16_t ax, ay, az, gx, gy, gz;
} imu_sample_t;

typedef struct {
    uint8_t version;
    uint16_t seq;
    uint32_t t0_ms;
    uint8_t n;
    imu_sample_t samples[PKT_MAX_SAMPLES];
    uint8_t batt;
    uint8_t flags;
} packet_t;

int packet_encode(const packet_t *pkt, uint8_t *buf, size_t buf_size);

int packet_decode(const uint8_t *buf, size_t len, packet_t *pkt);