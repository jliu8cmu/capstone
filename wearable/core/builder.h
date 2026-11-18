#pragma once

#include <stdint.h>
#include "packet.h"
#include "ringbuffer.h"

#define SAMPLE_PERIOD_US 9615u

typedef struct {
    uint16_t next_seq;
    uint32_t last_overruns;
} builder_t;

void builder_init(builder_t *b);

void builder_build(builder_t *b, ringbuf_t *rb, uint32_t now_ms, uint8_t batt, uint8_t extra_flags, packet_t *pkt);



