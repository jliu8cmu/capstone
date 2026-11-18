#include "builder.h"
#include <string.h>

void builder_init(builder_t *b) {
    b->next_seq = 0;
    b->last_overruns = 0;

}

void builder_build(builder_t *b, ringbuf_t *rb, uint32_t now_ms, uint8_t batt, uint8_t extra_flags, packet_t *pkt) {
    memset(pkt, 0, sizeof *pkt);
    pkt->version = PKT_VERSION;

    pkt->seq = b->next_seq++;

    uint32_t n = rb_pop_up_to(rb, pkt->samples, PKT_MAX_SAMPLES);

    pkt->n = (uint8_t)(n);

    uint32_t back_ms = (n * SAMPLE_PERIOD_US + 500u ) / 1000u;
    pkt-> t0_ms = now_ms - back_ms;

    uint8_t flags = extra_flags;

    uint32_t ov = rb_overruns(rb);
    
    if (ov != b->last_overruns) {
        flags |= PKT_FLAG_OVERRUN;
        b->last_overruns = ov;
    }

    pkt->flags = flags;
    pkt->batt = batt;
}
