#include <stdio.h>
#include <stdint.h>
#include "../core/crc16.h"
#include "../core/packet.h"
#include "../core/ringbuffer.h"
#include <string.h>
#include <builder.h>

static int failures = 0;



#define CHECK(cond) do {\
if(!(cond)) {\
    failures++;\
    printf("%s:%d: FAILED: %s\n", __FILE__, __LINE__, #cond);\
}} while(0) 

static void test_crc16(void)
{
    /* 1. Official check value for CRC-16/CCITT-FALSE */
    const uint8_t *digits = (const uint8_t *)"123456789";
    CHECK(crc16_ccitt(digits, 9) == 0x29B1);

    /* 2. Empty input returns the starting value unchanged */
    CHECK(crc16_ccitt(digits, 0) == 0xFFFF);

    /* 3. Single byte 'A' */
    const uint8_t a[] = { 0x41 };
    CHECK(crc16_ccitt(a, 1) == 0xB915);

    /* 4. Single zero byte */
    const uint8_t zero[] = { 0x00 };
    CHECK(crc16_ccitt(zero, 1) == 0xE1F0);
}

/* ---- helpers used only by test_packet ---- */

/* Fill a packet with known, varied values (including int16 extremes). */
static void make_test_packet(packet_t *p, uint8_t n)
{
    memset(p, 0, sizeof *p);
    p->version = PKT_VERSION;
    p->seq     = 0xBEEF;
    p->t0_ms   = 0x87654321;          /* top bit set: exercises the 32-bit shift fix */
    p->n       = n;
    p->batt    = 0xC8;
    p->flags   = PKT_FLAG_OVERRUN;
    for (uint8_t i = 0; i < n; i++) {
        imu_sample_t *s = &p->samples[i];
        s->ax = 32767;                /* max int16 */
        s->ay = -32768;               /* min int16 */
        s->az = (int16_t)(1000 * i - 1);
        s->gx = (int16_t)(-7 * (i + 1));
        s->gy = (int16_t)(i * 1234);
        s->gz = -1;                   /* all-ones bit pattern */
    }
}

/* Field-by-field comparison; only the first n samples are meaningful. */
static int packets_equal(const packet_t *a, const packet_t *b)
{
    if (a->version != b->version || a->seq != b->seq || a->t0_ms != b->t0_ms ||
        a->n != b->n || a->batt != b->batt || a->flags != b->flags)
        return 0;
    for (uint8_t i = 0; i < a->n; i++) {
        const imu_sample_t *x = &a->samples[i], *y = &b->samples[i];
        if (x->ax != y->ax || x->ay != y->ay || x->az != y->az ||
            x->gx != y->gx || x->gy != y->gy || x->gz != y->gz)
            return 0;
    }
    return 1;
}

static void test_packet(void)
{
    packet_t in, out;
    uint8_t  buf[PKT_MAX_LEN];

    /* 0. Constant arithmetic */
    CHECK(PKT_MAX_LEN == 84);

    /* 1. Round trip for several sample counts */
    const uint8_t counts[] = { 0, 1, 5, PKT_MAX_SAMPLES };
    for (size_t k = 0; k < sizeof counts; k++) {
        uint8_t n = counts[k];
        make_test_packet(&in, n);
        int len = packet_encode(&in, buf, sizeof buf);
        CHECK(len == (int)(PKT_HEADER_LEN + n * PKT_SAMPLE_LEN + PKT_TRAILER_LEN));

        memset(&out, 0, sizeof out);
        CHECK(packet_decode(buf, (size_t)len, &out) == 0);
        CHECK(packets_equal(&in, &out));
    }

    /* 2. Byte order is little-endian (the Pi parser relies on this) */
    make_test_packet(&in, 0);
    in.seq = 0x0102;
    CHECK(packet_encode(&in, buf, sizeof buf) > 0);
    CHECK(buf[0] == PKT_VERSION);
    CHECK(buf[1] == 0x02 && buf[2] == 0x01);                       /* seq   */
    CHECK(buf[3] == 0x21 && buf[4] == 0x43 &&
          buf[5] == 0x65 && buf[6] == 0x87);                       /* t0_ms */

    /* 3. Any single flipped bit is caught by the CRC */
    make_test_packet(&in, 5);
    int len = packet_encode(&in, buf, sizeof buf);
    for (int byte = 0; byte < len; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            buf[byte] ^= (uint8_t)(1u << bit);
            CHECK(packet_decode(buf, (size_t)len, &out) == PKT_ERR_CRC);
            buf[byte] ^= (uint8_t)(1u << bit);                     /* undo */
        }
    }
    CHECK(packet_decode(buf, (size_t)len, &out) == 0);             /* intact again */

    /* 4. Buffer one byte too small: error, and nothing written past the limit */
    uint8_t big[PKT_MAX_LEN + 8];
    memset(big, 0xAA, sizeof big);
    size_t need = (size_t)len;
    CHECK(packet_encode(&in, big, need - 1) == PKT_ERR_BUF_SMALL);
    for (size_t i = need - 1; i < sizeof big; i++)
        CHECK(big[i] == 0xAA);

    /* 5. Invalid inputs */
    in.n = PKT_MAX_SAMPLES + 1;
    CHECK(packet_encode(&in, buf, sizeof buf) == PKT_ERR_FORMAT);
    CHECK(packet_decode(buf, 3, &out) == PKT_ERR_FORMAT);           /* too short */

    /* 6. Truncated packet (valid bytes, wrong length) is rejected */
    make_test_packet(&in, 5);
    len = packet_encode(&in, buf, sizeof buf);
    CHECK(packet_decode(buf, (size_t)len - 1, &out) != 0);
}

/* Make a sample whose fields all encode one number, so order is easy to check. */
static imu_sample_t mk(int v)
{
    imu_sample_t s = { (int16_t)v, (int16_t)(v + 1), (int16_t)(v + 2),
                       (int16_t)-v, (int16_t)(-v - 1), (int16_t)(-v - 2) };
    return s;
}

static int same(const imu_sample_t *a, const imu_sample_t *b)
{
    return a->ax == b->ax && a->ay == b->ay && a->az == b->az &&
           a->gx == b->gx && a->gy == b->gy && a->gz == b->gz;
}

static void test_ringbuf(void)
{
    ringbuf_t rb;
    imu_sample_t out[RB_SIZE + 8];
    imu_sample_t s;

    /* 1. New buffer is empty */
    rb_init(&rb);
    CHECK(rb_count(&rb) == 0);
    CHECK(rb_overruns(&rb) == 0);
    CHECK(rb_pop_up_to(&rb, out, 10) == 0);

    /* 2. FIFO order, count tracks pushes */
    for (int i = 0; i < 5; i++) { s = mk(i); CHECK(rb_push(&rb, &s)); }
    CHECK(rb_count(&rb) == 5);
    CHECK(rb_pop_up_to(&rb, out, 10) == 5);
    for (int i = 0; i < 5; i++) { s = mk(i); CHECK(same(&out[i], &s)); }
    CHECK(rb_count(&rb) == 0);

    /* 3. max is respected; the rest stays for the next pop */
    for (int i = 0; i < 5; i++) { s = mk(100 + i); rb_push(&rb, &s); }
    CHECK(rb_pop_up_to(&rb, out, 3) == 3);
    CHECK(rb_count(&rb) == 2);
    s = mk(102); CHECK(same(&out[2], &s));
    CHECK(rb_pop_up_to(&rb, out, 3) == 2);
    s = mk(103); CHECK(same(&out[0], &s));
    s = mk(104); CHECK(same(&out[1], &s));

    /* 4. Full: oldest 32 kept, newest 8 dropped and counted */
    rb_init(&rb);
    for (int i = 0; i < 40; i++) {
        s = mk(i);
        bool ok = rb_push(&rb, &s);
        CHECK(ok == (i < (int)RB_SIZE));
    }
    CHECK(rb_count(&rb) == RB_SIZE);
    CHECK(rb_overruns(&rb) == 8);
    CHECK(rb_pop_up_to(&rb, out, RB_SIZE + 8) == RB_SIZE);
    for (int i = 0; i < (int)RB_SIZE; i++) { s = mk(i); CHECK(same(&out[i], &s)); }
    CHECK(rb_overruns(&rb) == 8);                 /* popping never resets it */

    /* 5. Counters wrapping past UINT32_MAX */
    rb_init(&rb);
    atomic_store(&rb.head, UINT32_MAX - 3);
    atomic_store(&rb.tail, UINT32_MAX - 3);
    for (int i = 0; i < 10; i++) { s = mk(200 + i); CHECK(rb_push(&rb, &s)); }
    CHECK(rb_count(&rb) == 10);
    CHECK(atomic_load(&rb.head) == 6);            /* (UINT32_MAX - 3) + 10 wrapped */
    CHECK(rb_pop_up_to(&rb, out, 10) == 10);
    for (int i = 0; i < 10; i++) { s = mk(200 + i); CHECK(same(&out[i], &s)); }
    CHECK(rb_count(&rb) == 0);

    /* 6. Interleaved push 3 / pop 2: every value exactly once, in order */
    rb_init(&rb);
    int next_in = 0, next_out = 0;
    for (int round = 0; round < 100; round++) {
        for (int k = 0; k < 3; k++) { s = mk(next_in); if (rb_push(&rb, &s)) next_in++; }
        uint32_t n = rb_pop_up_to(&rb, out, 2);
        for (uint32_t k = 0; k < n; k++) { s = mk(next_out++); CHECK(same(&out[k], &s)); }
        CHECK(rb_count(&rb) <= RB_SIZE);
    }
    uint32_t n = rb_pop_up_to(&rb, out, RB_SIZE);  /* drain the rest */
    for (uint32_t k = 0; k < n; k++) { s = mk(next_out++); CHECK(same(&out[k], &s)); }
    CHECK(next_out == next_in);
    CHECK(rb_overruns(&rb) > 0);                  /* 3 in / 2 out must eventually overflow */
}

/* Uses mk() and same() from the ring buffer tests. */
static void push_n(ringbuf_t *rb, int first, int count)
{
    for (int i = 0; i < count; i++) {
        imu_sample_t s = mk(first + i);
        rb_push(rb, &s);
    }
}

static void test_builder(void)
{
    builder_t b;
    ringbuf_t rb;
    packet_t  p, q;
    uint8_t   buf[PKT_MAX_LEN];
    imu_sample_t s;

    /* 1. Empty buffer -> valid packet with n = 0 */
    builder_init(&b); rb_init(&rb);
    builder_build(&b, &rb, 1000, 200, 0, &p);
    CHECK(p.version == PKT_VERSION);
    CHECK(p.n == 0);
    CHECK(p.seq == 0);
    CHECK(p.t0_ms == 1000);
    CHECK(p.flags == 0);
    CHECK(p.batt == 200);
    CHECK(packet_encode(&p, buf, sizeof buf) == PKT_HEADER_LEN + PKT_TRAILER_LEN);

    /* 2. Sequence numbers count up */
    builder_build(&b, &rb, 1000, 0, 0, &p); CHECK(p.seq == 1);
    builder_build(&b, &rb, 1000, 0, 0, &p); CHECK(p.seq == 2);

    /* 3. Sequence wraps 65535 -> 0 */
    b.next_seq = 0xFFFF;
    builder_build(&b, &rb, 1000, 0, 0, &p); CHECK(p.seq == 0xFFFF);
    builder_build(&b, &rb, 1000, 0, 0, &p); CHECK(p.seq == 0);

    /* 4. At most PKT_MAX_SAMPLES per packet, order kept across packets */
    builder_init(&b); rb_init(&rb);
    push_n(&rb, 0, 10);
    builder_build(&b, &rb, 1000, 0, 0, &p);
    CHECK(p.n == PKT_MAX_SAMPLES);
    for (int i = 0; i < PKT_MAX_SAMPLES; i++) { s = mk(i); CHECK(same(&p.samples[i], &s)); }
    builder_build(&b, &rb, 1050, 0, 0, &p);
    CHECK(p.n == 10 - PKT_MAX_SAMPLES);
    for (int i = 0; i < p.n; i++) { s = mk(PKT_MAX_SAMPLES + i); CHECK(same(&p.samples[i], &s)); }
    builder_build(&b, &rb, 1100, 0, 0, &p);
    CHECK(p.n == 0);

    /* 5. Timestamp = now - n * 9.615 ms, rounded */
    builder_init(&b); rb_init(&rb);
    push_n(&rb, 0, 5);
    builder_build(&b, &rb, 1000, 0, 0, &p);
    CHECK(p.t0_ms == 1000 - 48);                    /* 5 * 9615 us = 48.075 ms */
    push_n(&rb, 0, 6);
    builder_build(&b, &rb, 1000, 0, 0, &p);
    CHECK(p.t0_ms == 1000 - 58);                    /* 6 * 9615 us = 57.69 ms */

    /* 6. Timestamp near zero wraps instead of misbehaving */
    push_n(&rb, 0, 5);
    builder_build(&b, &rb, 10, 0, 0, &p);
    CHECK(p.t0_ms == (uint32_t)(10u - 48u));

    /* 7. Overrun flag appears once per new overflow */
    builder_init(&b); rb_init(&rb);
    push_n(&rb, 0, RB_SIZE + 8);                    /* 8 overruns */
    builder_build(&b, &rb, 1000, 0, 0, &p);
    CHECK(p.flags & PKT_FLAG_OVERRUN);
    builder_build(&b, &rb, 1050, 0, 0, &p);
    CHECK(!(p.flags & PKT_FLAG_OVERRUN));           /* cleared next time */
    while (rb_count(&rb) > 0) builder_build(&b, &rb, 1100, 0, 0, &p);   /* drain */
    push_n(&rb, 0, RB_SIZE + 1);                    /* 1 more overrun */
    builder_build(&b, &rb, 1150, 0, 0, &p);
    CHECK(p.flags & PKT_FLAG_OVERRUN);

    /* 8. Platform flags pass through untouched */
    builder_init(&b); rb_init(&rb);
    builder_build(&b, &rb, 1000, 0, PKT_FLAG_IMU_ERR | PKT_FLAG_LOW_BATT, &p);
    CHECK(p.flags & PKT_FLAG_IMU_ERR);
    CHECK(p.flags & PKT_FLAG_LOW_BATT);
    CHECK(!(p.flags & PKT_FLAG_OVERRUN));

    /* 9. Full pipeline: push -> build -> encode -> decode */
    builder_init(&b); rb_init(&rb);
    b.next_seq = 1234;
    push_n(&rb, 500, 5);
    builder_build(&b, &rb, 777777, 0xC8, PKT_FLAG_LOW_BATT, &p);
    int len = packet_encode(&p, buf, sizeof buf);
    CHECK(len == PKT_HEADER_LEN + 5 * PKT_SAMPLE_LEN + PKT_TRAILER_LEN);
    memset(&q, 0, sizeof q);
    CHECK(packet_decode(buf, (size_t)len, &q) == 0);
    CHECK(q.seq == 1234);
    CHECK(q.t0_ms == 777777 - 48);
    CHECK(q.batt == 0xC8);
    CHECK(q.flags == PKT_FLAG_LOW_BATT);
    CHECK(q.n == 5);
    for (int i = 0; i < 5; i++) { s = mk(500 + i); CHECK(same(&q.samples[i], &s)); }
}

int main(void) {
    test_crc16();
    test_packet();
    test_ringbuf();
    test_builder();

    if(failures==0) {
        printf("all test passed\n");
    } else {
        printf("failures: %d", failures);
    }

    return failures == 0 ? 0 : 1;
}

