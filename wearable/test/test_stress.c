/*
 * test_stress.c - simulated workloads for the wearable core.
 *
 *   A. Workload simulation (single thread, simulated time):
 *      104 Hz IMU with timing jitter, 20 Hz packet builder that is sometimes
 *      late (BLE busy). Every packet is encoded and decoded; checks that every
 *      sample arrives exactly once, in order, or is counted as an overrun.
 *
 *   B. Concurrency stress (two real threads):
 *      producer thread = IMU interrupt, consumer thread = main loop, millions of
 *      samples as fast as possible. Build with -fsanitize=thread to detect data races.
 *
 * Written with assistance from Claude (Anthropic).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "../core/packet.h"
#include "../core/ringbuffer.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { failures++; \
    printf("%s:%d: FAILED: %s\n", __FILE__, __LINE__, #c); } } while (0)

/* A 32-bit sample number stored in ax (low 16 bits) and ay (high 16 bits). */
static imu_sample_t tag(uint32_t seq)
{
    imu_sample_t s = { (int16_t)(seq & 0xFFFF), (int16_t)(seq >> 16), 0, 0, 0, 0 };
    return s;
}
static uint32_t untag(const imu_sample_t *s)
{
    return (uint16_t)s->ax | ((uint32_t)(uint16_t)s->ay << 16);
}

/* Small deterministic random generator so failures are reproducible. */
static uint32_t rng = 12345;
static uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng >> 8; }

/* ---------------- A. workload simulation ---------------- */
static void sim_workload(double seconds, double late_prob, double late_ms)
{
    ringbuf_t rb;
    rb_init(&rb);

    const double imu_dt = 1000.0 / 104.0;          /* ms */
    double t_imu = 0, t_pkt = 50.0;
    uint32_t produced = 0, received = 0, packets = 0, expect = 0, max_fill = 0;
    uint16_t seq = 0;
    uint8_t buf[PKT_MAX_LEN];

    while (t_imu < seconds * 1000.0 || t_pkt < seconds * 1000.0) {
        if (t_imu <= t_pkt) {                       /* next event: IMU sample */
            imu_sample_t s = tag(produced++);
            rb_push(&rb, &s);
            uint32_t c = rb_count(&rb);
            if (c > max_fill) max_fill = c;
            t_imu += imu_dt + ((int)(rnd() % 1001) - 500) / 1000.0 * 0.5;   /* +/-0.25 ms jitter */
        } else {                                    /* next event: build a packet */
            packet_t p, q;
            memset(&p, 0, sizeof p);
            p.version = PKT_VERSION;
            p.seq = seq++;
            p.t0_ms = (uint32_t)t_pkt;
            p.n = (uint8_t)rb_pop_up_to(&rb, p.samples, PKT_MAX_SAMPLES);
            int len = packet_encode(&p, buf, sizeof buf);
            CHECK(len > 0);
            CHECK(packet_decode(buf, (size_t)len, &q) == 0);
            CHECK(q.seq == p.seq && q.n == p.n);
            for (int i = 0; i < q.n; i++) {
                uint32_t v = untag(&q.samples[i]);
                CHECK(v >= expect);                 /* in order; gaps only from overruns */
                expect = v + 1;
                received++;
            }
            packets++;
            double next = 50.0 + ((int)(rnd() % 1001) - 500) / 1000.0 * 4.0; /* +/-2 ms jitter */
            if ((rnd() % 10000) < late_prob * 10000) next += late_ms;          /* BLE busy */
            t_pkt += next;
        }
    }
    /* drain what is left */
    imu_sample_t rest[RB_SIZE];
    uint32_t n = rb_pop_up_to(&rb, rest, RB_SIZE);
    for (uint32_t i = 0; i < n; i++) { CHECK(untag(&rest[i]) >= expect); expect = untag(&rest[i]) + 1; }
    received += n;

    CHECK(received + rb_overruns(&rb) == produced);  /* nothing lost silently */
    printf("  sim %4.0fs late=%3.0f%% x%3.0fms: %6u samples, %5u packets, "
           "max fill %2u/%u, overruns %u\n",
           seconds, late_prob * 100, late_ms, produced, packets, max_fill, RB_SIZE,
           rb_overruns(&rb));
}

/* ---------------- B. two-thread stress ---------------- */
#define N_STRESS 3000000u
static ringbuf_t srb;
static atomic_uint producer_done;
static uint32_t dropped_by_producer;

static void *producer(void *arg)
{
    (void)arg;
    for (uint32_t i = 0; i < N_STRESS; i++) {
        imu_sample_t s = tag(i);
        if (!rb_push(&srb, &s)) dropped_by_producer++;
    }
    atomic_store(&producer_done, 1);
    return NULL;
}

static void stress_threads(void)
{
    rb_init(&srb);
    atomic_init(&producer_done, 0);
    dropped_by_producer = 0;

    pthread_t th;
    pthread_create(&th, NULL, producer, NULL);

    imu_sample_t out[PKT_MAX_SAMPLES];
    uint32_t received = 0, next_min = 0, order_errors = 0;
    while (1) {
        int done = atomic_load(&producer_done);    /* read BEFORE popping */
        uint32_t n = rb_pop_up_to(&srb, out, PKT_MAX_SAMPLES);
        for (uint32_t i = 0; i < n; i++) {
            uint32_t v = untag(&out[i]);
            if (v < next_min) order_errors++;
            next_min = v + 1;
        }
        received += n;
        if (done && n == 0) break;
    }
    pthread_join(th, NULL);

    CHECK(order_errors == 0);
    CHECK(rb_overruns(&srb) == dropped_by_producer);
    CHECK(received + dropped_by_producer == N_STRESS);
    printf("  threads: %u pushed, %u received, %u dropped (buffer full), order errors %u\n",
           N_STRESS, received, dropped_by_producer, order_errors);
}

int main(void)
{
    printf("A. workload simulation\n");
    sim_workload(60, 0.00, 0);       /* ideal timing                     */
    sim_workload(60, 0.05, 100);     /* 5% of packets 100 ms late        */
    sim_workload(60, 0.05, 300);     /* 5% of packets 300 ms late        */
    sim_workload(600, 0.02, 150);    /* 10 minutes, occasional delays    */

    printf("B. concurrency stress\n");
    stress_threads();

    printf(failures ? "%d failures\n" : "all stress tests passed\n", failures);
    return failures != 0;
}