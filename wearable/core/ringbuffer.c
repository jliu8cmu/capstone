#include "ringbuffer.h"
#include <stdatomic.h>

void rb_init(ringbuf_t *rb) {
    atomic_init( &rb->head, 0);
    atomic_init(&rb->tail, 0);
    atomic_init(&rb->overruns, 0);
}

bool rb_push(ringbuf_t *rb, const imu_sample_t   *s) {
    uint32_t h = atomic_load_explicit(&rb->head, memory_order_relaxed);
    uint32_t t = atomic_load_explicit(&rb->tail, memory_order_acquire);

    if ( h-t == RB_SIZE) {
        atomic_fetch_add_explicit(&rb->overruns, 1, memory_order_relaxed);
        return false;
    }

    rb->buf[h & (RB_SIZE - 1)] = *s;

    atomic_store_explicit(&rb->head, h + 1, memory_order_release);
    return true;
}

uint32_t rb_pop_up_to(ringbuf_t *rb, imu_sample_t *out, uint32_t max) {
    uint32_t t = atomic_load_explicit(&rb->tail, memory_order_relaxed);

    uint32_t h = atomic_load_explicit(&rb->head, memory_order_acquire);

    uint32_t n = h - t;

    if(n > max) {
        n = max;
    }

    for(size_t i = 0; i < n; i++) {
        out[i] = rb->buf[(t + i) & (RB_SIZE - 1)];
    }

    atomic_store_explicit(&rb->tail, t + n, memory_order_release);

    return n;
}

uint32_t rb_count(const ringbuf_t *rb) {
     uint32_t t = atomic_load_explicit(&rb->tail, memory_order_relaxed);

    uint32_t h = atomic_load_explicit(&rb->head, memory_order_acquire);

    uint32_t n = h - t;

    return n;
}

uint32_t rb_overruns(const ringbuf_t *rb) {
    return atomic_load_explicit(&rb->overruns, memory_order_relaxed);
}