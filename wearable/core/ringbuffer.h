#pragma once

#include "packet.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

#define RB_SIZE 32u

_Static_assert((RB_SIZE & (RB_SIZE-1)) == 0, "RB_Size not power of 2");

typedef struct {
    imu_sample_t buf[RB_SIZE];
    atomic_uint head;
    atomic_uint tail;
    atomic_uint overruns;
} ringbuf_t;

void rb_init(ringbuf_t *rb);

bool rb_push(ringbuf_t *rb, const imu_sample_t   *s);

uint32_t rb_pop_up_to(ringbuf_t *rb, imu_sample_t *out, uint32_t max);

uint32_t rb_count(const ringbuf_t *rb);

uint32_t rb_overruns(const ringbuf_t *rb);