#pragma once

#include <stdbool.h>
#include "packet.h"

bool imu_init(void);
void imu_read(imu_sample_t *s);
bool imu_ok(void);

int imu_irq_setup(void (*handler)(void));
void imu_irq_enable(bool on);
