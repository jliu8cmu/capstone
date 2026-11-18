#pragma once
#include <stdbool.h>
#include "packet.h"

#define IMU_PIN_MISO 16
#define IMU_PIN_CS 17
#define IMU_PIN_SCK 18
#define IMU_PIN_MOSI 19
#define IMU_PIN_INT1 20

bool imu_init(void);
void imu_read(imu_sample_t *s);
bool imu_ok(void);