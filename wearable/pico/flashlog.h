#ifndef FLASHLOG_H
#define FLASHLOG_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void flashlog_init(void);

bool flashlog_write(const uint8_t *data, size_t len);

void flashlog_erase(void);

void flashlog_dump(void);

uint32_t flashlog_used(void);

uint32_t flashlog_capacity(void);
#endif