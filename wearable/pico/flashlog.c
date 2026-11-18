#include "flashlog.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#define LOG_OFFSET (256u*1024u)

#define LOG_SIZE (PICO_FLASH_SIZE_BYTES - LOG_OFFSET)

#define LOG_PAGES (LOG_SIZE  / FLASH_PAGE_SIZE)

#define DUMP_LINE 32

static const uint8_t *const log_base = (const uint8_t *)(XIP_BASE + LOG_OFFSET);
static uint8_t page[FLASH_PAGE_SIZE];
static size_t page_fill;
static uint32_t next_page;

static bool is_erased(uint32_t offset, size_t len) {
    const uint8_t *p = log_base + offset;
    for (size_t i = 0; i < len; i++) {
        if (p[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

void flashlog_init(void) {
    next_page = 0;
    while (next_page < LOG_PAGES && !is_erased(next_page * FLASH_PAGE_SIZE, FLASH_PAGE_SIZE)) {
        next_page++;
    }
    page_fill = 0;
}


bool flashlog_write(const uint8_t *data, size_t len){
    while (len > 0) {
        if (next_page >= LOG_PAGES) {
            return false;
        }
        size_t n = FLASH_PAGE_SIZE - page_fill;
        if (n > len) {
            n = len;
        }
        memcpy(&page[page_fill], data, n);
        page_fill += n;

        data += n;
        len -= n;

        if (page_fill == FLASH_PAGE_SIZE) {
            uint32_t st = save_and_disable_interrupts();
            flash_range_program(LOG_OFFSET + next_page * FLASH_PAGE_SIZE, page, FLASH_PAGE_SIZE);
            restore_interrupts(st);
            next_page++;
            page_fill = 0;
        }
    }
    return true;
}


void flashlog_erase(   void ) {
    for (uint32_t off  = 0; off < LOG_SIZE; off += FLASH_SECTOR_SIZE) {
        if (is_erased(off, FLASH_SECTOR_SIZE)) {
            continue;
        }
        uint32_t st = save_and_disable_interrupts();
        flash_range_erase(LOG_OFFSET + off, FLASH_SECTOR_SIZE);
        restore_interrupts(st);
    }
    next_page = 0;
    page_fill = 0;
}

void flashlog_dump(void) {
    uint32_t used = flashlog_used();
    printf("LOGBEGIN %lu\n", (unsigned long)(used));
    for(uint32_t off = 0; off < used; off += DUMP_LINE) {
        printf("LOG ");
        for (uint32_t i = 0; i < DUMP_LINE; i++) {
            printf("%02x", log_base[off+i]);

        }
        printf("\n");
    }
    printf("LOGEND\n");
}

uint32_t flashlog_used(void) {
    return next_page * FLASH_PAGE_SIZE;
}

uint32_t flashlog_capacity(void) {
    return LOG_SIZE;
}

