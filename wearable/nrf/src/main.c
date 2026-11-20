#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include "imu.h"
#include "packet.h"
#include "ringbuffer.h"
#include "builder.h"
#include "ble_payload.h"

#define TICK_MS 50
#define TICKS_PER_SEC (1000 / TICK_MS)
#define LOW_BATT_MV 3400

#define IMU_STACK_SIZE 1024
#define IMU_PRIORITY K_PRIO_COOP(7)

static ringbuf_t rb;
static builder_t builder;
static volatile uint32_t isr_count;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct adc_dt_spec batt_adc = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

K_SEM_DEFINE(drdy_sem, 0, 1);
K_TIMER_DEFINE(tick_timer, NULL, NULL);

__weak void ble_set_payload(const uint8_t *buf, size_t len) {
    ARG_UNUSED(buf);
    ARG_UNUSED(len);
}

static void on_drdy(void) {
    isr_count++;
    k_sem_give(&drdy_sem);
}

static void imu_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    imu_sample_t s;
    while (true) {
        k_sem_take(&drdy_sem, K_FOREVER);
        imu_read(&s);
        rb_push(&rb, &s);
    }
}

K_THREAD_DEFINE(imu_tid, IMU_STACK_SIZE, imu_thread, NULL, NULL, NULL, IMU_PRIORITY, 0, 0);

static bool batt_init(void) {
    return adc_is_ready_dt(&batt_adc) && adc_channel_setup_dt(&batt_adc) == 0;
}

static bool batt_read_mv(uint32_t *mv) {
    int16_t raw;
    struct adc_sequence seq = {
        .buffer = &raw,
        .buffer_size = sizeof raw,
    };
    if (adc_sequence_init_dt(&batt_adc, &seq) < 0 || adc_read_dt(&batt_adc, &seq) < 0) {
        return false;
    }
    int32_t val = raw;
    if (adc_raw_to_millivolts_dt(&batt_adc, &val) < 0) {
        return false;
    }
    *mv = val < 0 ? 0 : (uint32_t)val;
    return true;
}

static uint8_t batt_byte(uint32_t mv) {
    if (mv <= 2500) {
        return 0;
    }
    uint32_t v = (mv - 2500) / 10;
    return v > 255 ? 255 : (uint8_t)v;
}

int main(void) {
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    bool have_batt = batt_init();
    if (!have_batt) {
        printk("batt: adc not ready\n");
    }

    rb_init(&rb);
    builder_init(&builder);

    imu_irq_setup(on_drdy);
    bool imu_alive = imu_init();
    printk("imu_init = %d\n", imu_alive);
    if (imu_alive) {
        imu_irq_enable(true);
    }

    packet_t pkt;
    uint8_t buf[PKT_MAX_LEN];
    uint32_t tick = 0;
    int empty_streak = 0;
    uint32_t mv = 0;
    if (have_batt) {
        batt_read_mv(&mv);
    }

    k_timer_start(&tick_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));

    while (true) {
        k_timer_status_sync(&tick_timer);
        tick++;

        if (tick % TICKS_PER_SEC == 0) {
            if (imu_alive) {
                imu_alive = imu_ok() && empty_streak < TICKS_PER_SEC;
            }
            if (!imu_alive) {
                imu_irq_enable(false);
                imu_alive = imu_init();
                if (imu_alive) {
                    empty_streak = 0;
                    imu_irq_enable(true);
                }
                printk("imu reinit: %d\n", imu_alive);
            }
            if (have_batt) {
                batt_read_mv(&mv);
            }
        }

        uint8_t extra = 0;
        if (!imu_alive) {
            extra |= PKT_FLAG_IMU_ERR;
        }
        if (mv < LOW_BATT_MV) {
            extra |= PKT_FLAG_LOW_BATT;
        }

        builder_build(&builder, &rb, k_uptime_get_32(), batt_byte(mv), extra, &pkt);

        empty_streak = (pkt.n == 0) ? empty_streak + 1 : 0;
        if (empty_streak >= 2) {
            pkt.flags |= PKT_FLAG_IMU_ERR;
        }

        int len = packet_encode(&pkt, buf, sizeof buf);
        if (len > 0) {
            ble_set_payload(buf, (size_t)len);
        }

        if (tick % 10 == 0) {
            gpio_pin_toggle_dt(&led);
        }

        if (tick % TICKS_PER_SEC == 0) {
            printk("seq = %u | n = %u | mv = %u | flags = 0x%02x | ovr = %u | isr_count = %u\n",
                   pkt.seq, pkt.n, mv, pkt.flags, rb_overruns(&rb), isr_count);
        }
    }
    return 0;
}
