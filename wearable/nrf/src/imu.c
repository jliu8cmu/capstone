#include "imu.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

#define IMU_NODE DT_NODELABEL(imu)

#define REG_COUNTER_BDR1 0x0B
#define REG_INT1_CTRL 0x0D
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_OUTX_L_G 0x22
#define WHO_AM_I_VALUE 0x6C

#define CTRL1_XL_VAL 0x4C
#define CTRL2_G_VAL 0x4C
#define CTRL3_C_RESET 0x01
#define CTRL3_C_VAL 0x44
#define COUNTER_BDR1_VAL 0x80
#define INT1_CTRL_VAL 0x01

#define BURST_LEN 12

static const struct spi_dt_spec bus =
    SPI_DT_SPEC_GET(IMU_NODE, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB);
static const struct gpio_dt_spec int1 = GPIO_DT_SPEC_GET(IMU_NODE, irq_gpios);

static struct gpio_callback int1_cb;
static void (*int1_handler)(void);

K_MUTEX_DEFINE(imu_lock);

static int xfer(const uint8_t *tx, uint8_t *rx, size_t len) {
    const struct spi_buf tx_buf = {.buf = (void *)tx, .len = len};
    const struct spi_buf rx_buf = {.buf = rx, .len = len};
    const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
    return spi_transceive_dt(&bus, &tx_set, rx ? &rx_set : NULL);
}

static uint8_t reg_read(uint8_t reg) {
    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2] = {0};
    xfer(tx, rx, sizeof tx);
    return rx[1];
}

static void reg_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg & 0x7F, val};
    xfer(tx, NULL, sizeof tx);
}

static void burst_read(uint8_t reg, uint8_t *rx) {
    uint8_t tx[1 + BURST_LEN] = {reg | 0x80};
    xfer(tx, rx, sizeof tx);
}

static const uint8_t cfg[][2] = {
    {REG_CTRL3_C, CTRL3_C_VAL},
    {REG_CTRL1_XL, CTRL1_XL_VAL},
    {REG_CTRL2_G, CTRL2_G_VAL},
    {REG_COUNTER_BDR1, COUNTER_BDR1_VAL},
    {REG_INT1_CTRL, INT1_CTRL_VAL},
};
#define N_CFG ARRAY_SIZE(cfg)

static bool configure(void) {
    uint8_t id = reg_read(REG_WHO_AM_I);
    if (id != WHO_AM_I_VALUE) {
        printk("imu: WHO_AM_I=0x%02x\n", id);
        return false;
    }

    reg_write(REG_CTRL3_C, CTRL3_C_RESET);
    int tries = 0;
    while (reg_read(REG_CTRL3_C) & CTRL3_C_RESET) {
        if (++tries > 10) {
            printk("imu: reset stuck\n");
            return false;
        }
        k_msleep(1);
    }

    for (size_t i = 0; i < N_CFG; i++) {
        reg_write(cfg[i][0], cfg[i][1]);
    }

    for (size_t i = 0; i < N_CFG; i++) {
        uint8_t got = reg_read(cfg[i][0]);
        if (got != cfg[i][1]) {
            printk("imu: reg 0x%02x = 0x%02x, want 0x%02x\n", cfg[i][0], got, cfg[i][1]);
            return false;
        }
    }
    return true;
}

bool imu_init(void) {
    if (!spi_is_ready_dt(&bus) || !gpio_is_ready_dt(&int1)) {
        printk("imu: bus not ready\n");
        return false;
    }
    if (gpio_pin_configure_dt(&int1, GPIO_INPUT) < 0) {
        return false;
    }
    printk("imu: spi %u Hz\n", bus.config.frequency);

    k_msleep(20);

    k_mutex_lock(&imu_lock, K_FOREVER);
    bool ok = configure();
    k_mutex_unlock(&imu_lock);
    return ok;
}

static inline int16_t le16(const uint8_t *p) {
    return (int16_t)(uint16_t)(p[0] | (p[1] << 8));
}

void imu_read(imu_sample_t *s) {
    uint8_t rx[1 + BURST_LEN] = {0};
    k_mutex_lock(&imu_lock, K_FOREVER);
    burst_read(REG_OUTX_L_G, rx);
    k_mutex_unlock(&imu_lock);
    const uint8_t *d = &rx[1];

    s->ax = le16(&d[6]);
    s->ay = le16(&d[8]);
    s->az = le16(&d[10]);

    s->gx = le16(&d[0]);
    s->gy = le16(&d[2]);
    s->gz = le16(&d[4]);
}

bool imu_ok(void) {
    k_mutex_lock(&imu_lock, K_FOREVER);
    uint8_t id = reg_read(REG_WHO_AM_I);
    k_mutex_unlock(&imu_lock);
    return id == WHO_AM_I_VALUE;
}

static void int1_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    if (int1_handler) {
        int1_handler();
    }
}

int imu_irq_setup(void (*handler)(void)) {
    int1_handler = handler;
    gpio_init_callback(&int1_cb, int1_isr, BIT(int1.pin));
    return gpio_add_callback_dt(&int1, &int1_cb);
}

void imu_irq_enable(bool on) {
    gpio_pin_interrupt_configure_dt(&int1, on ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
}
