#include "imu.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include <stdio.h>

#define IMU_SPI spi0
#define REG_COUNTER_BDR1 0x0B
#define REG_INT1_CTRL 0x0D
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_OUTX_L_G 0x22
#define WHO_AM_I_VALUE 0x6C

#define CTRL1_XL_VAL 0x4C //accel
#define CTRL2_G_VAL 0x4C //gyroscope

#define IMU_SPI_HZ (4 * 1000000)

#define CTRL3_C_RESET 0x01
#define CTRL3_C_VAL 0x44
#define COUNTER_BDR1_VAL 0x80
#define INT1_CTRL_VAL 0x01

#define BURST_LEN 12

static void cs(bool active) {
    gpio_put(IMU_PIN_CS, !active);
}

static uint8_t reg_read(uint8_t reg) {
    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2];
    cs(true);
    spi_write_read_blocking(IMU_SPI, tx, rx, 2);
    cs(false);
    return rx[1];
}

static void reg_write(uint8_t reg, uint8_t val) {
        uint8_t tx[2] = {reg & 0x7F , val};
        cs(true); 
        spi_write_blocking(IMU_SPI, tx, 2);
        cs(false);
}  

static void burst_read(uint8_t reg, uint8_t *rx) {
    uint8_t tx[1+ BURST_LEN] = {reg | 0x80};
    cs(true);
    spi_write_read_blocking(IMU_SPI, tx, rx, 1 + BURST_LEN);
    cs(false);
}

static const uint8_t cfg[][2] = {
    {REG_CTRL3_C, CTRL3_C_VAL},
    {REG_CTRL1_XL, CTRL1_XL_VAL},
    {REG_CTRL2_G, CTRL2_G_VAL},
    {REG_COUNTER_BDR1, COUNTER_BDR1_VAL},
    {REG_INT1_CTRL, INT1_CTRL_VAL},
};
#define N_CFG (sizeof cfg / sizeof cfg[0])


bool imu_init(void) {
    uint actual = spi_init(IMU_SPI, IMU_SPI_HZ);
    spi_set_format(IMU_SPI, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    printf("imu: spi %u HZ\n", actual);

    gpio_set_function(IMU_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(IMU_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(IMU_PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(IMU_PIN_CS);
    gpio_put(IMU_PIN_CS, 1);
    gpio_set_dir(IMU_PIN_CS, GPIO_OUT);

    gpio_init(IMU_PIN_INT1);
    gpio_set_dir(IMU_PIN_INT1, GPIO_IN);
    gpio_pull_down(IMU_PIN_INT1);

    sleep_ms(20);
    uint8_t id = reg_read(REG_WHO_AM_I);
    if (id != WHO_AM_I_VALUE) {
        printf("imu: WHO_AM_I=0x%02x\n",  id);
        return false;
    }

    reg_write(REG_CTRL3_C, CTRL3_C_RESET);
    int tries =0;
    while (reg_read(REG_CTRL3_C) & CTRL3_C_RESET) {
        if(++tries > 10) {
            printf("imu: reset stuck\n");
            return false;
        }
        sleep_ms(1);
    }

    for (size_t i = 0; i < N_CFG; i++) {
        reg_write(cfg[i][0], cfg[i][1]);
    }

    for(size_t i = 0; i < N_CFG; i++) {
        uint8_t got = reg_read(cfg[i][0]);
        if (got != cfg[i][1]) {
            printf("imu: reg 0x%02x = 0x%02x, want 0x%02x\n", cfg[i][0], got, cfg[i][1]);
            return false;
        }

        
    }
    return true;
}


static inline int16_t le16(const uint8_t *p) {
    return (int16_t)(uint16_t)(p[0] | (p[1] << 8));
}

void imu_read(imu_sample_t *s) {
    uint8_t rx[1 + BURST_LEN];
    burst_read(REG_OUTX_L_G, rx);
    const uint8_t *d = &rx[1];

    s->ax = le16(&d[6]);
    s->ay = le16(&d[8]);
    s-> az = le16(&d[10]);

    s->gx = le16(&d[0]);
    s->gy = le16(&d[2]);
    s->gz = le16(&d[4]);
}

bool imu_ok(void) {
    uint32_t st = save_and_disable_interrupts();
    uint8_t id = reg_read(REG_WHO_AM_I);
    restore_interrupts(st);
    return id == WHO_AM_I_VALUE;
}
